/**
* This file is part of DM-VIO.
*
* Copyright (c) 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>.
* for more information see <http://vision.in.tum.de/dm-vio>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DM-VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DM-VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DM-VIO. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DMVIO_BAIMULOGIC_H
#define DMVIO_BAIMULOGIC_H

#include <util/GTData.hpp>
#include <GTSAMIntegration/PoseTransformationIMU.h>
#include "GTSAMIntegration/BAGTSAMIntegration.h"
#include "IMUTypes.h"
#include "IMUSettings.h"
#include "GTSAMIntegration/DelayedMarginalization.h"

namespace dmvio
{

// Information the Coarse tracking needs from the previous BA optimization when the KF changes.
struct InformationBAToCoarse
{
    double latestBAScale;
    gtsam::Vector3 latestBAVel;
    gtsam::imuBias::ConstantBias latestBABias;
    gtsam::Pose3 latestBAPose, latestBAPosePrevKeyframe;

    // Transform poses from BA to IMU frame. Typically of type TransformDSOToIMU
    std::shared_ptr<PoseTransformation> transformBAToIMU;
    // Transform poses from IMU to coarse DSO poses. Typically of type TransformIMUToDSOForCoarse<TransformDSOToIMUNew>.
    std::unique_ptr<PoseTransformation> transformIMUToDSOForCoarse;

    gtsam::LinearContainerFactor::shared_ptr priorFactor; // Factor with priors to add to the graph.
};

// Provides the preintegrated BA data for BAIMULogic.
// Responsible for making sure that it always contains the preintegrated data for the frame which actually became a KF
// (which can change in realtime mode because DSO can change its mind which frame becomes a keyframe).
class PreintegrationProviderBA
{
public:
    virtual const gtsam::PreintegratedImuMeasurements& getPreintegratedMeasurements(int keyframeId) = 0;
};

class IMUInitializer;

// Responsible for integrating IMU data into the DSO Bundle Adjustment (BA).
// This class adds velocity, bias, scale and gravity direction as additional variables to be optimized.
// It adds IMU factors and bias random walk factors between successive keyframes.
class BAIMULogic : public BAExtension
{
public:
    // We have three groups of factors. NO_IMU_GROUP contains all DSO factors, BIAS_AND_PRIOR_group contains bias
    // random walk factors and prior factors. METRIC_GROUP contains IMU factors.
    // This is used e.g. for adding only visual factors (group 0) to the delayed graph for IMU reinitialization.
    enum FactorGroups
    {
        NO_IMU_GROUP = 0, BIAS_AND_PRIOR_GROUP, METRIC_GROUP
    };

    // Note: A reference to preintegrationProvider, imuCalibration, imuSettings, and baIntegration is kept, so they all must be kept alive.
    BAIMULogic(PreintegrationProviderBA* preintegrationProvider, BAGTSAMIntegration* baIntegration,
               const IMUCalibration& imuCalibration, IMUSettings& imuSettings);

    // Methods called by BAGTSAMIntegration:
    virtual void addFirstBAFrame(int keyframeId, BAGraphs* baGraphs, gtsam::Values::shared_ptr baValues) override;

    virtual void addKeyframe(BAGraphs* baGraphs, gtsam::Values::shared_ptr baValues, int keyframeId,
                             const Sophus::SE3d& keyframePose, std::vector<dso::EFFrame*>& frames) override;

    virtual void
    updateBAOrdering(std::vector<dso::EFFrame*>& frames, gtsam::Ordering* ordering, KeyDimMap& baDimMap) override;

    virtual void addKeysToMarginalize(int fullId, gtsam::FastVector<gtsam::Key>& keysToMarginalize) override;

    virtual void preSolve(gtsam::Matrix& HFull, gtsam::Vector& bFull, int dimensionDSOH) override;

    virtual bool
    postSolve(gtsam::Values::shared_ptr values, gtsam::Values::shared_ptr newValues, const gtsam::Vector& inc,
              const gtsam::Ordering& ordering, const KeyDimMap& baDimMap) override;

    virtual void acceptUpdate(gtsam::Values::shared_ptr values, gtsam::Values::shared_ptr newValues) override;

    // --------------------------------------------------

    // Takeover result from the IMU initializer. Values contains the new values (DSO variables will not replaced though).
    // If reinit this is a reinitialization, meaning that the IMU is being used already.
    // If willReplaceGraph the IMU initializer will replace the marginalization prior (typically obtained by readvancing).
    void initFromIMUInit(const gtsam::Values& values, bool reinit, bool willReplaceGraph);

    // Methods called by IMUIntegration:

    // Called when all keyframe operations (including marginalization) are finished.
    // Note that this can be after the tracking reference has already changed
    // e.g. the order can be: BA finished (finishKeyframeOptimization) -> tracking reference changed ->
    // -> in other thread marginalization and post BA stuff -> finishKeyframeOperations())
    void finishKeyframeOperations(int keyframeId);

    // Called before finishKeyframeOptimization (and before the IMU init might be called).
    void postOptimization(int keyframeId);

    // Called just before the new KF will become the new coarse tracking reference.
    // (called from the BA thread but with mutex on coarseTrackerSwapMutex.)
    std::unique_ptr<InformationBAToCoarse> finishKeyframeOptimization(int keyframeId);

    // Called to set the velocity of the next keyframe (to transfer the velocity from the coarse tracking).
    void setNextBAVel(const gtsam::Vector3& velocity, int frameId);

    // Set groundtruth data if available (for printing result to file).
    // Should only be used in non-RT mode for now, because it keeps a reference and the objected might be deleted otherwise.
    void setCurrGtData(dmvio::GTData* currGtData, int frameId);

    // Return the (optimized and regularly updated) transform from DSO to IMU.
    // Should only used inside the BA thread, otherwise there might be a race condition.
    std::shared_ptr<TransformDSOToIMU> getTransformDSOToIMU() const;

    bool isScaleFixed() const;

    double computeDynamicDSOWeight(double lastDSOEnergy, double lastRMSE, bool coarseTrackingWasGood);

private:
    bool addIMUVarsForKey(int keyframeId);

    BAGTSAMIntegration* baIntegration;
    PreintegrationProviderBA* preintegrationProvider;

    // Shared with parent IMUIntegration.
    IMUSettings& imuSettings;
    const IMUCalibration& imuCalibration;

    // Pose transformation used for the IMU factors.
    // Transforms from DSO frame to IMU (metric) frame.
    std::shared_ptr<TransformDSOToIMU> transformDSOToIMU;

    // If set to a positive value, the IMU integration is disabled starting from the KF with id (used for debug purposes):
    int disableFromKF = -1;
    int noIMUInOrderingUntilKFId = -1; // if set, we don't add IMU keys to the ordering for kfid < noIMUInOrderingUntilKFId

    // True if the variables inside the transformDSOToIMU are optimized at all.
    bool optimizeTransform;
    // Pointers are shared with transformDSOToIMUNew.
    std::shared_ptr<bool> optimizeScalePtr, optimizeGravityPtr, optimizedIMUExtrinsicsPtr;
    // For convenience: references to the pointers above.
    bool& optimizeScale;
    bool& optimizeGravity;
    bool& optimizeIMUExtrinsics;

    int previousKeyframeId{-1};
    int currKeyframeId{-1};
    double firstBATimestamp{-1};

    // Used for skipping the first keyframe for IMU data.
    int firstFrameId{-1};

    // Variables for determining when to fix the scale.
    bool scaleFixed = false;
    // Maximum and minimum scale during this keyframe optimization.
    double maxScaleInterval = 0.0, minScaleInterval = 1000;
    std::deque<std::pair<double, double> > scaleQueue; // Saves maximum and minimum scale for the last keyframes.

    dmvio::GTData* currGTData = nullptr;
    int gtFrameId = -1;

    // Files to which results are saved.
    std::ofstream scaleFile, baBiasFile, baGravDirFile, baVelFile;

    // Prior factor to be used by the CoarseIMULogic. It is obtained by marginalizing everything in the BA graph, except
    // the newest bias and velocity. This is a useful prior for the coarse tracking.
    gtsam::LinearContainerFactor::shared_ptr factorForCoarseGraph;
    // computes the factor. Also computes uncertainty for some more variables for saving to file.
    gtsam::LinearContainerFactor::shared_ptr computeFactorForCoarseGraphAndMarginalCovariances();

    gtsam::Matrix biasCovariance;
    int biasCovForKF = -1;

    // Used to get the velocity from the coarse tracking.
    gtsam::Vector3 nextVelocity;
    int nextVelocityFrameId = -1;
};

}

#endif //DMVIO_BAIMULOGIC_H
