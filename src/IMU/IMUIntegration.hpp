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

#ifndef IMUIntegration_hpp
#define IMUIntegration_hpp

#include <stdio.h>
#include <vector>
#include <Eigen/Eigenvalues>

#include "IMUTypes.h"
#include "IMUSettings.h"

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <sophus/sophus.hpp>
#include <sophus/se3.hpp>

#include "dso/util/NumType.h"

#include "OptimizationBackend/EnergyFunctional.h"
#include "util/GTData.hpp"
#include "BAIMULogic.h"
#include <fstream>
#include <dso/util/FrameShell.h>

#include "CoarseIMULogic.h"
#include "IMUInitialization/IMUInitializer.h"

namespace dmvio
{


// Main class for the integration of IMU data into DSO. Mostly a facade as the implementation logic is forwarded to
// either CoarseIMULogic (for the coarse tracking), BAIMULogic (for the integration of IMU into the main bundle
// adjustment), and IMUInitializer (for the IMU initialization).
// This class is responsible for making sure that the preintegration and interactions between CoarseIMULogic
// and BAIMULogic work correctly in realtime mode (where they are in different threads).
class IMUIntegration : public PreintegrationProviderBA
{
public:
    // linearizeOperation is true in non-realtime mode (means that there is only a single thread used).
    // Note that a reference to imuSettings is kept, so it needs to stay alive.
    IMUIntegration(dso::CalibHessian* HCalib, const IMUCalibration& imuCalibrationPassed,
                   IMUSettings& imuSettingsPassed, bool linearizeOperationPassed);

    ~IMUIntegration();

    // Return true if the CoarseTracking logic is initialized.
    bool isCoarseInitialized();

    // called when the coarse tracking reference is switched.
    // Returns transformation from last keyframe to new keyframe.
    Sophus::SE3d initCoarseGraph();

    // Called to add IMU data for the coarse tracking (and the IMU initializer).
    // Adds a new frame with IMU data to the coarse factor graph, marginalizes old variables, and returns an estimate
    // for the relative pose of the newly added frame.
    Sophus::SE3 addIMUData(const IMUData& imuData,
                           int frameId, double frameTimestamp, bool firstFrameAfterKFChange,
                           int lastFrameId, bool onlyForHint = false);


    // Add IMU data for the bundle adjustment
    void addIMUDataToBA(const IMUData& imuData);

    // Called when the first initializer frame changes.
    void resetBAPreintegration();

    // Passes the new coarse tracking pose.
    void updateCoarsePose(const Sophus::SE3& pose);

    // This method integrates the CoarseTracker optimization with GTSAM. It is called in each iteration, and
    // will compute the increment for the optimization iteration.
    // returns new refToFrame, gets H and b as an input. incA and incB are output parameters that contain the
    // increment of the affine lightning transforms after the method call, incNorm is the norm of the increment.
    // b contains the following parameters: 3 for the rotation ref_to_frame, 3 for the translation ref_to_frame, and
    // 2 for affine lightning parameters.
    Sophus::SE3 computeCoarseUpdate(const dso::Mat88& H, const dso::Vec8& b, float extrapFac, float lambda,
                                    double& incA, double& incB, double& incNorm);

    // Apply the update computed by the last call of computeCoarseUpdate.
    void acceptCoarseUpdate();

    void addVisualToCoarseGraph(const dso::Mat88& H, const dso::Vec8& b, bool trackingIsGood);

    // Returns the pose of the current keyframe as computed by the coarse tracking as a gtsam Pose (imu to world)
    Sophus::SE3d getCoarseKFPose();

    // Called when DSO finishes coarse tracking.
    void finishCoarseTracking(const dso::FrameShell& frameShell, bool willBecomeKeyframe);

    // prepareKeyframe tells the IMU-Integration that this frame will probably become a keyframe. (-> don' marginalize it during addIMUData...)
    // Also resets the IMU preintegration for the BA.
    void prepareKeyframe(int frameId);

    virtual const gtsam::PreintegratedImuMeasurements& getPreintegratedMeasurements(int keyframeId) override;

    bool isPreparedKFCreated() const;

    int getPreparedKeyframe() const;

    // tells the IMU-Integration that this frame has become a keyframe that will shortly be bundle-adjusted.
    void keyframeCreated(int frameId);

    void skipPreparedKeyframe();

    bool newTrackingRefNeedsHandling();

    // These methods are both called right after the BA optimization, the former outside and the latter inside the
    // mutex for changing the coarse tracking ref.
    void postOptimization(int keyframeId);
    // updateBAValues should be called before this. Returns true if this BA optimization has used IMU data (which is
    // also true if and only if the next CoarseTracking will use IMU data.)
    bool finishKeyframeOptimization(int keyframeId);

    // Called when all keyframe operations (including marginalization) are finished.
    // Note that this can be after the tracking reference has already changed
    // e.g. the order can be: BA finished (finishKeyframeOptimization) -> tracking reference changed -> in other
    // thread marginalization and post BA stuff -> finishKeyframeOperations()
    void finishKeyframeOperations(int keyframeId);

    Sophus::SE3 TS_cam_imu;

    // Sets groundtruth data for a frame for printing out information. Should only be used in non-rt mode as it currently
    // does not handle multiple threads correctly.
    void setGTData(dmvio::GTData* gtData, int frameId);

    IMUSettings& getImuSettings() const;

    // Called when a new energy threshold is computed by DSO. Can optionally update the threshold.
    // We use this to make sure the energy threshold cannot get arbitrarily high (and allow extremely bad points
    // into the optimization).
    void newFrameEnergyTH(float& energyThreshold);

    const std::unique_ptr<BAGTSAMIntegration>& getBAGTSAMIntegration() const;

    // Return current transform between IMU and DSO frame (updated during the BA). Should only be called from BA thread.
    TransformDSOToIMU& getTransformDSOToIMU();
    // Get the scale of TransformDSOToIMU used for the coarse tracking currently (can be called from any thread).
    double getCoarseScale();

private:
    IMUCalibration imuCalibration;
    IMUSettings& imuSettings;

    bool linearizeOperation;

    boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams;

    boost::shared_ptr<gtsam::PreintegratedImuMeasurements> preintegratedBA, preintegratedBACurr, preintegratedForNextCoarse;

    int preparedKeyframe;
    gtsam::Vector3 preparedCoarseVel;
    bool preparedKFCreated;

    // Changed (only) in the coarse tracker's thread.
    bool imuDataPreintegrated = false;

    IMUData lastIMUData;

    std::shared_ptr<BAIMULogic> baLogic;
    std::unique_ptr<CoarseIMULogic> coarseLogic;
    std::unique_ptr<BAGTSAMIntegration> baGTSAMIntegration;
    gtsam::imuBias::ConstantBias latestBias;

    std::unique_ptr<IMUInitializer> imuInitializer;

    // These variables are set when the BA optimization is finished, so the coarse tracking can later initialize with these values.
    std::unique_ptr<InformationBAToCoarse> informationBAToCoarse;

    bool baInitialized = false;
    bool coarseInitialized = false;
    bool initializedBeforePostOptimization = false;

    float lastDSOEnergyTH;
};

}

#endif /* IMUIntegration_hpp */


