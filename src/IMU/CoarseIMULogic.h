/**
* This file is part of DM-VIO.
* The code in this file is in part based on code written by Vladyslav Usenko for the paper "Direct Visual-Inertial Odometry with Stereo Cameras".
*
* Copyright (c) 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>, Vladyslav Usenko
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

#ifndef DMVIO_COARSEIMULOGIC_H
#define DMVIO_COARSEIMULOGIC_H

#include "IMUTypes.h"
#include "IMUSettings.h"
#include "BAIMULogic.h"

#include <sophus/sophus.hpp>
#include <sophus/se3.hpp>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <dso/util/NumType.h>

namespace dmvio
{
// Contains logic how to integrate IMU data into the coarse tracking (see CoarseTracker.h)
// To this end it changes the optimized variables from the relative pose from referenceToFrame to absolute poses
// in IMU frame. It adds IMU-Factors between successive frames.
class CoarseIMULogic
{
public:
    // Typically initCoarseGraph should also be called before using this.
    // Note that a reference to imuCalibration and imuSettings is kept, so they need to be kept alive.
    CoarseIMULogic(std::unique_ptr<PoseTransformation> transformBAToIMU,
                   boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams,
                   const IMUCalibration& imuCalibration,
                   IMUSettings& imuSettings);

    // (Re-)initialize the coarse tracking graph after a new reference frame has been activated.
    Sophus::SE3d initCoarseGraph(int keyframeId, std::unique_ptr<InformationBAToCoarse> informationBAToCoarse);

    // Adds an new frame with IMU data to the coarse factor graph, marginalizes old variables, and returns an estimate
    // for the relative pose of the newly added frame.
    // dontMargFrame is the id of a frame (usually a prepared KF) which should not be marginalized.
    Sophus::SE3d addIMUData(const IMUData& imuData,
                            int frameId, double frameTimestamp,
                            int lastFrameId,
                            boost::shared_ptr<gtsam::PreintegratedImuMeasurements> additionalMeasurements = nullptr,
                            int dontMargFrame = -1);

    // ------------------------------
    // Called by CoarseTracker (at the moment they are forwarded through IMUIntegration):

    // Passes the new coarse pose.
    void updateCoarsePose(const Sophus::SE3d& pose);

    // This method integrates the CoarseTracker optimization with GTSAM. It is called in each iteration, and
    // will compute the increment for the optimization iteration.
    // returns new refToFrame, gets H and b as an input. incA and incB are output parameters that contain the
    // increment of the affine lightning transforms after the method call, incNorm is the norm of the increment.
    // b contains the following parameters: 3 for the rotation ref_to_frame, 3 for the translation ref_to_frame, and
    // 2 for affine lightning parameters.
    Sophus::SE3d computeCoarseUpdate(const dso::Mat88& H, const dso::Vec8& b, float extrapFac, float lambda,
                                     double& incA, double& incB, double& incNorm);

    // Apply the update computed by the last call of computeCoarseUpdate.
    void acceptCoarseUpdate();

    // Add linearized visual factor to the coarse graph.
    void addVisualToCoarseGraph(const dso::Mat88& H, const dso::Vec8& b, bool trackingIsGood);


    Sophus::SE3d getCoarseKFPose();
    gtsam::imuBias::ConstantBias getBias(int frameId);
    gtsam::Vector3 getVelocity(int frameId);
    void printCoarseBiases(const dmvio::GTData* gtData, int frameId);
    double getScale() const;

private:
    // Shared with parent IMUIntegration.
    IMUSettings& imuSettings;
    const IMUCalibration& imuCalibration;

    std::shared_ptr<PoseTransformation> transformBAToIMU;
    // Usually of type TransformDSOToIMU
    std::unique_ptr<PoseTransformation> transformIMUToDSOForCoarse; // Usually of type TransformIMUToDSOForCoarse<T>
    double scale = 1.0;

    boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams;

    boost::shared_ptr<gtsam::NonlinearFactorGraph> coarseGraph;
    boost::shared_ptr<gtsam::Values> coarseValues;

    gtsam::Key currentPoseKey, refPoseKey;

    gtsam::Ordering coarseOrdering;
    gtsam::Values::shared_ptr newCoarseValues;

    int currentKeyframeId = -1;
    double currCoarseTimestamp;
    bool firstCoarseInit = true;

    std::ofstream coarseBiasFile;
};


}
#endif //DMVIO_COARSEIMULOGIC_H
