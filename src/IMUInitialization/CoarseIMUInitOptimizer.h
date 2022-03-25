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

#ifndef DMVIO_COARSEIMUINITOPTIMIZER_H
#define DMVIO_COARSEIMUINITOPTIMIZER_H


#include "sophus/se3.hpp"
#include "GTSAMIntegration/PoseTransformationIMU.h"
#include "IMU/IMUTypes.h"
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include "IMU/IMUSettings.h"
#include <fstream>

namespace dso
{
class FrameShell;
}

namespace dmvio
{
// This is the Coarse IMU Initializer from the paper
// It only optimizes IMU variables (scale, gravity direction, bias, velocities).
// To be exact: velocities, bias and all symbols optimized by the passed PoseTransformation are optimized.
// Owned (and methods called) by IMUInitializer, doesn't know DSO.
class CoarseIMUInitOptimizer
{
public:
    // transformDSOToIMU is updated during optimization.
    // Note that a reference to the settings and calibration is kept!
    // Note: the owner of this class is responsible for adding initial values and priors for the variables optimized
    // by transformDSOToIMU (e.g s0, g0).
    explicit CoarseIMUInitOptimizer(std::shared_ptr<PoseTransformation> transformDSOToIMU,
                                    const IMUCalibration& imuCalibration,
                                    const CoarseIMUInitOptimizerSettings& settingsPassed);

    // Add frame to the optimizer.
    void addPose(int frameId, const Sophus::SE3d& camToWorld, const gtsam::PreintegratedImuMeasurements* imuData);
    void addPose(const dso::FrameShell& shell, const gtsam::PreintegratedImuMeasurements* imuData);

    struct OptimizationResult
    {
        OptimizationResult(int numIterations, double error, double normalizedError, bool good);
        int numIterations;
        double error;
        double normalizedError;
        bool good;
    };

    OptimizationResult optimize();
    std::shared_ptr<PoseTransformation> getUpdatedTransform();
    gtsam::imuBias::ConstantBias getBias();
    gtsam::Key getBiasKey();

    gtsam::Marginals getMarginals();

    void takeOverOptimizedValues();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    gtsam::Values optimizedValues;
    int numFrames = 0;
    int imuFactorsRemovedUntil = -1;
private:
    void handleFirstFrame(int frameId);

    const CoarseIMUInitOptimizerSettings& settings;
    const IMUCalibration& imuCalibration;

    std::shared_ptr<PoseTransformation> transformDSOToIMU;

    int prevFrameId = -1;
    gtsam::Pose3 prevFramePose; // Needed for fixPoses.

    gtsam::LevenbergMarquardtParams params = gtsam::LevenbergMarquardtParams::CeresDefaults();

    // Only used if fixPoses == false;
    gtsam::SharedNoiseModel posePriorModel;

    // For implementing maxNumPoses:
    std::deque<int> poseIds; // pose ids currently in the graph.

    // used to get updated poses from DSO before optimizing.
    std::map<int, const dso::FrameShell*> activeShells;

};


}

#endif //DMVIO_COARSEIMUINITOPTIMIZER_H
