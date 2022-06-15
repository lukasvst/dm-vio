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

#include "util/TimeMeasurement.h"

#include <memory>
#include "IMUInitializerLogic.h"
#include "IMUInitializerStates.h"

dmvio::IMUInitializerLogic::IMUInitializerLogic(std::string resultsPrefix,
                                                boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams,
                                                const dmvio::IMUCalibration& imuCalibration,
                                                dmvio::IMUInitSettings& settings,
                                                DelayedMarginalizationGraphs* delayedMarginalization,
                                                bool linearizeOperation, InitCallback callOnInit,
                                                IMUInitStateChanger& stateChanger)
        : imuCalibration(imuCalibration), settings(settings),
          imuMeasurements(preintegrationParams),
          optScale(new bool(true)), optGravity(new bool(true)), optT_cam_imu(new bool(false)),
          callOnInit(callOnInit), delayedMarginalizationGraphs(delayedMarginalization),
          stateChanger(stateChanger)
{
    if(linearizeOperation)
    {
        realtimePGBA = settings.multithreadedInitDespiteNonRT;
        realtimeCoarseIMUInit = settings.multithreadedInitDespiteNonRT;
    }else
    {
        // Realtime mode
        realtimePGBA = true;
        realtimeCoarseIMUInit = true;
    }

    transformDSOToIMU.reset(new TransformDSOToIMU(gtsam::Pose3(imuCalibration.T_cam_imu.matrix()),
                                                  optScale, optGravity, optT_cam_imu, true, 0));

    transformDSOToIMUAfterPGBA.reset(new TransformDSOToIMU(*transformDSOToIMU));

    // Initialize CoarseIMUInitOptimizer:
    coarseIMUOptimizer = std::make_unique<CoarseIMUInitOptimizer>(transformDSOToIMU, imuCalibration,
                                                                  settings.coarseInitSettings);

    // Add priors for transform related variables to the imuOptimGraph:
    auto factors = getPriorsAndAddValuesForTransform(*transformDSOToIMU, settings.transformPriors,
                                                     coarseIMUOptimizer->values);
    for(auto&& factor : factors)
    {
        coarseIMUOptimizer->graph.add(factor);
    }

    assert(delayedMarginalization);
    pgba = std::make_unique<PoseGraphBundleAdjustment>(delayedMarginalization, imuCalibration, settings.pgbaSettings,
                                                       transformDSOToIMUAfterPGBA);

}

void dmvio::IMUInitializerLogic::addPose(const dso::FrameShell& shell, bool willBecomeKeyframe, const IMUData* imuData)
{
    bool imuDataAvailable = false;
    if(imuData)
    {
        imuDataAvailable = true;
        integrateIMUData(*imuData, imuMeasurements);
    }

    if(settings.onlyKFs && !willBecomeKeyframe) return;

    dmvio::TimeMeasurement addPoseTimeMeas("IMUInitAddPose");
    if(imuDataAvailable)
    {
        coarseIMUOptimizer->addPose(shell, &imuMeasurements);
        imuMeasurements.resetIntegrationAndSetBias(coarseIMUOptimizer->getBias());
    }else
    {
        // First frame: No IMU data yet.
        coarseIMUOptimizer->addPose(shell, nullptr);
        imuMeasurements.resetIntegration();
    }
}

dmvio::IMUInitVariances dmvio::IMUInitializerLogic::performCoarseIMUInit(double timestamp)
{
    dmvio::TimeMeasurement optimTime("IMUInitOptimize");
    CoarseIMUInitOptimizer::OptimizationResult result = coarseIMUOptimizer->optimize();
    double time = optimTime.end();

    IMUInitVariances variances;
    if(result.good)
    {
        gtsam::Marginals marginals = coarseIMUOptimizer->getMarginals();
        variances = IMUInitVariances(marginals, gtsam::Symbol('s', 0), coarseIMUOptimizer->getBiasKey());

        std::cout << "CoarseIMUInit normalized error: " << result.normalizedError << " variance: " <<
                  variances.scaleVariance << " scale: " << transformDSOToIMU->getScale() << std::endl;
    }

    return variances;
}


dmvio::IMUInitVariances::IMUInitVariances(const gtsam::Marginals& marginals, gtsam::Key scaleKey, gtsam::Key biasKey)
{
    indetermined = false;
    try
    {
        gtsam::Matrix scaleCovariance = marginals.marginalCovariance(scaleKey);
        scaleVariance = scaleCovariance(0, 0);
        biasCovariance = marginals.marginalCovariance(biasKey);
    }catch(gtsam::IndeterminantLinearSystemException& exc)
    {
        indetermined = true;
    }
}
