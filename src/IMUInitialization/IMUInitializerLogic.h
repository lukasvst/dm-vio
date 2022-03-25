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

#ifndef DMVIO_IMUINITIALIZERLOGIC_H
#define DMVIO_IMUINITIALIZERLOGIC_H


#include <memory>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include "CoarseIMUInitOptimizer.h"
#include "PoseGraphBundleAdjustment.h"
#include "IMUInitStateChanger.h"

namespace dmvio
{

class IMUInitializerState;

class IMUInitVariances
{
public:
    IMUInitVariances() = default;
    IMUInitVariances(const gtsam::Marginals& marginals, gtsam::Key scaleKey, gtsam::Key biasKey);

    bool indetermined = true;
    double scaleVariance;
    gtsam::Matrix biasCovariance;
};

class StateTransitionModel;

// Helper class which encapsulates common logic and data for the states of the IMUInitializer.
class IMUInitializerLogic
{
public:
    typedef std::function<void(const gtsam::Values& values, bool)> InitCallback;

    IMUInitializerLogic(std::string resultsPrefix,
                        boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams,
                        const dmvio::IMUCalibration& imuCalibration,
                        dmvio::IMUInitSettings& settings,
                        DelayedMarginalizationGraphs* delayedMarginalization,
                        bool linearizeOperation, InitCallback callOnInit,
                        IMUInitStateChanger& stateChanger);

    // This is required to change the state from different threads.
    IMUInitStateChanger& stateChanger;

    DelayedMarginalizationGraphs* delayedMarginalizationGraphs; // Used to replace the main graph on init.

    // if true the factory methods will create the CoarseIMUInitState (or the PGBAState respectively) in realtime mode.
    bool realtimeCoarseIMUInit;
    bool realtimePGBA;

    InitCallback callOnInit;

    std::shared_ptr<TransformDSOToIMU> transformDSOToIMU;
    std::shared_ptr<TransformDSOToIMU> transformDSOToIMUAfterPGBA;
    std::shared_ptr<bool> optScale;
    std::shared_ptr<bool> optGravity;
    std::shared_ptr<bool> optT_cam_imu;

    const IMUCalibration& imuCalibration;
    IMUInitSettings& settings;

    // This is the bias used for the preintegration in the main system.
    gtsam::imuBias::ConstantBias latestBias;

    // For CoarseIMUInit:
    std::unique_ptr<CoarseIMUInitOptimizer> coarseIMUOptimizer;
    gtsam::PreintegratedImuMeasurements imuMeasurements;
    void addPose(const dso::FrameShell& shell, bool willBecomeKeyframe, const IMUData* imuData);
    IMUInitVariances performCoarseIMUInit(double timestamp);

    // For PGBA.
    std::unique_ptr<PoseGraphBundleAdjustment> pgba;

};


}


#endif //DMVIO_IMUINITIALIZERLOGIC_H
