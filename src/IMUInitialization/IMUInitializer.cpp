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

#include "IMUInitializer.h"
#include "dso/util/FrameShell.h"
#include "IMU/IMUUtils.h"
#include "util/TimeMeasurement.h"
#include "GTSAMIntegration/Marginalization.h"
#include "GTSAMIntegration/Sim3GTSAM.h"
#include "IMU/BAIMULogic.h"
#include "IMUInitializerLogic.h"
#include "IMUInitializerStates.h"
#include "IMUInitializerTransitions.h"

dmvio::IMUInitializer::IMUInitializer(std::string resultsPrefix,
                                      boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams,
                                      const IMUCalibration& imuCalibration, IMUInitSettings& settings,
                                      DelayedMarginalizationGraphs* delayedMarginalization, bool linearizeOperation,
                                      InitCallback callOnInit)
{
    logic = std::make_unique<IMUInitializerLogic>(resultsPrefix, preintegrationParams,
                                                  imuCalibration, settings, delayedMarginalization, linearizeOperation,
                                                  callOnInit, *this);

    transitionModel = createTransitionModel(InitTransitionMode(settings.transitionModel), *logic);
    setState(transitionModel->getInitialState());
}

dmvio::IMUInitializer::~IMUInitializer() = default;

void dmvio::IMUInitializer::addIMUData(const dmvio::IMUData& data, int frameId)
{
    currentCoarseFrameId = frameId;
    currentCoarseIMUData = std::make_unique<IMUData>(data);
}

void dmvio::IMUInitializer::addPose(const dso::FrameShell& shell, bool willBecomeKeyframe)
{
    assert(shell.id == currentCoarseFrameId || !currentCoarseIMUData);
    std::unique_ptr<IMUInitializerState> newState;
    {
        // Forward to current state...
        std::shared_lock<std::shared_timed_mutex> lock(mutex);
        newState = currentState->addPose(shell, willBecomeKeyframe, currentCoarseIMUData.get());
    }
    // ... and change state if necessary.
    // For changing the state we need a unique_lock.
    lockAndSetState(std::move(newState));
}

bool dmvio::IMUInitializer::initializeIfReady()
{
    std::pair<std::unique_ptr<IMUInitializerState>, bool> ret;
    {
        std::shared_lock<std::shared_timed_mutex> lock(mutex);
        ret = currentState->initializeIfReady();
    }
    lockAndSetState(std::move(ret.first));
    return ret.second;
}

void dmvio::IMUInitializer::postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor,
                                       const gtsam::Values& baValues, double timestamp,
                                       const gtsam::PreintegratedImuMeasurements& imuMeasurements)
{
    std::unique_ptr<IMUInitializerState> newState;
    {
        std::shared_lock<std::shared_timed_mutex> lock(mutex);
        newState = currentState->postBAInit(keyframeId, activeHBFactor, baValues, timestamp, imuMeasurements);
    }
    lockAndSetState(std::move(newState));
}

const gtsam::imuBias::ConstantBias& dmvio::IMUInitializer::getLatestBias() const
{
    return logic->latestBias;
}

void dmvio::IMUInitializer::setState(std::unique_ptr<IMUInitializerState>&& newState)
{
    if(newState)
    {
        // Change state, the caller is responsible for making sure we have a lock.
        currentState = std::move(newState);
        std::cout << "Switching to initializer state: " << *currentState << std::endl;
    }
}

void dmvio::IMUInitializer::lockAndSetState(std::unique_ptr<IMUInitializerState>&& newState)
{
    if(newState)
    {
        // For changing the state we need a unique_lock.
        std::unique_lock<std::shared_timed_mutex> lock(mutex);
        setState(std::move(newState));
    }
}

std::unique_lock<std::shared_timed_mutex> dmvio::IMUInitializer::acquireSetStateLock()
{
    return std::unique_lock<std::shared_timed_mutex>(mutex);
}

bool dmvio::thresholdVariableChanges(IMUThresholdSettings settings, const gtsam::Values& baValues,
                                     const gtsam::Values& fejValues)
{
    gtsam::Key scaleKey = gtsam::Symbol('s', 0);
    gtsam::Key gravKey = gtsam::Symbol('g', 0);

    if(fejValues.exists(scaleKey))
    {
        double scale = baValues.at<ScaleGTSAM>(scaleKey).scale;
        double fejScale = fejValues.at<ScaleGTSAM>(scaleKey).scale;
        double scaleDiff = scale / fejScale;
        if(scaleDiff < 1.0)
        {
            scaleDiff = 1.0 / scaleDiff;
        }
        if(scaleDiff > settings.threshScale) return true;
    }

    if(fejValues.exists(gravKey))
    {
        Sophus::SO3d R_dsoW_metricW(baValues.at<gtsam::Rot3>(gravKey).matrix());
        Sophus::SO3d fejR_dsoW_metricW(fejValues.at<gtsam::Rot3>(gravKey).matrix());
        double gravDiff = (R_dsoW_metricW.inverse() * fejR_dsoW_metricW).log().norm();
        if(gravDiff > settings.threshGravdir) return true;
    }

    return false;
}

