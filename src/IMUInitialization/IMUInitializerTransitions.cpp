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

#include "IMUInitializerTransitions.h"

using namespace dmvio;

std::unique_ptr<dmvio::StateTransitionModel>
dmvio::createTransitionModel(InitTransitionMode mode, IMUInitializerLogic& logic)
{
    switch(mode)
    {
        case InitTransitionMode::COMBINED_MODEL:
            return std::make_unique<CombinedTransitionModel>(logic);
        case InitTransitionMode::COMBINED_REPLACEMENT_MODEL:
            return std::make_unique<CombinedWithMarginalizationReplacementModel>(logic);
        case InitTransitionMode::COMBINED_NO_INITIAL_REPLACEMENT:
            return std::make_unique<CombinedTransitionModelNoInitialMarginalizationReplacement>(logic);
        case InitTransitionMode::ONLY_COARSE_IMU_INIT:
            return std::make_unique<OnlyCoarseIMUInitTransitionModel>(logic);
        default:
            std::cout << "ERROR: Trying to use non-implemented transition model!" << std::endl;
            assert(false);
    }
}

dmvio::CombinedTransitionModel::CombinedTransitionModel(dmvio::IMUInitializerLogic& logic)
        : logic(logic)
{}

dmvio::IMUInitializerState::unique_ptr dmvio::CombinedTransitionModel::getInitialState()
{
    // start with coarse imu init.
    return createCoarseIMUInitState(logic, *this);
}

dmvio::IMUInitializerState::unique_ptr
dmvio::CombinedTransitionModel::coarseIMUInitOptimized(const gtsam::Values& optimizedValues,
                                                       dmvio::IMUInitVariances initVariances)
{
    // after coarse imu init continue with pgba.
    if(!initVariances.indetermined && initVariances.scaleVariance < logic.settings.coarseScaleUncertaintyThresh)
    {
        return createPGBAState(logic, *this, std::make_unique<gtsam::Values>(optimizedValues));
    }
    return nullptr;
}

std::pair<bool, IMUInitializerState::unique_ptr>
dmvio::CombinedTransitionModel::pgbaOptimized(std::unique_ptr<gtsam::Values>& optimizedValues,
                                              dmvio::IMUInitVariances initVariances)
{
    // We distinguish the first initialization vs reinitialization.
    if(lastInitUncert.indetermined)
    {
        // Takeover initialization if < threshold.
        if(!initVariances.indetermined &&
           initVariances.scaleVariance < logic.settings.pgbaSettings.scaleUncertaintyThresh)
        {
            return takeOverLargeBAOptim(optimizedValues, std::move(initVariances));
        }else
        {
            // If init failed we try again with the CoarseIMUInitOptimizer next time.
            return std::make_pair(false, createCoarseIMUInitState(logic, *this));
        }
    }else
    {
        // Takeover initialization if uncertainty < last init.
        if(!initVariances.indetermined && initVariances.scaleVariance <= lastInitUncert.scaleVariance)
        {
            return takeOverLargeBAOptim(optimizedValues, std::move(initVariances));
        }else
        {
            // Try reinit again (note that we don't go back to the CoarseIMUInitOptimizer here).
            return std::make_pair(false, nullptr);
        }
    }
}

std::pair<bool, IMUInitializerState::unique_ptr>
dmvio::CombinedTransitionModel::takeOverLargeBAOptim(std::unique_ptr<gtsam::Values>& optimizedValues,
                                                     IMUInitVariances&& initVariances)
{
    lastInitUncert = std::move(initVariances);
    // The state is responsible for creating either the InitializedFromPGBAState or the InitializedFromRealtimePGBAState
    return std::make_pair(true, nullptr);
}

dmvio::IMUInitializerState::unique_ptr dmvio::CombinedTransitionModel::initialized()
{
    if(lastInitUncert.scaleVariance > logic.settings.pgbaSettings.reinitScaleUncertaintyThresh)
    {
        // Reinit as long as the reinit threshold is exceeded.
        // Giving no initValues will try to use the newest values from BA.
        return createPGBAState(logic, *this, nullptr);
    }else
    {
        // Else we stop.
        return std::make_unique<InactiveIMUInitializerState>();
    }
}

dmvio::CombinedWithMarginalizationReplacementModel::CombinedWithMarginalizationReplacementModel(
        dmvio::IMUInitializerLogic& logic)
        : CombinedTransitionModel(logic)
{}

dmvio::IMUInitializerState::unique_ptr dmvio::CombinedWithMarginalizationReplacementModel::initialized()
{
    if(lastInitUncert.scaleVariance > logic.settings.pgbaSettings.reinitScaleUncertaintyThresh)
    {
        // Reinit as long as the reinit threshold is exceeded.
        // Giving no initValues will try to use the newest values from BA.
        return createPGBAState(logic, *this, nullptr);
    }else
    {
        // Else we continue with potential marginalization replacement.
        assert(startIdSecondTh >= 0);
        return std::make_unique<PotentialMarginalizationReplacementState>(logic, *this, startIdSecondTh);
    }
}

std::pair<bool, IMUInitializerState::unique_ptr>
dmvio::CombinedWithMarginalizationReplacementModel::takeOverLargeBAOptim(
        std::unique_ptr<gtsam::Values>& optimizedValues,
        IMUInitVariances&& initVariances)
{
    // We need to find out when more than 50% of IMU factors will be lost.
    // From then on the second threshold will be used (which we set to infinity in practice, meaning that from then
    // on no marginalization replacement will be done anymore).
    int n = (int) (logic.settings.percentageSwitchToSecondTH * logic.settings.pgbaSettings.delay);
    startIdSecondTh = logic.pgba->getIdOfNthIMUData(n);
    return CombinedTransitionModel::takeOverLargeBAOptim(optimizedValues, std::move(initVariances));
}

dmvio::IMUInitializerState::unique_ptr
dmvio::CombinedWithMarginalizationReplacementModel::marginalizationReplacementReady(
        std::unique_ptr<gtsam::Values>&& optimizedValues)
{
    // Update startId for second TH.
    int n = (int) (logic.settings.percentageSwitchToSecondTH * logic.settings.pgbaSettings.delay);
    startIdSecondTh = logic.pgba->getIdOfNthIMUData(n);
    return std::make_unique<MarginalizationReplacementReadyState>(logic, *this, std::move(optimizedValues));
}

dmvio::IMUInitializerState::unique_ptr dmvio::CombinedWithMarginalizationReplacementModel::marginalizationReplaced()
{
    // Potentially do more marginalization replacements in the future.
    return std::make_unique<PotentialMarginalizationReplacementState>(logic, *this, startIdSecondTh);
}

dmvio::CombinedTransitionModelNoInitialMarginalizationReplacement::CombinedTransitionModelNoInitialMarginalizationReplacement(
        dmvio::IMUInitializerLogic& logic) : CombinedTransitionModel(logic)
{}

std::pair<bool, IMUInitializerState::unique_ptr>
dmvio::CombinedTransitionModelNoInitialMarginalizationReplacement::takeOverLargeBAOptim(
        std::unique_ptr<gtsam::Values>& optimizedValues,
        IMUInitVariances&& initVariances)
{
    lastInitUncert = std::move(initVariances);
    return std::make_pair(true, std::make_unique<NoMarginalizationReplacementInitializedFromPGBAState>(logic, *this,
                                                                                                       std::move(
                                                                                                               optimizedValues)));
}

dmvio::OnlyCoarseIMUInitTransitionModel::OnlyCoarseIMUInitTransitionModel(dmvio::IMUInitializerLogic& logic)
        : logic(logic)
{}

dmvio::IMUInitializerState::unique_ptr dmvio::OnlyCoarseIMUInitTransitionModel::getInitialState()
{
    return createCoarseIMUInitState(logic, *this);
}

dmvio::IMUInitializerState::unique_ptr
dmvio::OnlyCoarseIMUInitTransitionModel::coarseIMUInitOptimized(const gtsam::Values& optimizedValues,
                                                                dmvio::IMUInitVariances initVariances)
{
    if(!initVariances.indetermined && initVariances.scaleVariance < logic.settings.coarseScaleUncertaintyThresh)
    {
        return std::make_unique<InitializedFromCoarseIMUInitState>(logic, *this);
    }
    return nullptr;
}

std::pair<bool, IMUInitializerState::unique_ptr>
dmvio::OnlyCoarseIMUInitTransitionModel::pgbaOptimized(std::unique_ptr<gtsam::Values>& optimizedValues,
                                                       dmvio::IMUInitVariances initVariances)
{
    assert(false);
    return std::make_pair(false, nullptr);
}

dmvio::IMUInitializerState::unique_ptr dmvio::OnlyCoarseIMUInitTransitionModel::initialized()
{
    if(lastInitUncert.scaleVariance > logic.settings.pgbaSettings.reinitScaleUncertaintyThresh)
    {
        // Reinit as long as the reinit threshold is exceeded.
        // Giving no initValues will try to use the newest values from BA.
        return createCoarseIMUInitState(logic, *this);
    }else
    {
        // Else we stop.
        return std::make_unique<InactiveIMUInitializerState>();
    }
}