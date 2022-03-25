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

#ifndef DMVIO_IMUINITIALIZERTRANSITIONS_H
#define DMVIO_IMUINITIALIZERTRANSITIONS_H

#include "IMUInitializerStates.h"

namespace dmvio
{

// Defines how to transition between states. There are multiple different transition models for the main method and
// ablation studies.
class StateTransitionModel
{
public:
    virtual ~StateTransitionModel() = default;

    // Return the initial state.
    virtual IMUInitializerState::unique_ptr getInitialState() = 0;

    // Called when the coarse IMU init optimization has finished and returns the new state.
    virtual IMUInitializerState::unique_ptr coarseIMUInitOptimized(const gtsam::Values& optimizedValues,
                                                                   dmvio::IMUInitVariances initVariances) = 0;

    // Called when a PGBA has been finished. If return.first is true the result of this optimization shall
    // be taken over. In this case, if return.second is nullptr the caller state is responsible for creating a proper new state itself.
    // If return.first is true the also returned state shall always be used.
    // Note that the passed optimizedValues can be moved away, if pair.second != nullptr.
    virtual std::pair<bool, IMUInitializerState::unique_ptr>
    pgbaOptimized(std::unique_ptr<gtsam::Values>& optimizedValues, IMUInitVariances initVariances) = 0;

    virtual IMUInitializerState::unique_ptr initialized() = 0;

    virtual IMUInitializerState::unique_ptr
    marginalizationReplacementReady(std::unique_ptr<gtsam::Values>&& optimizedValues)
    {};

    virtual IMUInitializerState::unique_ptr marginalizationReplaced()
    {};
};

// Enumeration for the available transition models.
// (The legacy options were removed but are still in the enum so that the indices are the same as earlier version).
// - 2 (COMBINED_REPLACEMENT_MODEL) is the default
// - 1 is no marginalization replacement ablation
// - 4 is no initial readvancing ablation
// - 5 is no PGBA ablation.
enum class InitTransitionMode
{
    LEGACY_UNUSED, COMBINED_MODEL, COMBINED_REPLACEMENT_MODEL, LEGACY_UNUSED2, COMBINED_NO_INITIAL_REPLACEMENT,
    ONLY_COARSE_IMU_INIT
};

std::unique_ptr<StateTransitionModel>
createTransitionModel(InitTransitionMode mode, IMUInitializerLogic& logic);


// This is similar to the transition model shown in the paper but without the marginalization replacement.
// TransitionModel with 2 thresholds, where reinitialization is performed as long as the scale uncertainty is greater
// than a second larger threshold.
class CombinedTransitionModel : public StateTransitionModel
{
public:
    // A reference to logic is kept and must be alive together with this object.
    CombinedTransitionModel(IMUInitializerLogic& logic);

    IMUInitializerState::unique_ptr getInitialState() override;

    IMUInitializerState::unique_ptr
    coarseIMUInitOptimized(const gtsam::Values& optimizedValues, dmvio::IMUInitVariances initVariances) override;

    std::pair<bool, IMUInitializerState::unique_ptr> pgbaOptimized(
            std::unique_ptr<gtsam::Values>& optimizedValues,
            IMUInitVariances initVariances) override;

    IMUInitializerState::unique_ptr initialized() override;

protected:
    IMUInitializerLogic& logic;
    IMUInitVariances lastInitUncert;

    virtual std::pair<bool, IMUInitializerState::unique_ptr>
    takeOverLargeBAOptim(std::unique_ptr<gtsam::Values>& optimizedValues,
                         IMUInitVariances&& initVariances);
};

// This is the full transition model as shown in the paper.
// We base it on the CombinedTransitionModel and only override the methods which need to be changed.
class CombinedWithMarginalizationReplacementModel : public CombinedTransitionModel
{
public:
    CombinedWithMarginalizationReplacementModel(IMUInitializerLogic& logic);

    IMUInitializerState::unique_ptr initialized() override;

    virtual IMUInitializerState::unique_ptr
    marginalizationReplacementReady(std::unique_ptr<gtsam::Values>&& optimizedValues) override;

    virtual IMUInitializerState::unique_ptr marginalizationReplaced() override;

protected:
    std::pair<bool, IMUInitializerState::unique_ptr>
    takeOverLargeBAOptim(std::unique_ptr<gtsam::Values>& optimizedValues,
                         IMUInitVariances&& initVariances) override;

    int startIdSecondTh = -1;
};

// For ablations: This transition model does not do a marginalization replacement after initializing (despite using
// PGBA).
class CombinedTransitionModelNoInitialMarginalizationReplacement : public CombinedTransitionModel
{
public:
    CombinedTransitionModelNoInitialMarginalizationReplacement(IMUInitializerLogic& logic);

protected:
    std::pair<bool, IMUInitializerState::unique_ptr>
    takeOverLargeBAOptim(std::unique_ptr<gtsam::Values>& optimizedValues,
                         IMUInitVariances&& initVariances) override;
};

// This transition model only uses the CoarseIMUInit.
class OnlyCoarseIMUInitTransitionModel : public StateTransitionModel
{
public:
    // A reference to logic is kept and must be alive together with this object.
    OnlyCoarseIMUInitTransitionModel(IMUInitializerLogic& logic);

    IMUInitializerState::unique_ptr getInitialState() override;

    IMUInitializerState::unique_ptr
    coarseIMUInitOptimized(const gtsam::Values& optimizedValues, dmvio::IMUInitVariances initVariances) override;

    std::pair<bool, IMUInitializerState::unique_ptr> pgbaOptimized(
            std::unique_ptr<gtsam::Values>& optimizedValues,
            IMUInitVariances initVariances) override;

    IMUInitializerState::unique_ptr initialized() override;

protected:
    IMUInitializerLogic& logic;
    IMUInitVariances lastInitUncert;
};

}

#endif //DMVIO_IMUINITIALIZERTRANSITIONS_H
