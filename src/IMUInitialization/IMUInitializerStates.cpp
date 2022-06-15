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
#include "IMUInitializerStates.h"
#include "IMUInitializerLogic.h"
#include "IMUInitializer.h"
#include "IMUInitializerTransitions.h"
#include "GTSAMIntegration/GTSAMUtils.h"

using namespace dmvio;

dmvio::DefaultActiveIMUInitializerState::DefaultActiveIMUInitializerState(IMUInitializerLogic& imuInitLogic,
                                                                          StateTransitionModel& transitionModel)
        : logic(imuInitLogic), transitionModel(transitionModel)
{}

std::unique_ptr<IMUInitializerState>
dmvio::DefaultActiveIMUInitializerState::addPose(const dso::FrameShell& shell, bool willBecomeKeyframe,
                                                 const IMUData* imuData)
{
    logic.addPose(shell, willBecomeKeyframe, imuData);
    return nullptr;
}

std::unique_ptr<IMUInitializerState>
dmvio::DefaultActiveIMUInitializerState::postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor,
                                                    const gtsam::Values& baValues, double timestamp,
                                                    const gtsam::PreintegratedImuMeasurements& imuMeasurements)
{
    // This is actually not called for the very first KF which is what we want in this case.
    logic.pgba->addKeyframe(keyframeId, imuMeasurements);
    return nullptr;
}

std::pair<std::unique_ptr<IMUInitializerState>, bool> dmvio::DefaultActiveIMUInitializerState::initializeIfReady()
{
    return std::make_pair(nullptr, false);
}

dmvio::CoarseIMUInitState::CoarseIMUInitState(dmvio::IMUInitializerLogic& imuInitLogic,
                                              StateTransitionModel& transitionModel)
        : DefaultActiveIMUInitializerState(imuInitLogic, transitionModel)
{}

std::unique_ptr<IMUInitializerState>
dmvio::CoarseIMUInitState::addPose(const dso::FrameShell& shell, bool willBecomeKeyframe,
                                   const IMUData* imuData)
{
    DefaultActiveIMUInitializerState::addPose(shell, willBecomeKeyframe, imuData);

    if(logic.coarseIMUOptimizer->numFrames > 5 && willBecomeKeyframe)
    {
        IMUInitVariances variances = logic.performCoarseIMUInit(shell.timestamp);

        if(!variances.indetermined)
        {
            // Maybe adding a bias covariance threshold here would be a good idea.
            logic.latestBias = logic.coarseIMUOptimizer->getBias();
        }

        // Get new state from transition model.
        auto newState = transitionModel.coarseIMUInitOptimized(logic.coarseIMUOptimizer->optimizedValues, variances);
        if(newState)
        {
            // This means that the initialization was successful and the CoarseIMUOptimizer can take over the values
            // to the new optimization.
            logic.coarseIMUOptimizer->takeOverOptimizedValues();
            logic.pgba->removeIMUFactorsUntil = logic.coarseIMUOptimizer->imuFactorsRemovedUntil;
            return std::move(newState);
        }
    }
    return nullptr;
}

void dmvio::CoarseIMUInitState::print(std::ostream& str) const
{
    str << "CoarseIMUInit";
}

dmvio::RealtimeCoarseIMUInitState::RealtimeCoarseIMUInitState(dmvio::IMUInitializerLogic& imuInitLogic,
                                                              dmvio::StateTransitionModel& transitionModel)
        : DefaultActiveIMUInitializerState(imuInitLogic, transitionModel)
{}

std::unique_ptr<IMUInitializerState>
dmvio::RealtimeCoarseIMUInitState::addPose(const dso::FrameShell& shell, bool willBecomeKeyframe,
                                           const IMUData* imuData)
{
    dmvio::TimeMeasurement meas("RealtimeCoarseIMUInitState::addPose");
    switch(status)
    {
        case NOT_RUNNING:
            DefaultActiveIMUInitializerState::addPose(shell, willBecomeKeyframe, imuData);
            if(logic.coarseIMUOptimizer->numFrames > 5 && willBecomeKeyframe && !dso::setting_fullResetRequested)
            {
                optimizingTimestamp = shell.timestamp;
                // perform optimization in separate thread.
                runthread = std::thread{&RealtimeCoarseIMUInitState::threadRun, this};
                status = RUNNING;
                runthread.detach();
            }
            break;
        case RUNNING:
            // Save data coming in while the thread is running.
            cachedData.emplace_back(&shell, willBecomeKeyframe, *imuData);
            break;
    }
    return nullptr;
}

void dmvio::RealtimeCoarseIMUInitState::threadRun()
{
    dmvio::TimeMeasurement timeMeasurement("RealtimeCoarseIMUInitState::threadRun");
    IMUInitVariances variances = logic.performCoarseIMUInit(optimizingTimestamp);

    if(!variances.indetermined)
    {
        // Maybe adding a bias covariance threshold here would be a good idea.
        logic.latestBias = logic.coarseIMUOptimizer->getBias();
    }

    auto newState = transitionModel.coarseIMUInitOptimized(logic.coarseIMUOptimizer->optimizedValues, variances);
    if(newState)
    {
        // This means that the initialization was successful and the CoarseIMUInitOptimizer can take over the values to
        // the new optimization.
        logic.coarseIMUOptimizer->takeOverOptimizedValues();
        logic.pgba->removeIMUFactorsUntil = logic.coarseIMUOptimizer->imuFactorsRemovedUntil;
    }

    // Acquire lock to change the state. This ensures that the addPose method will not be called for this state in
    // the meantime, which would lead to IMU data being lost.
    auto lock = logic.stateChanger.acquireSetStateLock();
    // Add cached imu data and poses.
    for(auto&& data : cachedData)
    {
        DefaultActiveIMUInitializerState::addPose(*std::get<0>(data), std::get<1>(data), &std::get<2>(data));
    }
    cachedData.clear();
    if(!newState) status = NOT_RUNNING;
    logic.stateChanger.setState(std::move(newState));
}

void dmvio::RealtimeCoarseIMUInitState::print(std::ostream& str) const
{
    str << "RealtimeCoarseIMUInitState";
}

std::unique_ptr<dmvio::IMUInitializerState>
dmvio::createCoarseIMUInitState(dmvio::IMUInitializerLogic& imuInitLogic, dmvio::StateTransitionModel& transitionModel)
{
    if(imuInitLogic.realtimeCoarseIMUInit)
    {
        return std::make_unique<RealtimeCoarseIMUInitState>(imuInitLogic, transitionModel);
    }else
    {
        return std::make_unique<CoarseIMUInitState>(imuInitLogic, transitionModel);
    }
}

std::unique_ptr<IMUInitializerState>
dmvio::createPGBAState(IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel,
                       std::unique_ptr<gtsam::Values>&& initValues)
{
    if(imuInitLogic.realtimePGBA)
    {
        return std::make_unique<RealtimePGBAState>(imuInitLogic, transitionModel, std::move(initValues));
    }else
    {
        return std::make_unique<PGBAState>(imuInitLogic, transitionModel, std::move(initValues));
    }
}

dmvio::PGBAState::PGBAState(dmvio::IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel,
                            std::unique_ptr<gtsam::Values>&& initValuesPassed)
        : DefaultActiveIMUInitializerState(imuInitLogic, transitionModel), initValues(std::move(initValuesPassed))
{}

gtsam::Values* prepareInitValuesForPGBA(std::unique_ptr<gtsam::Values>& initValuesNew,
                                        const std::unique_ptr<gtsam::Values>& initValues,
                                        const gtsam::Values& baValues, dmvio::IMUInitializerLogic& logic)
{

    if(!initValues)
    {
        // If BA
        if(baValues.exists(gtsam::Symbol('s', 0)))
        {
            // Note: These delayedCurrValues also contain IMU variables (even though the graph doesn't) (only if IMU
            // is initialized in the main graph).
            initValuesNew = std::make_unique<gtsam::Values>(logic.pgba->getInputDelayedGraph()->getDelayedCurrValues());
            for(auto&& pair : baValues)
            {
                eraseAndInsert(*initValuesNew, pair.key, pair.value);
            }
        }else
        {
            initValuesNew = std::make_unique<gtsam::Values>();
        }
        return initValuesNew.get();
    }else
    {
        for(auto&& pair : baValues)
        {
            if(!initValues->exists(pair.key))
            {
                initValues->insert(pair.key, pair.value);
            }
        }
        return initValues.get();
    }
}

std::unique_ptr<IMUInitializerState>
dmvio::PGBAState::postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor,
                             const gtsam::Values& baValues,
                             double timestamp, const gtsam::PreintegratedImuMeasurements& imuMeasurements)
{
    DefaultActiveIMUInitializerState::postBAInit(keyframeId, activeHBFactor, baValues, timestamp, imuMeasurements);

    if(logic.pgba->getNumKFs() < 3)
    {
        return nullptr;
    }

    // Don't overwrite member variable to not influence the potential next optimization.
    std::unique_ptr<gtsam::Values> initValuesNew;
    gtsam::Values* initValuesUsed = prepareInitValuesForPGBA(initValuesNew, initValues, baValues, logic);

    try
    {
        logic.pgba->prepareOptimization();
        auto optimizedValues = std::make_unique<gtsam::Values>(
                logic.pgba->optimize(activeHBFactor, baValues, *initValuesUsed, false));

        logic.transformDSOToIMUAfterPGBA->updateWithValues(*optimizedValues);

        IMUInitVariances variances(logic.pgba->getMarginals(*optimizedValues),
                                   gtsam::Symbol('s', logic.transformDSOToIMUAfterPGBA->getSymbolInd()),
                                   logic.pgba->getBiasKey());

        // Write out result!
        auto&& bias = optimizedValues->at<gtsam::imuBias::ConstantBias>(logic.pgba->getBiasKey());
        if(!variances.indetermined)
        {
            std::cout << "PGBA: " << logic.transformDSOToIMUAfterPGBA->getScale() << " var: " << variances.scaleVariance
                      << " " << variances.biasCovariance.diagonal().transpose() << std::endl;
            logic.latestBias = bias;
        }

        auto pair = transitionModel.pgbaOptimized(optimizedValues, variances);
        if(pair.first)
        {
            if(pair.second) return std::move(pair.second);
            // If values are taken, and pair.second is not nullptr, over we are responsible for creating the new state.
            auto newState = std::make_unique<InitializedFromPGBAState>(logic, transitionModel,
                                                                       std::move(optimizedValues));
            return std::move(newState);
        }else
        {
            logic.pgba->optimizationResultNotUsed();
            return std::move(pair.second);
        }
    }catch(gtsam::IndeterminantLinearSystemException& exc)
    {
        std::cout << "ERROR during PGBA!" << std::endl;
        logic.pgba->optimizationResultNotUsed();
        std::unique_ptr<gtsam::Values> emptyVals;
        return transitionModel.pgbaOptimized(emptyVals, IMUInitVariances()).second;
    }

}

void dmvio::PGBAState::print(std::ostream& str) const
{
    str << "PGBAState";
}


dmvio::RealtimePGBAState::RealtimePGBAState(dmvio::IMUInitializerLogic& imuInitLogic,
                                            StateTransitionModel& transitionModel,
                                            std::unique_ptr<gtsam::Values>&& initValuesPassed)
        : DefaultActiveIMUInitializerState(imuInitLogic, transitionModel), initValues(std::move(initValuesPassed))
{}

std::unique_ptr<IMUInitializerState>
dmvio::RealtimePGBAState::postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor,
                                     const gtsam::Values& baValues,
                                     double timestamp, const gtsam::PreintegratedImuMeasurements& imuMeasurements)
{
    dmvio::TimeMeasurement meas("RealtimePGBAState::postBAInit");
    if(running)
    {
        cachedData.emplace_back(imuMeasurements, keyframeId);
    }else
    {
        DefaultActiveIMUInitializerState::postBAInit(keyframeId, activeHBFactor, baValues, timestamp, imuMeasurements);

        if(logic.pgba->getNumKFs() < 3)
        {
            return nullptr;
        }

        // Prepare optimization
        initValuesUsed = prepareInitValuesForPGBA(initValuesNew, initValues, baValues, logic);
        this->activeHBFactor = activeHBFactor;
        optimizingTimestamp = timestamp;
        this->baValues = baValues;

        logic.pgba->prepareOptimization();

        // Start optimization thread
        std::thread runthread{&RealtimePGBAState::threadRun, this};
        running = true;
        runthread.detach();
    }
    return nullptr;
}

void RealtimePGBAState::threadRun()
{
    dmvio::TimeMeasurement meas("RealtimePGBAState::threadRun");
    std::pair<bool, IMUInitializerState::unique_ptr> newStatePair;
    std::unique_ptr<gtsam::Values> optimizedValues;
    try
    {
        optimizedValues = std::make_unique<gtsam::Values>(
                logic.pgba->optimize(activeHBFactor, baValues, *initValuesUsed, false));
        logic.transformDSOToIMUAfterPGBA->updateWithValues(*optimizedValues);

        IMUInitVariances variances(logic.pgba->getMarginals(*optimizedValues),
                                   gtsam::Symbol('s', logic.transformDSOToIMUAfterPGBA->getSymbolInd()),
                                   logic.pgba->getBiasKey());

        // Write out result!
        auto&& bias = optimizedValues->at<gtsam::imuBias::ConstantBias>(logic.pgba->getBiasKey());
        if(!variances.indetermined)
        {
            std::cout << "PGBA: " << logic.transformDSOToIMUAfterPGBA->getScale() << " var: " << variances.scaleVariance
                      << " " << variances.biasCovariance.diagonal().transpose() << std::endl;
            logic.latestBias = bias;
        }

        newStatePair = transitionModel.pgbaOptimized(optimizedValues, variances);
    }catch(gtsam::IndeterminantLinearSystemException& exc)
    {
        std::cout << "ERROR during PGBA!" << std::endl;
        std::unique_ptr<gtsam::Values> emptyVals;
        newStatePair = transitionModel.pgbaOptimized(emptyVals, IMUInitVariances());
    }
    initValuesNew.reset();

    if(!newStatePair.first) // Don't take over result (but maybe switch to new state provided by transition)
    {
        logic.pgba->optimizationResultNotUsed();

        // Lock and add new things.
        // Acquire lock to change the state. This ensures that the postBAInit method will not be called for this
        // state in the meantime, which would lead to data being lost.
        auto lock = logic.stateChanger.acquireSetStateLock();

        // Add cached imu data and poses.
        for(auto&& data : cachedData)
        {
            logic.pgba->addKeyframe(data.second, std::move(data.first));
        }
        cachedData.clear();

        if(!newStatePair.second) running = false; // do one or more optimization.

        logic.stateChanger.setState(std::move(newStatePair.second));

    }else // Takeover result.
    {
        assert(!newStatePair.second); // other state created not yet supported for RT mode.

        // Unroll graph as far as we can in this separate thread..
        logic.pgba->preparePreparation(*optimizedValues);

        // Lock before creating the new state (so that no cachedData gets lost)!
        auto lock = logic.stateChanger.acquireSetStateLock();

        // Pass cachedData to RealtimeInitializedState
        auto newState = std::make_unique<InitializedFromRealtimePGBAState>(logic, transitionModel,
                                                                           std::move(optimizedValues),
                                                                           std::move(cachedData));

        logic.stateChanger.setState(std::move(newState));
    }

}

void dmvio::RealtimePGBAState::print(std::ostream& str) const
{
    str << "RealtimePGBAState";
}

dmvio::InitializedFromPGBAState::InitializedFromPGBAState(dmvio::IMUInitializerLogic& imuInitLogic,
                                                          StateTransitionModel& transitionModel,
                                                          std::unique_ptr<gtsam::Values>&& optimizedValuesPassed)
        : DefaultActiveIMUInitializerState(imuInitLogic, transitionModel),
          optimizedValues(std::move(optimizedValuesPassed))
{}

std::pair<std::unique_ptr<IMUInitializerState>, bool> dmvio::InitializedFromPGBAState::initializeIfReady()
{
    auto delayedGraph = logic.pgba->prepareGraphForMainOptimization(*optimizedValues);

    logic.callOnInit(delayedGraph->getDelayedCurrValues(), true);
    delayedGraph->setMaxGroupInGraph(BAIMULogic::METRIC_GROUP); // From now on all factors should be added to this.
    logic.delayedMarginalizationGraphs->replaceMainGraph(std::move(delayedGraph));
    return std::make_pair(transitionModel.initialized(), true);
}

void dmvio::InitializedFromPGBAState::print(std::ostream& str) const
{
    str << "InitializedFromPGBAState";
}

dmvio::InitializedFromRealtimePGBAState::InitializedFromRealtimePGBAState(dmvio::IMUInitializerLogic& imuInitLogic,
                                                                          StateTransitionModel& transitionModel,
                                                                          std::unique_ptr<gtsam::Values>&& optimizedValuesPassed,
                                                                          PoseGraphBundleAdjustment::KeyframeDataContainer&& cachedDataPassed)
        : DefaultActiveIMUInitializerState(imuInitLogic, transitionModel),
          optimizedValues(std::move(optimizedValuesPassed)),
          cachedData(std::move(cachedDataPassed))
{}

std::unique_ptr<IMUInitializerState>
InitializedFromRealtimePGBAState::postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor,
                                             const gtsam::Values& baValues, double timestamp,
                                             const gtsam::PreintegratedImuMeasurements& imuMeasurements)
{
    // First add the newest KF.
    cachedData.emplace_back(imuMeasurements, keyframeId);

    optimizedValues = std::make_unique<gtsam::Values>(
            logic.pgba->extendGraph(activeHBFactor, std::move(*optimizedValues), baValues, cachedData));
    cachedData.clear();
    prepared = true;
    return nullptr;
}

std::pair<std::unique_ptr<IMUInitializerState>, bool> dmvio::InitializedFromRealtimePGBAState::initializeIfReady()
{
    if(!prepared)
        return std::make_pair(nullptr,
                              false); // postBAInit was not run yet, which means that the new factors have not been added.
    dmvio::TimeMeasurement meas("InitializedFromRealtimePGBAState::initializeIfReady");
    auto delayedGraph = logic.pgba->prepareGraphForMainOptimization(*optimizedValues);

    logic.callOnInit(delayedGraph->getDelayedCurrValues(), true);
    delayedGraph->setMaxGroupInGraph(BAIMULogic::METRIC_GROUP); // From now on all factors should be added to this.
    logic.delayedMarginalizationGraphs->replaceMainGraph(std::move(delayedGraph));
    return std::make_pair(transitionModel.initialized(), true);
}

void dmvio::InitializedFromRealtimePGBAState::print(std::ostream& str) const
{
    str << "InitializedFromRealtimePGBAState";
}

dmvio::NoMarginalizationReplacementInitializedFromPGBAState::NoMarginalizationReplacementInitializedFromPGBAState(
        dmvio::IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel,
        std::unique_ptr<gtsam::Values>&& optimizedValuesPassed)
        : DefaultActiveIMUInitializerState(imuInitLogic, transitionModel),
          optimizedValues(std::move(optimizedValuesPassed))
{}

std::pair<std::unique_ptr<IMUInitializerState>, bool>
dmvio::NoMarginalizationReplacementInitializedFromPGBAState::initializeIfReady()
{
    // If this ablation is done in RT mode (which it was not in the paper) building of the graph should be removed as
    // well.
    auto delayedGraph = logic.pgba->prepareGraphForMainOptimization(*optimizedValues);

    logic.callOnInit(delayedGraph->getDelayedCurrValues(), false);

    return std::make_pair(transitionModel.initialized(), true);
}

void dmvio::NoMarginalizationReplacementInitializedFromPGBAState::print(std::ostream& str) const
{
    str << "NoMarginalizationReplacementInitializedFromPGBAState";
}

dmvio::InitializedFromCoarseIMUInitState::InitializedFromCoarseIMUInitState(dmvio::IMUInitializerLogic& imuInitLogic,
                                                                            StateTransitionModel& transitionModel)
        : DefaultActiveIMUInitializerState(imuInitLogic, transitionModel)
{}

std::pair<std::unique_ptr<IMUInitializerState>, bool> dmvio::InitializedFromCoarseIMUInitState::initializeIfReady()
{
    logic.callOnInit(logic.coarseIMUOptimizer->optimizedValues, false);

    return std::make_pair(transitionModel.initialized(), true);
}

void dmvio::InitializedFromCoarseIMUInitState::print(std::ostream& str) const
{
    str << "InitializedFromCoarseIMUInitState";
}

dmvio::PotentialMarginalizationReplacementState::PotentialMarginalizationReplacementState(
        dmvio::IMUInitializerLogic& imuInitLogic, dmvio::StateTransitionModel& transitionModel, int startIdForSecondTh)
        : DefaultActiveIMUInitializerState(imuInitLogic, transitionModel), startIdForSecondTH(startIdForSecondTh)
{}

std::unique_ptr<IMUInitializerState> dmvio::PotentialMarginalizationReplacementState::postBAInit(int keyframeId,
                                                                                                 gtsam::NonlinearFactor::shared_ptr activeHBFactor,
                                                                                                 const gtsam::Values& baValues,
                                                                                                 double timestamp,
                                                                                                 const gtsam::PreintegratedImuMeasurements& imuMeasurements)
{
    DefaultActiveIMUInitializerState::postBAInit(keyframeId, activeHBFactor, baValues, timestamp, imuMeasurements);

    auto* fejVals = logic.delayedMarginalizationGraphs->getMainGraph()->fejValues.get();
    if(!fejVals) return nullptr;
    bool changed = thresholdVariableChanges(
            useSecondTH ? logic.settings.secondThresholdSettings : logic.settings.thresholdSettings, baValues,
            fejVals->fejValues);

    if(!changed) return nullptr;

    dmvio::TimeMeasurement meas("MarginalizationReplacementBuildGraph");

    gtsam::Values initValues(logic.pgba->getInputDelayedGraph()->getDelayedCurrValues());
    for(auto&& pair : baValues)
    {
        eraseAndInsert(initValues, pair.key, pair.value);
    }
    logic.pgba->prepareOptimization();
    // We pass noOptimization=true, so that the graph is only built, but not actually optimized.
    auto values = std::make_unique<gtsam::Values>(
            logic.pgba->optimize(activeHBFactor, baValues, initValues, true));

    // Get minimum connected pose and maybe switch to second threshold!
    int firstId = logic.pgba->getFirstIdWithIMUData();
    if(!useSecondTH && firstId >= startIdForSecondTH)
    {
        // From now on we need to use the second threshold.
        useSecondTH = true;
        // That means we need to check again if this threshold is met as well.
        bool reallyChanged = thresholdVariableChanges(logic.settings.secondThresholdSettings, baValues,
                                                      fejVals->fejValues);
        if(!reallyChanged)
        {
            std::cout << "IMUInitialization (switch): Found out that second threshold was not met!" << std::endl;
            logic.pgba->optimizationResultNotUsed();
            return nullptr;
        }
    }

    return transitionModel.marginalizationReplacementReady(std::move(values));
}

void dmvio::PotentialMarginalizationReplacementState::print(std::ostream& str) const
{
    str << "PotentialMarginalizationReplacementState";
}

dmvio::MarginalizationReplacementReadyState::MarginalizationReplacementReadyState(
        dmvio::IMUInitializerLogic& imuInitLogic,
        dmvio::StateTransitionModel& transitionModel,
        std::unique_ptr<gtsam::Values>&& optimizedValues)
        : DefaultActiveIMUInitializerState(imuInitLogic, transitionModel),
          optimizedValues(std::move(optimizedValues))
{}

std::pair<std::unique_ptr<IMUInitializerState>, bool> dmvio::MarginalizationReplacementReadyState::initializeIfReady()
{
    dmvio::TimeMeasurement meas("MarginalizationReplacementReplaceGraph");

    auto delayedGraph = logic.pgba->prepareGraphForMainOptimization(*optimizedValues);
    delayedGraph->setMaxGroupInGraph(BAIMULogic::METRIC_GROUP); // From now on all factors should be added to this.
    logic.delayedMarginalizationGraphs->replaceMainGraph(std::move(delayedGraph));

    return std::make_pair(transitionModel.marginalizationReplaced(), false);
}

void dmvio::MarginalizationReplacementReadyState::print(std::ostream& str) const
{
    str << "MarginalizationReplacementReadyState";
}
