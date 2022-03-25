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

#ifndef DMVIO_IMUINITIALIZERSTATES_H
#define DMVIO_IMUINITIALIZERSTATES_H

#include "dso/util/FrameShell.h"
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "IMU/IMUTypes.h"
#include "IMUInitializerLogic.h"
#include <thread>
#include <tuple>
#include "IMUInitStateChanger.h"

namespace dmvio
{

// Abstract base class for the states of our state machine.
// States can be switched in two ways:
// - The called method can return the new state (or nullptr if the same state should be kept).
// - By manually calling the methods of IMUInitStateChanger (stored in IMUInitializerLogic). This is for changing the
//   state in a separate thread and mainly used by the realtime states.
//   IMPORTANT: This should not be called inside the methods defined by IMUInitializerState, as they already have a lock
//   on the state (and it would result in a deadlock). They should instead just return the new state.
// Initialization is either performed in addPose (for the CoarseIMUInit) or in postBAInit (for the PGBA).
class IMUInitializerState
{
public:
    virtual ~IMUInitializerState() = default;

    // Add coarse pose for use in initialization.
    // Potentially run coarse IMU init.
    virtual std::unique_ptr<IMUInitializerState> addPose(const dso::FrameShell& shell, bool willBecomeKeyframe,
                                                         const IMUData* imuData) = 0;

    // Called after the Bundle Adjustment (BA), used by some states to run PGBA.
    virtual std::unique_ptr<IMUInitializerState>
    postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor, const gtsam::Values& baValues,
               double timestamp, const gtsam::PreintegratedImuMeasurements& imuMeasurements) = 0;

    // Called after all keyframe operations are finished. This is when the IMU in the main system should be
    // initialized (if the initialized values are ready). Returns true if an initialization is performed.
    virtual std::pair<std::unique_ptr<IMUInitializerState>, bool> initializeIfReady() = 0;

    friend std::ostream& operator<<(std::ostream& str, IMUInitializerState const& data)
    {
        data.print(str);
        return str;
    }

    using unique_ptr = std::unique_ptr<IMUInitializerState>;

protected:
    // print state name.
    virtual void print(std::ostream& str) const = 0;
};

// State for doing no initialization anymore.
class InactiveIMUInitializerState : public IMUInitializerState
{
public:
    std::unique_ptr<IMUInitializerState> addPose(const dso::FrameShell& shell, bool willBecomeKeyframe,
                                                 const IMUData* imuData) override
    { return nullptr; }

    std::unique_ptr<IMUInitializerState>
    postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor, const gtsam::Values& baValues,
               double timestamp, const gtsam::PreintegratedImuMeasurements& imuMeasurements) override
    { return nullptr; }

    std::pair<std::unique_ptr<IMUInitializerState>, bool> initializeIfReady() override
    {
        return std::make_pair(nullptr, false);
    }

    void print(std::ostream& str) const override
    {
        str << "InactiveIMUInitializerState";
    }
};

// Default state which forwards poses and keyframes to the CoarseIMUInitOptimizer and PoseGraphBundleAdjustment
// respectively, but doesn't do anything else.
class DefaultActiveIMUInitializerState : public IMUInitializerState
{
public:
    // We keep a reference to IMUInitializerLogic and transitionModel without using shared_ptr, as the parent
    // IMUInitializer is responsible for handling memory.
    DefaultActiveIMUInitializerState(IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel);

    std::unique_ptr<IMUInitializerState> addPose(const dso::FrameShell& shell, bool willBecomeKeyframe,
                                                 const IMUData* imuData) override;

    std::unique_ptr<IMUInitializerState>
    postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor, const gtsam::Values& baValues,
               double timestamp, const gtsam::PreintegratedImuMeasurements& imuMeasurements) override;

    std::pair<std::unique_ptr<IMUInitializerState>, bool> initializeIfReady() override;

protected:
    IMUInitializerLogic& logic;
    StateTransitionModel& transitionModel;
};

// State when the CoarseIMUInitOptimizer is the next step.
class CoarseIMUInitState : public DefaultActiveIMUInitializerState
{
public:
    CoarseIMUInitState(IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel);

    std::unique_ptr<IMUInitializerState> addPose(const dso::FrameShell& shell, bool willBecomeKeyframe,
                                                 const IMUData* imuData) override;

    void print(std::ostream& str) const override;
};

// Realtime version of the CoarseIMUInitState: Performs optimization in a separate thread.
class RealtimeCoarseIMUInitState : public DefaultActiveIMUInitializerState
{
public:
    RealtimeCoarseIMUInitState(IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel);

    std::unique_ptr<IMUInitializerState> addPose(const dso::FrameShell& shell, bool willBecomeKeyframe,
                                                 const IMUData* imuData) override;

    void print(std::ostream& str) const override;
private:
    void threadRun();

    enum ThreadStatus
    {
        NOT_RUNNING, RUNNING
    };
    ThreadStatus status = NOT_RUNNING;
    std::thread runthread;

    double optimizingTimestamp;
    using AddPoseData = std::tuple<const dso::FrameShell*, bool, IMUData>; // We need to save the pointers because
    // CoarseIMUInitOptimizer will use them to get the updated poses later.
    std::vector<AddPoseData> cachedData;
};

// depending on the value of imuInitLogic.realtimeCoaresIMUInit this creates either a CoarseIMUInitState or a
// RealtimeCoarseIMUInitState
std::unique_ptr<IMUInitializerState>
createCoarseIMUInitState(IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel);
// same as last method, but for PGBA.
std::unique_ptr<IMUInitializerState>
createPGBAState(IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel,
                std::unique_ptr<gtsam::Values>&& initValues);

class PGBAState : public DefaultActiveIMUInitializerState
{
public:
    PGBAState(IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel,
              std::unique_ptr<gtsam::Values>&& initValues);
    std::unique_ptr<IMUInitializerState>
    postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor, const gtsam::Values& baValues,
               double timestamp, const gtsam::PreintegratedImuMeasurements& imuMeasurements) override;

    void print(std::ostream& str) const override;
private:
    std::unique_ptr<gtsam::Values> initValues;
};

class RealtimePGBAState : public DefaultActiveIMUInitializerState
{
public:
    RealtimePGBAState(IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel,
                      std::unique_ptr<gtsam::Values>&& initValues);

    std::unique_ptr<IMUInitializerState>
    postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor, const gtsam::Values& baValues,
               double timestamp, const gtsam::PreintegratedImuMeasurements& imuMeasurements) override;

    void print(std::ostream& str) const override;

    void threadRun();

private:
    // Variables for the optimization
    std::unique_ptr<gtsam::Values> initValues;
    std::unique_ptr<gtsam::Values> initValuesNew; // this one is deleted after every optimization.
    gtsam::Values* initValuesUsed = nullptr; // points to one of the above vars.
    gtsam::NonlinearFactor::shared_ptr activeHBFactor;
    gtsam::Values baValues;

    bool running = false;
    double optimizingTimestamp;
    PoseGraphBundleAdjustment::KeyframeDataContainer cachedData;
};

class InitializedFromPGBAState : public DefaultActiveIMUInitializerState
{
public:
    InitializedFromPGBAState(IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel,
                             std::unique_ptr<gtsam::Values>&& optimizedValues);

    std::pair<std::unique_ptr<IMUInitializerState>, bool> initializeIfReady() override;

    void print(std::ostream& str) const override;
private:
    std::unique_ptr<gtsam::Values> optimizedValues;
};

class InitializedFromRealtimePGBAState : public DefaultActiveIMUInitializerState
{
public:
    InitializedFromRealtimePGBAState(IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel,
                                     std::unique_ptr<gtsam::Values>&& optimizedValues,
                                     PoseGraphBundleAdjustment::KeyframeDataContainer&& cachedData);

    std::unique_ptr<IMUInitializerState>
    postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor, const gtsam::Values& baValues,
               double timestamp, const gtsam::PreintegratedImuMeasurements& imuMeasurements) override;

    std::pair<std::unique_ptr<IMUInitializerState>, bool> initializeIfReady() override;

    void print(std::ostream& str) const override;
private:
    std::unique_ptr<gtsam::Values> optimizedValues;
    PoseGraphBundleAdjustment::KeyframeDataContainer cachedData;
    bool prepared = false;
};


// For ablations: do not perform the initial marginalization replacement.
class NoMarginalizationReplacementInitializedFromPGBAState : public DefaultActiveIMUInitializerState
{
public:
    NoMarginalizationReplacementInitializedFromPGBAState(IMUInitializerLogic& imuInitLogic,
                                                         StateTransitionModel& transitionModel,
                                                         std::unique_ptr<gtsam::Values>&& optimizedValues);

    std::pair<std::unique_ptr<IMUInitializerState>, bool> initializeIfReady() override;

    void print(std::ostream& str) const override;
private:
    std::unique_ptr<gtsam::Values> optimizedValues;
};

class InitializedFromCoarseIMUInitState : public DefaultActiveIMUInitializerState
{
public:
    InitializedFromCoarseIMUInitState(IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel);

    std::pair<std::unique_ptr<IMUInitializerState>, bool> initializeIfReady() override;

    void print(std::ostream& str) const override;
};

// State where the marginalization factor is replaced if the (scale) diff is larger than a threshold.
class PotentialMarginalizationReplacementState : public DefaultActiveIMUInitializerState
{
public:
    PotentialMarginalizationReplacementState(IMUInitializerLogic& imuInitLogic, StateTransitionModel& transitionModel,
                                             int startIdForSecondTh);

    std::unique_ptr<IMUInitializerState>
    postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor, const gtsam::Values& baValues,
               double timestamp, const gtsam::PreintegratedImuMeasurements& imuMeasurements) override;

    void print(std::ostream& str) const override;

protected:
    int startIdForSecondTH;
    bool useSecondTH = false;
};

// Replaces the marginalization prior of the main graph, but doesn't call callOnInit as the values shall
// not be replaced (in contrast to a normal init).
class MarginalizationReplacementReadyState : public DefaultActiveIMUInitializerState
{
public:
    MarginalizationReplacementReadyState(IMUInitializerLogic& imuInitLogic,
                                         StateTransitionModel& transitionModel,
                                         std::unique_ptr<gtsam::Values>&& optimizedValues);

    std::pair<std::unique_ptr<IMUInitializerState>, bool> initializeIfReady() override;

    void print(std::ostream& str) const override;

private:
    std::unique_ptr<gtsam::Values> optimizedValues;
};


}

#endif //DMVIO_IMUINITIALIZERSTATES_H
