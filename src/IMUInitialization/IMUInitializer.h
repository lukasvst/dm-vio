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

#ifndef DMVIO_IMUINITIALIZER_H
#define DMVIO_IMUINITIALIZER_H


#include "GTSAMIntegration/PoseTransformationIMU.h"
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/ImuFactor.h>
#include "FullSystem/HessianBlocks.h"
#include "IMU/IMUTypes.h"
#include "CoarseIMUInitOptimizer.h"
#include "PoseGraphBundleAdjustment.h"
#include "IMU/BAIMULogic.h"
#include "GTSAMIntegration/DelayedMarginalization.h"
#include <shared_mutex>
#include "IMUInitStateChanger.h"

namespace dmvio
{

class IMUInitializerLogic;
class IMUInitializerState;
class StateTransitionModel;

// IMU Initializer as described in the DM-VIO paper.
// Implementation is performed with a state machine.
// This is more of a facade class which forwards most of the actual logic to:
// - IMUInitializerStates define the states of the state machine.
// - IMUInitializerTransitions define when to transition between which states (there are multiple transition models e.g. for ablation studies).
// - IMUInitializerLogic contains the main member variables and logic for calling the correct optimizers.
// - The actual optimizations are performed by CoarseIMUInitOptimizer and PoseGraphBundleAdjustment (PGBA)
class IMUInitializer : public IMUInitStateChanger
{
public:
    typedef std::function<void(const gtsam::Values& values, bool willReplaceGraph)> InitCallback;

    // Note that a reference to the settings and imuCalibration, and also delayedMarginalization is kept!
    IMUInitializer(std::string resultsPrefix, boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams,
                   const IMUCalibration& imuCalibration, IMUInitSettings& settings,
                   DelayedMarginalizationGraphs* delayedMarginalization, bool linearizeOperation,
                   InitCallback callOnInit);

    ~IMUInitializer();

    // --------------------------------------------------
    // Called from coarse tracking thread.
    // --------------------------------------------------
    // Called when IMU data for a frame is available.
    void addIMUData(const IMUData& data, int frameId);
    // Called when coarse tracking has finished.
    // --> forwards to CoarseIMUInitOptimizer, can also filter out non-KFs.
    void addPose(const dso::FrameShell& shell, bool willBecomeKeyframe);


    // --------------------------------------------------
    // Called from BA thread.
    // --------------------------------------------------
    // Called after all keyframe operations are finished. This is when the IMU in the main system  should be
    // initialized (if the initialized values are ready).
    bool initializeIfReady();

    // Called after the main BA.
    // here the PGBA can be run.
    void postBAInit(int keyframeId, gtsam::NonlinearFactor::shared_ptr activeHBFactor,
                    const gtsam::Values& baValues, double timestamp,
                    const gtsam::PreintegratedImuMeasurements& imuMeasurements);


    const gtsam::imuBias::ConstantBias& getLatestBias() const;

    // --------------------------------------------------
    // Methods overridden from IMUInitStateChanger. These allow states and transitions to set the current state.
    // --------------------------------------------------
    // Can directly be called to change the state.
    void lockAndSetState(std::unique_ptr<IMUInitializerState>&& newState) final;

    // Aquire lock for calling setState.
    std::unique_lock<std::shared_timed_mutex> acquireSetStateLock() final;
    // Before calling setState a lock must be acquired with the previous method!
    void setState(std::unique_ptr<IMUInitializerState>&& newState) final;

private:
    // Variables for the state machine.
    std::unique_ptr<IMUInitializerLogic> logic; // contains main initialization data and logic common for all states.
    std::unique_ptr<IMUInitializerState> currentState;
    std::unique_ptr<StateTransitionModel> transitionModel; // contains the logic which states transition to which new states.

    // requires a shared_lock for calls to the currentState, and a unique_lock to change the state.
    std::shared_timed_mutex mutex;

    // Used to couple the IMU data with the corresponding frame pose.
    std::unique_ptr<IMUData> currentCoarseIMUData;
    int currentCoarseFrameId = -1;
};

// Returns true if if gravity direction and scale have changed more than a threshold (according to the settings).
bool thresholdVariableChanges(dmvio::IMUThresholdSettings settings, const gtsam::Values& baValues,
                              const gtsam::Values& fejValues);


}
#endif //DMVIO_IMUINITIALIZER_H
