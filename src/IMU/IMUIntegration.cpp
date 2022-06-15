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

#include "IMUIntegration.hpp"

#include "OptimizationBackend/EnergyFunctionalStructs.h"

#include "FullSystem/HessianBlocks.h"
#include "dso/util/FrameShell.h"


#include <util/TimeMeasurement.h>
#include <GTSAMIntegration/Marginalization.h>
#include "GTSAMIntegration/ExtUtils.h"
#include "IMUUtils.h"
#include "GTSAMIntegration/DelayedMarginalization.h"

using namespace dmvio;
using std::cout;
using std::endl;

IMUIntegration::IMUIntegration(dso::CalibHessian* HCalib, const IMUCalibration& imuCalibrationPassed,
                               IMUSettings& imuSettingsPassed, bool linearizeOperationPassed)
        : linearizeOperation(linearizeOperationPassed), preparedKeyframe(-1), preparedKFCreated(false),
          imuCalibration(imuCalibrationPassed), imuSettings(imuSettingsPassed)
{
    // Create preintegrationParams
    double accelVar = imuCalibration.accel_sigma * imuCalibration.accel_sigma;
    double gyroVar = imuCalibration.gyro_sigma * imuCalibration.gyro_sigma;
    double integrationVar = imuCalibration.integration_sigma * imuCalibration.integration_sigma;

    // --------------------------------------------------
    preintegrationParams.reset(new gtsam::PreintegrationParams(imuCalibrationPassed.gravity));
    preintegrationParams->setIntegrationCovariance(integrationVar * Eigen::Matrix3d::Identity());
    preintegrationParams->setAccelerometerCovariance(accelVar * Eigen::Matrix3d::Identity());
    preintegrationParams->setGyroscopeCovariance(gyroVar * Eigen::Matrix3d::Identity());


    // Create Delayed Marginalization Graphs.
    std::unique_ptr<BAGraphs> baGraphs;
    DelayedMarginalizationGraphs* delayedGraphs = new DelayedMarginalizationGraphs(0, BAIMULogic::METRIC_GROUP);
    baGraphs.reset(delayedGraphs);

    // Create BAGTSAMIntegration.
    // Pass empty transformation, because DSO and baGraph have the same coordinate system (except for the side of epsilon).
    std::unique_ptr<TransformIdentity> transformationDSOToBA(new TransformIdentity());
    GTSAMIntegrationSettings baGTSAMSettings;
    baGTSAMSettings.weightDSOToGTSAM = imuSettings.setting_weightDSOToGTSAM;
    baGTSAMIntegration.reset(
            new BAGTSAMIntegration(std::move(baGraphs), std::move(transformationDSOToBA), baGTSAMSettings, HCalib));

    // Create classes handling the IMUIntegration in BA and Coarse tracking respectively.
    baLogic.reset(new BAIMULogic(this, baGTSAMIntegration.get(), imuCalibration, imuSettings));
    std::unique_ptr<PoseTransformation> coarsePoseTransformation = baLogic->getTransformDSOToIMU()->clone();
    coarseLogic.reset(
            new CoarseIMULogic(std::move(coarsePoseTransformation), preintegrationParams, imuCalibration, imuSettings));

    if(!imuSettings.initSettings.disableVIOUntilFirstInit) // Only add the extension right away if we start with VIO immediately.
    {
        baGTSAMIntegration->addExtension(baLogic);
        coarseInitialized = true;
        baInitialized = true;
    }

    // IMUInitializer:
    imuInitializer.reset(new IMUInitializer(imuSettings.resultsPrefix, preintegrationParams, imuCalibration,
                                            imuSettings.initSettings, delayedGraphs, linearizeOperation,
                                            [this](const gtsam::Values& values, bool willReplaceGraph)
                                            {
                                                // Callback called upon IMU initialization.
                                                bool reinit = baInitialized;
                                                if(!reinit)
                                                {
                                                    baGTSAMIntegration->addExtension(baLogic);
                                                }
                                                baLogic->initFromIMUInit(values, reinit, willReplaceGraph);
                                                baInitialized = true; // Note: not threadsafe for RT yet if we
                                                // initialize from CoarseIMUInit (which we do not do in the normal
                                                // transition mode).
                                            }));

    // --------------------------------------------------
    TS_cam_imu = imuCalibration.T_cam_imu;

    preintegratedBA.reset(new gtsam::PreintegratedImuMeasurements(preintegrationParams));
    preintegratedBACurr.reset(new gtsam::PreintegratedImuMeasurements(preintegrationParams));
}

// return lastKeyframe to newKeyframe.
Sophus::SE3d IMUIntegration::initCoarseGraph()
{
    if(!dso::setting_debugout_runquiet)
    {
        std::cout << "Prepared keyframe id: " << preparedKeyframe << std::endl;
    }
    int keyframeId = preparedKeyframe;
    preparedKeyframe = -1;

    if(!informationBAToCoarse)
    {
        return Sophus::SE3d{};
    }
    coarseInitialized = true;
    return coarseLogic->initCoarseGraph(keyframeId, std::move(informationBAToCoarse));
}

// updateBAValues should be called before this...
void IMUIntegration::finishKeyframeOperations(int keyframeId)
{
    if(baInitialized)
    {
        dmvio::TimeMeasurement timeMeasurement("IMUIntegration::finishKeyframeOperations");

        // Forward to baLogic which does the work.
        baLogic->finishKeyframeOperations(keyframeId);
    }
    if(imuInitializer)
    {
        imuInitializer->initializeIfReady();
    }
}

void IMUIntegration::addIMUDataToBA(const IMUData& imuData)
{
    dmvio::TimeMeasurement timeMeasurement("addIMUDataToBA");
    integrateIMUData(imuData, *preintegratedBACurr);
    lastIMUData = imuData;
}

// returns estimated referenceToFrame.
Sophus::SE3 IMUIntegration::addIMUData(const IMUData& imuData, int frameId, double frameTimestamp,
                                       bool firstFrameAfterKFChange,
                                       int lastFrameId, bool onlyForHint)
{
    boost::shared_ptr<gtsam::PreintegratedImuMeasurements> additionalMeasurements;
    if(firstFrameAfterKFChange)
    {
        additionalMeasurements = preintegratedForNextCoarse;
        preintegratedForNextCoarse.reset();
    }

    if(preparedKeyframe != -1 && !onlyForHint)
    {
        // Currently a new bundle adjustment is in progress -> Also put the imu data into the preintegration for the next coarse tracking.
        if(imuData.size() > 0)
        {
            integrateIMUData(imuData, *preintegratedForNextCoarse);
            imuDataPreintegrated = true;
        }
    }

    if(imuInitializer)
    {
        imuInitializer->addIMUData(imuData, frameId);
    }

    if(!isCoarseInitialized()) return Sophus::SE3d{};

    return coarseLogic->addIMUData(imuData, frameId, frameTimestamp, lastFrameId, additionalMeasurements,
                                   preparedKeyframe);
}


void IMUIntegration::updateCoarsePose(const Sophus::SE3& refToFrame)
{
    if(!coarseInitialized) return;
    coarseLogic->updateCoarsePose(refToFrame);
}

Sophus::SE3
IMUIntegration::computeCoarseUpdate(const dso::Mat88& H_in, const dso::Vec8& b_in, float extrapFac, float lambda,
                                    double& incA, double& incB, double& incNorm)
{

    assert(isCoarseInitialized()); // Caller is responsible for not calling if not initialized.
    Sophus::SE3d newReferenceToFrame = coarseLogic->computeCoarseUpdate(H_in, b_in, extrapFac, lambda, incA, incB,
                                                                        incNorm);

    return newReferenceToFrame;
}

void IMUIntegration::acceptCoarseUpdate()
{
    if(!isCoarseInitialized()) return;
    coarseLogic->acceptCoarseUpdate();
}

void IMUIntegration::addVisualToCoarseGraph(const dso::Mat88& H, const dso::Vec8& b, bool trackingIsGood)
{
    if(!isCoarseInitialized()) return;
    coarseLogic->addVisualToCoarseGraph(H, b, trackingIsGood);
}

void IMUIntegration::setGTData(dmvio::GTData* gtData, int frameId)
{
    dmvio::TimeMeasurement timeMeasurement("printBiases");

    // This must be done only for linearizeOperation! (because otherwise it's not the correct gtData for the KF + it might be deleted...)
    baLogic->setCurrGtData(gtData, frameId);

    // Note: At the moment coarse biases are only printed for frames that will become a KF (because otherwise this method is not called).
    coarseLogic->printCoarseBiases(gtData, frameId);
}

IMUIntegration::~IMUIntegration() = default;

// Called in the coarse tracking thread.
// This contains code relevant for both, CoarseTracking and BA. Mainly to make it work in realtime mode.
void IMUIntegration::prepareKeyframe(int frameId)
{
    bool previouslyPrepared = false; // The last frame was already prepared to be a KF.

    // Make sure that the previous keyframe was finished!
    if(preparedKeyframe != -1 && !linearizeOperation)
    {
        if(!dso::setting_debugout_runquiet)
        {
            std::cout << "Note: there is already a keyframe prepared! " << preparedKeyframe << std::endl;
        }
        assert(frameId == preparedKeyframe + 1);
        assert(!preparedKFCreated);

        previouslyPrepared = true;
    }
    assert(!linearizeOperation || preparedKeyframe == -1);

    preparedKeyframe = frameId;
    preparedKFCreated = false;

    gtsam::imuBias::ConstantBias currentBias = coarseLogic->getBias(frameId);
    preparedCoarseVel = coarseLogic->getVelocity(
            frameId); // We can call this without mutex because we are in the coarse tracking thread.

    preintegratedForNextCoarse.reset(new gtsam::PreintegratedImuMeasurements(preintegrationParams, currentBias));
    imuDataPreintegrated = false;

    // We solve that sometimes a new keyframe shall be prepared even though the old one hasn't finished BA yet.
    // There are two ways this can happen:
    // 1. The old keyframe is currently in BA and the system already needs a new KF.
    // 2. The system has just prepared the previous preparedKeyframe, but hasn't started BA yet. In this case, normal DSO
    // would optimize the new frame and the old preparedKeyframe would not become a KF.

    if(previouslyPrepared)
    {
        // If the last KF was also prepared the two buffers were already swapped. Then the latest IMU-data has to be added to the current buffer.
        // In the multithreaded case we need the preintegratedBACurr
        integrateIMUData(lastIMUData, *preintegratedBA);
    }else
    {
        boost::shared_ptr<gtsam::PreintegratedImuMeasurements> swap = preintegratedBA;
        preintegratedBA = preintegratedBACurr;
        preintegratedBACurr = swap;
    }
    // Take latest bias from BA. Maybe this should actually locked. Currently however the BA must be finished
    // already, as else the assertion would have failed. Therefore the latestBias cannot be changed right now.
    preintegratedBACurr->resetIntegrationAndSetBias(latestBias);
}

const gtsam::PreintegratedImuMeasurements& IMUIntegration::getPreintegratedMeasurements(int keyframeId)
{
    assert(linearizeOperation || keyframeId == preparedKeyframe);

    return *preintegratedBA;
}

// called right before finishKeyframeOptimization, but outside the mutex.
void IMUIntegration::postOptimization(int keyframeId)
{
    initializedBeforePostOptimization = baInitialized;
    if(initializedBeforePostOptimization)
    {
        baLogic->postOptimization(keyframeId);
    }
    if(imuInitializer)
    {
        imuInitializer->postBAInit(keyframeId, baGTSAMIntegration->getActiveDSOFactor(),
                                   *(baGTSAMIntegration->getBaValues()), baGTSAMIntegration->getCurrBaTimestamp(),
                                   getPreintegratedMeasurements(keyframeId));
    }
}

// This method gets the latest BA values for the coarse tracker (for the next graph).
bool IMUIntegration::finishKeyframeOptimization(int keyframeId)
{
    if(!initializedBeforePostOptimization)
    {
        // Get latestBias from initializer until initialized
        latestBias = imuInitializer->getLatestBias();
        return false;
    }
    informationBAToCoarse = baLogic->finishKeyframeOptimization(keyframeId);
    latestBias = informationBAToCoarse->latestBABias;
    return true;
}

bool IMUIntegration::newTrackingRefNeedsHandling()
{
    return imuDataPreintegrated;
}

int IMUIntegration::getPreparedKeyframe() const
{
    return preparedKeyframe;
}

void IMUIntegration::skipPreparedKeyframe()
{
    preparedKeyframe = -1;
}

void IMUIntegration::keyframeCreated(int frameId)
{
    assert(frameId == preparedKeyframe);
    preparedKFCreated = true;
    // This is locked with mutex, meaning that there cannot be a BA, or a call to prepareKeyframe at the same time.
    baLogic->setNextBAVel(preparedCoarseVel, preparedKeyframe);
}

bool IMUIntegration::isPreparedKFCreated() const
{
    return preparedKFCreated;
}

Sophus::SE3d IMUIntegration::getCoarseKFPose()
{
    if(!isCoarseInitialized()) return Sophus::SE3d{};
    Sophus::SE3d pose = coarseLogic->getCoarseKFPose();
    return pose;
}

IMUSettings& IMUIntegration::getImuSettings() const
{
    return imuSettings;
}

const std::unique_ptr<BAGTSAMIntegration>& IMUIntegration::getBAGTSAMIntegration() const
{
    return baGTSAMIntegration;
}

TransformDSOToIMU& IMUIntegration::getTransformDSOToIMU()
{
    return *(baLogic->getTransformDSOToIMU());
}

void IMUIntegration::newFrameEnergyTH(float& energyThreshold)
{
    lastDSOEnergyTH = energyThreshold;
    float th = imuSettings.maxFrameEnergyThreshold;
    if(th > 0.0 && energyThreshold > th)
    {
        energyThreshold = th;
    }
}

void IMUIntegration::finishCoarseTracking(const dso::FrameShell& frameShell, bool willBecomeKeyframe)
{
    if(imuInitializer)
    {
        imuInitializer->addPose(frameShell, willBecomeKeyframe);
    }
}

bool IMUIntegration::isCoarseInitialized()
{
    return coarseInitialized;
}

void IMUIntegration::resetBAPreintegration()
{
    preintegratedBACurr->resetIntegration();
}

double IMUIntegration::getCoarseScale()
{
    return coarseLogic->getScale();
}