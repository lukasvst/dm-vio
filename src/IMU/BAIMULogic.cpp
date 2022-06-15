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

#include <gtsam/inference/Symbol.h>
#include <GTSAMIntegration/Sim3GTSAM.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <iomanip>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <GTSAMIntegration/Marginalization.h>
#include "BAIMULogic.h"
#include "dso/util/FrameShell.h"
#include "GTSAMIntegration/ExtUtils.h"
#include "IMUInitialization/GravityInitializer.h"
#include "dso/util/settings.h"
#include "IMUUtils.h"
#include <memory>
#include <numeric>
#include <GTSAMIntegration/GTSAMUtils.h>
#include <util/TimeMeasurement.h>
#include "GTSAMIntegration/FEJNoiseModelFactor.h"

using namespace dmvio;
using gtsam::Symbol;
using gtsam::symbol_shorthand::S, gtsam::symbol_shorthand::G;

constexpr char end = '\n';
// Version for flushing always (slower, but useful for crashes).
//template<class CharT, class Traits>
//auto end(std::basic_ostream<CharT, Traits>& os) -> decltype(std::endl(os))&
//{
//    return std::endl(os);
//}

BAIMULogic::BAIMULogic(PreintegrationProviderBA* preintegrationProvider, BAGTSAMIntegration* baIntegration,
                       const IMUCalibration& imuCalibration,
                       IMUSettings& imuSettings)
        : preintegrationProvider(preintegrationProvider), baIntegration(baIntegration), imuSettings(imuSettings),
          imuCalibration(imuCalibration), scaleQueue(imuSettings.generalScaleIntervalSize),
          optimizeScalePtr(new bool()), optimizeGravityPtr(new bool()), optimizedIMUExtrinsicsPtr(new bool()),
          optimizeScale(*optimizeScalePtr), optimizeGravity(*optimizeGravityPtr),
          optimizeIMUExtrinsics(*optimizedIMUExtrinsicsPtr)
{
    if(!imuSettings.initSettings.disableVIOUntilFirstInit)
    {
        optimizeScale = imuSettings.setting_optScaleBA;
        optimizeGravity = imuSettings.setting_optGravity;
        optimizeIMUExtrinsics = imuSettings.setting_optIMUExtrinsics;
        optimizeTransform = optimizeScale || optimizeGravity || optimizeIMUExtrinsics;
    }else
    {
        optimizeScale = optimizeGravity = optimizeIMUExtrinsics = false;
        optimizeTransform = true;
    }

    // Create PoseTransformation
    transformDSOToIMU.reset(new TransformDSOToIMU(gtsam::Pose3(imuCalibration.T_cam_imu.matrix()), optimizeScalePtr,
                                                  optimizeGravityPtr, optimizedIMUExtrinsicsPtr,
                                                  imuSettings.gravityDirectionFixZ));

    baIntegration->setDynamicDSOWeightCallback([this](double lastDSOEnergy, double lastRMSE, bool coarseTrackingWasGood)
                                               {
                                                   return computeDynamicDSOWeight(lastDSOEnergy, lastRMSE,
                                                                                  coarseTrackingWasGood);
                                               });

    scaleFile.open(imuSettings.resultsPrefix + "scalesdso.txt"); // contains the current scale for all times.
    baBiasFile.open(imuSettings.resultsPrefix + "babiasdso.txt"); // contains estimated and groundtruth bias.
    baGravDirFile.open(imuSettings.resultsPrefix + "bagravdir.txt"); // contains gravity directions.
    baVelFile.open(imuSettings.resultsPrefix + "bavel.txt"); // contains estimated velocities.
}

bool BAIMULogic::addIMUVarsForKey(int keyframeId)
{
    bool addIMUKeys = !imuSettings.skipFirstKeyframe || keyframeId != firstFrameId;
    if(disableFromKF > 0 && keyframeId > disableFromKF)
    {
        addIMUKeys = false;
    }
    if(noIMUInOrderingUntilKFId >= 0 && keyframeId < noIMUInOrderingUntilKFId)
    {
        addIMUKeys = false;
    }
    return addIMUKeys;
}

void
dmvio::BAIMULogic::updateBAOrdering(std::vector<dso::EFFrame*>& frames, gtsam::Ordering* ordering, KeyDimMap& baDimMap)
{
    // Add IMU related variables to the BAOrdering.

    // First the variables from the transform.
    int index = transformDSOToIMU->getSymbolInd(); // == 0 usually.
    bool addScaleKey = optimizeScale && !scaleFixed;
    // To fix the scale we simply don't add it to the ordering.  The BAGTSAMIntegration will fix all keys which are
    // not in the ordering.
    if(addScaleKey)
    {
        gtsam::Symbol scaleKey('s', index);
        ordering->push_back(scaleKey);
        baDimMap[scaleKey] = 1;
    }
    if(optimizeGravity)
    {
        gtsam::Symbol gravityKey('g', index);
        ordering->push_back(gravityKey);
        baDimMap[gravityKey] = 3;
    }
    if(imuSettings.setting_optIMUExtrinsics && optimizeIMUExtrinsics)
    {
        gtsam::Symbol extrinsicsKey('i', index);
        ordering->push_back(extrinsicsKey);
        baDimMap[extrinsicsKey] = 6;
    }

    // Add velocities and biases.
    int i = 0;
    for(dso::EFFrame* h : frames)
    {
        int id = h->idx;

        assert(id == i);

        long fullId = h->data->shell->id;

        gtsam::Symbol velKey('v', fullId);
        gtsam::Symbol biasKey('b', fullId);

        if(addIMUVarsForKey(fullId))
        {
            ordering->push_back(velKey);
            ordering->push_back(biasKey);
        }

        baDimMap[velKey] = 3;
        baDimMap[biasKey] = 6;

        ++i;
    }
}

void dmvio::BAIMULogic::addKeysToMarginalize(int fullId, gtsam::FastVector<gtsam::Key>& keysToMarginalize)
{
    gtsam::Symbol velKey('v', fullId);
    gtsam::Symbol biasKey('b', fullId);

    if(addIMUVarsForKey(fullId))
    {
        keysToMarginalize.push_back(velKey);
        keysToMarginalize.push_back(biasKey);
    }
}

// This method is only relevant if the legacy setting disableVIOUntilFirstInit is set to false.
// Called from addKeyframe, and from initializeFromInitializer!
// It is first called from initializeFromInitializer, but then only the first 5 lines of the method are executed.
// Then it will be called from the first calling of addKeyframe.
void dmvio::BAIMULogic::addFirstBAFrame(int keyframeId, BAGraphs* baGraphs, gtsam::Values::shared_ptr baValues)
{
    if(imuSettings.skipFirstKeyframe && firstFrameId == -1)
    {
        firstFrameId = keyframeId;
        previousKeyframeId = -1;
        return;
    }
    gtsam::Key vel_current_key = gtsam::Symbol('v', keyframeId);
    gtsam::Key bias_current_key = gtsam::Symbol('b', keyframeId);

    std::cout << "Add first keyframe: Add priors for keyframe " << keyframeId << std::endl;
    gtsam::Vector3 initialVelocity = (gtsam::Vector(3) << 0, 0, 0).finished();
    gtsam::imuBias::ConstantBias initialBias;

    baValues->insert(vel_current_key, initialVelocity);
    baValues->insert(bias_current_key, initialBias);

    if(imuSettings.setting_prior_bias)
    {
        gtsam::noiseModel::Diagonal::shared_ptr bias_prior_model = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3).finished());
        baGraphs->addFactor(
                boost::make_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(bias_current_key, initialBias,
                                                                                     bias_prior_model),
                BIAS_AND_PRIOR_GROUP);
    }

    if(imuSettings.setting_prior_velocity)
    {
        gtsam::noiseModel::Diagonal::shared_ptr velocity_prior_model = gtsam::noiseModel::Isotropic::Sigma(3, 1e-1);
        baGraphs->addFactor(boost::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(vel_current_key, initialVelocity,
                                                                                   velocity_prior_model),
                            BIAS_AND_PRIOR_GROUP);
    }

    // Add priors for gravity direction and extrinsics + insert them and the scale into values.
    auto factors = getPriorsAndAddValuesForTransform(*transformDSOToIMU, imuSettings.transformPriors, *baValues);
    for(auto&& factor : factors)
    {
        baGraphs->addFactor(factor, BIAS_AND_PRIOR_GROUP);
    }

    previousKeyframeId = keyframeId; // Set so that the next call of addKeyframeToBA knows about this first frame.

    maxScaleInterval = 0.0;
    minScaleInterval = 1000.0;

}

void dmvio::BAIMULogic::setNextBAVel(const gtsam::Vector3& velocity, int frameId)
{
    nextVelocity = velocity;
    nextVelocityFrameId = frameId;
}

void dmvio::BAIMULogic::addKeyframe(BAGraphs* baGraphs, gtsam::Values::shared_ptr baValues, int keyframeId,
                                    const Sophus::SE3& keyframePose, std::vector<dso::EFFrame*>& frames)
{
    if(disableFromKF > 0 && keyframeId > disableFromKF)
    {
        return;
    }

    double currBATimestamp = baIntegration->getCurrBaTimestamp();
    if(firstBATimestamp < 0)
    {
        firstBATimestamp = currBATimestamp;
    }

    int lastKeyframeId = previousKeyframeId;
    currKeyframeId = keyframeId;

    if(lastKeyframeId == -1 || lastKeyframeId == keyframeId)
    {
        assert(imuSettings.skipFirstKeyframe); // should not happen unless we want to skip the first keyframe!
        std::cout << "First keyframe was successfully skipped." << std::endl;
        std::cout << lastKeyframeId << std::endl;
        addFirstBAFrame(keyframeId, baGraphs, baValues);
    }else
    {
        // Insert IMU factor.

        gtsam::Key currPoseKey = gtsam::Symbol('p', keyframeId);
        gtsam::Key currVelKey = gtsam::Symbol('v', keyframeId);
        gtsam::Key currBiasKey = gtsam::Symbol('b', keyframeId);

        gtsam::Key prevPoseKey = gtsam::Symbol('p', lastKeyframeId);
        gtsam::Key prevVelKey = gtsam::Symbol('v', lastKeyframeId);
        gtsam::Key prevBiasKey = gtsam::Symbol('b', lastKeyframeId);

        auto&& imuMeasurements = preintegrationProvider->getPreintegratedMeasurements(keyframeId);

        // Alternative: Example how the GTSAM IMU Factor can be modified to handle First Estimates Jacobians in the
        // correct way. We disable it because it does not seem to improve the results, as it is sufficient to handle
        // FEJ in the PoseTransformationFactor only).
        // The same technique with our FEJNoiseModelFator can also be used for the bias random walk factors.
//        gtsam::NonlinearFactor::shared_ptr imuFactorWithFEJ(
//                new FEJNoiseModelFactor<gtsam::ImuFactor>(pose_prev_key, vel_prev_key,
//                                                          pose_current_key, vel_current_key, bias_prev_key,
//                                                          imuMeasurements));

        gtsam::NonlinearFactor::shared_ptr imuFactor(
                new gtsam::ImuFactor(prevPoseKey, prevVelKey,
                                     currPoseKey, currVelKey, prevBiasKey,
                                     imuMeasurements));

        // Note: The graph optimizes poses with worldToCam in DSO scale. The IMU factor works on metric poses with imuToWorld.
        // To transform between the two we use our PoseTransformationFactor and the transformDSOToIMU.
        // This will also add the scale and gravity direction as optimizable variables to our factor graph.
        auto transformedFactor = boost::make_shared<PoseTransformationFactor>(imuFactor,
                                                                              *transformDSOToIMU,
                                                                              PoseTransformationFactor::JACOBIAN_FACTOR);
        baGraphs->addFactor(transformedFactor, METRIC_GROUP);

        gtsam::noiseModel::Diagonal::shared_ptr biasNoiseModel = computeBiasNoiseModel(imuCalibration, imuMeasurements);

        gtsam::NonlinearFactor::shared_ptr biasFactor(
                new gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                        prevBiasKey, currBiasKey,
                        gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(),
                                                     gtsam::Vector3::Zero()), biasNoiseModel));

        baGraphs->addFactor(biasFactor, BIAS_AND_PRIOR_GROUP);

        // Insert previous bias as initial new bias.
        gtsam::imuBias::ConstantBias currentBias = baValues->at<gtsam::imuBias::ConstantBias>(prevBiasKey);
        baValues->insert(currBiasKey, currentBias);

        gtsam::Vector3 currentVelocity = baValues->at<gtsam::Vector3>(prevVelKey);
        // If available use velocity from coarse tracking.
        if(nextVelocityFrameId != -1)
        {
            if(nextVelocityFrameId == keyframeId)
            {
                currentVelocity = nextVelocity;
                nextVelocityFrameId = -1;
            }else
            {
                std::cout << "ERROR: nextVelocity frame not as expected! " << nextVelocityFrameId << " " << nextVelocity
                          << std::endl;
            }
        }
        baValues->insert(currVelKey, currentVelocity);
    }
}

void dmvio::BAIMULogic::preSolve(gtsam::Matrix& HFull, gtsam::Vector& bFull, int dimensionDSOH)
{
    if(disableFromKF > 0) return;

    if(imuSettings.useScaleDiagonalHack && optimizeScale && !scaleFixed)
    {
        // Hack which is useful in case we initialize immediately without a scale prior. This stops the scale from converging
        // even if it is far from the optimum.
        // Usually disabled because with the IMU initializer of DM-VIO we don't need it.
        int scalePos = dimensionDSOH;
        HFull(scalePos, scalePos) += 5.0;
        HFull(scalePos, scalePos) *= (1 + 1.0);
    }
}

bool dmvio::BAIMULogic::postSolve(gtsam::Values::shared_ptr values, gtsam::Values::shared_ptr newValues,
                                  const gtsam::Vector& inc, const gtsam::Ordering& ordering, const KeyDimMap& baDimMap)
{
    if(disableFromKF > 0) return true;

    bool dontBreak = false;

    if(imuSettings.alwaysCanBreakIMU)
    {
        return true;
    }
    // Compute mean squared increment norm for velocities, biases, scale, etc.
    std::map<char, MeanAccumulatorD> accums;
    accums['v'] = MeanAccumulatorD{};
    accums['b'] = MeanAccumulatorD{};
    accums['s'] = MeanAccumulatorD{};
    accums['g'] = MeanAccumulatorD{};
    accums['i'] = MeanAccumulatorD{};
    // Thresholds analogous to computation in FullSystem::doStepFromBackup.
    std::map<char, double> thresholds = {{'v', 0.0001},
                                         {'b', 0.0005},
                                         {'s', 0.0005},
                                         {'g', 0.0005},
                                         {'i', 0.0005}};
    int pos = 0;
    for(auto&& key : ordering)
    {
        Symbol s(key);
        auto&& accum = accums.find(s.chr());
        if(accum != accums.end())
        {
            gtsam::Vector increment = inc.segment(pos, baDimMap.at(key));
            accum->second.add(increment.squaredNorm());
        }
        pos += baDimMap.at(key);
    }
    bool canBreak = true;
    for(auto&& pair : accums)
    {
        double val = std::sqrt(pair.second.getMean());
        double thresh = thresholds[pair.first] * dso::setting_thOptIterations;
        canBreak = canBreak && val < thresh;
    }
    return canBreak && !dontBreak;
}

void dmvio::BAIMULogic::acceptUpdate(gtsam::Values::shared_ptr values, gtsam::Values::shared_ptr newValues)
{
    if(optimizeTransform)
    {
        // Update scale and gravity direction in the transform.
        transformDSOToIMU->updateWithValues(*newValues);
    }

    gtsam::Key scaleKey = gtsam::Symbol('s', 0);
    if(optimizeScale && !scaleFixed)
    {
        double newScale = transformDSOToIMU->getScale();
        if(!dso::setting_debugout_runquiet)
        {
            std::cout << "Optimized scale: " << newScale << end;
        }
        if(newScale > maxScaleInterval)
        {
            maxScaleInterval = newScale;
        }
        if(newScale < minScaleInterval)
        {
            minScaleInterval = newScale;
        }
    }
    if(optimizeIMUExtrinsics)
    {
        gtsam::Pose3 newT_cam_imu = newValues->at<gtsam::Pose3>(Symbol('i', 0));
        std::cout << "Optimized T_cam_imu: " << newT_cam_imu.translation().transpose() << end;
    }
}

// Signals that the BA and marginalization operations for this keyframe are finished (might be after the KF is already the new tracking ref)
void dmvio::BAIMULogic::finishKeyframeOperations(int keyframeId)
{
    if(disableFromKF > 0) return;

    auto&& baValues = baIntegration->getBaValues();
    gtsam::imuBias::ConstantBias currentBias = baValues->at<gtsam::imuBias::ConstantBias>(
            gtsam::Symbol('b', keyframeId));

    double currBATimestamp = baIntegration->getCurrBaTimestamp();

    if(currGTData)
    {
        // Print estimated bias and corresponding GT bias to file.
        Eigen::Vector3d gtTrans = currGTData->biasTranslation;
        Eigen::Vector3d gtRot = currGTData->biasRotation;
        Eigen::Vector3d errorTrans = currentBias.accelerometer() - gtTrans;
        Eigen::Vector3d errorRot = currentBias.gyroscope() - gtRot;
        currGTData = 0; // Note that this will not work in realtime mode...

        baBiasFile << std::fixed << std::setprecision(6) << currBATimestamp << ' ' << std::setprecision(20)
                   << gtTrans.transpose() << ' ' << gtRot.transpose() << ' ' << currentBias.accelerometer().transpose()
                   << ' ' << currentBias.gyroscope().transpose();
    }else
    {
        Eigen::Vector3d gtTrans = Eigen::Vector3d::Zero();
        Eigen::Vector3d gtRot = Eigen::Vector3d::Zero();
        baBiasFile << std::fixed << std::setprecision(6) << currBATimestamp << ' ' << std::setprecision(20)
                   << gtTrans.transpose() << ' ' << gtRot.transpose() << ' ' << currentBias.accelerometer().transpose()
                   << ' ' << currentBias.gyroscope().transpose();
    }
    if(biasCovForKF == keyframeId)
    {
        // We also have computed bias covariance.
        baBiasFile << ' ' << biasCovariance.diagonal().transpose();
    }
    baBiasFile << end;

    if(optimizeScale && !scaleFixed)
    {
        // Compute how much the scale has changed lately to determine if it should be fixed.
        scaleQueue.push_back(std::make_pair(maxScaleInterval, minScaleInterval));
        scaleQueue.pop_front();
        maxScaleInterval = minScaleInterval = transformDSOToIMU->getScale();
        double realMaxScale = maxScaleInterval, realMinScale = minScaleInterval;
        // Compute and minimum and maximum scale over the last n keyframes optimizations.
        for(auto&& pair : scaleQueue)
        {
            if(pair.first > realMaxScale)
            {
                realMaxScale = pair.first;
            }
            if(pair.second < realMinScale)
            {
                realMinScale = pair.second;
            }
        }
        double diff = realMaxScale / realMinScale - 1.0;

        if(diff < imuSettings.setting_scaleFixTH)
        {
            // Fix scale!
            scaleFixed = true;
            std::cout << "Fixing scale at time: " << std::fixed << std::setprecision(6) << currBATimestamp << std::endl;

            // Useful for testing. Allows to evaluate the visual-only system, for the majority of the time but with the metric scale obtained from IMU.
            if(imuSettings.setting_visualOnlyAfterScaleFixing == 1)
            {
                std::cout << "DISABLING IMU AND THE GTSAM INTEGRATION COMPLETELY!" << std::endl;
                dso::setting_useIMU = false;
                dso::setting_useGTSAMIntegration = false;
            }else if(imuSettings.setting_visualOnlyAfterScaleFixing == 2)
            {
                std::cout << "DISABLING IMU COMPLETELY!" << std::endl;
                dso::setting_useIMU = false;
                disableFromKF = keyframeId;
            }
        }
    }
}


// computes the factor. Also computes uncertainty for some more variables for saving to file.
gtsam::LinearContainerFactor::shared_ptr BAIMULogic::computeFactorForCoarseGraphAndMarginalCovariances()
{
    dmvio::TimeMeasurement meas("computeFactorForCoarseGraphAndMarginalCovariances");

    // The idea behind this method is that we want both, a factor for the coarse tracking, and marginal uncertainty for some more variables.
    // These need to do similar processing, hence we combine them:
    // First we marginalize everything that is neither in factorOrdering or uncertOrdering, and compute marginal covariances.
    // Then we also marginalize everything in uncertOrdering to obtain the factor for the coarse graph.

    gtsam::Ordering factorOrdering; // This contains all keys which shall be in the factor.

    gtsam::Key biasKey = gtsam::Symbol('b', currKeyframeId);
    gtsam::Key velKey = gtsam::Symbol('v', currKeyframeId);
    factorOrdering.push_back(biasKey);
    factorOrdering.push_back(velKey);

    gtsam::Ordering uncertOrdering; // This contains all keys for which we need uncertainty (but which shall not be in the factor).
    gtsam::Key scaleKey = gtsam::Symbol('s', transformDSOToIMU->getSymbolInd());
    gtsam::Key gravKey = gtsam::Symbol('g', transformDSOToIMU->getSymbolInd());
    gtsam::Key newestPoseKey = gtsam::Symbol('p', currKeyframeId);
    uncertOrdering.push_back(scaleKey);
    uncertOrdering.push_back(gravKey);

    gtsam::Ordering ordering;
    ordering.insert(ordering.end(), uncertOrdering.begin(), uncertOrdering.end());
    ordering.insert(ordering.end(), factorOrdering.begin(), factorOrdering.end());

    auto& keyDimMap = baIntegration->getBaDimMap();
    gtsam::Ordering additionalKeys;

    // We first compute the Hessian H and gradient vector b using the new values.
    auto graphPair = baIntegration->getBaGraphs().getHAndB(*(baIntegration->getBaValues()), ordering,
                                                           keyDimMap, &additionalKeys);

    gtsam::Ordering completeOrdering;
    completeOrdering.insert(completeOrdering.begin(), ordering.begin(), ordering.end());
    completeOrdering.insert(completeOrdering.end(), additionalKeys.begin(), additionalKeys.end());

    // Also add the active DSO hessian.
    auto activeDSOFactor = baIntegration->getActiveDSOFactor();
    gtsam::NonlinearFactorGraph activeDSOGraph;
    activeDSOGraph.push_back(activeDSOFactor);
    auto linearizedDSOGraph = activeDSOGraph.linearize(*(baIntegration->getBaValues()));

    AugmentedScatter scatter(*linearizedDSOGraph, completeOrdering, keyDimMap);
    std::pair<gtsam::Matrix, gtsam::Vector> pair = scatter.computeHessian(*linearizedDSOGraph);

    gtsam::Ordering newAdditionalKeys;
    fillAdditionalKeysFromScatter(completeOrdering, newAdditionalKeys, scatter);
    additionalKeys.insert(additionalKeys.end(), newAdditionalKeys.begin(), newAdditionalKeys.end());

    assert(graphPair.first.size() <= pair.first.size());
    pair.first.block(0, 0, graphPair.first.rows(), graphPair.first.cols()) += graphPair.first;
    pair.second.segment(0, graphPair.second.size()) += graphPair.second;

    auto accumFun = [&keyDimMap](int num, const gtsam::Key& key)
    {
        return num + keyDimMap.at(key);
    };
    int aSize = std::accumulate(ordering.begin(), ordering.end(), 0, accumFun);
    int mSize = std::accumulate(additionalKeys.begin(), additionalKeys.end(), 0, accumFun);

    // Then we first marginalize everything except the necessary variables.

    // Marginalize with preconditioning:
    // First create the proper input (first variables to marginalize, then rest) for computeSchurComplement.
    gtsam::Matrix margInputH(aSize + mSize, aSize + mSize);
    margInputH.block(0, 0, mSize, mSize) = pair.first.block(aSize, aSize, mSize, mSize);
    margInputH.block(mSize, mSize, aSize, aSize) = pair.first.block(0, 0, aSize, aSize);
    margInputH.block(0, mSize, mSize, aSize) = pair.first.block(aSize, 0, mSize, aSize);
    margInputH.block(mSize, 0, aSize, mSize) = pair.first.block(0, aSize, aSize, mSize);
    gtsam::Vector margInputB(aSize + mSize);
    margInputB.segment(0, mSize) = pair.second.segment(aSize, mSize);
    margInputB.segment(mSize, aSize) = pair.second.segment(0, aSize);

    gtsam::Matrix marginalized = computeSchurComplement(
            augmentedHessianFromPair(std::make_pair(margInputH, margInputB)), mSize, aSize);

    // Compute covariances
    gtsam::Matrix margH = marginalized.block(0, 0, aSize, aSize);

    // Covariance is the inverse of the Hessian.
    gtsam::Matrix inversed = margH.inverse();

    std::vector<gtsam::Matrix> uncertainties;
    int pos = 0;
    for(auto&& key : ordering)
    {
        int dim = keyDimMap.at(key);
        uncertainties.emplace_back(inversed.block(pos, pos, dim, dim));
        pos += dim;
    }

    gtsam::Matrix biasCov = uncertainties[uncertOrdering.size()];
#ifdef DEBUG
    gtsam::Matrix testMat = pair.first.inverse().block(4, 4, 6, 6);
    assertEqEigen(testMat, biasCov);
#endif
    biasCovariance = std::move(biasCov);
    biasCovForKF = currKeyframeId;

    // Print covariances to file.
    scaleFile << std::fixed << std::setprecision(6) << baIntegration->getCurrBaTimestamp() << ' '
              << transformDSOToIMU->getScale() << ' ' << uncertainties[0](0, 0) << end;

    Eigen::Vector3d rotLog = transformDSOToIMU->getR_dsoW_metricW().log();
    baGravDirFile << std::fixed << std::setprecision(6) << baIntegration->getCurrBaTimestamp() << ' '
                  << rotLog.transpose() << ' ' << uncertainties[1].diagonal().transpose() << end;

    baVelFile << std::fixed << std::setprecision(6) << baIntegration->getCurrBaTimestamp() << ' ' <<
              baIntegration->getBaValues()->at<gtsam::Vector3>(velKey).transpose() << ' '
              << uncertainties[uncertOrdering.size() + 1].diagonal().transpose() << end;

    // Now marginalize everything in uncertOrdering, because these variables shall not be in the factor for the coarse graph.
    int secondASize = std::accumulate(factorOrdering.begin(), factorOrdering.end(), 0, accumFun);
    int secondMSize = std::accumulate(uncertOrdering.begin(), uncertOrdering.end(), 0, accumFun);
    gtsam::Matrix margForFactor = computeSchurComplement(marginalized, secondMSize,
                                                         secondASize); // This method assumes that the variables to marginalize are first.

    gtsam::FastVector<size_t> connectedDims;
    for(auto&& key : factorOrdering)
    {
        connectedDims.push_back(keyDimMap.at(key));
    }
    gtsam::SymmetricBlockMatrix sm(connectedDims, true);
    sm.setFullMatrix(margForFactor * imuSettings.transferCovToCoarseMultiplier);

    gtsam::LinearContainerFactor::shared_ptr lcf(
            new gtsam::LinearContainerFactor(gtsam::HessianFactor(factorOrdering, sm),
                                             *(baIntegration->getBaValues())));
    return lcf;

}

void BAIMULogic::postOptimization(int keyframeId)
{
    factorForCoarseGraph = computeFactorForCoarseGraphAndMarginalCovariances();
}

std::unique_ptr<InformationBAToCoarse> BAIMULogic::finishKeyframeOptimization(int keyframeId)
{
    // Prepare information for the coarse tracking.
    std::unique_ptr<InformationBAToCoarse> returning(new InformationBAToCoarse);
    returning->latestBAScale = transformDSOToIMU->getScale();
    auto&& baValues = baIntegration->getBaValues();

    returning->latestBAVel = baValues->at<gtsam::Vector3>(gtsam::Symbol('v', keyframeId));
    returning->latestBABias = baValues->at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', keyframeId));
    returning->latestBAPose = baValues->at<gtsam::Pose3>(gtsam::Symbol('p', keyframeId));
    returning->latestBAPosePrevKeyframe = baValues->at<gtsam::Pose3>(gtsam::Symbol('p', previousKeyframeId));

    assert(transformDSOToIMU != nullptr);
    std::shared_ptr<TransformDSOToIMU> copy(new TransformDSOToIMU(*transformDSOToIMU));
    returning->transformBAToIMU = copy;
    returning->transformIMUToDSOForCoarse = std::make_unique<TransformIMUToDSOForCoarse<TransformDSOToIMU>>(copy,
                                                                                                            keyframeId);

    if(imuSettings.setting_transferCovToCoarse)
    {
        returning->priorFactor = factorForCoarseGraph;
    }

    previousKeyframeId = currKeyframeId;

    return returning;
}

void BAIMULogic::setCurrGtData(dmvio::GTData* currGtData, int frameId)
{
    currGTData = currGtData;
    gtFrameId = frameId;
}

std::shared_ptr<TransformDSOToIMU> BAIMULogic::getTransformDSOToIMU() const
{
    return transformDSOToIMU;
}

// Takeover result from the IMU initializer. Values contains the new values (DSO variables will not replaced though).
// If reinit this is a reinitialization, meaning that the IMU is being used already.
// If willReplaceGraph the IMU initializer will replace the marginalization prior (typically obtained by readvancing).
void BAIMULogic::initFromIMUInit(const gtsam::Values& values, bool reinit, bool willReplaceGraph)
{
    auto* targValues = baIntegration->getMutableValues();
    int latestKFId = -1;

    if(!willReplaceGraph && !reinit) // For reinit we don't need to do this.
    {
        // If the IMU initializer does not provide a new marginalization prior (e.g. for the ablation studies), we
        // have to manually add the priors.

        // find out the latestKFId
        for(auto&& val : values)
        {
            gtsam::Symbol sym(val.key);
            if(sym.chr() == 'v' && static_cast<long long>(sym.index()) > latestKFId)
            {
                latestKFId = sym.index();
            }
        }

        // Add zero prior for rotation!
        gtsam::Values nullValues;
        auto factors = getPriorsAndAddValuesForTransform(*transformDSOToIMU, imuSettings.transformPriors, nullValues);
        for(auto&& factor : factors)
        {
            baIntegration->getBaGraphs().addFactor(factor, BIAS_AND_PRIOR_GROUP);
        }

        if(imuSettings.initSettings.scalePriorAfterInit > 0)
        {
            // Add scale prior.
            gtsam::Vector1 scaleModel;
            scaleModel(0) = imuSettings.initSettings.scalePriorAfterInit;
            gtsam::PriorFactor<ScaleGTSAM>::shared_ptr scalePrior(
                    new gtsam::PriorFactor<ScaleGTSAM>(S(0), values.at<ScaleGTSAM>(S(0)),
                                                       gtsam::noiseModel::Diagonal::Sigmas(scaleModel)));
            baIntegration->getBaGraphs().addFactor(scalePrior, BIAS_AND_PRIOR_GROUP);
        }

        // don't add keys before this key, because there will not be respective IMU factors in the graph.
        noIMUInOrderingUntilKFId = latestKFId;
    }else if(!reinit)
    {
        // If we skipped KFs in the PGBA we must skip them now as well.
        // In general it might happen that there is still a very old KF in the active window for which did not get
        // included in the PGBA.
        long long minKFId = std::numeric_limits<long long>::max();
        for(auto&& val : values)
        {
            gtsam::Symbol sym(val.key);
            if(sym.chr() == 'v' && static_cast<long long>(sym.index()) < minKFId)
            {
                minKFId = sym.index();
            }
        }
        noIMUInOrderingUntilKFId = minKFId;
    }

    // Replace the values.
    for(auto&& val : values)
    {
        bool replaceValue = true;
        gtsam::Symbol sym(val.key);
        if(sym.chr() == 'v' && static_cast<long long>(sym.index()) > latestKFId)
        {
            latestKFId = sym.index();
        }
        if(!imuSettings.initSettings.initDSOParams)
        {
            // Don't replace the values of poses (and affine brightness).
            replaceValue = sym.chr() != 'p' && sym.chr() != 'a';
        }
        if(noIMUInOrderingUntilKFId >= 0)
        {
            if((sym.chr() == 'v' || sym.chr() == 'b') && sym.index() < noIMUInOrderingUntilKFId)
            {
                replaceValue = false;
            }
        }
        if(replaceValue)
        {
            eraseAndInsert(*targValues, val.key, val.value);
        }
    }

    // When initializing from CoarseIMUInit (ablation study) we might have optimized only a single bias.
    if(!targValues->exists(gtsam::Symbol('b', latestKFId)))
    {
        targValues->insert(gtsam::Symbol('b', latestKFId), values.at(gtsam::Symbol('b', 0)));
    }

    if(!reinit)
    {
        std::cout << "INITIALIZED IMU Integration. Using IMU data in main optimization from now on!" << std::endl;

        optimizeScale = imuSettings.setting_optScaleBA;
        optimizeGravity = imuSettings.setting_optGravity;
        optimizeIMUExtrinsics = imuSettings.setting_optIMUExtrinsics;

        previousKeyframeId = latestKFId;
    }else
    {
        std::cout << "REINIT IMU Integration." << std::endl;
    }

    // Set transform from values
    transformDSOToIMU->updateWithValues(values);


    maxScaleInterval = 0.0;
    minScaleInterval = 1000.0;
}

bool BAIMULogic::isScaleFixed() const
{
    return scaleFixed;
}

double BAIMULogic::computeDynamicDSOWeight(double lastDSOEnergy, double lastRMSE, bool coarseTrackingWasGood)
{
    // Compute dynamic photometric weight. Basically a threshold robust cost function.
    double rmseThresh = imuSettings.dynamicWeightRMSEThresh;
    if(lastRMSE < rmseThresh || std::isnan(lastRMSE)) return 1.0;
    double sqrtWeight = rmseThresh / lastRMSE;
    return sqrtWeight * sqrtWeight;
}
