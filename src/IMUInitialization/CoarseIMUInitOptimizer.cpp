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

#include "CoarseIMUInitOptimizer.h"
#include "IMU/IMUUtils.h"
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <GTSAMIntegration/Sim3GTSAM.h>
#include "GTSAMIntegration/PoseTransformationFactor.h"
#include "GTSAMIntegration/ExtUtils.h"
#include "dso/util/FrameShell.h"
#include "GTSAMIntegration/GTSAMUtils.h"

using namespace dmvio;
using namespace gtsam;
using symbol_shorthand::P, symbol_shorthand::S, symbol_shorthand::V, symbol_shorthand::B;

dmvio::CoarseIMUInitOptimizer::CoarseIMUInitOptimizer(std::shared_ptr<PoseTransformation> transformDSOToIMU,
                                                      const IMUCalibration& imuCalibration,
                                                      const CoarseIMUInitOptimizerSettings& settingsPassed)
        : transformDSOToIMU(transformDSOToIMU), imuCalibration(imuCalibration), settings(settingsPassed)
{
    gtsam::Vector6 posePriorVector;
    posePriorVector.segment(0, 3).setConstant(settings.priorRotSigma);
    posePriorVector.segment(3, 3).setConstant(settings.priorTransSigma);
    posePriorModel = noiseModel::Diagonal::Sigmas(posePriorVector);

    params.lambdaLowerBound = settings.lambdaLowerBound;
}

void CoarseIMUInitOptimizer::handleFirstFrame(int frameId)
{
    // Note: the owner of this class is responsible for adding initial values and priors for the variables optimized
    // by transformDSOToIMU (e.g S0, G0).
    gtsam::Vector3 initialVel = Vector3::Zero();
    values.insert(V(frameId), initialVel);
    gtsam::Key biasKey;
    if(settings.multipleBiases)
    {
        biasKey = B(frameId);
    }else
    {
        biasKey = B(0);
    }
    values.insert(biasKey, imuBias::ConstantBias(gtsam::Vector6::Zero()));
}

void dmvio::CoarseIMUInitOptimizer::addPose(int frameId, const Sophus::SE3d& camToWorld,
                                            const gtsam::PreintegratedImuMeasurements* imuData)
{
    // Note that we are optimizing worldToCam!
    gtsam::Pose3 framePose(camToWorld.inverse().matrix());

    if(prevFrameId > 0)
    {
        assert(imuData != nullptr);
        gtsam::Key prevBiasKey;
        if(settings.multipleBiases)
        {
            // Add random walk factor.
            prevBiasKey = B(prevFrameId);
            auto biasNoiseModel = computeBiasNoiseModel(imuCalibration, *imuData);

            gtsam::NonlinearFactor::shared_ptr bias_factor(
                    new BetweenFactor<gtsam::imuBias::ConstantBias>(
                            prevBiasKey, B(frameId),
                            gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(),
                                                         gtsam::Vector3::Zero()), biasNoiseModel));
            graph.add(bias_factor);
            values.insert(B(frameId), values.at<imuBias::ConstantBias>(B(prevFrameId)));
        }else
        {
            prevBiasKey = B(0);
        }
        // Add IMU factor
        gtsam::NonlinearFactor::shared_ptr imuFactor(
                new gtsam::ImuFactor(P(prevFrameId), V(prevFrameId),
                                     P(frameId), V(frameId), prevBiasKey,
                                     *imuData));

        // The IMUFactor needs to be transformed (from IMU frame to DSO frame).
        gtsam::Values fixedValues;
        if(settings.fixPoses)
        {
            fixedValues.insert(P(prevFrameId), prevFramePose);
            fixedValues.insert(P(frameId), framePose);
        }
        auto transformedFactor = boost::make_shared<PoseTransformationFactor>(imuFactor,
                                                                              *transformDSOToIMU,
                                                                              settings.conversionType, fixedValues);

        graph.add(transformedFactor);

        values.insert(V(frameId), values.at<gtsam::Vector3>(V(prevFrameId)));
    }else
    {
        handleFirstFrame(frameId);
    }

    if(!settings.fixPoses)
    {
        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(P(frameId), framePose, posePriorModel));
        values.insert(P(frameId), framePose);
    }

    poseIds.push_back(frameId);

    // Remove factors if the maximum number of frames is reached.
    if(settings.maxNumPoses > 0)
    {
        std::set<gtsam::Key> keysToRemove;
        while(poseIds.size() > settings.maxNumPoses)
        {
            // Remove a pose and the corresponding factors from the graph.
            int toRemove = poseIds[0];
            if(!settings.fixPoses)
            {
                keysToRemove.insert(P(toRemove));
            }
            keysToRemove.insert(V(toRemove));
            if(settings.multipleBiases)
            {
                keysToRemove.insert(B(toRemove));
            }
            poseIds.pop_front();
            numFrames--;

        }
        removeKeysFromGraph(graph, keysToRemove, 5);
        for(auto&& key : keysToRemove)
        {
            values.erase(key);
        }
    }

    numFrames++;
    prevFrameId = frameId;
    prevFramePose = framePose;
}

dmvio::CoarseIMUInitOptimizer::OptimizationResult dmvio::CoarseIMUInitOptimizer::optimize()
{
    if(settings.updatePoses)
    {
        // Get the newest poses from DSO.
        boost::unique_lock<boost::mutex> lock(dso::FrameShell::shellPoseMutex);
        for(auto&& factor : graph)
        {
            PoseTransformationFactor* casted = dynamic_cast<PoseTransformationFactor*>(factor.get());
            if(casted)
            {
                auto&& keys = casted->fixedValues.keys();
                for(auto&& key : keys)
                {
                    gtsam::Symbol sym(key);
                    if(sym.chr() == 'p')
                    {
                        const auto* shell = activeShells.at(sym.index());
                        // compute updated camToWorld
                        Sophus::SE3d camToWorld = shell->camToWorld;
                        if(shell->keyframeId == -1)
                        {
                            camToWorld = shell->trackingRef->camToWorld * shell->camToTrackingRef;
                        }
                        assert(sym.index() == shell->id);
                        eraseAndInsert(casted->fixedValues, key, gtsam::Pose3(camToWorld.inverse().matrix()));
                    }
                }
            }
        }
    }

    LevenbergMarquardtOptimizer optimizer(graph, values, params);
    optimizedValues = optimizer.optimize();
    transformDSOToIMU->updateWithValues(optimizedValues);
    double error = optimizer.error();
    double normalizedError = error / numFrames;

    bool good = true;
    // If error is too high we assume that odometry failed and request a full reset.
    if((settings.requestFullResetErrorThreshold > 0 && error > settings.requestFullResetErrorThreshold) ||
       (settings.requestFullResetNormalizedErrorThreshold > 0 &&
        normalizedError > settings.requestFullResetNormalizedErrorThreshold))
    {
        std::cout << "Large CoarseIMUInitializer error! Requesting full reset! " << normalizedError << std::endl;
        good = false;
        dso::setting_fullResetRequested = true;
    }

    return OptimizationResult(optimizer.iterations(), error, normalizedError, good);
}

std::shared_ptr<PoseTransformation> dmvio::CoarseIMUInitOptimizer::getUpdatedTransform()
{
    return transformDSOToIMU;
}

gtsam::Key CoarseIMUInitOptimizer::getBiasKey()
{
    return settings.multipleBiases ? B(prevFrameId) : B(0);
}

gtsam::imuBias::ConstantBias CoarseIMUInitOptimizer::getBias()
{
    return values.at<gtsam::imuBias::ConstantBias>(getBiasKey());
}

gtsam::Marginals CoarseIMUInitOptimizer::getMarginals()
{
    return gtsam::Marginals(graph, optimizedValues);
}

void CoarseIMUInitOptimizer::takeOverOptimizedValues()
{
    values = optimizedValues;
}

void CoarseIMUInitOptimizer::addPose(const dso::FrameShell& shell, const gtsam::PreintegratedImuMeasurements* imuData)
{
    boost::unique_lock<boost::mutex> lock(dso::FrameShell::shellPoseMutex);
    if(settings.updatePoses)
    {
        activeShells[shell.id] = &shell;
    }
    addPose(shell.id, shell.camToWorld, imuData);
}


CoarseIMUInitOptimizer::OptimizationResult::OptimizationResult(int numIterations, double error, double normalizedError,
                                                               bool good)
        : numIterations(numIterations), error(error), normalizedError(normalizedError), good(good)
{}
