/**
* This file is part of DM-VIO.
* The code in this file is in part based on code written by Vladyslav Usenko for the paper "Direct Visual-Inertial Odometry with Stereo Cameras".
*
* Copyright (c) 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>, Vladyslav Usenko
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

#include <GTSAMIntegration/Marginalization.h>
#include <util/TimeMeasurement.h>
#include "CoarseIMULogic.h"
#include "IMUInitialization/GravityInitializer.h"
#include "IMUUtils.h"
#include "GTSAMIntegration/GTSAMUtils.h"


dmvio::CoarseIMULogic::CoarseIMULogic(std::unique_ptr<PoseTransformation> transformBAToIMU,
                                      boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams,
                                      const IMUCalibration& imuCalibration, dmvio::IMUSettings& imuSettings)
        : transformBAToIMU(std::move(transformBAToIMU)),
          preintegrationParams(preintegrationParams),
          imuSettings(imuSettings),
          imuCalibration(imuCalibration)
{
    coarseBiasFile.open(imuSettings.resultsPrefix + "coarsebiasdso.txt");
}


Sophus::SE3d dmvio::CoarseIMULogic::addIMUData(const dmvio::IMUData& imuData, int frameId, double frameTimestamp,
                                               int lastFrameId,
                                               boost::shared_ptr<gtsam::PreintegratedImuMeasurements> additionalMeasurements,
                                               int dontMargFrame)
{
    dmvio::TimeMeasurement timeMeasurement("addIMUData");
    if(lastFrameId < 0)
    {
        lastFrameId = frameId - 1;
    }

    int keyframeId = currentKeyframeId;

    currCoarseTimestamp = frameTimestamp;

    // add symbols to graph:
    gtsam::Pose3 currentPose = coarseValues->at<gtsam::Pose3>(gtsam::Symbol('p', lastFrameId));
    gtsam::imuBias::ConstantBias currentBias = coarseValues->at<gtsam::imuBias::ConstantBias>(
            gtsam::Symbol('b', lastFrameId));
    gtsam::Vector3 currentVelocity = coarseValues->at<gtsam::Vector3>(gtsam::Symbol('v', lastFrameId));

    // Select factors to marginalize out. We want to keep in the graph keyframe pose, previous and current states. We also want to keep the prepared keyframe pose
    gtsam::FastVector<gtsam::Key> keysToMarginalize;
    gtsam::FastSet<gtsam::Key> keysInGraph = coarseGraph->keys();

    gtsam::FastSet<gtsam::Key> setOfKeysToMarginalize;

    for(const gtsam::Key& k : keysInGraph)
    {
        gtsam::Symbol s(k);

        int idx = s.index();

        if(s.chr() == 's')
            continue;

        if(idx == frameId || idx == lastFrameId)
            continue;

        if((idx == keyframeId || idx == dontMargFrame) && s.chr() == 'p')
            continue;

        if(setOfKeysToMarginalize.find(k) == setOfKeysToMarginalize.end())
        {
            keysToMarginalize.push_back(k);
            setOfKeysToMarginalize.insert(k);
        }
    }

    if(!keysToMarginalize.empty())
    {
        coarseGraph = marginalizeOut(*coarseGraph, *coarseValues, keysToMarginalize, nullptr, true);
    }

    // Define keys to be used in this iteration
    gtsam::Key poseKeyframeKey = gtsam::Symbol('p',
                                               keyframeId);

    gtsam::Key poseCurrentKey = gtsam::Symbol('p', frameId);
    gtsam::Key velCurrentKey = gtsam::Symbol('v', frameId);
    gtsam::Key biasCurrentKey = gtsam::Symbol('b', frameId);

    gtsam::Key posePrevKey = gtsam::Symbol('p', lastFrameId);
    gtsam::Key velPrevKey = gtsam::Symbol('v', lastFrameId);
    gtsam::Key biasPrevKey = gtsam::Symbol('b', lastFrameId);

    // Integrate IMU data
    boost::shared_ptr<gtsam::PreintegratedImuMeasurements> imuMeasurements;
    if(additionalMeasurements)
    {
        imuMeasurements = additionalMeasurements;
    }else
    {
        imuMeasurements.reset(new gtsam::PreintegratedImuMeasurements(preintegrationParams, currentBias));
    }
    for(size_t i = 0; i < imuData.size(); i++)
    {
        auto& measurement = imuData[i];
        if(measurement.getIntegrationTime() == 0.0) continue;
        imuMeasurements->integrateMeasurement(gtsam::Vector(measurement.getAccData()),
                                              gtsam::Vector(measurement.getGyrData()),
                                              measurement.getIntegrationTime());
    }

    // Create IMU factor.
    gtsam::ImuFactor::shared_ptr imuFactor(
            new gtsam::ImuFactor(posePrevKey, velPrevKey,
                                 poseCurrentKey, velCurrentKey, biasPrevKey,
                                 *imuMeasurements));

    if(imuMeasurements->preintMeasCov().hasNaN() || imuFactor->noiseModel()->sigmas().hasNaN())
    {
        std::cout << "Exiting because of bad measurement covariance." << std::endl;
        exit(1);
    }

    gtsam::noiseModel::Diagonal::shared_ptr biasNoiseModel = computeBiasNoiseModel(imuCalibration, *imuMeasurements);

    // Add bias random walk factor.
    gtsam::NonlinearFactor::shared_ptr bias_factor(
            new gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                    biasPrevKey, biasCurrentKey,
                    gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(),
                                                 gtsam::Vector3::Zero()), biasNoiseModel));

    // In the coarse graph we optimize poses in metric frame (imu to world), so we don't need any PoseTransformationFactors.
    // Instead, we transform the DSO Hessian to the metric frame.
    coarseGraph->push_back(imuFactor);
    coarseGraph->push_back(bias_factor);

    coarseValues->insert(poseCurrentKey, currentPose);
    coarseValues->insert(velCurrentKey, currentVelocity);
    coarseValues->insert(biasCurrentKey, currentBias);

    if(currentPose.matrix().hasNaN() || currentVelocity.hasNaN() || currentBias.vector().hasNaN())
    {
        std::cout << "ERROR: NaNs in the system, exiting!" << std::endl;
        exit(1);
    }

    // Predict the new pose based on the IMU data (will be used as an initialization).
    gtsam::LevenbergMarquardtOptimizer::shared_ptr optimizer(
            new gtsam::LevenbergMarquardtOptimizer(*coarseGraph, *coarseValues));
    gtsam::Values optimizedValues = optimizer->optimize();
    gtsam::Values newValues;
    for(gtsam::Values::iterator it = optimizedValues.begin(); it != optimizedValues.end(); ++it)
    {
        if(gtsam::Symbol((*it).key).index() == currentKeyframeId && imuSettings.fixKeyframeDuringCoarseTracking)
        {
            // Don't change the values of the keyframe...
            newValues.insert(it->key, coarseValues->at(it->key));
        }else
        {
            newValues.insert(it->key, it->value);
        }
    }
    *coarseValues = newValues;

    transformIMUToDSOForCoarse->updateWithValues(*coarseValues);
    // Convert T_w_f to T_f_r:
    Sophus::SE3d referenceToFrame(
            transformIMUToDSOForCoarse->transformPose(coarseValues->at<gtsam::Pose3>(poseCurrentKey).matrix()));

    currentPoseKey = poseCurrentKey;
    refPoseKey = poseKeyframeKey;

    coarseOrdering.clear();
    coarseOrdering.push_back(refPoseKey);
    coarseOrdering.push_back(currentPoseKey);

    gtsam::KeySet set;
    set.insert(refPoseKey);
    set.insert(currentPoseKey);

    for(const gtsam::Key& k : coarseGraph->keys())
    {
        if(k != refPoseKey && k != currentPoseKey)
        {
            coarseOrdering.push_back(k);
            set.insert(k);
        }
    }

    // The returned prediction will be used as an initialization for the coarse direct image alignment.
    return referenceToFrame;
}

Sophus::SE3d
dmvio::CoarseIMULogic::initCoarseGraph(int keyframeId, std::unique_ptr<InformationBAToCoarse> informationBAToCoarse)
{
    currentKeyframeId = keyframeId;

    gtsam::Key poseKey0 = gtsam::Symbol('p', keyframeId);
    gtsam::Key velocityKey0 = gtsam::Symbol('v', keyframeId);
    gtsam::Key biasKey0 = gtsam::Symbol('b', keyframeId);

    // Take over transforms from BA.
    if(informationBAToCoarse)
    {
        transformBAToIMU = std::move(informationBAToCoarse->transformBAToIMU);
        transformIMUToDSOForCoarse = std::move(informationBAToCoarse->transformIMUToDSOForCoarse);
        scale = informationBAToCoarse->latestBAScale;
    }

    coarseGraph.reset(new gtsam::NonlinearFactorGraph());
    coarseValues.reset(new gtsam::Values);

    currentKeyframeId = keyframeId;

    gtsam::Vector3 initialVelocity = (gtsam::Vector(3) << 0, 0, 0).finished();
    gtsam::imuBias::ConstantBias initialBias = gtsam::imuBias::ConstantBias();
    gtsam::Pose3 poseFromBA;
    bool gotBABias = !(imuSettings.skipFirstKeyframe && firstCoarseInit);

    if(informationBAToCoarse)
    {
        initialVelocity = informationBAToCoarse->latestBAVel;
        initialBias = informationBAToCoarse->latestBABias;
        poseFromBA = informationBAToCoarse->latestBAPose;
    }

    // Transform DSO pose to IMU.
    gtsam::Pose3 initialPose(transformBAToIMU->transformPose(poseFromBA.matrix()));

    // Add pose prior on the keyframe
    double rotVariance = imuSettings.baToCoarseRotVariance;
    double poseVariance = imuSettings.baToCoarsePoseVariance;
    double velVariance = imuSettings.baToCoarseVelVariance;
    double accBiasVariance = imuSettings.baToCoarseAccBiasVariance;
    double gyrBiasVariance = imuSettings.baToCoarseGyrBiasVariance;

    gtsam::noiseModel::Diagonal::shared_ptr pose_prior_model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6)
            << rotVariance, rotVariance, rotVariance, poseVariance, poseVariance, poseVariance).finished());
    coarseGraph->add(gtsam::PriorFactor<gtsam::Pose3>(poseKey0, initialPose, pose_prior_model));

    // Add prior on bias and velocity.
    if(gotBABias)
    {
        if(imuSettings.setting_transferCovToCoarse)
        {
            coarseGraph->add(informationBAToCoarse->priorFactor);
        }else
        {
            gtsam::noiseModel::Diagonal::shared_ptr vel_prior_model = gtsam::noiseModel::Diagonal::Variances(
                    (gtsam::Vector(3) << velVariance, velVariance, velVariance).finished());
            coarseGraph->add(gtsam::PriorFactor<gtsam::Vector3>(velocityKey0, initialVelocity, vel_prior_model));

            gtsam::noiseModel::Diagonal::shared_ptr bias_prior_model = gtsam::noiseModel::Diagonal::Variances(
                    (gtsam::Vector(6)
                            << accBiasVariance, accBiasVariance, accBiasVariance, gyrBiasVariance, gyrBiasVariance, gyrBiasVariance).finished());
            coarseGraph->add(
                    gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(biasKey0, initialBias, bias_prior_model));
        }
    }

    coarseValues->insert(poseKey0, initialPose);
    coarseValues->insert(velocityKey0, initialVelocity);
    coarseValues->insert(biasKey0, initialBias);

    firstCoarseInit = false;

    // BA gtsam poses are cam to world
    gtsam::Pose3 lastKFToCurr;
    if(informationBAToCoarse)
    {
        lastKFToCurr = informationBAToCoarse->latestBAPose.inverse() * informationBAToCoarse->latestBAPosePrevKeyframe;
    }
    return Sophus::SE3d(lastKFToCurr.matrix());
}

Sophus::SE3d
dmvio::CoarseIMULogic::computeCoarseUpdate(const dso::Mat88& H_in, const dso::Vec8& b_in, float extrapFac, float lambda,
                                           double& incA, double& incB, double& incNorm)
{
    dmvio::TimeMeasurement timeMeasurement("computeCoarseUpdate");

    PoseTransformation& transformIMUToCoarse = *transformIMUToDSOForCoarse;
    transformIMUToCoarse.updateWithValues(*coarseValues); // Set reference pose.
    // Convert Hessian and b to absolute poses.
    auto dsoHAndB = convertCoarseHToGTSAM(transformIMUToCoarse, H_in * imuSettings.setting_weightDSOCoarse,
                                          b_in * imuSettings.setting_weightDSOCoarse,
                                          coarseValues->at<gtsam::Pose3>(currentPoseKey));

    // Linearize factor graph.
    gtsam::GaussianFactorGraph::shared_ptr gfg = coarseGraph->linearize(*coarseValues);
    std::map<gtsam::Key, size_t> keyDimMap = gfg->getKeyDimMap();

    std::pair<gtsam::Matrix, gtsam::Vector> gtsamHAndB = gfg->hessian(coarseOrdering);

    int nrowsGT = gtsamHAndB.first.rows();
    gtsam::Matrix HComplete(nrowsGT + 2, nrowsGT + 2);
    gtsam::Vector bComplete(nrowsGT + 2);

    HComplete.block(2, 2, nrowsGT, nrowsGT) = gtsamHAndB.first; // Fill correct part with the matrix from GTSAM
    HComplete.block(0, 0, nrowsGT + 2, 2) = gtsam::Matrix::Zero(nrowsGT + 2, 2); // Fill the rest with
    // zeros.
    HComplete.block(0, 2, 2, nrowsGT) = gtsam::Matrix::Zero(2, nrowsGT);

    // Add DSO part of the Hessian.
    HComplete.block(0, 0, 14, 14) += dsoHAndB.first;

    bComplete.segment(0, 2) = gtsam::Matrix::Zero(2, 1);
    bComplete.segment(2, nrowsGT) = -gtsamHAndB.second; // The b in GTSAM resembles -b in DSO!
    bComplete.segment(0, 14) += dsoHAndB.second;

    // Use lambda multiplication...
    for(int i = 0; i < nrowsGT + 2; i++) HComplete(i, i) *= (1 + lambda);

    // --------------------------------------------------
    // Compute update step
    // --------------------------------------------------
    gtsam::Vector inc = HComplete.ldlt().solve(-bComplete);

    inc *= extrapFac;

    if(imuSettings.fixKeyframeDuringCoarseTracking)
    {
        // GTSAM Pose contains first rotation, then translation -> only remove the translational part.
        inc.segment(5, 3) = gtsam::Matrix::Zero(3, 1);
    }

    // Apply update.
    newCoarseValues.reset(new gtsam::Values());
    int current_pos = 2;
    for(size_t i = 0; i < coarseOrdering.size(); i++)
    {
        gtsam::Key k = coarseOrdering[i];
        size_t s = keyDimMap[k];
        newCoarseValues->insert(k, *(coarseValues->at(k).retract_(inc.segment(current_pos, s))));
        current_pos += s;
    }

    // Compute increment, norm, and updated relative pose for CoarseTracker.
    incA = inc(0);
    incB = inc(1);

    incNorm = inc.norm();
    transformIMUToCoarse.updateWithValues(*newCoarseValues); // Set reference pose.
    Sophus::SE3d newReferenceToFrame(
            transformIMUToCoarse.transformPose(newCoarseValues->at<gtsam::Pose3>(currentPoseKey).matrix()));

    return newReferenceToFrame;

}

Sophus::SE3d dmvio::CoarseIMULogic::getCoarseKFPose()
{
    return Sophus::SE3d(coarseValues->at<gtsam::Pose3>(gtsam::Symbol('p', currentKeyframeId)).matrix());
}

void dmvio::CoarseIMULogic::updateCoarsePose(const Sophus::SE3& refToFrame)
{
    // GTSAM expects currentImu to world, we passed referenceCamera to currentCamera.
    PoseTransformation& transformIMUToCoarse = *transformIMUToDSOForCoarse;
    transformIMUToCoarse.updateWithValues(*coarseValues); // Set reference pose.

    gtsam::Pose3 currentIMUToWorld(transformIMUToCoarse.transformPoseInverse(refToFrame.matrix()));

    eraseAndInsert(coarseValues, currentPoseKey, currentIMUToWorld);
}

void dmvio::CoarseIMULogic::acceptCoarseUpdate()
{
    coarseValues = newCoarseValues;
}

// Our factor graph contains (and marginalizes old frames), so we need to add the linearized direct image alignment factor.
void dmvio::CoarseIMULogic::addVisualToCoarseGraph(const dso::Mat88& H, const dso::Vec8& b, bool trackingIsGood)
{
    if(!imuSettings.addVisualToCoarseGraphIfTrackingBad && !trackingIsGood) return;

    PoseTransformation& transformIMUToCoarse = *transformIMUToDSOForCoarse;
    transformIMUToCoarse.updateWithValues(*coarseValues); // Set reference pose.
    auto dsoHAndB = convertCoarseHToGTSAM(transformIMUToCoarse, H * imuSettings.setting_weightDSOCoarse,
                                          b * imuSettings.setting_weightDSOCoarse,
                                          coarseValues->at<gtsam::Pose3>(currentPoseKey));
    gtsam::Matrix HFull = std::move(dsoHAndB.first);
    gtsam::Vector bFull = std::move(dsoHAndB.second);

    bFull = -bFull; // The b in GTSAM resembles -b in DSO!

    // Marginalize out a, b as they shall not be included in the factor graph...
    gtsam::Matrix Hmm = HFull.block(0, 0, 2, 2);
    gtsam::Matrix Hma = HFull.block(0, 2, 2, 12);
    gtsam::Matrix Haa = HFull.block(2, 2, 12, 12);

    gtsam::Vector bm = bFull.segment(0, 2);
    gtsam::Vector ba = bFull.segment(2, 12);

    gtsam::Matrix HmmInv = Hmm.completeOrthogonalDecomposition().pseudoInverse();

    gtsam::Matrix HaaNew = Haa - Hma.transpose() * HmmInv * Hma;
    gtsam::Vector baNew = ba - Hma.transpose() * HmmInv * bm;

    gtsam::LinearContainerFactor::shared_ptr lcf(new gtsam::LinearContainerFactor(
            gtsam::HessianFactor(refPoseKey, currentPoseKey, HaaNew.block(0, 0, 6, 6), HaaNew.block(0, 6, 6, 6),
                                 baNew.segment(0, 6), HaaNew.block(6, 6, 6, 6), baNew.segment(6, 6), 0),
            *coarseValues));

    coarseGraph->add(lcf);
}

gtsam::imuBias::ConstantBias dmvio::CoarseIMULogic::getBias(int frameId)
{
    gtsam::imuBias::ConstantBias currentBias;
    if(coarseValues)
    {
        currentBias = coarseValues->at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', frameId));
    }
    return currentBias;
}

gtsam::Vector3 dmvio::CoarseIMULogic::getVelocity(int frameId)
{
    gtsam::Vector3 velocity = gtsam::Vector3::Identity();
    if(coarseValues)
    {
        velocity = coarseValues->at<gtsam::Vector3>(gtsam::Symbol('v', frameId));
    }
    return velocity;
}


void dmvio::CoarseIMULogic::printCoarseBiases(const dmvio::GTData* gtData, int frameId)
{
    if(gtData && coarseValues)
    {
        gtsam::imuBias::ConstantBias currentBias = coarseValues->at<gtsam::imuBias::ConstantBias>(
                gtsam::Symbol('b', frameId));
        Eigen::Vector3d gtTrans = gtData->biasTranslation;
        Eigen::Vector3d gtRot = gtData->biasRotation;
        Eigen::Vector3d errorTrans = currentBias.accelerometer() - gtTrans;
        Eigen::Vector3d errorRot = currentBias.gyroscope() - gtRot;

        coarseBiasFile << std::fixed << std::setprecision(6) << currCoarseTimestamp << ' ' << std::setprecision(20)
                       << gtTrans.transpose() << ' ' << gtRot.transpose() << ' '
                       << currentBias.accelerometer().transpose() << ' ' << currentBias.gyroscope().transpose() << '\n';
    }
}

double dmvio::CoarseIMULogic::getScale() const
{
    return scale;
}

