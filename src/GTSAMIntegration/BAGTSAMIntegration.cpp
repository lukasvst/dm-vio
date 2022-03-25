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

#include "BAGTSAMIntegration.h"

#include <util/TimeMeasurement.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/SymmetricBlockMatrix.h>
#include "Marginalization.h"
#include "AugmentedScatter.hpp"
#include <gtsam/linear/GaussianFactorGraph.h>
#include <iomanip>
#include <numeric>

#include "FullSystem/HessianBlocks.h"
#include "dso/util/FrameShell.h"
#include "GTSAMIntegration/ExtUtils.h"
#include "GTSAMUtils.h"


using namespace dmvio;
using dso::VecX, dso::MatXX, dso::EFFrame;
using std::vector;
using gtsam::Symbol;


BAGTSAMIntegration::BAGTSAMIntegration(std::unique_ptr<BAGraphs> baGraphs,
                                       std::unique_ptr<PoseTransformation> transformationDSOToBA,
                                       const GTSAMIntegrationSettings& integrationSettings, dso::CalibHessian* HCalib)
        : baGraphs(std::move(baGraphs)), transformationDSOToBA(std::move(transformationDSOToBA)), HCalib(HCalib),
          settings(integrationSettings)
{
    baValues.reset(new gtsam::Values);
    baEvalValues.reset(new gtsam::Values);
}

// called from addKeyframeToBA and from outside (FullSystem::makeKeyFrame)
void BAGTSAMIntegration::updateBAOrdering(std::vector<dso::EFFrame*>& frames)
{
    baOrdering.clear();

    gtsam::Symbol calib('c', 0);
    baOrdering.push_back(calib);
    baDimMap[calib] = CPARS;

    int i = 0;
    for(dso::EFFrame* h : frames)
    {
        int id = h->idx;
        assert(id == i);

        long fullId = h->data->shell->id;

        gtsam::Symbol poseKey('p', fullId);
        gtsam::Symbol affineKey('a', fullId);

        baOrdering.push_back(poseKey);
        baOrdering.push_back(affineKey);

        baDimMap[poseKey] = 6;
        baDimMap[affineKey] = 2; // two affine parameters.

        ++i;
    }

    baOrderingSmall.clear();
    baOrderingSmall.insert(baOrderingSmall.end(), baOrdering.begin(), baOrdering.end());

    // Let extensions add their variables
    for(auto& extension : extensions)
    {
        extension->updateBAOrdering(frames, &baOrdering, baDimMap);
    }
}

void BAGTSAMIntegration::updateBAValues(std::vector<dso::EFFrame*>& frames)
{
    dmvio::TimeMeasurement timeMeasurement("updateBAValues");

    // Filling baValues.
    dso::VecC calibVal = HCalib->value;
    gtsam::Symbol calibKey('c', 0);

    eraseAndInsert(baValues, calibKey, calibVal);

    for(dso::EFFrame* h : frames)
    {
        dso::Vec10 state = h->data->get_state();
        // Transform pose with (even though at the moment this is just the identity).
        gtsam::Pose3 pose(transformationDSOToBA->transformPose(h->data->PRE_worldToCam.matrix()));
        gtsam::Vector2 affine = state.segment(6, 2);

        long fullId = h->data->shell->id;
        gtsam::Symbol poseKey('p', fullId);
        gtsam::Symbol affineKey('a', fullId);

        eraseAndInsert(baValues, poseKey, pose);
        eraseAndInsert(baValues, affineKey, affine);
    }
}

dso::VecX
BAGTSAMIntegration::computeBAUpdate(const dso::MatXX& inputH, const VecX& inputB, double lambda,
                                    vector<EFFrame*>& frames,
                                    const MatXX& HNoLambda)
{
    dmvio::TimeMeasurement timeMeasurement("computeBAUpdate");

    updateBAValues(frames);
    *baEvalValues = *baValues;
    computeEvaluationPointValues(frames, baEvalValues);
    baGraphs->updateEvalValues(*baEvalValues);

    dmvio::TimeMeasurement timeMeasGraphs("baUpdateBuildHessians");
    lastDSOH = HNoLambda;
    lastDSOB = inputB;
    baValuesAfterBAUpdate = baValues;

    // Compute DSO Hessian and b.
    // --------------------------------------------------
    // Convert H and b from DSO to GTSAM (swapping rotation and translation, converting left increment to right increment).
    auto convertedHAndB = convertHAndBFromDSO(inputH, inputB, *transformationDSOToBA, computeDSOWeight(), baOrdering,
                                              *baEvalValues, baDimMap);
    gtsam::Matrix HFromDSO = convertedHAndB.first;
    gtsam::Vector bFromDSO = convertedHAndB.second;

    int n = HFromDSO.rows();
    int smallN = n;

    // Compute GTSAM Hessian and b
    // --------------------------------------------------
    gtsam::Ordering additionalKeys; // keys in the graphs but not in the ordering.
    auto gtsamHb = baGraphs->getHAndB(*baValues, baOrdering, baDimMap, &additionalKeys);

    // Fix all keys which were not in the ordering.
    auto accumFun = [this](int num, const gtsam::Key& key)
    {
        return num + baDimMap.at(key);
    };
    int nonFixedSize = std::accumulate(baOrdering.begin(), baOrdering.end(), 0, accumFun);
    gtsam::Matrix HFull = gtsamHb.first.topLeftCorner(nonFixedSize, nonFixedSize);
    gtsam::Vector bFull = gtsamHb.second.head(nonFixedSize);

    // The H from DSO is already multiplied with lambda correctly. We have to multiply our Hessian as well though:
    int fullN = HFull.rows();
    for(int i = 0; i < fullN; ++i)
    {
        HFull(i, i) *= (1 + lambda);
    }

    // Because the photometric factors computed by DSO and the factors in the GTSAM graph are independent we can simply add them.
    // --------------------------------------------------
    HFull.block(0, 0, n, n) += HFromDSO;
    bFull.segment(0, n) += bFromDSO;

    timeMeasGraphs.end();

    for(auto& extension : extensions)
    {
        extension->preSolve(HFull, bFull, n);
    }

    // Solve system (with preconditioning).
    dso::VecX SVecI = (HFull.diagonal() + dso::VecX::Constant(HFull.cols(), 10)).cwiseSqrt().cwiseInverse();
    gtsam::Matrix H_scaled = SVecI.asDiagonal() * HFull * SVecI.asDiagonal();

    dmvio::TimeMeasurement matrixInversionMeasurement("baMatrixInversion");
    gtsam::Vector inc = SVecI.asDiagonal() * H_scaled.ldlt().solve(SVecI.asDiagonal() * bFull);
    matrixInversionMeasurement.end();

    // Update values based on the computed increment.
    newBAValues.reset(new gtsam::Values());
    int current_pos = 0;
    for(size_t i = 0; i < baOrdering.size(); i++)
    {
        gtsam::Key k = baOrdering[i];
        size_t s = baDimMap[k];
        newBAValues->insert(k, *(baValues->at(k).retract_(inc.segment(current_pos, s))));
        current_pos += s;
    }

    canBreakOptimization = true;
    for(auto& extension : extensions)
    {
        canBreakOptimization =
                canBreakOptimization && extension->postSolve(baValues, newBAValues, inc, baOrdering, baDimMap);
    }

    // Go through keys that were not in the ordering
    // These will be fixed during optimization, but we display a warning to alert the user.
    for(auto&& key : additionalKeys)
    {
        if(!newBAValues->exists(key))
        {
            std::cout << "WARNING: Key was not in the ordering! " << gtsam::Symbol(key).chr()
                      << gtsam::Symbol(key).index() << std::endl;
            // Backup: Insert old values!
            newBAValues->insert(key, baValues->at(key));
        }
    }

    // Prepare increment for DSO (which will be used to update the point estimates).
    dso::VecX returning = inc.segment(0, smallN);

    // Exchange R and T (to convert it back to dso convention).
    for(dso::EFFrame* h : frames)
    {
        int id = CPARS + 8 * h->idx;

        long fullId = h->data->shell->id;
        gtsam::Symbol poseKey('p', fullId);

        // In practice this is usually the identity transformation.
        dso::Mat44 worldToCam = transformationDSOToBA->transformPoseInverse(
                newBAValues->at<gtsam::Pose3>(poseKey).matrix());
        Sophus::SE3d newVal(worldToCam);

        Sophus::SE3d oldVal = h->data->PRE_worldToCam;

        // Note that there might be a more efficient way to compute the increment using adjoints!
        // newVal = exp(inc) * oldVal -> inc = log(newVal * oldVal^{-1})
        dso::Vec6 increment = (newVal * oldVal.inverse()).log();

        returning.segment(id, 6) = increment;
    }

    // In DSO actually -inc is used at this point...
    return -returning;
}

void
BAGTSAMIntegration::addMarginalizedPointsBA(const dso::MatXX& H_in, const dso::VecX& b_in,
                                            std::vector<dso::EFFrame*>& frames)
{
    updateBAValues(frames);
    *baEvalValues = *baValues;
    computeEvaluationPointValues(frames, baEvalValues);

    auto convertedHAndB = convertHAndBFromDSO(H_in, b_in, *transformationDSOToBA, computeDSOWeight(), baOrdering,
                                              *baEvalValues, baDimMap);

    gtsam::LinearContainerFactor::shared_ptr lcf = convertedDSOHAndBToFactor(convertedHAndB.first,
                                                                             convertedHAndB.second, *baEvalValues);

    baGraphs->addFactor(lcf, false);

}

gtsam::LinearContainerFactor::shared_ptr
BAGTSAMIntegration::convertedDSOHAndBToFactor(const gtsam::Matrix& H, const gtsam::Vector& b,
                                              const gtsam::Values& values, double energy)
{
    // Make Hessian symmetric for numeric reasons.
    int n = H.rows();

    gtsam::Matrix Hbaa_new(n + 1, n + 1);
    Hbaa_new.setZero();
    Hbaa_new.topLeftCorner(n, n) = 0.5 * (H.transpose() + H);
    Hbaa_new.topRightCorner(n, 1) = b;
    Hbaa_new.bottomLeftCorner(1, n) = b.transpose();
    Hbaa_new(n, n) = energy;

    gtsam::Ordering ordering = baOrderingSmall;
    gtsam::FastVector<size_t> connected_dims;

    for(gtsam::Key& key : ordering)
    {
        connected_dims.push_back(baDimMap[key]);
    }

    gtsam::SymmetricBlockMatrix sm(connected_dims, true);

    sm.setFullMatrix(Hbaa_new);
    gtsam::LinearContainerFactor::shared_ptr lcf(
            new gtsam::LinearContainerFactor(gtsam::HessianFactor(ordering, sm), values));
    return lcf;
}

// Adds a prior to a frame of the BA.
void BAGTSAMIntegration::addPriorBA(dso::EFFrame* h, const dso::Vec8& H_diag, const dso::Vec8& b_in)
{
    long fullId = h->data->shell->id;

    // Convert H and b to GTSAM
    dso::MatXX H_in = dso::Mat88::Zero();
    H_in.diagonal() = H_diag;

    gtsam::Ordering ordering;
    gtsam::Symbol poseKey('p', fullId);
    gtsam::Symbol affineKey('a', fullId);
    ordering.push_back(poseKey);
    ordering.push_back(affineKey);

    auto convertedHAndB = convertHAndBFromDSO(H_in, b_in, *transformationDSOToBA, computeDSOWeight(), ordering,
                                              *baEvalValues, baDimMap);
    dso::Mat66 H_pose = convertedHAndB.first.topLeftCorner<6, 6>();
    dso::Mat22 H_affine = convertedHAndB.first.bottomRightCorner<2, 2>();
    dso::Vec6 b_pose = convertedHAndB.second.segment(0, 6);
    dso::Vec2 b_affine = convertedHAndB.second.segment(6, 2);

    gtsam::LinearContainerFactor::shared_ptr lcf(
            new gtsam::LinearContainerFactor(gtsam::HessianFactor(poseKey, H_pose, b_pose, 0.0), *baEvalValues));
    gtsam::LinearContainerFactor::shared_ptr lcfA(
            new gtsam::LinearContainerFactor(gtsam::HessianFactor(affineKey, H_affine, b_affine, 0.0), *baEvalValues));

    baGraphs->addFactor(lcf, false);
    baGraphs->addFactor(lcfA, false);
}

// Computes values where the FEJValue is used for DSO variables (pose and affine brightness) **only**.
void BAGTSAMIntegration::computeEvaluationPointValues(const std::vector<dso::EFFrame*>& frames,
                                                      gtsam::Values::shared_ptr values)
{
    dso::VecC calib = HCalib->value_zero;
    gtsam::Symbol calibKey('c', 0);
    eraseAndInsert(values, calibKey, calib);

    // Replace current value with FEJ value for poses and affine brightness.
    for(dso::EFFrame* h : frames)
    {
        dso::Vec10 stateZero = h->data->get_state_zero();
        Sophus::SE3d evalPoint = h->data->get_worldToCam_evalPT();
        gtsam::Pose3 pose(evalPoint.matrix());
        assert(stateZero.segment(0, 6).norm() == 0);
        gtsam::Vector2 affine = stateZero.segment(6, 2);

        long fullId = h->data->shell->id;

        gtsam::Symbol poseKey('p', fullId);
        gtsam::Symbol affineKey('a', fullId);

        eraseAndInsert(values, poseKey, pose);
        eraseAndInsert(values, affineKey, affine);
    }
}

void BAGTSAMIntegration::acceptBAUpdate(double energy)
{
    for(auto& extension : extensions)
    {
        extension->acceptUpdate(baValues, newBAValues);
    }

    baValues = newBAValues;
    lastDSOEnergy = energy;
}

// The method assumes that baValues was updated (e.g. by a previous call to addMarginalizedPointsBA.
void BAGTSAMIntegration::marginalizeBAFrame(dso::EFFrame* fh)
{
    gtsam::FastVector<gtsam::Key> keysToMarginalize;
    long fullId = fh->data->shell->id;

    gtsam::Symbol poseKey('p', fullId);
    gtsam::Symbol affineKey('a', fullId);

    keysToMarginalize.push_back(poseKey);
    keysToMarginalize.push_back(affineKey);

    for(auto& extension : extensions)
    {
        extension->addKeysToMarginalize(fullId, keysToMarginalize);
    }

    // baGraphs contains the logic for the actual marginalization.
    // This method will also erase the marginalized keys from baEvalValues.
    baGraphs->marginalizeFrame(keysToMarginalize, baEvalValues, baDimMap,
                               currBATimestamp, baValues);

    for(auto&& key : keysToMarginalize)
    {
        // Marginalize erases the keys from baEvalValues, but not from baValues.
        baValues->erase(key);
    }
}

void BAGTSAMIntegration::addFirstBAFrame(int keyframeId)
{
    assert(keyframeId == 0);
    for(auto& extension : extensions)
    {
        extension->addFirstBAFrame(keyframeId, baGraphs.get(), baValues);
    }
}

void
BAGTSAMIntegration::addKeyframeToBA(int keyframeId, const Sophus::SE3d& keyframePose, vector<dso::EFFrame*>& frames)
{
    dmvio::TimeMeasurement timeMeasurement("addKeyframeToBA");

    // Note: a keyframe in DSO has two IDs:
    // fh->shell->id: The number of the frame (counting all frames).
    // fh->frameId: The number of the keyframe (counting only keyframes).
    // Here we use the former.

    currBATimestamp = frames.back()->data->shell->timestamp;
    // Forward to extensions.
    for(auto& extension : extensions)
    {
        extension->addKeyframe(baGraphs.get(), baValues, keyframeId, keyframePose, frames);
    }

    updateBAOrdering(frames);
}

double BAGTSAMIntegration::getCurrBaTimestamp() const
{
    return currBATimestamp;
}

const gtsam::Values* BAGTSAMIntegration::getBaValues() const
{
    return baValues.get();
}

const gtsam::Values* BAGTSAMIntegration::getBaEvalValues() const
{
    return baEvalValues.get();
}

double BAGTSAMIntegration::getBAEnergy(bool useNewValues)
{
    return 2.0 * baGraphs->getError(useNewValues ? *newBAValues : *baValues) /
           settings.weightDSOToGTSAM/* / computeDSOWeight()*/; // caller is responsible for dividing by dynamic weight
}

void BAGTSAMIntegration::addExtension(std::shared_ptr<BAExtension> extension)
{
    extensions.push_back(extension);
}

bool BAGTSAMIntegration::canBreak() const
{
    return canBreakOptimization;
}

void BAGTSAMIntegration::removeExtension(BAExtension* extension)
{
    for(auto it = extensions.begin(); it != extensions.end(); ++it)
    {
        if(it->get() == extension)
        {
            extensions.erase(it);
            return;
        }
    }
    std::cerr << "Error: couldn't delete extension!" << std::endl;
    std::cout << "Error: couldn't delete extension!" << std::endl;
    assert(0);
}

gtsam::NonlinearFactor::shared_ptr BAGTSAMIntegration::getActiveDSOFactor()
{
    // We recompute so that no lambda is added.
    auto convertedHAndB = convertHAndBFromDSO(lastDSOH, lastDSOB, *transformationDSOToBA, computeDSOWeight(),
                                              baOrdering, *baEvalValues, baDimMap);
    return convertedDSOHAndBToFactor(convertedHAndB.first, convertedHAndB.second, *baValuesAfterBAUpdate,
                                     lastDSOEnergy *
                                     computeDSOWeight()); // note that we need to use baValues for the active Hessian.
}

gtsam::Values* BAGTSAMIntegration::getMutableValues()
{
    return baValues.get();
}

BAGraphs& BAGTSAMIntegration::getBaGraphs()
{
    return *baGraphs;
}

const std::map<gtsam::Key, size_t>& BAGTSAMIntegration::getBaDimMap() const
{
    return baDimMap;
}

const gtsam::Ordering& BAGTSAMIntegration::getBaOrdering() const
{
    return baOrdering;
}

const gtsam::Ordering& BAGTSAMIntegration::getBaOrderingSmall() const
{
    return baOrderingSmall;
}

void BAGTSAMIntegration::postOptimization(vector<dso::EFFrame*>& frames)
{
    baEvalValues.reset(new gtsam::Values());
    *baEvalValues = *baValues;
    computeEvaluationPointValues(frames, baEvalValues);
    baGraphs->updateEvalValues(*baEvalValues);
}

void BAGTSAMIntegration::setDynamicDSOWeightCallback(BAGTSAMIntegration::DynamicWeightCallback callback)
{
    dynamicWeightCallback = std::move(callback);
}

double BAGTSAMIntegration::computeDSOWeight() const
{
    return settings.weightDSOToGTSAM * dynamicDSOWeight;
}

double BAGTSAMIntegration::updateDynamicWeight(double energy, double rmse, bool coarseTrackingWasGood)
{
    if(dynamicWeightCallback)
    {
        dynamicDSOWeight = dynamicWeightCallback(energy, rmse, coarseTrackingWasGood);
    }else
    {
        dynamicDSOWeight = 1.0;
    }
    lastDSOEnergy = energy;
    return dynamicDSOWeight;
}

void dmvio::fillAdditionalKeysFromScatter(const gtsam::Ordering& originalOrdering, gtsam::Ordering& orderingToFill,
                                          const AugmentedScatter& scatter)
{
    if(originalOrdering.size() < scatter.size())
    {
        auto it = scatter.begin();
        it += originalOrdering.size();
        std::transform(it, scatter.end(), std::back_inserter(orderingToFill),
                       [](const gtsam::SlotEntry& entry) -> gtsam::Key
                       { return entry.key; });
    }
}

// Default implementation has just one factor graph and simply calls add.
void BAGraphs::addFactor(gtsam::NonlinearFactor::shared_ptr factor, int group)
{
    graph->push_back(factor);
}

// Default implementation has just one factor graph and simply linearizes and computes the Hessian.
std::pair<gtsam::Matrix, gtsam::Vector>
BAGraphs::getHAndB(const gtsam::Values& values, const gtsam::Ordering& ordering,
                   const std::map<gtsam::Key, size_t>& keyDimMap, gtsam::Ordering* fillAdditionalKeys)
{
    gtsam::GaussianFactorGraph::shared_ptr gfg = graph->linearize(values);
    AugmentedScatter scatter(*gfg, ordering, keyDimMap);
    if(fillAdditionalKeys)
    {
        fillAdditionalKeysFromScatter(ordering, *fillAdditionalKeys, scatter);
    }
    return scatter.computeHessian(*gfg);
}