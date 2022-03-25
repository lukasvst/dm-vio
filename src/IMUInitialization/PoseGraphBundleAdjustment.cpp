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

#include "PoseGraphBundleAdjustment.h"

#include <utility>
#include "util/TimeMeasurement.h"
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include "GTSAMIntegration/Sim3GTSAM.h"
#include "GTSAMIntegration/Marginalization.h"
#include "GTSAMIntegration/FEJNoiseModelFactor.h"
#include "GTSAMIntegration/GTSAMUtils.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace dmvio;
using namespace gtsam;

using symbol_shorthand::P, symbol_shorthand::S, symbol_shorthand::V, symbol_shorthand::B, symbol_shorthand::A;

PoseGraphBundleAdjustment::PoseGraphBundleAdjustment(DelayedMarginalizationGraphs* delayedMarginalizationPassed,
                                                     const IMUCalibration& imuCalibration, const PGBASettings& settings,
                                                     std::shared_ptr<TransformDSOToIMU> transformDSOToIMUPassed)
        : delayedMarginalization(delayedMarginalizationPassed),
          imuCalibration(imuCalibration),
          settings(settings),
          transformDSOToIMU(std::move(transformDSOToIMUPassed))
{
    inputDelayedGraph = delayedMarginalization->addDelayedGraph(settings.delay,
                                                                BAIMULogic::NO_IMU_GROUP); // This graph will only contain visual factors.
}

void PoseGraphBundleAdjustment::prepareOptimization()
{
    // clone DelayedGraph
    delayedGraph = std::make_unique<DelayedGraph>(*inputDelayedGraph);
    disconnectedGraph = delayedMarginalization->addDisconnectedGraph(delayedGraph->getMaxGroupInGraph());
}

gtsam::Values dmvio::PoseGraphBundleAdjustment::optimize(gtsam::NonlinearFactor::shared_ptr activeDSOFactor,
                                                         const gtsam::Values& baValues,
                                                         const gtsam::Values& imuInitValues, bool noOptimization)
{
    dmvio::TimeMeasurement fullMeas("PGBAFull");

    graph = delayedGraph->getGraph().get();

    if(!imuInitValues.empty())
    {
        transformDSOToIMU->updateWithValues(imuInitValues);
        std::cout << "Updating transform with index " << transformDSOToIMU->getSymbolInd() << " to val: "
                  << transformDSOToIMU->getScale() << std::endl;
    }

    graph->push_back(activeDSOFactor);
    dsoFactorPos = graph->size() - 1;

    const gtsam::Values& inputValues = delayedGraph->getDelayedCurrValues();
    // Call buildGraph to fill the graph with the IMUFactors
    gtsam::Values values = buildGraph(*graph, baValues, inputValues, imuInitValues, noOptimization);

    if(noOptimization) return values;

    dmvio::TimeMeasurement optMeas("PGBA");

    // Enable FEJ for this optimization.
    delayedGraph->setFEJValuesForFactors(true);

    // Call gtsam optimize and return the value.
    gtsam::LevenbergMarquardtParams params = gtsam::LevenbergMarquardtParams::CeresDefaults();
    LevenbergMarquardtOptimizer optimizer(*graph, values, params);
    Values newValues = optimizer.optimize();
    transformDSOToIMU->updateWithValues(newValues);

    double error = optimizer.error();
    std::cout << "PGBA Error: " << error << std::endl;

    delayedGraph->setFEJValuesForFactors(false);

    return newValues;
}

gtsam::Marginals PoseGraphBundleAdjustment::getMarginals(const Values& values)
{
    return gtsam::Marginals(*graph, values);
}

// Extend graph with the disconnected graph.
gtsam::Values dmvio::PoseGraphBundleAdjustment::extendGraph(gtsam::NonlinearFactor::shared_ptr activeDSOFactor,
                                                            gtsam::Values&& previouslyOptimizedValues,
                                                            const gtsam::Values& baValues,
                                                            const KeyframeDataContainer& cachedData)
{
    dmvio::TimeMeasurement fullMeas("ExtendGraph");
    // Note: We need to add the cachedData before calling insertIMUFactorsAndValues, because it accesses prevKFIds, which is updated here.
    for(auto&& data : cachedData)
    {
        addKeyframe(data.second, data.first);
    }

    // New factors added, this means that probably also new variables were added.
    // --> Unless all variables are available otherwise we need to re-optimize.

    // Find last available bias and velocity from previouslyOptimizedValue
    auto previouslyOptimizedKeys = previouslyOptimizedValues.keys();

    gtsam::Vector3 velocity = Vector3::Zero();
    imuBias::ConstantBias imuBias;

    gtsam::Key latestVelKey = getMaxKeyWithChr(previouslyOptimizedKeys, 'v');
    gtsam::Key latestBiasKey = getMaxKeyWithChr(previouslyOptimizedKeys, 'b');
    velocity = previouslyOptimizedValues.at<Vector3>(latestVelKey);
    imuBias = previouslyOptimizedValues.at<imuBias::ConstantBias>(latestBiasKey);

    this->graph = delayedGraph->getGraph().get(); // also update member variable.
    gtsam::NonlinearFactorGraph& graph = *(delayedGraph->graph);
    // previouslyOptimizedValues must not be used below this!
    gtsam::Values values(std::move(previouslyOptimizedValues));


    // -------------------- Extend graph with new IMU factors.
    const gtsam::Values& inputValues2 = disconnectedGraph->delayedCurrValues;

    gtsam::KeyVector addedKeys(activeDSOFactor->keys());
    for(auto&& factor : disconnectedGraph->addedFactors)
    {
        graph.add(factor);
        auto&& keys = factor->keys();
        addedKeys.insert(addedKeys.end(), keys.begin(), keys.end());
    }
    // sort and make unique.
    std::sort(addedKeys.begin(), addedKeys.end());
    addedKeys.erase(std::unique(addedKeys.begin(), addedKeys.end()), addedKeys.end());

    disconnectedGraph->addedFactors.clear();

    // add active DSO factor.
    graph.push_back(activeDSOFactor);
    dsoFactorPos = graph.size() - 1;


    gtsam::Values imuInputValues = disconnectedGraph->delayedCurrValues;
    // Use baValues if possible, otherwise new delayedCurrValues.
    for(auto&& pair : baValues)
    {
        eraseAndInsert(imuInputValues, pair.key, pair.value);
    }

    long long minConnectedPoseInd = prevKFIds.at(cachedData.front().second);

    // Add new values
    for(auto&& key : addedKeys)
    {
        if(!values.exists(key))
        {
            values.insert(key, imuInputValues.at(key));
        }
    }

    // We don't do negative energy compensation here again, although maybe it might be useful.
    if(!settings.prepareGraphAddDelValues)
    {
        for(auto&& val : disconnectedGraph->delayedValues)
        {
            if(!delayedGraph->delayedValues.exists(val.key))
            {
                delayedGraph->delayedValues.insert(val.key, val.value);
            }
        }
    }


    // We want to use disconnectedGraph->delayedCurrValues, and eraseAndInsert all baValues.
    numIMUFactors = insertIMUFactorsAndValues(graph, cachedData, imuInputValues, values, minConnectedPoseInd, imuBias,
                                              velocity);

    // Optimize with the newly added variables.
    delayedGraph->setFEJValuesForFactors(true);

    gtsam::LevenbergMarquardtParams params = gtsam::LevenbergMarquardtParams::CeresDefaults();
    LevenbergMarquardtOptimizer optimizer(graph, values, params);
    Values newValues = optimizer.optimize();
    transformDSOToIMU->updateWithValues(newValues);

    delayedGraph->setFEJValuesForFactors(false);

    return newValues;
}

gtsam::Values
dmvio::PoseGraphBundleAdjustment::buildGraph(gtsam::NonlinearFactorGraph& graph, const gtsam::Values& poseInputValues,
                                             const gtsam::Values& poseInputValues2, const gtsam::Values& imuInputValues,
                                             bool noOptimization)
{
    gtsam::Values values;
    // get Ordering of all poses.
    auto&& keyVector = graph.keyVector(); // this method already sorts it!

    // find out first pose which is connected to the end!
    long long minConnectedPoseInd = insertValuesAndGetMinConnectedPoseInd(poseInputValues, poseInputValues2, values,
                                                                          keyVector);

    allPosesUsed = minConnectedPoseInd == 0;

    // First remove all preintegrated measurements before minConnectedPoseInd
    while(preintegratedForKF.size() > 0 && preintegratedForKF.front().second <= minConnectedPoseInd)
    {
        preintegratedForKF.pop_front();
    }
    minConnectedPoseInd = std::max((int) minConnectedPoseInd, prevKFIds[preintegratedForKF[0].second]);
    assert(prevKFIds[preintegratedForKF[0].second] == minConnectedPoseInd);

    firstId = minConnectedPoseInd;

    if(!noOptimization)
    {
        // Workaround: Unfortunately the DSO marginalization factors can get a (large) negative energy which is problematic
        // for the GTSAM optimizer. So we do one GN-iteration, and manually add this energy to the graph.
        auto negFac = compensateNegativeEnergy(graph, values, *transformDSOToIMU);
        if(negFac)
        {
            graph.push_back(negFac);
        }
    }

    // From there one insert all IMU factors.
    imuBias::ConstantBias imuBias;
    gtsam::Vector3 velocity = Vector3::Zero();
    if(imuInputValues.exists(B(0)))
    {
        imuBias = imuInputValues.at<imuBias::ConstantBias>(B(0));
    }

    // Insert values for first KF:
    if(imuInputValues.exists(V(firstId)))
    {
        velocity = imuInputValues.at<Vector3>(V(firstId));
    }
    if(imuInputValues.exists(B(firstId)))
    {
        imuBias = imuInputValues.at<imuBias::ConstantBias>(B(firstId));
    }
    values.insert(B(firstId), imuBias);
    values.insert(V(firstId), velocity);

    // Insert values for IMU data: If initial velocity or bias does not exist for a KF we use the values of the previous KF.
    numIMUFactors = insertIMUFactorsAndValues(graph, preintegratedForKF, imuInputValues, values, minConnectedPoseInd,
                                              imuBias, velocity);

    latestInd = preintegratedForKF.back().second;

    std::cout << "PGBA: Built graph with " << numIMUFactors << " IMU factors " << "and " << numOptimizedPoses << " "
                                                                                                                 "poses."
              << std::endl;

    // Insert priors for TransformationFactor!
    auto factors = getPriorsAndAddValuesForTransform(*transformDSOToIMU, settings.transformPriors, values);
    for(auto&& factor : factors)
    {
        graph.add(factor);
    }
    return values;
}

int dmvio::PoseGraphBundleAdjustment::insertIMUFactorsAndValues(NonlinearFactorGraph& graph,
                                                                const std::deque<std::pair<gtsam::PreintegratedImuMeasurements, int>>& preintegratedMeasurements,
                                                                const Values& imuInputValues,
                                                                Values& values, long long int minConnectedPoseInd,
                                                                imuBias::ConstantBias& imuBias, Vector3& velocity)
{
    int numFactors = 0;
    for(auto&& pair : preintegratedMeasurements)
    {
        assert(pair.second > minConnectedPoseInd);
        int id = pair.second;
        int prevId = prevKFIds.at(id);
        auto&& imuData = pair.first;

        Key prevBiasKey = B(prevId);
        NonlinearFactor::shared_ptr imuFactor(
                new ImuFactor(P(prevId), V(prevId),
                              P(id), V(id), prevBiasKey,
                              imuData));

        // The IMUFactor needs to be transformed (from IMU frame to DSO frame).
        auto transformedFactor = boost::make_shared<PoseTransformationFactor>(imuFactor,
                                                                              *transformDSOToIMU,
                                                                              settings.conversionType);
        graph.add(transformedFactor);

        auto biasNoiseModel = computeBiasNoiseModel(imuCalibration, imuData);
        NonlinearFactor::shared_ptr bias_factor(
                new BetweenFactor<imuBias::ConstantBias>(
                        prevBiasKey, B(id),
                        imuBias::ConstantBias(gtsam::Vector3::Zero(),
                                              gtsam::Vector3::Zero()), biasNoiseModel));
        graph.add(bias_factor);

        if(imuInputValues.exists(V(id)))
        {
            velocity = imuInputValues.at<Vector3>(V(id));
        }
        if(imuInputValues.exists(B(id)))
        {
            imuBias = imuInputValues.at<imuBias::ConstantBias>(B(id));
        }

        values.insert(B(id), imuBias);
        values.insert(V(id), velocity);

        numFactors++;
    }
    return numFactors;
}

long long int dmvio::PoseGraphBundleAdjustment::insertValuesAndGetMinConnectedPoseInd(const Values& poseInputValues,
                                                                                      const Values& poseInputValues2,
                                                                                      Values& values,
                                                                                      const KeyVector& keyVector)
{
    long long minConnectedPoseInd = -1;
    long long lastPoseInd = -1;
    numOptimizedPoses = 0;
    for(auto&& key : keyVector)
    {
        // Use poseInputValues if possible otherwise poseInputValues2
        // The reason is that the newest pose is not yet in the values stored inside DelayedMarginalization.
        auto firstIt = poseInputValues.find(key);
        if(firstIt != poseInputValues.end())
        {
            values.insert(key, firstIt->value);
        }else
        {
            values.insert(key, poseInputValues2.at(key));
        }
        Symbol sym(key);
        if(sym.chr() == 'p')
        {
            long long ind = sym.index();
            if(lastPoseInd == -1)
            {
                minConnectedPoseInd = ind;
            }else
            {
                assert(ind > lastPoseInd);
                bool skipDueToMinIMUFactor = removeIMUFactorsUntil >= 0 && minConnectedPoseInd < removeIMUFactorsUntil;
                if(prevKFIds[ind] != lastPoseInd || skipDueToMinIMUFactor)
                {
                    minConnectedPoseInd = ind;
                }
            }
            lastPoseInd = ind;
            numOptimizedPoses++;
        }
    }
    return minConnectedPoseInd;
}

gtsam::NonlinearFactor::shared_ptr dmvio::compensateNegativeEnergy(NonlinearFactorGraph& graph, const Values& values,
                                                                   const TransformDSOToIMU& transformForFakeFactor)
{
    GaussNewtonOptimizer gnOptim(graph, values);
    double optimError = 0.0;
    try
    {
        GaussNewtonOptimizer optim(graph, values);
        optim.optimize();
        optimError = optim.error();
    }catch(IndeterminantLinearSystemException& exc)
    {
        std::cout << "WARNING: INDETERMINED LINEAR SYSTEM EXCEPTION" << std::endl;

        LevenbergMarquardtOptimizer lmOptim(graph, values);
        lmOptim.iterate();
        optimError = lmOptim.error();
    }
    if(optimError < 0)
    {
        optimError *= 2; // to be sure we add twice this cost...
        Vector bConst(1);
        bConst(0) = sqrt(-optimError * 2.0);
        // GTSAM doesn't like factors with just a constant so we add the scale as well.
        Key scaleKey = S(transformForFakeFactor.getSymbolInd());
        JacobianFactor::shared_ptr constantFactor(new JacobianFactor(scaleKey, gtsam::Matrix11::Zero(), bConst));
        Values linPoint;
        linPoint.insert(scaleKey, ScaleGTSAM(transformForFakeFactor.getScale()));
        auto lcf = boost::make_shared<LinearContainerFactor>(constantFactor, linPoint);
        return lcf;
    }
    return nullptr;
}

void PoseGraphBundleAdjustment::addKeyframe(int id, gtsam::PreintegratedImuMeasurements imuMeasurements)
{
    if(skipped < settings.skipFirstKFs)
    {
        lastKFId = id;
        skipped++;
        return;
    }
    preintegratedForKF.emplace_back(std::move(imuMeasurements), id);
    prevKFIds[id] = lastKFId;
    lastKFId = id;
}

void PoseGraphBundleAdjustment::updateGraphMarginalizationOrder()
{
    auto optSymbols = this->transformDSOToIMU->getAllOptimizedSymbols();
    gtsam::FastSet<gtsam::Key> dontMargKeys;
    dontMargKeys.insert(optSymbols.begin(), optSymbols.end());

    std::deque<gtsam::FastVector<gtsam::Key>>& marginalizationOrder = this->delayedGraph->marginalizationOrder;
    std::set<gtsam::Key> keySet;
    // Add IMU variables to keys to marginalize.
    for(auto& keyVector : marginalizationOrder)
    {
        keySet.clear();
        keySet.insert(keyVector.begin(), keyVector.end());

        // Make sure that we don't marginalize symbols optimized by the current transform!
        keyVector.erase(std::remove_if(keyVector.begin(), keyVector.end(), [&dontMargKeys](const gtsam::Key& key)
        { return dontMargKeys.exists(key); }), keyVector.end());

        for(int i = 0; i < keyVector.size(); ++i)
        {
            gtsam::Symbol sym(keyVector[i]);
            int ind = sym.index();
            if(sym.chr() == 'p' && ind >= this->firstId)
            {
                // Add the IMU variables as well for all poses.
                if(keySet.find(V(ind)) == keySet.end())
                {
                    keyVector.push_back(V(ind));
                }
                if(keySet.find(B(ind)) == keySet.end())
                {
                    keyVector.push_back(B(ind));
                }
            }
        }
    }
}

// Prepare for prepareGraphForMainOptimization (usually called in a separate thread)
void PoseGraphBundleAdjustment::preparePreparation(const gtsam::Values& optimizedValues)
{
    dmvio::TimeMeasurement fullMeas("PreparePreparation");
    // Remove active DSO factor again!
    removeDSOFactorIfNeeded();

    mainPreparation(optimizedValues);
}

void PoseGraphBundleAdjustment::mainPreparation(const gtsam::Values& optimizedValues)
{
    // Update Delayed Graph
    updateGraphMarginalizationOrder();

    // update delayedValues
    gtsam::Values& delayedValues = delayedGraph->delayedValues;
    for(auto&& pair : optimizedValues)
    {
        auto chr = gtsam::Symbol(pair.key).chr();
        if(chr != 'a' &&
           chr != 'p') // We don't overwrite DSO variables, as the evaluation point values shall be used for them.
        {
            eraseAndInsert(delayedValues, pair.key, pair.value);
        }
    }

    // Replace values with optimized values!
    delayedGraph->delayedCurrValues = optimizedValues;

    // Change to delay 0.
    {
        dmvio::TimeMeasurement meas("GraphUnrolling");
        delayedGraph->readvanceGraph(0);
    }
}

void PoseGraphBundleAdjustment::removeDSOFactorIfNeeded()
{
    if(dsoFactorPos < 0) return;
    graph->erase(graph->begin() + dsoFactorPos);
    dsoFactorPos = -1;
}

std::unique_ptr<DelayedGraph>
PoseGraphBundleAdjustment::prepareGraphForMainOptimization(const gtsam::Values& optimizedValues)
{
    dmvio::TimeMeasurement fullMeas("PrepareGraphForMainOptimization");
    // Remove active DSO factor again!
    removeDSOFactorIfNeeded();

    // These should just be the factors arising from one call to addMarginalizedPointsBA
    if(settings.prepareGraphAddFactors)
    {
        for(auto&& factor : disconnectedGraph->addedFactors)
        {
            delayedGraph->graph->add(factor);
        }
        disconnectedGraph->addedFactors.clear();
    }

    // delayedGraph->delayedCurrValues will be replaced with optimizedValues later on (in mainPreparation).
    // Non-DSO variables in delayedGraph->delayedValues will be replaced with optimizedValues there as well.
    // This means that we need to add the new DSO delayedValues here.
    if(settings.prepareGraphAddDelValues)
    {
        for(auto&& val : disconnectedGraph->delayedValues)
        {
            if(!delayedGraph->delayedValues.exists(val.key))
            {
                delayedGraph->delayedValues.insert(val.key, val.value);
            }
        }
    }

    delayedGraph->marginalizationOrder.insert(delayedGraph->marginalizationOrder.end(),
                                              disconnectedGraph->marginalizationOrder.begin(),
                                              disconnectedGraph->marginalizationOrder.end());
    // This method must be called from within the BA thread, otherwise a mutex would have to be added here.
    delayedMarginalization->removeDisconnectedGraph(disconnectedGraph.get());
    disconnectedGraph.reset();

    mainPreparation(optimizedValues);

    return std::move(delayedGraph);
}

int PoseGraphBundleAdjustment::getNumKFs()
{
    return preintegratedForKF.size();
}

gtsam::Key PoseGraphBundleAdjustment::getBiasKey()
{
    return B(latestInd);
}

int PoseGraphBundleAdjustment::getNumOptimizedPoses() const
{
    return numOptimizedPoses;
}

int PoseGraphBundleAdjustment::getNumImuFactors() const
{
    return numIMUFactors;
}

int PoseGraphBundleAdjustment::getLatestInd() const
{
    return latestInd;
}

std::shared_ptr<DelayedGraph> PoseGraphBundleAdjustment::getInputDelayedGraph() const
{
    return inputDelayedGraph;
}

void PoseGraphBundleAdjustment::optimizationResultNotUsed()
{
    delayedMarginalization->removeDisconnectedGraph(disconnectedGraph.get());
    disconnectedGraph.reset();
}

bool PoseGraphBundleAdjustment::isAllPosesUsed() const
{
    return allPosesUsed;
}

int PoseGraphBundleAdjustment::getFirstIdWithIMUData() const
{
    return firstId;
}

int PoseGraphBundleAdjustment::getIdOfNthIMUData(int n) const
{
    auto&& keys = graph->keyVector();
    int num = 0;
    int lastInd = -1;
    for(auto&& key : keys)
    {
        gtsam::Symbol sym(key);
        if(sym.chr() == 'v')
        {
            if(num == n) return sym.index();
            lastInd = sym.index();
            num++;
        }
    }
    assert(lastInd != -1);
    return lastInd + (n - num) * 5;
}
