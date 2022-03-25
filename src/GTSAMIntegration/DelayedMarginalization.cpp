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

#include "DelayedMarginalization.h"
#include "Marginalization.h"
#include "util/TimeMeasurement.h"
#include "GTSAMUtils.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/GaussianFactorGraph.h>

using namespace dmvio;

DelayedMarginalizationGraphs::DelayedMarginalizationGraphs(int mainGraphDelay, int maxGroupInMainGraph)
{
    addMainGraph(std::make_shared<DelayedGraph>(mainGraphDelay, maxGroupInMainGraph));
}

std::shared_ptr<DelayedGraph> dmvio::DelayedMarginalizationGraphs::addDelayedGraph(int delayN, int maxGroupInGraph)
{
    delayedGraphs.emplace_back(new DelayedGraph(delayN, maxGroupInGraph));
    return delayedGraphs.back();
}

void DelayedMarginalizationGraphs::addDelayedGraph(std::shared_ptr<DelayedGraph> graph)
{
    delayedGraphs.emplace_back(std::move(graph));
}

void dmvio::DelayedMarginalizationGraphs::addFactor(gtsam::NonlinearFactor::shared_ptr factor, int group)
{
    for(auto&& graph : delayedGraphs)
    {
        graph->addFactor(factor, group);
    }
    for(auto&& graph : disconnectedGraphs)
    {
        graph->addFactor(factor, group);
    }
}

std::pair<gtsam::Matrix, gtsam::Vector>
dmvio::DelayedMarginalizationGraphs::getHAndB(const gtsam::Values& values, const gtsam::Ordering& ordering,
                                              const std::map<gtsam::Key, size_t>& keyDimMap,
                                              gtsam::Ordering* fillAdditionalKeys)
{
    auto graph = getMainGraph()->getGraph();

    // Make sure that the GTSAM factors use the FEJValues.
    getMainGraph()->setFEJValuesForFactors(true);
    // Lienarize the graph
    gtsam::GaussianFactorGraph::shared_ptr gfg = graph->linearize(values);
    getMainGraph()->setFEJValuesForFactors(false);

    // Compute the Hessian and gradient vector. We use the AugmentedScatter which also works for keys which don't exist in the graph.
    AugmentedScatter scatter(*gfg, ordering, keyDimMap);
    if(fillAdditionalKeys)
    {
        fillAdditionalKeysFromScatter(ordering, *fillAdditionalKeys, scatter);
    }
    return scatter.computeHessian(*gfg);
}

void dmvio::DelayedMarginalizationGraphs::marginalizeFrame(const gtsam::FastVector<gtsam::Key>& keysToMarginalize,
                                                           gtsam::Values::shared_ptr values,
                                                           std::map<gtsam::Key, size_t>& keyDimMap,
                                                           double currBATimestamp,
                                                           gtsam::Values::shared_ptr currValues)
{
    dmvio::TimeMeasurement meas("DelayedMarginalization");

    // First only marginalize the main graph so that we have separate time measurements.
    dmvio::TimeMeasurement mainMeas(
            "MarginalizeMainGraph"); // This saves only the time necessary for marginalizing the main graph.
    auto* mainGraph = getMainGraph().get();
    mainGraph->marginalize(keysToMarginalize, values, currValues);
    mainMeas.end();

    dmvio::TimeMeasurement measDelayed("DelayedMarginalizationOnly");
    for(auto&& graph : delayedGraphs)
    {
        if(graph.get() == mainGraph) continue;
        graph->marginalize(keysToMarginalize, values, currValues);
    }
    for(auto&& graph : disconnectedGraphs)
    {
        graph->marginalize(keysToMarginalize, values, currValues);
    }
    meas.end();
}

double dmvio::DelayedMarginalizationGraphs::getError(const gtsam::Values& values)
{
    return getMainGraph()->getGraph()->error(values);
}

void DelayedMarginalizationGraphs::addMainGraph(std::shared_ptr<DelayedGraph> delayedGraph)
{
    mainGraphInd = delayedGraphs.size();
    delayedGraphs.emplace_back(std::move(delayedGraph));
    for(auto&& callback : mainGraphCallbacks)
    {
        callback(delayedGraph);
    }
}

void DelayedMarginalizationGraphs::replaceMainGraph(std::shared_ptr<DelayedGraph> delayedGraph)
{
    delayedGraphs[mainGraphInd] = std::move(delayedGraph);
    for(auto&& callback : mainGraphCallbacks)
    {
        callback(delayedGraphs[mainGraphInd]);
    }
}

std::shared_ptr<DelayedGraph> DelayedMarginalizationGraphs::getMainGraph()
{
    return delayedGraphs[mainGraphInd];
}

void DelayedMarginalizationGraphs::removeDelayedGraph(const DelayedGraph* graph)
{
    auto it = std::find_if(delayedGraphs.begin(), delayedGraphs.end(),
                           [graph](const std::shared_ptr<DelayedGraph>& comp)
                           { return comp.get() == graph; });
    delayedGraphs.erase(it);
}

void DelayedMarginalizationGraphs::registerMainGraphReplacementCallback(
        DelayedMarginalizationGraphs::GraphReplacementCallback callback)
{
    mainGraphCallbacks.push_back(std::move(callback));
}

void DelayedMarginalizationGraphs::updateEvalValues(const gtsam::Values& evalValues)
{
    for(auto&& graph : delayedGraphs)
    {
        graph->fejValues->insertConnectedKeys(gtsam::Ordering(), evalValues);
    }
}

std::shared_ptr<DisconnectedDelayedGraph>
DelayedMarginalizationGraphs::addDisconnectedGraph(int maxGroupInGraph)
{
    disconnectedGraphs.emplace_back(new DisconnectedDelayedGraph(maxGroupInGraph));
    return disconnectedGraphs.back();
}

void DelayedMarginalizationGraphs::removeDisconnectedGraph(const DisconnectedDelayedGraph* graph)
{
    auto it = std::find_if(disconnectedGraphs.begin(), disconnectedGraphs.end(),
                           [graph](const std::shared_ptr<DisconnectedDelayedGraph>& comp)
                           { return comp.get() == graph; });
    disconnectedGraphs.erase(it);
}

dmvio::DelayedGraph::DelayedGraph(int delayN, int maxGroupInGraph) : delayN(delayN), maxGroupInGraph(maxGroupInGraph)
{
    graph.reset(new gtsam::NonlinearFactorGraph());
    fejValues.reset(new FEJValues);
}

void dmvio::DelayedGraph::addFactor(gtsam::NonlinearFactor::shared_ptr factor, int group)
{
    if(group <= maxGroupInGraph)
    {
        graph->add(factor);
    }
}

void
dmvio::DelayedGraph::marginalize(const gtsam::FastVector<gtsam::Key>& keysToMarginalize,
                                 gtsam::Values::shared_ptr values, gtsam::Values::shared_ptr currValues)
{
    marginalizationOrder.push_back(keysToMarginalize);

    // update values
    for(auto&& val : *values)
    {
        eraseAndInsert(delayedValues, val.key, val.value);
    }
    if(currValues)
    {
        for(auto&& val : *currValues)
        {
            eraseAndInsert(delayedCurrValues, val.key, val.value);
        }
    }

    readvanceUntilDelay();
}

void dmvio::DelayedGraph::readvanceUntilDelay()
{
    if(marginalizationPaused) return;
    while(marginalizationOrder.size() > delayN)
    {
        auto&& keysToMarg = marginalizationOrder.front();

        // Insert values for keysToMarginalize, even though they will be removed below, but they are necessary
        // so that all fejValues contains all values needed for the linearization below.
        fejValues->insertConnectedKeys(keysToMarg, delayedValues);

        // The connected keys have to be inserted into the fejMap before the linearization inside marginalizeOut.
        auto connectedKeyCallback = [this](const gtsam::FastSet<gtsam::Key>& connectedKeys)
        {
            // Get connected keys and insert fejMap.
            // delayedValues should be baEvalValues, i.e. evaluation point values for poses and affine, and the current values for everything else.
            fejValues->insertConnectedKeys(connectedKeys, delayedValues);
        };

        if(!keysToMarg.empty())
        {
            // Note that the connectedKeyCallback fills fejValues.fejValues with the needed values before the linearization happens.
            graph = marginalizeOut(*graph, fejValues->fejValues, keysToMarg, connectedKeyCallback);

            for(auto&& key : keysToMarg)
            {
                if(delayedCurrValues.exists(key))
                {
                    delayedCurrValues.erase(key);
                }
                if(delayedValues.exists(key))
                {
                    delayedValues.erase(key);
                }
            }

            fejValues->keysRemoved(keysToMarg);
        }

        marginalizationOrder.pop_front();
    }
}

void DelayedGraph::readvanceGraph(int newDelay)
{
    assert(newDelay <= delayN);
    delayN = newDelay;
    if(marginalizationPaused)
    {
        std::cout << "WARNING: Trying to readvance graph while marginalization is paused!" << std::endl;
    }
    readvanceUntilDelay();
}

int DelayedGraph::getDelayN() const
{
    return delayN;
}

gtsam::NonlinearFactorGraph::shared_ptr DelayedGraph::getGraph() const
{
    return graph;
}

const gtsam::Values& DelayedGraph::getDelayedValues() const
{
    return delayedValues;
}

const gtsam::Values& DelayedGraph::getDelayedCurrValues() const
{
    return delayedCurrValues;
}

int DelayedGraph::getMaxGroupInGraph() const
{
    return maxGroupInGraph;
}

const std::deque<gtsam::FastVector<gtsam::Key>>& DelayedGraph::getMarginalizationOrder() const
{
    return marginalizationOrder;
}

DelayedGraph::DelayedGraph(int delayN, int maxGroupInGraph, const gtsam::NonlinearFactorGraph::shared_ptr& graph,
                           std::deque<gtsam::FastVector<gtsam::Key>> marginalizationOrder,
                           gtsam::Values delayedValues, gtsam::Values delayedCurrValues,
                           std::shared_ptr<FEJValues> fejValuesPassed)
        : delayN(delayN), maxGroupInGraph(maxGroupInGraph), graph(graph),
          marginalizationOrder(std::move(marginalizationOrder)), delayedValues(std::move(delayedValues)),
          delayedCurrValues(std::move(delayedCurrValues))
{
    fejValues = std::move(fejValuesPassed);
}

DelayedGraph::DelayedGraph(const DelayedGraph& other)
        : delayN(other.delayN), maxGroupInGraph(other.maxGroupInGraph),
          graph(boost::make_shared<gtsam::NonlinearFactorGraph>(other.graph->clone())),
          marginalizationOrder(other.marginalizationOrder), delayedValues(other.delayedValues),
          delayedCurrValues(other.delayedCurrValues), fejValues(new FEJValues(*(other.fejValues)))
{}

void DelayedGraph::setMarginalizationPaused(bool marginalizationPausedPassed)
{
    marginalizationPaused = marginalizationPausedPassed;
    if(!marginalizationPaused)
    {
        readvanceUntilDelay();
    }
}

void DelayedGraph::setMaxGroupInGraph(int maxGroupInGraph)
{
    DelayedGraph::maxGroupInGraph = maxGroupInGraph;
}

void DelayedGraph::setDelayN(int delayN)
{
    DelayedGraph::delayN = delayN;
}

void DelayedGraph::setFEJValuesForFactors(bool useFEJ)
{
    setFEJMapForGraph(*graph, useFEJ ? fejValues : nullptr);
}

DisconnectedDelayedGraph::DisconnectedDelayedGraph(int maxGroupInGraph) : maxGroupInGraph(maxGroupInGraph)
{}

void DisconnectedDelayedGraph::addFactor(gtsam::NonlinearFactor::shared_ptr factor, int group)
{
    if(group <= maxGroupInGraph)
    {
        addedFactors.push_back(factor);
    }
}

void DisconnectedDelayedGraph::marginalize(const gtsam::FastVector<gtsam::Key>& keysToMarginalize,
                                           gtsam::Values::shared_ptr values, gtsam::Values::shared_ptr currValues)
{

    marginalizationOrder.push_back(keysToMarginalize);

    // Also save values
    for(auto&& val : *values)
    {
        eraseAndInsert(delayedValues, val.key, val.value);
    }
    if(currValues)
    {
        for(auto&& val : *currValues)
        {
            eraseAndInsert(delayedCurrValues, val.key, val.value);
        }
    }
}
