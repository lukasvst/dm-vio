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

#ifndef DMVIO_DELAYEDMARGINALIZATION_H
#define DMVIO_DELAYEDMARGINALIZATION_H

#include "GTSAMIntegration/BAGTSAMIntegration.h"
#include "GTSAMIntegration/PoseTransformation.h"
#include "GTSAMIntegration/FEJValues.h"

namespace dmvio
{

// This is a delayed graph. It has a delay (can also be 0) and maxGroupInGraph which defines which factors are added to it.
// It is usually managed by DelayedMarginalizationGraphs.
class DelayedGraph
{
public:
    // Constructor taking the delay and the maximum factor group which will be added to the graph.
    DelayedGraph(int delayN, int maxGroupInGraph);

    DelayedGraph(int delayN, int maxGroupInGraph, const gtsam::NonlinearFactorGraph::shared_ptr& graph,
                 std::deque<gtsam::FastVector<gtsam::Key>> marginalizationOrder,
                 gtsam::Values delayedValues, gtsam::Values delayedCurrValues, std::shared_ptr<FEJValues> fejValues);

    DelayedGraph(const DelayedGraph& other); // copy constructor.

    // changes the delay, but doesn't readvance.
    void setDelayN(int delayN);

    // readvance graph to have a smaller delay now.
    void readvanceGraph(int newDelay);

    // add factor if group <= maxGroupInGraph.
    void addFactor(gtsam::NonlinearFactor::shared_ptr factor, int group);

    void marginalize(const gtsam::FastVector<gtsam::Key>& keysToMarginalize, gtsam::Values::shared_ptr values,
                     gtsam::Values::shared_ptr currValues);

    void setMaxGroupInGraph(int maxGroupInGraph);

    void setMarginalizationPaused(bool marginalizationPausedPassed);

    int getDelayN() const;
    gtsam::NonlinearFactorGraph::shared_ptr getGraph() const;
    const gtsam::Values& getDelayedValues() const;
    const gtsam::Values& getDelayedCurrValues() const;
    int getMaxGroupInGraph() const;
    const std::deque<gtsam::FastVector<gtsam::Key>>& getMarginalizationOrder() const;

    std::shared_ptr<FEJValues> fejValues; // These contain FEJValues for all variables connected to a marginalization factor (including all DSO poses).

    void setFEJValuesForFactors(bool useFEJ);

    friend class PoseGraphBundleAdjustment;
protected:
    // perform marginalization until we match the wanted delay.
    void readvanceUntilDelay();

    int delayN;

    // Only add factors with group <= maxGroupInGraph
    int maxGroupInGraph;
    bool marginalizationPaused = false; // While true, no readvancing is performed (even if the delay becomes larger than delayN).

    gtsam::NonlinearFactorGraph::shared_ptr graph;

    std::deque<gtsam::FastVector<gtsam::Key>> marginalizationOrder;

    // delayedValues contain the baEvalValues, meaning FEJValues for DSO variables, and current values for all other variables.
    // delayedCurrValues contain baValues, meaning current values for all variables.
    gtsam::Values delayedValues, delayedCurrValues; // contain the delayed values (including new keys).
};

// This is a delayed graph which just saves the factors and marginalization commands in the right order.
// It can be used to "reconnect" a DelayedGraph which has not been updated for a while.
// Used e.g. for the realtime version of the PGBA (which has to run in a separate thread decoupled from the main BA).
class DisconnectedDelayedGraph
{
public:
    explicit DisconnectedDelayedGraph(int maxGroupInGraph);

    void addFactor(gtsam::NonlinearFactor::shared_ptr factor, int group);
    void marginalize(const gtsam::FastVector<gtsam::Key>& keysToMarginalize, gtsam::Values::shared_ptr values,
                     gtsam::Values::shared_ptr currValues);

    std::deque<gtsam::FastVector<gtsam::Key>> marginalizationOrder;
    std::vector<gtsam::NonlinearFactor::shared_ptr> addedFactors;
    gtsam::Values delayedValues, delayedCurrValues;

    int maxGroupInGraph;
};


// Main class responsible for the DelayedMarginalization.
class DelayedMarginalizationGraphs : public BAGraphs
{
public:
    // Constructor, pass arguments for the main DelayedGraph (usually has delay 0).
    DelayedMarginalizationGraphs(int mainGraphDelay, int maxGroupInMainGraph);

    // Should usually be called before operation starts.
    // Returns shared_ptr to the created graph.
    std::shared_ptr<DelayedGraph> addDelayedGraph(int delayN, int maxGroupInGraph);
    void addDelayedGraph(std::shared_ptr<DelayedGraph> graph);
    void removeDelayedGraph(const DelayedGraph* graph);

    std::shared_ptr<DisconnectedDelayedGraph> addDisconnectedGraph(int maxGroupInGraph);
    void removeDisconnectedGraph(const DisconnectedDelayedGraph* graph);

    // Add a new graph which becomes the main graph.
    void addMainGraph(std::shared_ptr<DelayedGraph> delayedGraph);
    // like addMainGraph, but also delete old main graph. delayedGraph must not be in the delayedGraphs yet!
    void replaceMainGraph(std::shared_ptr<DelayedGraph> delayedGraph);

    std::shared_ptr<DelayedGraph> getMainGraph();

    using GraphReplacementCallback = std::function<void(const std::shared_ptr<DelayedGraph>& graph)>;
    // Register a callback which will be called when the main graph is replaced.
    void registerMainGraphReplacementCallback(GraphReplacementCallback callback);

    // Override from BAGraphs.
    // --------------------------------------------------
    void addFactor(gtsam::NonlinearFactor::shared_ptr factor, int group) override;
    std::pair<gtsam::Matrix, gtsam::Vector> getHAndB(const gtsam::Values& values, const gtsam::Ordering& ordering,
                                                     const std::map<gtsam::Key, size_t>& keyDimMap,
                                                     gtsam::Ordering* fillAdditionalKeys) override;
    void marginalizeFrame(const gtsam::FastVector<gtsam::Key>& keysToMarginalize, gtsam::Values::shared_ptr values,
                          std::map<gtsam::Key, size_t>& keyDimMap, double currBATimestamp,
                          gtsam::Values::shared_ptr currValues) override;
    double getError(const gtsam::Values& values) override;

    void updateEvalValues(const gtsam::Values& evalValues) override;

private:
    // Delayed graphs to use. (doesn't contain main graph).
    std::vector<std::shared_ptr<DelayedGraph>> delayedGraphs;
    // disconnected delayed graphs.
    std::vector<std::shared_ptr<DisconnectedDelayedGraph>> disconnectedGraphs;
    int mainGraphInd = -1;

    std::vector<GraphReplacementCallback> mainGraphCallbacks;

};

}

#endif //DMVIO_DELAYEDMARGINALIZATION_H
