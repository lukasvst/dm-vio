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

#ifndef DMVIO_POSEGRAPHBUNDLEADJUSTMENT_H
#define DMVIO_POSEGRAPHBUNDLEADJUSTMENT_H


#include <GTSAMIntegration/DelayedMarginalization.h>
#include "IMU/BAIMULogic.h"
#include <gtsam/nonlinear/Marginals.h>

namespace dmvio
{


// PoseGraphBundleAdjustment (PGBA) as described in the paper.
class PoseGraphBundleAdjustment
{
public:
    PoseGraphBundleAdjustment(DelayedMarginalizationGraphs* delayedMarginalization,
                              const IMUCalibration& imuCalibration, const PGBASettings& settings,
                              std::shared_ptr<TransformDSOToIMU> transformDSOToIMU);


    using AddKeyframeData = std::pair<gtsam::PreintegratedImuMeasurements, int>;
    using KeyframeDataContainer = std::deque<AddKeyframeData>;

    // -------------------- Main methods which have to be called

    // Should be called for every keyframe added, providing the corresponding IMU measurements.
    void addKeyframe(int id, gtsam::PreintegratedImuMeasurements imuMeasurements);

    // This must be called before calling optimize!
    // This method must be called from the BA thread (between optimization and marginalization), wheres
    // optimize can be called later in a different thread.
    void prepareOptimization();

    // Adds IMU factors to the delayed factor graph and then optimizes it (unless noOptimization==true).
    // After calling optimize either optimizationResultNotUsed must be called or prepareGraphForMainOptimization.
    // if noOptimization is true the graph is only built, but no optimization is performed (useful e.g. for
    // marginalization replacement).
    gtsam::Values optimize(gtsam::NonlinearFactor::shared_ptr activeDSOFactor,
                           const gtsam::Values& baValues,
                           const gtsam::Values& imuInitValues, bool noOptimization); // <-- called by IMUInitializer

    // Notifies that prepareGraphForMainOptimization will **not** be called for this optimization result.
    void optimizationResultNotUsed();

    // This method can optionally be called (usually in a separate thread) before prepareGraphForMainOptimization.
    void preparePreparation(const gtsam::Values& optimizedValues);

    // If optimization has been performed in a separate thread this method must be called before prepareGraphForMainOptimization.
    // It adds the cached IMU data which was added during optimization to the graph, and potentially performs another,
    // smaller optimization to get biases and velocities for the newly added variables.
    // This method must be called from the main BA thread.
    gtsam::Values
    extendGraph(gtsam::NonlinearFactor::shared_ptr activeDSOFactor, gtsam::Values&& previouslyOptimizedValues,
                const gtsam::Values& baValues, const KeyframeDataContainer& cachedData);

    // This method must be called from the main BA thread.
    // It unrolls the delayed graph, so that it can be used in the main optimization.
    std::unique_ptr<DelayedGraph> prepareGraphForMainOptimization(const gtsam::Values& optimizedValues);

    gtsam::Marginals getMarginals(const gtsam::Values& values);

    gtsam::Key getBiasKey();

    int getNumKFs();

    int getNumOptimizedPoses() const;

    int getNumImuFactors() const;

    int getLatestInd() const;

    // returns id of the nth pose that has IMU data.
    int getIdOfNthIMUData(int n) const;

    int getFirstIdWithIMUData() const;

    std::shared_ptr<DelayedGraph> getInputDelayedGraph() const;

    bool isAllPosesUsed() const;

    int removeIMUFactorsUntil = -1;
private:
    gtsam::Values buildGraph(gtsam::NonlinearFactorGraph& graph, const gtsam::Values& poseInputValues,
                             const gtsam::Values& poseInputValues2, const gtsam::Values& imuInputValues,
                             bool noOptimization); // builds IMU graph for all factors connected to the latest KF in the delayedGraph.

    DelayedMarginalizationGraphs* delayedMarginalization;

    // Note that it needs to add IMUFactors using the TransformationFactor!
    std::shared_ptr<DelayedGraph> inputDelayedGraph; // this also specifies the delay of the larger optimization.

    // This graph is cloned
    std::unique_ptr<DelayedGraph> delayedGraph;
    gtsam::NonlinearFactorGraph* graph = nullptr; // points to the graph of delayedGraph
    std::shared_ptr<DisconnectedDelayedGraph> disconnectedGraph;


    const IMUCalibration& imuCalibration;
    const PGBASettings& settings;

    std::shared_ptr<TransformDSOToIMU> transformDSOToIMU;

    // stores preintegrated, keyframeId. filled forward, read backwards.
    KeyframeDataContainer preintegratedForKF;
    std::map<int, int> prevKFIds; // for each kf it stores the id of the previous KF.
    int lastKFId = 0; // We use the fact that the very first KF always has index 0.

    int numOptimizedPoses = -1;
    int numIMUFactors = -1;
    int latestInd = -1;
    int firstId = -1; // Id of first pose that is connected to IMU factors.
    int dsoFactorPos = -1;

    int skipped = 0;

    bool allPosesUsed = false;

    // Add IMU variables to keys to marginalize.
    void updateGraphMarginalizationOrder();

    void removeDSOFactorIfNeeded();

    void mainPreparation(const gtsam::Values& optimizedValues);

    long long int
    insertValuesAndGetMinConnectedPoseInd(const gtsam::Values& poseInputValues, const gtsam::Values& poseInputValues2,
                                          gtsam::Values& values, const gtsam::KeyVector& keyVector);

    int insertIMUFactorsAndValues(gtsam::NonlinearFactorGraph& graph,
                                  const std::deque<std::pair<gtsam::PreintegratedImuMeasurements, int>>& preintegratedMeasurements,
                                  const gtsam::Values& imuInputValues,
                                  gtsam::Values& values, long long int minConnectedPoseInd,
                                  gtsam::imuBias::ConstantBias& imuBias, gtsam::Vector3& velocity);

};

gtsam::NonlinearFactor::shared_ptr
compensateNegativeEnergy(gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                         const TransformDSOToIMU& transformForFakeFactor);

}


#endif //DMVIO_POSEGRAPHBUNDLEADJUSTMENT_H
