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

#ifndef DMVIO_BAGTSAMINTEGRATION_H
#define DMVIO_BAGTSAMINTEGRATION_H


#include <vector>
#include <OptimizationBackend/EnergyFunctionalStructs.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/geometry/Pose3.h>
#include <fstream>

#include "PoseTransformation.h"
#include "AugmentedScatter.hpp"


// This source file, and in particular the class BAGTSAMIntegration is responsible for integrating the Bundle Adjustment for DSO into GTSAM.
// While the photometric factors are still computed using the original DSO code, new factors can easily be added
// as standard GTSAM factors, as long as they do not depend on points depths.
// This works by registering a BAExtension, which can add new factors and keys.
namespace dmvio
{

typedef std::map<gtsam::Key, size_t> KeyDimMap;

class BAGraphs;

// Interface for an extension to the Bundle Adjustment of DSO.
// It can add additional keys and factors which will be organized.
class BAExtension
{
public:

    virtual ~BAExtension() = default;

    // Called when the first keyframe is added.
    virtual void addFirstBAFrame(int keyframeId, BAGraphs* baGraphs, gtsam::Values::shared_ptr baValues) = 0;
    // Called when a keyframe (except the first one) is added.
    // The method is supposed to add its factors to baGraphs, and new variables into baValues.
    virtual void addKeyframe(BAGraphs* baGraphs, gtsam::Values::shared_ptr baValues, int keyframeId,
                             const Sophus::SE3d& keyframePose, std::vector<dso::EFFrame*>& frames) = 0;

    // The extension can update the passed ordering with more variables.
    virtual void
    updateBAOrdering(std::vector<dso::EFFrame*>& frames, gtsam::Ordering* ordering, KeyDimMap& baDimMap) = 0;

    // Called when a KF gets marginalized. The method is supposed to add keys which shall be marginalized.
    virtual void addKeysToMarginalize(int fullId, gtsam::FastVector<gtsam::Key>& keysToMarginalize) = 0;

    // These methods are called before and after solving the linear system in computeBAUpdate.
    virtual void preSolve(gtsam::Matrix& HFull, gtsam::Vector& bFull, int dimensionDSOH) = 0;
    // called after solve the system. Should return true if the optimization can break.
    virtual bool
    postSolve(gtsam::Values::shared_ptr values, gtsam::Values::shared_ptr newValues, const gtsam::Vector& inc,
              const gtsam::Ordering& ordering, const KeyDimMap& baDimMap) = 0;

    // Called when the BA update has been accepted and values will be replaced with newValues shortly.
    virtual void acceptUpdate(gtsam::Values::shared_ptr values, gtsam::Values::shared_ptr newValues) = 0;

};

// Interface for either a gtsam::NonlinearFactorGraph or a collection of them.
// This exists to abstract techniques like Delayed Marginalization or Dynamic Marginalization from the BAExtension.
// Currently the only implementation is DelayedMarginalizationGraphs.
class BAGraphs
{
public:
    virtual ~BAGraphs() = default;

    // Adds factor to the right factor graphs.
    // Factors can have different groups, which can later be used to perform operations only on specific groups.
    virtual void addFactor(gtsam::NonlinearFactor::shared_ptr factor, int group) = 0;

    // Return H and b (Hessian and gradient) at a linearization point defined by values.
    virtual std::pair<gtsam::Matrix, gtsam::Vector>
    getHAndB(const gtsam::Values& values, const gtsam::Ordering& ordering,
             const std::map<gtsam::Key, size_t>& keyDimMap, gtsam::Ordering* fillAdditionalKeys) = 0;

    // Marginalize the given keys from the factor graph.
    virtual void
    marginalizeFrame(const gtsam::FastVector<gtsam::Key>& keysToMarginalize, gtsam::Values::shared_ptr values,
                     std::map<gtsam::Key, size_t>& keyDimMap, double currBATimestamp,
                     gtsam::Values::shared_ptr currValues) = 0;

    virtual double getError(const gtsam::Values& values) = 0;

    // Called when the evalValues (given by DSO) are updated.
    virtual void updateEvalValues(const gtsam::Values& evalValues)
    {}

private:
    gtsam::NonlinearFactorGraph::shared_ptr graph;
};


// Settings for the GTSAM Integration.
class GTSAMIntegrationSettings
{
public:
    // All these settings are typically overwritten by IMUSettings member. Therefore not registered as arg.
    double weightDSOToGTSAM = 1.0;
};

// This class interacts with the DSO Bundle Adjustment and integrates GTSAM into it.
// This works by intercepting the DSO Hessian and gradient vector right before the update is computed.
// Then the Hessian and gradient vector computed from the GTSAM factors is added.
// Special care is taken of the different conventions between DSO and GTSAM (left vs right sided update, different
// pose types), and on making the keyframe marginalization works (which is taken over by GTSAM now).
// This way the GTSAM integration can be decoupled from the Bundle-Adjustment code of DSO, mathematically the result
// is equal to a joint optimization of all variables, as long as no GTSAM factor depends on the point depths.
class BAGTSAMIntegration
{
public:
    // Will take ownership of passed unique_ptrs.
    BAGTSAMIntegration(std::unique_ptr<BAGraphs> baGraphs, std::unique_ptr<PoseTransformation> transformationDSOToBA,
                       const GTSAMIntegrationSettings& integrationSettings, dso::CalibHessian* HCalib);

    // Add an extension which can add new factors and keys to the optimization.
    void addExtension(std::shared_ptr<BAExtension> extension);

    // In order to set a dynamic weight for the DSO variables a DynamicWeightCallback can be registered, which
    // should the new dynamic weight part.
    using DynamicWeightCallback = std::function<double(double lastDSOEnergy, double lastRMSE,
                                                       bool coarseTrackingWasGood)>;
    void setDynamicDSOWeightCallback(DynamicWeightCallback callback);

    // Get a factor which contains the active DSO part.
    // Must only be called between an optimization iteration and the next marginalization (as otherwise the ordering does not match the matrices).
    gtsam::NonlinearFactor::shared_ptr getActiveDSOFactor();

    void removeExtension(BAExtension* extension);


    // Getters
    // --------------------------------------------------
    double getCurrBaTimestamp() const;

    // There are both, baValues and baEvalValues. The former are the current values obtained during the optimization,
    // and the latter use the First Estimates values **for DSO variables only** (i.e. poses and affine brightness).
    // FEJ values for all other variables are handled by the class FEJValues.
    const gtsam::Values* getBaValues() const;
    gtsam::Values* getMutableValues();
    const gtsam::Values* getBaEvalValues() const;
    BAGraphs& getBaGraphs();
    const std::map<gtsam::Key, size_t>& getBaDimMap() const;
    const gtsam::Ordering& getBaOrdering() const;
    const gtsam::Ordering& getBaOrderingSmall() const;

    // --------------------------------------------------
    // Most other methods will be called by DSO code, and except for the getters they should not be called from an extension.
    // --------------------------------------------------

    void addFirstBAFrame(int keyframeId);

    void addKeyframeToBA(int keyframeId, const Sophus::SE3d& keyframePose, std::vector<dso::EFFrame*>& frames);

    // called from addKeyframeToBA and from outside (FullSystem::makeKeyFrame)
    void updateBAOrdering(std::vector<dso::EFFrame*>& frames);

    // updates the baValues with the data from DSO.
    void updateBAValues(std::vector<dso::EFFrame*>& frames);

    // inserts the data at the evaluation points from DSO into values.
    void computeEvaluationPointValues(const std::vector<dso::EFFrame*>& frames, gtsam::Values::shared_ptr values);

    // This is the main method which intercepts the DSO update: It gets the Hessian (H) and gradient vector (b) computed by DSO, and outputs the update.
    dso::VecX computeBAUpdate(const dso::MatXX& inputH, const dso::VecX& inputB, double lambda,
                              std::vector<dso::EFFrame*>& frames, const dso::MatXX& HNoLambda);

    void acceptBAUpdate(double energy);
    double updateDynamicWeight(double energy, double rmse, bool coarseTrackingWasGood);

    // returns true if the step was small enough for the optimization to break.
    bool canBreak() const;

    // called after the optimization finished.
    void postOptimization(std::vector<dso::EFFrame*>& frames);

    void marginalizeBAFrame(dso::EFFrame* fh);

    void addMarginalizedPointsBA(const dso::MatXX& H, const dso::VecX& b, std::vector<dso::EFFrame*>& frames);

    void addPriorBA(dso::EFFrame* h, const dso::Vec8& H_diag, const dso::Vec8& b_in);

    double getBAEnergy(bool useNewValues);

protected:
    std::vector<std::shared_ptr<BAExtension>> extensions;

private:

    std::unique_ptr<BAGraphs> baGraphs;

    // Will be the identity transformation for now, but convert epsilon from right to left.
    std::unique_ptr<PoseTransformation> transformationDSOToBA;

    GTSAMIntegrationSettings settings;

    boost::shared_ptr<gtsam::Values> baValues, baEvalValues, newBAValues, baValuesAfterBAUpdate;
    gtsam::Ordering baOrdering, baOrderingSmall; // baOrdering contains all keys and baOrderingSmall only the ones known by DSO (only poses).
    std::map<gtsam::Key, size_t> baDimMap;

    bool canBreakOptimization = false;

    dso::CalibHessian* HCalib;

    double currBATimestamp{-1};

    // stored for getActiveDSOFactor.
    dso::MatXX lastDSOH;
    dso::VecX lastDSOB;
    double lastDSOEnergy = 0.0;

    gtsam::LinearContainerFactor::shared_ptr
    convertedDSOHAndBToFactor(const gtsam::Matrix& H, const gtsam::Vector& b,
                              const gtsam::Values& values, double energy = 0.0);

    DynamicWeightCallback dynamicWeightCallback;
    double computeDSOWeight() const;
    double dynamicDSOWeight = 1.0;
};

void fillAdditionalKeysFromScatter(const gtsam::Ordering& originalOrdering, gtsam::Ordering& orderingToFill,
                                   const AugmentedScatter& scatter);

}


#endif //DMVIO_BAGTSAMINTEGRATION_H
