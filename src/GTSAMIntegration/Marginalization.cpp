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

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include "Marginalization.h"
#include "GTSAMUtils.h"

gtsam::NonlinearFactorGraph::shared_ptr
dmvio::marginalizeOut(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                      const gtsam::FastVector<gtsam::Key>& keysToMarginalize,
                      std::function<void(const gtsam::FastSet<gtsam::Key>&)> connectedKeyCallback)
{
    if(keysToMarginalize.empty())
    {
        std::cout << "WARNING: Calling marginalizeOut with empty keysToMarginalize." << std::endl;
        return boost::shared_ptr<gtsam::NonlinearFactorGraph>(new gtsam::NonlinearFactorGraph(graph));
    }

    boost::shared_ptr<gtsam::NonlinearFactorGraph> newGraph(new gtsam::NonlinearFactorGraph);

    gtsam::NonlinearFactorGraph marginalizedOutGraph;

    gtsam::FastSet<gtsam::Key> setOfKeysToMarginalize(keysToMarginalize);
    gtsam::FastSet<gtsam::Key> connectedKeys;

    extractKeysToMarginalize(graph, *newGraph, marginalizedOutGraph, setOfKeysToMarginalize, connectedKeys);

    if(connectedKeyCallback)
    {
        connectedKeyCallback(connectedKeys);
    }

    gtsam::GaussianFactorGraph::shared_ptr linearizedFactorsToMarginalize = marginalizedOutGraph.linearize(values);
    std::map<gtsam::Key, size_t> keyDimMap = linearizedFactorsToMarginalize->getKeyDimMap();

    int mSize = 0;
    int aSize = 0;

    gtsam::Ordering ordering;

    gtsam::Ordering connectedOrdering;
    gtsam::FastVector<size_t> connectedDims;
    for(const gtsam::Key& k : setOfKeysToMarginalize)
    {
        ordering.push_back(k);
        mSize += keyDimMap[k];
    }
    for(const gtsam::Key& k : connectedKeys)
    {
        ordering.push_back(k);
        connectedOrdering.push_back(k);
        connectedDims.push_back(keyDimMap[k]);
        aSize += keyDimMap[k];
    }

    gtsam::Matrix hessian = linearizedFactorsToMarginalize->augmentedHessian(ordering);

    gtsam::Matrix HAfterSchurComplement = computeSchurComplement(hessian, mSize, aSize);

    gtsam::SymmetricBlockMatrix sm(connectedDims, true);
    sm.setFullMatrix(HAfterSchurComplement);

    gtsam::LinearContainerFactor::shared_ptr lcf(new gtsam::LinearContainerFactor(
            gtsam::HessianFactor(connectedOrdering, sm), values));
    newGraph->add(lcf);

    return newGraph;
}

gtsam::NonlinearFactorGraph::shared_ptr
dmvio::marginalizeOut(const gtsam::NonlinearFactorGraph& graph, gtsam::Values& values,
                      const gtsam::FastVector<gtsam::Key>& keysToMarginalize,
                      std::function<void(const gtsam::FastSet<gtsam::Key>&)> connectedKeyCallback,
                      bool deleteFromValues)
{
    auto ret = marginalizeOut(graph, values, keysToMarginalize, connectedKeyCallback);
    if(deleteFromValues)
    {
        for(size_t i = 0; i < keysToMarginalize.size(); i++)
        {
            values.erase(keysToMarginalize[i]);
        }
    }
    return ret;
}

void dmvio::extractKeysToMarginalize(const gtsam::NonlinearFactorGraph& graph, gtsam::NonlinearFactorGraph& newGraph,
                                     gtsam::NonlinearFactorGraph& marginalizedOutGraph,
                                     gtsam::FastSet<gtsam::Key>& setOfKeysToMarginalize,
                                     gtsam::FastSet<gtsam::Key>& connectedKeys)
{
    for(size_t i = 0; i < graph.size(); i++)
    {
        gtsam::NonlinearFactor::shared_ptr factor = graph.at(i);

        gtsam::FastSet<gtsam::Key> set_of_factor_keys(factor->keys());

        gtsam::FastSet<gtsam::Key> intersection;

        std::set_intersection(setOfKeysToMarginalize.begin(), setOfKeysToMarginalize.end(),
                              set_of_factor_keys.begin(), set_of_factor_keys.end(),
                              std::inserter(intersection, intersection.begin()));

        if(!intersection.empty())
        {
            std::set_difference(set_of_factor_keys.begin(), set_of_factor_keys.end(),
                                setOfKeysToMarginalize.begin(), setOfKeysToMarginalize.end(),
                                std::inserter(connectedKeys, connectedKeys.begin()));

            marginalizedOutGraph.add(factor);
        }else
        {
            newGraph.add(factor);
        }
    }
}

gtsam::Matrix dmvio::computeSchurComplement(const gtsam::Matrix& augmentedHessian, int mSize, int aSize)
{
    auto pair = dmvio::pairFromAugmentedHessian(augmentedHessian);

    // Preconditioning like in DSO code.
    gtsam::Vector SVec = (pair.first.diagonal().cwiseAbs() +
                          gtsam::Vector::Constant(pair.first.cols(), 10)).cwiseSqrt();
    gtsam::Vector SVecI = SVec.cwiseInverse();

    gtsam::Matrix hessianScaled = SVecI.asDiagonal() * pair.first * SVecI.asDiagonal();
    gtsam::Vector bScaled = SVecI.asDiagonal() * pair.second;

    gtsam::Matrix Hmm = hessianScaled.block(0, 0, mSize, mSize);
    gtsam::Matrix Hma = hessianScaled.block(0, mSize, mSize, aSize);
    gtsam::Matrix Haa = hessianScaled.block(mSize, mSize, aSize, aSize);

    gtsam::Vector bm = bScaled.segment(0, mSize);
    gtsam::Vector ba = bScaled.segment(mSize, aSize);

    // Compute inverse.
    gtsam::Matrix HmmInv = Hmm.completeOrthogonalDecomposition().pseudoInverse();

    gtsam::Matrix HaaNew = Haa - Hma.transpose() * HmmInv * Hma;
    gtsam::Vector baNew = ba - Hma.transpose() * HmmInv * bm;

    // Unscale
    gtsam::Vector SVecUpdated = SVec.segment(mSize, aSize);
    gtsam::Matrix HNewUnscaled = SVecUpdated.asDiagonal() * HaaNew * SVecUpdated.asDiagonal();
    gtsam::Matrix bNewUnscaled = SVecUpdated.asDiagonal() * baNew;

    // Make Hessian symmetric for numeric reasons.
    HNewUnscaled = 0.5 * (HNewUnscaled.transpose() + HNewUnscaled).eval();

    gtsam::Matrix augmentedHRes(aSize + 1, aSize + 1);
    augmentedHRes.setZero();
    augmentedHRes.topLeftCorner(aSize, aSize) = HNewUnscaled;
    augmentedHRes.topRightCorner(aSize, 1) = bNewUnscaled;
    augmentedHRes.bottomLeftCorner(1, aSize) = bNewUnscaled.transpose();

    return augmentedHRes;
}

