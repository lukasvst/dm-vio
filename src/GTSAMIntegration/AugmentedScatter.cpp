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
// This code file is based on the file Scatter.cpp from the project GTSAM, which has been released under the following conditions:
/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */
#include "AugmentedScatter.hpp"
#include <gtsam/linear/GaussianFactorGraph.h>

using namespace gtsam;

dmvio::AugmentedScatter::AugmentedScatter(const GaussianFactorGraph& gfg, boost::optional<const Ordering&> ordering, const std::map<gtsam::Key, size_t>& keyDimMap)
{
    // If we have an ordering, pre-fill the ordered variables first
    if (ordering) {
        for (Key key : *ordering) {
            std::map<gtsam::Key, size_t>::const_iterator it = keyDimMap.find(key);
            unsigned long dim = 0;
            if(it != keyDimMap.end())
            {
                dim = it->second;
            }
            add(key, dim);
        }
    }
    
    // Now, find dimensions of variables and/or extend
    for (const auto& factor : gfg) {
        if (!factor)
            continue;
        
        // TODO: Fix this hack to cope with zero-row Jacobians that come from BayesTreeOrphanWrappers
        const JacobianFactor* asJacobian = dynamic_cast<const JacobianFactor*>(factor.get());
        if (asJacobian && asJacobian->cols() <= 1) continue;
        
        // loop over variables
        for (GaussianFactor::const_iterator variable = factor->begin();
             variable != factor->end(); ++variable) {
            const Key key = *variable;
            iterator it = findNew(key); // theoretically expensive, yet cache friendly
            if (it!=end())
                it->dimension = factor->getDim(variable);
            else
                add(key, factor->getDim(variable));
        }
    }
    
    // To keep the same behavior as before, sort the keys after the ordering
    iterator first = begin();
    if (ordering) first += ordering->size();
    if (first != end()) std::sort(first, end());
    
    // Filter out keys with zero dimensions (if ordering had more keys)
    erase(std::remove_if(begin(), end(), SlotEntry::Zero), end());
}

FastVector<SlotEntry>::iterator dmvio::AugmentedScatter::findNew(Key key) {
    iterator it = begin();
    while(it != end()) {
        if (it->key == key)
            return it;
        ++it;
    }
    return it; // end()
}

std::pair<gtsam::Matrix, gtsam::Vector> dmvio::AugmentedScatter::computeHessian(const GaussianFactorGraph& gfg)
{
    gtsam::HessianFactor combined(gfg, *this);
    gtsam::Matrix augmented = combined.info().selfadjointView();
    size_t n = augmented.rows() - 1;
    return std::make_pair(augmented.topLeftCorner(n, n), augmented.topRightCorner(n, 1));
}

gtsam::Matrix dmvio::AugmentedScatter::computeAugmentedHessian(const gtsam::GaussianFactorGraph &gfg)
{
    gtsam::HessianFactor combined(gfg, *this);
    gtsam::Matrix augmented = combined.info().selfadjointView();
    return augmented;
}
