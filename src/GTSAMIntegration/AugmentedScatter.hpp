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

#ifndef AugmentedScatter_hpp
#define AugmentedScatter_hpp

#include <stdio.h>

#include <gtsam/nonlinear/LinearContainerFactor.h>

#include "util/NumType.h"
#include "OptimizationBackend/EnergyFunctionalStructs.h"

#include <gtsam/linear/Scatter.h>

namespace dmvio
{
class AugmentedScatter : public gtsam::Scatter
{
public:
    // Scatter that can additionally handle keys that don't exist in the factor graph. For those keys the dimension must be specified in keyDimMap
    AugmentedScatter(const gtsam::GaussianFactorGraph& gfg,boost::optional<const gtsam::Ordering&> ordering, const std::map<gtsam::Key, size_t>& keyDimMap);

    iterator findNew(gtsam::Key key);

    std::pair<gtsam::Matrix, gtsam::Vector> computeHessian(const gtsam::GaussianFactorGraph& gfg);
    gtsam::Matrix computeAugmentedHessian(const gtsam::GaussianFactorGraph& gfg);
};
}


#endif /* AugmentedScatter_hpp */
