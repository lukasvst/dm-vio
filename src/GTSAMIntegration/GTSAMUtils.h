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

#ifndef DMVIO_GTSAMUTILS_H
#define DMVIO_GTSAMUTILS_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <set>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace dmvio
{

std::pair<gtsam::Matrix, gtsam::Vector> pairFromAugmentedHessian(const gtsam::Matrix& matrix);

gtsam::Matrix augmentedHessianFromPair(const std::pair<gtsam::Matrix, gtsam::Vector>& pair);

void removeKeysFromGraph(gtsam::NonlinearFactorGraph& graph, const std::set<gtsam::Key>& keysToRemove,
                         int stopAfterNoRemoval = -1);

template<typename T> void eraseAndInsert(gtsam::Values& values, gtsam::Key key, const T& value)
{
    if(values.exists(key))
    {
        values.update(key, value);
    }else
    {
        values.insert(key, value);
    }
}

template<typename T> void eraseAndInsert(gtsam::Values::shared_ptr values, gtsam::Key key, const T& value)
{
    eraseAndInsert(*values, key, value);
}

template<typename C> gtsam::Key getMinKeyWithChr(const C& keys, unsigned char chr)
{
    auto minIt = std::min_element(keys.begin(), keys.end(), [chr](const gtsam::Key& key1, const gtsam::Key& key2)
    {
        gtsam::Symbol sym1(key1);
        gtsam::Symbol sym2(key2);
        return (sym1.chr() == chr && sym1.index() < sym2.index()) || sym2.chr() != chr;
    });
    return *minIt;
}

template<typename C> gtsam::Key getMaxKeyWithChr(const C& keys, unsigned char chr)
{
    // we still use min_element but reverse the comparison
    auto minIt = std::min_element(keys.begin(), keys.end(), [chr](const gtsam::Key& key1, const gtsam::Key& key2)
    {
        gtsam::Symbol sym1(key1);
        gtsam::Symbol sym2(key2);
        return (sym1.chr() == chr && sym1.index() > sym2.index()) || sym2.chr() != chr;
    });
    return *minIt;
}

}

#endif //DMVIO_GTSAMUTILS_H
