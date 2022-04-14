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

#include "GTSAMUtils.h"

using namespace gtsam;

gtsam::Matrix dmvio::augmentedHessianFromPair(const std::pair<gtsam::Matrix, gtsam::Vector>& pair)
{
    int n = pair.first.rows();

    gtsam::Matrix returning = gtsam::Matrix::Zero(n + 1, n + 1);
    returning.block(0, 0, n, n) = pair.first;
    returning.block(n, 0, 1, n) = pair.second.transpose();
    returning.block(0, n, n, 1) = pair.second;
    return returning;
}

std::pair<gtsam::Matrix, gtsam::Vector> dmvio::pairFromAugmentedHessian(const gtsam::Matrix& matrix)
{
    int n = matrix.rows() - 1;
    return std::pair<gtsam::Matrix, gtsam::Vector>(matrix.block(0, 0, n, n), matrix.block(0, n, n, 1));
}

void dmvio::removeKeysFromGraph(gtsam::NonlinearFactorGraph& graph, const std::set<gtsam::Key>& keysToRemove,
                                int stopAfterNoRemoval)
{
    if(keysToRemove.empty())
    {
        return;
    }
    int noRemoval = 0;
    // We use the fact that we inserted the factors in order.
    for(auto it = graph.begin(); it != graph.end(); ++it)
    {
        auto&& keys = (*it)->keys();
        bool keyContained = false;
        for(auto&& key : keys)
        {
            if(keysToRemove.find(key) != keysToRemove.end())
            {
                keyContained = true;
                break;
            }
        }
        if(keyContained)
        {
            it = graph.erase(it);
            it--;
            noRemoval = 0;
        }else
        {
            noRemoval++;
            if(stopAfterNoRemoval > 0 && noRemoval >= stopAfterNoRemoval) break;
        }
    }
}


