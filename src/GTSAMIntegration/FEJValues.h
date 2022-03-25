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

#ifndef DMVIO_FEJVALUES_H
#define DMVIO_FEJVALUES_H

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include "dso/util/settings.h"
#include "Marginalization.h"
#include "GTSAMUtils.h"

namespace dmvio
{

// Handles First-Estimates Jacobians (FEJ) for GTSAM.
class FEJValues
{
public:
    gtsam::Values fejValues;

    // Called when keys are become connected to marginalization factors. Their values will be inserted into fejValues.
    template<typename T> void insertConnectedKeys(const T& connectedKeys, const gtsam::Values& currentValues)
    {
        // always insert current poses and affine brightness (no matter if connected).
        for(auto&& val : currentValues)
        {
            auto chr = gtsam::Symbol(val.key).chr();
            if(chr == 'p' || chr == 'a')
            {
                eraseAndInsert(fejValues, val.key, val.value);
            }
        }
        for(auto&& key : connectedKeys)
        {
            if(!fejValues.exists(key))
            {
                fejValues.insert(key, currentValues.at(key));
            }
        }
    }

    // Remove keys from FEJMap, as they have been marginalized / removed.
    template<typename T> void keysRemoved(const T& keysRemoved)
    {
        for(auto&& key : keysRemoved)
        {
            fejValues.erase(key);
        }
    }

    // Returns values containing neededKeys, using fejValues where available, otherwise current values.
    template<typename T> gtsam::Values buildValues(const T& neededKeys, const gtsam::Values& currentValues)
    {
        gtsam::Values ret;
        for(auto&& key : neededKeys)
        {
            bool use = fejValues.exists(key);
            if(use)
            {
                ret.insert(key, fejValues.at(key));
            }else
            {
                ret.insert(key, currentValues.at(key));
            }
        }
        return ret;
    }

};

// Interface for factors which can handle FEJ.
// Implemented by PoseTransformationFactor and FEJNoiseModelFactor.
class FactorHandlingFEJ
{
public:
    virtual void setFEJValues(std::shared_ptr<FEJValues> fej) = 0;
};

// calls setFEJValues for all factors in graph which implement FactorHandlingFEJ.
void setFEJMapForGraph(gtsam::NonlinearFactorGraph& graph, const std::shared_ptr<FEJValues>& fejValues);

}

#endif //DMVIO_FEJVALUES_H
