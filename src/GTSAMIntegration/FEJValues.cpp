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

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include "FEJValues.h"

void dmvio::setFEJMapForGraph(gtsam::NonlinearFactorGraph& graph, const std::shared_ptr<FEJValues>& fejValues)
{
    for(auto&& factor : graph)
    {
        auto* casted = dynamic_cast<FactorHandlingFEJ*>(factor.get());
        if(casted)
        {
            casted->setFEJValues(fejValues);
        }
    }
}
