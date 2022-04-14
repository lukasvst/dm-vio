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

#include <iostream>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include "Sim3GTSAM.h"

using namespace gtsam;

ScaleGTSAM::ScaleGTSAM(double scale)
        : scale(scale)
{
}

Sophus::Sim3d ScaleGTSAM::sim() const
{
    Sophus::Sim3d ret;
    ret.setScale(scale);
    return ret;
}

void ScaleGTSAM::print(const std::string& str) const
{
    std::cout << str << " Scale: " << scale << std::endl;
}

size_t ScaleGTSAM::dim() const
{
    return 1;
}

bool ScaleGTSAM::equals(const ScaleGTSAM& other, double tol) const
{
    return fabs(scale - other.scale) < tol;
}

ScaleGTSAM ScaleGTSAM::identity()
{
    return ScaleGTSAM(0);
}

ScaleGTSAM ScaleGTSAM::operator*(const ScaleGTSAM& T) const
{
    return ScaleGTSAM((sim() * T.sim()).scale());
}

ScaleGTSAM ScaleGTSAM::inverse() const
{
    return ScaleGTSAM(sim().inverse().scale());
}

gtsam::Vector1 ScaleGTSAM::Logmap(const ScaleGTSAM& s, gtsam::OptionalJacobian<1, 1> Hm)
{
    Sophus::Sim3d::Tangent tangent = Sophus::Sim3d::log(s.sim());
    gtsam::Vector1 ret;
    ret(0) = tangent(6);
    return ret;
}

ScaleGTSAM ScaleGTSAM::Expmap(const Vector1& v, gtsam::OptionalJacobian<1, 1> Hm)
{
    double scaleInc = v(0);
    // For larger values the exp would get too large.
    if(std::abs(scaleInc) > 10)
    {
        scaleInc *= 10 / std::abs(scaleInc);
    }
    gtsam::Vector7 myInc = gtsam::Vector7::Zero();
    myInc(6) = scaleInc;
    Sophus::Sim3d simAfter = Sophus::Sim3d::exp(myInc);
    return ScaleGTSAM(simAfter.scale());
}

gtsam::Matrix1 ScaleGTSAM::AdjointMap() const
{
    // This is probably just one always, which could be used to make it faster.
    gtsam::Matrix77 adj = sim().Adj();
    return adj.block(6, 6, 1, 1);
}
