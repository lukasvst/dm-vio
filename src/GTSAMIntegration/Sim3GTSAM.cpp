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

Sim3GTSAM::Sim3GTSAM(double scale) : sim()
{
    sim.setScale(scale);
}

Sim3GTSAM::Sim3GTSAM(const Sophus::Sim3d& sim) : sim(sim)
{

}

void Sim3GTSAM::print(const std::string& str) const
{
    std::cout << str << "Sim3: " << std::endl << sim.matrix() << std::endl;
}

size_t Sim3GTSAM::dim() const
{
    return 7;
}

bool Sim3GTSAM::equals(const Sim3GTSAM& other, double tol) const
{
    Rot3 test(sim.rotationMatrix());
    return (Rot3(sim.rotationMatrix())).equals(Rot3(other.sim.rotationMatrix()), tol) &&
           (Point3(sim.translation())).equals(Point3(other.sim.translation()), tol) &&
           (fabs(sim.scale() - other.sim.scale()) < tol);
}

Sim3GTSAM Sim3GTSAM::retract(const gtsam::Vector& inc) const
{
    // Switch rotation and translation to match the GTSAM convention!
    gtsam::Vector myInc = inc;
    myInc.segment(0, 3) = inc.segment(3, 3);
    myInc.segment(3, 3) = inc.segment(0, 3);
    // For larger values the exp would get too large.
    if(std::abs(myInc(6)) > 10)
    {
        myInc(6) *= 10 / std::abs(myInc(6));
    }
    Sophus::Sim3d after = sim * Sophus::Sim3d::exp(myInc);
    return Sim3GTSAM(after);
}


// Basically we have to solve the following for inc: : this->retract(inc)=other
// -> sim * exp(inc) = other -> exp(inc) = sim^{-1} * other -> inc = log(sim^{-1} * other)
gtsam::Vector7 Sim3GTSAM::localCoordinates(const Sim3GTSAM& other) const
{
    // Switch rotation and translation to match the GTSAM convention!
    Sophus::Sim3d::Tangent tangent = Sophus::Sim3d::log(sim.inverse() * other.sim);
    Sophus::Sim3d::Tangent returning = tangent;
    returning.segment(0, 3) = tangent.segment(3, 3);
    returning.segment(3, 3) = tangent.segment(0, 3);
    return returning;
}

Sim3GTSAM::Sim3GTSAM(const Sim3GTSAM& other) : sim(other.sim)
{

}


bool gtsam::traits<Sim3GTSAM>::Equals(const Sim3GTSAM& val1, const Sim3GTSAM& val2, double tol)
{
    return val1.equals(val2, tol);
}

int gtsam::traits<Sim3GTSAM>::GetDimension(const Sim3GTSAM& sim)
{
    return 7;
}

gtsam::Vector7 gtsam::traits<Sim3GTSAM>::Local(Sim3GTSAM origin, Sim3GTSAM other)
{
    return origin.localCoordinates(other);
}

Sim3GTSAM traits<Sim3GTSAM>::Retract(const Sim3GTSAM& origin, const gtsam::Vector7& v)
{
    return origin.retract(v);
}

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
