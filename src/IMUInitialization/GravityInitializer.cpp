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

#include "GravityInitializer.h"
#include <boost/math/constants/constants.hpp>

using dmvio::GravityInitializer;
using dmvio::IMUIntegration;

GravityInitializer::GravityInitializer(int numMeasurementsToUse, const IMUCalibration& imuCalibration)
        : maxNumMeasurements(numMeasurementsToUse)
{
    gravity = imuCalibration.gravity;
}

Sophus::SE3d
GravityInitializer::addMeasure(const IMUData& imuData, const Sophus::SE3d& currToFirst)
{
    int numMeasure = 0;
    Eigen::Vector3d measure(0.0, 0.0, 0.0);
    for(int i = 0; i < imuData.size(); ++i)
    {
        Eigen::Vector3d curr = imuData[i].getAccData();
        measure += curr;
        numMeasure++;
    }
    measure /= (double) numMeasure;

    measures.push_back(measure);

    if(measures.size() > maxNumMeasurements)
    {
        measures.pop_front();
    }

    Eigen::Vector3d filteredM(0.0, 0.0, 0.0);
    for(auto&& m : measures)
    {
        filteredM += m;
    }
    filteredM /= (double) measures.size();

    measure = filteredM;

    Eigen::Quaterniond quat;
    quat.setFromTwoVectors(measure, -gravity);

    Sophus::SE3d imuToWorld(quat, Eigen::Vector3d::Zero());

    return imuToWorld;
}

double dmvio::getGravityError(const Sophus::SE3d& imuToWorld, const Sophus::SE3d& imuToWorldGT)
{
    Eigen::Vector3d g = (gtsam::Vector(3)
            << 0, 0, -9.8082).finished(); // Only the direction actually matters so it's ok if this is not the actually used gravity.
    // g is in world coordinates, so check what g is in drone coordinates.
    Eigen::Vector3d gDrone = imuToWorld.inverse().rotationMatrix() * g;
    Eigen::Vector3d gDroneGT = imuToWorldGT.inverse().rotationMatrix() * g;

    // Compute angle between the two vectors.
    double angle = std::acos(gDrone.dot(gDroneGT) / (gDrone.norm() * gDroneGT.norm()));
    double degrees = (angle * 180.0) / boost::math::constants::pi<double>();

    return degrees;
}
