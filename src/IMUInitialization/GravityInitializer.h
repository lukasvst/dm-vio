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

#ifndef DMVIO_GRAVITYINITIALIZER_H
#define DMVIO_GRAVITYINITIALIZER_H

#include "sophus/se3.hpp"
#include "IMU/IMUIntegration.hpp"

namespace dmvio
{
// Averages accelerometer measurements to provide a simple initialization for gravity direction.
class GravityInitializer
{
public:
    GravityInitializer(int numMeasurementsToUse, const IMUCalibration& imuCalibration);

    // returns an approximate imuToWorld transform (only rotation).
    Sophus::SE3d addMeasure(const IMUData& imuData, const Sophus::SE3d& currToFirst);

private:
    int maxNumMeasurements; // Num of last gravity measurements to average.
    std::deque<Eigen::Vector3d> measures;
    Eigen::Vector3d gravity;
};

double getGravityError(const Sophus::SE3d& imuToWorld, const Sophus::SE3d& imuToWorldGT);

}

#endif //DMVIO_GRAVITYINITIALIZER_H
