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

#ifndef DMVIO_IMUTYPES_H
#define DMVIO_IMUTYPES_H

#include <Eigen/Core>
#include <vector>

namespace dmvio
{
class IMUMeasurement
{
public:
    IMUMeasurement(const Eigen::Vector3d& accData, const Eigen::Vector3d& gyrData, double integrationTime);

    const Eigen::Vector3d& getAccData() const;

    const Eigen::Vector3d& getGyrData() const;

    double getIntegrationTime() const;

private:
    Eigen::Vector3d accData{};
    Eigen::Vector3d gyrData{};
    double integrationTime; // time between this and previous IMU measurement.
};

typedef std::vector<IMUMeasurement> IMUData;
}

#endif //DMVIO_IMUTYPES_H
