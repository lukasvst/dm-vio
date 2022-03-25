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

#ifndef GTData_hpp
#define GTData_hpp

#include <stdio.h>

#include <Eigen/Eigenvalues>

#include <sophus/sophus.hpp>
#include <sophus/se3.hpp>

namespace dmvio
{
class GTData
{
public:

    inline GTData()
    {}

    inline GTData(Sophus::SE3 pose, Eigen::Vector3d velocity, Eigen::Vector3d biasRotation,
                  Eigen::Vector3d biasTranslation)
            : pose(pose), velocity(velocity), biasRotation(biasRotation), biasTranslation(biasTranslation)
    {}


    Sophus::SE3 pose;
    Eigen::Vector3d velocity; // Note: velocities might be in the vicon frame instead of the world frame...
    Eigen::Vector3d biasRotation;
    Eigen::Vector3d biasTranslation;
};
}

#endif /* GTData_hpp */
