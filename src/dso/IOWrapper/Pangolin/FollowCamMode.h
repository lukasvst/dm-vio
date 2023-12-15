/**
* This file is part of DM-VIO.
*
* Copyright (c) 2023 Lukas von Stumberg <lukas dot stumberg at tum dot de>.
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

#ifndef DMVIO_FOLLOWCAMMODE_H
#define DMVIO_FOLLOWCAMMODE_H

#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

namespace dso
{
namespace IOWrap
{

class FollowCamMode
{
public:
    FollowCamMode() = default;

    pangolin::OpenGlRenderState* updateVisualizationCam(const Sophus::SE3d& camToWorld,
                                                        pangolin::OpenGlRenderState& renderStateIn);

    void createPangolinSettings();

private:
    Sophus::SE3d getAverageCamToWorld(const Sophus::SE3d& camToWorld);

    void setOffsetForFollowCam(Sophus::SE3d& camToWorld);
    void disableOffsetForFollowCam(Sophus::SE3d& camToWorld);

    std::unique_ptr<pangolin::Var<bool>> followCamSetting, followCamTransSetting;
    std::unique_ptr<pangolin::Var<int>> smoothnessSetting;

    bool followActive = false;
    bool transActive = false;
    Sophus::SE3d currOffset;

    pangolin::OpenGlRenderState output;

    // camToWorld translations / rotations for calculating the smoothed follow trajectory.
    std::deque<Eigen::Quaterniond> rotations;
    std::deque<Eigen::Vector3d> translations;
    bool active = false;
};

// Compute average of multiple Eigen::Quaterniond. Assumes that the quaternions are normalized!
template<typename T> Eigen::Quaterniond computeAverageQuat(T begin, T end)
{
    // compute sum of q * qT
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero();
    for(auto it = begin; it != end; ++it)
    {
        matrix += it->coeffs() * it->coeffs().transpose();
    }
    // solution is the unit Eigen vector corresponding to the largest Eigen value.
    auto eigenVecs = Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d>(matrix).eigenvectors();
    return Eigen::Quaterniond(eigenVecs.col(eigenVecs.cols() - 1));
}

}
}

#endif //DMVIO_FOLLOWCAMMODE_H
