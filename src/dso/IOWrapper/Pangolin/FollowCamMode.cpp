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

#include "FollowCamMode.h"

namespace dso
{
namespace IOWrap
{

pangolin::OpenGlRenderState*
FollowCamMode::updateVisualizationCam(const Sophus::SE3d& camToWorldIn, pangolin::OpenGlRenderState& renderStateIn)
{
    bool transOnly = followCamTransSetting->Get();
    bool followCam = followCamSetting->Get(); // curr setting value.
    followCam = followCam || transOnly;
    active = active || followCam;

    if(!active)
    {
        return &renderStateIn;
    }

    output.SetProjectionMatrix(renderStateIn.GetProjectionMatrix());

    auto&& modelViewIn = renderStateIn.GetModelViewMatrix();
    Sophus::SE3d T_pcam_world(modelViewIn); // world to Pangolin-Cam.

    Sophus::SE3d output_cam_w;

    Sophus::SE3d camToWorld = getAverageCamToWorld(camToWorldIn);

    if(transOnly && !transActive && followActive)
    {
        // We want transOnly, but before, we were using normal followCam. --> first disable that, then proceed as
        // normal.
        disableOffsetForFollowCam(camToWorld);
        followActive = false;
    }

    if(followCam && !transOnly && transActive)
    {
        // We want followCam, but before we were using transOnly --> first disable that.
        Sophus::SE3d noRot{Eigen::Quaterniond::Identity(), camToWorld.translation()};
        disableOffsetForFollowCam(noRot);
        followActive = false;
    }else if(transOnly || transActive)
    {
        // We are using translation only or have just disabled it --> set rotation to identity.
        camToWorld.setQuaternion(Eigen::Quaterniond::Identity());
    }

    if(followCam)
    {
        if(!followActive)
        {
            // Activate follow cam --> set offset!
            setOffsetForFollowCam(camToWorld);
        }
        output_cam_w = T_pcam_world * currOffset * camToWorld.inverse();
    }else
    {
        if(followActive)
        {
            // Disable follow cam --> set offset!
            disableOffsetForFollowCam(camToWorld);
        }
        output_cam_w = T_pcam_world * currOffset;
    }

    output.SetModelViewMatrix(output_cam_w.matrix());

    followActive = followCam;
    transActive = transOnly;

    return &output;
}

void FollowCamMode::setOffsetForFollowCam(Sophus::SE3d& camToWorld)
{
    currOffset = currOffset * camToWorld;
}

void FollowCamMode::disableOffsetForFollowCam(Sophus::SE3d& camToWorld)
{
    currOffset = currOffset * camToWorld.inverse();
}

void FollowCamMode::createPangolinSettings()
{
    followCamSetting = std::make_unique<pangolin::Var<bool>>("ui.followCam", false, true);
    followCamTransSetting = std::make_unique<pangolin::Var<bool>>("ui.followCamTrans", false, true);
    smoothnessSetting = std::make_unique<pangolin::Var<int>>("ui.smoothness", 50, 1, 100, false);
}

Sophus::SE3d FollowCamMode::getAverageCamToWorld(const Sophus::SE3d& camToWorld)
{
    int num = smoothnessSetting->Get();

    rotations.emplace_back(camToWorld.unit_quaternion());
    translations.emplace_back(camToWorld.translation());

    while(rotations.size() > num)
    {
        rotations.pop_front();
    }
    while(translations.size() > num)
    {
        translations.pop_front();
    }

    Eigen::Quaterniond avgQuat = computeAverageQuat(rotations.begin(), rotations.end());
    Eigen::Vector3d avgTrans;
    avgTrans.setZero();
    for(auto&& trans : translations)
    {
        avgTrans += trans;
    }
    avgTrans /= translations.size();

    return Sophus::SE3d(avgQuat, avgTrans);
}

}
}