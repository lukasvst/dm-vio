/**
* This file is part of DM-VIO.
*
* Copyright (c) 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>.
* for more information see <http://vision.in.tum.de/dm-vio>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* NOTE: The boilerplate code for interacting with Librealsense used in this file is based on
* https://github.com/VladyslavUsenko/basalt-mirror/blob/master/src/device/rs_t265.cpp
* which was written by Vladyslav Usenko, Michael Loipführer and Nikolaus Demmel
* and published unter the BSD 3-Clause License
* Most of the logic was heavily modified but the parts taken over are still licensed under BSD
* which is why we provide a copy of the BSD-3-Clause License below.
*
* NOTE: In this file we include the method frame_to_mat which was taken over from librealsense
* (see end of the file for more details.)
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

// License for the (modified) parts of the code taken over from
// https://github.com/VladyslavUsenko/basalt-mirror/blob/master/src/device/rs_t265.cpp
// (see notes above for more details).
/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko, Michael Loipführer and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "RealsenseT265.h"
#include <iostream>
#include <iomanip>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include "IMU/IMUTypes.h"
#include "sophus/se3.hpp"
#include "util/MinimalImage.h"

using namespace dmvio;
using std::vector;

static cv::Mat frame_to_mat(const rs2::frame& f);

dmvio::RealsenseT265::RealsenseT265(FrameContainer& frameContainer, std::string cameraCalibSavePath, DatasetSaver*
datasetSaver)
        : imuInt(frameContainer, datasetSaver), cameraCalibSavePath(cameraCalibSavePath), saver(datasetSaver)
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    pipe = rs2::pipeline(context);

    config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    // We don't need the second image, but librealsense only supports querying both cameras.
    config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    if(context.query_devices().size() == 0)
    {
        std::cout << "Waiting for device to be connected" << std::endl;
        rs2::device_hub hub(context);
        hub.wait_for_device();
    }

    for(auto& s : context.query_devices()[0].query_sensors())
    {
        std::cout << "Sensor " << s.get_info(RS2_CAMERA_INFO_NAME)
                  << ". Supported options:" << std::endl;

        for(const auto& o : s.get_supported_options())
        {
            std::cout << "\t" << rs2_option_to_string(o) << std::endl;
        }
    }

    auto device = context.query_devices()[0];
    device.hardware_reset();

    std::cout << "Device " << device.get_info(RS2_CAMERA_INFO_NAME)
              << " connected" << std::endl;
}

void dmvio::RealsenseT265::start()
{
    auto callback = [&](const rs2::frame& frame)
    {
        if(!calibrationRead) return;
        if(auto fp = frame.as<rs2::motion_frame>())
        {
            auto motion = frame.as<rs2::motion_frame>();

            if(motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
               motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                auto motionData = motion.get_motion_data();
                vector<float> data(3);
                data[0] = motionData.x;
                data[1] = motionData.y;
                data[2] = motionData.z;

                // Multiply by factory calibration scale and subtract bias.
                for(int i = 0; i < 3; ++i)
                {
                    data[i] = data[i] * gyroIntrinsics.data[i][i] - gyroIntrinsics.data[i][3];
                }

                // timestamp is in milliseconds, but shall be in seconds
                imuInt.addGyrData(data, motion.get_timestamp() / 1000.0);

            }else if(motion &&
                     motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
                     motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                auto motionData = motion.get_motion_data();
                vector<float> data(3);
                data[0] = motionData.x;
                data[1] = motionData.y;
                data[2] = motionData.z;

                // Multiply by factory calibration scale and subtract bias.
                for(int i = 0; i < 3; ++i)
                {
                    data[i] = data[i] * accelIntrinsics.data[i][i] - accelIntrinsics.data[i][3];
                }

                imuInt.addAccData(data, motion.get_timestamp() / 1000.0);
            }
        }else if(auto fs = frame.as<rs2::frameset>())
        {
            auto f = fs[useCam]; // We only use left camera
            if(!f.as<rs2::video_frame>())
            {
                std::cout << "Weird Frame, skipping" << std::endl;
                return;
            }
            auto vf = f.as<rs2::video_frame>();

            double timestamp = vf.get_timestamp();

            // We somehow seem to get each image twice.
            if(undistorter && std::abs(timestamp - lastImgTimestamp) > 0.001)
            {
                cv::Mat mat = frame_to_mat(f);
                assert(mat.type() == CV_8U);

                // Multiply exposure by 1000, as we want milliseconds.
                double exposure = vf.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE) * 1e-3;

                if(saver)
                {
                    saver->addImage(mat, timestamp / 1000.0, exposure);
                }

                auto img = std::make_unique<dso::MinimalImageB>(mat.cols, mat.rows);
                memcpy(img->data, mat.data, mat.rows * mat.cols);

                // timestamp is in milliseconds, but shall be in seconds
                double finalTimestamp = timestamp / 1000.0;
                // gets float exposure and double timestamp
                std::unique_ptr<dso::ImageAndExposure> finalImage(undistorter->undistort<unsigned char>(
                        img.get(),
                        static_cast<float>(exposure),
                        finalTimestamp));
                img.reset();

                // Add image to the IMU interpolator, which will forward it to the FrameContainer, once the
                // corresponding IMU data is available.
                imuInt.addImage(std::move(finalImage), finalTimestamp);

                lastImgTimestamp = timestamp;
            }
        }
    };

    profile = pipe.start(config, callback);
    readCalibration();
}

void dmvio::RealsenseT265::readCalibration()
{
    if(calibrationRead) return;
    auto accel_stream = profile.get_stream(RS2_STREAM_ACCEL);
    auto gyro_stream = profile.get_stream(RS2_STREAM_GYRO);
    auto cam0_stream = profile.get_stream(RS2_STREAM_FISHEYE, 1);
    auto cam1_stream = profile.get_stream(RS2_STREAM_FISHEYE, 2);

    // get gyro extrinsics
    if(auto gyro = gyro_stream.as<rs2::motion_stream_profile>())
    {
        gyroIntrinsics = gyro.get_motion_intrinsics();

        Eigen::Matrix<double, 3, 4> gyroMatrix;
        for(int x = 0; x < 3; ++x)
        {
            for(int y = 0; y < 4; ++y)
            {
                gyroMatrix(x, y) = gyroIntrinsics.data[x][y];
            }
        }

        // When receiving gyro measurements we need to multiply by the scale and subtract the bias!
        std::cout << "Gyro Matrix\n" << gyroMatrix << std::endl;

        Eigen::Vector3d gyro_noise_std = Eigen::Vector3d(gyroIntrinsics.noise_variances[0],
                                                         gyroIntrinsics.noise_variances[1],
                                                         gyroIntrinsics.noise_variances[2]).cwiseSqrt();

        Eigen::Vector3d gyro_bias_std = Eigen::Vector3d(gyroIntrinsics.bias_variances[0],
                                                        gyroIntrinsics.bias_variances[1],
                                                        gyroIntrinsics.bias_variances[2]).cwiseSqrt();

        std::cout << "Gyro noise var: " << gyro_noise_std
                  << " bias var: " << gyro_bias_std << std::endl;
    }else
    {
        std::abort();
    }

    // get accel extrinsics
    if(auto accel = accel_stream.as<rs2::motion_stream_profile>())
    {
        accelIntrinsics = accel.get_motion_intrinsics();
        Eigen::Matrix<double, 3, 4> accelMatrix;
        for(int x = 0; x < 3; ++x)
        {
            for(int y = 0; y < 4; ++y)
            {
                accelMatrix(x, y) = accelIntrinsics.data[x][y];
            }
        }

        Eigen::Vector3d accel_noise_std = Eigen::Vector3d(accelIntrinsics.noise_variances[0],
                                                          accelIntrinsics.noise_variances[1],
                                                          accelIntrinsics.noise_variances[2]).cwiseSqrt();

        Eigen::Vector3d accel_bias_std = Eigen::Vector3d(accelIntrinsics.bias_variances[0],
                                                         accelIntrinsics.bias_variances[1],
                                                         accelIntrinsics.bias_variances[2]).cwiseSqrt();

        std::cout << "Accel noise var: " << accel_noise_std
                  << " bias var: " << accel_bias_std << std::endl;
    }else
    {
        std::abort();
    }

    // get camera ex-/intrinsics
    for(const auto& cam_stream : {useCam == 0 ? cam0_stream : cam1_stream})
    {
        if(auto cam = cam_stream.as<rs2::video_stream_profile>())
        {
            // extrinsics
            rs2_extrinsics ex = cam.get_extrinsics_to(gyro_stream);
            Eigen::Matrix3f rot = Eigen::Map<Eigen::Matrix3f>(ex.rotation);
            Eigen::Vector3f trans = Eigen::Map<Eigen::Vector3f>(ex.translation);

            Sophus::SE3d T_imu_cam(rot.cast<double>(), trans.cast<double>());

            std::cout << "T_imu_cam: " << T_imu_cam.matrix() << std::endl;
            std::cout << "T_cam_imu: " << T_imu_cam.inverse().matrix() << std::endl;

            imuCalibration = std::make_unique<dmvio::IMUCalibration>(T_imu_cam.inverse());

            // intrinsics
            rs2_intrinsics intrinsics = cam.get_intrinsics();

            // We assume Kanala Brandt model.
            assert(intrinsics.model == RS2_DISTORTION_KANNALA_BRANDT4);

            // Write camera calibration to file.
            std::ofstream calibStream(cameraCalibSavePath);

            calibStream << "KannalaBrandt " << intrinsics.fx << " " << intrinsics.fy << " " << intrinsics.ppx << " "
                        << intrinsics.ppy << " " << intrinsics.coeffs[0] << " " << intrinsics.coeffs[1] << " " <<
                        intrinsics.coeffs[2] << " " << intrinsics.coeffs[3] << "\n";
            calibStream << cam.width() << " " << cam.height() << "\n";
            // We rectify to a focal length of 0.2 instead of using the full size as otherwise too much of the
            // rectified image will be focus on a small outer part of the original fisheye image.
            calibStream << "0.2 0.2 0.499 0.499 0\n";
            calibStream << cam.width() << " " << cam.height() << "\n";
            calibStream.flush();

        }else
        {
            std::abort();
        }
    }

    calibrationRead = true;

}

void RealsenseT265::setUndistorter(dso::Undistort* undistort)
{
    this->undistorter = undistort;
}


// This Method was copied from https://github.com/IntelRealSense/librealsense/blob/master/wrappers/opencv/cv-helpers.hpp
// License: Apache 2.0. See http://www.apache.org/licenses/LICENSE-2.0 or below.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
// Convert rs2::frame to cv::Mat
static cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if(f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*) f.get_data(), Mat::AUTO_STEP);
    }else if(f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*) f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }else if(f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*) f.get_data(), Mat::AUTO_STEP);
    }else if(f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*) f.get_data(), Mat::AUTO_STEP);
    }else if(f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*) f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Apache License, only applicable for the method frame_to_mat()!
//    Apache License
//                           Version 2.0, January 2004
//                        http://www.apache.org/licenses/
//
//   TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION
//
//   1. Definitions.
//
//      "License" shall mean the terms and conditions for use, reproduction,
//      and distribution as defined by Sections 1 through 9 of this document.
//
//      "Licensor" shall mean the copyright owner or entity authorized by
//      the copyright owner that is granting the License.
//
//      "Legal Entity" shall mean the union of the acting entity and all
//      other entities that control, are controlled by, or are under common
//      control with that entity. For the purposes of this definition,
//      "control" means (i) the power, direct or indirect, to cause the
//      direction or management of such entity, whether by contract or
//      otherwise, or (ii) ownership of fifty percent (50%) or more of the
//      outstanding shares, or (iii) beneficial ownership of such entity.
//
//      "You" (or "Your") shall mean an individual or Legal Entity
//      exercising permissions granted by this License.
//
//      "Source" form shall mean the preferred form for making modifications,
//      including but not limited to software source code, documentation
//      source, and configuration files.
//
//      "Object" form shall mean any form resulting from mechanical
//      transformation or translation of a Source form, including but
//      not limited to compiled object code, generated documentation,
//      and conversions to other media types.
//
//      "Work" shall mean the work of authorship, whether in Source or
//      Object form, made available under the License, as indicated by a
//      copyright notice that is included in or attached to the work
//      (an example is provided in the Appendix below).
//
//      "Derivative Works" shall mean any work, whether in Source or Object
//      form, that is based on (or derived from) the Work and for which the
//      editorial revisions, annotations, elaborations, or other modifications
//      represent, as a whole, an original work of authorship. For the purposes
//      of this License, Derivative Works shall not include works that remain
//      separable from, or merely link (or bind by name) to the interfaces of,
//      the Work and Derivative Works thereof.
//
//      "Contribution" shall mean any work of authorship, including
//      the original version of the Work and any modifications or additions
//      to that Work or Derivative Works thereof, that is intentionally
//      submitted to Licensor for inclusion in the Work by the copyright owner
//      or by an individual or Legal Entity authorized to submit on behalf of
//      the copyright owner. For the purposes of this definition, "submitted"
//      means any form of electronic, verbal, or written communication sent
//      to the Licensor or its representatives, including but not limited to
//      communication on electronic mailing lists, source code control systems,
//      and issue tracking systems that are managed by, or on behalf of, the
//      Licensor for the purpose of discussing and improving the Work, but
//      excluding communication that is conspicuously marked or otherwise
//      designated in writing by the copyright owner as "Not a Contribution."
//
//      "Contributor" shall mean Licensor and any individual or Legal Entity
//      on behalf of whom a Contribution has been received by Licensor and
//      subsequently incorporated within the Work.
//
//   2. Grant of Copyright License. Subject to the terms and conditions of
//      this License, each Contributor hereby grants to You a perpetual,
//      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
//      copyright license to reproduce, prepare Derivative Works of,
//      publicly display, publicly perform, sublicense, and distribute the
//      Work and such Derivative Works in Source or Object form.
//
//   3. Grant of Patent License. Subject to the terms and conditions of
//      this License, each Contributor hereby grants to You a perpetual,
//      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
//      (except as stated in this section) patent license to make, have made,
//      use, offer to sell, sell, import, and otherwise transfer the Work,
//      where such license applies only to those patent claims licensable
//      by such Contributor that are necessarily infringed by their
//      Contribution(s) alone or by combination of their Contribution(s)
//      with the Work to which such Contribution(s) was submitted. If You
//      institute patent litigation against any entity (including a
//      cross-claim or counterclaim in a lawsuit) alleging that the Work
//      or a Contribution incorporated within the Work constitutes direct
//      or contributory patent infringement, then any patent licenses
//      granted to You under this License for that Work shall terminate
//      as of the date such litigation is filed.
//
//   4. Redistribution. You may reproduce and distribute copies of the
//      Work or Derivative Works thereof in any medium, with or without
//      modifications, and in Source or Object form, provided that You
//      meet the following conditions:
//
//      (a) You must give any other recipients of the Work or
//          Derivative Works a copy of this License; and
//
//      (b) You must cause any modified files to carry prominent notices
//          stating that You changed the files; and
//
//      (c) You must retain, in the Source form of any Derivative Works
//          that You distribute, all copyright, patent, trademark, and
//          attribution notices from the Source form of the Work,
//          excluding those notices that do not pertain to any part of
//          the Derivative Works; and
//
//      (d) If the Work includes a "NOTICE" text file as part of its
//          distribution, then any Derivative Works that You distribute must
//          include a readable copy of the attribution notices contained
//          within such NOTICE file, excluding those notices that do not
//          pertain to any part of the Derivative Works, in at least one
//          of the following places: within a NOTICE text file distributed
//          as part of the Derivative Works; within the Source form or
//          documentation, if provided along with the Derivative Works; or,
//          within a display generated by the Derivative Works, if and
//          wherever such third-party notices normally appear. The contents
//          of the NOTICE file are for informational purposes only and
//          do not modify the License. You may add Your own attribution
//          notices within Derivative Works that You distribute, alongside
//          or as an addendum to the NOTICE text from the Work, provided
//          that such additional attribution notices cannot be construed
//          as modifying the License.
//
//      You may add Your own copyright statement to Your modifications and
//      may provide additional or different license terms and conditions
//      for use, reproduction, or distribution of Your modifications, or
//      for any such Derivative Works as a whole, provided Your use,
//      reproduction, and distribution of the Work otherwise complies with
//      the conditions stated in this License.
//
//   5. Submission of Contributions. Unless You explicitly state otherwise,
//      any Contribution intentionally submitted for inclusion in the Work
//      by You to the Licensor shall be under the terms and conditions of
//      this License, without any additional terms or conditions.
//      Notwithstanding the above, nothing herein shall supersede or modify
//      the terms of any separate license agreement you may have executed
//      with Licensor regarding such Contributions.
//
//   6. Trademarks. This License does not grant permission to use the trade
//      names, trademarks, service marks, or product names of the Licensor,
//      except as required for reasonable and customary use in describing the
//      origin of the Work and reproducing the content of the NOTICE file.
//
//   7. Disclaimer of Warranty. Unless required by applicable law or
//      agreed to in writing, Licensor provides the Work (and each
//      Contributor provides its Contributions) on an "AS IS" BASIS,
//      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
//      implied, including, without limitation, any warranties or conditions
//      of TITLE, NON-INFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A
//      PARTICULAR PURPOSE. You are solely responsible for determining the
//      appropriateness of using or redistributing the Work and assume any
//      risks associated with Your exercise of permissions under this License.
//
//   8. Limitation of Liability. In no event and under no legal theory,
//      whether in tort (including negligence), contract, or otherwise,
//      unless required by applicable law (such as deliberate and grossly
//      negligent acts) or agreed to in writing, shall any Contributor be
//      liable to You for damages, including any direct, indirect, special,
//      incidental, or consequential damages of any character arising as a
//      result of this License or out of the use or inability to use the
//      Work (including but not limited to damages for loss of goodwill,
//      work stoppage, computer failure or malfunction, or any and all
//      other commercial damages or losses), even if such Contributor
//      has been advised of the possibility of such damages.
//
//   9. Accepting Warranty or Additional Liability. While redistributing
//      the Work or Derivative Works thereof, You may choose to offer,
//      and charge a fee for, acceptance of support, warranty, indemnity,
//      or other liability obligations and/or rights consistent with this
//      License. However, in accepting such obligations, You may act only
//      on Your own behalf and on Your sole responsibility, not on behalf
//      of any other Contributor, and only if You agree to indemnify,
//      defend, and hold each Contributor harmless for any liability
//      incurred by, or claims asserted against, such Contributor by reason
//      of your accepting any such warranty or additional liability.
//
//   END OF TERMS AND CONDITIONS
//
//   APPENDIX: How to apply the Apache License to your work.
//
//      To apply the Apache License to your work, attach the following
//      boilerplate notice, with the fields enclosed by brackets "[]"
//      replaced with your own identifying information. (Don't include
//      the brackets!)  The text should be enclosed in the appropriate
//      comment syntax for the file format. We also recommend that a
//      file or class name and description of purpose be included on the
//      same "printed page" as the copyright notice for easier
//      identification within third-party archives.
//
//    Copyright 2017 Intel Corporation
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this project except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
