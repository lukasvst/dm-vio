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


#include "OakdS2.h"
#include <iostream>
#include <iomanip>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <utility>
#include "IMU/IMUTypes.h"
#include "sophus/se3.hpp"
#include "util/MinimalImage.h"

class now;

using namespace dmvio;
using std::vector;

dmvio::OakdS2::OakdS2(FrameContainer &frameContainer, std::string cameraCalibSavePath, DatasetSaver *
datasetSaver, uint32_t exposureTimeUs)
        : imuInt(frameContainer, datasetSaver), cameraCalibSavePath(std::move(cameraCalibSavePath)),
          saver(datasetSaver), exposureTimeUs(exposureTimeUs) {
    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("frames");
    // Properties
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setFps(30.0);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setFps(30.0);

//    monoLeft->initialControl.setManualExposure(exposureTimeUs, 150);
    stereo->setOutputRectified(true);
    stereo->setOutputDepth(false);
    stereo->setRectifyEdgeFillColor(0);
    stereo->setRectifyMirrorFrame(false);
    stereo->setExtendedDisparity(false);

    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20);
    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->rectifiedLeft.link(xout->input);
    imu->out.link(xout->input);
    // Connect to device and start pipeline
    device.startPipeline(pipeline);
}

void dmvio::OakdS2::start() {
    // Clear queue events
    readCalibration();
    auto newFrame = [&](const std::shared_ptr<dai::ADatatype> &frame) {
        if (!calibrationRead) {
            std::cout << "==============================not calibrationRead==============================" << std::endl;
            return;
        }
        if (dynamic_cast<dai::IMUData *>(frame.get())) {
            auto *motion = static_cast<dai::IMUData *>(frame.get());
            for (auto &imuPacket: motion->packets) {
                imuInt.addAccData({imuPacket.acceleroMeter.x, imuPacket.acceleroMeter.y, imuPacket.acceleroMeter.z},
                                  std::chrono::duration_cast<std::chrono::milliseconds>(
                                          imuPacket.acceleroMeter.timestamp.get().time_since_epoch()).count());
                imuInt.addGyrData({imuPacket.gyroscope.x, imuPacket.gyroscope.y, imuPacket.gyroscope.z},
                                  std::chrono::duration_cast<std::chrono::milliseconds>(
                                          imuPacket.gyroscope.timestamp.get().time_since_epoch()).count());
            }
        } else if (dynamic_cast<dai::ImgFrame *>(frame.get())) {
            auto *fs = static_cast<dai::ImgFrame *>(frame.get());
            double timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                    fs->getTimestamp().time_since_epoch()).count();
            // We somehow seem to get each image twice.
            if (undistorter && std::abs(timestamp - lastImgTimestamp) > 0.001) {
                cv::Mat mat = fs->getCvFrame();
                assert(mat.type() == CV_8U);
                // Multiply exposure by 1000, as we want milliseconds.
                double exposure = exposureTimeUs * 1e-3;
                if (saver) {
                    saver->addImage(mat, timestamp / 1000.0, exposure);
                }
                auto img = std::make_unique<dso::MinimalImageB>(mat.cols, mat.rows);
                memcpy(img->data, mat.data, mat.rows * mat.cols);
                // timestamp is in milliseconds, but shall be in seconds
                double finalTimestamp = timestamp;
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
    device.getOutputQueue("frames", 100, false)->addCallback(newFrame);
}

void printMatrix(std::vector<std::vector<float>> matrix) {
    using namespace std;
    std::string out = "[";
    for (auto row: matrix) {
        out += "[";
        for (auto val: row) out += to_string(val) + ", ";
        out = out.substr(0, out.size() - 2) + "]\n";
    }
    out = out.substr(0, out.size() - 1) + "]\n\n";
    cout << out;
}

void dmvio::OakdS2::readCalibration() {
    using namespace std;
    dai::CalibrationHandler calibData = device.readCalibration();
    // calibData.eepromToJsonFile(filename);
    std::vector<std::vector<float>> intrinsics;
    int width, height;
    cout << "Intrinsics from defaultIntrinsics function:" << endl;
    std::tie(intrinsics, width, height) = calibData.getDefaultIntrinsics(dai::CameraBoardSocket::LEFT);
    printMatrix(intrinsics);
    cout << "Width: " << width << endl;
    cout << "Height: " << height << endl;
    cout << "Stereo baseline distance: " << calibData.getBaselineDistance() << " cm" << endl;
    cout << "Mono FOV from camera specs: " << calibData.getFov(dai::CameraBoardSocket::LEFT)
         << ", calculated FOV: " << calibData.getFov(dai::CameraBoardSocket::LEFT, false) << endl;
    cout << "Intrinsics from getCameraIntrinsics function full resolution:" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::LEFT);
    printMatrix(intrinsics);
    cout << "Intrinsics from getCameraIntrinsics function 640 x 400:" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::LEFT, 640, 400);
    printMatrix(intrinsics);
    std::vector<std::vector<float>> extrinsics;
    printMatrix(extrinsics);
    calibrationRead = true;
    if (calibrationRead) return;
}

void OakdS2::setUndistorter(dso::Undistort *undistort) {
    this->undistorter = undistort;
}
