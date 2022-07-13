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

#ifndef DMVIO_OAKDS2_H
#define DMVIO_OAKDS2_H

#include <depthai/depthai.hpp>
#include <boost/thread.hpp>
#include "IMU/IMUSettings.h"
#include "live/IMUInterpolator.h"
#include "FrameContainer.h"
#include "util/Undistort.h"
#include "DatasetSaver.h"

namespace dmvio {
// Class for interacting with the RealsenseT265 camera.
    class OakdS2 {
    public:
        // Images and IMU data will be passed into frameContainer which can be used to get synchronized image and IMU data.
        // Factory calibration will be saved to cameraCalibSavePath.
        // If datasetSaver is set, the IMU data and images will also be saved to file.
        OakdS2(FrameContainer &frameContainer, std::string cameraCalibSavePath, DatasetSaver *datasetSaver,uint32_t exposureTimeUs);

        // Start receiving data.
        void start();

        // Set the undistorter to use. Until this is set, no images are passed forward to the frameContainer.
        void setUndistorter(dso::Undistort *undistort);

        std::unique_ptr<IMUCalibration> imuCalibration;
    private:
        void readCalibration();
        dai::Device device;
        dai::Pipeline pipeline;
        std::string cameraCalibSavePath;

        // Use left camera.
        int useCam = 0;
        uint32_t exposureTimeUs;
        std::atomic<bool> calibrationRead{false};
        boost::thread dataThread;
        // IMU interpolator will take care of creating "fake measurements" to synchronize the sensors by interpolating IMU data.
        IMUInterpolator imuInt;
        dso::Undistort *undistorter = nullptr;
        double lastImgTimestamp = -1.0;
        DatasetSaver *saver;
    };

}

#endif //DMVIO_OAKDS2_H
