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

#ifndef DMVIO_FRAMECONTAINER_H
#define DMVIO_FRAMECONTAINER_H

#include <Eigen/Dense>
#include "util/ImageAndExposure.h"
#include "IMU/IMUTypes.h"
#include <Eigen/StdVector>
#include <mutex>
#include <condition_variable>
#include <deque>

namespace dmvio
{
// Data structure for IMU data during interpolation (see also IMUInterpolator.h).
class IMUDataDuringInterpolation
{
public:
    IMUDataDuringInterpolation(double timestamp);

    bool operator<(const IMUDataDuringInterpolation& other) const;

    enum SaveStatus
    {
        DONT_SAVE, SHALL_SAVE, SAVED
    };

    std::vector<float> accData; // Accelerometer data at a specific timestamp (usually 3 elements for x,y,z).
    std::vector<float> gyrData; // Gyroscope data at a specific timestamp (usually 3 elements for x,y,z).
    double timestamp;           // Timestamp of this IMU data sample.
    bool gyrSet;                // True if gyroscope data has been set.
    bool accSet;                // True if accelerometer data has been set.
    SaveStatus saveStatus = DONT_SAVE; // only relevant for saving IMU data to file while running.
};

// Image frame and corresponding IMU imu data for storage in the FrameContainer.
class Frame
{
public:
    Frame() = default;
    Frame(std::unique_ptr<dso::ImageAndExposure>&& img, double imgTimestamp);
    std::unique_ptr<dso::ImageAndExposure> img;
    std::vector<IMUDataDuringInterpolation> imuData;
    double imgTimestamp = 0.0;
};

// Used to store (addFrame) and retrieve (getImageAndIMUData) images and corresponding IMU data asynchronously.
// Also contains logic to skip frames if necessary.
class FrameContainer
{
public:
    FrameContainer() = default;

    // Retrieve the newest image and corresponding IMU data.
    // Will wait until a new image arrives if no image is in the queue.
    // If there is more than one image in the queue it will skip maxSkipFrames (if maxSkipFrames is >= 0).
    // If maxSkipFrames it will always skip to the newest image.
    std::pair<std::unique_ptr<dso::ImageAndExposure>, IMUData> getImageAndIMUData(int maxSkipFrames = -1);

    // Returns the number of images in the queue.
    int getQueueSize();

    // Adds a new image and corresponding IMU data to the queue. Can be called in a different thread than the calls
    // to getImageAndIMUData.
    void addFrame(Frame frame);

    // Can be used to stop a call to getImageAndIMUData and return an empty image.
    void stop();

private:
    std::mutex framesMutex; // Protects the frames array.
    std::condition_variable frameArrivedCond;

    std::deque<Frame> frames;

    double prevTimestamp = -1.0; // timestamp of last measurement.

    bool stopSystem = false;
};
}


#endif //DMVIO_FRAMECONTAINER_H
