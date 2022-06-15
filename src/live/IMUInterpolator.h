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

#ifndef DMVIO_IMUINTERPOLATOR_H
#define DMVIO_IMUINTERPOLATOR_H

#include <vector>
#include <deque>
#include <fstream>
#include <map>
#include "FrameContainer.h"
#include "DatasetSaver.h"
#include <mutex>

namespace dmvio
{

// Contains either accelerometer or gyroscope data recorded at a particular timestamp.
class PartialIMUData
{
public:
    // data contains x,y,z data of the sensor.
    PartialIMUData(const std::vector<float>& data, double timestamp);

    bool operator<(const PartialIMUData& other) const;

public:
    std::vector<float> data;
    double timestamp;
};


// Interpolate between two data points.
std::vector<float> interpolateData(const PartialIMUData& data1, const PartialIMUData& data2, double timestamp);

enum class IMUInterpolationResult
{
    FOUND,                  // IMU data interpolated and returned.
    TIMESTAMP_TOO_EARLY,    // The passed timestamp is before all timestamps in array. It will need to be skipped.
    NOT_AVAILABLE_YET       // The passed timestamp is after all timestamps in array. IMU data for it will arrive later.
};
// Compute interpolated IMU measurement from a *sorted* array of measurements.
// Finds the nearest two data points and calls interpolateData on them.
std::pair<std::vector<float>, IMUInterpolationResult>
interpolateDataFromArray(const std::vector<PartialIMUData>& array, double timestamp);

// Supports live interpolating IMU data to fit the image data (meaning there should be an interpolated IMU measurement
// for each image timestamp).
// It is designed for cases where accelerometer and gyroscope data are not synchronized (meaning they arrive with
// different timestamps). This means that all accelerometer data is interpolated to fit the timestamps of the
// gyroscope data).
// It can also handle cases where the IMU data arrives only after the corresponding image.
class IMUInterpolator
{
public:
    // A reference to frameContainer is kept. This is where IMU data and images are sent.
    IMUInterpolator(FrameContainer& frameContainer, DatasetSaver* datasetSaver);

    // Shall be called everytime accelerometer data arrives.
    void addAccData(std::vector<float> data, double timestamp);
    // Shall be called everytime gyroscope data arrives.
    void addGyrData(std::vector<float> data, double timestamp);

    // Called when a new image arrives. Will be forwarded to the frameContainer, as soon as the IMU data for it has
    // arrived.
    void addImage(std::unique_ptr<dso::ImageAndExposure> image, double timestamp);

private:

    FrameContainer& frameContainer;
    DatasetSaver* saver = nullptr; // also save IMU data to file.

    // Protects all methods.
    std::mutex mutex;

    std::vector<PartialIMUData> accData; // Contains all acceleration data (plus timestamp)
    std::vector<PartialIMUData> gyrData; // Contains all gyroscope data (plus timestamp)

    // Contains all (individual) interpolated IMU measurements (Acc + Gyr + Timestamp)
    // This class works by first storing IMU data in accData and gyrData. Then for all gyr data and all images, an
    // entry in output is created. The methods insertAccDataIfNecessary and addGyrDataIfNecessary fill in missing IMU
    // data in output by interpolating the nearest measurement inside accData/gyrData.
    std::deque<IMUDataDuringInterpolation> output;

    double lastAccTimestamp = 0.0;
    double lastGyrTimestamp = 0.0;

    // You should get a lock on mutex before calling this method.
    // For all entries in output with accDataSet==false, this method adds an (interpolated) accelerometer measurement
    // if possible.
    void insertAccDataIfNecessary();
    // You should get a lock on mutex before calling this method.
    // For all entries in output with gyrDataSet==false, this method adds an (interpolated) gyroscope measurement
    // if possible.
    void insertGyrDataIfNecessary();

    // Send all imagesInProcess to frameContainer for which IMU data exists.
    // You should get a lock on mutex before calling this method.
    void trySendingImages();

    // These images have arrived but could not yet be send to FrameContainer as they are missing IMU data.
    std::deque<Frame> imagesInProcess;

    // Maximum number of IMU measurements stored in accData/gyrData before IMU interpolation.
    static constexpr int maxIMUQueueSize = 25;
};
}


#endif //DMVIO_IMUINTERPOLATOR_H
