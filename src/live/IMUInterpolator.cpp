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

#include "IMUInterpolator.h"
#include "util/ImageAndExposure.h"
#include "FrameContainer.h"
#include "util/TimeMeasurement.h"
#include <iostream>
#include <iomanip>
#include <algorithm>

using std::vector;

dmvio::IMUInterpolator::IMUInterpolator(dmvio::FrameContainer& frameContainer, DatasetSaver* datasetSaver)
        : frameContainer(frameContainer), saver(datasetSaver)
{}

void dmvio::IMUInterpolator::addAccData(vector<float> data, double timestamp)
{
    dmvio::TimeMeasurement measurement("IMUInterpolator::addAccData");
    std::unique_lock<std::mutex> lock(mutex);
    accData.emplace_back(data, timestamp);

    if(timestamp < lastAccTimestamp)
    {
        throw std::invalid_argument("ERROR: Timestamp out of order: SHOULD NOT HAPPEN!");
    }
    lastAccTimestamp = timestamp;

    insertAccDataIfNecessary();

    if(accData.size() > maxIMUQueueSize)
    {
        int numDel = accData.size() - maxIMUQueueSize;
        accData.erase(accData.begin(), accData.begin() + numDel);
    }

    trySendingImages();
}

void dmvio::IMUInterpolator::insertAccDataIfNecessary()
{
    for(auto it = output.begin(); it != output.end();)
    {
        auto&& data = *it;
        if(!data.accSet)
        {
            auto pair = interpolateDataFromArray(accData, data.timestamp);

            if(pair.second == IMUInterpolationResult::TIMESTAMP_TOO_EARLY)
            {
                // This output is too early, we need to remove it!
                it = output.erase(it, it + 1);
                continue; // to not execute it++;
            }else if(pair.second == IMUInterpolationResult::NOT_AVAILABLE_YET)
            {
                // There is no IMU data yet --> break and try again next time.
                break;
            }

            data.accData = std::move(pair.first);
            data.accSet = true;
        }
        it++;
    }
    if(!saver) return;
    // Save IMU data to file.
    for(auto&& data : output)
    {
        if(data.saveStatus != IMUDataDuringInterpolation::DONT_SAVE)
        {
            if(!data.accSet || !data.gyrSet)
            {
                // Not set yet --> stop saving.
                break;
            }
            if(data.saveStatus == IMUDataDuringInterpolation::SHALL_SAVE)
            {
                saver->addIMUData(data.timestamp, data.accData, data.gyrData);
                data.saveStatus = IMUDataDuringInterpolation::SAVED;
            }
        }
    }
}

void dmvio::IMUInterpolator::addGyrData(vector<float> data, double timestamp)
{
    dmvio::TimeMeasurement measurement("IMUInterpolator::addGyrData");
    std::unique_lock<std::mutex> lock(mutex);
    gyrData.emplace_back(data, timestamp);

    insertGyrDataIfNecessary();


    auto it = std::find_if(output.begin(), output.end(), [timestamp](const IMUDataDuringInterpolation& data)
    {
        return data.timestamp == timestamp;
    });
    // Maybe there is already an image with this timestamp --> just insert.
    if(it == output.end())
    {
        output.emplace_back(timestamp);
        it = output.end() - 1;
    }
    it->gyrData = data;
    it->gyrSet = true;
    it->saveStatus = IMUDataDuringInterpolation::SHALL_SAVE;

    if(timestamp < lastGyrTimestamp)
    {
        throw std::invalid_argument("ERROR: Timestamp out of order: SHOULD NOT HAPPEN!");
    }
    lastGyrTimestamp = timestamp;

    if(gyrData.size() > maxIMUQueueSize)
    {
        int numDel = gyrData.size() - maxIMUQueueSize;
        gyrData.erase(gyrData.begin(), gyrData.begin() + numDel);
    }

    trySendingImages();
}

void dmvio::IMUInterpolator::insertGyrDataIfNecessary()
{
    // Check whether previous data needs gyr information.
    for(auto it = output.begin(); it != output.end();)
    {
        auto&& data = *it;
        if(!data.gyrSet)
        {
            auto pair = interpolateDataFromArray(gyrData, data.timestamp);

            if(pair.second == IMUInterpolationResult::TIMESTAMP_TOO_EARLY)
            {
                // This output is too early, we need to remove it!
                it = output.erase(it, it + 1);
                continue; // to not execute it++;
            }else if(pair.second == IMUInterpolationResult::NOT_AVAILABLE_YET)
            {
                // There is no IMU data yet --> break and try again next time.
                break;
            }

            data.gyrData = std::move(pair.first);
            data.gyrSet = true;
        }
        it++;
    }
}

vector<float> dmvio::interpolateData(const PartialIMUData& data1, const PartialIMUData& data2, double timestamp)
{
    double firstTime = data1.timestamp;
    double secondTime = data2.timestamp;

    double secondMult = (timestamp - firstTime) / (secondTime - firstTime);
    double firstMult = 1.0 - secondMult;

    std::vector<float> data;
    for(int i = 0; i < 3; ++i)
    {
        data.push_back(firstMult * data1.data[i] + secondMult * data2.data[i]);
    }

    return data;
}

std::pair<std::vector<float>, dmvio::IMUInterpolationResult>
dmvio::interpolateDataFromArray(const vector<PartialIMUData>& array, double timestamp)
{
    auto it = std::lower_bound(array.begin(), array.end(), PartialIMUData(std::vector<float>{}, timestamp));

    if(it == array.end())
    {
        return std::make_pair(std::vector<float>{}, IMUInterpolationResult::NOT_AVAILABLE_YET);
    }

    // Now it->timestamp >= timestamp.
    if(it == array.begin())
    {
        if(it->timestamp == timestamp)
        {
            return std::make_pair(it->data, IMUInterpolationResult::FOUND);
        }else
        {
            return std::make_pair(std::vector<float>{}, IMUInterpolationResult::TIMESTAMP_TOO_EARLY);
        }
    }

    return std::make_pair(interpolateData(*(it - 1), *it, timestamp), IMUInterpolationResult::FOUND);
}

void dmvio::IMUInterpolator::addImage(std::unique_ptr<dso::ImageAndExposure> image, double timestamp)
{
    dmvio::TimeMeasurement measurement("IMUInterpolator::addImage");
    std::unique_lock<std::mutex> lock(mutex);

    auto it = std::find_if(output.begin(), output.end(), [timestamp](const IMUDataDuringInterpolation& data)
    {
        return data.timestamp == timestamp;
    });
    // If there's already an IMU data for this timestamp we don't need to insert it.
    if(it == output.end())
    {
        output.emplace_back(timestamp);
    }

    insertAccDataIfNecessary();
    insertGyrDataIfNecessary();

    imagesInProcess.emplace_back(std::move(image), timestamp);

    trySendingImages();

}

void dmvio::IMUInterpolator::trySendingImages()
{
    std::sort(output.begin(), output.end());

    while(!imagesInProcess.empty())
    {
        auto&& frame = imagesInProcess[0];
        bool finished = true;

        int removeNum = 0; // remove removable items at the beginning of output.
        for(auto&& data : output)
        {
            if(data.timestamp > frame.imgTimestamp)
            {
                // Later than the image --> this image is finished.
                break;
            }

            if(!data.accSet || !data.gyrSet)
            {
                // No IMU data available yet --> try sending this image later.
                finished = false;
                break;
            }

            frame.imuData.push_back(data);
            frame.imuData[frame.imuData.size() - 1].timestamp = data.timestamp;

            removeNum++;
        }

        if(removeNum > 0)
        {
            output.erase(output.begin(), output.begin() + removeNum);
        }

        if(!finished)
        {
            break;
        }

        // If the frame came before the first IMU measurement it can happen that it does not have corresponding IMU
        // data. In that case we don't send it, and just delete it.
        if(!frame.imuData.empty() && frame.imuData.back().timestamp == frame.imgTimestamp)
        {
            frameContainer.addFrame(std::move(frame));
        }else
        {
            std::cout << "WARNING: Not sending frame, because it does not have IMU data yet." << std::endl;
        }
        imagesInProcess.pop_front();
    }
}

dmvio::PartialIMUData::PartialIMUData(
        const vector<float>& data,
        double timestamp) : data(data), timestamp(timestamp)
{}

bool dmvio::PartialIMUData::operator<(const dmvio::PartialIMUData& other) const
{
    return timestamp < other.timestamp;
}

dmvio::IMUDataDuringInterpolation::IMUDataDuringInterpolation(double timestamp) : timestamp(timestamp)
{
    gyrSet = false;
    accSet = false;
}

bool dmvio::IMUDataDuringInterpolation::operator<(const dmvio::IMUDataDuringInterpolation& other) const
{
    return timestamp < other.timestamp;
}

