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

#include <iomanip>
#include <dso/util/settings.h>
#include "FrameContainer.h"
#include "IMUInterpolator.h"
#include "IMU/IMUTypes.h"

std::pair<std::unique_ptr<dso::ImageAndExposure>, dmvio::IMUData>
dmvio::FrameContainer::getImageAndIMUData(int maxSkipFrames)
{
    std::unique_lock<std::mutex> lock(framesMutex);
    while(frames.size() == 0 && !stopSystem) // Wait for new image.
    {
        frameArrivedCond.wait(lock);
    }

    if(stopSystem) return std::make_pair(nullptr, dmvio::IMUData{});

    IMUData imuData;

    // Skip frames if necessary.
    // Now frames.size() must be greater than 0.
    size_t useFrame = frames.size() - 1;
    int numFramesAfter = 0;
    if(frames.size() > 1)
    {
        int framesToSkip = frames.size() - 1; // also the index of the frame that will be used.
        if(maxSkipFrames >= 0 && maxSkipFrames < framesToSkip)
        {
            framesToSkip = maxSkipFrames;
        }
        useFrame = framesToSkip;
        numFramesAfter = frames.size() - useFrame - 1;
        if(!dso::setting_debugout_runquiet)
        {
            std::cout << "SKIPPING " << framesToSkip << " FRAMES!" << " frames remaining in queue: "
                      << numFramesAfter << std::endl;
        }
    }

    auto returnImg = std::move(frames[useFrame].img);

    // Fill IMU data to return, also consider IMU data for skipped frames.
    for(int j = 0; j <= useFrame; ++j)
    {
        std::vector<dmvio::IMUDataDuringInterpolation>& data = frames[j].imuData;
        for(int i = 0; i < data.size(); ++i)
        {
            Eigen::Vector3d acc, gyr;
            for(int x = 0; x < 3; ++x)
            {
                acc(x) = data[i].accData[x];
                gyr(x) = data[i].gyrData[x];
            }

            double integrationTime = 0.0;

            integrationTime = data[i].timestamp - prevTimestamp;
            assert(integrationTime >= 0);

            imuData.emplace_back(acc, gyr, integrationTime);

            prevTimestamp = data[i].timestamp;
        }
    }
    if(prevTimestamp < 0.0)
    {
        prevTimestamp = frames[useFrame].imgTimestamp;
    }
    assert(std::abs(frames[useFrame].imgTimestamp - prevTimestamp) < 0.0001);
    frames.erase(frames.begin(), frames.begin() + useFrame + 1);
    assert(frames.size() == numFramesAfter);

    return std::make_pair(std::move(returnImg), imuData);
}

void dmvio::FrameContainer::addFrame(Frame frame)
{
    {
        std::unique_lock<std::mutex> lock(framesMutex);
        frames.push_back(std::move(frame));
    }
    frameArrivedCond.notify_all();
}

int dmvio::FrameContainer::getQueueSize()
{
    std::unique_lock<std::mutex> lock(framesMutex);
    return frames.size();
}

void dmvio::FrameContainer::stop()
{
    stopSystem = true;
    frameArrivedCond.notify_all();
}

dmvio::Frame::Frame(std::unique_ptr<dso::ImageAndExposure>&& img, double imgTimestamp) : img(std::move(img)),
                                                                                         imgTimestamp(imgTimestamp)
{}
