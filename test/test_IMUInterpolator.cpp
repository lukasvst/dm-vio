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


#include <gtest/gtest.h>
#include "live/IMUInterpolator.h"
#include "live/FrameContainer.h"

using namespace dmvio;

TEST(TestIMUInterpolator, SimpleTest)
{
    FrameContainer frameContainer;
    IMUInterpolator imuInt(frameContainer, nullptr);

    Frame frame0;
    frame0.imgTimestamp = 0.5;
    frameContainer.addFrame(std::move(frame0));

    auto pair = frameContainer.getImageAndIMUData().second;

    imuInt.addGyrData({1.0, 1.0, 1.0}, 2);
    imuInt.addAccData({1.0, 1.0, 1.0}, 1);
    imuInt.addAccData({2.0, 2.0, 2.0}, 3);
    // Should be interpolated to one acc measurement with 1.5

    imuInt.addAccData({5.0, 5.0, 5.0}, 4);
    imuInt.addGyrData({5.0, 5.0, 5.0}, 4);

    imuInt.addImage(nullptr, 5);

    imuInt.addGyrData({7.0, 7.0, 7.0}, 8);
    imuInt.addAccData({7.0, 7.0, 7.0}, 6);

    // 4, 5, 8 --> one quarter of the way from 5 to 7 --> 5.5

    auto imuData = frameContainer.getImageAndIMUData().second;

    EXPECT_EQ(imuData.size(), 3);

    // ensure that all measurements have valid acc and gyr data.
    for(auto&& data : imuData)
    {
        EXPECT_TRUE(data.getAccData().size() > 0);
        EXPECT_TRUE(data.getAccData().size() > 0);
    }


//    EXPECT_EQ(allData[0].timestamp, 2.0);
    EXPECT_EQ(imuData[0].getAccData()[0], 1.5);
    EXPECT_EQ(imuData[0].getGyrData()[0], 1.0);

//    EXPECT_EQ(allData[1].timestamp, 4.0);
    EXPECT_EQ(imuData[1].getAccData()[0], 5.0);
    EXPECT_EQ(imuData[1].getGyrData()[0], 5.0);

//    EXPECT_EQ(allData[2].timestamp, 5);
    EXPECT_EQ(imuData[2].getAccData()[0], 6.0);
    EXPECT_EQ(imuData[2].getGyrData()[0], 5.5);


    EXPECT_EQ(imuData[0].getIntegrationTime(), 1.5);
    EXPECT_EQ(imuData[1].getIntegrationTime(), 2.0);
    EXPECT_EQ(imuData[2].getIntegrationTime(), 1.0);
}

TEST(TestIMUInterpolator, ProblematicInputOrders)
{
    FrameContainer frameContainer;
    IMUInterpolator imuInt(frameContainer, nullptr);

    Frame frame0;
    frame0.imgTimestamp = 0.5;
    frameContainer.addFrame(std::move(frame0));

    auto pair = frameContainer.getImageAndIMUData().second;

    // First gyr data should not be put into the output, as it's impossible to interpolate acc data for it.
    imuInt.addGyrData({1.0, 1.0, 1.0}, 2);
    imuInt.addAccData({2.0, 2.0, 2.0}, 3);
    imuInt.addGyrData({2.0, 2.0, 2.0}, 4);
    imuInt.addAccData({3.0, 3.0, 3.0}, 5);

    imuInt.addImage(nullptr, 6);
    imuInt.addAccData({5.0, 5.0, 5.0}, 6);
    imuInt.addGyrData({5.0, 5.0, 5.0}, 6);

    auto imuData = frameContainer.getImageAndIMUData().second;

    EXPECT_EQ(imuData.size(), 2);
    EXPECT_EQ(imuData[0].getGyrData()[0], 2.0);
    EXPECT_EQ(imuData[0].getAccData()[0], 2.5);
    EXPECT_EQ(imuData[0].getIntegrationTime(), 3.5);
    EXPECT_EQ(imuData[1].getGyrData()[0], 5);
    EXPECT_EQ(imuData[1].getAccData()[0], 5);
    EXPECT_EQ(imuData[1].getIntegrationTime(), 2);
}

void addEmptyFrame(IMUInterpolator& imuInt, double time, float imuData)
{
    imuInt.addAccData({imuData, imuData, imuData}, time);
    imuInt.addGyrData({imuData, imuData, imuData}, time);
    imuInt.addImage(std::make_unique<dso::ImageAndExposure>(0, 0, time), time);
}

TEST(TestIMUInterpolator, TestFrameSkipping)
{
    FrameContainer frameContainer;
    IMUInterpolator imuInt(frameContainer, nullptr);

    imuInt.addAccData({0.0, 0.0, 0.0}, 0.0);
    imuInt.addGyrData({0.0, 0.0, 0.0}, 0.0);

    addEmptyFrame(imuInt, 1.0, 1.0);
    addEmptyFrame(imuInt, 2.0, 2.0);
    auto&& pair = frameContainer.getImageAndIMUData(10);
    EXPECT_EQ(pair.first->timestamp, 2.0);

    addEmptyFrame(imuInt, 3.0, 3.0);
    addEmptyFrame(imuInt, 4.0, 4.0);
    addEmptyFrame(imuInt, 5.0, 5.0);

    pair = frameContainer.getImageAndIMUData(1);
    EXPECT_EQ(pair.first->timestamp, 4.0);
    EXPECT_EQ(pair.second.size(), 2);
    EXPECT_EQ(pair.second[0].getAccData()[0], 3.0);
    EXPECT_EQ(pair.second[1].getAccData()[0], 4.0);

    pair = frameContainer.getImageAndIMUData(1);
    EXPECT_EQ(pair.first->timestamp, 5.0);
    EXPECT_EQ(pair.second.size(), 1);
    EXPECT_EQ(pair.second[0].getAccData()[0], 5.0);
}

// Test, where the first frame doesn't have IMU data yet.
TEST(TestIMUInterpolator, TestNoPrevAccData)
{
    FrameContainer frameContainer;
    IMUInterpolator imuInt(frameContainer, nullptr);

    // This image should also be skipped, because it comes before the IMU data.
    imuInt.addImage(std::make_unique<dso::ImageAndExposure>(0, 0, 0.5), 0.5);

    float imuData = 1.0f;
    imuInt.addAccData({imuData, imuData, imuData}, 1.2);
    imuInt.addGyrData({imuData, imuData, imuData}, 1.2);
    double time = 1.0;
    // This image should be skipped, because the IMU data starts only after it.
    imuInt.addImage(std::make_unique<dso::ImageAndExposure>(0, 0, time), time);

    imuInt.addAccData({3, 3, 3}, 1.5);
    imuInt.addGyrData({3, 3, 3}, 1.5);

    imuInt.addImage(std::make_unique<dso::ImageAndExposure>(0, 0, 2.0), 2.0);
    imuInt.addAccData({4, 4, 4}, 3.5);
    imuInt.addGyrData({4, 4, 4}, 3.5);

    auto pair = frameContainer.getImageAndIMUData(0); // don't skip frames.

    // Note that the IMU data of the first frame is not used (as there is no previous frame), but we test it anyway.
    EXPECT_EQ(pair.first->timestamp, 2.0);
    EXPECT_EQ(pair.second.size(), 3);
    EXPECT_EQ(pair.second[0].getAccData()[0], imuData);

    EXPECT_DOUBLE_EQ(pair.second[1].getIntegrationTime(), 0.3);
    EXPECT_EQ(pair.second[1].getAccData()[0], 3.0);

    EXPECT_EQ(pair.second[2].getIntegrationTime(), 0.5);
    EXPECT_EQ(pair.second[2].getAccData()[0], 3.25);
}
