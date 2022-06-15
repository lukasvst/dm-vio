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

#include "DatasetSaver.h"
#include <boost/filesystem.hpp>
#include <thread>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <iomanip>

dmvio::DatasetSaver::DatasetSaver(std::string saveFolder)
{
    // Throw exception if folder exists!
    if(boost::filesystem::exists(saveFolder))
    {
        throw boost::filesystem::filesystem_error("Folder already exists.", boost::system::error_code());
    }
    boost::filesystem::create_directory(saveFolder);
    boost::filesystem::path savePath(saveFolder);
    boost::filesystem::create_directory(savePath / "cam0");

    imgSaveFolder = (savePath / "cam0").string();

    timesFile.open((savePath / "times.txt").string());
    imuFile.open((savePath / "imu_orig.txt").string());

    imageSaveThread = std::thread{&DatasetSaver::saveImagesWorker, this};

}

void dmvio::DatasetSaver::saveImagesWorker()
{
    while(running)
    {
        std::tuple<cv::Mat, double, double> tuple;
        {
            std::unique_lock<std::mutex> lock(mutex);
            while(imageQueue.size() == 0)
            {
                if(!running) return;
                frameArrivedCond.wait(lock);
            }

            tuple = std::move(imageQueue.front());
            imageQueue.pop_front();

            if(imageQueue.size() > 1)
            {
                std::cout << "Save image queue size: " << imageQueue.size() << std::endl;
            }
        }

        double timestamp = std::get<1>(tuple);
        long long id = static_cast<long long>(timestamp * 1e9);

        std::stringstream filename;
        filename << imgSaveFolder << "/" << id << ".jpg";

        timesFile << id << " " << std::fixed << timestamp << " " << std::get<2>(tuple) << "\n";

        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 99}; // jpg quality.
        cv::imwrite(filename.str(), std::get<0>(tuple), compression_params);
    }
}

void dmvio::DatasetSaver::addImage(cv::Mat mat, double timestamp, double exposure)
{
    {
        std::unique_lock<std::mutex> lock(mutex);
        imageQueue.emplace_back(mat, timestamp, exposure);
    }
    frameArrivedCond.notify_all();
}

void dmvio::DatasetSaver::addIMUData(double timestamp, std::vector<float> accData, std::vector<float> gyrData)
{
    imuFile << static_cast<long long>(timestamp * 1e9);
    for(int i = 0; i < 3; ++i)
    {
        imuFile << " " << gyrData[i];
    }
    for(int i = 0; i < 3; ++i)
    {
        imuFile << " " << accData[i];
    }
    imuFile << "\n";
}

void dmvio::DatasetSaver::end()
{
    running = false;
    frameArrivedCond.notify_all();
    imageSaveThread.join();
}

