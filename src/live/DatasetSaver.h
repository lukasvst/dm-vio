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

#ifndef DMVIO_DATASETSAVER_H
#define DMVIO_DATASETSAVER_H

#include <string>
#include <opencv2/core/mat.hpp>
#include <mutex>
#include <deque>
#include <fstream>
#include <thread>
#include <condition_variable>

namespace dmvio
{

// Helper for recording live data to file.
class DatasetSaver
{
public:
    DatasetSaver(std::string saveFolder);

    // timestamp in seconds, exposure in milliseconds.
    void addImage(cv::Mat mat, double timestamp, double exposure);

    void addIMUData(double timestamp, std::vector<float> accData, std::vector<float> gyrData);

    void saveImagesWorker();

    void end();

private:
    std::string imgSaveFolder;

    std::ofstream timesFile, imuFile;

    std::thread imageSaveThread;

    // protects image queue.
    std::mutex mutex;
    std::condition_variable frameArrivedCond;
    std::deque<std::tuple<cv::Mat, double, double>> imageQueue;

    bool running = true;

};


}
#endif //DMVIO_DATASETSAVER_H
