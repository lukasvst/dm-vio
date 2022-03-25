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

#ifndef DMVIO_TIMEMEASUREMENT_H
#define DMVIO_TIMEMEASUREMENT_H

#include <string>
#include <fstream>
#include <chrono>
#include <map>
#include <memory>


namespace dmvio
{
// Saves mean, maximum, and variance.
class MeasurementLog
{
public:
    MeasurementLog() = default;

    void addMeasurement(double time);

    void writeLogLine(std::ostream& stream) const;

    int getNum() const;
    double getMax() const;
    double getMean() const;
    double getVariance() const;
private:
    double sum{0};
    double max{0};
    int num{0};

    double first{-1};
    // For computing stddev we shift by the first sample.
    double sumShifted{0};
    double sumSquared{0};

};

// Used to measure and log wall time for different code parts.
// Note that this class is only partially thread-safe, meaning there should not be measurements with the same name
// in different threads, otherwise there can be an endless loop / segfault in the first call!
class TimeMeasurement final
{
public:
    TimeMeasurement(std::string name);
    TimeMeasurement(const TimeMeasurement&) = delete;
    ~TimeMeasurement();

    // End the measurement interval. Optional, if not called the destructor will call it.
    double end();

    // Cancel the time measurement.
    void cancel();

    static void saveResults(std::string filename);

private:
    static bool saveFileOpen;
    static std::ofstream saveFile;
    static std::map<std::string, MeasurementLog> logs;

    std::string name;
    std::chrono::high_resolution_clock::time_point begin;
    bool ended{false};
};
}

std::ostream& operator<<(std::ostream& os, const dmvio::MeasurementLog& obj);


#endif //DMVIO_TIMEMEASUREMENT_H
