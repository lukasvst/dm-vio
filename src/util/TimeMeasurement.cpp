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

#include "TimeMeasurement.h"
#include <iostream>

using namespace dmvio;
using namespace std::chrono;


std::map<std::string, dmvio::MeasurementLog> dmvio::TimeMeasurement::logs = std::map<std::string, MeasurementLog>();
bool dmvio::TimeMeasurement::saveFileOpen = false;

dmvio::TimeMeasurement::TimeMeasurement(std::string name)
        : name(name)
{
    begin = high_resolution_clock::now();
}

dmvio::TimeMeasurement::~TimeMeasurement()
{
    end();
}

double dmvio::TimeMeasurement::end()
{
    if(ended)
    {
        return -1;
    }

    auto end = high_resolution_clock::now();
    double duration = duration_cast<std::chrono::duration<double>>(end - begin).count();

    logs[name].addMeasurement(duration);

    ended = true;

    return duration;
}

void dmvio::TimeMeasurement::saveResults(std::string filename)
{
    std::ofstream saveFile;
    saveFile.open(filename);

    for(const auto& pair : logs)
    {
        saveFile << pair.first << ' ' << pair.second << '\n';
    }
    saveFile.close();
}

void dmvio::TimeMeasurement::cancel()
{
    ended = true;
}

void dmvio::MeasurementLog::addMeasurement(double time)
{
    if(num == 0)
    {
        first = time;
    }
    sum += time;
    num++;

    double shifted = time - first;
    sumShifted += shifted;
    sumSquared += shifted * shifted;

    if(time > max)
    {
        max = time;
    }
}

void dmvio::MeasurementLog::writeLogLine(std::ostream& stream) const
{
    double mean = getMean();
    double variance = getVariance();

    stream << mean << ' ' << variance << ' ' << max << ' ' << num;

}

double dmvio::MeasurementLog::getVariance() const
{
    double variance = (sumSquared - (sumShifted * sumShifted) / num) / (num - 1);
    return variance;
}

double dmvio::MeasurementLog::getMean() const
{
    double mean = sum / (double) num;
    return mean;
}

double dmvio::MeasurementLog::getMax() const
{
    return max;
}

int dmvio::MeasurementLog::getNum() const
{
    return num;
}

std::ostream& operator<<(std::ostream& os, const dmvio::MeasurementLog& obj)
{
    obj.writeLogLine(os);
    return os;
}
