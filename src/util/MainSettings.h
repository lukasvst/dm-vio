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

// This file contains common utils used for the main files.

#ifndef DMVIO_MAINSETTINGS_H
#define DMVIO_MAINSETTINGS_H

#include <util/SettingsUtil.h>

namespace dmvio
{

// Parses the main commandline arguments needed for DM-VIO and forwards all other arguments to the SettingsUtil.
class MainSettings
{
public:
    // Parse all commandline arguments. Unknown arguments will be forwarded to settingsUtil.
    void parseArguments(int argc, char** argv, SettingsUtil& settingsUtil);

    // Parse a single argument. Unknown arguments will be forwarded to settingsUtil.
    void parseArgument(char* arg, dmvio::SettingsUtil& settingsUtil);

    // Register args for these settings and for global DSO settings.
    void registerArgs(dmvio::SettingsUtil& set);

    std::string vignette = "";
    std::string gammaCalib = "";
    std::string calib = "";
    std::string imuCalibFile = "";

    // only relevant for datasets.
    float playbackSpeed = 0;    // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.
    bool preload = false;

    // 0 means photometric calibration (exposure times, vignette and response calibration) is available, 1 means no
    // photometric calibration there.
    // Note that the vignette will only be used if set to 0.
    int mode = 0;

    void settingsDefault(int preset);
};

}


#endif //DMVIO_MAINSETTINGS_H
