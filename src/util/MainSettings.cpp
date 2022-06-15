/**
* This file is part of DM-VIO.
*
* Copyright (c) 2022 Lukas von Stumberg <lukas dot stumberg at tum dot de>.
* for more information see <http://vision.in.tum.de/dm-vio>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* The methods parseArgument and settingsDefault are based on the file
* main_dso_pangolin.cpp of the project DSO written by Jakob Engel, but have been
* modified for the inclusion in DM-VIO. The original versions have the copyright
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
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

#include "MainSettings.h"
#include "dso/util/settings.h"

using namespace dmvio;
using namespace dso;

void MainSettings::parseArguments(int argc, char** argv, SettingsUtil& settingsUtil)
{
    for(int i = 1; i < argc; i++)
        parseArgument(argv[i], settingsUtil);
}


void MainSettings::parseArgument(char* arg, SettingsUtil& settingsUtil)
{
    int option;
    float foption;
    char buf[1000];

    // --------------------------------------------------
    // These are mostly the original DSO commandline arguments which also work for DM-VIO.
    // The DM-VIO specific settings can also be set with commandline arguments (and also with the yaml settings file)
    // and have been registered in the main function. See IMUSettings and its members for details.
    // --------------------------------------------------
    if(1 == sscanf(arg, "quiet=%d", &option))
    {
        if(option == 1)
        {
            setting_debugout_runquiet = true;
            printf("QUIET MODE, I'll shut up!\n");
        }
        return;
    }

    if(1 == sscanf(arg, "preset=%d", &option))
    {
        settingsDefault(option);
        return;
    }


    if(1 == sscanf(arg, "nolog=%d", &option))
    {
        if(option == 1)
        {
            setting_logStuff = false;
            printf("DISABLE LOGGING!\n");
        }
        return;
    }
    if(1 == sscanf(arg, "nogui=%d", &option))
    {
        if(option == 1)
        {
            disableAllDisplay = true;
            printf("NO GUI!\n");
        }
        return;
    }
    if(1 == sscanf(arg, "nomt=%d", &option))
    {
        if(option == 1)
        {
            multiThreading = false;
            printf("NO MultiThreading!\n");
        }
        return;
    }

    if(1 == sscanf(arg, "useimu=%d", &option))
    {
        if(option == 0)
        {
            printf("Disabling IMU integration!\n");
            setting_useIMU = false;
        }else if(option == 1)
        {
            printf("Enabling IMU integration!\n");
            setting_useIMU = true;
        }
        return;
    }

    if(1 == sscanf(arg, "save=%d", &option))
    {
        if(option == 1)
        {
            debugSaveImages = true;
            if(42 == system("rm -rf images_out"))
                printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
            if(42 == system("mkdir images_out"))
                printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
            if(42 == system("rm -rf images_out"))
                printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
            if(42 == system("mkdir images_out"))
                printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
            printf("SAVE IMAGES!\n");
        }
        return;
    }

    if(1 == sscanf(arg, "mode=%d", &option))
    {

        mode = option;
        if(option == 0)
        {
            printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
        }
        if(option == 1)
        {
            printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
        }
        if(option == 2)
        {
            printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_minGradHistAdd = 3;
        }
        return;
    }

    if(1 == sscanf(arg, "settingsFile=%s", buf))
    {
        YAML::Node settings = YAML::LoadFile(buf);
        settingsUtil.tryReadFromYaml(settings);
        printf("Loading settings from yaml file: %s!\n", buf);
        return;
    }

    if(settingsUtil.tryReadFromCommandLine(arg))
    {
        return;
    }

    printf("could not parse argument \"%s\"!!!!\n", arg);
    assert(0);
}

void MainSettings::registerArgs(SettingsUtil& set)
{
    set.registerArg("vignette", vignette);
    set.registerArg("gamma", gammaCalib);
    set.registerArg("calib", calib);
    set.registerArg("imuCalib", imuCalibFile);
    set.registerArg("speed", playbackSpeed);
    set.registerArg("preload", preload);

    // We don't register preset and mode as they will be handled in parseArgument.

    // Register global settings.
    set.registerArg("setting_minOptIterations", setting_minOptIterations);
    set.registerArg("setting_maxOptIterations", setting_maxOptIterations);
    set.registerArg("setting_minIdepth", setting_minIdepth);
    set.registerArg("setting_solverMode", setting_solverMode);
    set.registerArg("setting_weightZeroPriorDSOInitY", setting_weightZeroPriorDSOInitY);
    set.registerArg("setting_weightZeroPriorDSOInitX", setting_weightZeroPriorDSOInitX);
    set.registerArg("setting_forceNoKFTranslationThresh", setting_forceNoKFTranslationThresh);
    set.registerArg("setting_minFramesBetweenKeyframes", setting_minFramesBetweenKeyframes);

}

void dmvio::MainSettings::settingsDefault(int preset)
{
    printf("\n=============== PRESET Settings: ===============\n");
    if(preset == 0 || preset == 1)
    {
        printf("DEFAULT settings:\n"
               "- %s real-time enforcing\n"
               "- 2000 active points\n"
               "- 5-7 active frames\n"
               "- 1-6 LM iteration each KF\n"
               "- original image resolution\n", preset == 0 ? "no " : "1x");

        playbackSpeed = (preset == 0 ? 0 : 1.0);
        preload = preset == 1;
        setting_desiredImmatureDensity = 1500;
        setting_desiredPointDensity = 1000;
        setting_minFrames = 5;
        setting_maxFrames = 7;
        setting_maxOptIterations = 6;
        setting_minOptIterations = 1;

        setting_logStuff = false;
    }

    if(preset == 2 || preset == 3)
    {
        // Note: These presets were not tested with DM-VIO yet, you will probably need to adjust benchmarkSetting_width
        // and benchmarkSetting_height at least.
        printf("FAST settings:\n"
               "- %s real-time enforcing\n"
               "- 800 active points\n"
               "- 4-6 active frames\n"
               "- 1-4 LM iteration each KF\n"
               "- 424 x 320 image resolution\n", preset == 0 ? "no " : "5x");

        playbackSpeed = (preset == 2 ? 0 : 5);
        preload = preset == 3;
        setting_desiredImmatureDensity = 600;
        setting_desiredPointDensity = 800;
        setting_minFrames = 4;
        setting_maxFrames = 6;
        setting_maxOptIterations = 4;
        setting_minOptIterations = 1;

        benchmarkSetting_width = 424;
        benchmarkSetting_height = 320;

        setting_logStuff = false;
    }

    printf("==============================================\n");
}

