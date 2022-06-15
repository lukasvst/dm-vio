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

#include "IMUSettings.h"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>

using namespace dmvio;

IMUCalibration::IMUCalibration()
{
    initDefault();
}

// Directly pass the camchain.yaml (EuRoC format).
IMUCalibration::IMUCalibration(std::string settingsFilename)
{
    initDefault();

    loadFromFile(settingsFilename);
}

void IMUCalibration::initDefault()
{
    // Default Settings are for EuRoC.
    // Init T_cam_imu
    Eigen::Matrix3d m;
    Eigen::Vector3d p;
    m << 0.0148655429818, -0.999880929698, 0.00414029679422,
            0.999557249008, 0.0149672133247, 0.025715529948,
            -0.0257744366974, 0.00375618835797, 0.999660727178;
    p << -0.0216401454975, -0.064676986768, 0.00981073058949;

    Sophus::SE3d imu_cam(m, p);
    T_cam_imu = imu_cam.inverse();
}

void IMUCalibration::registerArgs(dmvio::SettingsUtil& set)
{
    set.registerArg("accelerometer_random_walk", sigma_between_b_a);
    set.registerArg("gyroscope_random_walk", sigma_between_b_g);
    set.registerArg("accelerometer_noise_density", accel_sigma);
    set.registerArg("gyroscope_noise_density", gyro_sigma);
    set.registerArg("integration_sigma", integration_sigma);
}

void IMUCalibration::loadFromFile(std::string settingsFilename)
{
    if(settingsFilename == "")
    {
        return;
    }

    std::cout << "Loading IMU parameter file at: " << settingsFilename << std::endl;
    YAML::Node config = YAML::LoadFile(settingsFilename)["cam0"];
    std::vector<std::vector<double> > theVector = config["T_cam_imu"].as<std::vector<std::vector<double> > >();
    Eigen::Matrix4d matrix;
    for(int x = 0; x < 4; ++x)
    {
        for(int y = 0; y < 4; ++y)
        {
            matrix(x, y) = theVector[x][y];
        }
    }
    std::cout << "Used T_cam_imu: " << std::endl << matrix << std::endl;
    T_cam_imu = Sophus::SE3d(matrix);

    if(config["accelerometer_random_walk"] || config["gyroscope_random_walk"] || config["accelerometer_noise_density"]  ||
    config["gyroscope_noise_density"])
    {
        std::cout << "WARNING IMPORTANT: Passing IMU noise values via the IMU camchain.yaml file is not supported any"
                     " more! Please pass them via the settings file or as a commandline parameter!" << std::endl;
    }

    std::cout << "Used noise values: " << sigma_between_b_a << " " << sigma_between_b_g << " " << accel_sigma << " "
              << gyro_sigma << std::endl;
}

void IMUCalibration::saveToFile(std::string filename)
{
    YAML::Node node;
    std::vector<std::vector<double>> vector(4, std::vector<double>(4, 0.0));
    Eigen::Matrix4d matrix = T_cam_imu.matrix();
    for(int x = 0; x < 4; ++x)
    {
        for(int y = 0; y < 4; ++y)
        {
            vector[x][y] = matrix(x, y);
        }
    }

    node["cam0"]["T_cam_imu"] = vector;

    std::ofstream stream(filename);
    stream << node;
}

IMUCalibration::IMUCalibration(const Sophus::SE3d& tCamImu) : T_cam_imu(tCamImu)
{}

void IMUSettings::registerArgs(dmvio::SettingsUtil& set)
{
    set.registerArg("resultsPrefix", resultsPrefix);

    set.registerArg("maxTimeBetweenInitFrames", maxTimeBetweenInitFrames);

    set.registerArg("skipFirstKeyframe", skipFirstKeyframe);

    set.registerArg("setting_weightDSOCoarse", setting_weightDSOCoarse);
    set.registerArg("setting_weightDSOToGTSAM", setting_weightDSOToGTSAM);
    set.registerArg("maxFrameEnergyThreshold", maxFrameEnergyThreshold);

    set.registerArg("dynamicWeightRMSEThresh", dynamicWeightRMSEThresh);
    set.registerArg("updateDynamicWeightDuringOptimization", updateDynamicWeightDuringOptimization);

    set.registerArg("setting_scaleFixTH", setting_scaleFixTH);
    set.registerArg("generalScaleIntervalSize", generalScaleIntervalSize);

    set.registerArg("numMeasurementsGravityInit", numMeasurementsGravityInit);

    set.registerArg("setting_optScaleBA", setting_optScaleBA);
    set.registerArg("setting_optGravity", setting_optGravity);
    set.registerArg("setting_optIMUExtrinsics", setting_optIMUExtrinsics);

    set.registerArg("setting_prior_bias", setting_prior_bias);
    set.registerArg("setting_prior_velocity", setting_prior_velocity);
    transformPriors.registerArgs(set, "");
    set.registerArg("gravityDirectionFixZ", gravityDirectionFixZ);

    set.registerArg("alwaysCanBreakIMU", alwaysCanBreakIMU);

    set.registerArg("useScaleDiagonalHack", useScaleDiagonalHack);

    set.registerArg("fixKeyframeDuringCoarseTracking", fixKeyframeDuringCoarseTracking);
    set.registerArg("addVisualToCoarseGraphIfTrackingBad", addVisualToCoarseGraphIfTrackingBad);

    set.registerArg("baToCoarseRotVariance", baToCoarseRotVariance);
    set.registerArg("baToCoarsePoseVariance", baToCoarsePoseVariance);
    set.registerArg("baToCoarseVelVariance", baToCoarseVelVariance);
    set.registerArg("baToCoarseAccBiasVariance", baToCoarseAccBiasVariance);
    set.registerArg("baToCoarseGyrBiasVariance", baToCoarseGyrBiasVariance);

    set.registerArg("setting_transferCovToCoarse", setting_transferCovToCoarse);
    set.registerArg("transferCovToCoarseMultiplier", transferCovToCoarseMultiplier);

    set.registerArg("setting_visualOnlyAfterScaleFixing", setting_visualOnlyAfterScaleFixing);

    initSettings.registerArgs(set);
}
