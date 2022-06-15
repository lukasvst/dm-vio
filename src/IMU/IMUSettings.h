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

#ifndef DMVIO_IMUSETTINGS_H
#define DMVIO_IMUSETTINGS_H

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <util/SettingsUtil.h>
#include <gtsam/inference/Ordering.h>
#include <GTSAMIntegration/PoseTransformation.h>
#include "IMUInitialization/IMUInitSettings.h"

namespace dmvio
{

// Contains various IMU related settings.
// There are the following ways to set them (where the latter ones overwrite the former ones):
// - Default values set in the code (here or in the constructor).
// - Values set using a settings.yaml file.
// - Values set using commandline arguments.
class IMUSettings
{
public:
    void registerArgs(dmvio::SettingsUtil& set);

    // Prefix for all results files.
    std::string resultsPrefix = "";

    IMUInitSettings initSettings; // settings for the IMU initializer.
    double maxTimeBetweenInitFrames = 100000.0; // Maximum time between the first 2 frames for DSO.

    // Don't add IMU data between the first two keyframes. Should not be set when the IMU initializer is active (unless
    // disableVIOUntilFirstInit=false).
    bool skipFirstKeyframe = false;

    // Weight wrt DSO.
    double setting_weightDSOCoarse = 1.0 / 1000; // DSO weight for coarse tracking.
    double setting_weightDSOToGTSAM = 1.0 / 60000;// DSO weight for BA.
    float maxFrameEnergyThreshold = 5000; // Maximum energy threshold for DSO.

    // ----------- BA Settings -----------
    // Settings regarding dynamic photometric weight.
    double dynamicWeightRMSEThresh = 8.0;
    bool updateDynamicWeightDuringOptimization = true;

    // When the scale changes less than this threshold over generalScaleIntervalSize optimizations we fix it.
    // Disabled by default but could be usefull for some applications. 0.005 or 0.007 are good values for it.
    double setting_scaleFixTH = 0;
    int generalScaleIntervalSize = 60;

    // Maximum number of measurements to include for the simple gravity initializer.
    int numMeasurementsGravityInit = 40;

    // Settings what to optimize in the main BA.
    bool setting_optScaleBA = true;
    bool setting_optGravity = true;
    bool setting_optIMUExtrinsics = false;

    // Settings regarding priors.
    bool setting_prior_bias = false; // Only relevant if disableVIOUntilFirstInit=false
    bool setting_prior_velocity = false; // Only relevant if disableVIOUntilFirstInit=false
    IMUTransformPriorSettings transformPriors; // Prior settings for gravity and IMU extrinsics.
    bool gravityDirectionFixZ = true; // Fix z-axis of gravity direction (as yaw is not observable).

    // Don't include IMU variables when calculating whether the BA optimization can break.
    bool alwaysCanBreakIMU = false;

    bool useScaleDiagonalHack = false; // This can be used to improve performance when the initial scale is very far from optimum.

    // ----------- Settings for Coarse Tracking -----------
    bool fixKeyframeDuringCoarseTracking = true;
    bool addVisualToCoarseGraphIfTrackingBad = false; // Add visual factor even if tracking is bad.

    // Priors from BA when initialization CoarseGraph:
    double baToCoarseRotVariance = 1.0;
    double baToCoarsePoseVariance = 1e-1;
    double baToCoarseVelVariance = 1e-1;
    double baToCoarseAccBiasVariance = 1000.0;
    double baToCoarseGyrBiasVariance = 5e-2;

    // Settings regarding bias transfer between coarse tracking and BA.
    bool setting_transferCovToCoarse = true; // Transfer covariance from BA to tracking.
    double transferCovToCoarseMultiplier = 1.0;

    // ----------- Settings for debugging. -----------
    // Use the visual only system after scale has been fixed, which can be useful for debugging (only makes sense together with setting_scaleFixTH).
    // 1 means that also the gtsamIntegration is not used anymore, while 2 means that the gtsamIntegration is still used with IMUExtension removed.
    int setting_visualOnlyAfterScaleFixing = 0;
};

// Contains IMU-Calibration and can read them from file.
// Default contains values for EuRoC.
class IMUCalibration
{
public:
    IMUCalibration();

    IMUCalibration(std::string settingsFilename);
    IMUCalibration(const Sophus::SE3d& tCamImu);
    void loadFromFile(std::string settingsFilename);
    void saveToFile(std::string filename); // Save T_cam_imu to as a camchain.yaml.

    // The noise values are registered as settings so they can be set from commandline and from the settings yaml.
    void registerArgs(dmvio::SettingsUtil& set);

    Sophus::SE3d T_cam_imu;
    // Old defaults for EuRoC.
    double sigma_between_b_a = 0.00447213;
    double sigma_between_b_g = 0.0014142;
    double accel_sigma = 0.316227;
    double gyro_sigma = 0.1;
    double integration_sigma = 0.316227;

    // Currently not read from settings.
    gtsam::Vector3 gravity = (gtsam::Vector(3) << 0, 0, -9.8082).finished();;

private:
    void initDefault();
};

}

#endif //DMVIO_IMUSETTINGS_H
