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

#ifndef DMVIO_IMUINITSETTINGS_H
#define DMVIO_IMUINITSETTINGS_H

#include "util/SettingsUtil.h"
#include "IMU/IMUUtils.h"
#include "GTSAMIntegration/PoseTransformationFactor.h"

namespace dmvio
{

class CoarseIMUInitOptimizerSettings
{
public:
    void registerArgs(dmvio::SettingsUtil& set, std::string prefix);

    int maxNumPoses = 100; // forgets older poses.
    bool fixPoses = true, multipleBiases = false;

    // Prior on rotation and translation (not used if fixPoses=true).
    double priorRotSigma = 0.00001;
    double priorTransSigma = 0.00001;

    // Optimizer settings
    double lambdaLowerBound = 1e-16;

    PoseTransformationFactor::ConversionType conversionType = PoseTransformationFactor::JACOBIAN_FACTOR;

    bool updatePoses = true; // if true we get the updated poses from DSO before optimizing.

    double requestFullResetErrorThreshold = -1; // if the error gets higher than this request a full reset.
    double requestFullResetNormalizedErrorThreshold = -1; // if the normalized error gets higher than this request a full reset.
};

class PGBASettings
{
public:
    void registerArgs(dmvio::SettingsUtil& set, std::string prefix);

    // Delay of the graph used for the PGBA.
    int delay = 100;

    double scaleUncertaintyThresh = 1.0; // Threshold for first init to succeed.
    double reinitScaleUncertaintyThresh = 0.5; // Threshold to stop reinitializing.

    int skipFirstKFs = 0; // if positive the first n KFs are skipped.

    // Should theoretically be best set to true, but in practice false seems to be better.
    bool prepareGraphAddFactors = false;
    bool prepareGraphAddDelValues = false;

    PoseTransformationFactor::ConversionType conversionType = PoseTransformationFactor::JACOBIAN_FACTOR;
    IMUTransformPriorSettings transformPriors;
};

class IMUThresholdSettings
{
public:
    void registerArgs(dmvio::SettingsUtil& set, std::string prefix = "thresh_");

    double threshScale = 1000.0;
    double threshGravdir = 1000.0;
};

// Settings related to the initializer
class IMUInitSettings
{
public:
    void registerArgs(dmvio::SettingsUtil& set, std::string prefix = "init_");

    // For available options see enum InitTransitionMode in file IMUInitializerTransitions.
    int transitionModel = 2;

    // Settings regarding coarse initializer.
    CoarseIMUInitOptimizerSettings coarseInitSettings;
    bool onlyKFs = true; // Only include keyframes in the coarse IMU initializer.
    // Priors for the coarse optimizer.
    IMUTransformPriorSettings transformPriors;
    double coarseScaleUncertaintyThresh = 1.0; // Scale uncertainty must be below this to consider the coarse init to have succeeded.

    // Settings regarding PGBA based initializer.
    PGBASettings pgbaSettings;

    // Threshold settings for the marginalization replacement.
    IMUThresholdSettings thresholdSettings; // default for scale threshold is 1.02
    double percentageSwitchToSecondTH = 0.5; // switch to second threshold once this fraction of IMU factors would be lost.
    IMUThresholdSettings secondThresholdSettings; // default for second scale threshold is effectively infinity (see cpp file).

    // Also init IMU params.
    bool initDSOParams = true;

    // If positive add a scale prior with this stddev after the initialization (meant to be used with
    // transitionModels 4 and 5 for the ablations).
    double scalePriorAfterInit = 0.0;

    // If false, the visual-inertial system is initialized immediately (with scale 1). Then we
    // still run the initializer for re-initialization though. A value of false will not work well if the objects in
    // the scene are far away. This is mostly a legacy setting which might not work very well at the moment, but it
    // could be useful for future extensions.
    bool disableVIOUntilFirstInit = true;

    // Setting for debugging. Do IMU initialization in separate thread, even if we are in non-realtime mode.
    bool multithreadedInitDespiteNonRT = false;

};

}

#endif //DMVIO_IMUINITSETTINGS_H
