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

#ifndef DMVIO_IMUUTILS_H
#define DMVIO_IMUUTILS_H

#include "IMUTypes.h"
#include <gtsam/navigation/ImuFactor.h>
#include "GTSAMIntegration/PoseTransformationIMU.h"
#include "util/SettingsUtil.h"

namespace dmvio
{

class IMUCalibration;
class TransformDSOToIMU;

void integrateIMUData(const IMUData& imuData, gtsam::PreintegratedImuMeasurements& preintegrated);

gtsam::noiseModel::Diagonal::shared_ptr
computeBiasNoiseModel(const IMUCalibration& imuCalibration, const gtsam::PreintegratedImuMeasurements& imuMeasurements);

// Settings regarding prior on the symbols optimized by TransformDSOToIMU.
class IMUTransformPriorSettings
{
public:
    double priorExtrinsicsRot = 0.01; // rotational and translational prior for extrinsics.
    double priorExtrinsicsTrans = 0.1;
    double priorGravityDirection = 0.4; // Prior on gravity direction (first xy and then z).
    double priorGravityDirectionZ = 0.0001;

    void registerArgs(dmvio::SettingsUtil& set, std::string prefix = "");
};

// Add values for the variables optimized by TransformDSOToIMU and return a list of prior factors,
// according to the IMUTransformPriorSettings.
std::vector<gtsam::NonlinearFactor::shared_ptr> getPriorsAndAddValuesForTransform(
        const TransformDSOToIMU& transform, const IMUTransformPriorSettings& settings, gtsam::Values& values);

}
#endif //DMVIO_IMUUTILS_H
