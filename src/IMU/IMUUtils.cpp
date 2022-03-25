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

#include <gtsam/slam/PriorFactor.h>
#include <GTSAMIntegration/Sim3GTSAM.h>
#include "IMUUtils.h"
#include "IMUTypes.h"
#include "IMUSettings.h"

using namespace dmvio;
using namespace gtsam;

void dmvio::integrateIMUData(const IMUData& imuData, gtsam::PreintegratedImuMeasurements& preintegrated)
{
    for(const auto& measurement : imuData)
    {
        if(measurement.getIntegrationTime() == 0.0) continue;
        preintegrated.integrateMeasurement(gtsam::Vector(measurement.getAccData()),
                                           gtsam::Vector(measurement.getGyrData()),
                                           measurement.getIntegrationTime());
    }
}

gtsam::noiseModel::Diagonal::shared_ptr dmvio::computeBiasNoiseModel(const IMUCalibration& imuCalibration,
                                                                     const gtsam::PreintegratedImuMeasurements& imuMeasurements)
{
    double sigma_b_a = imuCalibration.sigma_between_b_a * sqrt(imuMeasurements.deltaTij());
    double sigma_b_g = imuCalibration.sigma_between_b_g * sqrt(imuMeasurements.deltaTij());
    return gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << sigma_b_a, sigma_b_a, sigma_b_a, sigma_b_g, sigma_b_g, sigma_b_g).finished());
}

void IMUTransformPriorSettings::registerArgs(dmvio::SettingsUtil& set, std::string prefix)
{
    set.registerArg(prefix + "priorExtrinsicsRot", priorExtrinsicsRot);
    set.registerArg(prefix + "priorExtrinsicsTrans", priorExtrinsicsTrans);
    set.registerArg(prefix + "priorGravityDirection", priorGravityDirection);
    set.registerArg(prefix + "priorGravityDirectionZ", priorGravityDirectionZ);
}

std::vector<gtsam::NonlinearFactor::shared_ptr> dmvio::getPriorsAndAddValuesForTransform(
        const TransformDSOToIMU& transform, const IMUTransformPriorSettings& settings, gtsam::Values& values)
{
    std::vector<gtsam::NonlinearFactor::shared_ptr> ret;
    int symInd = transform.getSymbolInd();
    if(transform.optimizeScale())
    {
        gtsam::Key scaleKey = gtsam::Symbol('s', symInd);
        ScaleGTSAM sim(transform.getScale());
        values.insert(scaleKey, sim);
    }

    if(transform.optimizeGravity())
    {
        gtsam::Key gravityKey = Symbol('g', symInd);
        gtsam::Rot3 initialRot(transform.getR_dsoW_metricW().matrix());
        gtsam::Rot3 zeroRot; // Initialize with current transform but set prior to zero transform.
        values.insert(gravityKey, initialRot);

        Eigen::Vector3d gravityModel;
        double rotationalSigma = settings.priorGravityDirection;
        gravityModel.setConstant(rotationalSigma);
        gravityModel(2) = settings.priorGravityDirectionZ;
        gtsam::PriorFactor<gtsam::Rot3>::shared_ptr rotationPrior(
                new gtsam::PriorFactor<gtsam::Rot3>(gravityKey, zeroRot,
                                                    gtsam::noiseModel::Diagonal::Sigmas(gravityModel)));
        ret.push_back(rotationPrior);
    }

    if(transform.optimizeExtrinsics())
    {
        gtsam::Key extrinsicsKey = Symbol('i', symInd);
        gtsam::Pose3 initialExtr(transform.getT_cam_imu().matrix());
        values.insert(extrinsicsKey, initialExtr);

        gtsam::Vector6 extrinsicsModel;
        extrinsicsModel.segment(0, 3).setConstant(settings.priorExtrinsicsRot);
        extrinsicsModel.segment(3, 3).setConstant(settings.priorExtrinsicsTrans);
        gtsam::PriorFactor<gtsam::Pose3>::shared_ptr extrinsicsPrior(new gtsam::PriorFactor<gtsam::Pose3>(
                extrinsicsKey, initialExtr, gtsam::noiseModel::Diagonal::Sigmas(extrinsicsModel)
        ));
        ret.push_back(extrinsicsPrior);
    }
    return ret;
}
