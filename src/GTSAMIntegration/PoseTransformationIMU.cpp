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

#include "PoseTransformationIMU.h"
#include <util/TimeMeasurement.h>
#include "dso/util/FrameShell.h"
#include "ExtUtils.h"
#include "Sim3GTSAM.h"

using namespace dmvio;
using dso::Vec8;
using gtsam::Matrix, gtsam::Vector;
using gtsam::Symbol;

TransformDSOToIMU::TransformDSOToIMU(const gtsam::Pose3& T_cam_imu, std::shared_ptr<bool> optScale,
                                     std::shared_ptr<bool> optGravity, std::shared_ptr<bool> optT_cam_imu,
                                     bool fixZ,
                                     int symbolInd)
        : T_cam_imu(T_cam_imu.matrix()), optScale(optScale), optGravity(optGravity), optT_cam_imu(optT_cam_imu),
          symbolInd(symbolInd),
          fixZ(fixZ)
{
    fillKeyDimMap();
}

TransformDSOToIMU::TransformDSOToIMU(const TransformDSOToIMU& other, std::shared_ptr<bool> optScalePassed,
                                     std::shared_ptr<bool> optGravityPassed,
                                     std::shared_ptr<bool> optT_cam_imuPassed)
        : TransformDSOToIMU(other) // first call default copy constructor
{
    optScale = optScalePassed;
    optGravity = optGravityPassed;
    optT_cam_imu = optT_cam_imuPassed;
}

// pose is T_cam_dsoW in DSO scale.
PoseTransformation::PoseType TransformDSOToIMU::transformPose(const PoseTransformation::PoseType& pose) const
{
    Sophus::Sim3d scaledT_w_cam = T_S_DSO * Sophus::Sim3d(pose).inverse() * T_S_DSO.inverse(); // in metric scale.
    assert(std::abs(scaledT_w_cam.scale() - 1.0) < 0.0001);
    Sophus::SE3d T_metricW_imu;
    T_metricW_imu = Sophus::SE3d(R_dsoW_metricW.inverse(), Sophus::Vector3d::Zero()) *
                    Sophus::SE3d(scaledT_w_cam.matrix()) * T_cam_imu;
    PoseType returning = T_metricW_imu.matrix();

#ifdef DEBUG
    // Check that inverse is the actual inverse.
    assertEqEigen(pose, transformPoseInverse(returning), 1e-4);
#endif

    return returning;
}

// pose is T_metricW_imu in metric scale.
PoseTransformation::PoseType TransformDSOToIMU::transformPoseInverse(const PoseTransformation::PoseType& pose) const
{
    // dso world to cam in metric scale:
    Sophus::SE3d T_cam_dsoW_metric = Sophus::SE3d();
    T_cam_dsoW_metric = T_cam_imu * Sophus::SE3d(pose).inverse() *
                        Sophus::SE3d(R_dsoW_metricW.inverse(), Sophus::Vector3d::Zero());
    // in DSO scale:
    Sophus::Sim3d T_cam_dsoW = T_S_DSO.inverse() * Sophus::Sim3d(T_cam_dsoW_metric.matrix()) * T_S_DSO;
    if(!(std::abs(T_cam_dsoW.scale() - 1.0) < 0.0001))
    {
        std::cout << T_cam_dsoW.matrix() << std::endl;
    }
    assert(std::abs(T_cam_dsoW.scale() - 1.0) < 0.0001 ||
           std::isnan(T_cam_dsoW.scale())); // Scale should be close to 1 (unless it is Nan which can happen sometimes).
    return T_cam_dsoW.matrix();
}

void TransformDSOToIMU::precomputeForDerivatives()
{
    if(precomputedValid) return;
    precomputedValid = true;
    precomputed = Sophus::Sim3d(T_cam_imu.inverse().matrix()) * T_S_DSO;
    precomputedAdj = precomputed.Adj();
}

gtsam::Matrix66
TransformDSOToIMU::getPoseDerivative(const PoseTransformation::PoseType& pose, DerivativeDirection direction)
{
    if(direction == DerivativeDirection::RIGHT_TO_RIGHT)
    {
        // Analytic derivatives
        assert(precomputedValid);
        Sophus::Sim3d intermediateRes = precomputed * Sophus::Sim3d(pose);
        auto firstAdj = intermediateRes.Adj();
        gtsam::Matrix66 poseJ = convertJacobianToGTSAM(-firstAdj).block<6, 6>(0, 0);
        return poseJ;
    }
    return PoseTransformation::getPoseDerivative(pose, direction);
}

std::vector<gtsam::Matrix>
TransformDSOToIMU::getAllDerivatives(const PoseTransformation::PoseType& pose, DerivativeDirection direction)
{
    std::vector<gtsam::Matrix> analyticDerivs;
    bool analyticDerivsFilled = false;
    if(direction == DerivativeDirection::RIGHT_TO_RIGHT)
    {
        // Analytic derivatives:
        assert(precomputedValid);
        analyticDerivsFilled = true;
        // Intermediate res is:  T_cam_imu^-1 * T_S_DSO * T_cam_world
        Sophus::Sim3d intermediateRes = precomputed * Sophus::Sim3d(pose);
        auto firstAdj = intermediateRes.Adj();
        gtsam::Matrix66 poseJ = convertJacobianToGTSAM(-firstAdj).block<6, 6>(0, 0);
        analyticDerivs.push_back(poseJ);

        if(*optScale)
        {
            gtsam::Matrix scaleJ = convertJacobianToGTSAM(firstAdj - precomputedAdj);
            analyticDerivs.push_back(scaleJ.topRightCorner<6, 1>());
        }
        if(*optGravity)
        {
            Sophus::SE3d innerAdjoint((intermediateRes * T_S_DSO.inverse()).matrix());
            gtsam::Matrix66 gravityJac;
            // J = -(T_cam_imu.inverse() * T_S_DSO * pose * T_S_DSO.inverse() * R_dsoW_metricW).Adj();
            gravityJac = convertJacobianToGTSAM(
                    -(innerAdjoint * Sophus::SE3d(R_dsoW_metricW, Sophus::Vector3d::Zero())).Adj());
            if(fixZ)
            {
                // Set the yaw derivative to zero here.
                gravityJac.block<6, 1>(0, 2).setZero();
            }
            analyticDerivs.push_back(gravityJac.topLeftCorner<6, 3>());
        }
        if(*optT_cam_imu)
        {
            // Derivative is one.
            analyticDerivs.push_back(gtsam::Matrix66::Identity());
        }

#ifndef DEBUG
        // In debug mode don't return yet, but later after we compared to numeric derivatives!
        return analyticDerivs;
#endif
    }

    // Compute numeric Jacobians.
    std::vector<gtsam::Matrix> returning;
    returning.push_back(PoseTransformation::getPoseDerivative(pose, direction));
    if(*optScale)
    {
        gtsam::Matrix numJac = computeNumericJacobian(*this, Sophus::SE3d(pose), &T_S_DSO, direction);
        // Set translational and rotational part to zero, because we only want to optimize scale!
        numJac.topLeftCorner<6, 6>().setZero();
        returning.push_back(numJac.topRightCorner<6, 1>());
    }
    if(*optGravity)
    {
        gtsam::Matrix numJac = gtsam::Matrix();
        numJac = computeNumericJacobian(*this, Sophus::SE3d(pose), &R_dsoW_metricW, direction);
        if(fixZ)
        {
            // Set the yaw derivative to zero here. But I'm not sure which one it is yet!
            numJac.block<6, 1>(0, 2).setZero();
        }
        returning.push_back(numJac);
    }
    if(*optT_cam_imu)
    {
        gtsam::Matrix numJac = computeNumericJacobian(*this, Sophus::SE3d(pose), &T_cam_imu, direction);
        returning.push_back(numJac);
    }

    if(analyticDerivsFilled)
    {
        assert(analyticDerivs.size() == returning.size());
        for(int i = 0; i < analyticDerivs.size(); ++i)
        {
            assertNumericJac(returning[i], analyticDerivs[i]);
        }
        return analyticDerivs;
    }
    return returning;

}

std::vector<gtsam::Key> TransformDSOToIMU::getAllOptimizedSymbols() const
{
    std::vector<gtsam::Key> returning;
    if(*optScale)
    {
        returning.push_back(gtsam::Symbol('s', symbolInd));
    }
    if(*optGravity)
    {
        returning.push_back(gtsam::Symbol('g', symbolInd));
    }
    if(*optT_cam_imu)
    {
        // i for IMU intrinsics.
        returning.push_back(gtsam::Symbol('i', symbolInd));
    }
    return returning;
}

void TransformDSOToIMU::updateWithValues(const gtsam::Values& values)
{
    if(*optScale)
    {
        double scaleBefore = T_S_DSO.scale();
        T_S_DSO = values.at<ScaleGTSAM>(Symbol('s', symbolInd)).sim();
        if(fabs(scaleBefore - T_S_DSO.scale()) >= 1e-9)
        {
            precomputedValid = false;
        }
        assert(T_S_DSO.rotationMatrix().isIdentity(0.000001));
        assert(T_S_DSO.translation().isZero(0.000001));
    }
    if(*optGravity)
    {
        gtsam::Rot3 rot = values.at<gtsam::Rot3>(Symbol('g', symbolInd));
        if(!rot.equals(gtsam::Rot3(R_dsoW_metricW.matrix())))
        {
            precomputedValid = false;
        }
        R_dsoW_metricW = Sophus::SO3d(rot.matrix());
    }
    if(*optT_cam_imu)
    {
        gtsam::Pose3 newExtr = values.at<gtsam::Pose3>(Symbol('i', symbolInd));
        if(!newExtr.equals(gtsam::Pose3(T_cam_imu.matrix()))) precomputedValid = false;
        T_cam_imu = Sophus::SE3d(newExtr.matrix());
    }
}

void TransformDSOToIMU::setScale(double variable)
{
    precomputedValid = false;
    T_S_DSO.setScale(variable);
}

double TransformDSOToIMU::getScale() const
{
    return T_S_DSO.scale();
}

std::unique_ptr<PoseTransformation> TransformDSOToIMU::clone() const
{
    return std::unique_ptr<PoseTransformation>(new TransformDSOToIMU(*this));
}

void TransformDSOToIMU::resetGravityDirection()
{
    R_dsoW_metricW = Sophus::SO3d{};
}

template<typename T>
TransformIMUToDSOForCoarse<T>::TransformIMUToDSOForCoarse(std::shared_ptr<T> transformDSOToIMU, int keyframeId)
        : transformToIMU(transformDSOToIMU), keyframeId(keyframeId)
{
    keyDimMap[Symbol('p', keyframeId)] = 6;
}

template<typename T>
dmvio::PoseTransformation::PoseType TransformIMUToDSOForCoarse<T>::transformPose(const PoseType& pose) const
{
    // First convert from IMU to camera frame.
    // pose is imuToWorld, and we also have reference to world (both IMU metric frame).
    PoseType T_f_w = transformToIMU->transformPoseInverse(pose);
    PoseType T_r_w = transformToIMU->transformPoseInverse(referenceToWorld.matrix());

    // We want to output T_f_r (reference to frame).
    PoseType T_f_r = T_f_w * T_r_w.inverse();

#ifdef DEBUG
    // Check that inverse is the actual inverse.
    if(!T_f_r.hasNaN())
    {
        assertEqEigen(pose, transformPoseInverse(T_f_r));
    }
#endif

    return T_f_r;
}

template<typename T>
dmvio::PoseTransformation::PoseType TransformIMUToDSOForCoarse<T>::transformPoseInverse(const PoseType& poseT_f_r) const
{
    // First get worldToReference
    PoseType T_r_w = transformToIMU->transformPoseInverse(referenceToWorld.matrix());
    // We got T_f_r as input ->
    PoseType T_f_w = poseT_f_r * T_r_w;

    return transformToIMU->transformPose(T_f_w);
}

// Helper functions for the derivatives, so that we can specialize them to compute analytic derivatives for each class type.
template<typename T>
gtsam::Matrix66 dmvio::getCoarsePoseDerivative(const PoseTransformation::PoseType& pose,
                                               const DerivativeDirection& direction, T& transform,
                                               TransformIMUToDSOForCoarse<T>& transformForCoarse)
{
    // Default is numeric Jacobian.
    return transformForCoarse.PoseTransformation::getPoseDerivative(pose, direction);
}

// Analytic derivatives for TransformIMUToDSOForCoarse<TransformDSOToIMUNew>
template<> gtsam::Matrix66 dmvio::getCoarsePoseDerivative(const PoseTransformation::PoseType& pose,
                                                          const DerivativeDirection& direction,
                                                          TransformDSOToIMU& transform,
                                                          TransformIMUToDSOForCoarse<TransformDSOToIMU>& transformForCoarse)
{
    gtsam::Matrix poseJac = convertJacobianToGTSAM(
            -(transform.T_S_DSO.inverse() * Sophus::Sim3d(transform.T_cam_imu.matrix())).Adj()).topLeftCorner(
            6, 6);
    return poseJac;
}

template<typename T> gtsam::Matrix dmvio::getCoarseReferenceDerivative(const PoseTransformation::PoseType& pose,
                                                                       DerivativeDirection direction, T& transform,
                                                                       TransformIMUToDSOForCoarse<T>& transformForCoarse)
{
    // Default to numeric Jacobian.
    gtsam::Matrix numJac = computeNumericJacobian(transformForCoarse, Sophus::SE3d(pose),
                                                  &transformForCoarse.referenceToWorld, direction);
    return numJac;
}

// Analytic derivatives for TransformIMUToDSOForCoarse<TransformDSOToIMUNew>
template<> gtsam::Matrix dmvio::getCoarseReferenceDerivative(const PoseTransformation::PoseType& pose,
                                                             DerivativeDirection direction,
                                                             TransformDSOToIMU& transform,
                                                             TransformIMUToDSOForCoarse<TransformDSOToIMU>& transformForCoarse)
{
    // compute derivative w.r.t reference to world.
    Sophus::Sim3d T_w_f_imu(pose);
    Sophus::Sim3d T_w_r_imu(transformForCoarse.referenceToWorld.matrix());
    gtsam::Matrix referenceJac = convertJacobianToGTSAM(
            (transform.T_S_DSO.inverse() * Sophus::Sim3d(transform.T_cam_imu.matrix()) *
             T_w_f_imu.inverse() *
             T_w_r_imu).Adj()).topLeftCorner(6, 6);
    return referenceJac;
}

const Sophus::SE3d& TransformDSOToIMU::getT_cam_imu() const
{
    return T_cam_imu;
}

void TransformDSOToIMU::setSymbolInd(int symbolInd)
{
    TransformDSOToIMU::symbolInd = symbolInd;
    keyDimMap.clear();
    fillKeyDimMap();
}

int TransformDSOToIMU::getSymbolInd() const
{
    return symbolInd;
}

void TransformDSOToIMU::fillKeyDimMap()
{
    keyDimMap[Symbol('s', symbolInd)] = 1;
    keyDimMap[Symbol('g', symbolInd)] = 3;
    keyDimMap[Symbol('i', symbolInd)] = 6;
}

const Sophus::SO3d& TransformDSOToIMU::getR_dsoW_metricW() const
{
    return R_dsoW_metricW;
}

template<typename T>
gtsam::Matrix66 TransformIMUToDSOForCoarse<T>::getPoseDerivative(const PoseType& pose, DerivativeDirection direction)
{
    if(direction == DerivativeDirection::RIGHT_TO_LEFT)
    {
        gtsam::Matrix poseJac = getCoarsePoseDerivative(pose, direction, *transformToIMU, *this);
#ifdef DEBUG
        assertNumericJac(PoseTransformation::getPoseDerivative(pose, direction), poseJac);
#endif
        return poseJac;
    }
    return PoseTransformation::getPoseDerivative(pose, direction);
}

// Computes the derivative w.r.t T_w_f and also T_w_r.
template<typename T> std::vector<gtsam::Matrix>
TransformIMUToDSOForCoarse<T>::getAllDerivatives(const PoseType& pose, DerivativeDirection direction)
{
    std::vector<gtsam::Matrix> returning;
    returning.push_back(getPoseDerivative(pose, direction));

    if(direction == DerivativeDirection::RIGHT_TO_LEFT)
    {
        // compute derivative w.r.t reference to world.
        Sophus::Sim3d T_w_f_imu(pose);
        Sophus::Sim3d T_w_r_imu(referenceToWorld.matrix());
        gtsam::Matrix referenceJac = getCoarseReferenceDerivative(pose, direction, *transformToIMU, *this);
#ifdef DEBUG
        gtsam::Matrix numJac = computeNumericJacobian(*this, Sophus::SE3d(pose), &referenceToWorld, direction);
        assertNumericJac(numJac, referenceJac);
#endif
        returning.push_back(referenceJac);
    }else
    {
        gtsam::Matrix numJac = computeNumericJacobian(*this, Sophus::SE3d(pose), &referenceToWorld, direction);
        returning.push_back(numJac);
    }
    return returning;
}

template<typename T> std::vector<gtsam::Key> TransformIMUToDSOForCoarse<T>::getAllOptimizedSymbols() const
{
    std::vector<gtsam::Key> returning;
    // We also optimize the reference pose.
    returning.push_back(gtsam::Symbol('p', keyframeId));
    return returning;
}

template<typename T>
void TransformIMUToDSOForCoarse<T>::updateWithValues(const gtsam::Values& values)
{
    referenceToWorld = Sophus::SE3d(values.at<gtsam::Pose3>(Symbol('p', keyframeId)).matrix());
}

template<typename T>
std::unique_ptr<PoseTransformation> TransformIMUToDSOForCoarse<T>::clone() const
{
    return std::unique_ptr<PoseTransformation>(new TransformIMUToDSOForCoarse<T>(*this));
}

namespace dmvio
{
template class TransformIMUToDSOForCoarse<TransformDSOToIMU>;
}
