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

#ifndef DMVIO_POSETRANSFORMATIONIMU_H
#define DMVIO_POSETRANSFORMATIONIMU_H

#include <IMU/IMUUtils.h>
#include "PoseTransformation.h"

namespace dmvio
{

template<typename T> class TransformIMUToDSOForCoarse;

// Helper methods which compute the analytic derivatives for TransformIMUToDSOForCoarse
template<typename T>
gtsam::Matrix66 getCoarsePoseDerivative(const PoseTransformation::PoseType& pose,
                                        const DerivativeDirection& direction, T& transform,
                                        TransformIMUToDSOForCoarse<T>& transformForCoarse);
template<typename T> gtsam::Matrix getCoarseReferenceDerivative(const PoseTransformation::PoseType& pose,
                                                                DerivativeDirection direction, T& transform,
                                                                TransformIMUToDSOForCoarse<T>& transformForCoarse);

// PoseTransformation which transforms from DSO frame (world to cam in DSO scale) to metric IMU frame (imu to world in metric scale).
// This also needs to be used to convert Hessians and factors the other way round (from metric frame to DSO frame).
class TransformDSOToIMU : public PoseTransformation
{
public:
    // optScale, optGravity, and optT_Cam_imu define which additional variables shall be optimized.
    // We pass them as a pointers, so that they can be set to false for all instances of this class simultaneously when optimization of them shall be stopped.
    // if fixZ is true, the z-axis of gravity direction is fixed (which makes sense because yaw is not observable with an IMU).
    // symbol ind is the GTSAM symbol index of the additionally optimized variables (typically 0).
    TransformDSOToIMU(const gtsam::Pose3& T_cam_imu, std::shared_ptr<bool> optScale,
                      std::shared_ptr<bool> optGravity, std::shared_ptr<bool> optT_cam_imu, bool fixZ,
                      int symbolInd = 0);

    // Copy constructor which sets different pointers for bools which determine what is optimized.
    TransformDSOToIMU(const TransformDSOToIMU& other, std::shared_ptr<bool> optScale, std::shared_ptr<bool> optGravity,
                      std::shared_ptr<bool> optT_cam_imu);


    // Override from PoseTransformation
    // ------------------------------------------------------------

    // We transform from worldToCam to imuToWorld;
    PoseType transformPose(const PoseType& pose) const override;
    PoseType transformPoseInverse(const PoseType& pose) const override;

    void precomputeForDerivatives() override;

    // Compute the derivative w.r.t the pose.
    gtsam::Matrix66 getPoseDerivative(const PoseType& pose, DerivativeDirection direction) override;
    // Compute the derivatives for all variables which are optimized, first the pose and then all optimized symbols (e.g. scale, T_cam_imu, etc.).
    std::vector<gtsam::Matrix> getAllDerivatives(const PoseType& pose, DerivativeDirection direction) override;
    // Returns the symbols of the additional variables (except the pose) which are optimized.
    std::vector<gtsam::Key> getAllOptimizedSymbols() const override;
    // Updated all optimized symbols using the value in values (if available).
    void updateWithValues(const gtsam::Values& values) override;

    // Setters and getters.
    // --------------------------------------------------
    void setScale(double variable);
    double getScale() const;

    const Sophus::SE3d& getT_cam_imu() const;
    void resetGravityDirection();

    std::unique_ptr<PoseTransformation> clone() const override;

    bool optimizeScale() const
    { return *optScale; }

    bool optimizeGravity() const
    { return *optGravity; }

    bool optimizeExtrinsics() const
    { return *optT_cam_imu; }

    int getSymbolInd() const;
    void setSymbolInd(int symbolInd);

    const Sophus::SO3d& getR_dsoW_metricW() const;

private:
    std::shared_ptr<bool> optScale, optGravity, optT_cam_imu;

    void fillKeyDimMap();

    Sophus::Sim3d T_S_DSO; // Scale: Sim(3) transformation from DSO scale to metric scale.

    Sophus::SO3d R_dsoW_metricW; // The dso world (dsoW) is a rotated version of the metric world (metricW).
    // Note that we represent rotation this way (not with the inverse) because we have a right-sided increment,
    // and this way we can fix the z axis of this rotation.

    Sophus::SE3d T_cam_imu;

    bool precomputedValid{false};
    Sophus::Sim3d precomputed;
    gtsam::Matrix77 precomputedAdj;

    int symbolInd = 0;

    bool fixZ;

    friend gtsam::Matrix66 dmvio::getCoarsePoseDerivative<>(const PoseTransformation::PoseType& pose,
                                                            const DerivativeDirection& direction,
                                                            TransformDSOToIMU& transform,
                                                            TransformIMUToDSOForCoarse<TransformDSOToIMU>& transformForCoarse);

    friend gtsam::Matrix dmvio::getCoarseReferenceDerivative<>(const PoseTransformation::PoseType& pose,
                                                               DerivativeDirection direction,
                                                               TransformDSOToIMU& transform,
                                                               TransformIMUToDSOForCoarse<TransformDSOToIMU>& transformForCoarse);
};

// DSO Coarse tracking does not optimize global poses (camToWorld), but the relative pose T_f_r (reference to frame).
// To convert between these and the global poses (frame to world and reference to world) we use this transform.
// Converts poses from T_w_f (frame to world) (while also using reference to world T_w_r) in IMU frame to T_f_r in DSO frame (and therefore to convert Hessians from DSO frame to IMU frame).
// Uses the fields of the original transform transformDSOToIMU for the conversion from camera to metric frame.
template<typename T> class TransformIMUToDSOForCoarse : public PoseTransformation
{
public:
    // Pass the used transformDSOToIMU and the id of the keyframe (reference frame).
    // Note, that a pointer to transformDSOToIMU is kept, so changes to it will also effect this object.
    TransformIMUToDSOForCoarse(std::shared_ptr<T> transformDSOToIMU, int keyframeId);

    // pose is the T_w_f in IMU frame, output is T_f_r in DSO frame. For this the additionally optimized symbol T_w_r is used.
    PoseType transformPose(const PoseType& pose) const override;
    PoseType transformPoseInverse(const PoseType& pose) const override;

    gtsam::Matrix66 getPoseDerivative(const PoseType& pose, DerivativeDirection direction) override;

    // Computes the derivative w.r.t T_w_f and also T_w_r.
    std::vector<gtsam::Matrix> getAllDerivatives(const PoseType& pose, DerivativeDirection direction) override;
    // Returns the symbol of the reference frame as this is also optimized in the coarse tracking.
    std::vector<gtsam::Key> getAllOptimizedSymbols() const override;
    void updateWithValues(const gtsam::Values& values) override;

    std::unique_ptr<PoseTransformation> clone() const override;
private:
    std::shared_ptr<T> transformToIMU;
    Sophus::SE3d referenceToWorld;
    int keyframeId = -1;

    friend gtsam::Matrix66 dmvio::getCoarsePoseDerivative<>(const PoseTransformation::PoseType& pose,
                                                            const DerivativeDirection& direction, T& transform,
                                                            TransformIMUToDSOForCoarse<T>& transformForCoarse);

    friend gtsam::Matrix dmvio::getCoarseReferenceDerivative<>(const PoseTransformation::PoseType& pose,
                                                               DerivativeDirection direction, T& transform,
                                                               TransformIMUToDSOForCoarse<T>& transformForCoarse);
};

}

#endif //DMVIO_POSETRANSFORMATIONIMU_H
