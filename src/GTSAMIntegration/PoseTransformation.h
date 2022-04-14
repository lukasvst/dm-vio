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

#ifndef DMVIO_POSETRANSFORMATION_H
#define DMVIO_POSETRANSFORMATION_H


#include <dso/util/NumType.h>
#include <gtsam/geometry/Pose3.h>
#include <OptimizationBackend/EnergyFunctionalStructs.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/Symbol.h>

namespace dmvio
{

// This file contains methods to convert poses betwen coordinate systems, and also compute relative derivatives (Jacobians)
// to convert Hessians and GTSAM factors between the systems.
// Usage: Define a child class of PoseTransformation which can transform poses and also compute relative derivatives.
// Then either convert a Hessian (H) and gradient vector (b) using convertHAndBWithPoseTransformation, or simply
// use a PoseTransformationFactor.

// Defines on which side of of the pose the epsilon is for the input (first) and output (second) transformation.
// When computing derivatives w.r.t. poses using Lie Groups, the increment can either be placed on the left or the right side.
// DSO places the increment on the left side. i.e. newPose = exp(increment) * oldPose.
// GTSAM places the increment on the right side, i.e. newPose = oldPose * exp(increment).
// Both approaches are equally valid, but the Jacobians need to be computed consistently with the increment side.
// With the relative Jacobians computed by our PoseTransformations we can convert between the two.
// Note: that for computing relative Jacobians they work on the inverse direction as the PoseTransformation, e.g.
// to convert Jacobians from DSO style to GTSAM style, one has to pass RIGHT_TO_LEFT.
// When working only with GTSAM style increments one can simply pass RIGHT_TO_RIGHT.
enum class DerivativeDirection
{
    LEFT_TO_LEFT,
    LEFT_TO_RIGHT,
    RIGHT_TO_LEFT,
    RIGHT_TO_RIGHT
};

// Abstract base class for transformations from one coordinate system to another.
// Main functionality is to transformm a pose to a different coordinate system.
// Also contains methods to compute relative derivatives. These will transform Jacobians according to the transformation.
// An example implementation is TransformDSOToIMU (in PoseTransformationIMU.h).
// --------------------------------------------------
// By default only derivatives w.r.t. to the pose are computed (by the method getPoseDerivative).
// However some transformations have additional variables which are optimized (e.g. the scale, or the transform T_cam_imu),
// which can influence the transformed pose.
// These additional variables can be queried with getAllOptimizedSymbols, and derivatives w.r.t. these variables can
// be computed with getAllDerivatives.
// When optimizing these additional variables (e.g. in a factor graph), they can be updated with updateẂithValues.
// --------------------------------------------------
// This class also contains a default implementation for getPoseDerivative using numeric Jacobians, which should work out of the box,
// in case the child class does not override the method with an analytic Jacobian computation.
// Child classes which optimize additional symbols need to always reimplement getAllDerivatives themself.
class PoseTransformation
{
public:
    typedef dso::Mat44 PoseType;

    virtual ~PoseTransformation() = default;

    // Transform pose with the transformation.
    virtual PoseType transformPose(const PoseType& pose) const = 0;
    // Perform the inverse transformation.
    virtual PoseType transformPoseInverse(const PoseType& pose) const = 0;

    // Perform precomputation which might be necessary for the derivative computation. Should be called once before
    // calling getPoseDerivative for a batch of variables.
    virtual void precomputeForDerivatives()
    {}

    // Compute the derivative w.r.t the pose.
    // This can be used to transform Jacobians from the target coordinate system to the source coordinate system.
    virtual gtsam::Matrix66 getPoseDerivative(const PoseType& pose, DerivativeDirection direction);
    // Compute the derivatives for all variables which are optimized, first the pose and then all optimized symbols (e.g. scale, T_cam_imu, etc.).
    virtual std::vector<gtsam::Matrix> getAllDerivatives(const PoseType& pose, DerivativeDirection direction);
    // Returns the symbols of the additional variables (except the pose) which are optimized.
    virtual std::vector<gtsam::Key> getAllOptimizedSymbols() const;

    // Updates all optimized symbols using the value in values (if available).
    virtual void updateWithValues(const gtsam::Values& values)
    {}

    int getOptimizedDim(gtsam::Key key) const
    { return keyDimMap.at(key); } // get dimension of optimized symbol

    virtual std::unique_ptr<PoseTransformation> clone() const = 0;

protected:
    std::map<gtsam::Key, int> keyDimMap; // only for optimized symbols, often times empty.
//    virtual std::unique_ptr<PoseTransformation> clone() = 0;
};

// Identity transformation. Used get the Jacobians to transform from left sided increment (DSO) to right sided increment (GTSAM), without changing the poses.
class TransformIdentity : public PoseTransformation
{
public:
    virtual PoseType transformPose(const PoseType& pose) const override
    {
        return pose;
    }

    virtual PoseType transformPoseInverse(const PoseType& pose) const override
    {
        return pose;
    }

    std::unique_ptr<PoseTransformation> clone() const override
    {
        return std::unique_ptr<PoseTransformation>(new TransformIdentity(*this));
    }

    virtual gtsam::Matrix66 getPoseDerivative(const PoseType& pose, DerivativeDirection direction) override;
};

// Create the inverse of a PoseTransformation.
template<typename T> class InversePoseTransform : public PoseTransformation
{
public:
    explicit InversePoseTransform(T& originalTransform)
            : transform(originalTransform)
    {}

    PoseType transformPose(const PoseType& pose) const override
    {
        return transform.transformPoseInverse(pose);
    }

    PoseType transformPoseInverse(const PoseType& pose) const override
    {
        return transform.transformPose(pose);
    }

    std::unique_ptr<PoseTransformation> clone() const override
    {
        return std::unique_ptr<PoseTransformation>(new InversePoseTransform<T>(*this));
    }

private:
    T& transform;
};

// Convert Hessian (and gradient vector b) from DSO Bundle Adjustment to GTSAM. Also uses the poseTransformation (typically just identity) to transform the poses.
// ordering and keyDimMap are used to identify which column / row of the Hessian corresponds to which variable.
std::pair<gtsam::Matrix, gtsam::Vector> convertHAndBFromDSO(const dso::MatXX& H, const dso::VecX& b,
                                                            PoseTransformation& poseTransformation,
                                                            double weightDSOToGTSAM,
                                                            const gtsam::Ordering& ordering,
                                                            const gtsam::Values& values,
                                                            const std::map<gtsam::Key, size_t>& keyDimMap);

// Convert H and b between 2 GTSAM factor graphs.
// Note that the transformation has to be defined "the other way round" compared to the Hessians:
// To convert a Hessian that is defined in the IMU space to the DSO space you have to provide the TransformDSOToIMU.
// The reason is that the relative Jacobians have to be defined this way.
std::pair<gtsam::Matrix, gtsam::Vector>
convertHAndBWithPoseTransformation(const std::pair<gtsam::Matrix, gtsam::Vector>& input,
                                   const gtsam::Ordering& ordering, const std::map<gtsam::Key, size_t>& keyDimMap,
                                   const gtsam::Values& values, PoseTransformation& poseTransformation,
                                   DerivativeDirection derivativeDirection);

// Method to build the relative Jacobian used for converting H,b in convertHAndBWithPoseTransformation and
// for the PoseTransformationFactor.
gtsam::Matrix
buildRelativeJacobian(int n, const gtsam::FastVector<gtsam::Key>& ordering, const std::vector<int>& dimensions,
                      const gtsam::Values& values, PoseTransformation& poseTransformation,
                      const DerivativeDirection& derivativeDirection, int numOpt,
                      const std::vector<int>& optPositions);

// Converts the DSO coarse tracking Hessian to GTSAM, transforming it with the PoseTransformation.
// The passed transformIMUToCoarse needs to provide derivatives for frameToWorld and for referenceToWorld (see TransformIMUToDSOForCoarse as an example).
std::pair<gtsam::Matrix, gtsam::Vector>
convertCoarseHToGTSAM(PoseTransformation& transformIMUToCoarse, const dso::Mat88& HInput, const dso::Vec8& bInput,
                      const gtsam::Pose3& currentPose);

// Helper function to convert Jacobians from DSO style to GTSAM: This switches rotation and translation because
// Sophus (used by DSO) and GTSAM have different conventions here.
Eigen::MatrixXd convertJacobianToGTSAM(const Eigen::MatrixXd& jacobian);

// Convert all poses in keysToInclude with the transformation.
void convertAllPosesWithTransform(const gtsam::Values& values, const PoseTransformation& transformation,
                                  const gtsam::KeyVector& keysToInclude, gtsam::Values& insertInto);

inline gtsam::Values convertAllPosesWithTransform(const gtsam::Values& values, const PoseTransformation& transformation,
                                                  const gtsam::KeyVector& keysToInclude)
{
    gtsam::Values returning;
    convertAllPosesWithTransform(values, transformation, keysToInclude, returning);
    return returning;
}

inline gtsam::Values
convertAllPosesWithTransform(gtsam::Values::shared_ptr values, const PoseTransformation& transformation)
{
    return convertAllPosesWithTransform(*values, transformation, values->keys());
}

// Returns true if all symbols optimized by the poseTransformation are contained inside values.
inline bool allOptimizedSymbolsInside(const gtsam::Values& values, const PoseTransformation& poseTransformation)
{
    for(auto&& key : poseTransformation.getAllOptimizedSymbols())
    {
        if(!values.exists(key)) return false;
    }
    return true;
}


// Assert that a numeric Jacobian is similar to analytic Jacobian.
template<typename T> inline void assertNumericJac(const gtsam::Matrix& numericJac, const T& analyticJac)
{
    if(!(analyticJac - numericJac).isZero(0.0001))
    {
        std::cout << "AnalyticJ:\n" << analyticJac << "\nNumJ: \n" << numericJac << std::endl;
        std::cout << "Diff:\n" << (analyticJac - numericJac) << std::endl;
        assert(0);
    }
}

// Compute the numeric Jacobian w.r.t the given variable, which can either be the argument (pose) itself or one of the members of transformation.
// Note that this method modifies the passed variableToChange (which can be an internal member of the PoseTransformation in some cases),
// so this method is typically very far from thread-safe!
template<typename T> gtsam::Matrix
computeNumericJacobian(PoseTransformation& transformation, const Sophus::SE3d& pose, T* variableToChange,
                       DerivativeDirection direction)
{
#ifndef DEBUG
    std::cout << "Using Numeric Jacobian!" << std::endl;
#endif
    double epsilon = 0.00001;

    gtsam::Matrix fullDerivative = gtsam::Matrix::Zero(6, T::DoF);

    // Numeric Jacobians work by slightly changing the pose in each direction, and then computing how much it effects the
    // converted pose.
    dso::Mat44 transformedPose = transformation.transformPose(pose.matrix());
    Sophus::SE3d transformedPoseInv = Sophus::SE3d(transformedPose).inverse();
    for(int i = 0; i < T::DoF; ++i)
    {
        gtsam::Vector incVec = gtsam::Vector::Zero(T::DoF);
        incVec(i) = epsilon;

        T variableBackup = *variableToChange;
        if(direction == dmvio::DerivativeDirection::RIGHT_TO_RIGHT ||
           direction == dmvio::DerivativeDirection::RIGHT_TO_LEFT)
        {
            *variableToChange = *variableToChange * T::exp(incVec);
        }else
        {
            *variableToChange = T::exp(incVec) * *variableToChange;
        }

        Sophus::SE3d transformedPoseNew(transformation.transformPose(pose.matrix()));
        *variableToChange = variableBackup;
        Sophus::SE3d relPose;
        if(direction == dmvio::DerivativeDirection::LEFT_TO_LEFT ||
           direction == dmvio::DerivativeDirection::RIGHT_TO_LEFT)
        {
            relPose = transformedPoseNew * transformedPoseInv;
        }else
        {
            relPose = transformedPoseInv * transformedPoseNew;
        }

        dso::Vec6 derivative = Sophus::SE3d::log(relPose);
        fullDerivative.col(i) = derivative / epsilon;
    }

    return convertJacobianToGTSAM(fullDerivative);
}
}

#endif //DMVIO_POSETRANSFORMATION_H
