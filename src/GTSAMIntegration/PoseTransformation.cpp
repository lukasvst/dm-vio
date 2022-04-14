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

#include <util/TimeMeasurement.h>
#include "PoseTransformation.h"
#include "dso/util/FrameShell.h"
#include "ExtUtils.h"
#include "Sim3GTSAM.h"

using namespace dmvio;
using dso::Vec8;
using gtsam::Matrix, gtsam::Vector;
using gtsam::Symbol;

// Default implementation for child classes uses numeric Jacobians and should work out of the box.
gtsam::Matrix66 PoseTransformation::getPoseDerivative(const PoseType& posePassed, DerivativeDirection direction)
{
    // copy, because it needs to be changed by computeNumericJacobian.
    Sophus::SE3d pose(posePassed);
    return computeNumericJacobian(*this, pose, &pose, direction);
}

std::vector<gtsam::Matrix>
PoseTransformation::getAllDerivatives(const PoseType& pose, DerivativeDirection direction)
{
    std::vector<gtsam::Matrix> returning;
    returning.push_back(getPoseDerivative(pose, direction));
    return returning;
}

std::vector<gtsam::Key> PoseTransformation::getAllOptimizedSymbols() const
{
    return std::vector<gtsam::Key>();
}

gtsam::Matrix66 TransformIdentity::getPoseDerivative(const PoseType& pose, DerivativeDirection direction)
{
    // Analytic derivative only implemented for RIGHT_to_LEFT at the moment.
    if(direction == DerivativeDirection::RIGHT_TO_LEFT)
    {
        gtsam::Matrix66 returning = gtsam::Pose3(pose).AdjointMap();
#ifdef DEBUG
        // Check numeric jacobian.
        Sophus::SE3d poseForNum(pose);
        gtsam::Matrix numJac = computeNumericJacobian(*this, poseForNum, &poseForNum, direction);
        assertNumericJac(numJac, returning);
#endif
        return returning;
    }
    return PoseTransformation::getPoseDerivative(pose, direction); // Use numeric derivative if not implemented.
}

// Exchanges rotation and translation (the first 6 rows/columns) in a Jacobian matrix.
Eigen::MatrixXd dmvio::convertJacobianToGTSAM(const Eigen::MatrixXd& jacobian)
{
    Eigen::MatrixXd ret = jacobian;
    int rows = jacobian.rows();
    int cols = jacobian.cols();

    // first exchange rows
    ret.block(0, 0, 3, cols) = jacobian.block(3, 0, 3, cols);
    ret.block(3, 0, 3, cols) = jacobian.block(0, 0, 3, cols);

    // then exchange columns (unless the derivative is w.r.t an SO(3) element).
    if(cols > 3)
    {
        Eigen::MatrixXd tmp = ret.block(0, 0, rows, 3);
        ret.block(0, 0, rows, 3) = ret.block(0, 3, rows, 3);
        ret.block(0, 3, rows, 3) = tmp;
    }

    return ret;
}

void dmvio::convertAllPosesWithTransform(const gtsam::Values& values, const PoseTransformation& transformation,
                                         const gtsam::KeyVector& keysToInclude, gtsam::Values& insertInto)
{
    for(int i = 0; i < keysToInclude.size(); ++i)
    {
        gtsam::Key key = keysToInclude.at(i);
        if(gtsam::Symbol(key).chr() == 'p')
        {
            gtsam::Pose3 pose = values.at<gtsam::Pose3>(key);
            insertInto.insert(key, gtsam::Pose3(transformation.transformPose(pose.matrix())));
        }else
        {
            insertInto.insert(key, values.at(key));
        }
    }
}

std::pair<gtsam::Matrix, gtsam::Vector>
dmvio::convertHAndBFromDSO(const dso::MatXX& HInput, const dso::VecX& bInput, PoseTransformation& poseTransformation,
                           double weightDSOToGTSAM, const gtsam::Ordering& ordering, const gtsam::Values& values,
                           const std::map<gtsam::Key, size_t>& keyDimMap)
{
    dmvio::TimeMeasurement timeMeasurement("convertHAndBFromDSO");
    int n = HInput.rows();
    assert(HInput.cols() == n);

    gtsam::Matrix H = HInput;
    gtsam::Vector b = bInput;

    // Here we use the fact that the Hessian from DSO will always contain first calibration parameters, and then poses (each followed by a and b).
    int numCPARS = 0;
    for(auto&& key : ordering)
    {
        if(Symbol(key).chr() != 'c') break;
        numCPARS += keyDimMap.at(key);
    }

    int numFrames = (n - numCPARS) / 8;
    assert(n == numFrames * 8 + numCPARS);

    // Exchange rotation with translation for all frames (because the order is different in GTSAM!)
    // First exchange the rows.
    for(int i = 0; i < numFrames; ++i)
    {
        int id = numCPARS + 8 * i;

        H.block(id, 0, 3, n) = HInput.block(id + 3, 0, 3, n);
        H.block(id + 3, 0, 3, n) = HInput.block(id, 0, 3, n);

        b.segment(id, 3) = bInput.segment(id + 3, 3);
        b.segment(id + 3, 3) = bInput.segment(id, 3);
    }

    // Then exchange the column
    for(int i = 0; i < numFrames; ++i)
    {
        int id = numCPARS + 8 * i;
        dso::MatXX tmp = H.block(0, id, n, 3);
        H.block(0, id, n, 3) = H.block(0, id + 3, n, 3);
        H.block(0, id + 3, n, 3) = tmp;
    }

    // multiply with weight
    H *= weightDSOToGTSAM;
    b *= weightDSOToGTSAM;

    // DSO has left sided increment and GTSAM has right sided increment.
    // For converting the Hessian the derivative is always the other way round, so here we have to pass RIGHT_TO_LEFT.
    auto pair = convertHAndBWithPoseTransformation(std::make_pair(H, b), ordering, keyDimMap, values,
                                                   poseTransformation,
                                                   DerivativeDirection::RIGHT_TO_LEFT);
    // DSO uses -b compared to GTSAM.
    pair.second = -pair.second;
    return pair;
}


std::pair<gtsam::Matrix, gtsam::Vector>
dmvio::convertHAndBWithPoseTransformation(const std::pair<gtsam::Matrix, gtsam::Vector>& input,
                                          const gtsam::Ordering& ordering,
                                          const std::map<gtsam::Key, size_t>& keyDimMap,
                                          const gtsam::Values& values, PoseTransformation& poseTransformation,
                                          DerivativeDirection derivativeDirection)
{
    // Find out positions of optimized transformation variables in the Hessian.
    auto&& optimizedVariables = poseTransformation.getAllOptimizedSymbols();
    int numOpt = optimizedVariables.size();
    std::vector<int> optPositions(numOpt, -1);
    if(numOpt > 0)
    {
        int numToFindOut = numOpt;
        int pos = 0;
        for(auto&& key : ordering)
        {
            int size = keyDimMap.at(key);
            for(int i = 0; i < optimizedVariables.size(); ++i)
            {
                if(key == optimizedVariables[i])
                {
                    optPositions[i] = pos;
                    numToFindOut--;
                }
            }
            if(numToFindOut == 0) break;
            pos += size;
        }
        assert(numToFindOut == 0);
    }

    // buildRelativeJacobian expects vector of sizes.
    std::vector<int> sizes;
    sizes.reserve(ordering.size());
    for(auto&& key : ordering)
    {
        sizes.push_back(keyDimMap.at(key));
    }

    // Build relative Jacobian
    int n = input.first.rows();
    gtsam::Matrix bigJ = buildRelativeJacobian(n, ordering, sizes, values, poseTransformation,
                                               derivativeDirection, numOpt,
                                               optPositions);

    gtsam::Matrix H = gtsam::Matrix::Zero(n, n);
    gtsam::Vector b = gtsam::Matrix::Zero(n, 1);

    // Multiply Hessian and b with the relative Jacobian.
    H = bigJ.transpose() * input.first * bigJ;
    b = bigJ.transpose() * input.second;

    return std::make_pair(H, b);
}

gtsam::Matrix
dmvio::buildRelativeJacobian(int n, const gtsam::FastVector<gtsam::Key>& ordering, const std::vector<int>& dimensions,
                             const gtsam::Values& values, PoseTransformation& poseTransformation,
                             const DerivativeDirection& derivativeDirection, int numOpt,
                             const std::vector<int>& optPositions)
{
    gtsam::Matrix bigJ = gtsam::Matrix::Identity(n, n);

    poseTransformation.precomputeForDerivatives();
    int pos = 0;
    for(int i = 0; i < ordering.size(); ++i)
    {
        gtsam::Key key = ordering[i];
        int size = dimensions[i];
        unsigned char chr = gtsam::Symbol(key).chr();
        if(chr == 'p') // Only convert poses.
        {
            gtsam::Pose3 pose = values.at<gtsam::Pose3>(key);
            // Get all derivatives from the poseTransformation.
            std::vector<Matrix> derivatives = poseTransformation.getAllDerivatives(pose.matrix(), derivativeDirection);

            // First put in pose derivative:
            bigJ.block(pos, pos, 6, 6) = derivatives[0];

            for(int j = 0; j < numOpt; ++j)
            {
                int optPos = optPositions[j];
                // At pos 1 because first one is pose derivative.
                auto&& deriv = derivatives[j + 1];
                assert(optPos >= 0);
                bigJ.block(pos, optPos, 6, deriv.cols()) = deriv;
            }
        }

        pos += size;
    }
    return bigJ;
}

std::pair<gtsam::Matrix, gtsam::Vector>
dmvio::convertCoarseHToGTSAM(PoseTransformation& transform, const dso::Mat88& HInput, const dso::Vec8& bInput,
                             const gtsam::Pose3& currentPose)
{
    // Correct order (switch translation and rotation and put the affine parameters to the front!)
    // The Sophus twist constains first translation and then rotation.
    // The GTSAM Pose3 contains first rotation and then translation!
    gtsam::Matrix8 H;
    gtsam::Vector8 b;

    H.block(2, 2, 3, 3) = HInput.block(3, 3, 3, 3);
    H.block(5, 5, 3, 3) = HInput.block(0, 0, 3, 3);

    H.block(5, 2, 3, 3) = HInput.block(0, 3, 3, 3);
    H.block(2, 5, 3, 3) = HInput.block(3, 0, 3, 3);

    H.block(0, 2, 2, 3) = HInput.block(6, 3, 2, 3);
    H.block(0, 5, 2, 3) = HInput.block(6, 0, 2, 3);

    H.block(2, 0, 3, 2) = HInput.block(3, 6, 3, 2);
    H.block(5, 0, 3, 2) = HInput.block(0, 6, 3, 2);

    H.block(0, 0, 2, 2) = HInput.block(6, 6, 2, 2);

    b.segment(2, 3) = bInput.segment(3, 3);
    b.segment(5, 3) = bInput.segment(0, 3);
    b.segment(0, 2) = bInput.segment(6, 2);

    // Compute Jacobian of frame and reference pose with respect to T_f_r
    gtsam::Matrix J(6, 12);
    std::vector<gtsam::Matrix> derivatives = transform.getAllDerivatives(currentPose.matrix(),
                                                                         DerivativeDirection::RIGHT_TO_LEFT);

    J.block(0, 6, 6, 6) = derivatives[0]; // J with respect to T_w_f
    J.block(0, 0, 6, 6) = derivatives[1]; // J with respect to T_w_r

    gtsam::Matrix JReal(8, 14);
    JReal.block(2, 2, 6, 12) = J;
    JReal.block(0, 2, 2, 12) = gtsam::Matrix::Zero(2, 12);
    JReal.block(0, 0, 2, 2) = gtsam::Matrix::Identity(2, 2);
    JReal.block(2, 0, 6, 2) = gtsam::Matrix::Zero(6, 2);

    gtsam::Matrix H_full = JReal.transpose() * H * JReal;
    gtsam::Vector b_full = JReal.transpose() * b;
    return std::make_pair(H_full, b_full);
}