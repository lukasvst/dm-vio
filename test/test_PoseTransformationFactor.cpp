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


#include <gtest/gtest.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include "GTSAMIntegration/PoseTransformationFactor.h"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <GTSAMIntegration/PoseTransformationIMU.h>
#include <GTSAMIntegration/Sim3GTSAM.h>

#include <memory>

using namespace gtsam;
using namespace dmvio;
using symbol_shorthand::P, symbol_shorthand::S;


class SimpleGraphTest : public ::testing::Test
{
protected:
    gtsam::NonlinearFactorGraph graph;
    Values values;
    Values trueValues;
    BetweenFactor<Pose3>::shared_ptr between01;
    BetweenFactor<Pose3>::shared_ptr between12;

    gtsam::SharedNoiseModel priorModel = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
    gtsam::SharedNoiseModel betweenModel = noiseModel::Diagonal::Sigmas((Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

    void SetUp() override
    {
        // P0 at (0, 0, 0)
        // P1 at (2, 0, 0)
        // P2 at (4, 0, 0)
        // scale is 2
        // Between factor in IMU scale:
        // between P0-P1 and P1-P2: (1, 0, 0)
        Pose3 pose0(gtsam::Rot3::identity(), gtsam::Point3(0.0, 0.0, 0.0));
        Pose3 pose1(gtsam::Rot3::identity(), gtsam::Point3(2.0, 0.0, 0.0));
        Pose3 pose2(gtsam::Rot3::identity(), gtsam::Point3(4.0, 0.0, 0.0));

        // We use the inverse, because TransformDSOToIMUNew also converts from worldToCam to camToWorld.
        pose0 = pose0.inverse();
        pose1 = pose1.inverse();
        pose2 = pose2.inverse();

        trueValues.insert(P(0), pose0);
        trueValues.insert(P(1), pose1);
        trueValues.insert(P(2), pose2);

        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(P(0), pose0, priorModel));
        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(P(2), pose2, priorModel));

        values.insert(P(0), Pose3{});
        values.insert(P(1), Pose3{});
        values.insert(P(2), Pose3{});

        between01.reset(new BetweenFactor<Pose3>(P(0), P(1),
                                                                            gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(1.0, 0.0, 0.0)), betweenModel));
        between12.reset(new BetweenFactor<Pose3>(P(1), P(2),
                                                 gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(1.0, 0.0, 0.0)), betweenModel));
    }
};

class SimpleGraphTestsWithParams : public SimpleGraphTest, public ::testing::WithParamInterface<PoseTransformationFactor::ConversionType>
{
};

TEST_F(SimpleGraphTest, NoPoseTransformation)
{
    graph.push_back(between01);
    graph.push_back(between12);
    LevenbergMarquardtOptimizer optimizer(graph, values);
    Values result = optimizer.optimize();
    EXPECT_FALSE(trueValues.equals(result, 0.1));
}

TEST_P(SimpleGraphTestsWithParams, WithTransformDSOToIMUNew)
{
    PoseTransformationFactor::ConversionType conversionType = GetParam();
    std::shared_ptr<TransformDSOToIMU> transform(
            new TransformDSOToIMU(gtsam::Pose3::identity(), std::make_shared<bool>(true),
                                  std::make_shared<bool>(false), std::make_shared<bool>(false), true, 0));

    graph.push_back(boost::make_shared<PoseTransformationFactor>(between01, *transform, conversionType));
    graph.push_back(boost::make_shared<PoseTransformationFactor>(between12, *transform, conversionType));

    values.insert(S(0), ScaleGTSAM(1.0));
    trueValues.insert(S(0), ScaleGTSAM(0.5));
    LevenbergMarquardtParams params = LevenbergMarquardtParams::CeresDefaults();
    LevenbergMarquardtOptimizer optimizer(graph, values, params);
    Values result = optimizer.optimize();
    EXPECT_TRUE(assert_equal(trueValues, result));

}

TEST(ValuesTest, ValuesPointerChange)
{
    Values values;
    values.insert(S(0), ScaleGTSAM(2.0));
    values.insert(S(1), 2.0);
    gtsam::Vector3 vector = gtsam::Vector3::Identity();
    values.insert(S(2), vector);

    const Values* pointerBefore = &values;
    EXPECT_TRUE(pointerBefore == &values);

    VectorValues incVec;
    gtsam::Vector1 inc;
    inc(0) = 0.001;
    incVec.insert(S(0), inc);
    values = values.retract(incVec);
}

INSTANTIATE_TEST_SUITE_P(PoseTransformationTests, SimpleGraphTestsWithParams,
                         ::testing::Values(PoseTransformationFactor::JACOBIAN_BAKED_IN, PoseTransformationFactor::JACOBIAN_FACTOR));




























