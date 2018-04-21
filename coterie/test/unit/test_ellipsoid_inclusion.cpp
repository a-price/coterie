/**
 * \file test_ellipsoid_inclusion.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2017-9-10
 *
 * \copyright
 *
 * Copyright (c) 2017, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "coterie/EllipsoidSolver.h"

#include <ros/ros.h>

#include <gtest/gtest.h>

using namespace coterie;

TEST(testEllipsoids, test1D)
{
	EllipsoidSolver& es = EllipsoidSolver::getInstance();

	Eigen::VectorXd c1(1);
	Eigen::MatrixXd Q1(1,1);
	Eigen::VectorXd c2(1);
	Eigen::MatrixXd Q2(1,1);

	c1 << 0; Q1 << 2;
	c2 << 0; Q2 << 1;
	ASSERT_TRUE(es.contains(EllipsoidalSet<1>::BRep(c1, Q1), EllipsoidalSet<1>::BRep(c2, Q2)));

	c1 << 1; Q1 << 1;
	c2 << 1; Q2 << 1;
	ASSERT_TRUE(es.contains(EllipsoidalSet<1>::BRep(c1, Q1), EllipsoidalSet<1>::BRep(c2, Q2)));

	c1 << 0; Q1 << 1;
	c2 << 0; Q2 << 2;
	ASSERT_FALSE(es.contains(EllipsoidalSet<1>::BRep(c1, Q1), EllipsoidalSet<1>::BRep(c2, Q2)));

	c1 << 0; Q1 << 2;
	c2 << 0.25; Q2 << 1;
	ASSERT_TRUE(es.contains(EllipsoidalSet<1>::BRep(c1, Q1), EllipsoidalSet<1>::BRep(c2, Q2)));

	c1 << 0; Q1 << 2;
	c2 << 2; Q2 << 1;
	ASSERT_FALSE(es.contains(EllipsoidalSet<1>::BRep(c1, Q1), EllipsoidalSet<1>::BRep(c2, Q2)));

	c1 << 0; Q1 << 2;
	c2 << 3; Q2 << 1;
	ASSERT_FALSE(es.contains(EllipsoidalSet<1>::BRep(c1, Q1), EllipsoidalSet<1>::BRep(c2, Q2)));

}

TEST(testEllipsoids, test3D)
{
	EllipsoidSolver& es = EllipsoidSolver::getInstance();

	Eigen::VectorXd c1(3);
	Eigen::MatrixXd Q1(3,3);
	Eigen::VectorXd c2(3);
	Eigen::MatrixXd Q2(3,3);

	c1 << 0.436432, 0.456719, 0.975518; // 0.001, 0, 0;
	c2 << 0, 0, 0;

	Q1 = 10.0 * Eigen::Matrix3d::Identity();
	Q2 = 0.1 * Eigen::Matrix3d::Identity();
	ASSERT_TRUE(es.contains(EllipsoidalSet<3>::BRep(c1, Q1), EllipsoidalSet<3>::BRep(c2, Q2)));

	Q2 << 0.180744, -0.0247337,  0.0578441,
		-0.0247337,   0.139892,  0.0664183,
		 0.0578441,  0.0664183,   0.122801;

	ASSERT_TRUE(es.contains(EllipsoidalSet<3>::BRep(c1, Q1), EllipsoidalSet<3>::BRep(c2, Q2)));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_ellipsoid_inclusion");

    return RUN_ALL_TESTS();
}
