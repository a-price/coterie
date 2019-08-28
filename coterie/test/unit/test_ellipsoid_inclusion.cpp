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

#if USE_MOSEK

#include "coterie/MosekEllipsoidSolver.hpp"
#include "coterie/subsets.hpp"

TEST(testEllipsoids, testMOSEK)
{
	using ::coterie::mosek::lhsContainsRhs;

	Eigen::VectorXd c1(1);
	Eigen::MatrixXd B1(1,1);
	Eigen::VectorXd c2(1);
	Eigen::MatrixXd B2(1,1);

	c1 << 0; B1 << 1;
	c2 << 0; B2 << 2;
	auto E1 = EllipsoidalSet<1>::BRep(c1, B1);
	auto E2 = EllipsoidalSet<1>::BRep(c2, B2);

	ASSERT_TRUE(lhsContainsRhs(E2, E1));
	ASSERT_TRUE(lhsContainsRhs(E1, E1));
	ASSERT_TRUE(lhsContainsRhs(E2, E2));
	ASSERT_FALSE(lhsContainsRhs(E1, E2));

	ASSERT_TRUE(contains(E2, E1));
	ASSERT_TRUE(contains(E1, E1));
	ASSERT_TRUE(contains(E2, E2));
	ASSERT_FALSE(contains(E1, E2));


	auto e1 = EllipsoidalSet<Dynamic>::BRep(c1, B1);
	auto e2 = EllipsoidalSet<Dynamic>::BRep(c2, B2);

	ASSERT_TRUE(lhsContainsRhs(e2, e1));
	ASSERT_TRUE(lhsContainsRhs(e1, e1));
	ASSERT_TRUE(lhsContainsRhs(e2, e2));
	ASSERT_FALSE(lhsContainsRhs(e1, e2));

//	ASSERT_TRUE(contains(e2, e1));
//	ASSERT_TRUE(contains(e1, e1));
//	ASSERT_TRUE(contains(e2, e2));
//	ASSERT_FALSE(contains(e1, e2));
}
#endif

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

	// MOSEK MWE

//	Model::t M = new Model("sdo1"); auto _M = finally([&]() { M->dispose(); });
//	Matrix::t S1  = Matrix::dense(new_array_ptr<double, 2>({{1.0, 0.0}, {0.0, -1.0}}));
//	Matrix::t S2  = Matrix::dense(new_array_ptr<double, 2>({{0.25, 0.0}, {0.0, -1.0}}));
//	Variable::t lambda = M->variable("lambda", Domain::greaterThan(0));
//	Variable::t X = M->variable("stub", Domain::inPSDCone(2));
//
//	M->objective(ObjectiveSense::Minimize, Expr::zeros(1));
//	M->constraint("c1", Expr::sub(X, Expr::sub(Expr::mul(lambda, S1), S2)), Domain::equalsTo(0));
//	M->solve();
//
//	M->writeTask("/home/arprice/mosek_task.ptf");
//
//	std::cout << M->getProblemStatus() << std::endl;
//
//	auto status = M->getProblemStatus();
//	switch(status)
//	{
//	case mosek::fusion::ProblemStatus::PrimalFeasible:
//	case mosek::fusion::ProblemStatus::PrimalAndDualFeasible:
//		std::cout << "Ellipsoid is contained." << std::endl;
//		break;
//	case ProblemStatus::DualInfeasible:
//		std::cout << "Dual infeasibility certificate found.\n";
//		std::cout << "Ellipsoid is not contained." << std::endl;
//		break;
//
//	case ProblemStatus::PrimalInfeasible:
//		std::cout << "Primal infeasibility certificate found.\n";
//		std::cout << "Ellipsoid is not contained." << std::endl;
//		break;
//
//	default:
//		std::stringstream str;
//		str << status;
//		throw std::logic_error("MOSEK return type is not handled in ellipsoid solver: '" + str.str() + "'.");
//	}


    return RUN_ALL_TESTS();
}
