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
#include "fusion.h"

using namespace mosek::fusion;
using namespace monty;


template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>,
	typename MatrixT=Eigen::Matrix<double, DIM, DIM>,
	typename std::enable_if<coterie::Dynamic != DIM>::type* = nullptr >
mosek::fusion::Matrix::t quadForm(const coterie::EllipsoidalSet<DIM, PointT, MatrixT>& es)
{
	auto Ac = es.A * es.c;
	Eigen::Matrix<double, DIM+1, DIM+1> Q;
	Q << es.A, -Ac, -Ac.transpose(), (es.c.transpose() * Ac) - 1.0;

	return Matrix::dense(std::make_shared<ndarray<double, 2>>(Q.data(), shape_t<2>{DIM+1, DIM+1}));
}


template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>,
	typename MatrixT=Eigen::Matrix<double, DIM, DIM>,
	typename std::enable_if<coterie::Dynamic == DIM>::type* = nullptr >
mosek::fusion::Matrix::t quadForm(const coterie::EllipsoidalSet<DIM, PointT, MatrixT>& es)
{
	auto Ac = es.A * es.c;
	Eigen::Matrix<double, coterie::Dynamic, coterie::Dynamic> Q(es.dimension + 1, es.dimension + 1);
	Q << es.A, -Ac, -Ac.transpose(), (es.c.transpose() * Ac) - 1.0;

	return Matrix::dense(std::make_shared<ndarray<double, 2>>(Q.data(), shape_t<2>{es.dimension + 1, es.dimension + 1}));
}

TEST(testEllipsoids, testMOSEK)
{
	Model::t M = new Model("sdo1"); auto _M = finally([&]() { M->dispose(); });

	Eigen::VectorXd c1(3);
	Eigen::MatrixXd B1(3,3);
	Eigen::VectorXd c2(3);
	Eigen::MatrixXd B2(3,3);

	c1 << 0.436432, 0.456719, 0.975518; // 0.001, 0, 0;
	c2 << 0, 0, 0;

	B1 = 10.0 * Eigen::Matrix3d::Identity();
	B2 = 0.1 * Eigen::Matrix3d::Identity();
	auto E1 = EllipsoidalSet<3>::BRep(c1, B1);
	auto E2 = EllipsoidalSet<3>::BRep(c2, B2);
//	ASSERT_TRUE(es.contains(E1, E2));

//	B2 << 0.180744, -0.0247337,  0.0578441,
//		-0.0247337,   0.139892,  0.0664183,
//		0.0578441,  0.0664183,   0.122801;
//	E2 = EllipsoidalSet<3>::BRep(c2, B2);

//	ASSERT_TRUE(es.contains(E1, E2));


	// Setting up the variables
	Variable::t x  = M->variable("x", Domain::greaterThan(0));

	// Setting up the constant coefficient matrices
	Matrix::t Q1 = quadForm(E1);
	Matrix::t Q2 = quadForm(E2);

	// Objective
	M->objective(ObjectiveSense::Minimize, Expr::zeros(1));

	// Constraints
	M->constraint("c1", Expr::sub(Expr::mul(x, Q2), Q1), Domain::inPSDCone(E2.dimension + 1));
//	M->constraint("c1", Expr::sub(Expr::mul(x, Q1), Q2), Domain::inPSDCone(E2.dimension + 1)); // Wrong...

	M->writeTask("/home/arprice/mosek_task.cbf");

	M->solve();


	std::cout << "Solution : " << std::endl;
//	std::cout << "  x = " << *(x->level()) << std::endl;
//	x->toString()
	std::cout << M->getProblemStatus() << std::endl;
	std::cout << M->getPrimalSolutionStatus() << std::endl;
	std::cout << M->getDualSolutionStatus() << std::endl;
	std::cout << M->getAcceptedSolutionStatus() << std::endl;
//	FAIL();



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

    return RUN_ALL_TESTS();
}
