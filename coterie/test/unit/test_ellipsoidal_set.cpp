/**
 * \file test_ellipsoidal_set.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-11-23
 *
 * \copyright
 *
 * Copyright (c) 2016, Georgia Tech Research Corporation
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

#include "coterie/EllipsoidalSet.hpp"
#include "coterie/subsets.hpp"
#include "coterie/construction.hpp"

#include <gtest/gtest.h>

// For debugging
#ifndef VISUALIZE_TEST
#define VISUALIZE_TEST false
#endif

#if VISUALIZE_TEST
#include "coterie/visualization/point_set.h"
#include "coterie/visualization/ellipsoidal_set.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#endif

// Not sure why we need this and can't link to the one in template_instantiations
template<int DIM, typename PointT>
constexpr int coterie::Set<DIM, PointT>::dimension;


TEST(EllipsoidalSet, testContains)
{
	// Test unit circle
	auto es = coterie::EllipsoidalSet<2>::ARep(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity());

	ASSERT_TRUE(es.contains({0,0}));
	ASSERT_TRUE(es.contains({0.5,-0.5}));
	ASSERT_FALSE(es.contains({3.14159,2.71828}));
	ASSERT_FALSE(es.contains({1,1}));

	coterie::AABB<2> aabb = es.getAABB();
	for (int i = 0; i < 2; ++i)
	{
		ASSERT_NEAR(aabb.min[i], -1, 1e-6);
		ASSERT_NEAR(aabb.max[i],  1, 1e-6);
	}

	ASSERT_TRUE(aabb.contains({0,0}));
	ASSERT_TRUE(aabb.contains({0.5,-0.5}));
	ASSERT_FALSE(aabb.contains({3.14159,2.71828}));
	ASSERT_TRUE(aabb.contains({1,1})); // AABB inflates ellipsoid

	// Squashed ellipsoid
	Eigen::Matrix2d Q; Q << 1.0/pow(2.0,2), 0, 0, 1.0/pow(0.5,2);
	es = coterie::EllipsoidalSet<2>::ARep(Eigen::Vector2d::Zero(), Q);

	ASSERT_TRUE(es.contains({0,0}));
	ASSERT_TRUE(es.contains({0.5,-0.25}));
	ASSERT_FALSE(es.contains({3.14159,2.71828}));
	ASSERT_FALSE(es.contains({1,1}));
	ASSERT_TRUE(es.contains({-1.5,0.25}));

	// Random Ellipsoid
	Eigen::Matrix2d M = Eigen::Matrix2d::Identity() + Eigen::Matrix2d::Random();
	es = coterie::EllipsoidalSet<2>::ARep(10.0*Eigen::Vector2d::Random(), 10.0*M*M.transpose());

	ASSERT_TRUE(es.contains(es.c));

	aabb = es.getAABB();

	ASSERT_TRUE(aabb.contains(es.c));

}


using Kernel = CGAL::Cartesian_d<double>;
using ET = CGAL::MP_Float;
using Traits = CGAL::Approximate_min_ellipsoid_d_traits_d<Kernel,ET>;
using Point = Traits::Point;
using Point_list = std::vector<Point>;
//using Point = Eigen::Vector2d;
//using Point_list = std::vector<Point, Eigen::aligned_allocator<Point>>;
using AME = CGAL::Approximate_min_ellipsoid_d<Traits>;

// https://doc.cgal.org/latest/Bounding_volumes/classCGAL_1_1Approximate__min__ellipsoid__d.html
TEST(EllipsoidalSet, test_fit_internals)
{
	const int      n = 1000;                // number of points
	const int      d = 2;                   // dimension
	const double eps = 0.01;                // approximation ratio is (1+eps)
	const double side = 100.0;

	// create a set of random points:
	Point_list P;
	CGAL::Random_points_in_cube_d<Point> rpg(d,side);
	for (int i = 0; i < n; ++i)
	{
		P.push_back(*rpg);
		++rpg;
	}
	// compute approximation:
	Traits traits;
	AME ame(eps, P.begin(), P.end(), traits);

	Eigen::VectorXd c = Eigen::VectorXd::Zero(d);

	// output center coordinates:
	const AME::Center_coordinate_iterator c_begin = ame.center_cartesian_begin();
	for (AME::Center_coordinate_iterator c_it = c_begin;
	     c_it != ame.center_cartesian_end();
	     ++c_it)
	{
		c[c_it-c_begin] = *c_it;
	}
	std::cout << "Cartesian center coordinates: ";
	std::cout << c.transpose();
	std::cout << ".\n";

	// https://en.wikipedia.org/wiki/Ellipsoid#As_quadric
	Eigen::VectorXd eigvals = Eigen::VectorXd::Zero(d);
	Eigen::MatrixXd eigvecs = Eigen::MatrixXd::Zero(d, d);

	if (d == 2 || d == 3)
	{
		// output  axes:
		AME::Axes_lengths_iterator axes = ame.axes_lengths_begin();
		for (int i = 0; i < d; ++i)
		{
			double len = *axes++;
			double eigval = 1.0/(len*len);
			eigvals[i] = eigval;
			std::cout << "Semiaxis " << i << " has length " << len << " (eig: " << eigval << ")\n"
			          << "and Cartesian coordinates ";

			AME::Axes_direction_coordinate_iterator d_begin = ame.axis_direction_cartesian_begin(i);
			for (AME::Axes_direction_coordinate_iterator d_it = d_begin;
			     d_it != ame.axis_direction_cartesian_end(i); ++d_it)
			{
				eigvecs.col(i)[d_it-d_begin] = *d_it;
				std::cout << *d_it << ' ';
			}
			std::cout << ".\n";
		}
	}

	for (int i = 0; i < d; ++i)
	{
		ASSERT_GE(eigvals[i], 0.0);
		ASSERT_LT(fabs(1.0-eigvecs.col(i).norm()), 1e-9);
		ASSERT_LT(eigvecs.col(i).dot(eigvecs.col((i+1)%d)), 1e-9);
	}

	// https://en.wikipedia.org/wiki/Eigendecomposition_of_a_matrix#Eigendecomposition_of_a_matrix
	// https://eigen.tuxfamily.org/dox/classEigen_1_1EigenSolver.html
	// NB: inverse() is ok here b/c eigvecs are lin. ind. and dim < 4
	Eigen::MatrixXd A = eigvecs * eigvals.asDiagonal() * eigvecs.inverse();

	std::cout << "A: \n" << A << std::endl;


	double eta = ame.defining_scalar();
	std::cout << "Epsilon: " << ame.achieved_epsilon() << std::endl;
	std::cout << "Eta: " << eta << std::endl;

	auto ell = coterie::EllipsoidalSet<-1>::ARep(c, A);

	coterie::PointSet<-1> ps(d);
	for (int i = 0; i < n; ++i)
	{
		Eigen::VectorXd p = Eigen::VectorXd::Zero(d);
		for (int j = 0; j < d; ++j) { p[j] = P[i].cartesian(j); }
		ASSERT_TRUE(ell.contains(p));
		ps.insert(p);
	}

	ASSERT_TRUE(coterie::contains(ell, ps));
}

template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>>
class PSI
{
public:
	static PointT initPoint(const int) { return PointT(); }
	static coterie::PointSet<DIM, PointT> initSet(const int) { return coterie::PointSet<DIM, PointT>(); }
};

template<>
class PSI<-1, Eigen::Matrix<double, -1, 1>>
{
public:
	using PointT = Eigen::Matrix<double, -1, 1>;
	static PointT initPoint(const int dim) { return  PointT(dim); }
	static coterie::PointSet<-1, PointT> initSet(const int dim) { return coterie::PointSet<-1, PointT>(dim); }
};

template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>>
void test_optimistic_fit()
{
	const int      n = 1000;                // number of points
	const int      d = (DIM < 0) ? 2 : DIM; // dimension
	const double side = 100.0;

	// create a set of random points:
	coterie::PointSet<DIM, PointT> ps = PSI<DIM, PointT>::initSet(d);

	ASSERT_EQ(d, ps.dimension);

	CGAL::Random_points_in_cube_d<Point> rpg(d,side);
	for (int i = 0; i < n; ++i)
	{
		Point p = *rpg;
		PointT q = PSI<DIM, PointT>::initPoint(d);
		for (int j = 0; j < d; ++j)
		{
			q[j] = p.cartesian(j);
		}
		ps.insert(q);
		++rpg;
	}

	coterie::EllipsoidalSet<DIM> ell = coterie::minVolumeEnclosingEllipsoid<DIM>(ps);

#if VISUALIZE_TEST
	static ros::NodeHandle nh;
	static ros::Publisher vizPub = nh.advertise<visualization_msgs::MarkerArray>("points", 1, true);
	visualization_msgs::MarkerArray ma;
	visualization_msgs::Marker marker = coterie::visualizePosition(ps);
	marker.ns = "points"+std::to_string(d);
	ma.markers.push_back(marker);
	marker = coterie::visualizePosition(ell);
	marker.ns = "ellipsoid"+std::to_string(d);
	ma.markers.push_back(marker);
	vizPub.publish(ma);
	ros::spinOnce();
	sleep(1);
#endif

	for (const PointT& m : ps.members)
	{
		ASSERT_TRUE(ell.contains(m));
	}

	ASSERT_TRUE(coterie::contains(ell, ps));

}

TEST(EllipsoidalSet, optimistic_fit)
{
	test_optimistic_fit<-1>();
	test_optimistic_fit<2>();
	test_optimistic_fit<3>();
}


int main(int argc, char **argv)
{
#if VISUALIZE_TEST
	ros::init(argc, argv, "test_ellipsoidal_set");
#endif
	::testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}
