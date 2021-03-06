/**
 * \file test_subset_relations.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-11-27
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

#include "coterie/subsets.hpp"

#include <random>
#include <gtest/gtest.h>

TEST(SubsetRelations, testPointSet)
{
	// Test unit circle
	auto es = coterie::EllipsoidalSet<2>::ARep(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity());

	coterie::PointSet<2> ps;
	ps.members.insert({0.1,0.1});
	ps.members.insert({0.5,-0.5});
	ps.members.insert({0.999,0});

	ASSERT_EQ(3, ps.members.size());
	ASSERT_TRUE(contains(es, ps));
	ASSERT_TRUE(contains(es.getAABB(), ps));

	ps.members.insert({1,1});

	ASSERT_EQ(4, ps.members.size());
	ASSERT_FALSE(contains(es, ps));

	coterie::PointSet<2> ps2(ps);
	ASSERT_EQ(ps.members.size(), ps2.members.size());
	ASSERT_TRUE(contains(ps, ps2));
	ASSERT_TRUE(contains(ps2, ps));

	// Test Polytope set
	coterie::PolytopeSet<2> poly(ps);
	ASSERT_TRUE(contains(poly, ps));
	ASSERT_TRUE(contains(poly.getAABB(), ps));

	ps.members.insert({-10,-10});

	ASSERT_FALSE(contains(poly, ps));
	ASSERT_FALSE(contains(poly.getAABB(), ps));
}

TEST(SubsetRelations, testDynamicPointSet)
{
	// Test unit circle
	auto es = coterie::EllipsoidalSet<coterie::Dynamic>::ARep(Eigen::VectorXd::Zero(2), Eigen::MatrixXd::Identity(2,2));

	coterie::PointSet<coterie::Dynamic> ps(2);
	ps.members.insert((Eigen::VectorXd(2) << 0.1,0.1).finished());
	ps.members.insert((Eigen::VectorXd(2) << 0.5,-0.5).finished());
	ps.members.insert((Eigen::VectorXd(2) << 0.999,0).finished());

	ASSERT_EQ(3, ps.members.size());
	ASSERT_TRUE(contains(es, ps));
	ASSERT_TRUE(contains(es.getAABB(), ps));

	ps.members.insert((Eigen::VectorXd(2) << 1,1).finished());

	ASSERT_EQ(4, ps.members.size());
	ASSERT_FALSE(contains(es, ps));

	coterie::PointSet<coterie::Dynamic> ps2(ps);
	ASSERT_EQ(ps.members.size(), ps2.members.size());
	ASSERT_TRUE(contains(ps, ps2));
	ASSERT_TRUE(contains(ps2, ps));

	// Test Polytope set
	coterie::PolytopeSet<coterie::Dynamic> poly(ps);
	ASSERT_TRUE(contains(poly, ps));
	ASSERT_TRUE(contains(poly.getAABB(), ps));

	ps.members.insert((Eigen::VectorXd(2) << -10,-10).finished());

	ASSERT_FALSE(contains(poly, ps));
	ASSERT_FALSE(contains(poly.getAABB(), ps));
}

TEST(SubsetRelations, testEllipsoidInPolytope)
{
	// Create a 3D Diamond
	coterie::PointSet<3> ps;
	ps.members.insert({0,0,2});
	ps.members.insert({0,0,-2});
	ps.members.insert({0,2,0});
	ps.members.insert({0,-2,0});
	ps.members.insert({2,0,0});
	ps.members.insert({-2,0,0});
	coterie::PolytopeSet<3> poly(ps);

	// Test unit sphere
	auto es = coterie::EllipsoidalSet<3>::ARep(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());

	ASSERT_TRUE(contains(poly, es));
	ASSERT_TRUE(contains(poly.getAABB(), es));

	// Create larger sphere (size = 3)
	es = coterie::EllipsoidalSet<3>::ARep(Eigen::Vector3d::Zero(), 1.0/pow(3.0,2) * Eigen::Matrix3d::Identity());

	ASSERT_FALSE(contains(poly, es));
	ASSERT_FALSE(contains(poly.getAABB(), es));
}

TEST(SubsetRelations, testEllipsoidInRaster)
{
	const int N = 3;
	coterie::RasterSet<N>::Shape shape{5,5,5};
	coterie::RasterSet<N>::Bounds bounds{{{-1,1},{-1,1},{-1,1}}};
	coterie::RasterSet<N> rs(shape, bounds);

	// Test unit sphere
	auto es = coterie::EllipsoidalSet<3>::ARep(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());

	const unsigned nElements = rs.data.num_elements();
	for (size_t i = 0; i < nElements; ++i)
	{
		coterie::RasterSet<N>::Index idx = rs.getCell(i);
		coterie::RasterSet<N>::point_type pt = rs.getState(idx);
		rs.data(idx) = es.contains(pt);
	}

	coterie::AABB<N> aabb = es.getAABB();
	coterie::AABB<N> aabb2 = rs.getAABB();
	ASSERT_LT((aabb.min-aabb2.min).squaredNorm(), 1e-6);
	ASSERT_LT((aabb.max-aabb2.max).squaredNorm(), 1e-6);

	coterie::RasterSetView<N> view = rs.getView(aabb);
	for (size_t i = 0; i < nElements; ++i)
	{
		coterie::RasterSet<N>::Index r_idx = rs.getCell(i);
		coterie::RasterSet<N>::Index v_idx = view.getCell(i);
		ASSERT_EQ(r_idx, v_idx);
		coterie::RasterSet<N>::point_type r_pt = rs.getState(r_idx);
		coterie::RasterSet<N>::point_type v_pt = view.getState(v_idx);
		ASSERT_TRUE(r_pt == v_pt); // Expect exactly equal
		ASSERT_EQ(rs.data(r_idx), es.contains(r_pt));
		ASSERT_EQ(view.dataView(v_idx), es.contains(v_pt));
	}


	ASSERT_TRUE(contains(rs, es));
	ASSERT_TRUE(contains(rs.getAABB(), es));

	coterie::RasterSet<N>::Index center{3,3,3};
	rs.data(center) = false;

	ASSERT_FALSE(contains(rs, es));
	ASSERT_TRUE(contains(rs.getAABB(), es));
}


TEST(SubsetRelations, testAABBInConvex)
{
	coterie::AABB<4> aabb;
	aabb.min = -0.5 * coterie::AABB<4>::point_type::Ones();
	aabb.max =  0.5 * coterie::AABB<4>::point_type::Ones();

	ASSERT_EQ(16, aabb.getCorners().size());

	// Create larger sphere (size = 2)
	auto es = coterie::EllipsoidalSet<4>::ARep(Eigen::Vector4d::Zero(), Eigen::Matrix4d::Identity());

	ASSERT_TRUE(contains(es, aabb));
	ASSERT_FALSE(contains(aabb, es));

	es = coterie::EllipsoidalSet<4>::ARep(Eigen::Vector4d::Zero(), 1.0/pow(0.4999,2) * Eigen::Matrix4d::Identity());

	ASSERT_FALSE(contains(es, aabb));
	ASSERT_TRUE(contains(aabb, es));

	// Create a 4D Diamond
	coterie::PointSet<4> ps;
	ps.members.insert({0,0,0,2});
	ps.members.insert({0,0,0,-2});
	ps.members.insert({0,0,2,0});
	ps.members.insert({0,0,-2,0});
	ps.members.insert({0,2,0,0});
	ps.members.insert({0,-2,0,0});
	ps.members.insert({2,0,0,0});
	ps.members.insert({-2,0,0,0});
	coterie::PolytopeSet<4> poly(ps);

	ASSERT_TRUE(contains(poly, aabb));
	ASSERT_FALSE(contains(aabb, poly));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
