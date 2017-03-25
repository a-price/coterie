/**
 * \file test_point_set.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-11-21
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

#include "coterie/PointSet.hpp"

#include <list>

#include <gtest/gtest.h>

template <typename PS>
void testMembership(PS ps)
{
	ps.insert({0,0});
	ps.insert({0,1.5});
	ps.insert({-1,12});
	ps.insert({-5,-2});
	ps.insert({3,6});
	ps.insert({3.14159,2.71828});

	ASSERT_TRUE(ps.contains({0,0}));
	ASSERT_TRUE(ps.contains({3.14159,2.71828}));
	ASSERT_FALSE(ps.contains({1,1}));

	coterie::AABB<2> aabb = ps.getAABB();
	for (const typename PS::point_type& p : ps.members)
	{
		ASSERT_TRUE(aabb.contains(p));
	}
}

TEST(PointSet, testContainsSet)
{
	coterie::PointSet<2> ps;

	testMembership(ps);

	// Verify direct member insert
	ps.members.insert({-1, -1});
}

TEST(PointSet, testContainsVector)
{
	coterie::PointSet<2, Eigen::Vector2d, std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>> ps;

	testMembership(ps);

	// Verify direct member insert
	ps.members.push_back({-1, -1});
}

TEST(PointSet, testContainsList)
{
	coterie::PointSet<2, Eigen::Vector2d, std::list<Eigen::Vector2d>> ps;

	testMembership(ps);

	// Verify direct member insert
	ps.members.push_back({-1, -1});
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
