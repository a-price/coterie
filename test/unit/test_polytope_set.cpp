/**
 * \file test_polytope_set.cpp
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

#include "coterie/PolytopeSet.hpp"

#include <gtest/gtest.h>

TEST(PolytopeSet, testContains)
{
	coterie::PointSet<2> points;
	for (size_t i = 0; i < 50; ++i)
	{
		points.members.insert(coterie::PointSet<2>::point_type::Random());
	}
	points.members.insert({ 1, 1});
	points.members.insert({ 1,-1});
	points.members.insert({-1, 1});
	points.members.insert({-1,-1});

	coterie::PolytopeSet<2> es(points);
	coterie::AABB<2> aabb = es.getAABB();

	for (size_t i = 0; i < 50; ++i)
	{
		coterie::PointSet<2>::point_type p = coterie::PointSet<2>::point_type::Random();
		ASSERT_TRUE(es.contains(p));
		ASSERT_TRUE(aabb.contains(p));
	}

	ASSERT_FALSE(es.contains({1.01,0}));
	ASSERT_FALSE(aabb.contains({1.01,0}));

	ASSERT_FALSE(es.contains({0,1.01}));
	ASSERT_FALSE(aabb.contains({0,1.01}));

	ASSERT_FALSE(es.contains({-1.01,0}));
	ASSERT_FALSE(aabb.contains({-1.01,0}));

	ASSERT_FALSE(es.contains({0,-1.01}));
	ASSERT_FALSE(aabb.contains({0,-1.01}));

}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
