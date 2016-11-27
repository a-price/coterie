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
	coterie::EllipsoidalSet<2> es(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity());

	coterie::PointSet<2> ps;
	ps.members.insert({0.1,0.1});
	ps.members.insert({0.5,-0.5});
	ps.members.insert({0.999,0});

	ASSERT_TRUE(contains(es, ps));
	ASSERT_TRUE(contains(es.getAABB(), ps));

	ps.members.insert({1,1});

	ASSERT_FALSE(contains(es, ps));

	coterie::PointSet<2> ps2(ps);
	ASSERT_TRUE(contains(ps, ps2));
	ASSERT_TRUE(contains(ps2, ps));

	// Test Polytope set
	coterie::PolytopeSet<2> poly(ps);
	ASSERT_TRUE(contains(poly.getAABB(), ps));
	ASSERT_TRUE(contains(poly, ps));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
