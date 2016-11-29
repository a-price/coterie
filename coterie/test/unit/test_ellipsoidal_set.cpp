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

#include <gtest/gtest.h>

TEST(EllipsoidalSet, testContains)
{
	// Test unit circle
	coterie::EllipsoidalSet<2> es(Eigen::Vector2d::Zero(), Eigen::Matrix2d::Identity());

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
	es = coterie::EllipsoidalSet<2>(Eigen::Vector2d::Zero(), Q);

	ASSERT_TRUE(es.contains({0,0}));
	ASSERT_TRUE(es.contains({0.5,-0.25}));
	ASSERT_FALSE(es.contains({3.14159,2.71828}));
	ASSERT_FALSE(es.contains({1,1}));
	ASSERT_TRUE(es.contains({-1.5,0.25}));

	// Random Ellipsoid
	Eigen::Matrix2d M = Eigen::Matrix2d::Identity() + Eigen::Matrix2d::Random();
	es = coterie::EllipsoidalSet<2>(10.0*Eigen::Vector2d::Random(), 10.0*M*M.transpose());

	ASSERT_TRUE(es.contains(es.c));

	aabb = es.getAABB();

	ASSERT_TRUE(aabb.contains(es.c));

}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
