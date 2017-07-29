/**
 * \file test_obb_set.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2017-7-28
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

#include "coterie/OrientedBoundingBoxSet.hpp"

#include <gtest/gtest.h>

using std::find;
using std::begin;
using std::end;

TEST(OrientedBoundingBoxSet, testPointsSquare)
{
	const int DIM = 2;
	Eigen::Matrix2d axes = -Eigen::Matrix2d::Identity();
	Eigen::Vector2d center = Eigen::Vector2d::UnitY();
	Eigen::Vector2d extents = Eigen::Vector2d(1.0, 0.5);

	coterie::OrientedBoundingBoxSet<DIM> obb(axes, center, extents);
	ASSERT_TRUE(obb.contains(center));
	ASSERT_FALSE(obb.contains(Eigen::Vector2d::Zero()));

	ASSERT_TRUE(obb.contains(Eigen::Vector2d(0.9, 1.0)));
	ASSERT_FALSE(obb.contains(Eigen::Vector2d(1.1, 1.0)));

	ASSERT_TRUE(obb.contains(Eigen::Vector2d(-0.9, 1.0)));
	ASSERT_FALSE(obb.contains(Eigen::Vector2d(-1.1, 1.0)));

	ASSERT_TRUE(fabs(2.0-obb.getVolume()) < 1e-9);

	auto corners = obb.getCorners();
	ASSERT_EQ(corners.size(), 4);
	ASSERT_TRUE(find(begin(corners), end(corners), Eigen::Vector2d(1.0, 1.5)) != end(corners));
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
