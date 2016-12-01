/**
 * \file test_projections.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-11-30
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

#include "coterie/projections.hpp"

#include <random>
#include <gtest/gtest.h>

TEST(SubsetProjections, testPointSet)
{
	// Create a 3D Diamond
	coterie::PointSet<3> ps;
	ps.members.insert({0,0,2});
	ps.members.insert({0,0,-2});
	ps.members.insert({0,2,0});
	ps.members.insert({0,-2,0});
	ps.members.insert({2,0,0});
	ps.members.insert({-2,0,0});

	coterie::PointSet<2> proj = coterie::projection<3,2>(ps, {true,true,false});

	ASSERT_TRUE(proj.contains({0,0}));
	ASSERT_TRUE(proj.contains({2,0}));
	ASSERT_TRUE(proj.contains({0,-2}));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
