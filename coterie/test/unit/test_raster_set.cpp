/**
 * \file test_raster_set.cpp
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

#include "coterie/RasterSet.hpp"
#include <coterie/RandomIndexGenerator.hpp>

#include <random>
#include <gtest/gtest.h>

TEST(RasterSet, testContains)
{
	const int N = 2;
	coterie::RasterSet<N>::Shape shape{5,5};
	coterie::RasterSet<N>::Bounds bounds{{{-1,1},{-1,1}}};
	coterie::RasterSet<N> rs(shape, bounds);

	coterie::RandomIndexGenerator<N> rig(shape);
	for (int rep = 0; rep < 5; ++rep)
	{
		coterie::RasterSet<N>::Index index = rig.getIndex();
		for (size_t i = 0; i < N; ++i)
		{
			ASSERT_TRUE(index[i] >= 0);
			ASSERT_TRUE(index[i] < static_cast<long int>(shape[i]));
		}
		rs.data(index) = true;
	}

	coterie::AABB<N> aabb = rs.getAABB();
	const unsigned nElements = rs.data.num_elements();
	for (size_t i = 0; i < nElements; ++i)
	{
		coterie::RasterSet<N>::Index idx = rs.getCell(i);
		if (rs.data(idx))
		{
			ASSERT_TRUE(aabb.contains(rs.getState(idx)));
			ASSERT_TRUE(rs.contains(rs.getState(rs.getCell(rs.getState(idx)))));
		}
		ASSERT_EQ(idx, rs.getCell(rs.getState(idx)));
	}
}

TEST(RasterSet, testRangesToShape)
{
	typedef boost::multi_array_types::index_range range;
	const int N = 6;
	coterie::RasterSetView<N>::Ranges ranges = {{range{2,8},range{5,10},range{3,7},range{10,13},range{2,4},range{8,9}}};
	coterie::RasterSetView<N>::Shape testShape = coterie::RasterSetView<N>::rangesToShape(ranges);
	coterie::RasterSetView<N>::Shape shape{6,5,4,3,2,1};
	for (size_t d = 0; d < N; ++d)
	{
		ASSERT_EQ(shape[d], testShape[d]);
	}
}

TEST(RasterSet, testView)
{
	const int N = 2;
	coterie::RasterSet<N>::Shape shape{5,5};
	coterie::RasterSet<N>::Bounds bounds{{{-1,1},{-1,1}}};
	coterie::RasterSet<N> rs(shape, bounds);

	coterie::RandomIndexGenerator<N> rig(shape);
	coterie::RasterSet<N>::Index index = rig.getIndex();
	rs.data(index) = true;

	coterie::RasterSetView<N> view = rs.getView(rs.getAABB());
	ASSERT_EQ(1, view.dataView.num_elements());
	for (size_t d = 0; d < N; ++d)
	{
		ASSERT_NEAR(0.4, view.bounds[d].second - view.bounds[d].first, 1e-9);
		ASSERT_EQ(1, view.axes[d].size());
	}

	rs.data(index) = false;

	rs.data(coterie::RasterSet<N>::Index{0, 0}) = true;
	rs.data(coterie::RasterSet<N>::Index{1, 1}) = true;

	coterie::RasterSetView<N> view2 = rs.getView(rs.getAABB());
	ASSERT_EQ(4, view2.dataView.num_elements());

	rs.data(coterie::RasterSet<N>::Index{4, 4}) = true;
	coterie::RasterSetView<N> view3 = rs.getView(rs.getAABB());
	ASSERT_EQ(shape[0]*shape[1], view3.dataView.num_elements());
	for (size_t d = 0; d < N; ++d)
	{
		ASSERT_NEAR(0.0, (view3.axes[d] - rs.axes[d]).squaredNorm(), 1e-9);
	}

//	rs.data[rs.getCell()]
}

TEST(RasterSet, testIteration)
{
	const int N = 4;
	coterie::RasterSet<N>::Shape shape{5,5,5,5};
	coterie::RasterSet<N>::Bounds bounds{{{-1,1},{-1,1},{-1,1},{-1,1}}};
	coterie::RasterSet<N> rs(shape, bounds);

	const unsigned nElements = rs.data.num_elements();
	for (size_t i = 0; i < nElements; ++i)
	{
		coterie::RasterSet<N>::Index idx = rs.getCell(i);
		if (rs.data(idx))
		{
			FAIL();
		}
	}
}

TEST(RasterSet, testSensitivity)
{
	const int N = 2;
	coterie::RasterSet<N>::Shape shape{5,5};
	coterie::RasterSet<N>::Bounds bounds{{{-1,1},{-1,1}}};
	coterie::RasterSet<N> rs(shape, bounds);

	coterie::Index<N> idx = {2, 2};

	Eigen::Vector2d s = rs.getState(idx);
	ASSERT_EQ(idx, rs.getCell(s));
	ASSERT_EQ(idx, rs.getCell(s + Eigen::Vector2d(0, 1e-6)));
	ASSERT_EQ(idx, rs.getCell(s - Eigen::Vector2d(0, 1e-6)));
	ASSERT_EQ(idx, rs.getCell(s + Eigen::Vector2d(1e-6, 0)));
	ASSERT_EQ(idx, rs.getCell(s - Eigen::Vector2d(1e-6, 0)));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
