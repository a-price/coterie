/**
 * \file test_file_serialization.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2017-3-28
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

#include <coterie/RasterSet.hpp>
#include <coterie/RandomIndexGenerator.hpp>
#include <coterie/serialization/AABB.hpp>
#include <coterie/serialization/Array.hpp>
#include <coterie/serialization/Eigen.hpp>
#include <coterie/serialization/RasterSet.hpp>
#include <coterie/serialization/compressed_file.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <random>

#include <gtest/gtest.h>

namespace fs = boost::filesystem;
namespace io = boost::iostreams;
namespace ar = boost::archive;

TEST(fileio, testString)
{
	fs::path temp_dir = fs::temp_directory_path();
	fs::path temp_file = temp_dir / fs::path("test_set.data");
	std::string dat = "Hello World.";
	coterie::saveGZFile(dat, temp_file.c_str());
	std::string s;
	coterie::loadGZFile(s, temp_file.c_str());
	ASSERT_EQ(dat, s);
}

TEST(fileio, testCompression)
{
	fs::path temp_dir = fs::temp_directory_path();
	fs::path temp_file = temp_dir / fs::path("test_set.data");

	const int N = 2;
	coterie::RasterSet<N>::Shape shape{25,25};
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
	coterie::saveGZFile(rs, temp_file.c_str());
	coterie::RasterSet<N> rs2(shape, bounds);
	coterie::loadGZFile(rs2, temp_file.c_str());

	ASSERT_EQ(rs.num_elements(), rs2.num_elements());

	const unsigned nElements = rs.data.num_elements();
	for (size_t i = 0; i < nElements; ++i)
	{
		coterie::RasterSet<N>::Index idx = rs.getCell(i);
		ASSERT_EQ(rs.data(idx), rs2.data(idx));
	}
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
