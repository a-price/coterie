/**
 * \file RandomIndexGenerator.hpp
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

#ifndef RANDOMINDEXGENERATOR_HPP
#define RANDOMINDEXGENERATOR_HPP

#include <coterie/RasterSet.hpp>
#include <random>

namespace coterie
{

template <int N>
class RandomIndexGenerator
{
public:
	RandomIndexGenerator(const coterie::Shape<N>& s)
	{
		rng = std::mt19937(rd());

		for (size_t i = 0; i < N; ++i)
		{
			generators[i] = std::uniform_int_distribution<int>(0, s[i]-1);
		}
	}

	coterie::Index<N> getIndex()
	{
		coterie::Index<N> idx;
		for (size_t i = 0; i < N; ++i)
		{
			idx[i] = generators[i](rng);
		}
		return idx;
	}

	std::random_device rd;
	std::mt19937 rng;
	std::array<std::uniform_int_distribution<int>, N> generators;
};

}

#endif // RANDOMINDEXGENERATOR_HPP
