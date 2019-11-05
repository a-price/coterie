/*
 * Copyright (c) 2019 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Derived from OMPL's RNG:
 * https://github.com/ompl/ompl/blob/master/src/ompl/util/RandomNumbers.h
 * By Ioan Sucan, Jonathan Gammell
 */

#ifndef COTERIE_RNG_H
#define COTERIE_RNG_H

#include <memory>
#include <random>
#include <cassert>
#include <cstdint>

namespace coterie
{

class RNG
{
public:
	/** \brief Constructor. Generates random seed with random_device */
	RNG();

	/** \brief Constructor. Set to the specified instance seed. */
	explicit
	RNG(std::uint_fast32_t localSeed);

	/** \brief Generate a random real between 0 and 1 */
	inline
	double uniform01()
	{
		return uniDist(generator);
	}

	/** \brief Generate a random real within given bounds: [\e lower_bound, \e upper_bound) */
	inline
	double uniformReal(double lower_bound, double upper_bound)
	{
		assert(lower_bound <= upper_bound);
		return (upper_bound - lower_bound) * uniDist(generator) + lower_bound;
	}

	/** \brief Generate a random integer within given bounds: [\e lower_bound, \e upper_bound] */
	inline
	int uniformInt(int lower_bound, int upper_bound)
	{
		auto r = (int)floor(uniformReal((double)lower_bound, (double)(upper_bound) + 1.0));
		return (r > upper_bound) ? upper_bound : r;
	}

	/** \brief Generate a random boolean */
	inline
	bool uniformBool()
	{
		return uniDist(generator) <= 0.5;
	}

	/** \brief Generate a random real using a normal distribution with mean 0 and variance 1 */
	inline
	double gaussian01()
	{
		return normalDist(generator);
	}

	/** \brief Generate a random real using a normal distribution with given mean and variance */
	inline
	double gaussian(double mean, double stddev)
	{
		return normalDist(generator) * stddev + mean;
	}

	std::uint_fast32_t localSeed;
	std::mt19937 generator;
	std::uniform_real_distribution<> uniDist{0, 1};
	std::normal_distribution<> normalDist{0, 1};
};

}

#endif // COTERIE_RNG_H
