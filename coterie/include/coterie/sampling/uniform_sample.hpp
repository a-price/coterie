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

#ifndef COTERIE_UNIFORM_SAMPLE_HPP
#define COTERIE_UNIFORM_SAMPLE_HPP

#include "coterie/Set.hpp"
#include "coterie/PointSet.hpp"
#include "coterie/construction.hpp"
#include "coterie/sampling/RNG.h"

#include <cfloat> // DBL_MAX
#include <cmath>  // std::nextafter

namespace coterie
{

template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>,
	typename RosterT=std::vector<Eigen::Matrix<double, DIM, 1>,
	                             Eigen::aligned_allocator<Eigen::Matrix<double, DIM, 1> > > >
bool sampleUniform(const Set<DIM, PointT>& set, const int numSamples, RosterT& samples, RNG& rng, const int attemptRatio=10)
{
	const auto aabb = set.getAABB();

	for (int i = 0; i < numSamples * attemptRatio; ++i)
	{
		PointT x = zeroVector(set);

		for (int d = 0; d < aabb.dimension; ++d)
		{
			x[d] = rng.uniformReal(aabb.min[d], std::nextafter(aabb.max[d], DBL_MAX));
		}

		if (set.contains(x))
		{
			append(samples, x);
		}

		if (samples.size() == numSamples)
		{
			return true;
		}
	}

	return false;
}

template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>,
	typename RosterT=std::set<Eigen::Matrix<double, DIM, 1>, vector_less_than<DIM> >,
	typename RosterT2=std::vector<Eigen::Matrix<double, DIM, 1>,
	                             Eigen::aligned_allocator<Eigen::Matrix<double, DIM, 1> > > >
bool sampleUniform(const PointSet<DIM, PointT, RosterT>& set, const int numSamplesRequested, RosterT2& samples, RNG& rng, const int attemptRatio=10)
{
	std::vector<unsigned int> indices(set.members.size());
	std::iota(indices.begin(), indices.end(), 0);
	std::shuffle(indices.begin(), indices.end(), rng.generator);

	const int numSamples = std::min(static_cast<int>(set.members.size()), numSamplesRequested);
	for (int i = 0; i < numSamples; ++i)
	{
		auto iter = set.members.begin();
		std::advance(iter, indices[i]);
		const PointT& x = *iter;
		append(samples, x);
	}

	return set.members.size() >= numSamplesRequested;
}

}

#endif //COTERIE_UNIFORM_SAMPLE_HPP
