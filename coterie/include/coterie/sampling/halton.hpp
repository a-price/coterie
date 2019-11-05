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

#ifndef COTERIE_HALTON_HPP
#define COTERIE_HALTON_HPP

#include <vector>
#include <array>
#include <cassert>
#include <iostream>
#include <random>

namespace coterie
{

namespace halton
{

const std::vector<int> primes{2, 3, 5, 7, 11, 13, 17, 19, 23, 29,
                              31, 37, 41, 43, 47, 53, 59, 61};

const std::vector<int> hardcoded_offsets{100, 120, 234, 182, 102,
                                         192, 476, 203, 120, 203, 203, 403, 203, 203, 120, 1045, 302, 102};

}

inline double haltonElement(int index, int base)
{
	double f = 1, r = 0;
	while (index > 0)
	{
		f = f / base;
		r = r + f * (index % base);
		index = index / base;
	}
	return r;
};

inline std::vector<double> haltonSeq(int base, int length, int offset)
{
	std::vector<double> seq;
	seq.reserve(length);
	for (int ind = offset; ind < length + offset; ind++)
	{
		seq.push_back(haltonElement(ind, base));
	}
	return seq;
};

inline std::vector<std::vector<double>> haltonPoints(std::vector<int> bases,
                                                     int length,
                                                     std::vector<int> offsets)
{
	assert(bases.size() == offsets.size());
	std::vector<std::vector<double>> seqs(bases.size());
	for (int i = 0; i < bases.size(); i++)
	{
		seqs[i] = haltonSeq(bases[i], length, offsets[i]);
	}

	std::vector<std::vector<double>> points(length, std::vector<double>(bases.size()));
	for (int i = 0; i < length; i++)
	{
		for (int j = 0; j < bases.size(); j++)
		{
			points[i][j] = seqs[j][i];
		}
	}
	return points;
};

// Primary
inline std::vector<std::vector<double>> haltonPoints(int length, int dim)
{
	std::vector<int> bases(dim), offsets(dim);
	for (int i = 0; i < dim; i++)
	{
		bases[i] = halton::primes[i];
		offsets[i] = halton::hardcoded_offsets[i];
	}
	return haltonPoints(bases, length, offsets);
};



template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>,
	typename RosterT=std::vector<Eigen::Matrix<double, DIM, 1>,
	Eigen::aligned_allocator<Eigen::Matrix<double, DIM, 1> > > >
bool sampleHalton(const Set<DIM, PointT>& set, const int numSamples, RosterT& samples, RNG& rng, const int attemptRatio=10)
{
	const auto aabb = set.getAABB();

	auto c = aabb.getCenter();
	auto D = aabb.getDimensions();
	double scale = 0;
	for (int d = 0; d < set.dimension; ++d)
	{
		scale = std::max(D[d], scale);
	}

	std::vector<std::vector<double>> haltonPts = haltonPoints(numSamples * attemptRatio, set.dimension);

	for (int i = 0; i < numSamples * attemptRatio; ++i)
	{
		PointT x = zeroVector(set);

		for (int d = 0; d < set.dimension; ++d)
		{
			x[d] = c[d] + (2.0 * (haltonPts[i][d] - 0.5) * scale);
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

//inline std::vector<std::vector<double> > haltonPoints(int length, int dim, int seed)
//{
//	std::mt19937 eng(seed);
//	std::uniform_int_distribution<> dist(0, 10000);
//
//	std::vector<int> bases(dim), offsets(dim);
//	for(int i=0; i<dim; i++)
//	{
//		bases[i] = halton::primes[i];
//		offsets[i] = dist(eng);
//	}
//	return haltonPoints(bases, length, offsets);
//};

}

#endif
