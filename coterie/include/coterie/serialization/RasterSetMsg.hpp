/**
 * \file serialization.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-11-29
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

#ifndef SERIALIZE_RASTERSETMSG_HPP
#define SERIALIZE_RASTERSETMSG_HPP

#include <coterie/RasterSet.hpp>
#include <coterie_msgs/RasterSet.h>

namespace coterie
{

template<int DIM>
void serialize(const RasterSetBase<DIM>& set, coterie_msgs::RasterSet& msg, const std::vector<std::string> labels)
{
	size_t nElements = 1;
	for (int s : set.shape) { nElements *= s; }
	assert(set.num_elements() == nElements);

	size_t mul = nElements;
	msg.extents.resize(DIM);
	msg.byteArray.layout.dim.resize(DIM);
	for (size_t d = 0; d < DIM; ++d)
	{
		msg.extents[d].min = set.bounds[d].first;
		msg.extents[d].max = set.bounds[d].second;

		msg.byteArray.layout.dim[d].label = (labels.empty() ? std::to_string(d) : labels[d]);
		msg.byteArray.layout.dim[d].size = set.shape[d];
		msg.byteArray.layout.dim[d].stride = mul;
		mul /= set.shape[d];
	}

	msg.byteArray.layout.data_offset = 0;
	msg.byteArray.data.resize(nElements);

	for (size_t i = 0; i < nElements; ++i)
	{
		typename RasterSet<DIM>::Index idx = set.getCell(i);
		msg.byteArray.data[i] = set(idx);
	}

}

template<int DIM>
void deserialize(const coterie_msgs::RasterSet& msg, RasterSet<DIM>& set)
{
	// Check dimensionality
	if (DIM != msg.extents.size())
	{
		throw std::runtime_error("Dimensionality mismatch when deserializing message.");
	}

	typename RasterSet<DIM>::Shape shape;
	typename RasterSet<DIM>::Bounds bounds;

	for (size_t d = 0; d < DIM; ++d)
	{
		// Populate shape
		shape[d] = msg.byteArray.layout.dim[d].size;

		// Populate bounds
		bounds[d].first = msg.extents[d].min;
		bounds[d].second = msg.extents[d].max;
	}

	// Reshape set
	set.data.resize(shape); // Required for assignment to work
	set = RasterSet<DIM>(shape, bounds);

	// Populate data
	size_t nElements = 1;
	for (int s : set.shape) { nElements *= s; }

	for (size_t i = 0; i < nElements; ++i)
	{
		typename RasterSet<DIM>::Index idx = set.getCell(i);
		set.data(idx) = msg.byteArray.data[i + msg.byteArray.layout.data_offset];
	}
}
}

#endif // SERIALIZE_RASTERSETMSG_HPP
