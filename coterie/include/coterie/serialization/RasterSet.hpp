/**
 * \file RasterSet.hpp
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

#ifndef SERIALIZE_RASTERSET_HPP
#define SERIALIZE_RASTERSET_HPP

#include <coterie/RasterSet.hpp>
#include <coterie/serialization/AABB.hpp>
#include <coterie/serialization/Array.hpp>
#include <coterie/serialization/Eigen.hpp>

#include <boost/multi_array.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/utility.hpp>

namespace boost
{
namespace serialization
{

template<class Archive, class T, std::size_t NumDims, typename Allocator>
inline void load(Archive & ar,
                 boost::multi_array<T, NumDims, Allocator> & t,
                 const unsigned int /*version*/)
{
	const unsigned int DIM = 2;
	typedef boost::multi_array<T,DIM> multi_array_;
	typedef typename multi_array_::size_type size_;

	std::array<size_, DIM> shape;
	for (unsigned int d = 0; d < DIM; ++d)
	{
		std::string name = "dim" + std::to_string(d);
		ar >> boost::serialization::make_nvp(name.c_str(), shape[d]);
	}

	t.resize(coterie::IndicesBuilder<std::array<size_, DIM>, boost::multi_array_types::extent_gen, DIM>::build(shape, boost::extents));
	ar >> boost::serialization::make_array(t.data(), t.num_elements());
}

template<typename Archive, typename T, std::size_t NumDims, typename Allocator>
inline void save(Archive & ar,
                 const boost::multi_array<T, NumDims, Allocator> & t,
                 const unsigned int /*version*/)
{
	const unsigned int DIM = 2;
	typedef boost::multi_array<T,DIM> multi_array_;
	typedef typename multi_array_::size_type size_;

	for (unsigned int d = 0; d < DIM; ++d)
	{
		std::string name = "dim" + std::to_string(d);
		size_ n = (t.shape()[d]);
		ar << boost::serialization::make_nvp(name.c_str(), n);
	}
	ar << boost::serialization::make_array(t.data(), t.num_elements());
}

template<class Archive, typename T, std::size_t NumDims, typename Allocator>
inline void serialize(Archive & ar,
                      boost::multi_array<T, NumDims, Allocator>& t,
                      const unsigned int version)
{
	split_free(ar, t, version);
}

template<typename Archive, unsigned int DIM, typename PointT>
void serialize(Archive & ar, coterie::RasterSet<DIM, PointT> & rs, const unsigned int /*version*/)
{
    ar & rs.coverage;
    ar & rs.axes;
    ar & rs.shape;
	ar & rs.bounds;
	ar & rs.data;
}

} // namespace serialization
} // namespace boost
#endif // SERIALIZE_RASTERSET_HPP
