/**
 * \file RasterSet.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-11-22
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

#ifndef RASTERSET_HPP
#define RASTERSET_HPP

#include "coterie/Set.hpp"
#include <boost/multi_array.hpp>

namespace coterie
{

template<unsigned int DIM>
using StateSet = boost::multi_array<bool, DIM>;

template<unsigned int DIM>
using Axes = std::array<Eigen::VectorXd, DIM>;

template<unsigned int DIM>
using Bounds = std::array<std::pair<double, double>, DIM>;

template<unsigned int DIM>
using Index = boost::array<long int, DIM>;

template<unsigned int DIM>
using Shape = boost::array<size_t, DIM>;


template<unsigned int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1> >
class RasterSetView : public Set<DIM, PointT>
{
public:
	typedef ::coterie::Axes<DIM> Axes;
	typedef ::coterie::Bounds<DIM> Bounds;
	typedef ::coterie::Index<DIM> Index;
	typedef ::coterie::Shape<DIM> Shape;

	Axes axes;
	Shape shape;
	Bounds bounds;

	RasterSetView(const Shape& _shape, const Bounds& _bounds)
	    : Set<DIM, PointT>(),
	      shape(_shape),
	      bounds(_bounds)
	{
		for (size_t d = 0; d < DIM; ++d)
		{
			axes[d].setLinSpaced(shape[d], bounds[d].first, bounds[d].second);
		}
	}
};


template<unsigned int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1> >
class RasterSet : public RasterSetView<DIM, PointT>
{
public:
	typedef ::coterie::Axes<DIM> Axes;
	typedef ::coterie::Bounds<DIM> Bounds;
	typedef ::coterie::Index<DIM> Index;
	typedef ::coterie::Shape<DIM> Shape;
	typedef ::coterie::StateSet<DIM> StateSet;

	StateSet data;

	RasterSet(const Shape& _shape, const Bounds& _bounds);

	virtual bool contains(const PointT& q) override;
	virtual AABB<DIM, PointT> getAABB() override;

	PointT getState(const Index& idx) const;
	Index getCell(const PointT& q) const;
	Index getCell(size_t index) const;
};

template<unsigned int DIM, typename PointT>
RasterSet<DIM, PointT>::RasterSet(const RasterSet::Shape& _shape, const RasterSet::Bounds& _bounds)
    : RasterSetView<DIM, PointT>(_shape, _bounds),
      data(_shape)
{

}

template<unsigned int DIM, typename PointT>
bool RasterSet<DIM, PointT>::contains(const PointT& q)
{
	return data(getCell(q));
}

template<unsigned int DIM, typename PointT>
AABB<DIM, PointT> RasterSet<DIM, PointT>::getAABB()
{
	// TODO: More efficient version of this
	Index minIdx{std::numeric_limits<long int>::max()};
	Index maxIdx{std::numeric_limits<long int>::min()};
	const unsigned nElements = data.num_elements();
	for (size_t i = 0; i < nElements; ++i)
	{
		Index idx = getCell(i);
		if (data(idx))
		{
			for (size_t d=0; d<DIM; ++d)
			{
				minIdx[d] = std::min(minIdx[d], idx[d]);
				maxIdx[d] = std::max(maxIdx[d], idx[d]);
			}
		}
	}
	AABB<DIM, PointT> aabb;
	aabb.min = getState(minIdx);
	aabb.max = getState(maxIdx);
	return aabb;
}

template<unsigned int DIM, typename PointT>
PointT RasterSet<DIM, PointT>::getState(const Index& idx) const
{
	PointT x;
	for (size_t d = 0; d < DIM; ++d) { x[d] = this->axes[d][idx[d]]; }
	return x;
}

template<unsigned int DIM, typename PointT>
Index<DIM> RasterSet<DIM, PointT>::getCell(const PointT& point) const
{
	Index idx;
	for (size_t d = 0; d < DIM; ++d)
	{
		assert(point[d] >= this->bounds[d].first);
		assert(point[d] < this->bounds[d].second);
		idx[d] = static_cast<int>(static_cast<double>(this->shape[d]-1)
		                          * (point[d]-this->bounds[d].first)/(this->bounds[d].second-this->bounds[d].first));
	}
	return idx;
}

// (shape)[index]
// (N)[i] -> i
// (R,C)[i,j] -> C*i + j
// (A,B,C)[i,j,k] -> (B*C)*i + C*j + k
// (M,N,O,P)[i,j,k,l] -> (N*O*P)*i + (O*P)*j + P*k + l
template<unsigned int DIM, typename PointT>
Index<DIM> RasterSet<DIM, PointT>::getCell(size_t index) const
{
	size_t nElements = 1;
	for (size_t i = 0; i < DIM; ++i)
	{
		size_t s = this->shape[i];
		nElements *= s;
	}
	assert(index < nElements);

	Index res;
	std::vector<size_t> coeffs;

	size_t mul = nElements;
	for (size_t i = 0; i < DIM; ++i)
	{
		mul /= this->shape[i];
		coeffs.push_back(mul);
		res[i] = index / mul;
		assert(res[i] < this->shape[i]);
		index -= res[i] * mul;
	}
	return res;
}

}

#endif // RASTERSET_HPP
