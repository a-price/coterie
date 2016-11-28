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
class RasterSetBase : public Set<DIM, PointT>
{
public:
	typedef ::coterie::Axes<DIM> Axes;
	typedef ::coterie::Bounds<DIM> Bounds;
	typedef ::coterie::Index<DIM> Index;
	typedef ::coterie::Shape<DIM> Shape;

	Axes axes;
	Shape shape;
	Bounds bounds;

	RasterSetBase(const Shape& _shape, const Bounds& _bounds)
	    : Set<DIM, PointT>(),
	      shape(_shape),
	      bounds(_bounds)
	{
		for (size_t d = 0; d < DIM; ++d)
		{
			axes[d].setLinSpaced(shape[d], bounds[d].first, bounds[d].second);
		}
	}

	PointT getState(const Index& idx) const;
	Index getCell(const PointT& q) const;
	Index getCell(size_t index) const;
};


template<unsigned int DIM, typename PointT, typename StorageT>
AABB<DIM, PointT> getActiveAABB(const RasterSetBase<DIM, PointT>& rsb, const StorageT& data)
{
	typedef typename RasterSetBase<DIM, PointT>::Index Index;
	Index minIdx; std::fill(minIdx.begin(), minIdx.end(), std::numeric_limits<typename Index::value_type>::max());
	Index maxIdx; std::fill(maxIdx.begin(), maxIdx.end(), std::numeric_limits<typename Index::value_type>::min());

	// TODO: More efficient version of this
	bool isEmpty = true;
	const unsigned nElements = data.num_elements();
	for (size_t i = 0; i < nElements; ++i)
	{
		Index idx = rsb.getCell(i);
		if (data(idx))
		{
			isEmpty = false;
			for (size_t d=0; d<DIM; ++d)
			{
				minIdx[d] = std::min(minIdx[d], idx[d]);
				maxIdx[d] = std::max(maxIdx[d], idx[d]);
			}
		}
	}

	AABB<DIM, PointT> aabb;
	if (isEmpty)
	{
		throw std::runtime_error("Operation not implemented.");
//		aabb.min = PointT::Ones() * std::numeric_limits<typename PointT::Scalar>::max();
//		aabb.max = PointT::Ones() * std::numeric_limits<typename PointT::Scalar>::min();
	}
	else
	{
		aabb.min = rsb.getState(minIdx);
		aabb.max = rsb.getState(maxIdx);
	}
	return aabb;
}

// Forward declare set view
template<unsigned int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1>,
         bool isConstView=true >
class RasterSetView;


template<unsigned int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1> >
class RasterSet : public RasterSetBase<DIM, PointT>
{
public:
	typedef RasterSetBase<DIM, PointT> Base;
	typedef typename Base::Axes Axes;
	typedef typename Base::Bounds Bounds;
	typedef typename Base::Index Index;
	typedef typename Base::Shape Shape;
	typedef ::coterie::StateSet<DIM> StateSet;

	StateSet data;

	RasterSet(const Shape& _shape, const Bounds& _bounds);

	virtual bool contains(const PointT& q) const override;
	virtual AABB<DIM, PointT> getAABB() const override;

//	RasterSetView<DIM, PointT, false> getView(const AABB<DIM, PointT>& aabb);
	RasterSetView<DIM, PointT,  true> getView(const AABB<DIM, PointT>& aabb) const;
};

// Helper functor to build indices.
template<typename RangeArrayType, size_t Dimension>
struct IndicesBuilder {
   // Recursively invoke the functor for the next lowest dimension and
   // add the next range.
   static auto build(const RangeArrayType& range)
      -> decltype(IndicesBuilder<RangeArrayType, Dimension - 1>::build(range)[range[Dimension - 1]]) {
      return IndicesBuilder<RangeArrayType, Dimension - 1>::build(range)[range[Dimension - 1]];
   }
};

// Helper functor specialization to terminate recursion.
template<typename RangeArrayType>
struct IndicesBuilder<RangeArrayType, 1> {
   static auto build(const RangeArrayType& range)
      -> decltype(boost::indices[range[0]]) {
      return boost::indices[range[0]];
   }
};

template<unsigned int DIM, typename PointT, bool isConstView>
class RasterSetView : public RasterSetBase<DIM, PointT>
{
public:
	typedef RasterSetBase<DIM, PointT> Base;
	typedef typename Base::Axes Axes;
	typedef typename Base::Bounds Bounds;
	typedef typename Base::Index Index;
	typedef typename Base::Shape Shape;
	typedef boost::multi_array_types::index_range Range; // NB: Range(low, high) is [low, high)
	typedef std::array<Range, DIM> Ranges;
	typedef boost::multi_array<bool, DIM> Array;
	typedef typename std::conditional<isConstView,
	                         typename Array::template const_array_view<DIM>::type,
	                         typename Array::template array_view<DIM>::type>::type View;

	View dataView;

	RasterSetView(const RasterSet<DIM, PointT>& rasterSet, const Ranges& ranges)
	    : RasterSetBase<DIM, PointT>(rangesToShape(ranges), rangesToBounds(ranges, rasterSet.axes)),
	      dataView(rasterSet.data[IndicesBuilder<Ranges, DIM>::build(ranges)])
	{

	}

	virtual bool contains(const PointT& q) const override
	{
		return dataView(this->getCell(q));
	}

	virtual AABB<DIM, PointT> getAABB() const override
	{
		return getActiveAABB(*this, dataView);
	}

	static Shape rangesToShape(const Ranges& ranges)
	{
		Shape shape;
		for (size_t d = 0; d < DIM; ++d)
		{
			const Range& r = ranges[d];
			shape[d] = r.finish() - r.start();
		}
		return shape;
	}

	static Bounds rangesToBounds(const Ranges& ranges, const Axes& axes)
	{
		Bounds newBounds;
		for (size_t d = 0; d < DIM; ++d)
		{
			const Range& r = ranges[d];
			newBounds[d] = {axes[d][r.start()], axes[d][r.finish()-1]};
		}
		return newBounds;
	}
};


template<unsigned int DIM, typename PointT>
RasterSet<DIM, PointT>::RasterSet(const RasterSet::Shape& _shape, const RasterSet::Bounds& _bounds)
    : RasterSetBase<DIM, PointT>(_shape, _bounds),
      data(_shape)
{

}

template<unsigned int DIM, typename PointT>
bool RasterSet<DIM, PointT>::contains(const PointT& q) const
{
	return data(this->getCell(q));
}

template<unsigned int DIM, typename PointT>
AABB<DIM, PointT> RasterSet<DIM, PointT>::getAABB() const
{
	return getActiveAABB(*this, data);
}

template<unsigned int DIM, typename PointT>
PointT RasterSetBase<DIM, PointT>::getState(const Index& idx) const
{
	PointT x;
	for (size_t d = 0; d < DIM; ++d) { x[d] = this->axes[d][idx[d]]; }
	return x;
}

template<unsigned int DIM, typename PointT>
typename RasterSetBase<DIM, PointT>::Index RasterSetBase<DIM, PointT>::getCell(const PointT& point) const
{
	Index idx;
	for (size_t d = 0; d < DIM; ++d)
	{
		assert(point[d] >= this->bounds[d].first);
		assert(point[d] <= this->bounds[d].second);
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
typename RasterSetBase<DIM, PointT>::Index RasterSetBase<DIM, PointT>::getCell(size_t index) const
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

template<unsigned int DIM, typename PointT>
RasterSetView<DIM, PointT, true> RasterSet<DIM, PointT>::getView(const AABB<DIM, PointT>& aabb) const
{
	typename RasterSetView<DIM, PointT>::Ranges ranges;
	if (!aabb.isWellFormed())
	{
		for (size_t d = 0; d < DIM; ++d)
		{
			ranges[d] = typename RasterSetView<DIM, PointT>::Range(0,0);
		}
	}
	else
	{
		Index minIdx = this->getCell(aabb.min);
		Index maxIdx = this->getCell(aabb.max);
		for (size_t d = 0; d < DIM; ++d)
		{
			ranges[d] = typename RasterSetView<DIM, PointT>::Range(minIdx[d], maxIdx[d]+1);
		}
	}
	return RasterSetView<DIM, PointT, true>(*this, ranges);
}

}

#endif // RASTERSET_HPP
