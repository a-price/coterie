/**
 * \file subsets.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-11-27
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

#ifndef SUBSETS_HPP
#define SUBSETS_HPP

#include "EllipsoidalSet.hpp"
#include "PointSet.hpp"
#include "PolytopeSet.hpp"
#include "RasterSet.hpp"
#include "NonparametricSets.hpp"

namespace coterie
{

template<int DIM,
         typename PointT>
bool contains(const UniversalSet<DIM, PointT>& /*outer*/, const Set<DIM, PointT>& /*inner*/)
{
	return true;
}

template<int DIM,
         typename PointT>
bool contains(const EmptySet<DIM, PointT>& /*outer*/, const Set<DIM, PointT>& /*inner*/)
{
	return false;
}

template<int DIM,
         typename PointT>
bool contains(const Set<DIM, PointT>& /*outer*/, const EmptySet<DIM, PointT>& /*inner*/)
{
	return true;
}

template<int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1>,
         typename RosterT=std::set<Eigen::Matrix<double, DIM, 1>, vector_less_than<DIM> > >
bool contains(const Set<DIM, PointT>& outer, const PointSet<DIM, PointT, RosterT>& inner)
{
	for (const PointT& pt : inner.members)
	{
		if (!outer.contains(pt)) { return false; }
	}
	return true;
}

template<int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1> >
bool contains(const AABB<DIM, PointT>& outer, const AABB<DIM, PointT>& inner)
{
	bool isSubset = true;
	for (size_t d = 0; d < DIM; ++d)
	{
		isSubset = isSubset && (outer.min[d] <= inner.min[d]) && (inner.max[d] <= outer.max[d]);
	}
	return isSubset;
}


template<int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1>,
         typename MatrixT=Eigen::Matrix<double, DIM, DIM> >
bool contains(const AABB<DIM, PointT>& outer, const EllipsoidalSet<DIM, PointT, MatrixT>& inner)
{
	return contains(outer, inner.getAABB());
}

template<int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1>,
         typename MatrixT=Eigen::Matrix<double, DIM, DIM>,
         typename RosterT=std::set<Eigen::Matrix<double, DIM, 1>, vector_less_than<DIM> > >
bool contains(const PolytopeSet<DIM, PointT, RosterT>& outer, const EllipsoidalSet<DIM, PointT, MatrixT>& inner)
{
	for (const typename PolytopeSet<DIM, PointT, RosterT>::Hyperplane& h : outer.supportPlanes)
	{
		if (!(inner.c.dot(h.normal) + (inner.Linv * h.normal).norm() < h.distance)) { return false; }
	}
	return true;
}

template<int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1>,
         typename MatrixT=Eigen::Matrix<double, DIM, DIM> >
bool contains(const RasterSet<DIM, PointT>& outer, const EllipsoidalSet<DIM, PointT, MatrixT>& inner)
{
	// Get neighborhood of ellipsoid
	RasterSetView<DIM, PointT> view = outer.getView(inner.getAABB());

	// Check that all cells inside ellipsoid are true
	const unsigned nElements = view.dataView.num_elements();
	for (size_t i = 0; i < nElements; ++i)
	{
		typename RasterSetView<DIM, PointT>::Index idx = view.getCell(i);
		const PointT pt = view.getState(idx);
		if (inner.contains(pt) && !outer.contains(pt))
		{
			return false;
		}
	}
	return true;
}

template<class OuterT,
         class InnerT>
typename std::enable_if<OuterT::is_always_convex && InnerT::is_always_convex && InnerT::is_polyhedral &&
                        std::is_convertible<typename InnerT::point_type, typename OuterT::point_type>::value &&
                        OuterT::dimension == InnerT::dimension,
bool>::type contains(const OuterT& outer, const InnerT& inner)
{
	for (const typename InnerT::point_type& pt : inner.getCorners())
	{
		if (!outer.contains(pt)) { return false; }
	}
	return true;
}

}

#endif // SUBSETS_HPP
