/**
 * \file OrientedBoundingBox.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2017-7-28
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

#ifndef ORIENTEDBOUNDINGBOX_HPP
#define ORIENTEDBOUNDINGBOX_HPP

#include "coterie/PointSet.hpp"
#include "coterie/PolytopeSet.hpp"

#include <vector>

namespace coterie
{

template<int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1>,
         typename OrientationT=Eigen::Matrix<double, DIM, DIM>,
         typename RosterT=std::vector<Eigen::Matrix<double, DIM, 1>,
                                      Eigen::aligned_allocator<Eigen::Matrix<double, DIM, 1> > > >
class OrientedBoundingBoxSet : public Set<DIM, PointT>
{
public:
	typedef PointT point_type;
	typedef OrientationT orientation_type;
	typedef RosterT roster_type;
	static constexpr bool is_always_convex = true;
	static constexpr bool is_polyhedral = true;

	/**
	 * @brief axes Orthonormal basis for the n-dimensional box
	 */
	OrientationT axes;

	/**
	 * @brief center Center point of the box
	 */
	PointT center;

	/**
	 * @brief extents Half-width of box
	 */
	PointT extents;

	ENABLE_IF_STATIC_DIMENSION
	OrientedBoundingBoxSet() : Set<D, PointT>() {}

	ENABLE_IF_STATIC_DIMENSION
	OrientedBoundingBoxSet(const OrientationT& axes_, const PointT& center_, const PointT& extents_)
	    : Set<D, PointT>(),
	      axes(axes_),
	      center(center_),
	      extents(extents_) {}

	ENABLE_IF_STATIC_DIMENSION
	OrientedBoundingBoxSet(const OrientedBoundingBoxSet<DIM, PointT, OrientationT, RosterT>& other)
	    : Set<D, PointT>(),
	      axes(other.axes),
	      center(other.center),
	      extents(other.extents) {}


	ENABLE_IF_DYNAMIC_DIMENSION
	OrientedBoundingBoxSet(const int dim_) : Set<D, PointT>(dim_) {}

	ENABLE_IF_DYNAMIC_DIMENSION
	OrientedBoundingBoxSet(const OrientationT& axes_, const PointT& center_, const PointT& extents_)
	    : Set<D, PointT>(extents_.size()),
	      axes(axes_),
	      center(center_),
	      extents(extents_) {}

	ENABLE_IF_DYNAMIC_DIMENSION
	OrientedBoundingBoxSet(const OrientedBoundingBoxSet<DIM, PointT, OrientationT, RosterT>& other)
	    : Set<D, PointT>(other.dimension),
	      axes(other.axes),
	      center(other.center),
	      extents(other.extents) {}

	virtual bool contains(const PointT& q) const override
	{
		const PointT dist = q - center;
		double proj = 0;
		for (int i = 0; i < DIM; ++i)
		{
			proj = axes.col(i).dot(dist);
			if (fabs(proj) > extents[i])
			{
				return false;
			}
		}
		return true;
	}

	virtual AABB<DIM, PointT> getAABB() const override
	{
		return PointSetSpecializations<DIM, PointT, RosterT>::getAABB(this->getCorners(), Set<DIM, PointT>::dimension);
	}

	virtual bool isConvex() const override { return true; }

	double getVolume() const
	{
		return (2.0*extents).prod();
	}

	RosterT getCorners() const
	{
		// There will be 2^DIM corners to deal with
		const int nCorners = (1 << Set<DIM, PointT>::dimension);
		RosterT corners(nCorners);
		for (int perm = 0; perm < nCorners; ++perm)
		{
			PointT pt = center;
			for (int d = 0; d < Set<DIM, PointT>::dimension; ++d)
			{
				pt += extents[d] * axes.col(d) * ((perm & (1<<d)) ? -1.0 : 1.0);
			}
			corners[perm] = pt;
		}
		return corners;
	}

	ENABLE_IF_STATIC_DIMENSION
	PolytopeSet<DIM, PointT, RosterT> getAsPolytope()
	{
		PointSet<DIM, PointT, RosterT> ps;
		ps.members = this->getCorners();
		return PolytopeSet<DIM, PointT, RosterT>(ps);
	}

	ENABLE_IF_DYNAMIC_DIMENSION
	PolytopeSet<DIM, PointT, RosterT> getAsPolytope()
	{
		PointSet<DIM, PointT, RosterT> ps(Set<D, PointT>::dimension);
		ps.members = this->getCorners();
		return PolytopeSet<DIM, PointT, RosterT>(ps);
	}
};

} // namespace coterie

#endif // ORIENTEDBOUNDINGBOX_HPP
