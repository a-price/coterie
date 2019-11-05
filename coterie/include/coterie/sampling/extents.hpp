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

#ifndef SRC_EXTENTS_HPP
#define SRC_EXTENTS_HPP

#include "coterie/Set.hpp"
#include "coterie/PointSet.hpp"
#include "coterie/PolytopeSet.hpp"
#include "coterie/EllipsoidalSet.hpp"

namespace coterie
{

template<int DIM, typename PointT, typename RosterT>
void getExtents(const AABB<DIM, PointT>& aabb, RosterT& extents)
{
	for (const auto& x : aabb.getCorners())
	{
		append(extents, x);
	}
}

template<int DIM, typename PointT, typename RosterT, typename RosterT2>
void getExtents(const PointSet<DIM, PointT, RosterT>& points, RosterT2& extents)
{
	for (const auto& x : points.members)
	{
		append(extents, x);
	}
}

template<int DIM, typename PointT, typename RosterT, typename RosterT2>
void getExtents(const PolytopeSet<DIM, PointT, RosterT>& polytope, RosterT2& extents)
{
	for (const auto& x : polytope.supportPoints.members)
	{
		append(extents, x);
	}
}

template<int DIM, typename PointT, typename MatrixT, typename RosterT2>
void getExtents(const EllipsoidalSet<DIM, PointT, MatrixT>& ellipsoid, RosterT2& extents)
{
	auto sA = ellipsoid.semiAxes();
	for (int i = 0; i < ellipsoid.dimension; ++i)
	{
		append(extents, ellipsoid.c + sA.col(i));
		append(extents, ellipsoid.c - sA.col(i));
	}
}

}

#endif //SRC_EXTENTS_HPP
