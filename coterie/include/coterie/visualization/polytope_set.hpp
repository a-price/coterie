/**
 * \file polytope_set.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2018-04-05
 *
 * \copyright
 *
 * Copyright (c) 2018, Georgia Tech Research Corporation
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

#ifndef VISUALIZE_POLYTOPE_SET_HPP
#define VISUALIZE_POLYTOPE_SET_HPP

#include "coterie/PolytopeSet.hpp"

#include <visualization_msgs/Marker.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <vector>

namespace coterie
{

template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>,
	typename RosterT=std::set<Eigen::Matrix<double, DIM, 1>,
		vector_less_than<DIM>,
		Eigen::aligned_allocator<Eigen::Matrix<double, DIM, 1> > > >
visualization_msgs::Marker visualizePosition(const PolytopeSet<DIM, PointT, RosterT> &ps,
                                             const PointT &scale = PointT::Ones())
{
//	typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
//	typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
//	typedef K::Point_3                                Point_3;
//	typedef K::Segment_3                              Segment_3;
//	typedef K::Triangle_3                             Triangle_3;

//	std::vector<Point_3> points;
//	for (const PointT& pt : ps.supportPoints.members)
//	{
//		points.push_back(pt);
//	}
}

} // namespace coterie

#endif //VISUALIZE_POLYTOPE_SET_HPP
