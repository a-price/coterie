/**
 * \file projections.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-11-30
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

#ifndef PROJECTIONS_HPP
#define PROJECTIONS_HPP

#include "coterie/PointSet.hpp"
#include "coterie/PolytopeSet.hpp"

#include <algorithm>

namespace coterie
{
/**
 * @brief Downprojects into a lower dimensional space by simply removing dimensions
 * @param keepDimension Array of dimensions to keep (# of trues must match NEW_DIM)
 * @return
 */
template<int OLD_DIM, int NEW_DIM,
         typename PointT=Eigen::Matrix<double, OLD_DIM, 1>,
         typename RosterT=std::set<Eigen::Matrix<double, OLD_DIM, 1>,
                                   vector_less_than<OLD_DIM>,
                                   Eigen::aligned_allocator<Eigen::Matrix<double, OLD_DIM, 1> > >,
         typename NewPointT=Eigen::Matrix<double, NEW_DIM, 1>,
         typename NewRosterT=std::set<Eigen::Matrix<double, NEW_DIM, 1>,
                                      vector_less_than<NEW_DIM>,
                                      Eigen::aligned_allocator<Eigen::Matrix<double, NEW_DIM, 1> > > >
PointSet<NEW_DIM, NewPointT, NewRosterT>
projection(const PointSet<OLD_DIM, PointT, RosterT>& ps,
           const std::array<bool, OLD_DIM>& keepDimension)
{
	assert(NEW_DIM == std::accumulate(keepDimension.begin(), keepDimension.end(), 0));

	// Copy remaining elements into a new array
	PointSet<NEW_DIM> newSet;
	for (const PointT& pt : ps.members)
	{
		NewPointT v;
		int j = 0;
		for (int k = 0; k < OLD_DIM; ++k)
		{
			if (keepDimension[k])
			{
				v[j++] = pt[k];
			}
		}
		newSet.members.insert(v);
	}

	return newSet;
}

template<int OLD_DIM, int NEW_DIM,
         typename PointT=Eigen::Matrix<double, OLD_DIM, 1>,
         typename RosterT=std::set<Eigen::Matrix<double, OLD_DIM, 1>,
                                   vector_less_than<OLD_DIM>,
                                   Eigen::aligned_allocator<Eigen::Matrix<double, OLD_DIM, 1> > >,
         typename NewPointT=Eigen::Matrix<double, NEW_DIM, 1>,
         typename NewRosterT=std::set<Eigen::Matrix<double, NEW_DIM, 1>,
                                      vector_less_than<NEW_DIM>,
                                      Eigen::aligned_allocator<Eigen::Matrix<double, NEW_DIM, 1> > > >
PolytopeSet<NEW_DIM, NewPointT, NewRosterT>
projection(const PolytopeSet<OLD_DIM, PointT, RosterT>& ps,
           const std::array<bool, OLD_DIM>& keepDimension)
{
	return PolytopeSet<NEW_DIM, NewPointT, NewRosterT>(
	            projection<OLD_DIM, NEW_DIM, PointT, RosterT, NewPointT, NewRosterT>(ps, keepDimension));
}

}

#endif // PROJECTIONS_HPP
