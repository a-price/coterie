/**
 * \file PointSet.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2018-12-1
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

#ifndef VISUALIZE_POINTSET_HPP
#define VISUALIZE_POINTSET_HPP

#include "coterie/PointSet.hpp"

#include <visualization_msgs/Marker.h>

#include <Eigen/Geometry>

namespace coterie
{

template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>,
	typename RosterT=std::set<Eigen::Matrix<double, DIM, 1>,
	                          vector_less_than<DIM>,
	                          Eigen::aligned_allocator<Eigen::Matrix<double, DIM, 1> > > >
visualization_msgs::Marker visualizePosition(const PointSet<DIM, PointT, RosterT>& ps,
                                             const PointT& scale = PointT::Ones())
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "World";
	marker.header.stamp = ros::Time();
	marker.ns = "position_belief";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.frame_locked = true;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;
	marker.scale.x = scale[0];
	marker.scale.y = scale[1];
	if (DIM >= 3)
	{
		marker.scale.z = scale[2];
	}
	else
	{
		marker.scale.z = 1;
	}

	for (const PointT& p : ps.members)
	{
		geometry_msgs::Point gp;
		gp.x = p.x();
		gp.y = p.y();
		if (DIM >= 3)
		{
			gp.z = p.z();
		}
		else
		{
			gp.z = 0;
		}
		marker.points.push_back(gp);
	}

	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	return marker;
}

}

#endif // VISUALIZE_POINTSET_HPP
