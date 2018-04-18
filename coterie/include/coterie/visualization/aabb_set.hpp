/**
 * \file aabb_set.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2018-04-17
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

#ifndef VISUALIZE_AABB_SET_HPP
#define VISUALIZE_AABB_SET_HPP

#include "coterie/Set.hpp"

#include <visualization_msgs/Marker.h>

namespace coterie
{

template<int DIM, typename PointT=Eigen::Matrix<double, DIM, 1> >
visualization_msgs::Marker visualizeExtents(const AABB<DIM, PointT> &aabb,
                                            const PointT &scale = PointT::Ones())
{
	PointT boxCenter = aabb.getCenter();
	PointT boxExtents = aabb.getDimensions();

	visualization_msgs::Marker box;
	box.header.frame_id = "World";
	box.header.stamp = ros::Time();
	box.ns = "position_belief";
	box.id = 0;
	box.type = visualization_msgs::Marker::CUBE;
	box.action = visualization_msgs::Marker::ADD;

	box.frame_locked = true;
	box.pose.position.x = boxCenter[0] * scale[0];
	box.pose.position.y = ((DIM >= 2) ? boxCenter[1] * scale[1] : 0);
	box.pose.position.z = ((DIM >= 3) ? boxCenter[2] * scale[2] : 0);
	box.pose.orientation.w = 1.0;

	box.scale.x = boxExtents[0] * scale[0];
	box.scale.y = ((DIM >= 2) ? boxExtents[1] * scale[1] : 0);
	box.scale.z = ((DIM >= 3) ? boxExtents[2] * scale[2] : 0);

	// Default to LightSkyBlue rgb(135,206,250)
	box.color.a = 0.5; // Don't forget to set the alpha!
	box.color.r = 135.0 / 255.0;
	box.color.g = 206.0 / 255.0;
	box.color.b = 250.0 / 255.0;

	return box;
}

} // namespace coterie

#endif //VISUALIZE_AABB_SET_HPP
