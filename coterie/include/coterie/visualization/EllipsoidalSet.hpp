/**
 * \file EllipsoidalSet.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-12-1
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

#ifndef VISUALIZE_ELLIPSOIDALSET_HPP
#define VISUALIZE_ELLIPSOIDALSET_HPP

#include "coterie/EllipsoidalSet.hpp"

#include <visualization_msgs/Marker.h>

#include <Eigen/Eigenvalues>

namespace coterie
{

template< typename PointT=Eigen::Matrix<double, DIM, 1>,
          typename MatrixT=Eigen::Matrix<double, DIM, DIM> >
visualization_msgs::Marker visualizePosition(const EllipsoidalSet<3, PointT, MatrixT>& es)
{
	// NB: computeDirect for 2x2 or 3x3 matrices
	Eigen::SelfAdjointEigenSolver<MatrixT> eigSolver;
	eigSolver.computeDirect(es.Q);

	Eigen::Quaterniond q(eigSolver.eigenvectors());

	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "position_belief";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = es.c[0];
	marker.pose.position.y = es.c[1];
	marker.pose.position.z = es.c[2];
	marker.pose.orientation.x = q.x();
	marker.pose.orientation.y = q.y();
	marker.pose.orientation.z = q.z();
	marker.pose.orientation.w = q.w();
	marker.scale.x = eigSolver.eigenvalues()[0];
	marker.scale.y = eigSolver.eigenvalues()[1];
	marker.scale.z = eigSolver.eigenvalues()[2];
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

	return marker;
}

}

#endif // VISUALIZE_ELLIPSOIDALSET_HPP
