/**
 * \file visualize_ellipsoidal_set.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2018-04-02
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

#include <ros/ros.h>
#include "coterie/EllipsoidalSet.hpp"
#include "coterie/visualization/ellipsoidal_set.hpp"
#include "coterie/construction.hpp"
#include <coterie_msgs/EllipsoidalSet.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>

std::string frame = "World";
std::string ns = "position_belief";

ros::Publisher markerPub;
ros::Subscriber r2Sub;
ros::Subscriber r3Sub;
ros::Subscriber se2Sub;
ros::Subscriber se3Sub;

bool checkSizes(const coterie_msgs::EllipsoidalSetConstPtr ps, const int dim)
{
	if (dim != (int)ps->c.data.size())
	{
		ROS_ERROR_STREAM("Requires " << dim << " dimensions: <x,y>. Message contains "
			                 << ps->c.data.size() << ".");
		return false;
	}

	if (dim*dim != (int)ps->A.size())
	{
		ROS_ERROR_STREAM("Requires " << dim << "x" << dim << " dimensions: <x,y>. Message 'A' matrix contains "
			                 << ps->A.size() << ".");
		return false;
	}

	return true;
}

void r2Callback(const coterie_msgs::EllipsoidalSetConstPtr ps)
{
	if (!checkSizes(ps, 2)) { return; }

	Eigen::Vector2d c = Eigen::Map<const Eigen::Vector2d>(ps->c.data.data());
	Eigen::Matrix2d A = Eigen::Map<const Eigen::Matrix2d>(ps->A.data());

	coterie::EllipsoidalSet<2> ell(c, A);

	visualization_msgs::Marker m = coterie::visualizePosition<2>(ell);

	m.ns = ns;
	m.header.frame_id = frame;
	m.header.stamp = ros::Time::now();
	markerPub.publish(m);
}

void r3Callback(const coterie_msgs::EllipsoidalSetConstPtr ps)
{
	if (!checkSizes(ps, 3)) { return; }

	Eigen::Vector3d c = Eigen::Map<const Eigen::Vector3d>(ps->c.data.data());
	Eigen::Matrix3d A = Eigen::Map<const Eigen::Matrix3d>(ps->A.data());

	coterie::EllipsoidalSet<3> ell(c, A);

	visualization_msgs::Marker m = coterie::visualizePosition<3>(ell);

	m.ns = ns;
	float r = rand() / (float)RAND_MAX;
	float g = rand() / (float)RAND_MAX;
	float b = rand() / (float)RAND_MAX;
	float norm = sqrt(r*r+g*g+b*b);
	m.color.r = r/norm;
	m.color.g = g/norm;
	m.color.b = b/norm;
	m.color.a = 0.5;
	m.header.frame_id = frame;
	m.header.stamp = ros::Time::now();
	markerPub.publish(m);
}

void se2Callback(const coterie_msgs::EllipsoidalSetConstPtr ps)
{
	if (!checkSizes(ps, 3)) { return; }

	Eigen::Vector3d c = Eigen::Map<const Eigen::Vector3d>(ps->c.data.data());
	Eigen::Matrix3d A = Eigen::Map<const Eigen::Matrix3d>(ps->A.data());

	coterie::EllipsoidalSet<3> ell(c, A);
	// TODO: PoseWithCovariance
}

void se3Callback(const coterie_msgs::EllipsoidalSetConstPtr ps)
{
	if (!checkSizes(ps, 6)) { return; }

	Eigen::Matrix<double, 6, 1> c = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(ps->c.data.data());
	Eigen::Matrix<double, 6, 6> A = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(ps->A.data());

	coterie::EllipsoidalSet<6> ell(c, A);
	// TODO: PoseWithCovariance
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "visualize_ellipsoid");
	ros::NodeHandle nh, pnh("~");
	srand (std::hash<std::string>{}(ros::this_node::getName()));

//	if (!nh.getParam("target_model", model))
//	{
//		ROS_FATAL("No target model provided, exiting.");
//		return -1;
//	}

	frame = pnh.param("frame", frame);
	ns = pnh.param("m_ns", ns);


	bool selfTest = pnh.param("self_test", false);

	markerPub = nh.advertise<visualization_msgs::Marker>("set_marker", 1, true);
	r2Sub = nh.subscribe<coterie_msgs::EllipsoidalSet>("r2", 1, &r2Callback);
	r3Sub = nh.subscribe<coterie_msgs::EllipsoidalSet>("r3", 1, &r3Callback);
	se2Sub = nh.subscribe<coterie_msgs::EllipsoidalSet>("se2", 1, &se2Callback);
	se3Sub = nh.subscribe<coterie_msgs::EllipsoidalSet>("se3", 1, &se3Callback);

	if (selfTest)
	{
		using Kernel = CGAL::Cartesian_d<double>;
		using ET = CGAL::MP_Float;
		using Traits = CGAL::Approximate_min_ellipsoid_d_traits_d<Kernel,ET>;
		using Point = Traits::Point;

		const int      n = 1000;                // number of points
		const int      d = 3;                   // dimension
		const double side = 0.1;

		ros::Publisher pointPub = nh.advertise<visualization_msgs::Marker>("test_points", 1, true);

		coterie::PointSet<3> ps;

		visualization_msgs::Marker marker;

		CGAL::Random_points_in_cube_d<Point> rpg(d,side);
		for (int i = 0; i < n; ++i)
		{
			Point p = *rpg;
			Eigen::Vector3d q(0.1, (i%2)*0.2, 0.0);
			for (int j = 0; j < d; ++j)
			{
				q[j] += p.cartesian(j);
			}
			ps.insert(q);
			++rpg;

			geometry_msgs::Point pt;
			pt.x = q[0]; pt.y = q[1]; pt.z = q[2];
			marker.points.push_back(pt);
		}

		coterie::EllipsoidalSet<3> ell = coterie::minVolumeEnclosingEllipsoid<3>(ps);

		marker.header.frame_id = frame;
		marker.header.stamp = ros::Time::now();
		marker.ns = "position_inputs";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;
		marker.pose.orientation.z = 0;
		marker.pose.orientation.w = 1;
		marker.scale.x = 0.002;
		marker.scale.y = 0.002;
		marker.scale.z = 1;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		marker.frame_locked = true;

		ros::Duration(3.0).sleep();

		pointPub.publish(marker);
		std::cerr << "Published " << marker.points.size() << " points." << std::endl;

		coterie_msgs::EllipsoidalSetPtr ptr = coterie_msgs::EllipsoidalSetPtr(new coterie_msgs::EllipsoidalSet());
		for (int i = 0; i < 3; ++i)
		{
			ptr->c.data.push_back(ell.c[i]);
			for (int j = 0; j < 3; ++j)
			{
				ptr->A.push_back(ell.Q(i, j));
			}
		}

		r3Callback(ptr);
	}
	else
	{
		std::cerr << "Fake news. Sad!" << std::endl;
	}

	ros::spin();
	return 0;
}