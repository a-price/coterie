/**
 * \file visualize_polytope_set.cpp
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
#include "coterie/PolytopeSet.hpp"
#include "coterie/visualization/polytope_set.hpp"
#include "coterie/construction.hpp"
#include <coterie_msgs/PolytopeSet.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Geometry>


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <vector>


typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
typedef K::Point_3                                Point_3;
typedef K::Segment_3                              Segment_3;
typedef K::Triangle_3                             Triangle_3;
typedef CGAL::Surface_mesh<Point_3> Surface_mesh;

namespace CGAL
{
::Polyhedron_3::Facet_const_iterator begin(const ::Polyhedron_3 &poly)
{ return poly.facets_begin(); }

::Polyhedron_3::Facet_const_iterator end(const ::Polyhedron_3 &poly)
{ return poly.facets_end(); }
}

std::string frame = "World";
std::string ns = "position_belief";

ros::Publisher markerPub;
ros::Subscriber r2Sub;
ros::Subscriber r3Sub;
ros::Subscriber se2Sub;
ros::Subscriber se3Sub;

bool checkSizes(const coterie_msgs::PolytopeSetConstPtr ps, const int dim)
{
	int size = 0;
	if (!ps->support_planes.empty())
	{
		size = (int)ps->support_planes.begin()->normal.data.size();
	}
	else if (!ps->support_points.empty())
	{
		size = (int)ps->support_points.begin()->data.size();
	}

	if (dim != size)
	{
		ROS_ERROR_STREAM("Requires " << dim << " dimensions: <x,y,...>. Message contains " << size << ".");
		return false;
	}

	return true;
}

void r2Callback(const coterie_msgs::PolytopeSetConstPtr ps)
{
	if (!checkSizes(ps, 2)) { return; }
}

void r3Callback(const coterie_msgs::PolytopeSetConstPtr ps)
{
	if (!checkSizes(ps, 3)) { return; }

//	coterie::PointSet<3, coterie_msgs::Point, std::vector<coterie_msgs::Point>> pointSet;
//	coterie::PolytopeSet<3> poly();
	std::vector<Point_3> points;
	for (const coterie_msgs::Point& pt : ps->support_points)
	{
		points.emplace_back(Point_3(pt.data[0], pt.data[1], pt.data[2]));
	}

	CGAL::Object obj;
	visualization_msgs::MarkerArray ma;
	visualization_msgs::Marker m;

	CGAL::convex_hull_3(points.begin(), points.end(), obj);

	if(const Point_3* p = CGAL::object_cast<Point_3>(&obj))
	{
		std::cout << "Point " << *p << std::endl;
	}
	else if(const Segment_3* s = CGAL::object_cast<Segment_3>(&obj))
	{
		std::cout << "Segment " << *s << std::endl;
	}
	else if(const Triangle_3* t = CGAL::object_cast<Triangle_3>(&obj))
	{
		std::cout << "Triangle " << *t << std::endl;
	}
	else  if(const Polyhedron_3* poly = CGAL::object_cast<Polyhedron_3>(&obj))
	{
		std::cout << "The convex hull contains " << poly->size_of_vertices() << " vertices" << std::endl;

		for (auto facet : *poly)
		{
			Polyhedron_3::Halfedge_handle h = facet.halfedge();
			int vertex_count = 0;
			do
			{
				Point_3 v = h->vertex()->point();
				geometry_msgs::Point pt;
				pt.x = v.x(); pt.y = v.y(); pt.z = v.z();
				m.points.push_back(pt);
				h = h->next();
				++vertex_count;
			} while (h != facet.halfedge());

			std_msgs::ColorRGBA rgb;
			rgb.a = 0.5;
			float r = rand() / (float)RAND_MAX;
			float g = rand() / (float)RAND_MAX;
			float b = rand() / (float)RAND_MAX;
			float norm = sqrt(r*r+g*g+b*b);
			rgb.r = r/norm;
			rgb.g = g/norm;
			rgb.b = b/norm;
//			rgb.r = 0.0; //rand() / (float)RAND_MAX;
//			float g = rand() / (float)RAND_MAX;
//			float b = rand() / (float)RAND_MAX;
//			rgb.g = g/(g*g+b*b);
//			rgb.b = b/(g*g+b*b);
			m.colors.push_back(rgb);

			if (3 != vertex_count)
				std::cout << "Facet contains " << vertex_count << " vertices." << std::endl;
		}

	}
	else
	{
		std::cout << "something else"<< std::endl;
	}

	m.ns = ns;
	m.id = 0;
	m.type = visualization_msgs::Marker::TRIANGLE_LIST;
	m.action = visualization_msgs::Marker::ADD;
	m.frame_locked = true;
	m.pose.position.x = 0;
	m.pose.position.y = 0;
	m.pose.position.z = 0;
	m.pose.orientation.x = 0;
	m.pose.orientation.y = 0;
	m.pose.orientation.z = 0;
	m.pose.orientation.w = 1;
	m.scale.x = 1;
	m.scale.y = 1;
	m.scale.z = 1;
	m.color.a = 0.5; // Don't forget to set the alpha!
	m.color.r = 0.0;
	m.color.g = 0.0;
	m.color.b = 1.0;

	m.header.frame_id = frame;
	m.header.stamp = ros::Time::now();
	markerPub.publish(m);
}

void se2Callback(const coterie_msgs::PolytopeSetConstPtr ps)
{
	if (!checkSizes(ps, 3)) { return; }
}

void se3Callback(const coterie_msgs::PolytopeSetConstPtr ps)
{
	if (!checkSizes(ps, 6)) { return; }
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "visualize_polytope");
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
	r2Sub = nh.subscribe<coterie_msgs::PolytopeSet>("r2", 1, &r2Callback);
	r3Sub = nh.subscribe<coterie_msgs::PolytopeSet>("r3", 1, &r3Callback);
	se2Sub = nh.subscribe<coterie_msgs::PolytopeSet>("se2", 1, &se2Callback);
	se3Sub = nh.subscribe<coterie_msgs::PolytopeSet>("se3", 1, &se3Callback);

	if (selfTest)
	{
		using Kernel = CGAL::Cartesian_d<double>;
		using ET = CGAL::MP_Float;
		using Traits = CGAL::Approximate_min_ellipsoid_d_traits_d<Kernel,ET>;
		using Point = Traits::Point;

		const int      n = 10;                // number of points
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
//
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
		ros::spinOnce();

		pointPub.publish(marker);
		std::cerr << "Published " << marker.points.size() << " points." << std::endl;

		coterie_msgs::PolytopeSetPtr ptr = coterie_msgs::PolytopeSetPtr(new coterie_msgs::PolytopeSet());
		for (int i = 0; i < marker.points.size(); ++i)
		{
			coterie_msgs::Point cp;
			cp.data.push_back(marker.points[i].x);
			cp.data.push_back(marker.points[i].y);
			cp.data.push_back(marker.points[i].z);

			ptr->support_points.push_back(cp);
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