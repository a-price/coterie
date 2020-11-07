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
#include "coterie/PointSet.hpp"

#include <visualization_msgs/Marker.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_3.h>
#include <CGAL/Segment_3.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

#include <vector>
#include <random>


typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
namespace CGAL
{

inline
::Polyhedron_3::Facet_const_iterator begin(const ::Polyhedron_3 &poly)
{ return poly.facets_begin(); }

inline
::Polyhedron_3::Facet_const_iterator end(const ::Polyhedron_3 &poly)
{ return poly.facets_end(); }

}

namespace coterie
{

inline
std_msgs::ColorRGBA randomColor()
{
	// Random number generation
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_real_distribution<float> interval(1.0f, std::nextafter(2.0f, std::numeric_limits<float>::max()));

	std_msgs::ColorRGBA rgb;
	rgb.a = 0.5;
	float r = interval(gen);
	float g = interval(gen);
	float b = interval(gen);
	float norm = static_cast<float>(sqrt(r*r+g*g+b*b));
	rgb.r = r/norm;
	rgb.g = g/norm;
	rgb.b = b/norm;

	return rgb;
}

inline
geometry_msgs::Point cvtPoint(const CGAL::Exact_predicates_inexact_constructions_kernel::Point_3& p)
{
	geometry_msgs::Point pt;
	pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
	return pt;
}

template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>,
	typename RosterT=std::set<Eigen::Matrix<double, DIM, 1>,
		vector_less_than<DIM>,
		Eigen::aligned_allocator<Eigen::Matrix<double, DIM, 1> > > >
visualization_msgs::Marker visualizePosition(const PolytopeSet<DIM, PointT, RosterT> &ps,
                                             const PointT &scale = PointT::Ones())
{
	typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
	typedef CGAL::Polyhedron_3<K>                     Polyhedron_3;
	typedef K::Point_3                                Point_3;
	typedef K::Segment_3                              Segment_3;
	typedef K::Triangle_3                             Triangle_3;



	std::vector<Point_3> points;
	for (const auto& pt : ps.supportPoints.members)
	{
		points.emplace_back(Point_3(pt[0], pt[1], pt[2]));
	}

	visualization_msgs::Marker m;
	if (points.empty())
	{
		std::cerr << "No points in polytope." << std::endl;
		return m;
	}

	CGAL::Object obj;
	CGAL::convex_hull_3(points.begin(), points.end(), obj);

	m.ns = "position_belief";
	m.id = 0;
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

	if(const Point_3* p = CGAL::object_cast<Point_3>(&obj))
	{
		std::cout << "Point " << *p << std::endl;

		m.type = visualization_msgs::Marker::POINTS;
		m.points.emplace_back(cvtPoint(*p));
		m.colors.emplace_back(randomColor());
		m.scale.x = 0.001;
		m.scale.y = 0.001;
	}
	else if(const Segment_3* s = CGAL::object_cast<Segment_3>(&obj))
	{
		std::cout << "Segment " << *s << std::endl;

		m.type = visualization_msgs::Marker::LINE_STRIP;
		m.points.emplace_back(cvtPoint(s->start()));
		m.points.emplace_back(cvtPoint(s->end()));
		m.colors.emplace_back(randomColor());
		m.colors.emplace_back(randomColor());
		m.scale.x = 0.001;
	}
	else if(const Triangle_3* t = CGAL::object_cast<Triangle_3>(&obj))
	{
		std::cout << "Triangle " << *t << std::endl;
		m.type = visualization_msgs::Marker::TRIANGLE_LIST;
		std_msgs::ColorRGBA rgb = randomColor();
		for (int i = 0; i < 3; ++i)
		{
			m.points.emplace_back(cvtPoint(t->vertex(i)));
			m.colors.emplace_back(rgb);
		}
	}
	else  if(const Polyhedron_3* poly = CGAL::object_cast<Polyhedron_3>(&obj))
	{
		m.type = visualization_msgs::Marker::TRIANGLE_LIST;
		for (auto facet : *poly)
		{
			Polyhedron_3::Halfedge_handle h = facet.halfedge();
			std_msgs::ColorRGBA rgb = randomColor();
			int vertex_count = 0;
			do
			{
				Point_3 v = h->vertex()->point();
				m.points.emplace_back(cvtPoint(v));
				m.colors.emplace_back(rgb);
				h = h->next();
				++vertex_count;
			} while (h != facet.halfedge());

			if (3 != vertex_count)
				std::cout << "Facet contains " << vertex_count << " vertices." << std::endl;
		}

	}
	else
	{
		std::cout << "something else"<< std::endl;
	}

	return m;
}

} // namespace coterie

#endif //VISUALIZE_POLYTOPE_SET_HPP
