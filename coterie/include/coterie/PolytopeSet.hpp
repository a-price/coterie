/**
 * \file PolytopeSet.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-11-23
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

#ifndef POLYTOPESET_HPP
#define POLYTOPESET_HPP

#include "coterie/PointSet.hpp"

#include <vector>
#include <iostream>

namespace coterie
{


template<int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1> >
class Hyperplane
{
public:
	PointT normal;
	double distance;

//	Vector center; = normal * distance
};

template<int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1>,
         typename RosterT=std::set<Eigen::Matrix<double, DIM, 1>,
                                   vector_less_than<DIM>,
                                   Eigen::aligned_allocator<Eigen::Matrix<double, DIM, 1> > > >
class PolytopeSet : public Set<DIM, PointT>
{
public:
	typedef PointT point_type;
	typedef RosterT roster_type;
	static constexpr bool is_always_convex = true;
	static constexpr bool is_polyhedral = true;

	typedef ::coterie::Hyperplane<DIM, PointT> Hyperplane;
	PointSet<DIM, PointT, RosterT> supportPoints;
	std::vector<Hyperplane, Eigen::aligned_allocator<Hyperplane> > supportPlanes;

	ENABLE_IF_STATIC_DIMENSION
	PolytopeSet(const PointSet<DIM, PointT, RosterT>& inputSet)
	    : Set<D, PointT>(),
	      supportPoints()
	{
		initialize(inputSet);
	}

	ENABLE_IF_DYNAMIC_DIMENSION
	PolytopeSet(const PointSet<DIM, PointT, RosterT>& inputSet)
	    : Set<D, PointT>(inputSet.dimension),
	      supportPoints(inputSet.dimension)
	{
		initialize(inputSet);
	}

	void initialize(const PointSet<DIM, PointT, RosterT>& inputSet)
	{
		bool succeeded = qhull(inputSet);
		if (!succeeded)
		{
			succeeded = qhull(inputSet, true);
		}
		if (!succeeded)
		{
			supportPoints.members.clear();
			supportPlanes.clear();
		}
	}

	ENABLE_IF_STATIC_DIMENSION
	inline PointT createPoint() const
	{
		return PointT();
	}

	ENABLE_IF_DYNAMIC_DIMENSION
	inline PointT createPoint() const
	{
		return PointT(Set<D, PointT>::dimension);
	}

	ENABLE_IF_STATIC_DIMENSION
	inline Hyperplane createHyperplane() const
	{
		return Hyperplane();
	}

	ENABLE_IF_DYNAMIC_DIMENSION
	inline Hyperplane createHyperplane() const
	{
		return Hyperplane{createPoint(), 0.0};
	}

	virtual bool contains(const PointT& q) const override
	{
		const double epsilon = 1e-9;
		for (const Hyperplane& h : supportPlanes)
		{
			if (!(q.dot(h.normal) <= h.distance + epsilon)) { return false; }
		}
		return supportPlanes.size() > 0;
	}
	virtual AABB<DIM, PointT> getAABB() const override { return supportPoints.getAABB(); }
	virtual bool isConvex() const override { return is_always_convex; }

	const RosterT& getCorners() const { return supportPoints.members; }
protected:
	bool qhull(const PointSet<DIM, PointT, RosterT>& inputSet, bool joggle = false);
};

extern "C"
{
#ifdef HAVE_QHULL_2011
#include "libqhull/libqhull.h"
#include "libqhull/mem.h"
#include "libqhull/qset.h"
#include "libqhull/geom.h"
#include "libqhull/merge.h"
#include "libqhull/poly.h"
#include "libqhull/io.h"
#include "libqhull/stat.h"
#else
#include "qhull/qhull.h"
#include "qhull/mem.h"
#include "qhull/qset.h"
#include "qhull/geom.h"
#include "qhull/merge.h"
#include "qhull/poly.h"
#include "qhull/io.h"
#include "qhull/stat.h"
#endif
}

template<int DIM, typename PointT, typename RosterT>
bool PolytopeSet<DIM, PointT, RosterT>::qhull(const PointSet<DIM, PointT, RosterT>& inputSet, bool joggle)
{
	// True if qhull should free points in qh_freeqhull() or reallocation
	boolT ismalloc = True;
	// output from qh_produce_output(), use NULL to skip qh_produce_output()
	FILE *outfile = NULL;//stderr;

	// option flags for qhull, see qh_opt.htm
	std::string qhull_cmd;
	if (joggle)
	{
		qhull_cmd = "qhull QJ n";
	}
	else
	{
		qhull_cmd = "qhull Qx n";
	}

	// error messages from qhull code
	FILE *errfile = stderr;

	// Array of coordinates for each point
	coordT *points = reinterpret_cast<coordT*> (calloc (inputSet.members.size() * Set<DIM, PointT>::dimension, sizeof (coordT)));

	// Copy data over to input array
	{
		int i = 0;
		for (const PointT& supportPoint : inputSet.members)
		{
			for (int j = 0; j < Set<DIM, PointT>::dimension; ++j)
			{
				points[i * Set<DIM, PointT>::dimension + j] = static_cast<coordT> (supportPoint[j]);
			}
			++i;
		}
	}

	// Compute convex hull
	if(qh_qh) { qh_save_qhull(); }
	int exitcode = qh_new_qhull (Set<DIM, PointT>::dimension, static_cast<int>(inputSet.members.size()), points, ismalloc, const_cast<char*>(qhull_cmd.c_str()), outfile, errfile);

	// 0 if no error from qhull
	if (exitcode != 0)
	{
		printf("ERROR: qhull was unable to compute a convex hull for the given point cloud!\n");

		qh_freeqhull (!qh_ALL);
		int curlong, totlong;
		qh_memfreeshort (&curlong, &totlong);

		return false;
	}

	// Recover the normals and centers from facets
	supportPlanes.reserve(qh_qh->num_facets);
	facetT* facet;
	for (facet=qh_qh->facet_list; facet && facet->next; facet=facet->next)
	{
		// NB: Per http://www.qhull.org/html/index.htm#structure,
		// QHull represents halfspaces in the format Vx+b<0, with V=nHat^T.
		// Our halfspace representation is Vx<b, so we need the negative offset
		assert(facet->normal);
		Hyperplane h = createHyperplane();
		h.distance = -facet->offset;
		for (int i = 0; i < Set<DIM, PointT>::dimension; ++i)
		{
			h.normal[i] = facet->normal[i];
		}

		supportPlanes.push_back(h);
	}

	// Recover the surviving vertices
	vertexT* vertex;
	for (vertex=qh_qh->vertex_list; vertex && vertex->next; vertex=vertex->next)
	{
		PointT newSupport = createPoint();
		for (int j = 0; j < Set<DIM, PointT>::dimension; ++j)
		{
			newSupport[j] = vertex->point[j];
		}
		supportPoints.insert(newSupport);
	}

	// Deallocates memory (also the points)
	qh_freeqhull (!qh_ALL);
	int curlong, totlong;
	qh_memfreeshort (&curlong, &totlong);

	return true;
}

extern template class PolytopeSet<1>;
extern template class PolytopeSet<2>;
extern template class PolytopeSet<3>;
extern template class PolytopeSet<4>;
extern template class PolytopeSet<6>;
extern template class PolytopeSet<-1>;

}

#endif // POLYTOPESET_HPP
