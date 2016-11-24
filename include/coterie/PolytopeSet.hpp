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


template<unsigned int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1> >
class Hyperplane
{
public:
	PointT normal;
	double distance;

//	Vector center; = normal * distance
};

template<unsigned int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1> >
class PolytopeSet : public Set<DIM, PointT>
{
public:
	PointSet<DIM, PointT> supportPoints;
	std::vector<Hyperplane<DIM>, Eigen::aligned_allocator<Hyperplane<DIM> > > supportPlanes;


	PolytopeSet(const PointSet<DIM, PointT>& inputSet)
	{
		qhull(inputSet);
	}

	virtual bool contains(const PointT& q) override
	{
		bool inSpace = true;
		for (const Hyperplane<DIM>& h : supportPlanes)
		{
			inSpace = inSpace && h.distance >= q.dot(h.normal);
		}
		return inSpace;
	}
	virtual AABB<DIM, PointT> getAABB() override { return supportPoints.getAABB(); }
	virtual bool isConvex() override { return true; }
protected:
	bool qhull(const PointSet<DIM, PointT>& inputSet, bool joggle = false);
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

template<unsigned int DIM, typename PointT>
bool PolytopeSet<DIM, PointT>::qhull(const PointSet<DIM, PointT>& inputSet, bool joggle)
{
	// True if qhull should free points in qh_freeqhull() or reallocation
	boolT ismalloc = True;
	// output from qh_produce_output(), use NULL to skip qh_produce_output()
	FILE *outfile = NULL;//stderr;

	// option flags for qhull, see qh_opt.htm
	char* flags;
	if (joggle)
	{
		flags = const_cast<char*>(std::string("qhull QJ n").c_str());
	}
	else
	{
		flags = const_cast<char*>(std::string("qhull Qx n").c_str());// qhull_flags.c_str ();
	}

	// error messages from qhull code
	FILE *errfile = stderr;

	// Array of coordinates for each point
	coordT *points = reinterpret_cast<coordT*> (calloc (inputSet.members.size() * DIM, sizeof (coordT)));

	// Copy data over to input array
	{
		size_t i = 0;
		for (const PointT& supportPoint : inputSet.members)
		{
			for (size_t j = 0; j < DIM; ++j)
			{
				points[i * DIM + j] = static_cast<coordT> (supportPoint[j]);
			}
			++i;
		}
	}

	// Compute convex hull
	if(qh_qh) { qh_save_qhull(); }
	int exitcode = qh_new_qhull (DIM, static_cast<int>(inputSet.members.size()), points, ismalloc, flags, outfile, errfile);

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
		assert(facet->normal);
		Hyperplane<DIM> h;
		h.distance = -facet->offset;
		for (int i = 0; i < DIM; ++i)
		{
			h.normal[i] = -facet->normal[i];
		}

		supportPlanes.push_back(h);
	}

	// Recover the surviving vertices
	vertexT* vertex;
	for (vertex=qh_qh->vertex_list; vertex && vertex->next; vertex=vertex->next)
	{
		PointT newSupport;
		for (size_t j = 0; j < DIM; ++j)
		{
			newSupport[j] = vertex->point[j];
		}
		supportPoints.members.insert(newSupport);
	}

	// Deallocates memory (also the points)
	qh_freeqhull (!qh_ALL);
	int curlong, totlong;
	qh_memfreeshort (&curlong, &totlong);

	return true;
}

}

#endif // POLYTOPESET_HPP
