/**
 * \file construction.h
 * \brief
 *
 * \author Andrew Price
 * \date 2018-04-03
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

#ifndef PROJECT_CONSTRUCTION_H
#define PROJECT_CONSTRUCTION_H

#include "coterie/PointSet.hpp"
#include "coterie/PolytopeSet.hpp"
#include "coterie/EllipsoidalSet.hpp"

#include <CGAL/Cartesian_d.h>
#include <CGAL/MP_Float.h>
#include <CGAL/point_generators_d.h>
#include <CGAL/Approximate_min_ellipsoid_d.h>
#include <CGAL/Approximate_min_ellipsoid_d_traits_d.h>
#include <Eigen/StdVector>
#include <vector>
#include <iostream>

namespace coterie
{

template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>>
PointT constructPoint(const int)
{ return PointT(); }

template<>
Eigen::Matrix<double, -1, 1> constructPoint<-1, Eigen::Matrix<double, -1, 1>>(const int d)
{ return Eigen::Matrix<double, -1, 1>(d); }

template<int DIM,
	typename MatrixT=Eigen::Matrix<double, DIM, DIM>>
MatrixT constructMatrix(const int)
{ return MatrixT(); }

template<>
Eigen::Matrix<double, -1, -1> constructMatrix<-1, Eigen::Matrix<double, -1, -1>>(const int d)
{ return Eigen::Matrix<double, -1, -1>(d, d); }


template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>,
	typename MatrixT=Eigen::Matrix<double, DIM, DIM>,
	typename RosterT=std::set<Eigen::Matrix<double, DIM, 1>,
		vector_less_than<DIM>,
		Eigen::aligned_allocator<Eigen::Matrix<double, DIM, 1> > > >
EllipsoidalSet<DIM, PointT, MatrixT> minVolumeEnclosingEllipsoid(const PointSet<DIM, PointT, RosterT>& ps)
{
	using Kernel = CGAL::Cartesian_d<double>;
	using ET = CGAL::MP_Float;
	using Traits = CGAL::Approximate_min_ellipsoid_d_traits_d<Kernel,ET>;
	using Point = Traits::Point;
	using Point_list = std::vector<Point>;
	using AME = CGAL::Approximate_min_ellipsoid_d<Traits>;

	const double eps = 0.01;

	// TODO: There's got to be a way to do this zero-copy...
	using RT = Kernel::RT;
	std::vector<RT> coords(ps.dimension);

	Point_list P;
	for (const PointT& m : ps.members)
	{
		for(int i=0; i < ps.dimension; ++i)
		{
			coords[i] = RT(m[i]);
		}

		Point p(ps.dimension, coords.begin(), coords.end() );
		P.push_back(p);
	}

	Traits traits;
	AME ame(eps, P.begin(), P.end(), traits);

	PointT c = constructPoint<DIM, PointT>(ps.dimension);

	// output center coordinates:
	const AME::Center_coordinate_iterator c_begin = ame.center_cartesian_begin();
	for (AME::Center_coordinate_iterator c_it = c_begin;
	     c_it != ame.center_cartesian_end();
	     ++c_it)
	{
		c[c_it-c_begin] = *c_it;
	}

	// https://en.wikipedia.org/wiki/Ellipsoid#As_quadric
	PointT eigvals = constructPoint<DIM, PointT>(ps.dimension);
	MatrixT eigvecs = constructMatrix<DIM, MatrixT>(ps.dimension);

	if (ps.dimension == 2 || ps.dimension == 3)
	{
		// output  axes:
		AME::Axes_lengths_iterator axes = ame.axes_lengths_begin();
		for (int i = 0; i < ps.dimension; ++i)
		{
			double len = *axes++;
			double eigval = 1.0/(len*len);
			eigvals[i] = eigval;

			AME::Axes_direction_coordinate_iterator d_begin = ame.axis_direction_cartesian_begin(i);
			for (AME::Axes_direction_coordinate_iterator d_it = d_begin;
			     d_it != ame.axis_direction_cartesian_end(i); ++d_it)
			{
				eigvecs.col(i)[d_it-d_begin] = *d_it;
			}
		}
	}

	MatrixT A = eigvecs * eigvals.asDiagonal() * eigvecs.inverse();
	coterie::EllipsoidalSet<DIM, PointT, MatrixT> ell(c, A);

	return ell;
}
} // namespace coterie



#endif //PROJECT_CONSTRUCTION_H
