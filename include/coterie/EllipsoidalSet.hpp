/**
 * \file EllipsoidalSet.hpp
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

#ifndef ELLIPSOIDALSET_HPP
#define ELLIPSOIDALSET_HPP

#include "coterie/Set.hpp"

#include <Eigen/Cholesky>
#include <Eigen/LU>

namespace coterie
{

template<unsigned int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1>,
         typename MatrixT=Eigen::Matrix<double, DIM, DIM> >
class EllipsoidalSet : public Set<DIM, PointT>
{
public:
	PointT c;
	MatrixT Q;
	Eigen::LLT<MatrixT> L; // Cholesky decomposition
	MatrixT Linv;

	/**
	 * @brief EllipsoidalSet
	 * @param center Center of the ellipsoid
	 * @param covar SPD (not SPSD!) covariance matrix
	 */
	EllipsoidalSet(const PointT& center, const MatrixT& covar)
	    : Set<DIM, PointT>(),
	      c(center),
	      Q(covar),
	      L(covar),
	      Linv(MatrixT(L.matrixL()).inverse())
	{

	}

	virtual bool contains(const PointT& q) override
	{
		PointT dist = q-c;
		return (dist.transpose() * Q * dist) <= 1.0;
	}

	virtual AABB<DIM, PointT> getAABB() override
	{
		// NB: Could specialize since nHat is always a basis vector
		AABB<DIM, PointT> aabb;
		for (size_t d=0; d<DIM; ++d)
		{
			PointT nHat = PointT::Zero();
			nHat[d] = 1;
			std::pair<double, double> extents = projectToCenteredRay(nHat);
			aabb.min[d] = extents.first;
			aabb.max[d] = extents.second;
		}
		return aabb;
	}

	virtual bool isConvex() override { return true; }

	// Line segment connects (first*nHat, second*nHat)
	std::pair<double, double> projectToCenteredRay(const Eigen::Matrix<double, DIM, 1>& nHat)
	{
		assert((nHat.squaredNorm() - 1.0) < 1e-6);
		double s0 = nHat.dot(c);
		PointT w = Linv * nHat;
		double wNorm = w.norm();
		return {s0-wNorm, s0+wNorm};
	}
};

}

#endif // ELLIPSOIDALSET_HPP
