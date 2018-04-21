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

template<int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1>,
         typename MatrixT=Eigen::Matrix<double, DIM, DIM> >
class EllipsoidalSet : public Set<DIM, PointT>
{
public:
	typedef PointT point_type;
	typedef MatrixT matrix_type;
	static constexpr bool is_always_convex = true;

	PointT c;
	MatrixT A;
	Eigen::LLT<MatrixT> chol; // Cholesky decomposition

	MatrixT L;
	MatrixT Linv;

	static inline
	EllipsoidalSet<DIM, PointT, MatrixT> ARep(const PointT& center, const MatrixT& A)
	{
		return EllipsoidalSet<DIM, PointT, MatrixT>(center, A);
	}

	static inline
	EllipsoidalSet<DIM, PointT, MatrixT> BRep(const PointT& center, const MatrixT& B)
	{
		return EllipsoidalSet<DIM, PointT, MatrixT>(center, (B*B.transpose()).inverse());
	}

	virtual bool contains(const PointT& q) const override
	{
		PointT dist = q-c;
		return (dist.transpose() * A * dist) <= 1.0;
//		return (dist.transpose() * L).norm() <= 1.0;
	}

	virtual AABB<DIM, PointT> getAABB() const override
	{
		// NB: Could specialize since nHat is always a basis vector
		AABB<DIM, PointT> aabb = initializeAABB();
		for (int d=0; d < Set<DIM, PointT>::dimension; ++d)
		{
			PointT nHat = zeroVector();
			nHat[d] = 1;
			std::pair<double, double> extents = projectToCenteredRay(nHat);
			aabb.min[d] = extents.first;
			aabb.max[d] = extents.second;
		}
		return aabb;
	}

	virtual bool isConvex() const override { return true; }

	ENABLE_IF_STATIC_DIMENSION
	inline AABB<DIM, PointT> initializeAABB() const
	{
		return AABB<DIM, PointT>::InitialBox();
	}

	ENABLE_IF_DYNAMIC_DIMENSION
	inline AABB<DIM, PointT> initializeAABB() const
	{
		return AABB<DIM, PointT>::InitialBox(Set<DIM, PointT>::dimension);
	}

	ENABLE_IF_STATIC_DIMENSION
	inline PointT zeroVector() const
	{
		return PointT::Zero();
	}

	ENABLE_IF_DYNAMIC_DIMENSION
	inline PointT zeroVector() const
	{
		return PointT::Zero(Set<DIM, PointT>::dimension);
	}

	MatrixT semiAxes() const;

	inline MatrixT B() const { return Linv.transpose(); }

	// Line segment connects (first*nHat, second*nHat)
	std::pair<double, double> projectToCenteredRay(const Eigen::Matrix<double, DIM, 1>& nHat) const
	{
		assert((nHat.squaredNorm() - 1.0) < 1e-6);
		double s0 = nHat.dot(c);
		PointT w = Linv * nHat;
		double wNorm = w.norm();
		return {s0-wNorm, s0+wNorm};
	}

protected:
	/**
	 * @brief EllipsoidalSet
	 * @param center Center of the ellipsoid
	 * @param covar SPD (not SPSD!) ellipsoid matrix
	 */
	ENABLE_IF_STATIC_DIMENSION
	EllipsoidalSet(const PointT& center, const MatrixT& Amat)
		: Set<DIM, PointT>(),
		  c(center),
		  A(Amat),
		  chol(Amat),
		  L(chol.matrixL()),
		  Linv(L.inverse())
	{

	}

	ENABLE_IF_DYNAMIC_DIMENSION
	EllipsoidalSet(const PointT& center, const MatrixT& Amat)
		: Set<DIM, PointT>(center.size()),
		  c(center),
		  A(Amat),
		  chol(Amat),
		  L(chol.matrixL()),
		  Linv(L.inverse())
	{

	}

public:

//	ENABLE_IF_STATIC_DIMENSION
//	EllipsoidalSet(const EllipsoidalSet<DIM, PointT, MatrixT>& other)
//		: Set<DIM, PointT>(),
//		  c(other.c),
//		  A(other.A),
//		  chol(other.chol),
//		  L(other.L),
//		  Linv(other.Linv)
//	{}
//
//	ENABLE_IF_DYNAMIC_DIMENSION
//	EllipsoidalSet(const EllipsoidalSet<DIM, PointT, MatrixT>& other)
//		: Set<DIM, PointT>(other.c.size()),
//		  c(other.c),
//		  A(other.A),
//		  chol(other.chol),
//		  L(other.L),
//		  Linv(other.Linv)
//	{}
//
//	EllipsoidalSet<DIM, PointT, MatrixT>& operator=(EllipsoidalSet<DIM, PointT, MatrixT> rhs)
//	{
//		swap(*this, rhs);
//		return *this;
//	}
//	friend void swap(EllipsoidalSet<DIM, PointT, MatrixT>& lhs, EllipsoidalSet<DIM, PointT, MatrixT>& rhs)
//	{
//		using std::swap;
//		swap(lhs.c, rhs.c);
//		swap(lhs.A, rhs.A);
//		swap(lhs.chol, rhs.chol);
//		swap(lhs.L, rhs.L);
//		swap(lhs.Linv, rhs.Linv);
//	}
};

}

#endif // ELLIPSOIDALSET_HPP
