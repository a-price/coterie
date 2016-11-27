/**
 * \file Set.h
 * \brief
 *
 * \author Andrew Price
 * \date 2016-11-21
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

#ifndef SET_H
#define SET_H

#include <Eigen/Core>

namespace coterie
{

template<unsigned int DIM, typename PointT=Eigen::Matrix<double, DIM, 1> >
class AABB;

template<unsigned int DIM, typename PointT=Eigen::Matrix<double, DIM, 1> >
class Set
{
public:
//	typedef AABB<DIM, PointT> AABB;
	typedef PointT point_type;
	virtual bool contains(const PointT& q) = 0;
	virtual AABB<DIM, PointT> getAABB() = 0;
	virtual bool isConvex() { return false; }
};

template<unsigned int DIM, typename PointT>
class AABB : public Set<DIM, PointT>
{
public:
	PointT min;
	PointT max;

	static AABB InitialBox()
	{
		AABB aabb;
		for (size_t d=0; d<DIM; ++d)
		{
			aabb.min[d] = std::numeric_limits<double>::infinity();
			aabb.max[d] = -std::numeric_limits<double>::infinity();
		}
		return aabb;
	}

	virtual bool contains(const PointT &q)
	{
		bool isInside = true;
		for (size_t d=0; d<DIM; ++d)
		{
			isInside = isInside && (q[d] >= min[d]);
			isInside = isInside && (q[d] <= max[d]);
		}
		return isInside;
	}

	virtual AABB<DIM, PointT> getAABB()
	{
		return *this;
	}

	virtual bool isConvex()
	{
		return true;
	}

	bool isWellFormed() const
	{
		bool valid = true;
		for (size_t d = 0; d < DIM; ++d)
		{
			valid = valid && (max[d] >= min[d]);
		}
		return valid;
	}

	double getVolume() const
	{
		double v = 1;
		for (size_t d=0; d<DIM; ++d)
		{
			double len = max[d] - min[d];
			v *= len;
		}
		if (v > 0 && !isWellFormed()) { v = -v; } // Return a negative volume for invalid
		return v;
	}
};


}

#endif // SET_H
