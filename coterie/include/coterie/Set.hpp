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
#include <Eigen/StdVector>

#include <type_traits>

namespace coterie
{

template<unsigned int DIM, typename PointT=Eigen::Matrix<double, DIM, 1> >
class AABB;

template<unsigned int DIM, typename PointT=Eigen::Matrix<double, DIM, 1> >
class Set
{
public:
	typedef PointT point_type;
	static constexpr bool is_always_convex = false;
	static constexpr unsigned int dimension = DIM;

	typedef ::coterie::AABB<DIM, PointT> AABB;

	virtual bool contains(const PointT& q) const = 0;
	virtual AABB getAABB() const = 0;
	virtual bool isConvex() const { return is_always_convex; }
};

// TODO: Add generative set

template<unsigned int DIM, typename PointT>
class AABB : public Set<DIM, PointT>
{
public:
	typedef PointT point_type;
	static constexpr bool is_always_convex = true;
	static constexpr bool is_polyhedral = true;
	static constexpr unsigned int dimension = DIM;

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

	virtual bool contains(const PointT &q) const
	{
		bool isInside = true;
		for (size_t d=0; d<DIM; ++d)
		{
			isInside = isInside && (q[d] >= min[d]);
			isInside = isInside && (q[d] <= max[d]);
		}
		return isInside;
	}

	virtual AABB<DIM, PointT> getAABB() const
	{
		return *this;
	}

	virtual bool isConvex()
	{
		return is_always_convex;
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
		for (size_t d = 0; d < DIM; ++d)
		{
			double len = max[d] - min[d];
			v *= len;
		}
		if (v > 0 && !isWellFormed()) { v = -v; } // Return a negative volume for invalid
		return v;
	}

	std::vector<PointT> getCorners() const
	{
		// There will be 2^DIM corners to deal with
		const int nCorners = (1<<DIM);
		std::vector<PointT> corners(nCorners);
		for (size_t perm = 0; perm < nCorners; ++perm)
		{
			PointT pt;
			for (size_t d = 0; d < DIM; ++d)
			{
				pt[d] = (perm & (1<<d)) ? min[d] : max[d];
			}
			corners[perm] = pt;
		}
		return corners;
	}

	/**
	 * @brief addPoint
	 * @param q
	 * @return true if point grew the bounding box
	 */
	bool addPoint(const PointT &q)
	{
		bool isInside = true;
		for (size_t d=0; d<DIM; ++d)
		{
			if (q[d] < min[d])
			{
				isInside = false;
				min[d] = q[d];
			}
			if (q[d] > max[d])
			{
				isInside = false;
				max[d] = q[d];
			}
		}
		return false;
	}

	PointT getCenter() const
	{
		PointT c;
		for (size_t d = 0; d < DIM; ++d)
		{
			c[d] = (max[d] + min[d])/2.0;
		}
		return c;
	}

	PointT getDimensions() const
	{
		PointT c;
		for (size_t d = 0; d < DIM; ++d)
		{
			c[d] = (max[d] - min[d]);
		}
		return c;
	}
};

template<unsigned int DIM, typename PointT=Eigen::Matrix<double, DIM, 1> >
static AABB<DIM, PointT> operator+(const AABB<DIM, PointT>& aabb, const PointT& pt)
{
	AABB<DIM, PointT> newBB(aabb);
	newBB.min += pt;
	newBB.max += pt;
	return newBB;
}


}

#endif // SET_H
