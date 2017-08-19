/**
 * \file PointSet.h
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

#ifndef POINTSET_H
#define POINTSET_H

#include "coterie/Set.hpp"

#include <algorithm>
#include <functional>
#include <iterator>
#include <set>
#include <assert.h>

#include <iostream>

namespace coterie
{

template<int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1>>
struct vector_less_than
{
	bool operator()(const PointT& a,
	                const PointT& b) const
	{
		for(int i=0; i<DIM; ++i)
		{
			if(a[i]<b[i]) return true;
			if(a[i]>b[i]) return false;
		}
		return false;
	}
};

template<typename PointT>
struct vector_less_than<Dynamic, PointT>
{
	bool operator()(const PointT& a,
	                const PointT& b) const
	{
		for(int i=0; i<a.size(); ++i)
		{
			if(a[i]<b[i]) return true;
			if(a[i]>b[i]) return false;
		}
		return false;
	}
};

template<class C, class T>
inline auto contains_impl(const C& c, const T& x, int)
-> decltype(c.find(x), true)
{ return std::end(c) != c.find(x); }

template<class C, class T>
inline bool contains_impl(const C& v, const T& x, ...)
//{ return std::end(v) != std::find(std::begin(v), std::end(v), x); }
{ return v.end() != std::find(v.begin(), v.end(), x); }

template<class C, class T>
auto contains(const C& c, const T& x)
-> decltype(std::end(c), true)
{ return contains_impl(c, x, 0); }    // 0 prefers int to ...


template <class C, class V>
auto append_impl(C& container, V&& value, int)
-> decltype(container.push_back(std::forward<V>(value)), void())
{ container.push_back(std::forward<V>(value));}

template <class C, class V>
void append_impl(C& container, V&& value, ...)
{ container.insert(std::forward<V>(value)); }

template <class C, class V>
void append(C& container, V&& value)
{ append_impl(container, std::forward<V>(value), 0); }


template<int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1>,
         typename RosterT=std::set<Eigen::Matrix<double, DIM, 1>,
                                   vector_less_than<DIM>,
                                   Eigen::aligned_allocator<Eigen::Matrix<double, DIM, 1> > > >
class PointSet : public Set<DIM, PointT>
{
public:
	using point_type = PointT;
	using roster_type = RosterT;
	static constexpr bool is_always_convex = false;

	RosterT members;

	ENABLE_IF_STATIC_DIMENSION
	PointSet() : Set<D, PointT>() {}

	ENABLE_IF_STATIC_DIMENSION
	PointSet(const PointSet<DIM, PointT, RosterT>& other) : Set<D, PointT>(), members(other.members) {}


	ENABLE_IF_DYNAMIC_DIMENSION
	PointSet(const int dim_) : Set<D, PointT>(dim_) {}

	ENABLE_IF_DYNAMIC_DIMENSION
	PointSet(const PointSet<DIM, PointT, RosterT>& other) : Set<D, PointT>(other.dimension), members(other.members) {}

	virtual bool contains(const PointT& q) const override { return ::coterie::contains(members, q); }
	virtual AABB<DIM, PointT> getAABB() const override;
	virtual bool isConvex() const override { return (members.size() <= 1); }

	virtual void insert(const PointT& p) { ::coterie::append(members, p); }
};

// Use to partially specialize class methods without specializing whole class (too much duplication, and virtuals can't be templated)

template<int DIM,
         typename PointT,
         typename RosterT>
class PointSetSpecializations
{
public:
	static
	AABB<DIM, PointT> getAABB(const RosterT& members, const int /*dim*/)
	{
		AABB<DIM, PointT> aabb = AABB<DIM, PointT>::InitialBox();

		for (const PointT& p : members)
		{
			for (int d=0; d < DIM; ++d)
			{
				aabb.min[d] = std::min(aabb.min[d], p[d]);
				aabb.max[d] = std::max(aabb.max[d], p[d]);
			}
		}

		return aabb;
	}
};

template<typename PointT,
         typename RosterT>
class PointSetSpecializations<Dynamic, PointT, RosterT>
{
public:
	static
	AABB<Dynamic, PointT> getAABB(const RosterT& members, const int dim)
	{
		AABB<Dynamic, PointT> aabb = AABB<Dynamic, PointT>::InitialBox(dim);

		for (const PointT& p : members)
		{
			for (int d=0; d < dim; ++d)
			{
				aabb.min[d] = std::min(aabb.min[d], p[d]);
				aabb.max[d] = std::max(aabb.max[d], p[d]);
			}
		}

		return aabb;
	}
};


template<int DIM,
         typename PointT,
         typename RosterT >
AABB<DIM, PointT> PointSet<DIM, PointT, RosterT>::getAABB() const
{
	return PointSetSpecializations<DIM, PointT, RosterT>::getAABB(members, Set<DIM, PointT>::dimension);
}

}

#endif // POINTSET_H
