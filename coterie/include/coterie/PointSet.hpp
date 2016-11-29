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

namespace coterie
{

template<unsigned int DIM>
struct vector_less_than
{
	bool operator()(const Eigen::Matrix<double, DIM, 1>& a,
	                const Eigen::Matrix<double, DIM, 1>& b) const
	{
		for(size_t i=0; i<DIM; ++i)
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
inline bool contains_impl(const C& v, const T& x, long)
//{ return std::end(v) != std::find(std::begin(v), std::end(v), x); }
{ return v.end() != std::find(v.begin(), v.end(), x); }

template<class C, class T>
auto contains(const C& c, const T& x)
-> decltype(std::end(c), true)
{ return contains_impl(c, x, 0); }    // 0 prefers int to long




//         template typename RosterT=std::set<typename> >
template<unsigned int DIM,
         typename PointT=Eigen::Matrix<double, DIM, 1>,
         typename RosterT=std::set<Eigen::Matrix<double, DIM, 1>, vector_less_than<DIM> > >
class PointSet : public Set<DIM, PointT>
{
public:
	RosterT members;
	virtual bool contains(const PointT& q) const override { return ::coterie::contains(members, q); }
	virtual AABB<DIM, PointT> getAABB() const override;
};

template<unsigned int DIM,
         typename PointT,
         typename RosterT >
AABB<DIM, PointT> PointSet<DIM, PointT, RosterT>::getAABB() const
{
	AABB<DIM, PointT> aabb = AABB<DIM, PointT>::InitialBox();

	for (const PointT& p : members)
	{
		for (size_t d=0; d<DIM; ++d)
		{
			aabb.min[d] = std::min(aabb.min[d], p[d]);
			aabb.max[d] = std::max(aabb.max[d], p[d]);
		}
	}

	return aabb;
}



}

#endif // POINTSET_H
