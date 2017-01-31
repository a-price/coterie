/**
 * \file CompositeSets.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2017-1-30
 *
 * \copyright
 *
 * Copyright (c) 2017, Georgia Tech Research Corporation
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

#ifndef COMPOSITESETS_HPP
#define COMPOSITESETS_HPP

#include "Set.hpp"
#include <memory>

namespace coterie
{

template<unsigned int DIM, typename PointT=Eigen::Matrix<double, DIM, 1> >
class UnionSet : public Set<DIM, PointT>
{
public:
	typedef PointT point_type;
	static constexpr bool is_always_convex = false;
	static constexpr unsigned int dimension = DIM;

	typedef ::coterie::AABB<DIM, PointT> AABB;

	virtual bool contains(const PointT&) const override;
	virtual AABB getAABB() const override;
	virtual bool isConvex() const override;

	std::vector<std::shared_ptr<Set>> members;
};

template<unsigned int DIM, typename PointT=Eigen::Matrix<double, DIM, 1> >
class IntersectionSet : public Set<DIM, PointT>
{
public:
	typedef PointT point_type;
	static constexpr bool is_always_convex = false;
	static constexpr unsigned int dimension = DIM;

	typedef ::coterie::AABB<DIM, PointT> AABB;

	virtual bool contains(const PointT&) const override;
	virtual AABB getAABB() const override;
	virtual bool isConvex() const override;

	std::vector<std::shared_ptr<Set>> members;
};

template<unsigned int DIM, typename PointT=Eigen::Matrix<double, DIM, 1> >
class CrossProductSet : public Set<DIM, PointT>
{
public:
	typedef PointT point_type;
	static constexpr bool is_always_convex = false;
	static constexpr unsigned int dimension = DIM;

	typedef ::coterie::AABB<DIM, PointT> AABB;

	virtual bool contains(const PointT&) const override = 0;
	virtual AABB getAABB() const override = 0;
	virtual bool isConvex() const override = 0;

	std::vector<std::shared_ptr<Set>> members;
};


// Implementations

template<unsigned int DIM, typename PointT>
bool UnionSet<DIM,PointT>::contains(const PointT& q) const
{
	for (const std::shared_ptr<Set>& s : members)
	{
		if (s->contains(q)) return true;
	}
	return false;
}

template<unsigned int DIM, typename PointT>
UnionSet<DIM,PointT>::AABB UnionSet::getAABB() const
{
	throw std::logic_error("Function not yet implemented.");
}

template<unsigned int DIM, typename PointT>
bool UnionSet<DIM,PointT>::isConvex() const
{
	return members.size() == 1 and members.front()->isConvex();
}


template<unsigned int DIM, typename PointT>
bool IntersectionSet<DIM,PointT>::contains(const PointT& q) const
{
	for (const std::shared_ptr<Set>& s : members)
	{
		if (!s->contains(q)) return false;
	}
	return true;
}

template<unsigned int DIM, typename PointT>
IntersectionSet<DIM,PointT>::AABB IntersectionSet::getAABB() const
{
	throw std::logic_error("Function not yet implemented.");
}

template<unsigned int DIM, typename PointT>
bool IntersectionSet<DIM,PointT>::isConvex() const
{
	for (const std::shared_ptr<Set>& s : members)
	{
		if (!s->isConvex()) return false;
	}
	return true;
}

} // namespace coterie

#endif // COMPOSITESETS_HPP
