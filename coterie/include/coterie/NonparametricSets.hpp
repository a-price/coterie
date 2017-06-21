/**
 * \file NonparametricSets.hpp
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

#ifndef NONPARAMETRICSETS_HPP
#define NONPARAMETRICSETS_HPP

#include "Set.hpp"

namespace coterie
{

template<int DIM, typename PointT=Eigen::Matrix<double, DIM, 1> >
class EmptySet : public Set<DIM, PointT>
{
public:
	typedef PointT point_type;
	static constexpr bool is_always_convex = true;
	static constexpr int dimension = DIM;

	typedef ::coterie::AABB<DIM, PointT> AABB;

	virtual bool contains(const PointT&) const override { return false; }
	virtual AABB getAABB() const override { return AABB::InitialBox(); }
	virtual bool isConvex() const override { return is_always_convex; } // P(x) (trivially) true \forall x \in \emptyset
};

template<int DIM, typename PointT=Eigen::Matrix<double, DIM, 1> >
class UniversalSet : public Set<DIM, PointT>
{
public:
	typedef PointT point_type;
	static constexpr bool is_always_convex = true;
	static constexpr int dimension = DIM;

	typedef ::coterie::AABB<DIM, PointT> AABB;

	virtual bool contains(const PointT&) const override { return true; }
	virtual AABB getAABB() const override;
	virtual bool isConvex() const override { return is_always_convex; }
};

template<int DIM, typename PointT>
typename UniversalSet<DIM,PointT>::AABB UniversalSet<DIM,PointT>::getAABB() const
{
	return AABB{-PointT::Ones()*std::numeric_limits<double>::infinity(), PointT::Ones()*std::numeric_limits<double>::infinity()};
}

} // namespace coterie
#endif // NONPARAMETRICSETS_HPP
