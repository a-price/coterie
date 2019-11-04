/**
 * \file CartesianProductSet.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2018-05-07
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

#ifndef PROJECT_CARTESIANPRODUCTSET_HPP
#define PROJECT_CARTESIANPRODUCTSET_HPP

namespace coterie
{

template<typename LHSSet, typename RHSSet>
struct joint_properties
{
	static constexpr bool is_always_convex = LHSSet::is_always_convex && RHSSet::is_always_convex;
	static constexpr int compile_time_dimension = (LHSSet::compile_time_dimension < 0 || RHSSet::compile_time_dimension < 0)
	                                              ? -1 : LHSSet::compile_time_dimension + RHSSet::compile_time_dimension;

	using point_type = std::pair<typename LHSSet::point_type, typename RHSSet::point_type>;

	static inline
	typename LHSSet::point_type& head(point_type& pt);

	static inline
	const typename LHSSet::point_type& head(const point_type& pt);

	static inline
	typename RHSSet::point_type& tail(point_type& pt);

	static inline
	const typename RHSSet::point_type& tail(const point_type& pt);
};

template<typename LHSSet, typename RHSSet>
class CartesianProductSet
	: public Set<joint_properties<LHSSet, RHSSet>::compile_time_dimension,
		typename joint_properties<LHSSet, RHSSet>::point_type>
{
public:
	using lhs_type = LHSSet;
	using rhs_type = RHSSet;
	using properties = joint_properties<LHSSet, RHSSet>;
	using point_type = typename properties::point_type;
	static constexpr bool is_always_convex = false;
	static constexpr int compile_time_dimension = properties::compile_time_dimension;
	const int dimension;

	using Base = Set<compile_time_dimension, point_type>;

	using AABB = ::coterie::AABB<compile_time_dimension, point_type>;
//	using AABB = CartesianProductSet<::coterie::AABB<LHSSet::dimension, LHSSet::point_type>,
//	                                 ::coterie::AABB<RHSSet::dimension, RHSSet::point_type>>;

	LHSSet lhsSet;
	RHSSet rhsSet;

	template<int D = compile_time_dimension, typename std::enable_if<Dynamic != D>::type* = nullptr>
	CartesianProductSet(const LHSSet& lhs, const RHSSet& rhs) : Base(), dimension(compile_time_dimension), lhsSet(lhs), rhsSet(rhs) {}

	template<int D = compile_time_dimension, typename std::enable_if<Dynamic == D>::type* = nullptr>
	CartesianProductSet(const LHSSet& lhs, const RHSSet& rhs) : Base(lhs.dimension+rhs.dimension), dimension(lhs.dimension+rhs.dimension), lhsSet(lhs), rhsSet(rhs) {}

	virtual bool contains(const point_type& pt) const override;

	virtual AABB getAABB() const override;

	virtual bool isConvex() const override;

};

} // namespace coterie

#endif //PROJECT_CARTESIANPRODUCTSET_HPP
