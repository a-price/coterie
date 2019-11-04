/**
 * \file PoseSetProperties.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2018-05-07
 *
 * \copyright
 *
 * Copyright (c) 2018, Andrew Price
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

#ifndef PROJECT_POSESETPROPERTIES_HPP
#define PROJECT_POSESETPROPERTIES_HPP


#include "coterie/QuaternionTraits.hpp"
#include "coterie/CartesianProductSet.hpp"

namespace coterie
{

template<>
struct joint_properties<EllipsoidalSet<3>, ManifoldSet<Eigen::Quaterniond, EllipsoidalSet<3>>>
{
	static constexpr bool is_always_convex = true;
	static constexpr int compile_time_dimension = 6;

	using translation_set = EllipsoidalSet<3>;
	using rotation_set = ManifoldSet<Eigen::Quaterniond, EllipsoidalSet<3>>;

	using translation_point_type = typename translation_set::point_type;
	using rotation_point_type = typename rotation_set::point_type;

	using point_type = ap::PoseState;

	static
	translation_point_type& head(point_type& pt)
	{
		return pt.t;
	}

	static
	const translation_point_type& head(const point_type& pt)
	{
		return pt.t;
	}

	static
	rotation_point_type& tail(point_type& pt)
	{
		return pt.q;
	}

	static
	const rotation_point_type& tail(const point_type& pt)
	{
		return pt.q;
	}
};

template<>
struct joint_properties<PolytopeSet<3>, ManifoldSet<Eigen::Quaterniond, PolytopeSet<3>>>
{
	static constexpr bool is_always_convex = true;
	static constexpr int compile_time_dimension = 6;

	using translation_set = PolytopeSet<3>;
	using rotation_set = ManifoldSet<Eigen::Quaterniond, PolytopeSet<3>>;

	using translation_point_type = typename translation_set::point_type;
	using rotation_point_type = typename rotation_set::point_type;

	using point_type = ap::PoseState;

	static
	translation_point_type& head(point_type& pt)
	{
		return pt.t;
	}

	static
	const translation_point_type& head(const point_type& pt)
	{
		return pt.t;
	}

	static
	rotation_point_type& tail(point_type& pt)
	{
		return pt.q;
	}

	static
	const rotation_point_type& tail(const point_type& pt)
	{
		return pt.q;
	}
};

template<>
struct joint_properties<PointSet<3>, ManifoldSet<Eigen::Quaterniond, PointSet<3>>>
{
	static constexpr bool is_always_convex = true;
	static constexpr int compile_time_dimension = 6;

	using translation_set = PointSet<3>;
	using rotation_set = ManifoldSet<Eigen::Quaterniond, PointSet<3>>;

	using translation_point_type = typename translation_set::point_type;
	using rotation_point_type = typename rotation_set::point_type;

	using point_type = ap::PoseState;

	static
	translation_point_type& head(point_type& pt)
	{
		return pt.t;
	}

	static
	const translation_point_type& head(const point_type& pt)
	{
		return pt.t;
	}

	static
	rotation_point_type& tail(point_type& pt)
	{
		return pt.q;
	}

	static
	const rotation_point_type& tail(const point_type& pt)
	{
		return pt.q;
	}
};

extern template
class ManifoldSet<Eigen::Quaterniond, EllipsoidalSet<3>>;

extern template
class CartesianProductSet<EllipsoidalSet<3>, ManifoldSet<Eigen::Quaterniond, EllipsoidalSet<3>>>;

} // namespace coterie

#endif //PROJECT_POSESETPROPERTIES_HPP
