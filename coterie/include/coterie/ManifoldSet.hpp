/**
 * \file ManifoldSet.hpp
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

#ifndef PROJECT_MANIFOLDSET_HPP
#define PROJECT_MANIFOLDSET_HPP

#include <coterie/Set.hpp>

namespace coterie
{

template <typename ManifoldT>
struct manifold_traits;

template<typename ManifoldT, typename VectorSpaceSetT>
class ManifoldSet : public VectorSpaceSetT
{
public:
	using manifold_type = ManifoldT; // eg: QuaternionGroup
	using vector_space_set_type = VectorSpaceSetT; // eg: EllipsoidalSet<3, Eigen::Vector3d, Eigen::Matrix3d>
	using embedded_point_type = typename manifold_traits<manifold_type>::embedded_point_type; // eg: Eigen::Quaterniond
	using tangent_vector_type = typename VectorSpaceSetT::point_type; // eg: Eigen::Vector3d

	using point_type = embedded_point_type;

	embedded_point_type origin;

	ManifoldSet(const embedded_point_type& o, const VectorSpaceSetT& set) : VectorSpaceSetT(set), origin(o) {}

	bool contains(const embedded_point_type &pt) const
	{
		return vector_space_set_type::contains(
			manifold_traits<manifold_type>::local(origin, pt));
	}

};

}

#endif //PROJECT_MANIFOLDSET_HPP
