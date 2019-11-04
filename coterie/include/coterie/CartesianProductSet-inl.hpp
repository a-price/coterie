/**
 * \file CartesianProductSet-inl.hpp
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

#ifndef PROJECT_CARTESIANPRODUCTSET_INL_HPP
#define PROJECT_CARTESIANPRODUCTSET_INL_HPP

#include "coterie/CartesianProductSet.hpp"

namespace coterie
{

template<typename LHSSet, typename RHSSet>
typename LHSSet::point_type&
joint_properties<LHSSet, RHSSet>::head(joint_properties<LHSSet, RHSSet>::point_type& pt)
{
	return pt.first;
}

template<typename LHSSet, typename RHSSet>
const typename LHSSet::point_type&
joint_properties<LHSSet, RHSSet>::head(const joint_properties<LHSSet, RHSSet>::point_type& pt)
{
	return pt.first;
}

template<typename LHSSet, typename RHSSet>
typename RHSSet::point_type&
joint_properties<LHSSet, RHSSet>::tail(joint_properties<LHSSet, RHSSet>::point_type& pt)
{
	return pt.second;
}

template<typename LHSSet, typename RHSSet>
const typename RHSSet::point_type&
joint_properties<LHSSet, RHSSet>::tail(const joint_properties<LHSSet, RHSSet>::point_type& pt)
{
	return pt.second;
}


template<typename LHSSet, typename RHSSet>
bool
CartesianProductSet<LHSSet, RHSSet>::contains(const CartesianProductSet<LHSSet, RHSSet>::point_type& pt) const
{
	return lhsSet.contains(properties::head(pt)) && rhsSet.contains(properties::tail(pt));
}

template<typename LHSSet, typename RHSSet>
typename CartesianProductSet<LHSSet, RHSSet>::AABB
CartesianProductSet<LHSSet, RHSSet>::getAABB() const
{
	throw std::runtime_error("Cartesian Product AABB not implemented.");
}

template<typename LHSSet, typename RHSSet>
bool
CartesianProductSet<LHSSet, RHSSet>::isConvex() const
{
	return lhsSet.isConvex() && rhsSet.isConvex();
}

} // namespace coterie

#endif //PROJECT_CARTESIANPRODUCTSET_INL_HPP
