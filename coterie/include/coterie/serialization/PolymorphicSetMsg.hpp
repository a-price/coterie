/*
 * Copyright (c) 2019 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SRC_POLYMORPHICSETMSG_HPP
#define SRC_POLYMORPHICSETMSG_HPP

#include <coterie_msgs/PolymorphicSet.h>

#include "coterie/serialization/AABBMsg.hpp"
#include "coterie/serialization/EllipsoidalSetMsg.hpp"
#include "coterie/serialization/PolytopeSetMsg.hpp"
#include "coterie/RasterSet.hpp"

namespace coterie
{

template<int DIM, typename PointT>
void serialize(coterie_msgs::PolymorphicSet& msg, const AABB<DIM, PointT>& set)
{
	msg.type = coterie_msgs::PolymorphicSet::TYPE_AABB_SET;
	msg.space = 0; //set.dimension

	msg.aabb.resize(1);
	serialize(msg.aabb[0], set);
}

template<int DIM, typename PointT, typename MatrixT>
void serialize(coterie_msgs::PolymorphicSet& msg, const EllipsoidalSet<DIM, PointT, MatrixT>& set)
{
	msg.type = coterie_msgs::PolymorphicSet::TYPE_ELLIPSOIDAL_SET;
	msg.space = 0; //set.dimension

	msg.ellipsoid.resize(1);
	serialize(msg.ellipsoid[0], set);
}

template<int DIM, typename PointT>
void serialize(coterie_msgs::PolymorphicSet& msg, const PointSet<DIM, PointT>& set)
{
	msg.type = coterie_msgs::PolymorphicSet::TYPE_POINT_SET;
	msg.space = 0; //set.dimension

	msg.point.resize(1);
	serialize(msg.point[0], set);
}

template<int DIM, typename PointT>
void serialize(coterie_msgs::PolymorphicSet& msg, const PolytopeSet<DIM, PointT>& set)
{
	msg.type = coterie_msgs::PolymorphicSet::TYPE_POLYTOPE_SET;
	msg.space = 0; //set.dimension

	msg.polytope.resize(1);
	serialize(msg.polytope[0], set);
}

} // namespace coterie

#endif //SRC_POLYMORPHICSETMSG_HPP
