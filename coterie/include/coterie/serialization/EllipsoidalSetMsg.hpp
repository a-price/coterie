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

#ifndef SRC_ELLIPSOIDALSETMSG_HPP
#define SRC_ELLIPSOIDALSETMSG_HPP

#include "coterie/EllipsoidalSet.hpp"
#include "coterie/serialization/PointMsg.hpp"

#include <coterie_msgs/EllipsoidalSet.h>

namespace coterie
{

template<int DIM, typename PointT>
void serialize(coterie_msgs::EllipsoidalSet& msg, const EllipsoidalSet<DIM, PointT>& set)
{
	serialize(msg.c, set.c);
	serialize(msg.A, set.A);
}

//template<typename PointT>
//void deserialize(coterie_msgs::EllipsoidalSet& msg, const EllipsoidalSet<Dynamic, PointT>& set)
//{
//	deserialize(msg.c, set.c);
//	deserialize(msg.A, set.A);
//}


EllipsoidalSet<Dynamic> deserialize(const coterie_msgs::EllipsoidalSet& msg)
{
	return EllipsoidalSet<Dynamic>::ARep(
		deserialize(msg.c),
		Eigen::Map<const Eigen::MatrixXd>(msg.A.data(), msg.c.data.size(), msg.c.data.size())
	);
}

} // namespace coterie

#endif //SRC_ELLIPSOIDALSETMSG_HPP
