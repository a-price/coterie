/**
 * \file QuaternionTraits.hpp
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

#ifndef PROJECT_QUATERNIONTRAITS_HPP
#define PROJECT_QUATERNIONTRAITS_HPP

#include "coterie/ManifoldSet.hpp"

#include <Eigen/Geometry>

namespace coterie
{

template <>
struct manifold_traits<Eigen::Quaterniond>
{
	using manifold_type = Eigen::Quaterniond;
	using embedded_point_type = Eigen::Quaterniond;
	using tangent_vector_type = Eigen::Vector3d;

	static inline
	embedded_point_type compose(const embedded_point_type& p, const embedded_point_type& q)
	{
		return p * q;
	}

	static inline
	embedded_point_type inverse(const embedded_point_type& q)
	{
		return q.inverse();
	}

	static inline
	tangent_vector_type local(const embedded_point_type& p, const embedded_point_type& q)
	{
		return logmap(compose(inverse(p), q));
	}

	static inline
	embedded_point_type retract(const embedded_point_type& p, const tangent_vector_type& q)
	{
		return compose(p, expmap(q));
	}

	// Derived from https://bitbucket.org/gtborg/gtsam/src/cc25ece0551ba6ad1116046f5d4a19436d952d00/gtsam/geometry/Quaternion.h?at=develop
	static
	embedded_point_type expmap(const tangent_vector_type& omega)
	{
		using std::cos;
		using std::sin;

		double theta2 = omega.dot(omega);
		if (theta2 > std::numeric_limits<double>::epsilon())
		{
			double theta = std::sqrt(theta2);
			double ha = double(0.5) * theta;
			Eigen::Vector3d vec = (sin(ha) / theta) * omega;
			return Eigen::Quaterniond(cos(ha), vec.x(), vec.y(), vec.z());
		}
		else
		{
			// first order approximation sin(theta/2)/theta = 0.5
			Eigen::Vector3d vec = double(0.5) * omega;
			return Eigen::Quaterniond(1.0, vec.x(), vec.y(), vec.z());
		}
	}

	// Derived from https://bitbucket.org/gtborg/gtsam/src/cc25ece0551ba6ad1116046f5d4a19436d952d00/gtsam/geometry/Quaternion.h?at=develop
	static
	tangent_vector_type logmap(const embedded_point_type& q)
	{
		using std::acos;
		using std::sqrt;

		// define these compile time constants to avoid std::abs:
		static const double twoPi = 2.0 * M_PI, NearlyOne = 1.0 - 1e-10,
			NearlyNegativeOne = -1.0 + 1e-10;

		Eigen::Vector3d omega;

		const double qw = q.w();
		// See Quaternion-Logmap.nb in doc for Taylor expansions
		if (qw > NearlyOne)
		{
			// Taylor expansion of (angle / s) at 1
			// (2 + 2 * (1-qw) / 3) * q.vec();
			omega = ( 8. / 3. - 2. / 3. * qw) * q.vec();
		}
		else if (qw < NearlyNegativeOne)
		{
			// Taylor expansion of (angle / s) at -1
			// (-2 - 2 * (1 + qw) / 3) * q.vec();
			omega = (-8. / 3. - 2. / 3. * qw) * q.vec();
		}
		else
		{
			// Normal, away from zero case
			double angle = 2 * acos(qw), s = sqrt(1 - qw * qw);
			// Important:  convert to [-pi,pi] to keep error continuous
			if (angle > M_PI)
				angle -= twoPi;
			else if (angle < -M_PI)
				angle += twoPi;
			omega = (angle / s) * q.vec();
		}

		return omega;
	}
};

}

#endif //PROJECT_QUATERNIONTRAITS_HPP
