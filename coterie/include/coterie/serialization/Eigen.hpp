/**
 * \file Eigen.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2017-3-28
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

#ifndef SERIALIZE_EIGEN_HPP
#define SERIALIZE_EIGEN_HPP

#include <boost/serialization/array.hpp>
#include <boost/serialization/utility.hpp>
#include <Eigen/Core>

namespace boost
{
namespace serialization
{

/**
 * @brief Serialize an Eigen Matrix
 * @param ar Archive into which to save the matrix
 * @param t The matrix to save
 * @param file_version
 */
template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options,
		int _MaxRows, int _MaxCols>
inline void serialize(Archive & ar,
		Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t,
		const unsigned int)
{
	typedef typename Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::Index Index;
	Index rows = t.rows(), cols = t.cols();
	ar & rows;
	ar & cols;
	if (rows * cols != t.size())
		t.resize(rows, cols);

//	for (Index i = 0; i < t.size(); ++i)
//		ar & t.data()[i];
	ar & boost::serialization::make_array(t.data(), t.size());
}

} // namespace serialization
} // namespace boost

#endif // SERIALIZE_EIGEN_HPP
