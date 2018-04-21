/**
 * \file EllipsoidSolver.h
 * \brief
 *
 * \author Andrew Price
 * \date 2017-9-10
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

#ifndef ELLIPSOIDSOLVER_H
#define ELLIPSOIDSOLVER_H

#include <coterie/EllipsoidalSet.hpp>
#include <Eigen/Core>
#include <memory>

namespace coterie
{

class EllipsoidSolver
{
public:
	static EllipsoidSolver& getInstance()
	{
		static EllipsoidSolver instance;
		return instance;
	}

	// check whether E1 is inside E2
	template<int DIM,
		typename PointT=Eigen::Matrix<double, DIM, 1>,
		typename MatrixT=Eigen::Matrix<double, DIM, DIM> >
	bool contains(const coterie::EllipsoidalSet<DIM, PointT, MatrixT>& e1,
	              const coterie::EllipsoidalSet<DIM, PointT, MatrixT>& e2)
	{
		assert(e1.c.size() == e2.c.size());
		assert(e1.c.size() == e1.A.rows() && e1.c.size() == e1.A.cols());
		assert(e2.c.size() == e2.A.rows() && e2.c.size() == e2.A.cols());

		if (e1.c == e2.c && e1.A == e2.A) { return true; }

		return this->contains(e1.c, e1.A, e2.c, e2.A);
	}

	// check whether E1 is inside E2
	bool contains(const Eigen::VectorXd& c1, const Eigen::MatrixXd& Q1,
	              const Eigen::VectorXd& c2, const Eigen::MatrixXd& Q2);

private:
	EllipsoidSolver();
	~EllipsoidSolver();

	// Isolate Python or ROS dependencies from rest of system
	class EllipsoidSolverImpl;
	std::unique_ptr<EllipsoidSolverImpl> impl;

public:
	EllipsoidSolver(EllipsoidSolver const&) = delete;
	void operator=(EllipsoidSolver const&) = delete;

};

} // namespace coterie

#endif // ELLIPSOIDSOLVER_H
