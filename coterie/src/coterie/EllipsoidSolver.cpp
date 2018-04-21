/**
 * \file EllipsoidSolver.cpp
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

#include "coterie/EllipsoidSolver.h"

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION // Disallow NumPy APIs older than 1.7
#include <Python.h>
#include <numpy/arrayobject.h>

namespace coterie
{

// NB: you have to call import_array() or PyArray_SimpleNewFromData will segfault
void init_numpy()
{
	import_array();
}

class EllipsoidSolver::EllipsoidSolverImpl
{
public:
	const std::string moduleName = "coterie.ellipsoids";
	const std::string functionName = "ellipsoid_contains";

	PyObject *pModuleName;
	PyObject *pModule, *pFunc;
	PyObject *pArgs, *pValue;

	EllipsoidSolverImpl()
		: pArgs(nullptr),
		  pValue(nullptr)
	{

		Py_Initialize();
		init_numpy();
		pModuleName = PyString_FromString(moduleName.c_str());
		pModule = PyImport_Import(pModuleName);
		Py_DECREF(pModuleName);

		if (pModule)
		{
			pFunc = PyObject_GetAttrString(pModule, functionName.c_str());
			/* pFunc is a new reference */

			if (!(pFunc && PyCallable_Check(pFunc)))
			{
				if (PyErr_Occurred())
					PyErr_Print();
				throw std::runtime_error("Cannot find Python function " + functionName);
			}
			Py_XDECREF(pFunc);
			Py_DECREF(pModule);
		}
		else
		{
			PyErr_Print();
			throw std::runtime_error("Failed to load Python module " + moduleName);
		}
	}

	~EllipsoidSolverImpl()
	{
		Py_Finalize();
	}

	bool contains(const Eigen::VectorXd& c1, const Eigen::MatrixXd& Q1,
	              const Eigen::VectorXd& c2, const Eigen::MatrixXd& Q2)
	{
		const int dim = static_cast<int>(c1.size());

		const int ND{ 2 };
		npy_intp c_dims[2]{dim, 1};
		npy_intp Q_dims[2]{dim, dim};

		// Do we need to worry about row vs column storage order? The matrices are symmetric...
		PyArrayObject *np_arg_c1, *np_arg_Q1, *np_arg_c2, *np_arg_Q2;
		np_arg_c1 = reinterpret_cast<PyArrayObject*>(PyArray_SimpleNewFromData(ND, c_dims, NPY_DOUBLE, const_cast<void*>(reinterpret_cast<const void*>(c1.data()))));
		np_arg_Q1 = reinterpret_cast<PyArrayObject*>(PyArray_SimpleNewFromData(ND, Q_dims, NPY_DOUBLE, const_cast<void*>(reinterpret_cast<const void*>(Q1.data()))));
		np_arg_c2 = reinterpret_cast<PyArrayObject*>(PyArray_SimpleNewFromData(ND, c_dims, NPY_DOUBLE, const_cast<void*>(reinterpret_cast<const void*>(c2.data()))));
		np_arg_Q2 = reinterpret_cast<PyArrayObject*>(PyArray_SimpleNewFromData(ND, Q_dims, NPY_DOUBLE, const_cast<void*>(reinterpret_cast<const void*>(Q2.data()))));

		pArgs = PyTuple_New(4);
		PyTuple_SetItem(pArgs, 0, reinterpret_cast<PyObject*>(np_arg_c1));
		PyTuple_SetItem(pArgs, 1, reinterpret_cast<PyObject*>(np_arg_Q1));
		PyTuple_SetItem(pArgs, 2, reinterpret_cast<PyObject*>(np_arg_c2));
		PyTuple_SetItem(pArgs, 3, reinterpret_cast<PyObject*>(np_arg_Q2));

		pValue = PyObject_CallObject(pFunc, pArgs);

		Py_DECREF(pArgs);
		if (pValue != NULL)
		{
			bool res = static_cast<bool>(PyObject_IsTrue(pValue));
			Py_DECREF(pValue);
			return res;
		}
		else
		{
			Py_DECREF(pFunc);
			Py_DECREF(pModule);
			PyErr_Print();
			throw std::runtime_error("Python call for ellipsoid containment failed.");
		}
	}
};

EllipsoidSolver::EllipsoidSolver()
{
	impl = std::make_unique<EllipsoidSolverImpl>();
}

EllipsoidSolver::~EllipsoidSolver() = default;

bool EllipsoidSolver::contains(const Eigen::VectorXd& c1, const Eigen::MatrixXd& Q1,
                               const Eigen::VectorXd& c2, const Eigen::MatrixXd& Q2)
{
	assert(c1.size() == c2.size());
	assert(c1.size() == Q1.rows() && c1.size() == Q1.cols());
	assert(c2.size() == Q2.rows() && c2.size() == Q2.cols());

	if (c1 == c2 && Q1 == Q2) { return true; }

	return impl->contains(c1, Q1, c2, Q2);
}

} // namespace coterie
