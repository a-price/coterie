//
// Created by arprice on 8/28/19.
//

#ifndef SRC_MOSEKELLIPSOIDSOLVER_H
#define SRC_MOSEKELLIPSOIDSOLVER_H

#if USE_MOSEK
#include "fusion.h"

//using namespace mosek::fusion;
//using namespace monty;

namespace coterie
{
namespace mosek
{


template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>,
	typename MatrixT=Eigen::Matrix<double, DIM, DIM>,
	typename std::enable_if<coterie::Dynamic != DIM>::type* = nullptr >
::mosek::fusion::Matrix::t quadForm(const coterie::EllipsoidalSet<DIM, PointT, MatrixT>& es)
{
	auto Ac = es.A * es.c;
	Eigen::Matrix<double, DIM+1, DIM+1> Q;
	Q << es.A, -Ac, -Ac.transpose(), (es.c.transpose() * Ac) - 1.0;

	using Shape = monty::shape_t<2>;

	return ::mosek::fusion::Matrix::dense(std::make_shared<monty::ndarray<double, 2>>(
		Q.data(),
		Shape{static_cast<Shape::dim_t>(DIM + 1),
		      static_cast<Shape::dim_t>(DIM + 1)}));
}


template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>,
	typename MatrixT=Eigen::Matrix<double, DIM, DIM>,
	typename std::enable_if<coterie::Dynamic == DIM>::type* = nullptr >
::mosek::fusion::Matrix::t quadForm(const coterie::EllipsoidalSet<DIM, PointT, MatrixT>& es)
{
	auto Ac = es.A * es.c;
	Eigen::Matrix<double, coterie::Dynamic, coterie::Dynamic> Q(es.dimension + 1, es.dimension + 1);
	Q << es.A, -Ac, -Ac.transpose(), (es.c.transpose() * Ac) - 1.0;

	using Shape = monty::shape_t<2>;

	return ::mosek::fusion::Matrix::dense(std::make_shared<monty::ndarray<double, 2>>(
		Q.data(),
		Shape{static_cast<Shape::dim_t>(es.dimension + 1),
		      static_cast<Shape::dim_t>(es.dimension + 1)}));
}

template<int DIM,
	typename PointT=Eigen::Matrix<double, DIM, 1>,
	typename MatrixT=Eigen::Matrix<double, DIM, DIM>>
bool lhsContainsRhs(const coterie::EllipsoidalSet<DIM, PointT, MatrixT>& lhs,
                    const coterie::EllipsoidalSet<DIM, PointT, MatrixT>& rhs)
{
	using ::mosek::fusion::Model;
	using ::mosek::fusion::Matrix;
	using ::mosek::fusion::Variable;
	using ::mosek::fusion::Expr;
	using ::mosek::fusion::Domain;
	using ::mosek::fusion::ObjectiveSense;
	using ::mosek::fusion::ProblemStatus;

	Matrix::t S1 = quadForm(rhs);
	Matrix::t S2 = quadForm(lhs);

	Model::t M = new Model("sdo1"); auto _M = monty::finally([&]() { M->dispose(); });
	Variable::t lambda = M->variable("lambda", Domain::greaterThan(0));
	Variable::t X = M->variable("stub", Domain::inPSDCone(2));

	M->objective(ObjectiveSense::Minimize, Expr::zeros(1));
	M->constraint("c1", Expr::sub(X, Expr::sub(Expr::mul(lambda, S1), S2)), Domain::equalsTo(0));
	M->solve();

//	M->writeTask("/home/arprice/mosek_task.ptf");

	auto status = M->getProblemStatus();
	switch(status)
	{
	case ProblemStatus::PrimalFeasible:
//	case ProblemStatus::DualFeasible:
	case ProblemStatus::PrimalAndDualFeasible:
		return true;

	case ProblemStatus::PrimalInfeasible:
//	case ProblemStatus::DualInfeasible:
//	case ProblemStatus::PrimalAndDualInfeasible:
		return false;

	default:
		std::stringstream str;
		str << status;
		throw std::logic_error("MOSEK return type is not handled in ellipsoid solver: '" + str.str() + "'.");
	}
}

} // namespace mosek
} // namespace coterie

#endif

#endif //SRC_MOSEKELLIPSOIDSOLVER_H
