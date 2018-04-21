#!/usr/bin/env python

from __future__ import division
import cvxpy
import numpy as np
import numpy.linalg as linalg


# TODO: Document why Q and not Qinv (A-form instead of B-form)
def ellipse_quadratic(c, Q):
    Qc = np.matmul(Q, c)
    return np.bmat([[Q, -Qc],[-Qc.transpose(), np.matmul(c.transpose(), Qc)-1.0]])

# http://systemanalysisdpt-cmc-msu.github.io/ellipsoids/doc/chap_ellcalc.html#checking-if-one-ellipsoid-contains-another
# def ellipse_quadratic(c, Q):
#     Qi = linalg.inv(Q)
#     Qic = np.matmul(Qi, c) # NB: In Python 3.5+, equivalent to '@'
#     return np.bmat([[Qi, -Qic],[-Qic.transpose(), np.matmul(c.transpose(), Qic)-1.0]])


# Returns true if ellipsoid1 contains ellipsoid2
def ellipsoid_contains(c1, Q1, c2, Q2):
    # Problem data.
    dim = len(c1)
    # Q1 = np.matrix([[1, 0], [0, 1]])
    # c1 = np.matrix([[0], [0]])
    # Q2 = np.matrix([[2, 0], [0, 2]])
    # c2 = np.matrix([[0], [0]])

    A = ellipse_quadratic(c1, Q1)
    B = ellipse_quadratic(c2, Q2)

    # Construct the problem.
    x = cvxpy.Variable()
    objective = cvxpy.Minimize(0)
    constraints = [0 <= x,
                   0 << x*B - A] # http://www.cvxpy.org/en/latest/tutorial/advanced/#semidefinite-matric
    prob = cvxpy.Problem(objective, constraints)

    # The optimal objective is returned by prob.solve().
    result = prob.solve(solver=cvxpy.CVXOPT, verbose=False)
    # The optimal value for x is stored in x.value.
    return x.value is not None






