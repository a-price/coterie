import cvxopt as cvx
import sympy as sp
import numpy as np

cvx.solvers.options['show_progress'] = False

from math import log, pi
from IPython.display import display
try: import pylab
except ImportError: pylab_installed = False
else: pylab_installed = True

import numpy.linalg as linalg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


sp.init_printing()


def diag(M):
    d = []
    for i in range(min(M.rows, M.cols)):
        d.append(M[i, i])
    return sp.Matrix(d)


def diag_indices(dim):
    indices = []

    counter = 0
    for i in range(dim):
        for j in range(i + 1):
            if i == j:
                indices.append(counter)
            counter += 1
    return indices


'''l=vec(L)'''
def symbolic_ellipsoid(dim):
    c = sp.zeros(dim, 1)
    l = sp.zeros(dim * (dim + 1) // 2, 1)
    L = sp.zeros(dim, dim)

    counter = 0
    for i in range(dim):
        c[i] = sp.symbols('c_' + str(i))
        for j in range(i + 1):
            L[i, j] = sp.symbols('L_' + str(i) + str(j))
            l[counter] = L[i, j]
            counter += 1
    return c, l, L


def symbolic_halfplane(dim):
    gk = sp.Matrix([sp.symbols('g_k' + str(i)) for i in range(dim)])
    hk = sp.symbols('h_k')

    return gk, hk


def tril_mask(dim):
    L = sp.zeros(dim, dim)
    for i in range(dim):
        for j in range(i + 1):
            L[i, j] = 1
    return L


"""The volume of an ellipsoid represented in lower-triangular form"""
def volume(L):
    return -sp.log(sp.det(L))


def grad(f, x):
    return sp.Matrix([[f.diff(i) for i in x]])


def hess(f, x):
    return sp.Matrix([[f.diff(i).diff(j) for i in x] for j in x])


class InscribedEllipsoid:
    """Functor object for fitting an ellipsoid inside hyperplanes"""

    def __init__(self, dim, hyperplanes):
        self.dim = dim
        self.num_nl_constraints = len(hyperplanes)
        self.gk = []
        self.hk = []
        for k in range(self.num_nl_constraints):
            assert(dim + 1 == len(hyperplanes[k]))
            self.gk.append(hyperplanes[k][:-1])
            self.hk.append(hyperplanes[k][-1])

        c, l, L = symbolic_ellipsoid(dim)
        x = sp.BlockMatrix([[l], [c]]).as_explicit()  # Poor man's vertcat
        gk, hk = symbolic_halfplane(dim)

        self.num_vars = x.rows

        f = volume(L) # Cost function
        Df = grad(f, x) # Gradient of cost function
        Hf = hess(f, x) # Hessian of cost function

        tau = ((sp.Matrix([[hk]])-gk.T*c)[0])
        g = ((L.T*gk).T*(L.T*gk))[0]/(tau)-tau # Constraint function
        Dg = grad(g, x) # Gradient of constraint function
        Hg = hess(g, x) # Hessian of constraint function
        # sp.matrix_multiply_elementwise(Hg,tril_mask(x.rows)))

        self.x0 = cvx.matrix(0.0, (l.rows + c.rows, 1))
        for i in diag_indices(dim):
            self.x0[i, 0] = 1.0

        # TODO: Bug report?
        # X = sp.MatrixSymbol('x', len(x), 1)
        # replacements = [(x[i], X[i]) for i in range(x.rows)]
        # f = f.subs(replacements)
        # print(f)
        # self.f = sp.lambdify(X, f) # (x[0], x[1], x[2], x[3], x[4])
        # print(self.f)

        self.f  = sp.lambdify(x, f)
        self.Df = sp.lambdify(x, Df)
        self.Hf = sp.lambdify(x, Hf)
        args = [xi for xi in x] + [gki for gki in gk] + [hk]
        self.g  = sp.lambdify(args, g)
        self.Dg = sp.lambdify(args, Dg)
        self.Hg = sp.lambdify(args, Hg)

    # NB: you can't have multiple functions with the same name in Python, so you have to settle for this stupid crap
    def __call__(self, x=None, z=None):
        if x is None:
            return self.num_nl_constraints, self.x0

        # The value of the constraint functions is a function of the current point plus the constraint points
        args = []
        for k in range(self.num_nl_constraints):
            args.append([xi for xi in x] + [gki for gki in self.gk[k]] + [self.hk[k]])

        # Populate values of cost and constraint functions
        f = cvx.matrix(0.0, (self.num_nl_constraints + 1, 1))
        f[0] = self.f(*x)
        for k in range(self.num_nl_constraints):
            f[k + 1] = float(self.g(*args[k]))

        # Populate gradients of cost and constraint functions
        Df = cvx.matrix(0.0, (self.num_nl_constraints + 1, self.num_vars))
        Df[0, :] = self.Df(*x).astype(np.double)
        for k in range(self.num_nl_constraints):
            Df[k + 1, :] = self.Dg(*args[k]).astype(np.double)

        if z is None:
            return f, Df

        H = cvx.matrix(0.0, (self.num_vars, self.num_vars))
        H += cvx.matrix(self.Hf(*x).astype(np.double))
        for k in range(self.num_nl_constraints):
            H += z[k] * cvx.matrix(self.Hg(*args[k]).astype(np.double))

        return f, Df, H

dimension = 3

if dimension == 2:
    # Extreme points (with first one appended at the end)
    X = cvx.matrix([ 0.55,  0.25, -0.20, -0.25,  0.00,  0.40,  0.55,
                 0.00,  0.35,  0.20, -0.10, -0.30, -0.20,  0.00 ], (7,2))
    m = X.size[0] - 1
    # Inequality description G*x <= h with h = 1
    G, h = cvx.matrix(0.0, (m,2)), cvx.matrix(0.0, (m,1)) # Zero-initialize
    G = (X[:m,:] - X[1:,:]) * cvx.matrix([0., -1., 1., 0.], (2,2))
    h = (G * X.T)[::m+1]
    G = cvx.mul(h[:,[0,0]]**-1, G)
    h = cvx.matrix(1.0, (m,1))

    ell = InscribedEllipsoid(2, [[G[k,0], G[k,1], h[k]] for k in range(m)])

    sol = cvx.solvers.cp(ell)
    L = cvx.matrix([sol['x'][0], sol['x'][1], 0.0, sol['x'][2]], (2,2))
    c = cvx.matrix([sol['x'][3], sol['x'][4]])

    if pylab_installed:
        pylab.figure(2, facecolor='w')
        pylab.plot(X[:, 0], X[:, 1], 'ko')  # , X[:, 0], X[:, 1], '-k')

        # polyhedron
        for k in range(m):
            edge = X[[k, k + 1], :] + 0.1 * cvx.matrix([1., 0., 0., -1.], (2, 2)) * (X[2 * [k], :] - X[2 * [k + 1], :])
            pylab.plot(edge[:, 0], edge[:, 1], 'k')

        # 1000 points on the unit circle
        nopts = 1000
        angles = cvx.matrix([a * 2.0 * pi / nopts for a in range(nopts)], (1, nopts))
        circle = cvx.matrix(0.0, (2, nopts))
        circle[0, :], circle[1, :] = cvx.cos(angles), cvx.sin(angles)

        # ellipse = L * circle + c
        ellipse = L * circle + c[:, nopts * [0]]
        # ellipse2 = 2.0 * L * circle + c[:, nopts*[0]]

        # pylab.plot(ellipse2[0,:].T, ellipse2[1,:].T, 'k-')
        pylab.fill(ellipse[0, :].T, ellipse[1, :].T, facecolor='#F0F0F0')
        pylab.title('Maximum volume inscribed ellipsoid (fig 8.4)')
        pylab.axis('equal')
        # pylab.axis('off')

        pylab.show()

elif dimension == 3:
    G = np.matrix([[0, 0, 2], [0, 0, -2],
                   [0, 1, 0], [0, -1, 0],
                   [1, 0, 0], [-1, 0, 0]])
    m = 6
    h = cvx.matrix(1.0, (m, 1))

    ell = InscribedEllipsoid(3, [[G[k, 0], G[k, 1], G[k, 2], h[k]] for k in range(m)])

    sol = cvx.solvers.cp(ell)
    x = sol['x']
    L = np.matrix([[x[0], 0.0, 0.0],
                   [x[1], x[2], 0.0],
                   [x[3], x[4], x[5]]])
    L = cvx.matrix(L)
    c = cvx.matrix([x[6], x[7], x[8]])

    print(L)
    print(c)

    A = np.array(L*L.T)
    center = np.array(c).squeeze()

    # find the rotation matrix and radii of the axes
    U, s, rotation = linalg.svd(A)
    radii = 1.0 / np.sqrt(s)

    u = np.linspace(0.0, 2.0 * np.pi, 100)
    v = np.linspace(0.0, np.pi, 100)
    x = radii[0] * np.outer(np.cos(u), np.sin(v))
    y = radii[1] * np.outer(np.sin(u), np.sin(v))
    z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
    for i in range(len(x)):
        for j in range(len(x)):
            [x[i, j], y[i, j], z[i, j]] = np.dot([x[i, j], y[i, j], z[i, j]], rotation) + center

    # plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_wireframe(x, y, z, rstride=4, cstride=4, color='b', alpha=0.2)
    # ax.plot([smin[0], smax[0]], [smin[1], smax[1]], zs=[smin[2], smax[2]], linewidth=2, color='r')
    plt.show()
