# Defines the set:
# { x \in R^n | (x-c)^T * A * (x-c) <= 1, c \in R^n, A \in R^{nxn} }
#
# Convert to visual ellipsoid via:
# A = V * D * V^{-1} (eigenvalue decomposition)
# Columns of V are the semi-axes, and the diagonal elements of D are the lengths

coterie_msgs/Point c
float64[] A
