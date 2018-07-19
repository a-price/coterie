# PolymorphicSet.msg
# Provides a "base" message type which can contain one of any of the known set message subtypes.

# Enum constants for type parameter
uint8 TYPE_AABB_SET=1
uint8 TYPE_POINT_SET=2
uint8 TYPE_POLYTOPE_SET=3
uint8 TYPE_ELLIPSOIDAL_SET=4
uint8 TYPE_RASTER_SET=4

uint8 type # Specifies which type of set is being transmitted

# Message data
# Subtypes are listed as arrays, so that it is clear which ones are not in use (size() == 0)
coterie_msgs/AABBSet[] aabb
coterie_msgs/PointSet[] point
coterie_msgs/PolytopeSet[] polytope
coterie_msgs/EllipsoidalSet[] ellipsoid
coterie_msgs/RasterSet[] raster