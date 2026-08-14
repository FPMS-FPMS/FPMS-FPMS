# One tracked obstacle, in the frame given by `header`.
#
# The polygon is the SUPPORT OUTLINE of the tracker's own returns, in order --
# not a hull, not a fitted circle. A root flare, a multi-stem clump and a fallen
# limb are all non-convex, and a convex approximation of a fallen limb closes a
# gap the rover could have driven through. forest_nav::DynamicObstacleLayer
# rasterises this outline as given.

std_msgs/Header header

uint8 STATIC=0     # not moving: a trunk
uint8 DYNAMIC=1    # moving, with evidence: velocity may be predicted forward
uint8 TERRAIN=2    # too large to be a discrete obstacle: hedge, deadfall pile,
                   # embankment. Routed around, but NEVER predicted forward.

uint32 id
uint8 classification
geometry_msgs/Point32 position     # tracked centroid
geometry_msgs/Vector3 velocity     # m/s; ZERO unless classification == DYNAMIC
float32 radius                     # max extent from the centroid, metres
float32 confidence                 # 0..1, from corroboration and coasting
geometry_msgs/Polygon polygon      # the non-convex support outline
