extends RefCounted
## Specification of the front telescopic suspension (M2.2): spring-damper law, spring seats and finite fork travel limits, all
## in SI units. Plain data; scene-tree independent. Validation lives in MotorcycleConfig.validate() (single home of the rules).
##
## Displacement convention (see front_suspension.gd and PHYSICS_CONVENTIONS.md): x = fork extension from the reference pose
## in metres, positive = extension (slider moves away from the steering assembly along the fork axis), negative = compression.
## Seat positions are WORLD points at the reference pose, on one line parallel to the fork axis.

## Spring seat on the steering assembly (upper end) and on the fork slider (lower end), world coordinates at the reference pose.
var upper_attachment: Vector3 = Vector3.ZERO
var lower_attachment: Vector3 = Vector3.ZERO
## N/m, finite, >= 0.
var stiffness: float = 0.0
## N s/m, finite, >= 0.
var damping: float = 0.0
## m, finite: displacement at which the spring force is zero (0 = unloaded at the reference pose).
var rest_displacement: float = 0.0
## Finite travel limits on the prismatic coordinate (m). Compression limit is the lower, extension limit the upper bound.
## When enabled: lower <= 0 <= upper (the reference pose lies inside the travel).
var travel_limits_enabled: bool = false
var lower_travel_limit: float = 0.0
var upper_travel_limit: float = 0.0
