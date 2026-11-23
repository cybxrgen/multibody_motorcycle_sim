extends RefCounted
## Specification of the rear spring-damper (M2.3): the two attachment points, the spring-damper law and the rest length, all in
## SI units. Plain data; scene-tree independent. Validation lives in MotorcycleConfig.validate() (single home of the rules).
##
## Convention (see rear_suspension.gd and PHYSICS_CONVENTIONS.md): l is the distance between the chassis attachment and the
## swingarm attachment; F = -k (l - l_0) - c l_dot acts along the instantaneous line between them. Attachment points are WORLD
## points at the reference pose; the builder converts each into its body's local frame. They need NOT lie on any body axis.

## Attachment point fixed in the chassis, world coordinates at the reference pose.
var chassis_attachment: Vector3 = Vector3.ZERO
## Attachment point fixed in the swingarm, world coordinates at the reference pose.
var swingarm_attachment: Vector3 = Vector3.ZERO
## N/m, finite, >= 0.
var stiffness: float = 0.0
## N s/m, finite, >= 0.
var damping: float = 0.0
## m, finite, > 0: the spring is unloaded at this attachment separation (explicit, independent of the reference pose).
var rest_length: float = 0.0
