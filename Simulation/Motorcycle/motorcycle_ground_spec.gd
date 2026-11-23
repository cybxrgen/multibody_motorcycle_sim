extends RefCounted
## Specification of the flat, static ground plane (M2.4.1): plain data, scene-tree independent. Validation lives in
## MotorcycleConfig.validate() (single home of the rules); WheelGroundContact.validate() repeats the rules that concern it.
##
## Plane convention (PHYSICS_CONVENTIONS.md, "Ground plane and wheel contact"): the plane is { x : n . (x - point) = 0 } with `normal`
## the unit normal pointing AWAY from the ground into the free half-space. The signed distance of a world point x is
## s(x) = n . (x - point): s > 0 above the ground, s = 0 on it, s < 0 below it (penetrating).
## The ground body is a static SimRigidBody located at `point` with identity orientation.

## A world point on the plane (m).
var point: Vector3 = Vector3.ZERO
## Unit normal, pointing away from the ground (the prototype uses +Y).
var normal: Vector3 = Vector3.UP
## Coulomb friction coefficient mu of the ground contact, finite and >= 0 (dimensionless).
var friction: float = 0.0
## Distance (m, finite, >= 0) beyond which a wheel is treated as clearly separated: its contact is inactive and its carried
## impulses are discarded. A wheel that would still reach the plane within the step stays active (speculative contact).
var activation_distance: float = 0.05


func _init(p_point: Vector3 = Vector3.ZERO, p_normal: Vector3 = Vector3.UP, p_friction: float = 0.0, p_activation_distance: float = 0.05) -> void:
	point = p_point
	normal = p_normal
	friction = p_friction
	activation_distance = p_activation_distance
