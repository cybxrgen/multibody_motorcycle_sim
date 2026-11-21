extends RefCounted
## Specification of ONE rigid body of the motorcycle assembly: everything SimWorld needs to create it.
## Plain data; scene-tree independent. Validation lives in MotorcycleConfig.validate() (single home of the rules).
##
## Frames (PHYSICS_CONVENTIONS.md): `position` is the center of mass in world coordinates at the reference pose,
## `orientation` rotates local -> world, `local_inertia` is the full 3x3 tensor ABOUT THE COM in the body frame.
## Nothing here is derived: the builder converts the world-space joint geometry into body-local hinge frames.

var body_name: String = ""
## kg. Must be finite and > 0: a body with mass <= 0 would be static, and every body of this assembly is dynamic.
var mass: float = 0.0
## kg m^2, about the COM, body frame. Validated with SimMassProperties.validate_inertia().
var local_inertia: Basis = Basis.IDENTITY
## World-space COM at the reference pose (m).
var position: Vector3 = Vector3.ZERO
## Local -> world unit quaternion at the reference pose.
var orientation: Quaternion = Quaternion.IDENTITY
## Full side lengths (m) of the box proxy in the body frame, centered on the COM. Presentation only; unused for
## wheels (their dimensions are the wheel radius / width fields of MotorcycleConfig).
var visual_size: Vector3 = Vector3.ZERO


func _init(p_name: String = "", p_mass: float = 0.0, p_local_inertia: Basis = Basis.IDENTITY, p_position: Vector3 = Vector3.ZERO,
		p_orientation: Quaternion = Quaternion.IDENTITY, p_visual_size: Vector3 = Vector3.ZERO) -> void:
	body_name = p_name
	mass = p_mass
	local_inertia = p_local_inertia
	position = p_position
	orientation = p_orientation
	visual_size = p_visual_size
