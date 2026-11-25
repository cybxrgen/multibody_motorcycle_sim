extends RefCounted
## Physical oriented-box collision geometry of ONE structural motorcycle body (M2.5.2). Plain data plus validate(); scene-tree
## independent. It is the SINGLE authoritative description of that body's physical extent: BoxGroundContact generates its ground contacts
## from it and the geometry wireframe (G) draws it. It is independent of the presentation mesh (MotorcycleBodySpec.visual_size is never
## used as a substitute) and of the wheels' tyre geometry (the wheels have no collision box).
##
## Frame: `center` and `orientation` are given in the OWNING BODY's frame (origin = body COM, orientation = body -> box rotation); `size`
## is the full side length along each box axis. Corner k (0..7) is  center + R * (sx * hx, sy * hy, sz * hz)  with
## sx = +1 if bit 0 of k is set else -1, sy from bit 1, sz from bit 2 (h = size / 2). The same ordering is used by the contact slots
## and the wireframe, so corner k means the same point everywhere.
##
## `friction` is the Coulomb coefficient of the box-ground pair (finite, >= 0).
## Nothing is clamped or repaired: validate() names every violated rule.

## The only bodies that can own a collision box: the four non-wheel bodies. A wheel can never own one (tyre contact is its sole ground
## contact authority).
const STRUCTURAL_BODIES: Array = ["chassis", "steering", "fork_slider", "swingarm"]
## | |q| - 1 | allowed for the box orientation (the bound SimWorld applies to stored orientations).
const ORIENTATION_NORM_TOLERANCE: float = 1.0e-5
const CORNER_COUNT: int = 8

## One of STRUCTURAL_BODIES.
var body_name: String = ""
var center: Vector3 = Vector3.ZERO
var orientation: Quaternion = Quaternion.IDENTITY
var size: Vector3 = Vector3.ZERO
var friction: float = 0.6


func _init(p_body_name: String = "", p_size: Vector3 = Vector3.ZERO, p_center: Vector3 = Vector3.ZERO,
		p_orientation: Quaternion = Quaternion.IDENTITY, p_friction: float = 0.6) -> void:
	body_name = p_body_name
	size = p_size
	center = p_center
	orientation = p_orientation
	friction = p_friction


## One message per violated rule; empty means valid.
func validate() -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	var label: String = "collision box of '%s'" % body_name
	if not (body_name in STRUCTURAL_BODIES):
		errors.append("%s: the owner must be one of %s (a wheel never owns a collision box)" % [label, STRUCTURAL_BODIES])
	if not (size.is_finite() and size.x > 0.0 and size.y > 0.0 and size.z > 0.0):
		errors.append("%s: size must be finite and > 0 on every axis (got %s)" % [label, size])
	if not center.is_finite():
		errors.append("%s: centre must be finite (got %s)" % [label, center])
	if not orientation.is_finite():
		errors.append("%s: orientation must be finite (got %s)" % [label, orientation])
	elif absf(orientation.length() - 1.0) > ORIENTATION_NORM_TOLERANCE:
		errors.append("%s: orientation must be a unit quaternion, not a degenerate one (|q| = %s)" % [label, orientation.length()])
	if not (is_finite(friction) and friction >= 0.0):
		errors.append("%s: friction must be finite and >= 0 (got %s)" % [label, friction])
	return errors


func is_valid() -> bool:
	return validate().is_empty()


func half_extents() -> Vector3:
	return size * 0.5


## Corner `k` (0..7) in the OWNING BODY's frame.
func corner_local(k: int) -> Vector3:
	var h: Vector3 = half_extents()
	var offset: Vector3 = Vector3(h.x * (1.0 if (k & 1) != 0 else -1.0), h.y * (1.0 if (k & 2) != 0 else -1.0), h.z * (1.0 if (k & 4) != 0 else -1.0))
	return center + orientation.normalized() * offset


## The eight corners in the owning body's frame, in corner order.
func corners_local() -> Array:
	var result: Array = []
	for k in CORNER_COUNT:
		result.append(corner_local(k))
	return result


## The eight corners in world space for a body pose (position of the COM, orientation body -> world).
func corners_world(body_position: Vector3, body_orientation: Quaternion) -> Array:
	var result: Array = []
	for k in CORNER_COUNT:
		result.append(body_position + body_orientation * corner_local(k))
	return result


## The 12 edges as 24 end points (line-list order), in the owning body's frame.
func edge_vertices() -> PackedVector3Array:
	var vertices: PackedVector3Array = PackedVector3Array()
	if not is_valid():
		return vertices
	var corners: Array = corners_local()
	for k in CORNER_COUNT:
		for bit in [1, 2, 4]:
			if (k & bit) == 0:
				vertices.append(corners[k])
				vertices.append(corners[k | bit])
	return vertices


## Dimension signature [size xyz, centre xyz, orientation xyzw]: changes whenever the physical geometry changes.
func signature() -> Array:
	return [size.x, size.y, size.z, center.x, center.y, center.z, orientation.x, orientation.y, orientation.z, orientation.w]


## True when the body-frame origin (the COM) lies inside the box (within `tolerance` m on every box axis).
func contains_body_origin(tolerance: float = 1.0e-6) -> bool:
	var p: Vector3 = orientation.normalized().inverse() * (-center)
	var h: Vector3 = half_extents()
	return absf(p.x) <= h.x + tolerance and absf(p.y) <= h.y + tolerance and absf(p.z) <= h.z + tolerance


func duplicate_spec():
	return (load("res://Simulation/Motorcycle/motorcycle_collision_box.gd")).new(body_name, size, center, orientation, friction)
