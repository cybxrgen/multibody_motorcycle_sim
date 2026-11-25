extends "res://Simulation/Solver/contact_constraint.gd"
## One corner of a structural collision box against the static ground plane (M2.5.2), solved by the generic ContactConstraint
## (unilateral normal + isotropic Coulomb friction). Motorcycle-level component: the generic solver is unchanged; this class only SUPPLIES
## THE GEOMETRY of one corner and decides whether that corner takes part. Wheels never get one (TireGroundContact is their sole
## ground-contact authority).
##
## ---- Contact slots, selection and reduction (deterministic) ----
## A box owns EIGHT persistent slots, one per corner k = 0..7 (MotorcycleCollisionBox corner order); the builder registers them in that
## order. A slot's identity never changes, so the contact set is a pure function of the pose and velocity (no tie-breaking, no ordering
## that could swap between frames). Every fixed step each slot re-derives its corner from the start-of-step pose:
##   corner world point  p = x + R (c + Rb s h)       gap g = n . (p - plane_point)       ground point = p - g n
## and is ACTIVE iff the corner is finite and  g <= activation_distance + approach * dt  (approach = max(0, -n . v_rel) of the corner;
## the speculative criterion of WheelGroundContact). Only the corners that are at/below the threshold are therefore ever active; the
## upper corners of a box standing above the ground take no part. Coplanar corners (a face on the plane) are deliberately ALL kept: they
## are independent support points that resist tipping (an edge would let the box rock), and the count is bounded by the 8 corners.
## When the geometry is unchanged the same corners are selected at every step.
##
## ---- Lifecycle ----
## prepare() (once per fixed step, from the solver) re-seats the local anchors and the local normal, then lets the generic contact
## build its rows. A slot that is inactive (separated, non-finite or invalid geometry) DISCARDS its carried impulses (reset_warm_start())
## and adds no impulse, so a re-contact starts cold; no stale impulse survives a separation. Removing the body removes its slots
## (SimWorld.remove_body); invalid geometry makes validate() report errors, so SimWorld refuses to step instead of simulating it.
## The box's friction coefficient is re-read each step.

## A corner with gap <= this (penetration included) is GEOMETRICALLY touching (0.1 mm; the wheel contacts use the same value).
const TOUCH_TOLERANCE: float = 1.0e-4
## Corners of a box (MotorcycleCollisionBox.CORNER_COUNT).
const CORNER_COUNT: int = 8

var box = null
var corner_index: int = 0
var plane_point: Vector3 = Vector3.ZERO
var plane_normal: Vector3 = Vector3.UP
var activation_distance: float = 0.05

var contact_active: bool = false
var geometry_valid: bool = false
var refresh_count: int = 0


func _init(p_ground: SimRigidBody, p_body: SimRigidBody, p_box, p_corner_index: int, p_plane_point: Vector3, p_plane_normal: Vector3,
		p_activation_distance: float, p_local_anchor_a: Vector3, p_local_anchor_b: Vector3, p_local_normal_a: Vector3) -> void:
	box = p_box  # set BEFORE the generic constructor validates the contact (validate() checks it)
	corner_index = p_corner_index
	plane_point = p_plane_point
	plane_normal = p_plane_normal
	activation_distance = p_activation_distance
	super(p_ground, p_body, p_local_anchor_a, p_local_anchor_b, p_local_normal_a, p_box.friction if p_box != null else 0.0)


## Builds the slot of corner `p_corner_index` from the body's CURRENT pose. `p_box` is the SAME object the wireframe draws (not a copy).
## Returns null (with an error) for invalid input.
static func create(ground: SimRigidBody, body: SimRigidBody, p_box, p_corner_index: int, p_plane_point: Vector3, p_plane_normal: Vector3,
		p_activation_distance: float = 0.05):
	if ground == null or body == null or p_box == null:
		push_error("BoxGroundContact.create: ground, body and box must not be null.")
		return null
	var errors: PackedStringArray = input_errors(p_box, p_corner_index, p_plane_point, p_plane_normal, p_activation_distance)
	if not errors.is_empty():
		push_error("BoxGroundContact.create: %s" % "; ".join(errors))
		return null
	var n: Vector3 = p_plane_normal.normalized()
	var geometry: Dictionary = corner_geometry(p_box, p_corner_index, body.position, body.orientation, p_plane_point, n)
	if not geometry.valid:
		push_error("BoxGroundContact.create: %s" % geometry.reason)
		return null
	var inv_a: Quaternion = ground.orientation.inverse()
	var inv_b: Quaternion = body.orientation.inverse()
	var contact = (load("res://Simulation/Motorcycle/box_ground_contact.gd")).new(ground, body, p_box, p_corner_index, p_plane_point, n, p_activation_distance,
		inv_a * (geometry.ground_point - ground.position), inv_b * (geometry.corner_point - body.position), inv_a * n)
	contact.geometry_valid = true
	return contact if contact.is_valid() else null


static func input_errors(p_box, p_corner_index: int, p_plane_point: Vector3, p_plane_normal: Vector3, p_activation_distance: float) -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if p_box == null:
		errors.append("collision box must not be null")
	else:
		errors.append_array(p_box.validate())
	if p_corner_index < 0 or p_corner_index >= 8:
		errors.append("corner index must be 0..7 (got %s)" % p_corner_index)
	if not p_plane_point.is_finite():
		errors.append("plane point must be finite (got %s)" % p_plane_point)
	if not p_plane_normal.is_finite():
		errors.append("plane normal must be finite (got %s)" % p_plane_normal)
	elif absf(p_plane_normal.length() - 1.0) > DIRECTION_TOLERANCE:
		errors.append("plane normal must be a unit vector (length %s)" % p_plane_normal.length())
	if not (is_finite(p_activation_distance) and p_activation_distance >= 0.0):
		errors.append("activation distance must be finite and >= 0 (got %s)" % p_activation_distance)
	return errors


## Pure geometry of one box corner against a plane for a body pose. Returns {valid, reason, corner_point, ground_point, gap}.
static func corner_geometry(p_box, p_corner_index: int, body_position: Vector3, body_orientation: Quaternion, p_plane_point: Vector3, plane_unit_normal: Vector3) -> Dictionary:
	var result: Dictionary = {"valid": false, "reason": "", "corner_point": Vector3.ZERO, "ground_point": Vector3.ZERO, "gap": 0.0}
	if not (body_position.is_finite() and body_orientation.is_finite() and p_plane_point.is_finite() and plane_unit_normal.is_finite()):
		result.reason = "non-finite body pose or plane data"
		return result
	var corner: Vector3 = body_position + body_orientation * p_box.corner_local(p_corner_index)
	var gap: float = plane_unit_normal.dot(corner - p_plane_point)
	var ground_point: Vector3 = corner - plane_unit_normal * gap
	if not (corner.is_finite() and ground_point.is_finite() and is_finite(gap)):
		result.reason = "non-finite transformed corner"
		return result
	result.valid = true
	result.corner_point = corner
	result.ground_point = ground_point
	result.gap = gap
	return result


func validate() -> PackedStringArray:
	var errors: PackedStringArray = super.validate()
	if body_a != null and not body_a.is_static():
		errors.append("the ground body must be static")
	if body_b != null and body_b.is_static():
		errors.append("the box body must be dynamic")
	errors.append_array(input_errors(box, corner_index, plane_point, plane_normal, activation_distance))
	return errors


func current_geometry() -> Dictionary:
	return corner_geometry(box, corner_index, body_b.position, body_b.orientation, plane_point, plane_normal)


## Signed gap of this corner above the plane from the current pose (m); NaN when the geometry is invalid.
func signed_gap() -> float:
	var geometry: Dictionary = current_geometry()
	return geometry.gap if geometry.valid else NAN


## True when the corner is geometrically touching (gap <= TOUCH_TOLERANCE, penetration included) from the current pose.
func is_touching() -> bool:
	var gap: float = signed_gap()
	return not is_nan(gap) and gap <= TOUCH_TOLERANCE


## Re-seats the anchors and the normal from the current pose. false (anchors untouched, geometry_valid = false) for invalid geometry.
func refresh() -> bool:
	refresh_count += 1
	var geometry: Dictionary = current_geometry()
	geometry_valid = geometry.valid
	if not geometry.valid:
		return false
	var inv_a: Quaternion = body_a.orientation.inverse()
	var inv_b: Quaternion = body_b.orientation.inverse()
	local_anchor_a = inv_a * (geometry.ground_point - body_a.position)
	local_anchor_b = inv_b * (geometry.corner_point - body_b.position)
	local_normal_a = (inv_a * plane_normal).normalized()
	friction = box.friction
	return true


func prepare(dt: float, config) -> void:
	var active: bool = box != null and box.is_valid() and refresh()
	if active:
		var geometry: Dictionary = current_geometry()
		var approach: float = maxf(0.0, -plane_normal.dot(body_b.velocity_at_point(geometry.corner_point) - body_a.velocity_at_point(geometry.ground_point)))
		active = geometry.gap <= activation_distance + approach * dt
	if not active:
		contact_active = false
		reset_warm_start()
		for row in rows:
			row.effective_mass = 0.0
			row.split_enabled = false
		return
	contact_active = true
	super.prepare(dt, config)


func solve_iteration() -> void:
	if contact_active:
		super.solve_iteration()


func solve_split_iteration() -> void:
	if contact_active:
		super.solve_split_iteration()


func finish() -> void:
	if contact_active:
		super.finish()
