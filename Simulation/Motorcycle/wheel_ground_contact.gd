extends "res://Simulation/Solver/contact_constraint.gd"
## One-point contact between a static ground plane (body A) and a wheel (body B), solved by the generic ContactConstraint.
## Motorcycle-level component (M2.4.1): the generic solver is unchanged; this class only SUPPLIES THE GEOMETRY and decides
## whether the contact is active. Equations and conventions: PHYSICS_CONVENTIONS.md ("Ground plane and wheel contact").
##
## ---- Geometry (valid operating domain) ----
## The wheel is approximated by a thin circle of radius R on its tyre centreline, centred on the COM, in the plane perpendicular to
## the axle u (the wheel's local X axis in world coordinates, u = q * (1,0,0)). For the unit plane normal n (pointing away from the
## ground) the lowest point of that circle is  p_w = x - R d,  d = (n - (n.u) u) / |n - (n.u) u|  (the direction of n in the wheel plane).
## The plane point below it is p_g = p_w - s n with s = n . (p_w - plane_point) (the signed gap). The contact normal, from A (ground)
## toward B (wheel), is +n, so the ContactConstraint gap n.(p_b - p_a) equals s.
## Approximation: zero tyre width and no tyre-section radius. For a camber angle the true lowest point lies on the tyre shoulder, up
## to w/2 sin(camber) away; this is ignored (one point per wheel; no contact patch). Domain: |n - (n.u) u| = sin(angle between axle and
## normal) must be >= MIN_PLANE_PROJECTION (0.1, i.e. the axle within about 84 degrees of the plane, camber up to ~84 degrees from
## upright). Below it the wheel lies almost flat, its lowest point is not defined by this model, and the contact is reported inactive
## (geometry_valid = false) instead of normalising a vanishing vector.
##
## ---- Lifecycle (every fixed step) ----
## prepare() is called by the solver once per fixed step from the start-of-step poses. It re-derives the geometry, re-seats the
## local anchors and the local normal (so the contact point migrates around the wheel as it rolls), and then lets the generic
## ContactConstraint build its rows. A contact that is invalid or clearly separated (gap > activation_distance + approach_speed * dt)
## is INACTIVE: its carried impulses (physical normal, friction) are discarded and it adds no impulse; when the wheel returns
## the contact starts cold. Nothing is mutated outside the solver's step.

const MIN_PLANE_PROJECTION: float = 0.1
## Wheel axle in the wheel's own frame (PHYSICS_CONVENTIONS.md: wheel axle = body-local X).
const WHEEL_AXLE_LOCAL: Vector3 = Vector3.RIGHT
## A wheel with gap <= this (penetration included) is GEOMETRICALLY touching: 0.1 mm. contact_state() reports it, and TireGroundContact
## (M2.4.2.1) lets the tyre act only then. The generic contact rows (speculative normal, split correction, Coulomb) never use it.
const TOUCH_TOLERANCE: float = 1.0e-4

const STATE_SEPARATED: String = "separated"
const STATE_APPROACHING: String = "approaching"
const STATE_TOUCHING: String = "touching"
const STATE_PENETRATING: String = "penetrating"
const STATE_SEPARATING: String = "separating"
const STATE_INACTIVE: String = "inactive"

var wheel_radius: float = 0.0
## A point on the plane and its unit normal (world frame; the plane is static).
var plane_point: Vector3 = Vector3.ZERO
var plane_normal: Vector3 = Vector3.UP
var activation_distance: float = 0.05

## True when the last refresh found valid geometry and the contact took part in the last prepare().
var contact_active: bool = false
var geometry_valid: bool = false
## Number of geometry refreshes (one per fixed step the contact was prepared, plus explicit refresh() calls).
var refresh_count: int = 0


func _init(p_ground: SimRigidBody, p_wheel: SimRigidBody, p_local_anchor_a: Vector3, p_local_anchor_b: Vector3, p_local_normal_a: Vector3,
		p_friction: float, p_wheel_radius: float, p_plane_point: Vector3, p_plane_normal: Vector3, p_activation_distance: float) -> void:
	# The wheel/plane data is set BEFORE the generic constructor validates the contact (validate() checks it).
	wheel_radius = p_wheel_radius
	plane_point = p_plane_point
	plane_normal = p_plane_normal
	activation_distance = p_activation_distance
	super(p_ground, p_wheel, p_local_anchor_a, p_local_anchor_b, p_local_normal_a, p_friction)


## Builds a contact from the wheel's CURRENT pose. Returns null (with an error) for invalid input or degenerate geometry.
## `ground` must be static, `wheel` dynamic; `p_plane_normal` is normalised (it must be unit length within DIRECTION_TOLERANCE).
static func create(ground: SimRigidBody, wheel: SimRigidBody, p_wheel_radius: float, p_plane_point: Vector3, p_plane_normal: Vector3,
		p_friction: float, p_activation_distance: float = 0.05):
	if ground == null or wheel == null:
		push_error("WheelGroundContact.create: bodies must not be null.")
		return null
	var errors: PackedStringArray = input_errors(p_wheel_radius, p_plane_point, p_plane_normal, p_activation_distance)
	if not errors.is_empty():
		push_error("WheelGroundContact.create: %s" % "; ".join(errors))
		return null
	var n: Vector3 = p_plane_normal.normalized()
	var geometry: Dictionary = compute_geometry(wheel.position, wheel.orientation, p_wheel_radius, p_plane_point, n)
	if not geometry.valid:
		push_error("WheelGroundContact.create: %s" % geometry.reason)
		return null
	var inv_a: Quaternion = ground.orientation.inverse()
	var inv_b: Quaternion = wheel.orientation.inverse()
	var contact = (load("res://Simulation/Motorcycle/wheel_ground_contact.gd")).new(ground, wheel,
		inv_a * (geometry.ground_point - ground.position), inv_b * (geometry.wheel_point - wheel.position), inv_a * n, p_friction,
		p_wheel_radius, p_plane_point, n, p_activation_distance)
	contact.geometry_valid = true
	return contact if contact.is_valid() else null


## Rules for the plane and wheel data (shared with MotorcycleConfig by value, repeated here for direct use).
static func input_errors(p_wheel_radius: float, p_plane_point: Vector3, p_plane_normal: Vector3, p_activation_distance: float) -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if not (is_finite(p_wheel_radius) and p_wheel_radius > 0.0):
		errors.append("wheel radius must be finite and > 0 (got %s)" % p_wheel_radius)
	if not p_plane_point.is_finite():
		errors.append("plane point must be finite (got %s)" % p_plane_point)
	if not p_plane_normal.is_finite():
		errors.append("plane normal must be finite (got %s)" % p_plane_normal)
	elif absf(p_plane_normal.length() - 1.0) > DIRECTION_TOLERANCE:
		errors.append("plane normal must be a unit vector (length %s)" % p_plane_normal.length())
	if not (is_finite(p_activation_distance) and p_activation_distance >= 0.0):
		errors.append("activation distance must be finite and >= 0 (got %s)" % p_activation_distance)
	return errors


## Pure geometry of one wheel against a plane. Returns {valid, reason, axle, wheel_point, ground_point, normal, gap}.
## `plane_unit_normal` must be a unit vector; nothing here normalises a vector that can vanish: a degenerate axle/normal
## combination (|n - (n.u) u| < MIN_PLANE_PROJECTION) or non-finite input gives valid = false and a reason.
static func compute_geometry(wheel_position: Vector3, wheel_orientation: Quaternion, radius: float, p_plane_point: Vector3, plane_unit_normal: Vector3) -> Dictionary:
	var result: Dictionary = {"valid": false, "reason": "", "axle": Vector3.ZERO, "wheel_point": Vector3.ZERO, "ground_point": Vector3.ZERO,
		"normal": plane_unit_normal, "gap": 0.0}
	if not (wheel_position.is_finite() and wheel_orientation.is_finite() and p_plane_point.is_finite() and plane_unit_normal.is_finite() and is_finite(radius)):
		result.reason = "non-finite wheel pose or plane data"
		return result
	var u: Vector3 = wheel_orientation * WHEEL_AXLE_LOCAL
	var in_plane: Vector3 = plane_unit_normal - u * plane_unit_normal.dot(u)
	var length: float = in_plane.length()
	result.axle = u
	if not (length >= MIN_PLANE_PROJECTION):
		result.reason = "degenerate wheel geometry: the axle is nearly parallel to the plane normal (|n - (n.u)u| = %s < %s)" % [length, MIN_PLANE_PROJECTION]
		return result
	var wheel_point: Vector3 = wheel_position - (in_plane / length) * radius
	var gap: float = plane_unit_normal.dot(wheel_point - p_plane_point)
	var ground_point: Vector3 = wheel_point - plane_unit_normal * gap
	if not (wheel_point.is_finite() and ground_point.is_finite() and is_finite(gap)):
		result.reason = "non-finite contact geometry"
		return result
	result.valid = true
	result.wheel_point = wheel_point
	result.ground_point = ground_point
	result.gap = gap
	return result


func validate() -> PackedStringArray:
	var errors: PackedStringArray = super.validate()
	if body_a != null and not body_a.is_static():
		errors.append("the ground body must be static")
	if body_b != null and body_b.is_static():
		errors.append("the wheel body must be dynamic")
	errors.append_array(input_errors(wheel_radius, plane_point, plane_normal, activation_distance))
	return errors


## Geometry from the bodies' CURRENT poses (valid flag, gap, points), without touching the constraint.
func current_geometry() -> Dictionary:
	return compute_geometry(body_b.position, body_b.orientation, wheel_radius, plane_point, plane_normal)


## Re-seats the local anchors and the local normal from the current poses. Returns false (anchors untouched, geometry_valid = false)
## for invalid or degenerate geometry. The solver calls it through prepare() at every fixed step.
func refresh() -> bool:
	refresh_count += 1
	var geometry: Dictionary = current_geometry()
	geometry_valid = geometry.valid
	if not geometry.valid:
		return false
	var inv_a: Quaternion = body_a.orientation.inverse()
	var inv_b: Quaternion = body_b.orientation.inverse()
	local_anchor_a = inv_a * (geometry.ground_point - body_a.position)
	local_anchor_b = inv_b * (geometry.wheel_point - body_b.position)
	local_normal_a = (inv_a * plane_normal).normalized()
	return true


## Signed gap of the wheel's lowest point above the plane from the current pose (m): > 0 separated, < 0 penetrating.
## NaN when the geometry is degenerate. Unlike gap() it does not use the anchors of the last step.
func signed_gap() -> float:
	var geometry: Dictionary = current_geometry()
	return geometry.gap if geometry.valid else NAN


## Normal velocity (m/s) of the wheel's lowest point relative to the plane, from the current velocities; > 0 separating.
func approach_velocity() -> float:
	var geometry: Dictionary = current_geometry()
	if not geometry.valid:
		return 0.0
	return plane_normal.dot(body_b.velocity_at_point(geometry.wheel_point) - body_a.velocity_at_point(geometry.ground_point))


## Contact situation from the current pose and velocities (diagnostic).
func contact_state() -> String:
	var geometry: Dictionary = current_geometry()
	if not geometry.valid:
		return STATE_INACTIVE
	var gap: float = geometry.gap
	var vn: float = plane_normal.dot(body_b.velocity_at_point(geometry.wheel_point) - body_a.velocity_at_point(geometry.ground_point))
	if gap < -TOUCH_TOLERANCE:
		return STATE_PENETRATING
	if gap <= TOUCH_TOLERANCE:
		return STATE_TOUCHING
	return STATE_APPROACHING if vn < 0.0 else (STATE_SEPARATING if vn > 0.0 else STATE_SEPARATED)


# --- Solver interface ---

func prepare(dt: float, config) -> void:
	var active: bool = refresh()
	if active:
		var geometry: Dictionary = current_geometry()
		var approach: float = maxf(0.0, -plane_normal.dot(body_b.velocity_at_point(geometry.wheel_point) - body_a.velocity_at_point(geometry.ground_point)))
		active = geometry.gap <= activation_distance + approach * dt
	if not active:
		contact_active = false
		reset_warm_start()
		for row in rows:
			row.effective_mass = 0.0
			row.split_enabled = false
		return
	contact_active = true
	_prepare_active(dt, config)


## Builds the rows of an ACTIVE contact (hook, M2.4.2). The default is the generic ContactConstraint preparation (normal row, generic
## Coulomb friction, warm start); TireGroundContact overrides it so that the tangential response belongs to the tyre model alone.
func _prepare_active(dt: float, config) -> void:
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
