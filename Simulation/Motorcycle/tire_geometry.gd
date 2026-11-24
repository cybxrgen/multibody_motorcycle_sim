extends RefCounted
## Finite-width tyre cross-section geometry (M2.5.1): plain data + validate() + pure static evaluation, scene-tree independent.
## One instance per tyre (front and rear are independent objects). Motorcycle-level; the generic core and solver do not know it.
## Equations and sign conventions: PHYSICS_CONVENTIONS.md ("Finite-width tyre geometry"). This component only describes SURFACE
## GEOMETRY. It adds no contact, force or constraint: the wheel-ground contact is still the one-point M2.4.1 contact (M2.5.2 will sample).
##
## ---- Profile ----
## Wheel frame: origin at the wheel COM, axle = local +X. A surface point is addressed by the lateral offset s along the axle
## (s in [-width/2, +width/2], + toward local +X) and by the direction e (unit, perpendicular to the axle) of its radius.
## The crown is a circular arc of radius rho (`crown_radius`) tangent to the centreline radius R (the wheel radius of the contact
## geometry) at s = 0:      r(s) = R - rho + sqrt(rho^2 - s^2)          (R at the centreline, smaller toward the shoulders)
## The shoulder drop is r(0) - r(w/2) = rho - sqrt(rho^2 - (w/2)^2); rho = w/2 is a semicircular section, a large rho a flat tread.
## Limits: symmetric about s = 0, circular crown only, no sidewall below the shoulder, no deformation, no measured/calibrated profile.
##
## ---- Lowest point against a plane (unit normal n, point p0) ----
## With a = n.u and L = |n - a u| = sqrt(1 - a^2) (L is the sine of the angle between axle and normal) the height of the surface point
## (s, e) is  h = n.(x - p0) + s a + r(s) (n.e).  Over e it is minimal for e = -(n - a u)/L, which gives
##   h(s) = h_c + a s - L r(s),   h_c = n.(x - p0)   (convex in s),
## and dh/ds = 0 gives s* = -rho a EXACTLY (independent of R and L). The lowest point has s = clamp(s*, -w/2, +w/2); a clamped
## point lies on a shoulder. Camber gamma (axle tilted by gamma from the plane) gives a = sin(gamma): the contact moves toward the
## LOWER end of the axle by rho sin(gamma) (until it reaches the shoulder) instead of staying at the centreline.
## Degenerate: L < MIN_PLANE_PROJECTION (axle within ~6 degrees of the normal): the lowest point is not defined, the result is invalid.

const MIN_PLANE_PROJECTION: float = 0.1
## Wheel axle in the wheel's own frame (the project convention: body-local X).
const WHEEL_AXLE_LOCAL: Vector3 = Vector3.RIGHT
## Admissible dimension ranges (m). The upper bounds keep every intermediate product (rho^2, ...) far from overflow.
const MAX_WIDTH: float = 0.5
const MAX_CROWN_RADIUS: float = 10.0
const MAX_WHEEL_RADIUS: float = 10.0
## The shoulder may be at most this fraction of the wheel radius below the centreline radius.
const MAX_SHOULDER_DROP_RATIO: float = 0.5
## |q| - 1 and |n| - 1 allowed for the pose quaternion and the plane normal (float32 rounding of a normalised value is ~6e-8).
const UNIT_TOLERANCE: float = 1.0e-5
## Slack when a requested lateral offset is compared with the half width.
const OFFSET_TOLERANCE: float = 1.0e-9

## Tyre width (m), measured along the axle between the two shoulders. Independent of the wheel's visual width and inertia width.
var width: float = 0.0
## Radius rho (m) of the circular crown in the cross-section, >= width / 2.
var crown_radius: float = 0.0


func _init(p_width: float = 0.0, p_crown_radius: float = 0.0) -> void:
	width = p_width
	crown_radius = p_crown_radius


## One message per violated rule; empty = valid. `wheel_radius` (m, the contact geometry radius of the wheel this tyre belongs to)
## adds the shoulder rule when it is > 0 and finite. This is the single home of the geometry rules.
func validate(wheel_radius: float = 0.0) -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if not (is_finite(width) and width > 0.0 and width <= MAX_WIDTH):
		errors.append("tire geometry: width must be finite and in (0, %s] m (got %s)" % [MAX_WIDTH, width])
	if not (is_finite(crown_radius) and crown_radius > 0.0 and crown_radius <= MAX_CROWN_RADIUS):
		errors.append("tire geometry: crown radius must be finite and in (0, %s] m (got %s)" % [MAX_CROWN_RADIUS, crown_radius])
	elif is_finite(width) and width > 0.0 and width <= MAX_WIDTH and crown_radius < 0.5 * width:
		errors.append("tire geometry: crown radius %s m must be >= half the width %s m (the arc must span the tread)" % [crown_radius, 0.5 * width])
	if errors.is_empty() and is_finite(wheel_radius) and wheel_radius > 0.0:
		if wheel_radius > MAX_WHEEL_RADIUS:
			errors.append("tire geometry: wheel radius must be <= %s m (got %s)" % [MAX_WHEEL_RADIUS, wheel_radius])
		elif shoulder_drop() > MAX_SHOULDER_DROP_RATIO * wheel_radius:
			errors.append("tire geometry: shoulder drop %s m exceeds %s of the wheel radius %s m" % [shoulder_drop(), MAX_SHOULDER_DROP_RATIO, wheel_radius])
	return errors


func is_valid(wheel_radius: float = 0.0) -> bool:
	return validate(wheel_radius).is_empty()


## An independent copy (callers may mutate it).
func duplicate_spec():
	return (load("res://Simulation/Motorcycle/tire_geometry.gd")).new(width, crown_radius)


## r(0) - r(w/2) (m); NaN when the dimensions are invalid.
func shoulder_drop() -> float:
	if not is_valid():
		return NAN
	return crown_radius - sqrt(maxf(crown_radius * crown_radius - 0.25 * width * width, 0.0))


## Radius r(s) of the surface at lateral offset s for centreline radius R. NaN for invalid dimensions or |s| > width / 2.
func profile_radius(wheel_radius: float, lateral_offset: float) -> float:
	if not (is_valid() and is_finite(wheel_radius) and wheel_radius > 0.0 and wheel_radius <= MAX_WHEEL_RADIUS):
		return NAN
	if not (is_finite(lateral_offset) and absf(lateral_offset) <= 0.5 * width + OFFSET_TOLERANCE):
		return NAN
	var s: float = clampf(lateral_offset, -0.5 * width, 0.5 * width)
	return wheel_radius - crown_radius + sqrt(maxf(crown_radius * crown_radius - s * s, 0.0))


## Lowest point of the tyre surface against a plane. Result keys (always present; meaningful only when valid):
##   valid, reason        - failure reason is non-empty exactly when valid is false
##   axle                 - world axle direction u (unit)
##   normal               - the plane normal n
##   wheel_point          - surface point, world frame
##   ground_point         - its projection onto the plane (wheel_point - gap n)
##   gap                  - signed distance n.(wheel_point - plane_point): > 0 above the plane, < 0 penetrating
##   lateral_offset       - s, metres along the axle from the centreline (+ toward local +X); in [-width/2, width/2]
##   lateral_fraction     - s / (width/2), in [-1, 1] (0 centreline, +-1 shoulder)
##   at_shoulder          - true when the unconstrained optimum -rho a lay beyond the shoulder and s was clamped to it
##   unclamped_offset     - -rho (n.u), the optimum of the unbounded circular crown
##   profile_radius       - r(s)
##   radial_direction     - unit world vector from the axle line to the point, perpendicular to the axle (e)
##   local_point          - wheel_point - wheel_position expressed in the wheel frame (orientation^-1 * ...)
## Nothing that can vanish is normalised: a degenerate axle/normal combination gives valid = false.
static func lowest_point(wheel_position: Vector3, wheel_orientation: Quaternion, geometry, wheel_radius: float,
		plane_point: Vector3, plane_unit_normal: Vector3) -> Dictionary:
	return _evaluate(wheel_position, wheel_orientation, geometry, wheel_radius, plane_point, plane_unit_normal, NAN)


## The lowest point of the single circle at lateral offset `lateral_offset` (|s| <= width/2): the building block of a later sampled
## patch. Same result keys as lowest_point(); `at_shoulder` is true for |s| = width/2 and `unclamped_offset` is still the global optimum.
static func point_at_offset(wheel_position: Vector3, wheel_orientation: Quaternion, geometry, wheel_radius: float,
		plane_point: Vector3, plane_unit_normal: Vector3, lateral_offset: float) -> Dictionary:
	if is_nan(lateral_offset):
		return _fail("lateral offset must not be NaN", plane_unit_normal)
	return _evaluate(wheel_position, wheel_orientation, geometry, wheel_radius, plane_point, plane_unit_normal, lateral_offset)


static func _fail(reason: String, normal: Vector3) -> Dictionary:
	return {"valid": false, "reason": reason, "axle": Vector3.ZERO, "normal": normal, "wheel_point": Vector3.ZERO, "ground_point": Vector3.ZERO,
		"gap": 0.0, "lateral_offset": 0.0, "lateral_fraction": 0.0, "at_shoulder": false, "unclamped_offset": 0.0, "profile_radius": 0.0,
		"radial_direction": Vector3.ZERO, "local_point": Vector3.ZERO}


static func _evaluate(wheel_position: Vector3, wheel_orientation: Quaternion, geometry, wheel_radius: float,
		plane_point: Vector3, n: Vector3, requested_offset: float) -> Dictionary:
	if geometry == null:
		return _fail("tire geometry is null", n)
	var errors: PackedStringArray = geometry.validate(wheel_radius)
	if not errors.is_empty():
		return _fail("; ".join(errors), n)
	if not (is_finite(wheel_radius) and wheel_radius > 0.0):
		return _fail("tire geometry: wheel radius must be finite and > 0 (got %s)" % wheel_radius, n)
	if not (wheel_position.is_finite() and wheel_orientation.is_finite() and plane_point.is_finite() and n.is_finite()):
		return _fail("non-finite wheel pose or plane data", n)
	var q_length: float = wheel_orientation.length()
	if not (absf(q_length - 1.0) <= UNIT_TOLERANCE):
		return _fail("wheel orientation must be a unit quaternion (length %s)" % q_length, n)
	var n_length: float = n.length()
	if not (absf(n_length - 1.0) <= UNIT_TOLERANCE):
		return _fail("plane normal must be a unit vector (length %s)" % n_length, n)

	var u: Vector3 = wheel_orientation * WHEEL_AXLE_LOCAL
	var a: float = n.dot(u)
	var in_plane: Vector3 = n - u * a
	var length: float = in_plane.length()
	if not (length >= MIN_PLANE_PROJECTION):
		var result: Dictionary = _fail("degenerate wheel geometry: the axle is nearly parallel to the plane normal (|n - (n.u)u| = %s < %s)" % [length, MIN_PLANE_PROJECTION], n)
		result.axle = u
		return result

	var half: float = 0.5 * geometry.width
	var unclamped: float = -geometry.crown_radius * a
	var s: float = clampf(unclamped, -half, half)
	var at_shoulder: bool = absf(unclamped) > half
	if not is_nan(requested_offset):
		if not (is_finite(requested_offset) and absf(requested_offset) <= half + OFFSET_TOLERANCE):
			var bad: Dictionary = _fail("lateral offset %s is outside the tread [-%s, %s]" % [requested_offset, half, half], n)
			bad.axle = u
			return bad
		s = clampf(requested_offset, -half, half)
		at_shoulder = absf(s) >= half - OFFSET_TOLERANCE
	var radius_at: float = wheel_radius - geometry.crown_radius + sqrt(maxf(geometry.crown_radius * geometry.crown_radius - s * s, 0.0))
	var e: Vector3 = -(in_plane / length)
	var point: Vector3 = wheel_position + u * s + e * radius_at
	var gap: float = n.dot(point - plane_point)
	var ground_point: Vector3 = point - n * gap
	var local_point: Vector3 = wheel_orientation.inverse() * (point - wheel_position)
	if not (point.is_finite() and ground_point.is_finite() and is_finite(gap) and local_point.is_finite() and is_finite(radius_at)):
		var overflow: Dictionary = _fail("non-finite contact geometry (the pose or plane data is too large for a finite result)", n)
		overflow.axle = u
		return overflow
	return {"valid": true, "reason": "", "axle": u, "normal": n, "wheel_point": point, "ground_point": ground_point, "gap": gap,
		"lateral_offset": s, "lateral_fraction": s / half, "at_shoulder": at_shoulder, "unclamped_offset": unclamped,
		"profile_radius": radius_at, "radial_direction": e, "local_point": local_point}
