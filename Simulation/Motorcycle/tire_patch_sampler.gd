extends RefCounted
## M2.5.3 Stage A: deterministic sampling of the finite-width tyre surface (geometry only).
## One pure static function, scene-tree independent, no state. It adds no contact, force or constraint; Stage B will
## use these samples for the compliant normal support. The surface is taken from `TireGeometry` (profile radius and
## crown radius); nothing about the profile is re-derived here except the parameterisation of the circular crown.
##
## ---- Parameterisation (exact, no approximation of the profile) ----
## Wheel frame: axle u = q * X (+X local), radial frame b1 = q * Y, b2 = q * Z; radial direction e(theta) = cos(theta) b1 + sin(theta) b2.
## Crown centre sits at radial distance R - rho on each meridian (R = wheel radius, rho = crown radius). Every meridian is a
## circular arc of radius rho with angle phi:  s = rho sin(phi),  r = R - rho + rho cos(phi)  (identical to TireGeometry's
## profile_radius(R, s) because sqrt(rho^2 - s^2) = rho cos(phi)). The surface point is  x(phi, theta) = p + u s + e r.
##
## ---- Normal (exact) ----
## Outward unit normal of the circle, which is also the surface normal: n = cos(phi) e + sin(phi) u. It is finite at the shoulders
## (no slope singularity: the crown is parameterised by phi, not by s), including the semicircular limit rho = w/2.
##
## ---- Area weights (quadrature) ----
## Surface area element dA = r * rho * dphi * dtheta exactly (|x_phi x x_theta| = rho r). The grid is a trapezoid rule over phi
## (end nodes at the shoulders with half weight, so the shoulders are included) times a uniform periodic rule over theta.
## Each sample carries weight = rho r(phi) * dphi_i * dtheta, in m^2: the area it represents. The sum of the weights converges to the
## exact tread area 2 pi rho ( 2 (R - rho) asin(w / 2 rho) + w ) as the grid is refined (Stage A tests). The area weights are the
## basis of the sample-density independent normal support of Stage B; they are NOT yet used by any force.
##
## ---- Pose and plane ----
## Samples are world-space points. `gap` is the signed distance of the sample from the plane (n . (x - p0)), >0 above, <0 penetrating.
## Degenerate orientations (axle parallel to the plane normal) are NOT an error here: the sampled surface and its gaps are well
## defined for every unit orientation (only TireGeometry.lowest_point() needs a non-degenerate axle). Invalid geometry, counts,
## non-finite or non-unit inputs fail with valid = false and no samples.

const TireGeometry = preload("res://Simulation/Motorcycle/tire_geometry.gd")

## Bounds on the sampling grid (bounded, deterministic cost: at most 64 * 256 samples per tyre).
const MIN_LATERAL_COUNT: int = 2
const MAX_LATERAL_COUNT: int = 64
const MIN_CIRCUMFERENTIAL_COUNT: int = 3
const MAX_CIRCUMFERENTIAL_COUNT: int = 256
## |q| - 1 and |n| - 1 allowed for the pose quaternion and the plane normal (float32 rounding is ~6e-8). Same as TireGeometry.
const UNIT_TOLERANCE: float = 1.0e-5
const AXLE_LOCAL: Vector3 = Vector3(1.0, 0.0, 0.0)
const RADIAL_Y_LOCAL: Vector3 = Vector3(0.0, 1.0, 0.0)
const RADIAL_Z_LOCAL: Vector3 = Vector3(0.0, 0.0, 1.0)


## One message per violated rule; empty = valid counts.
static func validate_counts(lateral_count: int, circumferential_count: int) -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if lateral_count < MIN_LATERAL_COUNT or lateral_count > MAX_LATERAL_COUNT:
		errors.append("tire patch: lateral sample count must be in [%d, %d] (got %d)" % [MIN_LATERAL_COUNT, MAX_LATERAL_COUNT, lateral_count])
	if circumferential_count < MIN_CIRCUMFERENTIAL_COUNT or circumferential_count > MAX_CIRCUMFERENTIAL_COUNT:
		errors.append("tire patch: circumferential sample count must be in [%d, %d] (got %d)"
				% [MIN_CIRCUMFERENTIAL_COUNT, MAX_CIRCUMFERENTIAL_COUNT, circumferential_count])
	return errors


## Unit radial direction e(theta) of the meridian at angle theta (rad) for wheel orientation q.
static func radial_direction(wheel_orientation: Quaternion, theta: float) -> Vector3:
	return (wheel_orientation * RADIAL_Y_LOCAL) * cos(theta) + (wheel_orientation * RADIAL_Z_LOCAL) * sin(theta)


## World point of the tyre surface at lateral offset s (|s| <= w/2) and angle theta. NaN vector for invalid geometry or offset.
static func surface_point(wheel_position: Vector3, wheel_orientation: Quaternion, geometry, wheel_radius: float, s: float, theta: float) -> Vector3:
	if geometry == null or not geometry.is_valid(wheel_radius):
		return Vector3(NAN, NAN, NAN)
	var r: float = geometry.profile_radius(wheel_radius, s)
	if is_nan(r):
		return Vector3(NAN, NAN, NAN)
	var u: Vector3 = wheel_orientation * AXLE_LOCAL
	return wheel_position + u * s + radial_direction(wheel_orientation, theta) * r


## Outward unit normal of the meridian circle at lateral offset s: (sqrt(rho^2 - s^2) e + s u) / rho. Finite for |s| <= rho.
static func surface_normal(wheel_orientation: Quaternion, geometry, s: float, theta: float) -> Vector3:
	var rho: float = geometry.crown_radius
	var u: Vector3 = wheel_orientation * AXLE_LOCAL
	var e: Vector3 = radial_direction(wheel_orientation, theta)
	return (e * sqrt(maxf(rho * rho - s * s, 0.0)) + u * s) / rho


## Samples the tyre surface on a lateral_count x circumferential_count grid. Result keys:
##   valid, reason        - reason is non-empty exactly when valid is false
##   samples              - Array of Dictionary, ordered lateral-major (index = i * circumferential_count + j), empty when invalid
##   lateral_count, circumferential_count, total_area (sum of the sample weights, m^2)
## Each sample: lateral_index, angular_index, lateral_offset (s, m), angle (theta, rad), position (world), normal (world, unit),
## radius (r(s), m), weight (m^2), gap (signed distance from the plane, m).
static func sample(wheel_position: Vector3, wheel_orientation: Quaternion, geometry, wheel_radius: float,
		plane_point: Vector3, plane_unit_normal: Vector3, lateral_count: int, circumferential_count: int) -> Dictionary:
	var counts_errors: PackedStringArray = validate_counts(lateral_count, circumferential_count)
	if not counts_errors.is_empty():
		return _fail("; ".join(counts_errors))
	if geometry == null:
		return _fail("tire geometry is null")
	var geometry_errors: PackedStringArray = geometry.validate(wheel_radius)
	if not geometry_errors.is_empty():
		return _fail("; ".join(geometry_errors))
	if not (is_finite(wheel_radius) and wheel_radius > 0.0):
		return _fail("tire geometry: wheel radius must be finite and > 0 (got %s)" % wheel_radius)
	if not (wheel_position.is_finite() and wheel_orientation.is_finite() and plane_point.is_finite() and plane_unit_normal.is_finite()):
		return _fail("non-finite wheel pose or plane data")
	var q_length: float = wheel_orientation.length()
	if not (absf(q_length - 1.0) <= UNIT_TOLERANCE):
		return _fail("wheel orientation must be a unit quaternion (length %s)" % q_length)
	var n_length: float = plane_unit_normal.length()
	if not (absf(n_length - 1.0) <= UNIT_TOLERANCE):
		return _fail("plane normal must be a unit vector (length %s)" % n_length)

	var rho: float = geometry.crown_radius
	var half: float = 0.5 * geometry.width
	var phi_max: float = asin(clampf(half / rho, -1.0, 1.0))
	var u: Vector3 = wheel_orientation * AXLE_LOCAL
	var samples: Array = []
	var total_area: float = 0.0
	var dtheta: float = TAU / float(circumferential_count)
	var last: int = lateral_count - 1
	for i in lateral_count:
		var is_end: bool = i == 0 or i == last
		var phi: float = -phi_max + 2.0 * phi_max * float(i) / float(last)
		var s: float = clampf(rho * sin(phi), -half, half)
		if i == 0:
			s = -half
		elif i == last:
			s = half
		var r: float = geometry.profile_radius(wheel_radius, s)
		if not (is_finite(r) and r > 0.0):
			return _fail("tyre profile radius %s m is not positive and finite at lateral offset %s" % [r, s])
		# Trapezoid weight over phi (half at the shoulders, so the shoulder nodes are included exactly once).
		var dphi: float = 2.0 * phi_max / float(last) * (0.5 if is_end else 1.0)
		var weight_lateral: float = rho * r * dphi
		for j in circumferential_count:
			var theta: float = dtheta * float(j)
			var e: Vector3 = radial_direction(wheel_orientation, theta)
			var point: Vector3 = wheel_position + u * s + e * r
			var normal: Vector3 = surface_normal(wheel_orientation, geometry, s, theta)
			var weight: float = weight_lateral * dtheta
			# The gap is a subtraction of two finite points and can overflow even when both are finite: check it before append.
			var gap: float = plane_unit_normal.dot(point - plane_point)
			if not (point.is_finite() and normal.is_finite() and is_finite(weight) and is_finite(gap)):
				return _fail("non-finite sample geometry or signed gap (the pose, plane or tyre data is too large for a finite result)")
			samples.append({"lateral_index": i, "angular_index": j, "lateral_offset": s, "angle": theta, "position": point,
				"normal": normal, "radius": r, "weight": weight, "gap": gap})
			total_area += weight
	return {"valid": true, "reason": "", "samples": samples, "lateral_count": lateral_count,
		"circumferential_count": circumferential_count, "total_area": total_area}


static func _fail(reason: String) -> Dictionary:
	return {"valid": false, "reason": reason, "samples": [], "lateral_count": 0, "circumferential_count": 0, "total_area": 0.0}
