extends "res://Tests/sim_test_case.gd"
## M2.5.1: finite-width tyre geometry (Simulation/Motorcycle/tire_geometry.gd) and its configuration/assembly integration.
##
## Every expectation is computed IN THIS FILE from the closed forms of the circular crown profile, written independently of the
## implementation (explicit coordinates of hand-built poses), or from a brute-force search over the whole tread surface
## (lateral offset x angle around the wheel) that uses none of the implementation's algebra. Tolerances: Godot's Vector3/Quaternion are
## float32 (about 6e-8 relative), a chain of a few operations on metre-sized values is good to ~2e-6, so vector/gap checks use 5e-6;
## the brute-force search has grid errors below 1e-5 (documented where it is used).

const TireGeometry = preload("res://Simulation/Motorcycle/tire_geometry.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")

const TOL: float = 5.0e-6
## Test tyre: width 0.18 m, crown radius 0.10 m, wheel radius 0.31 m.
const W: float = 0.18
const RHO: float = 0.10
const R: float = 0.31


func geo(width: float = W, crown: float = RHO):
	return TireGeometry.new(width, crown)


## Rotation of the wheel frame so that the axle (local +X) is (cos g, sin g, 0): a rotation about +Z by g. Positive g lifts the +X end.
func camber_q(g: float) -> Quaternion:
	return Quaternion(Vector3(0.0, 0.0, 1.0), g)


func radius_at(s: float, rho: float = RHO, r_wheel: float = R) -> float:
	return r_wheel - rho + sqrt(rho * rho - s * s)


## Independent brute force: minimum height over a grid of (lateral offset, angle about the axle). Returns {gap, s}.
func brute_force(position: Vector3, q: Quaternion, g, r_wheel: float, p0: Vector3, n: Vector3) -> Dictionary:
	var u: Vector3 = q * Vector3(1.0, 0.0, 0.0)
	var b1: Vector3 = q * Vector3(0.0, 1.0, 0.0)
	var b2: Vector3 = q * Vector3(0.0, 0.0, 1.0)
	var best: float = INF
	var best_s: float = 0.0
	var steps_s: int = 360
	var steps_t: int = 720
	for i in steps_s + 1:
		var s: float = -0.5 * g.width + g.width * float(i) / float(steps_s)
		var r: float = r_wheel - g.crown_radius + sqrt(maxf(g.crown_radius * g.crown_radius - s * s, 0.0))
		for j in steps_t:
			var t: float = TAU * float(j) / float(steps_t)
			var point: Vector3 = position + u * s + (b1 * cos(t) + b2 * sin(t)) * r
			var h: float = n.dot(point - p0)
			if h < best:
				best = h
				best_s = s
	return {"gap": best, "s": best_s}


# --- Profile and dimensions ---

func test_profile_values_at_centreline_shoulders_and_symmetry() -> void:
	var g = geo()
	check(g.is_valid() and g.is_valid(R), "the test tyre is valid")
	check_near(g.profile_radius(R, 0.0), R, 1.0e-12, "centreline radius equals the wheel radius")
	var shoulder: float = R - RHO + sqrt(RHO * RHO - 0.09 * 0.09)
	check_near(g.profile_radius(R, 0.5 * W), shoulder, 1.0e-12, "+ shoulder radius")
	check_near(g.profile_radius(R, -0.5 * W), shoulder, 1.0e-12, "- shoulder radius (symmetric)")
	check_near(shoulder, 0.31 - 0.1 + 0.043588989435, 1.0e-9, "shoulder radius against the hand value 0.253589")
	check_near(g.shoulder_drop(), 0.1 - 0.043588989435, 1.0e-9, "shoulder drop = rho - sqrt(rho^2 - (w/2)^2) = 0.0564110")
	for s in [0.01, 0.03, 0.05, 0.07, 0.09]:
		check_near(g.profile_radius(R, s), g.profile_radius(R, -s), 1.0e-12, "r(s) = r(-s) at %s" % s)
		check_near(g.profile_radius(R, s), radius_at(s), 1.0e-12, "r(s) matches the closed form at %s" % s)
		check(g.profile_radius(R, s) < g.profile_radius(R, 0.0), "the crown is the largest radius")
	check(is_nan(g.profile_radius(R, 0.5 * W + 1.0e-3)) and is_nan(g.profile_radius(R, -0.5 * W - 1.0e-3)), "outside the tread there is no surface")
	check(is_nan(g.profile_radius(R, NAN)) and is_nan(g.profile_radius(R, INF)), "non-finite offsets are refused")
	check_near(g.width, W, 0.0, "the configured width is kept")
	var semicircle = geo(0.2, 0.1)
	check_near(semicircle.shoulder_drop(), 0.1, 1.0e-12, "rho = w/2: a semicircular section drops by w/2")
	var flat = geo(0.2, 10.0)
	check(flat.shoulder_drop() < 1.1e-3, "a large crown radius is nearly flat")


func test_validation_rejects_invalid_and_degenerate_dimensions() -> void:
	check(geo(0.18, 0.10).validate().is_empty(), "valid")
	check(geo(0.18, 0.09).validate().is_empty(), "rho = w/2 is admissible")
	var bad_cases: Array = [
		[0.0, 0.1, "zero width"], [-0.1, 0.1, "negative width"], [NAN, 0.1, "NaN width"], [INF, 0.1, "INF width"],
		[0.1, 0.0, "zero crown radius"], [0.1, -0.1, "negative crown radius"], [0.1, NAN, "NaN crown radius"], [0.1, INF, "INF crown radius"],
		[0.2, 0.0999, "crown radius below half width"], [0.18, 0.089, "crown radius below half width (2)"],
		[1.0e30, 1.0e30, "huge finite width and crown (rho^2 would overflow)"], [1.0e200, 1.0e200, "float64-scale dimensions (rho^2 overflows)"],
		[0.6, 1.0, "width above the maximum"], [0.1, 11.0, "crown radius above the maximum"],
	]
	for case in bad_cases:
		var g = geo(case[0], case[1])
		check(not g.validate().is_empty() and not g.is_valid(), "rejected: %s" % case[2])
	check(not geo(0.18, 0.10).is_valid(0.1), "a shoulder drop above half the wheel radius is rejected (0.0564 > 0.05)")
	check(geo(0.18, 0.10).is_valid(0.12), "and accepted when it is below half (0.0564 <= 0.06)")
	check(not geo(0.18, 0.10).is_valid(20.0), "a wheel radius above the maximum is rejected")
	check(is_nan(geo(0.0, 0.1).shoulder_drop()), "an invalid geometry has no shoulder drop")
	var copy = geo().duplicate_spec()
	copy.width = 0.3
	check_near(geo().width, W, 0.0, "duplicate_spec is independent")
	check_near(copy.crown_radius, RHO, 0.0, "and carries the values")


# --- Orientation and camber ---

func test_upright_wheel_on_a_horizontal_plane() -> void:
	var r: Dictionary = TireGeometry.lowest_point(Vector3(0.0, 0.5, 0.0), Quaternion.IDENTITY, geo(), R, Vector3.ZERO, Vector3.UP)
	check(r.valid and r.reason == "", "valid, no reason")
	check_vec3_near(r.wheel_point, Vector3(0.0, 0.5 - R, 0.0), TOL, "lowest point is directly below the centre at the centreline radius")
	check_vec3_near(r.ground_point, Vector3(0.0, 0.0, 0.0), TOL, "its projection is on the plane")
	check_near(r.gap, 0.5 - R, TOL, "gap = height - R")
	check_near(r.lateral_offset, 0.0, TOL, "centreline contact")
	check_near(r.lateral_fraction, 0.0, TOL, "fraction 0")
	check(not r.at_shoulder, "not at a shoulder")
	check_near(r.profile_radius, R, TOL, "radius R")
	check_vec3_near(r.radial_direction, Vector3(0.0, -1.0, 0.0), TOL, "radial direction points down")
	check_vec3_near(r.local_point, Vector3(0.0, -R, 0.0), TOL, "local coordinates")
	check_vec3_near(r.axle, Vector3(1.0, 0.0, 0.0), TOL, "axle")
	# Same lowest point as the zero-width M2.4.1 contact (a = 0).
	var old: Dictionary = WheelGroundContact.compute_geometry(Vector3(0.0, 0.5, 0.0), Quaternion.IDENTITY, R, Vector3.ZERO, Vector3.UP)
	check_vec3_near(r.wheel_point, old.wheel_point, TOL, "agrees with the zero-width geometry when upright")
	check_near(r.gap, old.gap, TOL, "and the gap")


func test_camber_moves_the_contact_across_the_profile_in_closed_form() -> void:
	var g = geo()
	var yc: float = 0.4
	for deg in [-35.0, -20.0, -10.0, -2.0, 2.0, 10.0, 20.0, 35.0]:
		var gamma: float = deg_to_rad(deg)
		var sg: float = sin(gamma)
		var cg: float = cos(gamma)
		var r: Dictionary = TireGeometry.lowest_point(Vector3(0.0, yc, 0.0), camber_q(gamma), g, R, Vector3.ZERO, Vector3.UP)
		check(r.valid, "valid at %s deg" % deg)
		# Hand derivation: axle u = (cos g, sin g, 0), a = n.u = sin g, radial direction e = (sin g, -cos g, 0), s* = -rho sin g.
		var s: float = -RHO * sg
		var expected_point: Vector3 = Vector3(0.0, yc, 0.0) + Vector3(cg, sg, 0.0) * s + Vector3(sg, -cg, 0.0) * radius_at(s)
		check_near(r.lateral_offset, s, TOL, "offset = -rho sin(camber) at %s deg" % deg)
		check_vec3_near(r.wheel_point, expected_point, TOL, "world point at %s deg" % deg)
		check_near(r.gap, expected_point.y, TOL, "gap is the point height at %s deg" % deg)
		check_vec3_near(r.ground_point, Vector3(expected_point.x, 0.0, expected_point.z), TOL, "projection at %s deg" % deg)
		check(absf(r.lateral_offset) < absf(0.5 * W) and not r.at_shoulder, "inside the tread at %s deg" % deg)
		# Direction: toward the LOWER end of the axle, i.e. opposite to the tilt sign.
		check((r.lateral_offset < 0.0) == (deg > 0.0), "moves toward the lower axle end at %s deg" % deg)
	var r10: Dictionary = TireGeometry.lowest_point(Vector3(0.0, yc, 0.0), camber_q(deg_to_rad(10.0)), g, R, Vector3.ZERO, Vector3.UP)
	var r20: Dictionary = TireGeometry.lowest_point(Vector3(0.0, yc, 0.0), camber_q(deg_to_rad(20.0)), g, R, Vector3.ZERO, Vector3.UP)
	check(absf(r20.lateral_offset) > absf(r10.lateral_offset) + 0.01, "the contact migrates further as camber grows (not fixed at the centreline)")


func test_reversed_camber_is_the_mirror_image_for_a_symmetric_tire() -> void:
	var g = geo()
	for deg in [3.0, 12.0, 25.0, 40.0]:
		var p: Dictionary = TireGeometry.lowest_point(Vector3(0.3, 0.45, -0.2), camber_q(deg_to_rad(deg)), g, R, Vector3.ZERO, Vector3.UP)
		var m: Dictionary = TireGeometry.lowest_point(Vector3(-0.3, 0.45, -0.2), camber_q(-deg_to_rad(deg)), g, R, Vector3.ZERO, Vector3.UP)
		check_near(p.lateral_offset, -m.lateral_offset, TOL, "offsets are opposite at %s deg" % deg)
		check_near(p.gap, m.gap, TOL, "gaps are equal at %s deg" % deg)
		check_vec3_near(p.wheel_point, Vector3(-m.wheel_point.x, m.wheel_point.y, m.wheel_point.z), TOL, "points mirror in x at %s deg" % deg)
		check_near(p.profile_radius, m.profile_radius, TOL, "profile radius equal at %s deg" % deg)


func test_large_camber_clamps_the_contact_to_the_shoulder() -> void:
	# rho = 0.3, width 0.1: the unbounded optimum -rho sin(20 deg) = -0.1026 lies beyond the shoulder at -0.05.
	var g = geo(0.1, 0.3)
	var gamma: float = deg_to_rad(20.0)
	var r: Dictionary = TireGeometry.lowest_point(Vector3(0.0, 0.4, 0.0), camber_q(gamma), g, 0.3, Vector3.ZERO, Vector3.UP)
	check(r.valid and r.at_shoulder, "shoulder contact")
	check_near(r.lateral_offset, -0.05, 1.0e-9, "clamped to the shoulder")
	check_near(r.lateral_fraction, -1.0, 1.0e-9, "fraction -1")
	check_near(r.unclamped_offset, -0.3 * sin(gamma), TOL, "the unbounded optimum is reported")
	var s: float = -0.05
	var rad: float = 0.3 - 0.3 + sqrt(0.09 - s * s)
	var expected: Vector3 = Vector3(0.0, 0.4, 0.0) + Vector3(cos(gamma), sin(gamma), 0.0) * s + Vector3(sin(gamma), -cos(gamma), 0.0) * rad
	check_vec3_near(r.wheel_point, expected, TOL, "shoulder point")
	var mirrored: Dictionary = TireGeometry.lowest_point(Vector3(0.0, 0.4, 0.0), camber_q(-gamma), g, 0.3, Vector3.ZERO, Vector3.UP)
	check_near(mirrored.lateral_offset, 0.05, 1.0e-9, "the other width boundary")
	check(mirrored.at_shoulder, "at the + shoulder")


func test_lowest_point_agrees_with_a_brute_force_search_in_general_poses() -> void:
	var g = geo()
	var cases: Array = []
	# yaw (steering) + camber + spin about the axle, non-origin plane point, tilted normal
	var q1: Quaternion = Quaternion(Vector3.UP, 0.7) * camber_q(deg_to_rad(18.0)) * Quaternion(Vector3.RIGHT, 1.1)
	cases.append([Vector3(0.4, 0.5, -1.2), q1, Vector3(1.0, 0.1, -2.0), Vector3(0.0, 1.0, 0.0)])
	var n2: Vector3 = Vector3(0.2, 0.9, -0.3).normalized()
	var q2: Quaternion = Quaternion(Vector3(0.3, 0.8, 0.5).normalized(), 0.4) * Quaternion(Vector3.RIGHT, -2.3)
	cases.append([Vector3(-0.3, 0.7, 0.4), q2, Vector3(0.2, -0.1, 0.3), n2])
	var n3: Vector3 = Vector3(-0.35, 0.93, 0.1).normalized()
	cases.append([Vector3(2.0, 1.0, 3.0), Quaternion(Vector3.UP, -1.9) * camber_q(deg_to_rad(-30.0)), Vector3(2.0, 0.5, 3.0), n3])
	for i in cases.size():
		var c: Array = cases[i]
		var r: Dictionary = TireGeometry.lowest_point(c[0], c[1], g, R, c[2], c[3])
		var b: Dictionary = brute_force(c[0], c[1], g, R, c[2], c[3])
		check(r.valid, "case %d valid" % i)
		# Grid errors: angle step 0.5 deg -> R(1-cos 0.25 deg) = 3e-6 at most; offset step 5e-4 m -> < 1e-6. The search can only be >= the true minimum.
		check(r.gap <= b.gap + 1.0e-6, "case %d: the analytic minimum is not above the search minimum (%s vs %s)" % [i, r.gap, b.gap])
		check(r.gap >= b.gap - 1.0e-5, "case %d: and not below it by more than the grid error (%s vs %s)" % [i, r.gap, b.gap])
		check(absf(r.lateral_offset - b.s) <= 1.0e-3 or absf(r.gap - b.gap) < 1.0e-5, "case %d: offset agrees (%s vs %s)" % [i, r.lateral_offset, b.s])
		# Internal consistency of the result itself.
		check_near(c[3].dot(r.wheel_point - c[2]), r.gap, TOL, "case %d: gap is n.(point - p0)" % i)
		check_vec3_near(r.ground_point, r.wheel_point - c[3] * r.gap, TOL, "case %d: projection" % i)
		check_near(c[3].dot(r.ground_point - c[2]), 0.0, TOL, "case %d: the projection lies on the plane" % i)
		check_vec3_near(r.local_point, c[1].inverse() * (r.wheel_point - c[0]), TOL, "case %d: local coordinates" % i)
		check_near(r.local_point.x, r.lateral_offset, TOL, "case %d: local x is the lateral offset" % i)
		check_near(Vector2(r.local_point.y, r.local_point.z).length(), r.profile_radius, TOL, "case %d: local radial distance is r(s)" % i)


func test_spin_about_the_axle_does_not_move_the_world_point_but_rotates_the_local_point() -> void:
	var base: Quaternion = Quaternion(Vector3.UP, 0.4) * camber_q(deg_to_rad(15.0))
	var a: Dictionary = TireGeometry.lowest_point(Vector3(0.1, 0.45, 0.2), base, geo(), R, Vector3.ZERO, Vector3.UP)
	for angle in [0.5, 1.7, -2.4]:
		var spun: Quaternion = base * Quaternion(Vector3.RIGHT, angle)
		var b: Dictionary = TireGeometry.lowest_point(Vector3(0.1, 0.45, 0.2), spun, geo(), R, Vector3.ZERO, Vector3.UP)
		check_vec3_near(b.wheel_point, a.wheel_point, TOL, "world point unchanged by spin %s" % angle)
		check_near(b.gap, a.gap, TOL, "gap unchanged")
		check_near(b.local_point.x, a.local_point.x, TOL, "local x (lateral) unchanged")
		var expected_local: Vector3 = Quaternion(Vector3.RIGHT, -angle) * a.local_point
		check_vec3_near(b.local_point, expected_local, TOL, "local point rotates by -spin about the axle (%s)" % angle)


# --- Plane geometry ---

func test_non_origin_plane_and_non_vertical_normal_closed_form() -> void:
	var g = geo()
	# Vertical normal, plane at y = 0.25: gap = y - 0.25 - R upright.
	var up: Dictionary = TireGeometry.lowest_point(Vector3(1.0, 0.9, -2.0), Quaternion.IDENTITY, g, R, Vector3(5.0, 0.25, 7.0), Vector3.UP)
	check_near(up.gap, 0.9 - 0.25 - R, TOL, "gap against a raised plane")
	check_vec3_near(up.ground_point, Vector3(1.0, 0.25, -2.0), TOL, "projection onto the raised plane")
	# Normal tilted about Z by alpha, axle u = +X: a = n.u = -sin(alpha), in-plane part (0, cos(alpha), 0), e = (0,-1,0), s* = rho sin(alpha).
	var alpha: float = deg_to_rad(12.0)
	var n: Vector3 = Vector3(-sin(alpha), cos(alpha), 0.0)
	var centre: Vector3 = Vector3(0.2, 0.6, 0.1)
	var p0: Vector3 = Vector3(-0.5, 0.1, 0.4)
	var r: Dictionary = TireGeometry.lowest_point(centre, Quaternion.IDENTITY, g, R, p0, n)
	var s: float = RHO * sin(alpha)
	var expected_point: Vector3 = centre + Vector3(s, 0.0, 0.0) + Vector3(0.0, -radius_at(s), 0.0)
	check_near(r.lateral_offset, s, TOL, "offset on a tilted plane")
	check_vec3_near(r.wheel_point, expected_point, TOL, "point on a tilted plane")
	check_near(r.gap, n.dot(expected_point - p0), TOL, "signed gap along the tilted normal")
	check_vec3_near(r.ground_point, expected_point - n * n.dot(expected_point - p0), TOL, "projection along the tilted normal")
	check_vec3_near(r.normal, n, 0.0, "the normal is echoed")
	var below: Dictionary = TireGeometry.lowest_point(Vector3(0.0, 0.1, 0.0), Quaternion.IDENTITY, g, R, Vector3.ZERO, Vector3.UP)
	check(below.valid and below.gap < 0.0, "a wheel below the plane has a negative gap (penetration)")
	check_near(below.gap, 0.1 - R, TOL, "penetration depth")


func test_lowest_point_is_not_above_any_other_offset_and_matches_point_at_offset() -> void:
	var g = geo()
	var q: Quaternion = Quaternion(Vector3.UP, 0.3) * camber_q(deg_to_rad(22.0))
	var best: Dictionary = TireGeometry.lowest_point(Vector3(0.0, 0.5, 0.0), q, g, R, Vector3.ZERO, Vector3.UP)
	for i in 21:
		var s: float = -0.5 * W + W * float(i) / 20.0
		var p: Dictionary = TireGeometry.point_at_offset(Vector3(0.0, 0.5, 0.0), q, g, R, Vector3.ZERO, Vector3.UP, s)
		check(p.valid, "sample %d valid" % i)
		check(p.gap >= best.gap - 1.0e-7, "sample %d is not below the lowest point" % i)
		check_near(p.lateral_offset, s, 1.0e-9, "sample %d keeps its offset" % i)
		check_near(p.profile_radius, radius_at(s), TOL, "sample %d radius" % i)
	var at_best: Dictionary = TireGeometry.point_at_offset(Vector3(0.0, 0.5, 0.0), q, g, R, Vector3.ZERO, Vector3.UP, best.lateral_offset)
	check_vec3_near(at_best.wheel_point, best.wheel_point, TOL, "sampling at the optimum reproduces the lowest point")
	check_near(at_best.gap, best.gap, TOL, "and its gap")
	var left: Dictionary = TireGeometry.point_at_offset(Vector3.ZERO, Quaternion.IDENTITY, g, R, Vector3(0.0, -1.0, 0.0), Vector3.UP, -0.5 * W)
	var right: Dictionary = TireGeometry.point_at_offset(Vector3.ZERO, Quaternion.IDENTITY, g, R, Vector3(0.0, -1.0, 0.0), Vector3.UP, 0.5 * W)
	check(left.valid and right.valid and left.at_shoulder and right.at_shoulder, "both width boundaries are valid shoulder samples")
	check_vec3_near(left.wheel_point, Vector3(-0.5 * W, -radius_at(0.5 * W), 0.0), TOL, "- boundary point")
	check_vec3_near(right.wheel_point, Vector3(0.5 * W, -radius_at(0.5 * W), 0.0), TOL, "+ boundary point")
	check(not TireGeometry.point_at_offset(Vector3.ZERO, Quaternion.IDENTITY, g, R, Vector3.ZERO, Vector3.UP, 0.5 * W + 1.0e-3).valid, "beyond the + boundary is refused")
	check(not TireGeometry.point_at_offset(Vector3.ZERO, Quaternion.IDENTITY, g, R, Vector3.ZERO, Vector3.UP, -0.5 * W - 1.0e-3).valid, "beyond the - boundary is refused")
	check(not TireGeometry.point_at_offset(Vector3.ZERO, Quaternion.IDENTITY, g, R, Vector3.ZERO, Vector3.UP, NAN).valid, "NaN offset is refused")
	check(not TireGeometry.point_at_offset(Vector3.ZERO, Quaternion.IDENTITY, g, R, Vector3.ZERO, Vector3.UP, INF).valid, "INF offset is refused")


func test_lowest_point_never_exceeds_the_zero_width_centreline_point() -> void:
	for deg in [0.0, 5.0, 15.0, 30.0, -25.0]:
		var q: Quaternion = camber_q(deg_to_rad(deg))
		var finite: Dictionary = TireGeometry.lowest_point(Vector3(0.0, 0.5, 0.0), q, geo(), R, Vector3.ZERO, Vector3.UP)
		var thin: Dictionary = WheelGroundContact.compute_geometry(Vector3(0.0, 0.5, 0.0), q, R, Vector3.ZERO, Vector3.UP)
		check(finite.gap <= thin.gap + 1.0e-7, "the finite-width lowest point is not above the centreline point at %s deg" % deg)
		if absf(deg) > 1.0:
			check(finite.gap < thin.gap - 1.0e-4, "and is genuinely lower once the wheel is cambered (%s deg)" % deg)


func test_degenerate_and_non_finite_input_is_rejected_without_normalising_a_vanishing_vector() -> void:
	var g = geo()
	var flat: Quaternion = Quaternion(Vector3(0.0, 0.0, 1.0), PI / 2.0)  # axle along +Y = the plane normal
	var r: Dictionary = TireGeometry.lowest_point(Vector3(0.0, 0.5, 0.0), flat, g, R, Vector3.ZERO, Vector3.UP)
	check(not r.valid and "degenerate" in r.reason, "axle parallel to the normal is degenerate: %s" % r.reason)
	check(r.wheel_point.is_finite() and r.ground_point.is_finite() and is_finite(r.gap), "a failed result carries finite placeholders only")
	check_vec3_near(r.axle, Vector3(0.0, 1.0, 0.0), TOL, "the axle is still reported")
	var near_limit_ok: Dictionary = TireGeometry.lowest_point(Vector3(0.0, 0.5, 0.0), camber_q(deg_to_rad(80.0)), g, R, Vector3.ZERO, Vector3.UP)
	check(near_limit_ok.valid, "80 degrees of camber is still defined (L = cos 80 = 0.17)")
	var near_limit_bad: Dictionary = TireGeometry.lowest_point(Vector3(0.0, 0.5, 0.0), camber_q(deg_to_rad(86.0)), g, R, Vector3.ZERO, Vector3.UP)
	check(not near_limit_bad.valid, "86 degrees of camber is below the projection limit (L = 0.07)")
	check_near(TireGeometry.MIN_PLANE_PROJECTION, WheelGroundContact.MIN_PLANE_PROJECTION, 0.0, "same degenerate limit as the one-point contact")

	var ok_q: Quaternion = Quaternion.IDENTITY
	var bad_inputs: Array = [
		[Vector3(NAN, 0.5, 0.0), ok_q, Vector3.ZERO, Vector3.UP, "NaN position"],
		[Vector3(0.0, INF, 0.0), ok_q, Vector3.ZERO, Vector3.UP, "INF position"],
		[Vector3(0.0, 0.5, 0.0), Quaternion(NAN, 0.0, 0.0, 1.0), Vector3.ZERO, Vector3.UP, "NaN orientation"],
		[Vector3(0.0, 0.5, 0.0), Quaternion(0.0, 0.0, 0.0, 0.0), Vector3.ZERO, Vector3.UP, "zero quaternion"],
		[Vector3(0.0, 0.5, 0.0), Quaternion(0.0, 0.0, 0.0, 2.0), Vector3.ZERO, Vector3.UP, "non-unit quaternion"],
		[Vector3(0.0, 0.5, 0.0), ok_q, Vector3(0.0, NAN, 0.0), Vector3.UP, "NaN plane point"],
		[Vector3(0.0, 0.5, 0.0), ok_q, Vector3.ZERO, Vector3(0.0, NAN, 0.0), "NaN normal"],
		[Vector3(0.0, 0.5, 0.0), ok_q, Vector3.ZERO, Vector3(0.0, INF, 0.0), "INF normal"],
		[Vector3(0.0, 0.5, 0.0), ok_q, Vector3.ZERO, Vector3.ZERO, "zero normal"],
		[Vector3(0.0, 0.5, 0.0), ok_q, Vector3.ZERO, Vector3(0.0, 2.0, 0.0), "non-unit normal"],
	]
	for case in bad_inputs:
		var res: Dictionary = TireGeometry.lowest_point(case[0], case[1], g, R, case[2], case[3])
		check(not res.valid and res.reason != "", "rejected: %s" % case[4])
		check(res.wheel_point.is_finite() and is_finite(res.gap), "finite placeholders: %s" % case[4])
	check(not TireGeometry.lowest_point(Vector3(0.0, 0.5, 0.0), ok_q, null, R, Vector3.ZERO, Vector3.UP).valid, "null geometry")
	check(not TireGeometry.lowest_point(Vector3(0.0, 0.5, 0.0), ok_q, geo(0.0, 0.1), R, Vector3.ZERO, Vector3.UP).valid, "invalid geometry")
	check(not TireGeometry.lowest_point(Vector3(0.0, 0.5, 0.0), ok_q, geo(1.0e30, 1.0e30), R, Vector3.ZERO, Vector3.UP).valid, "geometry whose square overflows")
	for bad_radius in [0.0, -0.3, NAN, INF, 1.0e200]:
		check(not TireGeometry.lowest_point(Vector3(0.0, 0.5, 0.0), ok_q, g, bad_radius, Vector3.ZERO, Vector3.UP).valid, "wheel radius %s" % bad_radius)
	check(not TireGeometry.lowest_point(Vector3(0.0, 0.5, 0.0), ok_q, g, 0.05, Vector3.ZERO, Vector3.UP).valid, "shoulder drop above half the (small) wheel radius")


func test_finite_pose_values_whose_result_overflows_are_rejected() -> void:
	# Each coordinate is a finite float32 but a difference or product overflows to INF.
	var huge: float = 3.0e38
	var r: Dictionary = TireGeometry.lowest_point(Vector3(huge, huge, huge), Quaternion.IDENTITY, geo(), R, Vector3(-huge, -huge, -huge), Vector3.UP)
	check(not r.valid and r.reason != "", "overflowing gap/point is rejected (%s)" % r.reason)
	check(r.wheel_point.is_finite() and r.ground_point.is_finite() and is_finite(r.gap), "and carries finite placeholders")
	var r2: Dictionary = TireGeometry.lowest_point(Vector3(0.0, huge, 0.0), Quaternion.IDENTITY, geo(), R, Vector3(0.0, -huge, 0.0), Vector3.UP)
	check(not r2.valid, "an overflowing separation along the normal is rejected")
	var r3: Dictionary = TireGeometry.lowest_point(Vector3(1.0e9, 0.5, -1.0e9), Quaternion.IDENTITY, geo(), R, Vector3(1.0e9, 0.0, -1.0e9), Vector3.UP)
	check(r3.valid, "large but representable coordinates stay valid")
	check(is_finite(r3.gap) and r3.wheel_point.is_finite(), "with finite outputs (float32 resolution at 1e9 m is ~100 m, so no accuracy is claimed there)")


# --- Configuration, builder and assembly ---

func test_configuration_validates_front_and_rear_independently() -> void:
	var c: MotorcycleConfig = MotorcyclePrototype.config()
	check(c.front_tire_geometry == null and c.rear_tire_geometry == null, "no geometry by default (M2.4.x configurations are unchanged)")
	check(c.validate().is_empty(), "the default configuration is valid")
	c.front_tire_geometry = geo(0.12, 0.07)
	c.rear_tire_geometry = geo(0.18, 0.10)
	check(c.validate().is_empty(), "front and rear with different values are valid")
	check_near(c.front_tire_geometry.width, 0.12, 0.0, "front keeps its width")
	check_near(c.rear_tire_geometry.width, 0.18, 0.0, "rear keeps its width")
	var only_front: MotorcycleConfig = MotorcyclePrototype.config()
	only_front.front_tire_geometry = geo()
	check(only_front.validate().is_empty(), "only a front geometry is valid")
	var bad_front: MotorcycleConfig = MotorcyclePrototype.config()
	bad_front.front_tire_geometry = geo(0.0, 0.1)
	bad_front.rear_tire_geometry = geo()
	var errors: PackedStringArray = bad_front.validate()
	check(errors.size() == 1 and errors[0].begins_with("front tire geometry"), "only the front is reported: %s" % [errors])
	var bad_rear: MotorcycleConfig = MotorcyclePrototype.config()
	bad_rear.front_tire_geometry = geo()
	bad_rear.rear_tire_geometry = geo(0.18, 0.05)
	errors = bad_rear.validate()
	check(errors.size() == 1 and errors[0].begins_with("rear tire geometry"), "only the rear is reported: %s" % [errors])
	var shared: MotorcycleConfig = MotorcyclePrototype.config()
	var one = geo()
	shared.front_tire_geometry = one
	shared.rear_tire_geometry = one
	check(not shared.validate().is_empty(), "the same object for both tyres is rejected")
	var drop: MotorcycleConfig = MotorcyclePrototype.config()
	drop.front_tire_geometry = geo(0.3, 0.15)
	drop.front_wheel_radius = 0.2
	check(not drop.validate().is_empty(), "a shoulder drop above half the front wheel radius is rejected (semicircular 0.3 m tread: drop 0.15 m > 0.1 m)")


func test_builder_rejects_invalid_geometry_before_registering_anything() -> void:
	var cases: Array = []
	var a: MotorcycleConfig = MotorcyclePrototype.config_with_tires()
	a.front_tire_geometry = geo(NAN, 0.1)
	cases.append([a, "NaN front width"])
	var b: MotorcycleConfig = MotorcyclePrototype.config_with_tires()
	b.rear_tire_geometry = geo(0.18, 1.0e30)
	cases.append([b, "huge rear crown radius"])
	var c: MotorcycleConfig = MotorcyclePrototype.config()
	c.front_tire_geometry = geo()
	c.rear_tire_geometry = geo(-0.1, 0.1)
	cases.append([c, "negative rear width, valid front, no ground"])
	for case in cases:
		var world: SimWorld = SimWorld.new()
		var result = MotorcycleBuilder.build(case[0], world)
		check(result == null, "build returns null (%s)" % case[1])
		check(world.bodies.is_empty() and world.constraints.is_empty() and world.force_providers.is_empty(), "nothing registered (%s)" % case[1])


func test_assembly_gives_each_wheel_its_own_geometry_and_leaves_the_topology_unchanged() -> void:
	var plain = MotorcycleBuilder.build(MotorcyclePrototype.config_with_tires())
	var cfg: MotorcycleConfig = MotorcyclePrototype.config_with_tires()
	cfg.front_tire_geometry = geo(0.12, 0.07)
	cfg.rear_tire_geometry = geo(0.18, 0.10)
	var a = MotorcycleBuilder.build(cfg)
	check(a != null, "builds with geometry")
	check(plain.front_tire_geometry == null and plain.rear_tire_geometry == null, "an assembly without configured geometry has none")
	check(a.front_tire_geometry != null and a.rear_tire_geometry != null, "both wheels receive a geometry")
	check(a.front_tire_geometry != a.rear_tire_geometry, "as distinct objects")
	check_near(a.front_tire_geometry.width, 0.12, 0.0, "front width")
	check_near(a.rear_tire_geometry.width, 0.18, 0.0, "rear width")
	check_near(a.front_tire_geometry.crown_radius, 0.07, 0.0, "front crown radius")
	check_near(a.rear_tire_geometry.crown_radius, 0.10, 0.0, "rear crown radius")
	check(a.front_tire_geometry != cfg.front_tire_geometry and a.rear_tire_geometry != cfg.rear_tire_geometry, "the assembly holds copies of the configuration")
	cfg.front_tire_geometry.width = 0.4
	check_near(a.front_tire_geometry.width, 0.12, 0.0, "later configuration edits do not reach the assembly")
	check(a.world.bodies.size() == plain.world.bodies.size() and a.world.constraints.size() == plain.world.constraints.size() \
			and a.world.force_providers.size() == plain.world.force_providers.size(), "body, constraint and provider counts are unchanged")
	check(a.front_tire != null and a.rear_tire != null and a.front_tire == a.front_wheel_contact and a.rear_tire == a.rear_wheel_contact, "tyre and contact references are unchanged")
	check(a.front_wheel_contact.body_b == a.front_wheel and a.rear_wheel_contact.body_b == a.rear_wheel, "contacts still reference their wheels")
	check(a.front_wheel_contact.validate().is_empty() and a.rear_wheel_contact.validate().is_empty(), "contacts remain valid")
	check_near(a.front_wheel_contact.wheel_radius, plain.front_wheel_contact.wheel_radius, 0.0, "the contact radius is the wheel radius, unchanged")
	check(a.front_tire.geometry_valid and a.rear_tire.geometry_valid, "one-point contact geometry is unchanged")
	var geometry_front: Dictionary = TireGeometry.lowest_point(a.front_wheel.position, a.front_wheel.orientation, a.front_tire_geometry,
			a.config.front_wheel_radius, Vector3.ZERO, Vector3.UP)
	check(geometry_front.valid, "the stored front geometry evaluates against the assembled front wheel")
	check_near(geometry_front.lateral_offset, 0.0, 1.0e-4, "the reference pose is upright: centreline contact (the prototype wheel has no camber)")
	var no_ground: MotorcycleConfig = MotorcyclePrototype.config()
	no_ground.front_tire_geometry = geo()
	var b = MotorcycleBuilder.build(no_ground)
	check(b != null and b.front_tire_geometry != null and b.rear_tire_geometry == null and not b.has_ground(), "geometry does not require a ground and is independent per wheel")
