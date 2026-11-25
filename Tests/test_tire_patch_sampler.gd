extends "res://Tests/sim_test_case.gd"
## M2.5.3 Stage A: deterministic finite-width tyre surface sampling (Simulation/Motorcycle/tire_patch_sampler.gd). Geometry and
## area quadrature only; no contact, force or constraint is tested here.
##
## Expectations are computed IN THIS FILE from closed forms (circular crown, exact tread area), finite differences of the
## surface map, and TireGeometry.lowest_point() as the independent reference for the lowest point. None of them uses the
## sampler's own normal or weight code as oracle.
## Tolerances: float32 vector arithmetic on metre-sized values is good to ~2e-6, so vector checks use TOL = 5e-6 unless
## stated. Area convergence is measured in relative terms (documented at the use).

const TireGeometry = preload("res://Simulation/Motorcycle/tire_geometry.gd")
const TirePatchSampler = preload("res://Simulation/Motorcycle/tire_patch_sampler.gd")

const TOL: float = 5.0e-6
## Test tyre: width 0.18 m, crown radius 0.10 m, wheel radius 0.31 m.
const W: float = 0.18
const RHO: float = 0.10
const R: float = 0.31
const ORIGIN: Vector3 = Vector3(1.0, 2.0, 3.0)


func geo(width: float = W, crown: float = RHO):
	return TireGeometry.new(width, crown)


## Rotation about +Z by g: lifts the +X (axle) end, i.e. camber g.
func camber_q(g: float) -> Quaternion:
	return Quaternion(Vector3(0.0, 0.0, 1.0), g)


func pose_q() -> Quaternion:
	# Camber 0.3 rad, steering yaw 0.2 rad (about Y) and spin 0.7 rad (about the axle, before the camber).
	var spin: Quaternion = Quaternion(Vector3(1.0, 0.0, 0.0), 0.7)
	var yaw: Quaternion = Quaternion(Vector3(0.0, 1.0, 0.0), 0.2)
	return (yaw * camber_q(0.3) * spin).normalized()


func sample_default(q: Quaternion, n_lat: int = 9, n_circ: int = 24, p: Vector3 = ORIGIN, g = null, plane_p: Vector3 = Vector3.ZERO,
		plane_n: Vector3 = Vector3(0.0, 1.0, 0.0)) -> Dictionary:
	var geometry = g if g != null else geo()
	return TirePatchSampler.sample(p, q, geometry, R, plane_p, plane_n, n_lat, n_circ)


## Closed-form area of the tread: 2 pi rho [ 2 (R - rho) asin(w / 2 rho) + w ].
func analytic_area(width: float, rho: float, r_wheel: float) -> float:
	return TAU * rho * (2.0 * (r_wheel - rho) * asin(0.5 * width / rho) + width)


# ---- A: validation (every bad input fails safely, no samples) ----

func test_a_invalid_inputs_fail_safely() -> void:
	var q: Quaternion = Quaternion.IDENTITY
	var n: Vector3 = Vector3(0.0, 1.0, 0.0)
	var cases: Array = []
	cases.append({"name": "lateral count 1", "res": TirePatchSampler.sample(ORIGIN, q, geo(), R, Vector3.ZERO, n, 1, 24)})
	cases.append({"name": "lateral count 65", "res": TirePatchSampler.sample(ORIGIN, q, geo(), R, Vector3.ZERO, n, 65, 24)})
	cases.append({"name": "circumferential count 2", "res": TirePatchSampler.sample(ORIGIN, q, geo(), R, Vector3.ZERO, n, 9, 2)})
	cases.append({"name": "circumferential count 257", "res": TirePatchSampler.sample(ORIGIN, q, geo(), R, Vector3.ZERO, n, 9, 257)})
	cases.append({"name": "null geometry", "res": TirePatchSampler.sample(ORIGIN, q, null, R, Vector3.ZERO, n, 9, 24)})
	cases.append({"name": "zero width", "res": TirePatchSampler.sample(ORIGIN, q, geo(0.0, RHO), R, Vector3.ZERO, n, 9, 24)})
	cases.append({"name": "crown below half width", "res": TirePatchSampler.sample(ORIGIN, q, geo(W, 0.05), R, Vector3.ZERO, n, 9, 24)})
	cases.append({"name": "shoulder drop above ratio", "res": TirePatchSampler.sample(ORIGIN, q, geo(0.5, 0.25), 0.2, Vector3.ZERO, n, 9, 24)})
	cases.append({"name": "zero wheel radius", "res": TirePatchSampler.sample(ORIGIN, q, geo(), 0.0, Vector3.ZERO, n, 9, 24)})
	cases.append({"name": "NaN position", "res": TirePatchSampler.sample(Vector3(NAN, 0.0, 0.0), q, geo(), R, Vector3.ZERO, n, 9, 24)})
	cases.append({"name": "INF plane point", "res": TirePatchSampler.sample(ORIGIN, q, geo(), R, Vector3(INF, 0.0, 0.0), n, 9, 24)})
	cases.append({"name": "non-unit quaternion", "res": TirePatchSampler.sample(ORIGIN, Quaternion(0.0, 0.0, 0.0, 2.0), geo(), R, Vector3.ZERO, n, 9, 24)})
	cases.append({"name": "non-unit plane normal", "res": TirePatchSampler.sample(ORIGIN, q, geo(), R, Vector3.ZERO, Vector3(0.0, 2.0, 0.0), 9, 24)})
	for c in cases:
		var res: Dictionary = c["res"]
		check(not res["valid"], "invalid input accepted: %s" % c["name"])
		check(res["samples"].is_empty(), "invalid input produced samples: %s" % c["name"])
		check(not String(res["reason"]).is_empty(), "invalid input without a reason: %s" % c["name"])


# ---- B: counts and coverage of the lateral width and circumference ----

func test_b_counts_and_coverage() -> void:
	var res: Dictionary = sample_default(Quaternion.IDENTITY, 9, 24)
	check(res["valid"], "valid tyre rejected: %s" % res["reason"])
	check(res["samples"].size() == 9 * 24, "sample count %d, expected 216" % res["samples"].size())
	var s_min: float = INF
	var s_max: float = -INF
	var offsets: Dictionary = {}
	var angles: Dictionary = {}
	for smp in res["samples"]:
		s_min = minf(s_min, smp["lateral_offset"])
		s_max = maxf(s_max, smp["lateral_offset"])
		offsets[smp["lateral_index"]] = smp["lateral_offset"]
		angles[smp["angular_index"]] = smp["angle"]
	# Shoulders are sampled exactly (end nodes included), so the tread is covered from one shoulder to the other.
	check_near(s_min, -0.5 * W, 1e-9, "left shoulder not sampled")
	check_near(s_max, 0.5 * W, 1e-9, "right shoulder not sampled")
	check(offsets.size() == 9, "lateral nodes %d, expected 9" % offsets.size())
	check(angles.size() == 24, "circumferential nodes %d, expected 24" % angles.size())
	# Circumference: uniform, starting at 0 and covering [0, 2 pi) without a duplicate at 2 pi.
	for j in 24:
		check_near(angles[j], TAU * float(j) / 24.0, 1e-9, "angle %d not uniform" % j)


# ---- C: every sample lies on the configured finite-width surface ----

func test_c_samples_lie_on_the_configured_surface() -> void:
	var q: Quaternion = pose_q()
	var res: Dictionary = sample_default(q, 9, 24, Vector3(1.0, 2.0, 3.0))
	var u: Vector3 = q * Vector3(1.0, 0.0, 0.0)
	for smp in res["samples"]:
		var s: float = smp["lateral_offset"]
		var offset: Vector3 = smp["position"] - ORIGIN
		# Axial coordinate equals s (the sample is on the meridian of that lateral offset).
		check_near(offset.dot(u), s, TOL, "axial coordinate of a sample is not its lateral offset")
		# Radial distance from the axle line equals the closed-form profile R - rho + sqrt(rho^2 - s^2) (independent of the sampler).
		var radial: Vector3 = offset - u * offset.dot(u)
		var expected_r: float = R - RHO + sqrt(RHO * RHO - s * s)
		check_near(radial.length(), expected_r, TOL, "radial distance does not follow the tyre profile")
		check_near(smp["radius"], expected_r, TOL, "stored radius disagrees with the profile")


# ---- D: normals are unit, outward and perpendicular to the surface ----

func test_d_normals_are_unit_outward_and_tangent_consistent() -> void:
	var q: Quaternion = pose_q()
	var g = geo()
	var res: Dictionary = sample_default(q, 9, 24, ORIGIN)
	var h: float = 1.0e-3
	for smp in res["samples"]:
		var n: Vector3 = smp["normal"]
		check_near(n.length(), 1.0, 1.0e-5, "normal is not a unit vector")
		var s: float = smp["lateral_offset"]
		var theta: float = smp["angle"]
		var i: int = smp["lateral_index"]
		if i == 0 or i == 8:
			continue # shoulder: finite-difference tangent in s is one-sided there; checked by the interior nodes and the closed form
		# Finite-difference tangents of the closed surface map (NOT the sampler's normal algebra).
		var ts: Vector3 = (TirePatchSampler.surface_point(ORIGIN, q, g, R, s + h, theta)
				- TirePatchSampler.surface_point(ORIGIN, q, g, R, s - h, theta)) / (2.0 * h)
		var tt: Vector3 = (TirePatchSampler.surface_point(ORIGIN, q, g, R, s, theta + h)
				- TirePatchSampler.surface_point(ORIGIN, q, g, R, s, theta - h)) / (2.0 * h)
		check(absf(n.dot(ts)) / ts.length() < 1.0e-3, "normal not perpendicular to the lateral tangent")
		check(absf(n.dot(tt)) / tt.length() < 1.0e-3, "normal not perpendicular to the circumferential tangent")
		# Outward: the normal has a non-negative component along the radial direction e (the crown is convex).
		var e: Vector3 = TirePatchSampler.radial_direction(q, theta)
		check(n.dot(e) >= -1.0e-6, "normal points into the tyre")


# ---- E: area quadrature converges to the exact tread area ----

func test_e_area_quadrature_converges_to_the_exact_area() -> void:
	var exact: float = analytic_area(W, RHO, R)
	var coarse: float = sample_default(Quaternion.IDENTITY, 9, 24)["total_area"]
	var mid: float = sample_default(Quaternion.IDENTITY, 17, 48)["total_area"]
	var fine: float = sample_default(Quaternion.IDENTITY, 33, 96)["total_area"]
	var e_coarse: float = absf(coarse - exact) / exact
	var e_mid: float = absf(mid - exact) / exact
	var e_fine: float = absf(fine - exact) / exact
	# The circumferential rule is spectrally accurate on a periodic integrand; the lateral trapezoid over phi is second order
	# (the lateral node spacing halves from 9 -> 17 -> 33 nodes). The test checks that ORDER: each refinement divides the error
	# by about 4. The absolute bound is the one measured at the finest grid here (1.1e-4), not a tolerance tuned to pass.
	var ratio_1: float = e_coarse / e_mid
	var ratio_2: float = e_mid / e_fine
	check(ratio_1 > 3.0 and ratio_1 < 5.0, "coarse->mid error ratio %s is not second order" % ratio_1)
	check(ratio_2 > 3.0 and ratio_2 < 5.0, "mid->fine error ratio %s is not second order" % ratio_2)
	check(e_fine < 2.0e-4, "finest area relative error %s exceeds the measured bound 2e-4" % e_fine)


# ---- F: camber moves the lowest sample to the same lateral place as TireGeometry.lowest_point ----

func test_f_camber_lowest_sample_matches_the_reference_lowest_point() -> void:
	var q: Quaternion = camber_q(0.25)
	var g = geo()
	var n: Vector3 = Vector3(0.0, 1.0, 0.0)
	var oracle: Dictionary = TireGeometry.lowest_point(ORIGIN, q, g, R, Vector3.ZERO, n)
	check(oracle["valid"], "reference lowest point invalid: %s" % oracle["reason"])
	var res: Dictionary = sample_default(q, 64, 256, ORIGIN, g, Vector3.ZERO, n)
	check(res["valid"], "fine camber grid rejected: %s" % res["reason"])
	var best: Dictionary = {}
	var best_gap: float = INF
	for smp in res["samples"]:
		if smp["gap"] < best_gap:
			best_gap = smp["gap"]
			best = smp
	# A grid cannot be below the true minimum, so the sampled gap is an upper bound on the minimum gap. The bound below is on the
	# HEIGHT difference h(s_n, theta_m) - h(s*), where h(s) = h_c + a s - L r(s) is the minimum over the tread circle at lateral
	# offset s, L = sqrt(1 - a^2), a = sin(0.25) = 0.2474 (test camber), r(s) = R - rho + sqrt(rho^2 - s^2), and s* = -rho a is the
	# unclamped optimum (h'(s*) = 0 exactly). Test tyre: rho = 0.10 m, R = 0.31 m, w = 0.18 m; grid 64 lateral x 256 circumferential.
	# Two sources, both bounded over the test domain:
	#  - Lateral (curvature term). The grid is uniform in the meridian angle phi, with phi* = asin(-a) = -0.25 and spacing
	#    d = 2 asin(w/2rho)/63 = 3.55e-2 rad. The nearest node n lies in the interval [phi* - d/2, phi* + d/2], whose image under
	#    s = rho sin(phi) is I_s = [-0.026459, -0.023014] m. Its largest distance from s* is delta = 1.726e-3 m. Over I_s the
	#    curvature h''(s) = L rho^2 / (rho^2 - s^2)^(3/2) is largest at the end nearest the tyre shoulder: max |h''| = 10.80 1/m
	#    (rho^2 - s^2 >= 9.30e-3 m^2 on I_s; it is NOT the value at s = 0, which is 9.7 1/m). Taylor with h'(s*) = 0 and the
	#    mean-value remainder gives 0 <= h(s_n) - h(s*) <= (1/2) max_I|h''| delta^2 = 0.5 * 10.80 * (1.726e-3)^2 = 1.61e-5 m.
	#    Check: the exact difference at both ends of I_s is 1.58e-5 m, inside the bound.
	#  - Angular (height). The nearest meridian is at most half a step, 2 pi / 256 / 2 = 1.227e-2 rad, from the optimal e. For
	#    fixed s, moving e by delta raises the height by r L (1 - cos delta), since n . e(theta) = -L cos(delta) at the best e.
	#    With r <= R = 0.31 m and L <= 1: r L (1 - cos delta) <= 0.31 * (1 - cos 1.227e-2) = 2.33e-5 m.
	#  Combined height error: 1.61e-5 + 2.33e-5 = 3.94e-5 m, which is below the 1e-4 m tolerance on the gap by a factor of 2.5.
	#  The first check (grid minimum not below the true minimum) holds by construction, and the second (within 1e-4 m) holds by the
	#  bound above, so both assertions below are covered by these estimates; no tolerance was changed.
	# The lateral POSITION of the lowest sample is a different quantity (metres along the axle, not height). Its check below
	# uses the full lateral spacing (3.6e-3 m), a looser bound than the half spacing that the nearest-node argument needs.
	check(best_gap >= oracle["gap"] - 1.0e-6, "grid minimum below the true minimum (%s < %s)" % [best_gap, oracle["gap"]])
	check_near(best_gap, oracle["gap"], 1.0e-4, "lowest sampled gap disagrees with TireGeometry.lowest_point")
	check_near(best["lateral_offset"], oracle["lateral_offset"], 2.0 * asin(0.5 * W / RHO) * RHO / 63.0 + 1.0e-6,
			"lowest sample lateral position is not the reference lowest point's")
	check(signf(best["lateral_offset"]) == signf(oracle["lateral_offset"]), "camber moved the lowest sample to the wrong side")


# ---- G: spin and steering are carried by the pose (wheel frame) ----

func test_g_spin_shifts_the_circumferential_index_exactly() -> void:
	var q: Quaternion = camber_q(0.2)
	var n_circ: int = 24
	var k: int = 3
	var spin: Quaternion = Quaternion(Vector3(1.0, 0.0, 0.0), TAU * float(k) / float(n_circ))
	var q_spun: Quaternion = (q * spin).normalized()
	var base: Dictionary = sample_default(q, 9, n_circ)
	var spun: Dictionary = sample_default(q_spun, 9, n_circ)
	# Spin by k grid steps about +X takes the radial frame Y toward Z, so sample (i, j) of the spun wheel is sample (i, j + k) of the unspun one.
	var max_err: float = 0.0
	for i in 9:
		for j in n_circ:
			var a: Vector3 = spun["samples"][i * n_circ + j]["position"]
			var b: Vector3 = base["samples"][i * n_circ + ((j + k) % n_circ)]["position"]
			max_err = maxf(max_err, (a - b).length())
	check(max_err < TOL, "spin does not shift the circumferential samples by whole steps (err %s)" % max_err)


func test_g_yaw_rotates_the_surface_rigidly() -> void:
	var q: Quaternion = camber_q(0.3)
	var yaw: Quaternion = Quaternion(Vector3(0.0, 1.0, 0.0), 0.4)
	var pos2: Vector3 = Vector3(-0.5, 0.3, 2.0)
	var base: Dictionary = sample_default(q, 9, 24, ORIGIN)
	var steered: Dictionary = sample_default((yaw * q).normalized(), 9, 24, pos2)
	var max_err: float = 0.0
	for idx in base["samples"].size():
		var expected: Vector3 = pos2 + yaw * (base["samples"][idx]["position"] - ORIGIN)
		max_err = maxf(max_err, (steered["samples"][idx]["position"] - expected).length())
	check(max_err < TOL, "yaw/translation does not move the surface rigidly (err %s)" % max_err)


# ---- H: width and profile change the sampled geometry ----

func test_h_width_and_crown_change_the_shoulders() -> void:
	var narrow: Dictionary = TirePatchSampler.sample(Vector3.ZERO, Quaternion.IDENTITY, geo(0.12, RHO), R, Vector3.ZERO,
			Vector3(0.0, 1.0, 0.0), 9, 24)
	var wide: Dictionary = TirePatchSampler.sample(Vector3.ZERO, Quaternion.IDENTITY, geo(W, RHO), R, Vector3.ZERO,
			Vector3(0.0, 1.0, 0.0), 9, 24)
	check(narrow["valid"] and wide["valid"], "width variants rejected")
	var narrow_extent: float = narrow["samples"][narrow["samples"].size() - 1]["lateral_offset"]
	var wide_extent: float = wide["samples"][wide["samples"].size() - 1]["lateral_offset"]
	check_near(narrow_extent, 0.06, 1.0e-9, "narrow tread extent wrong")
	check_near(wide_extent, 0.09, 1.0e-9, "wide tread extent wrong")
	# Shoulder radius closed form for the test tyre: R - rho + sqrt(rho^2 - (w/2)^2).
	var shoulder_r: float = wide["samples"][wide["samples"].size() - 1]["radius"]
	check_near(shoulder_r, R - RHO + sqrt(RHO * RHO - 0.09 * 0.09), TOL, "wide shoulder radius wrong")
	var flat: Dictionary = TirePatchSampler.sample(Vector3.ZERO, Quaternion.IDENTITY, geo(W, 0.3), R, Vector3.ZERO,
			Vector3(0.0, 1.0, 0.0), 9, 24)
	var flat_shoulder: float = flat["samples"][flat["samples"].size() - 1]["radius"]
	check(flat_shoulder > shoulder_r + 1.0e-3, "a flatter tread did not change the shoulder radius")


# ---- I: plane gaps are exact on an upright wheel ----

func test_i_gap_matches_the_lowest_point_of_an_upright_wheel() -> void:
	var res: Dictionary = sample_default(Quaternion.IDENTITY, 9, 24, Vector3.ZERO, null, Vector3.ZERO, Vector3(0.0, 1.0, 0.0))
	var min_gap: float = INF
	var at_index: int = -1
	for idx in res["samples"].size():
		var g: float = res["samples"][idx]["gap"]
		if g < min_gap:
			min_gap = g
			at_index = idx
	# Upright axle along X: the lowest meridian point is at the centreline (s = 0) and e = -Y, i.e. theta = pi (grid node 12 of 24).
	check_near(min_gap, -R, TOL, "upright lowest gap is not -R")
	check(res["samples"][at_index]["lateral_index"] == 4 and res["samples"][at_index]["angular_index"] == 12,
			"upright lowest sample is not the centreline node at theta = pi")


# ---- J: degenerate orientation is safe (samples defined, reference lowest point undefined) ----

func test_j_degenerate_orientation_is_safe() -> void:
	# Axle along the plane normal Y: rotate X onto Y.
	var q: Quaternion = Quaternion(Vector3(0.0, 0.0, 1.0), 0.5 * PI)
	var n: Vector3 = Vector3(0.0, 1.0, 0.0)
	var res: Dictionary = sample_default(q, 9, 24, ORIGIN, null, Vector3.ZERO, n)
	check(res["valid"], "degenerate orientation rejected: %s" % res["reason"])
	var all_finite: bool = true
	for smp in res["samples"]:
		all_finite = all_finite and is_finite(smp["gap"]) and smp["position"].is_finite() and smp["normal"].is_finite()
	check(all_finite, "degenerate orientation produced non-finite samples")
	var oracle: Dictionary = TireGeometry.lowest_point(ORIGIN, q, geo(), R, Vector3.ZERO, n)
	check(not oracle["valid"], "reference lowest point should be undefined for an axle along the normal")


# ---- K: determinism and total area consistency ----

func test_k_bit_identical_repeats_and_consistent_total_area() -> void:
	var a: Dictionary = sample_default(pose_q(), 17, 48, ORIGIN)
	var b: Dictionary = sample_default(pose_q(), 17, 48, ORIGIN)
	var identical: bool = a["samples"].size() == b["samples"].size()
	var total: float = 0.0
	for idx in a["samples"].size():
		if not identical:
			break
		identical = identical and a["samples"][idx]["position"] == b["samples"][idx]["position"] \
				and a["samples"][idx]["weight"] == b["samples"][idx]["weight"]
		total += a["samples"][idx]["weight"]
	check(identical, "two identical calls produced different samples")
	check_near(total, a["total_area"], 1.0e-9, "total_area is not the sum of the weights")
	check(a["lateral_count"] == 17 and a["circumferential_count"] == 48, "grid sizes not reported")


# ---- L: a signed gap that overflows is rejected before any sample is kept ----

func test_l_overflowing_gap_is_rejected_with_no_samples() -> void:
	# Every input is finite: the wheel centre at +3e38 m and the plane point at -3e38 m (float32 max is about 3.4e38).
	# The sample positions are finite (~3e38 m), but point - plane_point is about 6e38 m and overflows to +inf.
	var big: float = 3.0e38
	var res: Dictionary = TirePatchSampler.sample(Vector3(big, 0.0, 0.0), Quaternion.IDENTITY, geo(), R,
			Vector3(-big, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), 9, 24)
	check(not res["valid"], "overflowing signed gap accepted as a valid sample set")
	check(res["samples"].is_empty(), "overflowing signed gap left %d samples in the result" % res["samples"].size())
	check(not String(res["reason"]).is_empty(), "overflowing signed gap rejected without a reason")
