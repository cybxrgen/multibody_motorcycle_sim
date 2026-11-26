extends "res://Tests/sim_test_case.gd"
## M2.5.3 Stage B: area-weighted compliant normal support of the finite-width tyre patch
## (Simulation/Motorcycle/tire_patch_normal_support.gd). Normal support only: no tangential force, slip or rolling resistance is tested.
##
## Expectations are computed IN THIS FILE: a closed-form 1D integral for the upright wheel, a fine 2D midpoint integral of the same
## surface map for a cambered wheel (independent of the sampler's quadrature and of the constraint), and the independent
## TirePatchSampler samples for the geometry. None of them uses the constraint's own load code as oracle.
##
## Fixtures. HEAVY one-step rig: a 1e7 kg wheel without gravity. Its velocity changes by ~1e-6 m/s in one step, so the implicit
## row reduces to lambda_i = -dt k_i g_i (vn' ~ 0, a relative effect of about 1e-5 on the loads) and the sample loads are the closed
## forms k_A A_i delta_i. DYNAMIC rig: a 100 kg wheel
## in gravity. A free upright wheel on a plane is unstable in roll, so the long runs use the TEST-ONLY vertical fixture of the other
## ground tests (a static anchor and a PrismaticJoint along world Y: vertical translation free, everything else removed); it is never
## part of any production object. Short runs (<= 40 steps) use no fixture.
## Test tyre: width 0.18 m, crown radius 0.10 m, wheel radius 0.31 m (the Stage A test tyre). Compliance: k_A = 4e6 N/m^3 (about 880 N
## per 2 cm of compression for this tyre), c_A = 1e5 N s/m^3, bottoming at 5 cm. These are test values, not tyre data.
## Tolerances: float32 vector arithmetic on metre-sized values resolves positions to ~3e-8 m, so loads are compared with k_A A_i times
## a 1 micrometre gap tolerance; aggregate checks use the measured quadrature errors stated at the use.

const TireGeometry = preload("res://Simulation/Motorcycle/tire_geometry.gd")
const TirePatchSampler = preload("res://Simulation/Motorcycle/tire_patch_sampler.gd")
const TirePatchNormalSupport = preload("res://Simulation/Motorcycle/tire_patch_normal_support.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const PrismaticJoint = preload("res://Simulation/Solver/prismatic_joint_constraint.gd")

const R: float = 0.31
const W: float = 0.18
const RHO: float = 0.10
const DT: float = 1.0 / 120.0
const G: float = 9.81
const K_A: float = 4.0e6
const C_A: float = 1.0e5
const D_MAX: float = 0.05
const HEAVY: float = 1.0e7
## Compression of the crown in the controlled fixtures (m).
const DELTA: float = 0.02
## One micrometre: the gap resolution of float32 positions of this size, as a tolerance on a sample's compression.
const GAP_TOL: float = 1.0e-6


# ---------------------------------------------------------------- fixtures

func geo() -> TireGeometry:
	return TireGeometry.new(W, RHO)


## Rig options: mass, height (wheel centre), camber, normal, plane_point, position, orientation, velocity, angular_velocity,
## gravity (positive number), k, c, dmax, lat, circ, inertia (body-frame Basis override), fixture (vertical-only test fixture). Returns {world, ground, wheel, support}.
func make_rig(opts: Dictionary = {}) -> Dictionary:
	var mass: float = opts.get("mass", 100.0)
	var q: Quaternion = opts.get("orientation", Quaternion(Vector3(0.0, 0.0, 1.0), opts.get("camber", 0.0)))
	var w: SimWorld = make_world(Vector3(0.0, -opts.get("gravity", 0.0), 0.0))
	var ground: SimRigidBody = make_static()
	var pos: Vector3 = opts.get("position", Vector3(0.0, opts.get("height", R), 0.0))
	var wheel: SimRigidBody = make_body(mass, opts.get("inertia", MotorcycleConfig.wheel_inertia(mass, R, 0.75 * R, W)), pos, q)
	wheel.linear_velocity = opts.get("velocity", Vector3.ZERO)
	wheel.angular_velocity = opts.get("angular_velocity", Vector3.ZERO)
	w.add_body(ground)
	w.add_body(wheel)
	var support = TirePatchNormalSupport.create(ground, wheel, geo(), R, opts.get("plane_point", Vector3.ZERO), opts.get("normal", Vector3.UP),
			opts.get("k", K_A), opts.get("c", C_A), opts.get("dmax", D_MAX), opts.get("lat", 9), opts.get("circ", 48))
	w.add_constraint(support)
	if opts.get("fixture", false):
		var anchor: SimRigidBody = make_static(pos)
		w.add_body(anchor)
		w.add_constraint(PrismaticJoint.from_world(anchor, wheel, pos, Vector3.UP, Vector3.RIGHT))
	return {"world": w, "ground": ground, "wheel": wheel, "support": support}


## HEAVY one-step rig with the crown compressed by `delta` (the wheel centre at R - delta cos(camber) keeps the crown-side lowest point).
func heavy_rig(lat: int, circ: int, camber: float = 0.0, delta: float = DELTA, extra: Dictionary = {}) -> Dictionary:
	var opts: Dictionary = {"mass": HEAVY, "height": R - delta * cos(camber), "camber": camber, "lat": lat, "circ": circ}
	for key in extra:
		opts[key] = extra[key]
	return make_rig(opts)


func try_create(over: Dictionary):
	var args: Dictionary = {"ground": make_static(), "wheel": make_body(100.0, MotorcycleConfig.wheel_inertia(100.0, R, 0.75 * R, W), Vector3(0.0, R, 0.0)),
		"geometry": geo(), "radius": R, "point": Vector3.ZERO, "normal": Vector3.UP, "k": K_A, "c": C_A, "dmax": D_MAX, "lat": 9, "circ": 48}
	for key in over:
		args[key] = over[key]
	return TirePatchNormalSupport.create(args["ground"], args["wheel"], args["geometry"], args["radius"], args["point"], args["normal"],
			args["k"], args["c"], args["dmax"], args["lat"], args["circ"])


func sample_index(smp: Dictionary, circ: int) -> int:
	return int(smp["lateral_index"]) * circ + int(smp["angular_index"])


# ---------------------------------------------------------------- independent oracles

## Load (N) of the UPRIGHT wheel at centre height h on the plane y = 0 from the closed form of the circumferential integral: for fixed
## lateral offset s the compression at angle theta = pi + t is r cos(t) - h, positive for |t| < t0 = acos(h / r), and its integral is
## 2 (r sin t0 - h t0). The remaining lateral integral over the meridian angle phi (dA = rho r dphi dtheta) is a midpoint rule with 2000 nodes.
func oracle_upright_load(h: float, k: float) -> float:
	var phi_max: float = asin(0.5 * W / RHO)
	var n: int = 2000
	var dphi: float = 2.0 * phi_max / float(n)
	var total: float = 0.0
	for i in n:
		var phi: float = -phi_max + (float(i) + 0.5) * dphi
		var r: float = R - RHO + RHO * cos(phi)
		if h < r:
			var t0: float = acos(h / r)
			total += RHO * r * dphi * 2.0 * (r * sin(t0) - h * t0)
	return k * total


## Centre height at which the upright wheel carries `load` (bisection on the monotone oracle).
func oracle_equilibrium_height(load: float, k: float) -> float:
	var lo: float = R - 0.2
	var hi: float = R
	for i in 60:
		var mid: float = 0.5 * (lo + hi)
		if oracle_upright_load(mid, k) > load:
			lo = mid
		else:
			hi = mid
	return 0.5 * (lo + hi)


## Force, moment (about the wheel centre, on the wheel) and footprint area of the wheel at centre (0, h, 0) with camber about +Z, from a
## fine midpoint rule of the SURFACE MAP x = p + u s + e r (nphi x nth nodes). Each point is evaluated from the closed forms, not from the sampler.
func oracle_grid(h: float, camber: float, k: float, nphi: int, nth: int) -> Dictionary:
	var q: Quaternion = Quaternion(Vector3(0.0, 0.0, 1.0), camber)
	var u: Vector3 = q * Vector3(1.0, 0.0, 0.0)
	var b1: Vector3 = q * Vector3(0.0, 1.0, 0.0)
	var b2: Vector3 = q * Vector3(0.0, 0.0, 1.0)
	var p0: Vector3 = Vector3(0.0, h, 0.0)
	var phi_max: float = asin(0.5 * W / RHO)
	var dphi: float = 2.0 * phi_max / float(nphi)
	var dth: float = TAU / float(nth)
	var force: float = 0.0
	var moment: Vector3 = Vector3.ZERO
	var area: float = 0.0
	for i in nphi:
		var phi: float = -phi_max + (float(i) + 0.5) * dphi
		var s: float = RHO * sin(phi)
		var r: float = R - RHO + RHO * cos(phi)
		for j in nth:
			var theta: float = (float(j) + 0.5) * dth
			var x: Vector3 = p0 + u * s + (b1 * cos(theta) + b2 * sin(theta)) * r
			var d: float = -x.y
			if d > 0.0:
				var da: float = RHO * r * dphi * dth
				force += k * d * da
				moment += (x - p0).cross(Vector3.UP) * (k * d * da)
				area += da
	return {"force": force, "moment": moment, "area": area}


# ---------------------------------------------------------------- A: validation

func test_a_invalid_parameters_are_rejected() -> void:
	check(try_create({}) != null, "the reference parameters must build a valid patch")
	var wheel_like: SimRigidBody = make_body(100.0, MotorcycleConfig.wheel_inertia(100.0, R, 0.75 * R, W), Vector3(0.0, R, 0.0))
	var cases: Array = [
		["null ground", {"ground": null}],
		["null wheel", {"wheel": null}],
		["dynamic ground", {"ground": wheel_like}],
		["static wheel", {"wheel": make_static()}],
		["null geometry", {"geometry": null}],
		["zero width geometry", {"geometry": TireGeometry.new(0.0, RHO)}],
		["crown below half width", {"geometry": TireGeometry.new(W, 0.05)}],
		["zero wheel radius", {"radius": 0.0}],
		["NaN wheel radius", {"radius": NAN}],
		["NaN plane point", {"point": Vector3(NAN, 0.0, 0.0)}],
		["INF plane point", {"point": Vector3(INF, 0.0, 0.0)}],
		["NaN plane normal", {"normal": Vector3(NAN, 1.0, 0.0)}],
		["non-unit plane normal", {"normal": Vector3(0.0, 2.0, 0.0)}],
		["lateral count 1", {"lat": 1}],
		["lateral count 65", {"lat": 65}],
		["circumferential count 2", {"circ": 2}],
		["circumferential count 257", {"circ": 257}],
		["zero stiffness density", {"k": 0.0}],
		["negative stiffness density", {"k": -1.0}],
		["NaN stiffness density", {"k": NAN}],
		["INF stiffness density", {"k": INF}],
		["stiffness density above the bound", {"k": 1.0e11}],
		["negative damping density", {"c": -1.0}],
		["NaN damping density", {"c": NAN}],
		["damping density above the bound", {"c": 1.0e10}],
		["zero maximum compression", {"dmax": 0.0}],
		["NaN maximum compression", {"dmax": NAN}],
		["maximum compression above half the wheel radius", {"dmax": 0.2}],
	]
	for c in cases:
		check(try_create(c[1]) == null, "invalid input accepted: %s" % c[0])
	# Zero damping is valid (a purely elastic patch) and the plane normal is normalised on creation.
	check(try_create({"c": 0.0}) != null, "zero damping density must be accepted")
	var support = try_create({"normal": Vector3(0.0, 1.0 + 4.0e-6, 0.0)})
	check(support != null and absf(support.plane_normal.length() - 1.0) < 1.0e-7, "the plane normal must be normalised within the unit tolerance")
	# validate() is the single home of the rules: editing a field of a built patch makes it invalid again (SimWorld refuses to step it).
	var built = try_create({})
	built.stiffness_density = NAN
	check(not built.is_valid(), "a patch edited to a non-finite stiffness must report errors")


func test_a_soft_coefficients_follow_the_implicit_spring_damper() -> void:
	var k: float = 800.0
	var c: float = 50.0
	var res: Dictionary = TirePatchNormalSupport.soft_coefficients(k, c, DT)
	# gamma = 1 / (dt (c + dt k)) and bias per gap = dt k gamma = k / (c + dt k).
	check_near(res["gamma"], 1.0 / (DT * (c + DT * k)), 1.0e-12 * res["gamma"], "gamma is not 1/(dt (c + dt k))")
	check_near(res["bias_per_gap"], k / (c + DT * k), 1.0e-9, "bias per gap is not k/(c + dt k)")
	# Closed form of the implicit step: a body of effective mass m (K = 1/m) at gap g with velocity v: lambda = -dt k g - dt (c + dt k) v'.
	# Solving the soft row analytically: v' = (v - K dt k g' ...) -- checked by substituting lambda back into the row equation.
	var m: float = 3.0
	var g: float = -0.01
	var v: float = -0.5
	var gamma: float = res["gamma"]
	var bias: float = res["bias_per_gap"] * g
	var lam: float = -(v + bias) / (1.0 / m + gamma)  # one exact row solve (effective mass 1/(K + gamma))
	var v_new: float = v + lam / m
	check_near(v_new + bias + gamma * lam, 0.0, 1.0e-12, "the soft row equation is not satisfied after the solve")
	check_near(lam, -DT * k * g - DT * (c + DT * k) * v_new, 1.0e-9, "the row impulse is not the implicit-Euler spring-damper impulse")
	var bad: Dictionary = TirePatchNormalSupport.soft_coefficients(NAN, c, DT)
	check(is_nan(bad["gamma"]), "non-finite stiffness must give NaN coefficients")
	check(is_nan(TirePatchNormalSupport.soft_coefficients(k, c, 0.0)["gamma"]), "a zero time step must give NaN coefficients")
	check(is_nan(TirePatchNormalSupport.soft_coefficients(0.0, c, DT)["gamma"]), "zero stiffness must give NaN coefficients")


# ---------------------------------------------------------------- B: no physical support with positive gaps

func test_b_positive_gaps_give_no_physical_support_and_no_speculative_response() -> void:
	for case in [[0.002, 0.0], [0.002, -20.0], [1.0e-6, -5.0], [0.04, -20.0]]:
		var rig: Dictionary = make_rig({"height": R + case[0], "velocity": Vector3(0.0, case[1], 0.0)})
		var s = rig["support"]
		var wheel: SimRigidBody = rig["wheel"]
		var res: Dictionary = TirePatchSampler.sample(wheel.position, wheel.orientation, geo(), R, Vector3.ZERO, Vector3.UP, 9, 48)
		var min_gap: float = INF
		for smp in res["samples"]:
			min_gap = minf(min_gap, smp["gap"])
		check(min_gap > 0.0, "fixture error: every sample gap must be positive (min %s)" % min_gap)
		rig["world"].step()
		var label: String = "gap %s m, vy %s m/s" % [case[0], case[1]]
		check(s.active_sample_count() == 0, "%s: no sample may be physically active" % label)
		check_near(s.normal_load(), 0.0, 0.0, "%s: physical normal load must be zero" % label)
		check(not s.is_touching() and not s.contact_active, "%s: the patch must not report contact" % label)
		check(s.rows.is_empty() and s.carried_row_count() == 0, "%s: no row (so no speculative response) may exist" % label)
		check_near(s.carried_impulse_total(), 0.0, 0.0, "%s: no impulse may be carried" % label)
		check(s.last_reason == "no compressed sample", "%s: the reason must be 'no compressed sample' (got '%s')" % [label, s.last_reason])
		check_near(s.max_penetration(), 0.0, 0.0, "%s: no compression" % label)
		check_vec3_near(wheel.linear_velocity, Vector3(0.0, case[1], 0.0), 1.0e-9, "%s: the wheel must receive no impulse (no collision prevention)" % label)
		check_vec3_near(wheel.angular_velocity, Vector3.ZERO, 1.0e-12, "%s: no angular impulse" % label)
		check(not is_finite(s.centre_of_pressure().x), "%s: no centre of pressure without load" % label)
		check_vec3_near(s.total_force_world(), Vector3.ZERO, 0.0, "%s: no force" % label)


# ---------------------------------------------------------------- C: footprint

func test_c_a_compressed_tire_has_a_distributed_footprint() -> void:
	var rig: Dictionary = heavy_rig(17, 96)
	var s = rig["support"]
	var wheel: SimRigidBody = rig["wheel"]
	var oracle_samples: Dictionary = TirePatchSampler.sample(wheel.position, wheel.orientation, geo(), R, Vector3.ZERO, Vector3.UP, 17, 96)
	var expected: Dictionary = {}
	for smp in oracle_samples["samples"]:
		if smp["gap"] < 0.0:
			expected[sample_index(smp, 96)] = smp
	rig["world"].step()
	var report: Array = s.sample_report()
	check(report.size() > 20, "a %s m compression must activate many samples (got %d)" % [DELTA, report.size()])
	check(s.active_sample_count() == report.size() and s.rows.size() == report.size(), "row and report counts must agree")
	# The active set is exactly the set of samples with a negative gap (independent sampler call), no more, no less.
	check(report.size() == expected.size(), "active count %d, expected %d compressed samples" % [report.size(), expected.size()])
	var lateral: Dictionary = {}
	var angular: Dictionary = {}
	var s_limit: float = sqrt(2.0 * RHO * DELTA - DELTA * DELTA)
	var t_limit: float = acos((R - DELTA) / R)
	var loaded: int = 0
	for entry in report:
		check(expected.has(entry["index"]), "sample %d is active but not compressed" % entry["index"])
		check(entry["gap"] < 0.0 and entry["compression"] > 0.0, "an active sample must have a negative gap")
		if entry["load"] > 0.0:
			loaded += 1
		var smp: Dictionary = expected.get(entry["index"], {})
		if not smp.is_empty():
			check_near(entry["gap"], smp["gap"], 1.0e-9, "reported gap differs from the Stage A sample")
			check_near(entry["area"], smp["weight"], 0.0, "reported area differs from the Stage A weight")
			check_vec3_near(entry["position"], smp["position"], 1.0e-9, "reported position differs from the Stage A sample")
			lateral[smp["lateral_index"]] = true
			angular[smp["angular_index"]] = true
			# Independent closed-form extent of the footprint of this compression: |s|^2 < 2 rho delta - delta^2, |theta - pi| < acos((R - delta)/R).
			check(absf(smp["lateral_offset"]) <= s_limit + 1.0e-9, "an active sample lies outside the closed-form lateral extent")
			check(absf(smp["angle"] - PI) <= t_limit + 1.0e-9, "an active sample lies outside the closed-form circumferential extent")
	check(loaded == report.size(), "every active sample must carry load (%d of %d)" % [loaded, report.size()])
	check(lateral.size() >= 5 and angular.size() >= 5, "the footprint must be finite in both directions (%d lateral, %d circumferential nodes)" % [lateral.size(), angular.size()])
	# A compressed sample outside the set never appears: every sample with gap >= 0 has no row.
	check(s.contact_area() > 0.0 and s.contact_area() < oracle_samples["total_area"], "the footprint is a proper part of the tread area")
	check(s.is_touching(), "a compressed tyre reports contact")


# ---------------------------------------------------------------- D: area weighting

func test_d_sample_loads_scale_with_the_represented_area() -> void:
	for res in [[9, 48], [33, 192]]:
		var rig: Dictionary = heavy_rig(res[0], res[1])
		rig["world"].step()
		var s = rig["support"]
		var areas: Array = []
		var worst: float = 0.0
		for entry in s.sample_report():
			# Heavy wheel at rest: lambda_i = -dt k_i g_i, so the sample load is k_A A_i delta_i (area times pressure k_A delta).
			var expected: float = K_A * entry["area"] * entry["compression"]
			worst = maxf(worst, absf(entry["load"] - expected) / (K_A * entry["area"]))
			check_near(entry["load"], expected, K_A * entry["area"] * GAP_TOL, "sample load is not k_A A delta at resolution %s" % [res])
			areas.append(entry["area"])
		check(worst < 2.0 * GAP_TOL, "the worst sample load error is %s m of compression (> 2 um) at %s" % [worst, res])
		areas.sort()
		# The areas differ (half-weight shoulder nodes and the radius r(s) change the weights), so the test cannot pass with a constant weight.
		check(areas[-1] > 1.05 * areas[0], "the active sample areas must differ to test the scaling (%s .. %s)" % [areas[0], areas[-1]])


func test_d_a_sample_at_the_same_place_carries_load_in_proportion_to_its_area() -> void:
	# The crown-centre sample (s = 0, theta = pi) exists in the 9 x 48 and in the 33 x 192 grid with the SAME gap (the pose is the same),
	# so it carries the same pressure k_A delta: its load is proportional to its represented area, i.e. to dphi dtheta, a factor 4 x 4 = 16.
	var coarse: Dictionary = heavy_rig(9, 48)
	var fine: Dictionary = heavy_rig(33, 192)
	coarse["world"].step()
	fine["world"].step()
	var a: Dictionary = {}
	var b: Dictionary = {}
	for entry in coarse["support"].sample_report():
		if entry["index"] == 4 * 48 + 24:
			a = entry
	for entry in fine["support"].sample_report():
		if entry["index"] == 16 * 192 + 96:
			b = entry
	check(not a.is_empty() and not b.is_empty(), "fixture error: the crown-centre sample must be active in both grids")
	if a.is_empty() or b.is_empty():
		return
	check_near(a["compression"], b["compression"], 2.0e-7, "the crown-centre sample must have the same compression in both grids")
	check_near(a["compression"], DELTA, 1.0e-6, "the crown-centre compression must be the fixture compression")
	check_near(a["area"] / b["area"], 16.0, 1.0e-9, "the weights of the two grids must differ by dphi dtheta = 4 x 4")
	check_near(a["load"] / b["load"], 16.0, 2.0e-4 * 16.0, "the same sample at the same place must carry load in proportion to its area")
	check_near(a["load"], K_A * a["area"] * DELTA, K_A * a["area"] * GAP_TOL, "the coarse crown sample must carry k_A A delta")


func test_d_doubling_the_stiffness_density_doubles_every_sample_and_the_aggregate() -> void:
	var one: Dictionary = heavy_rig(9, 48)
	var two: Dictionary = heavy_rig(9, 48, 0.0, DELTA, {"k": 2.0 * K_A})
	one["world"].step()
	two["world"].step()
	check(one["support"].active_sample_count() == two["support"].active_sample_count(), "stiffness must not change the footprint")
	check_near(two["support"].normal_load() / one["support"].normal_load(), 2.0, 2.0e-5, "the aggregate load must scale with the stiffness density")
	var a: Array = one["support"].sample_report()
	var b: Array = two["support"].sample_report()
	for i in a.size():
		check_near(b[i]["load"], 2.0 * a[i]["load"], 2.0e-5 * a[i]["load"] + 1.0e-9, "sample %d does not scale with k_A" % a[i]["index"])


# ---------------------------------------------------------------- E: direction and application points

func test_e_normal_response_acts_along_the_plane_normal_at_the_sample_positions() -> void:
	# Inclined plane (0.17 rad about Z) so that the ground normal is not the world axis; the wheel is upright relative to the plane.
	var a: float = 0.17
	var q: Quaternion = Quaternion(Vector3(0.0, 0.0, 1.0), a)
	var n: Vector3 = q * Vector3.UP
	var tangent: Vector3 = Vector3(0.0, 0.0, 1.0)
	check_near(n.dot(tangent), 0.0, 1.0e-12, "fixture error: the in-plane direction must be perpendicular to the normal")
	var rig: Dictionary = make_rig({"orientation": q, "position": n * (R - DELTA), "normal": n, "lat": 9, "circ": 48,
		"velocity": tangent * 3.0, "angular_velocity": -(q * Vector3.RIGHT) * 40.0})
	var wheel: SimRigidBody = rig["wheel"]
	var ground: SimRigidBody = rig["ground"]
	var s = rig["support"]
	var oracle_samples: Dictionary = TirePatchSampler.sample(wheel.position, wheel.orientation, geo(), R, Vector3.ZERO, n, 9, 48)
	var by_index: Dictionary = {}
	for smp in oracle_samples["samples"]:
		by_index[sample_index(smp, 48)] = smp
	var u: Vector3 = (q * Vector3.RIGHT).normalized()
	var v0: Vector3 = wheel.linear_velocity
	var w0: Vector3 = wheel.angular_velocity
	var pos0: Vector3 = wheel.position
	rig["world"].step()
	var report: Array = s.sample_report()
	check(report.size() > 5, "fixture error: the inclined wheel must have a footprint")
	for k in report.size():
		var entry: Dictionary = report[k]
		var row = s.rows[k]
		var smp: Dictionary = by_index[entry["index"]]
		check_vec3_near(row.linear_b, n, 1.0e-6, "the wheel-side row direction is not the ground normal")
		check_vec3_near(row.linear_a, -n, 1.0e-6, "the ground-side row direction is not minus the ground normal")
		# The lever arm is the ACTUAL sample position relative to the wheel centre (independent Stage A position): the full moment of a normal
		# impulse there, with nothing removed (the axle component is part of the wrench; the spin is excluded from the velocity-side Jacobian only, see test_n_*).
		var arm: Vector3 = (smp["position"] - pos0).cross(n)
		check_vec3_near(row.angular_b, arm, 1.0e-6, "the row does not act at the sample position")
		check_vec3_near(row.angular_a, -(smp["position"] - ground.position).cross(n), 1.0e-6, "the ground-side lever arm is not the sample position")
		check_near(row.min_impulse, 0.0, 0.0, "the sample row must be unilateral (lower bound 0)")
		check(row.max_impulse == INF, "the sample row must have no upper bound")
	# The wheel's velocity change is along the normal: no tangential impulse exists, the in-plane motion and the spin are untouched.
	var dv: Vector3 = wheel.linear_velocity - v0
	check(dv.dot(n) > 0.0, "the support must push the wheel away from the plane")
	check_near(dv.dot(tangent), 0.0, 1.0e-9, "no tangential velocity change (the patch has no friction)")
	check_near(dv.cross(n).length(), 0.0, 1.0e-6 * dv.length(), "the velocity change is not along the ground normal")
	# The quasi-static footprint is fore-aft symmetric, so the true moment about the axle vanishes. Two independent checks:
	# (1) the moment of the sample loads about the axle (no inertia, no stored spin involved): zero up to the rounding of the sample loads
	#     (measured 4e-8 N R; bound 1e-7 N R);
	# (2) the change of the stored spin, measured as an angular impulse I_axial d(omega . u). The wheel spins at 40 rad/s, so the stored world
	#     angular velocity resolves only 2^-23 * 40 = 4.8e-6 rad/s (one float32 ulp at 40); the measured change is 1.1e-5 rad/s (about 3 ulp of
	#     rounding, even in the spin and zero without spin), i.e. 8.6e-5 N m s. The bound is 6 ulp of the spin times the axial inertia (2.1e-4),
	#     still 1000 times below the 0.24 N m s a real 13 % axle moment would deliver. The previous bound (2e-6 N R dt = 3.7e-6 N m s) held only
	#     because the rounding happened to be 5e-7 N R dt with the earlier row arithmetic; it is a stored-spin resolution, not a physical effect.
	check_near(s.applied_wheel_moment().dot(u), 0.0, 1.0e-7 * s.normal_load() * R, "the loads of the symmetric footprint have no moment about the axle")
	var axial_inertia: float = MotorcycleConfig.wheel_inertia(100.0, R, 0.75 * R, W).x.x
	var axle_impulse: float = axial_inertia * (wheel.angular_velocity - w0).dot(u)
	check_near(axle_impulse, 0.0, 6.0 * absf(w0.dot(u)) * pow(2.0, -23.0) * axial_inertia, "no change of the spin beyond the float32 resolution of the stored spin")
	check_vec3_near(ground.linear_velocity, Vector3.ZERO, 0.0, "the static ground must take no velocity")
	check_vec3_near(ground.angular_velocity, Vector3.ZERO, 0.0, "the static ground must take no angular velocity")


func test_e_spin_does_not_change_the_normal_support() -> void:
	# The tread is a surface of revolution: a spin about the axle must neither load the front of the footprint more than the rear nor change
	# the load. The wrench keeps the full angular Jacobian; the tangent tread velocity is removed from the compression rate by the velocity-side
	# Jacobian of the row (header of the constraint; test_n_*). Without it the load grows +27 % / +76 % (c_A = 0) and +185 % / +412 % (c_A = 1e5) at 30 / 60 rad/s
	# and the moment about the axle is 13 % to 20 % of N R. Tolerances: the load is bit-for-bit independent of the spin up to float32
	# rounding of the sample arithmetic (1e-9 relative); the true axle moment of the symmetric footprint vanishes up to that rounding
	# (measured up to 1e-8 N R; bound 1e-7 N R; it was exactly 0 only while the axle component was cut out of the wrench).
	var base: Dictionary = {}
	for c in [0.0, C_A]:
		for spin in [0.0, 30.0, 60.0]:
			var rig: Dictionary = heavy_rig(17, 96, 0.0, DELTA, {"c": c, "angular_velocity": Vector3(-spin, 0.0, 0.0), "velocity": Vector3(0.0, 0.0, -spin * R)})
			var p0: Vector3 = rig["wheel"].position
			rig["world"].step()
			var s = rig["support"]
			var label: String = "c %s, spin %s rad/s" % [c, spin]
			if spin == 0.0:
				base = {"load": s.normal_load(), "count": s.active_sample_count()}
			else:
				check_near(s.normal_load(), base["load"], 1.0e-9 * base["load"], "%s: the spin changed the normal load" % label)
				check(s.active_sample_count() == base["count"], "%s: the spin changed the footprint" % label)
			check_near(s.applied_wheel_moment().dot(Vector3.RIGHT), 0.0, 1.0e-7 * s.normal_load() * R, "%s: axle moment of the symmetric footprint" % label)
			check_near(s.total_moment_world(p0).dot(Vector3.RIGHT), 0.0, 1.0e-7 * s.normal_load() * R, "%s: geometric axle moment of the symmetric footprint" % label)
			check_near(rig["wheel"].angular_velocity.x, -spin, 1.0e-9 + 1.0e-9 * spin, "%s: the spin changed" % label)


## Row convention, independent of the solver dynamics: ConstraintRow.apply_impulse(lambda) is the solver's own impulse application
## (domega = I_w^-1 J_ang lambda). Prescribed, deliberately ASYMMETRIC sample impulses (fore-aft and lateral gradients) are applied
## to the wheel through the installed rows; the resulting angular impulse I_w domega must equal the moment of the normal impulses at the
## ACTUAL sample positions, sum_i (x_i - p) x n lambda_i, computed here from the independent Stage A positions. The fore-aft gradient gives a
## non-zero component along the axle (= -z offset times lambda for n = +Y, u = +X); the axle-projected rows of the first Stage B version
## drop exactly that component, so this test fails for them (measured: the expected axle component is 18 % of |L|, the projected one 0).
func test_e_row_angular_impulse_is_the_moment_of_the_normal_impulse_at_the_sample() -> void:
	for camber in [0.0, 0.15]:
		var rig: Dictionary = make_rig({"mass": 100.0, "height": R - DELTA * cos(camber), "camber": camber, "lat": 9, "circ": 48})
		var wheel: SimRigidBody = rig["wheel"]
		var s = rig["support"]
		var label: String = "camber %s" % camber
		var p0: Vector3 = wheel.position
		var oracle: Dictionary = TirePatchSampler.sample(p0, wheel.orientation, geo(), R, Vector3.ZERO, Vector3.UP, 9, 48)
		var by_index: Dictionary = {}
		for smp in oracle["samples"]:
			by_index[sample_index(smp, 48)] = smp
		var u: Vector3 = (wheel.orientation * Vector3.RIGHT).normalized()
		s.prepare(DT, rig["world"].config)
		check(s.rows.size() > 8, "%s: fixture error: the footprint must hold several samples" % label)
		var lateral_positions: Dictionary = {}
		var expected_impulse: Vector3 = Vector3.ZERO
		var expected_angular: Vector3 = Vector3.ZERO
		var report: Array = s.sample_report()
		for k in s.rows.size():
			var pos: Vector3 = by_index[report[k]["index"]]["position"]
			var arm: Vector3 = pos - p0
			lateral_positions[snappedf(arm.dot(u), 1.0e-4)] = true
			# Asymmetric positive impulses: a load-like base with fore-aft (z) and lateral (x) gradients.
			var lambda: float = report[k]["area"] * (-report[k]["gap"]) * 1.0e6 * (1.0 - 4.0 * arm.z + 3.0 * arm.x)
			check(lambda > 0.0, "%s: fixture error: impulses must stay positive" % label)
			s.rows[k].accumulated_impulse = lambda
			s.rows[k].apply_impulse(lambda)
			expected_impulse += Vector3.UP * lambda
			expected_angular += arm.cross(Vector3.UP) * lambda
		check(lateral_positions.size() >= 3, "%s: the active samples must sit at several lateral positions" % label)
		check(absf(expected_angular.dot(u)) > 0.1 * expected_angular.length(), "%s: fixture error: the expected moment must have an axle component" % label)
		var inertia: Basis = wheel.inverse_inertia_world.inverse()
		var angular_impulse: Vector3 = inertia * wheel.angular_velocity
		check_vec3_near(angular_impulse, expected_angular, 1.0e-4 * expected_angular.length(), "%s: the solver's angular impulse is not the moment of the normal impulses" % label)
		check_near(angular_impulse.dot(u), expected_angular.dot(u), 1.0e-4 * expected_angular.length(), "%s: axle component of the applied angular impulse" % label)
		check_vec3_near(wheel.linear_velocity * wheel.mass, expected_impulse, 1.0e-4 * expected_impulse.length(), "%s: linear impulse" % label)
		# The reported moment is the same quantity: applied_wheel_moment() * dt (loads = impulse / dt), and the geometric one agrees.
		s.finish()
		check_vec3_near(s.applied_wheel_moment() * DT, angular_impulse, 1.0e-4 * angular_impulse.length(), "%s: reported applied moment differs from the solver's" % label)
		check_vec3_near(s.total_moment_world(p0) * DT, angular_impulse, 1.0e-4 * angular_impulse.length(), "%s: geometric moment differs from the solver's" % label)


## End-to-end, with REAL loads from the solver: the angular velocity change of one step (I_w domega, the solver's own impulse application
## including the soft rows) equals the moment of the sample loads (from sample_report) about the start-of-step centre, with the independent
## sampler positions, for a cambered, yawed and translating wheel with ZERO angular velocity (a spinning heavy wheel would add the solver's gyroscopic update to
## the comparison); the diagnostics agree.
func test_e_solver_applied_moment_matches_the_geometric_moment_of_the_sample_loads() -> void:
	var q: Quaternion = Quaternion(Vector3(0.0, 1.0, 0.0), 0.4) * Quaternion(Vector3(0.0, 0.0, 1.0), 0.15)
	var rig: Dictionary = make_rig({"mass": HEAVY, "orientation": q, "position": Vector3(0.1, R - 0.018, -0.2), "lat": 17, "circ": 96,
		"velocity": Vector3(0.5, -0.3, -1.0)})
	var wheel: SimRigidBody = rig["wheel"]
	var s = rig["support"]
	var p0: Vector3 = wheel.position
	var oracle: Dictionary = TirePatchSampler.sample(p0, wheel.orientation, geo(), R, Vector3.ZERO, Vector3.UP, 17, 96)
	var by_index: Dictionary = {}
	for smp in oracle["samples"]:
		by_index[sample_index(smp, 96)] = smp
	var inertia: Basis = wheel.inverse_inertia_world.inverse()
	var l0: Vector3 = Vector3.ZERO
	rig["world"].step()
	var report: Array = s.sample_report()
	check(report.size() > 20 and s.normal_load() > 0.0, "fixture error: the tilted wheel must be loaded")
	var expected: Vector3 = Vector3.ZERO
	for entry in report:
		expected += (by_index[entry["index"]]["position"] - p0).cross(Vector3.UP) * entry["load"] * DT
	var delta_l: Vector3 = wheel.inverse_inertia_world.inverse() * wheel.angular_velocity - l0
	# World inertia is evaluated at the start/end orientation (the heavy wheel turns by ~1e-7 rad), so the comparison is exact to float32.
	check_vec3_near(delta_l, expected, 1.0e-3 * expected.length() + 1.0e-6, "the solver's angular impulse is not the moment of the sample loads")
	check(expected.length() > 0.1, "fixture error: the moment must be substantial")
	check_vec3_near(s.applied_wheel_moment() * DT, expected, 1.0e-4 * expected.length(), "applied_wheel_moment differs from the independent moment")
	check_vec3_near(s.total_moment_world(p0) * DT, expected, 1.0e-4 * expected.length(), "total_moment_world differs from the independent moment")


# ---------------------------------------------------------------- F: aggregate force and moment

func test_f_aggregate_force_and_moment_match_the_independent_integral() -> void:
	for camber in [0.0, 0.15]:
		var h: float = R - DELTA * cos(camber)
		var oracle: Dictionary = oracle_grid(h, camber, K_A, 100, 720)
		if camber == 0.0:
			check_near(oracle["force"], oracle_upright_load(h, K_A), 1.0e-3 * oracle["force"], "the two independent oracles disagree for the upright wheel")
		var rig: Dictionary = heavy_rig(33, 192, camber)
		var wheel: SimRigidBody = rig["wheel"]
		rig["world"].step()
		var s = rig["support"]
		var label: String = "camber %s" % camber
		# Measured on this fixture: force error 0.06 % (upright) and 0.04 % (camber), moment error 0.03 %; bounds 0.3 % and 1 %.
		check_near(s.normal_load(), oracle["force"], 3.0e-3 * oracle["force"], "%s: aggregate load differs from the independent integral" % label)
		check_vec3_near(s.total_force_world(), Vector3.UP * s.normal_load(), 0.0, "%s: the force must be along the ground normal" % label)
		var moment: Vector3 = s.total_moment_world(wheel.position)
		# A leaned wheel: the load acts away from the wheel centre line, so the moment is large (about 21 N m); the upright moment vanishes by symmetry.
		check_near(moment.z, oracle["moment"].z, maxf(1.0e-2 * absf(oracle["moment"].z), 1.0e-3), "%s: moment about the longitudinal axis differs" % label)
		check_near(moment.x, 0.0, 1.0e-2, "%s: moment about the axle must vanish by symmetry" % label)
		check_near(moment.y, 0.0, 1.0e-3, "%s: vertical moment must vanish" % label)
		if camber > 0.0:
			check(absf(moment.z) > 5.0, "a leaned wheel must produce a lateral moment (got %s N m)" % moment.z)
		else:
			check(absf(moment.z) < 1.0e-3, "the upright wheel must produce no lateral moment (got %s N m)" % moment.z)
		# The load line passes through the centre of pressure: M = (c - p) x F, so the lateral offset of the centre of pressure equals M / N.
		var cop: Vector3 = s.centre_of_pressure()
		var from_cop: Vector3 = (cop - wheel.position).cross(Vector3.UP) * s.normal_load()
		check_vec3_near(from_cop, moment, 1.0e-3 * (moment.length() + 1.0), "%s: the centre of pressure does not reproduce the moment" % label)
		# What the wheel receives: Delta omega = I^-1 M dt (the heavy wheel's velocities are exact floats near zero).
		var applied: Vector3 = s.applied_wheel_moment()
		var angular_impulse: Vector3 = wheel.inverse_inertia_world.inverse() * wheel.angular_velocity
		check_vec3_near(angular_impulse, applied * DT, 2.0e-3 * (applied.length() * DT) + 1.0e-6, "%s: the wheel did not receive the reported moment" % label)
		check_near(wheel.linear_velocity.y * HEAVY, s.normal_load() * DT, 1.0e-4 * s.normal_load() * DT, "%s: the wheel did not receive the reported impulse" % label)


# ---------------------------------------------------------------- G: unilateral response

func test_g_the_response_is_unilateral_under_all_tested_motions() -> void:
	# Penetration 1 cm; vertical velocity from fast approach to fast separation (the damper would pull for separation without the bound).
	for vy in [-3.0, -0.2, 0.0, 0.2, 3.0, 10.0, 50.0]:
		var rig: Dictionary = make_rig({"height": R - 0.01, "velocity": Vector3(0.0, vy, 0.0)})
		var wheel: SimRigidBody = rig["wheel"]
		var s = rig["support"]
		rig["world"].step()
		var label: String = "vy %s m/s" % vy
		for entry in s.sample_report():
			check(entry["load"] >= 0.0, "%s: a sample load is negative (%s N): the ground pulled" % [label, entry["load"]])
		for row in s.rows:
			check(row.accumulated_impulse >= 0.0, "%s: an accumulated sample impulse is negative" % label)
		check(s.normal_load() >= 0.0, "%s: the aggregate load is negative" % label)
		check(wheel.linear_velocity.y >= vy - 1.0e-9, "%s: the support decelerated a separating or resting wheel into the ground (vy %s -> %s)" % [label, vy, wheel.linear_velocity.y])
	# A wheel separating faster than the springs can recover is not touched at all.
	var fast: Dictionary = make_rig({"height": R - 0.01, "velocity": Vector3(0.0, 50.0, 0.0)})
	fast["world"].step()
	check_near(fast["support"].normal_load(), 0.0, 0.0, "a fast separating wheel must carry no load")
	check_near(fast["wheel"].linear_velocity.y, 50.0, 1.0e-9, "a fast separating wheel must not be braked by the damper")
	# Approaching: positive load larger than the static spring load (the damper adds).
	var still: Dictionary = make_rig({"height": R - 0.01})
	var approaching: Dictionary = make_rig({"height": R - 0.01, "velocity": Vector3(0.0, -0.5, 0.0)})
	still["world"].step()
	approaching["world"].step()
	check(approaching["support"].normal_load() > still["support"].normal_load(), "approach must add damping load")


# ---------------------------------------------------------------- H: settling and equilibrium

func test_h_a_loaded_wheel_settles_at_the_equilibrium_compression() -> void:
	var rig: Dictionary = make_rig({"gravity": G, "fixture": true, "lat": 17, "circ": 96})
	var wheel: SimRigidBody = rig["wheel"]
	var s = rig["support"]
	var min_sample: float = INF
	var peak: float = 0.0
	var bottomed: int = 0
	for i in 480:
		rig["world"].step()
		for entry in s.sample_report():
			min_sample = minf(min_sample, entry["load"])
		peak = maxf(peak, R - wheel.position.y)
		bottomed = maxi(bottomed, s.bottomed_sample_count())
	var weight: float = 100.0 * G
	check(min_sample >= 0.0, "a sample load was negative during settling (%s)" % min_sample)
	check(bottomed == 0, "a free settling from touching must not reach the hard stop (peak compression %s m)" % peak)
	check(peak < D_MAX, "the peak compression %s m must stay below the bottoming compression" % peak)
	check_near(s.normal_load(), weight, 1.0e-3 * weight, "the settled load must carry the weight")
	check_near(wheel.linear_velocity.y, 0.0, 1.0e-4, "the wheel must come to rest")
	# Equilibrium compression of the independent oracle (the 17 x 96 quadrature is within 0.3 % in load, so about 0.15 % in compression).
	var h_eq: float = oracle_equilibrium_height(weight, K_A)
	check_near(wheel.position.y, h_eq, 2.0e-4, "the settled height differs from the independent equilibrium")
	check(s.carried_row_count() == s.active_sample_count() and s.active_sample_count() > 20, "settled footprint state is inconsistent")


# ---------------------------------------------------------------- I: excessive penetration

func test_i_penetration_is_bounded_by_the_hard_stop() -> void:
	for vy in [-1.5, -3.0, -6.0, -10.0]:
		var rig: Dictionary = make_rig({"gravity": G, "fixture": true, "velocity": Vector3(0.0, vy, 0.0)})
		var wheel: SimRigidBody = rig["wheel"]
		var s = rig["support"]
		var deepest: float = 0.0
		var bottomed: int = 0
		var min_load: float = INF
		for i in 480:
			rig["world"].step()
			deepest = maxf(deepest, R - wheel.position.y)
			bottomed = maxi(bottomed, s.bottomed_sample_count())
			min_load = minf(min_load, s.normal_load())
			check(wheel.position.is_finite() and wheel.linear_velocity.is_finite(), "non-finite state during the impact at %s m/s" % vy)
		var label: String = "impact %s m/s" % vy
		# A sample is classified from the start-of-step gap: the wheel can pass the hard-stop depth by one step of approach before the stop acts.
		check(deepest <= D_MAX + absf(vy) * DT + 1.0e-3, "%s: penetration %s m exceeds the hard stop %s plus one step of approach %s" % [label, deepest, D_MAX, absf(vy) * DT])
		check(min_load >= 0.0, "%s: negative aggregate load" % label)
		if absf(vy) >= 6.0:
			check(bottomed > 0, "%s: the hard stop never engaged" % label)
		if absf(vy) <= 1.5:
			check(bottomed == 0, "%s: a gentle impact must be absorbed by the compliance alone" % label)
		# After the impact the wheel rests on the compliant support again (the hard stop does not trap it).
		check_near(s.normal_load(), 100.0 * G, 2.0e-3 * 100.0 * G, "%s: the wheel does not settle on the compliant support" % label)
		check(s.bottomed_sample_count() == 0, "%s: samples are still bottomed at rest" % label)
	# Without the stop a very low limit still bounds the depth: bottoming at 5 mm.
	var tight: Dictionary = make_rig({"gravity": G, "fixture": true, "velocity": Vector3(0.0, -1.5, 0.0), "dmax": 0.005})
	var deepest_tight: float = 0.0
	for i in 240:
		tight["world"].step()
		deepest_tight = maxf(deepest_tight, R - tight["wheel"].position.y)
	check(deepest_tight <= 0.005 + 1.5 * DT + 1.0e-3, "a 5 mm hard stop must bound the penetration (got %s m)" % deepest_tight)


# ---------------------------------------------------------------- J: separation, stale state, re-contact

func test_j_complete_separation_clears_every_stale_state_and_recontact_is_cold() -> void:
	var rig: Dictionary = make_rig({"gravity": G, "height": R - 0.01})
	var wheel: SimRigidBody = rig["wheel"]
	var s = rig["support"]
	for i in 30:
		rig["world"].step()
	check(s.normal_load() > 100.0 and s.carried_impulse_total() > 0.1 and s.carried_row_count() > 5,
			"fixture error: the loaded patch must carry state before the separation")
	# Lift the wheel clear of the ground (all gaps positive) with an upward velocity.
	wheel.position.y += 0.1
	wheel.linear_velocity = Vector3(0.0, 1.0, 0.0)
	rig["world"].step()
	check_near(s.normal_load(), 0.0, 0.0, "physical load must be zero after complete separation")
	check(s.active_sample_count() == 0 and s.sample_report().is_empty(), "no physical sample may remain active")
	check(not s.contact_active and not s.is_touching(), "separation must deactivate the patch")
	check(s.carried_row_count() == 0, "separation must drop every row")
	check_near(s.carried_impulse_total(), 0.0, 0.0, "separation must clear the carried impulses")
	check_near(wheel.linear_velocity.y, 1.0 - G * DT, 1.0e-6, "no stale impulse may act on the separated wheel (gravity only)")
	check_vec3_near(s.total_force_world(), Vector3.ZERO, 0.0, "no force after separation")
	check_vec3_near(s.applied_wheel_moment(), Vector3.ZERO, 0.0, "no moment after separation")
	# Many steps in the air keep the state clear.
	for i in 5:
		rig["world"].step()
	check(s.carried_row_count() == 0 and s.normal_load() == 0.0, "the separated patch must stay clear")
	# Re-contact: the same state as a freshly built patch must give the same first-step response (nothing stale survives).
	wheel.position = Vector3(0.0, R - 0.01, 0.0)
	wheel.orientation = Quaternion.IDENTITY
	wheel.linear_velocity = Vector3.ZERO
	wheel.angular_velocity = Vector3.ZERO
	check(s.carried_row_count() == 0, "the patch must be cold before the re-contact")
	rig["world"].step()
	var twin: Dictionary = make_rig({"gravity": G, "height": R - 0.01})
	twin["world"].step()
	check(s.normal_load() > 0.0 and is_finite(s.normal_load()), "re-contact must produce valid support")
	check_near(s.normal_load(), twin["support"].normal_load(), 1.0e-9 * s.normal_load(), "re-contact differs from a fresh patch: stale state survived")
	check_vec3_near(wheel.linear_velocity, twin["wheel"].linear_velocity, 1.0e-9, "re-contact velocity differs from a fresh patch")
	var a: Array = s.sample_report()
	var b: Array = twin["support"].sample_report()
	check(a.size() == b.size() and a.size() > 5, "re-contact footprint differs from a fresh patch")
	for i in mini(a.size(), b.size()):
		check_near(a[i]["load"], b[i]["load"], 1.0e-9 * (b[i]["load"] + 1.0), "re-contact sample %d differs from a fresh patch" % i)


func test_j_partial_separation_drops_only_the_samples_that_left_the_ground() -> void:
	var rig: Dictionary = make_rig({"gravity": G, "height": R - 0.012})
	var wheel: SimRigidBody = rig["wheel"]
	var s = rig["support"]
	for i in 20:
		rig["world"].step()
	var before: Dictionary = {}
	for entry in s.sample_report():
		before[entry["index"]] = true
	check(before.size() > 10, "fixture error: the footprint must hold many samples")
	# Raise by 8 mm: the footprint shrinks but the tyre stays loaded.
	wheel.position.y += 0.008
	wheel.linear_velocity = Vector3.ZERO
	rig["world"].step()
	var after: Dictionary = {}
	for entry in s.sample_report():
		after[entry["index"]] = true
	check(after.size() > 0 and after.size() < before.size(), "the footprint must shrink but remain (%d -> %d samples)" % [before.size(), after.size()])
	for index in after:
		check(before.has(index), "a sample became active that was not before")
	check(s.carried_row_count() == after.size(), "only the still-compressed samples may keep a row (%d rows, %d active)" % [s.carried_row_count(), after.size()])
	for index in before:
		if not after.has(index):
			check_near(s.carried_impulse(index), 0.0, 0.0, "sample %d left the ground but its impulse was kept" % index)
	var kept_total: float = 0.0
	for index in after:
		kept_total += s.carried_impulse(index)
	check(kept_total > 0.0, "the samples that stay in contact must keep (their own) accumulated impulse")
	check(s.normal_load() > 0.0, "the partially separated tyre must still carry load")


# ---------------------------------------------------------------- K: sampling-density independence

func test_k_aggregate_support_converges_with_the_sampling_density() -> void:
	var resolutions: Array = [[5, 24], [9, 48], [17, 96], [33, 192], [64, 256]]
	for camber in [0.0, 0.15]:
		var h: float = R - DELTA * cos(camber)
		var oracle: Dictionary = oracle_grid(h, camber, K_A, 100, 720)
		var errors: Array = []
		var loads: Array = []
		var naive: Array = []
		var counts: Array = []
		for res in resolutions:
			var rig: Dictionary = heavy_rig(res[0], res[1], camber)
			var wheel: SimRigidBody = rig["wheel"]
			# What a fixed per-sample spring would give: k_A times the sum of the compressions, independent of the represented area.
			var raw: Dictionary = TirePatchSampler.sample(wheel.position, wheel.orientation, geo(), R, Vector3.ZERO, Vector3.UP, res[0], res[1])
			var fixed_spring: float = 0.0
			for smp in raw["samples"]:
				if smp["gap"] < 0.0:
					fixed_spring += K_A * -smp["gap"]
			rig["world"].step()
			var s = rig["support"]
			loads.append(s.normal_load())
			errors.append(absf(s.normal_load() - oracle["force"]) / oracle["force"])
			naive.append(fixed_spring)
			counts.append(raw["samples"].size())
			if camber > 0.0:
				var m_err: float = absf(s.total_moment_world(wheel.position).z - oracle["moment"].z) / absf(oracle["moment"].z)
				# Moment errors measured: 5x24 4.0 %, 9x48 3.2 %, 17x96 0.3 %, 33x192 0.03 %, 64x256 0.02 %.
				check(m_err < [0.06, 0.05, 0.01, 0.002, 0.001][loads.size() - 1], "camber moment error %s too large at resolution %s" % [m_err, res])
		var label: String = "camber %s" % camber
		# Measured load errors vs the independent integral (upright / camber): 5x24 2.3 / 1.6 %, 9x48 1.3 / 2.3 %, 17x96 0.2 / 0.25 %,
		# 33x192 0.06 / 0.04 %, 64x256 0.014 / 0.005 %. The bounds are about 1.3 to 2 times the measured values.
		var bounds: Array = [0.04, 0.03, 0.005, 0.0012, 0.0005]
		for i in resolutions.size():
			check(errors[i] < bounds[i], "%s: load error %s at %s exceeds %s" % [label, errors[i], resolutions[i], bounds[i]])
		# Convergence: the error of the finer grids is smaller than the one of the 9 x 48 grid and shrinks with every refinement from there.
		check(errors[2] < errors[1] and errors[3] < errors[2] and errors[4] < errors[3], "%s: the error must decrease with every refinement from 9x48 on (%s)" % [label, errors])
		# The sample count grows by a factor 136 (120 -> 16384) but the load changes by less than 6 %: it does not scale with the count.
		var ratio: float = float(counts[4]) / float(counts[0])
		var spread: float = (loads.max() - loads.min()) / loads.min()
		check(ratio > 100.0 and spread < 0.06, "%s: load spread %s over a sample count ratio %s" % [label, spread, ratio])
		# A fixed per-sample spring would multiply the support with the number of active samples (control, from the independent samples).
		check(naive[4] / naive[0] > 20.0, "%s: the fixed-spring control must scale with the sample count (%s)" % [label, naive[4] / naive[0]])
		check(loads[4] / loads[0] < 1.06 and loads[4] / loads[0] > 0.94, "%s: the area-weighted load changed with the density (%s)" % [label, loads[4] / loads[0]])


func test_k_damping_load_is_density_independent_too() -> void:
	# Heavy wheel approaching at 1 m/s: lambda_i / dt = k delta_i + (c + dt k) |v| per unit area weight, so
	# N = k integral(delta dA) + (c_A + dt k_A) |v| A_footprint. The footprint area converges first order (indicator function).
	var oracle: Dictionary = oracle_grid(R - DELTA, 0.0, K_A, 200, 1440)
	var expected: float = oracle["force"] + (C_A + DT * K_A) * 1.0 * oracle["area"]
	var results: Array = []
	for res in [[9, 48], [17, 96], [33, 192], [64, 256]]:
		var rig: Dictionary = heavy_rig(res[0], res[1], 0.0, DELTA, {"velocity": Vector3(0.0, -1.0, 0.0)})
		rig["world"].step()
		results.append(rig["support"].normal_load())
	var bounds: Array = [0.06, 0.03, 0.01, 0.005]
	for i in results.size():
		check(absf(results[i] - expected) / expected < bounds[i], "approach load %s differs from the integral %s by more than %s at resolution %d" % [results[i], expected, bounds[i], i])
	check(results[0] > 0.0 and (results.max() - results.min()) / results.min() < 0.08, "the damping load must not scale with the sample count")
	# The damping term itself is large (c_A |v| A is about 2.2 kN against 0.9 kN of spring load), so this test is sensitive to the area weighting.
	check(expected > 2.0 * oracle["force"], "fixture error: the damping term must dominate")


func test_k_angular_ripple_of_wheel_fixed_samples_shrinks_with_the_circumferential_count() -> void:
	# The samples rotate with the wheel (Stage A), so as the wheel spins the footprint boundary crosses the grid: the load of a fixed pose
	# varies with the spin angle. Measured spread over a one-step spin offset (max - min) / oracle: 9x48 3.9 %, 17x96 0.10 %, 33x192 0.034 %.
	var oracle: float = oracle_upright_load(R - DELTA, K_A)
	var spreads: Array = []
	for res in [[9, 48], [17, 96], [33, 192]]:
		var lo: float = INF
		var hi: float = -INF
		for k in 9:
			var alpha: float = TAU / float(res[1]) * float(k) / 8.0
			var rig: Dictionary = heavy_rig(res[0], res[1], 0.0, DELTA, {"orientation": Quaternion(Vector3(1.0, 0.0, 0.0), alpha)})
			rig["world"].step()
			lo = minf(lo, rig["support"].normal_load())
			hi = maxf(hi, rig["support"].normal_load())
		spreads.append((hi - lo) / oracle)
		check(lo > 0.97 * oracle and hi < 1.03 * oracle, "the load at resolution %s leaves the 3 percent band around the integral (%s .. %s of %s)" % [res, lo, hi, oracle])
	check(spreads[0] < 0.05 and spreads[1] < 0.002 and spreads[2] < 0.001, "angular ripple too large: %s" % [spreads])
	check(spreads[1] < spreads[0] and spreads[2] < spreads[1], "angular ripple must shrink with the circumferential count: %s" % [spreads])


# ---------------------------------------------------------------- L: invalid runtime state

func test_l_invalid_runtime_state_fails_safely_and_clears_the_patch() -> void:
	var rig: Dictionary = make_rig({"gravity": G, "height": R - 0.01})
	var wheel: SimRigidBody = rig["wheel"]
	var s = rig["support"]
	for i in 10:
		rig["world"].step()
	check(s.normal_load() > 0.0 and s.carried_row_count() > 0, "fixture error: loaded patch expected")
	var config = rig["world"].config
	var velocity: Vector3 = wheel.linear_velocity
	# (A non-unit orientation cannot reach the patch: SimRigidBody normalises on assignment; the sampler's own unit check is a Stage A test.)
	var bad_poses: Array = [
		["NaN position", Vector3(NAN, R, 0.0), Quaternion.IDENTITY],
		["INF position", Vector3(INF, R, 0.0), Quaternion.IDENTITY],
		["NaN orientation", Vector3(0.0, R - 0.01, 0.0), Quaternion(NAN, 0.0, 0.0, 1.0)],
	]
	for case in bad_poses:
		# Re-establish a loaded state, then feed the bad pose straight to the constraint (SimWorld would refuse to step it).
		wheel.position = Vector3(0.0, R - 0.01, 0.0)
		wheel.orientation = Quaternion.IDENTITY
		rig["world"].step()
		check(s.carried_row_count() > 0, "%s: fixture error: loaded state expected" % case[0])
		var before_velocity: Vector3 = wheel.linear_velocity
		wheel.position = case[1]
		wheel.orientation = case[2]
		s.prepare(DT, config)
		check(not s.contact_active and s.rows.is_empty(), "%s: the patch must be inactive" % case[0])
		check(s.carried_row_count() == 0 and s.carried_impulse_total() == 0.0, "%s: stale state must be cleared" % case[0])
		check(s.normal_load() == 0.0 and s.active_sample_count() == 0, "%s: no load, no active sample" % case[0])
		check(s.last_reason != "", "%s: a reason must be reported" % case[0])
		s.solve_iteration()
		s.solve_split_iteration()
		s.finish()
		check_vec3_near(wheel.linear_velocity, before_velocity, 0.0, "%s: the invalid patch must not change the wheel velocity" % case[0])
		check(wheel.linear_velocity.is_finite() and wheel.angular_velocity.is_finite(), "%s: non-finite wheel state" % case[0])
	# Invalid time steps.
	wheel.position = Vector3(0.0, R - 0.01, 0.0)
	wheel.orientation = Quaternion.IDENTITY
	for bad_dt in [0.0, -DT, NAN, INF]:
		s.prepare(bad_dt, config)
		check(s.rows.is_empty() and s.normal_load() == 0.0 and s.carried_row_count() == 0, "dt %s must leave an inactive, empty patch" % bad_dt)
	# A non-finite carried impulse is discarded before it can reach the wheel.
	rig["world"].step()
	check(s.carried_row_count() > 0, "fixture error: loaded state expected")
	s.rows[0].accumulated_impulse = NAN
	rig["world"].step()
	check(is_finite(s.normal_load()) and wheel.linear_velocity.is_finite() and wheel.position.is_finite(), "a non-finite carried impulse leaked into the step")
	# An aborted step: SimWorld calls reset_warm_start() on every constraint.
	check(s.carried_row_count() > 0, "fixture error: loaded state expected")
	s.reset_warm_start()
	check(s.carried_row_count() == 0 and s.normal_load() == 0.0 and s.rows.is_empty() and not s.contact_active, "reset_warm_start must clear the patch")
	check(velocity.is_finite(), "fixture sanity")


## A finite, ACCEPTED parameter set whose derived coefficient overflows: k_A = 1e-305 N/m^3 with c_A = 0 gives k_i ~ 1e-309 N/m for a sample of
## ~1e-4 m^2 and gamma = 1 / (dt (c + dt k)) > 1.8e308 (not representable). validate() cannot reject it without knowing the sample area and dt, so
## the patch must be invalidated at prepare time: never a skipped sample, never partial support.
func test_l_a_non_finite_derived_coefficient_invalidates_the_whole_patch() -> void:
	var tiny: float = 1.0e-305
	var rig: Dictionary = make_rig({"mass": 100.0, "height": R - 0.01, "lat": 9, "circ": 48, "k": tiny, "c": 0.0})
	var wheel: SimRigidBody = rig["wheel"]
	var s = rig["support"]
	var config = rig["world"].config
	# The input is finite and passes the parameter validation (the failure is NOT an input-validation failure).
	check(is_finite(s.stiffness_density) and s.stiffness_density > 0.0 and is_finite(s.damping_density), "fixture error: finite parameters expected")
	check(s.validate().is_empty() and s.is_valid(), "the parameters must be accepted by validate()")
	# The failure is caused by a derived coefficient: with the independent sample areas, gamma is not finite for every compressed sample.
	var oracle: Dictionary = TirePatchSampler.sample(wheel.position, wheel.orientation, geo(), R, Vector3.ZERO, Vector3.UP, 9, 48)
	var compressed: int = 0
	for smp in oracle["samples"]:
		if smp["gap"] < 0.0:
			compressed += 1
			var coeff: Dictionary = TirePatchNormalSupport.soft_coefficients(tiny * smp["weight"], 0.0, DT)
			check(not is_finite(coeff["gamma"]), "fixture error: gamma must overflow for the sample with area %s" % smp["weight"])
	check(compressed > 5, "fixture error: the wheel must be compressed (%d samples)" % compressed)
	# Carried state to clear: a loaded valid patch first, then the same patch with the tiny stiffness.
	s.stiffness_density = K_A
	s.damping_density = C_A
	for i in 5:
		rig["world"].step()
	check(s.carried_row_count() > 0 and s.carried_impulse_total() > 0.0 and s.normal_load() > 0.0, "fixture error: a loaded patch with carried impulses expected")
	s.stiffness_density = tiny
	s.damping_density = 0.0
	check(s.is_valid(), "the edited patch is still valid by validate()")
	var v0: Vector3 = wheel.linear_velocity
	var w0: Vector3 = wheel.angular_velocity
	s.prepare(DT, config)
	check(not s.contact_active and s.rows.is_empty() and s.active_sample_count() == 0, "an invalid derived coefficient must leave no active rows")
	check(s.last_reason.begins_with("invalid derived coefficient: gamma"), "last_reason must name the coefficient (got '%s')" % s.last_reason)
	check(s.last_reason.contains("sample") and s.last_reason.contains("lateral") and s.last_reason.contains("angular"), "last_reason must identify the sample (got '%s')" % s.last_reason)
	check(s.carried_row_count() == 0 and s.carried_impulse_total() == 0.0, "no stale sample impulse may remain")
	check(s.normal_load() == 0.0 and s.contact_area() == 0.0 and s.sample_report().is_empty(), "no physical load or report may remain")
	s.solve_iteration()
	s.solve_split_iteration()
	s.finish()
	check(s.normal_load() == 0.0 and s.applied_wheel_moment() == Vector3.ZERO, "no load or moment after the invalid preparation")
	check_vec3_near(wheel.linear_velocity, v0, 0.0, "no partial support may reach the wheel (linear)")
	check_vec3_near(wheel.angular_velocity, w0, 0.0, "no partial support may reach the wheel (angular)")
	# Deterministic: the same failure and reason again, and a world step applies nothing to the wheel.
	var reason: String = s.last_reason
	s.prepare(DT, config)
	check(s.last_reason == reason and s.rows.is_empty(), "the failure must be deterministic")
	rig["world"].step()
	check_vec3_near(wheel.linear_velocity, v0, 0.0, "a world step with the invalid patch must not accelerate the wheel (no gravity)")
	check(s.rows.is_empty() and s.carried_row_count() == 0 and s.normal_load() == 0.0, "the patch stays empty through a world step")
	# Recovery: a valid stiffness gives a fresh patch (cold start).
	s.stiffness_density = K_A
	s.damping_density = C_A
	rig["world"].step()
	check(s.contact_active and s.normal_load() > 0.0 and s.last_reason == "", "valid parameters must restore the support")


# ---------------------------------------------------------------- N: velocity-side Jacobian vs impulse-side wrench

## Stiff compliance with a LIGHT wheel (5 kg), used where the velocity-side effective mass must be distinguishable from the full-Jacobian one.
## With the test compliance (k_A = 4e6, c_A = 1e5) the softness gamma = 1/(dt (c_i + dt k_i)) ~ 37 m/(N s^2) swamps K ~ 0.2 and the two
## effective masses differ by ~3e-4. At the parameter maxima (k_A = 1e10, c_A = 1e9) gamma ~ 5e-3 and K_full - K_proj = (u . r x n)^2 / I_axial
## reaches 0.26: the two effective masses differ by tens of percent, so a test can tell them apart by a wide margin. Test values only.
const STIFF_K: float = 1.0e10
const STIFF_C: float = 1.0e9
## Light, low-axial-inertia body-frame inertia (m = 20 kg, I_axial = 0.2, I_transverse = 0.4 kg m^2: a thin disc-like wheel, I_a <= 2 I_t) so that
## (u . r x n)^2 / I_axial (up to 0.06 for the footprint's fore-aft arms of ~0.1 m) is comparable to K_vel ~ 0.3 and the footprint has several samples
## with an axle-parallel moment. Test values, not wheel data.
const LIGHT_MASS: float = 20.0
const LIGHT_INERTIA: Basis = Basis(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.4, 0.0), Vector3(0.0, 0.0, 0.4))


## Independent Stage A sample positions of the wheel's current pose by sample index.
func oracle_positions(wheel: SimRigidBody, lat: int, circ: int) -> Dictionary:
	var oracle: Dictionary = TirePatchSampler.sample(wheel.position, wheel.orientation, geo(), R, Vector3.ZERO, Vector3.UP, lat, circ)
	var by_index: Dictionary = {}
	for smp in oracle["samples"]:
		by_index[sample_index(smp, circ)] = smp["position"]
	return by_index


## A. The effective mass of a row is 1/(K_vel + gamma) with K_vel = 1/m + J_v . I^-1 J_v and J_v = (r x n) - u (u . r x n), computed here from the
## independent sample positions, not 1/(K_full + gamma) with the full physical Jacobian (which is what the impulse uses).
func test_n_effective_mass_uses_the_velocity_side_jacobian() -> void:
	for camber in [0.0, 0.15]:
		var rig: Dictionary = make_rig({"mass": LIGHT_MASS, "inertia": LIGHT_INERTIA, "height": R - DELTA * cos(camber), "camber": camber, "k": STIFF_K, "c": STIFF_C, "lat": 9, "circ": 48})
		var wheel: SimRigidBody = rig["wheel"]
		var s = rig["support"]
		var label: String = "camber %s" % camber
		var p0: Vector3 = wheel.position
		var positions: Dictionary = oracle_positions(wheel, 9, 48)
		var u: Vector3 = (wheel.orientation * Vector3.RIGHT).normalized()
		var inverse_inertia: Basis = wheel.inverse_inertia_world
		s.prepare(DT, rig["world"].config)
		var report: Array = s.sample_report()
		check(s.rows.size() > 8, "%s: fixture error: the footprint must hold several samples" % label)
		var distinguished: int = 0
		for k in s.rows.size():
			var row = s.rows[k]
			var moment: Vector3 = (positions[report[k]["index"]] - p0).cross(Vector3.UP)  # r x n, the full physical moment arm
			var projected: Vector3 = moment - u * u.dot(moment)
			var gamma: float = TirePatchNormalSupport.soft_coefficients(STIFF_K * report[k]["area"], STIFF_C * report[k]["area"], DT)["gamma"]
			var m_velocity: float = 1.0 / (1.0 / wheel.mass + projected.dot(inverse_inertia * projected) + gamma)
			var m_full: float = 1.0 / (1.0 / wheel.mass + moment.dot(inverse_inertia * moment) + gamma)
			check_near(row.effective_mass, m_velocity, 1.0e-4 * m_velocity, "%s sample %d: the effective mass is not that of the velocity-side Jacobian" % [label, k])
			if absf(moment.dot(u)) > 0.06:
				distinguished += 1
				check(absf(m_velocity - m_full) > 0.1 * m_full, "%s sample %d: fixture error: the two effective masses must differ by > 10 %%" % [label, k])
				check(absf(row.effective_mass - m_full) > 0.08 * m_full, "%s sample %d: the effective mass is the full-Jacobian value" % [label, k])
		check(distinguished >= 3, "%s: fixture error: several samples must have an axle-parallel moment component > 0.06 m (got %d)" % [label, distinguished])


## B. The compression velocity of every sample is independent of the axle spin: for identical translational velocity and non-axle angular
## velocity, the row's own velocity (compute_velocity, i.e. the solver's measurement, not the bias) equals n . (v + w_perp x r) from the
## independent sample positions, and the normal support does not change. Without the projection the 60 rad/s spin would add up to
## 60 * 0.3 = 18 m/s to a sample (measured here against the margin of the tolerance). Tolerances: J_v is perpendicular to u only to float32
## rounding (6e-8 * |J| = 2e-8, times 60 rad/s ~ 1e-6 m/s): 5e-6 m/s; loads: 1e-6 relative.
func test_n_compression_velocity_and_support_are_independent_of_the_axle_spin() -> void:
	var camber: float = 0.15
	var q: Quaternion = Quaternion(Vector3(0.0, 0.0, 1.0), camber)
	var u: Vector3 = q * Vector3.RIGHT
	var perpendicular: Vector3 = q * Vector3(0.0, 0.5, 0.3)  # angular velocity with no axle component
	var v: Vector3 = Vector3(0.2, -0.1, -1.5)
	var reference_vn: Array = []
	var reference_loads: Array = []
	var reference_total: float = 0.0
	for spin in [0.0, 30.0, 60.0]:
		var omega: Vector3 = perpendicular - u * spin
		var label: String = "spin %s rad/s" % spin
		# Velocity level (light wheel, no step).
		var rig: Dictionary = make_rig({"mass": LIGHT_MASS, "inertia": LIGHT_INERTIA, "height": R - DELTA * cos(camber), "camber": camber, "velocity": v, "angular_velocity": omega, "lat": 9, "circ": 48})
		var wheel: SimRigidBody = rig["wheel"]
		var s = rig["support"]
		var p0: Vector3 = wheel.position
		var positions: Dictionary = oracle_positions(wheel, 9, 48)
		s.prepare(DT, rig["world"].config)
		var report: Array = s.sample_report()
		check(s.rows.size() > 8, "%s: fixture error: the footprint must hold several samples" % label)
		var max_spin_term: float = 0.0
		for k in s.rows.size():
			var arm: Vector3 = positions[report[k]["index"]] - p0
			var expected: float = Vector3.UP.dot(v + perpendicular.cross(arm))
			var measured: float = s.rows[k].compute_velocity()
			check_near(measured, expected, 5.0e-6, "%s sample %d: the compression velocity is not n . (v + w_perp x r)" % [label, k])
			max_spin_term = maxf(max_spin_term, absf(spin * u.dot(arm.cross(Vector3.UP))))
			if spin == 0.0:
				reference_vn.append(measured)
			else:
				check_near(measured, reference_vn[k], 5.0e-6, "%s sample %d: the axle spin changed the compression velocity" % [label, k])
		check(spin == 0.0 or max_spin_term > 1.0, "%s: fixture error: the spin term must be large against the tolerance (got %s m/s)" % [label, max_spin_term])
		# Support level (heavy wheel, damping on): the solver's own iterations driven directly, so that the world's gyroscopic update (which turns
		# w_perp of a spinning wheel before the solve and is not part of the row model) does not enter the comparison.
		var heavy: Dictionary = heavy_rig(17, 96, camber, DELTA, {"velocity": v, "angular_velocity": omega})
		var support = heavy["support"]
		support.prepare(DT, heavy["world"].config)
		for iteration in heavy["world"].config.solver_iterations:
			support.solve_iteration()
		support.finish()
		var loads: Array = []
		for entry in support.sample_report():
			loads.append(entry["load"])
		if spin == 0.0:
			reference_loads = loads
			reference_total = support.normal_load()
			check(reference_total > 100.0, "fixture error: the heavy wheel must be loaded")
		else:
			check(loads.size() == reference_loads.size(), "%s: the spin changed the footprint" % label)
			check_near(support.normal_load(), reference_total, 1.0e-6 * reference_total, "%s: the spin changed the normal load" % label)
			for k in mini(loads.size(), reference_loads.size()):
				check_near(loads[k], reference_loads[k], 1.0e-6 * reference_total / float(loads.size()) + 1.0e-6, "%s sample %d: the spin changed the sample load" % [label, k])


## C. Spin changes DURING sequential solving. The solver applies each row's impulse through the full wrench, so the axle spin changes after
## every row (the early rows of an unconverged pass load fore-aft asymmetrically, and the sample moments (u . r x n) lambda do not cancel).
## (1) Live velocity side: after every solve_iteration the row's compression velocity is again n . (v + w_perp x r) with the wheel's CURRENT
##     v and w_perp (axle component removed), while the axle spin has demonstrably changed: nothing is frozen at preparation.
## (2) Independence of the axial inertia: a wheel with a 3.75 times larger axial inertia (its spin responds 3.75 times less) must give the same
##     sample loads, because the velocity map and the effective mass contain no axle spin; each wheel's spin change equals the FULL physical axle
##     moment sum_i (u . r_i x n) lambda_i / I_axial of the solver's own loads. A frozen-bias compensation (full Jacobian for rate and effective mass,
##     initial spin subtracted from the bias) fails (1) at once and (2) through the feedback of the changed spin on the later rows.
func test_n_axle_spin_change_during_sequential_solving_does_not_feed_back() -> void:
	var camber: float = 0.15
	var q: Quaternion = Quaternion(Vector3(0.0, 0.0, 1.0), camber)
	var u: Vector3 = q * Vector3.RIGHT
	var v0: Vector3 = Vector3(0.1, -0.6, -0.4)
	var perpendicular: Vector3 = q * Vector3(0.0, 0.2, 0.1)
	var spin0: float = 25.0
	var light_inertia: Basis = LIGHT_INERTIA
	# (1) live velocity side, iteration by iteration
	var rig: Dictionary = make_rig({"mass": LIGHT_MASS, "inertia": LIGHT_INERTIA, "height": R - DELTA * cos(camber), "camber": camber, "velocity": v0, "angular_velocity": perpendicular - u * spin0,
			"k": STIFF_K, "c": STIFF_C, "lat": 9, "circ": 48})
	var wheel: SimRigidBody = rig["wheel"]
	var s = rig["support"]
	var p0: Vector3 = wheel.position
	var positions: Dictionary = oracle_positions(wheel, 9, 48)
	s.prepare(DT, rig["world"].config)
	var report: Array = s.sample_report()
	check(s.rows.size() > 8, "fixture error: the footprint must hold several samples")
	var spin_before: float = wheel.angular_velocity.dot(u)
	var largest_spin_change: float = 0.0
	for iteration in 3:
		s.solve_iteration()
		var spin_now: float = wheel.angular_velocity.dot(u)
		largest_spin_change = maxf(largest_spin_change, absf(spin_now - spin_before))
		var w_perp: Vector3 = wheel.angular_velocity - u * spin_now
		for k in s.rows.size():
			var arm: Vector3 = positions[report[k]["index"]] - p0
			var expected: float = Vector3.UP.dot(wheel.linear_velocity + w_perp.cross(arm))
			check_near(s.rows[k].compute_velocity(), expected, 2.0e-5, "iteration %d sample %d: the compression velocity is not n . (v + w_perp x r) of the current state" % [iteration, k])
	check(largest_spin_change > 0.05, "fixture error: the sequential impulses must change the axle spin (got %s rad/s)" % largest_spin_change)
	# (2) the same wheel with a 3.75 times larger axial inertia: its spin responds 3.75 times less to the same axle moment. If the changed spin fed
	# back into the normal solve (full Jacobian in the rate or in K), the loads would depend on the axial inertia. 0.75 is the largest axial inertia
	# the triangle inequality I_a <= I_t + I_t (0.8) of a valid body allows with margin.
	for iterations in [1, 2, 4]:
		var loads_by_inertia: Array = []
		var spin_change_by_inertia: Array = []
		var impulse_by_inertia: Array = []
		var peak_load: float = 0.0
		for axial in [0.2, 0.75]:
			var step_rig: Dictionary = make_rig({"mass": LIGHT_MASS, "height": R - DELTA * cos(camber), "camber": camber, "velocity": v0, "angular_velocity": perpendicular - u * spin0,
					"k": STIFF_K, "c": STIFF_C, "lat": 9, "circ": 48, "inertia": Basis.from_scale(Vector3(axial, 0.4, 0.4))})
			var body: SimRigidBody = step_rig["wheel"]
			var support = step_rig["support"]
			var start: Vector3 = body.position
			var by_index: Dictionary = oracle_positions(body, 9, 48)
			var spin_start: float = body.angular_velocity.dot(u)
			# The solver's own iterations driven directly: the world's gyroscopic update (which depends on the axial inertia and is applied
			# before the solve) is not part of the row model and must not enter the comparison.
			support.prepare(DT, step_rig["world"].config)
			for iteration in iterations:
				support.solve_iteration()
			support.finish()
			var loads: Array = []
			var impulse: float = 0.0
			for entry in support.sample_report():
				loads.append(entry["load"])
				peak_load = maxf(peak_load, entry["load"])
				impulse += u.dot((by_index[entry["index"]] - start).cross(Vector3.UP)) * entry["load"] * DT
			loads_by_inertia.append(loads)
			impulse_by_inertia.append(impulse)
			spin_change_by_inertia.append(body.angular_velocity.dot(u) - spin_start)
		var label: String = "%d iteration(s)" % iterations
		check(loads_by_inertia[0].size() > 8 and loads_by_inertia[0].size() == loads_by_inertia[1].size(), "%s: the two wheels must have the same footprint" % label)
		for k in mini(loads_by_inertia[0].size(), loads_by_inertia[1].size()):
			check_near(loads_by_inertia[0][k], loads_by_inertia[1][k], 1.0e-4 * peak_load, "%s sample %d: the axle spin change fed back into the normal solve" % [label, k])
		# Each wheel's spin changed by the FULL physical axle moment of its own loads divided by its own axial inertia.
		check(absf(impulse_by_inertia[0]) > 1.0e-3, "%s: fixture error: the loads must carry a real axle moment (got %s N m s)" % [label, impulse_by_inertia[0]])
		check_near(spin_change_by_inertia[0] * 0.2, impulse_by_inertia[0], 1.0e-3 * absf(impulse_by_inertia[0]) + 1.0e-5, "%s: the spin change of the 0.2 kg m^2 wheel is not the full axle moment of its loads" % label)
		check_near(spin_change_by_inertia[1] * 0.75, impulse_by_inertia[1], 1.0e-3 * absf(impulse_by_inertia[1]) + 1.0e-5, "%s: the spin change of the 0.75 kg m^2 wheel is not the full axle moment of its loads" % label)
		check(absf(spin_change_by_inertia[0]) > 2.5 * absf(spin_change_by_inertia[1]), "%s: fixture error: the two wheels must change their spin by very different amounts" % label)


## The two row Jacobians are distinct, independent quantities: the impulse side keeps the FULL moment r x n (axle component included), the velocity
## side is the same vector minus its axle component, and the physical impulse applied through the row changes the axle spin while the velocity
## map is blind to the axle.
func test_n_row_holds_a_full_wrench_and_a_projected_velocity_jacobian() -> void:
	for camber in [0.0, 0.15]:
		var rig: Dictionary = make_rig({"mass": 100.0, "height": R - DELTA * cos(camber), "camber": camber, "lat": 9, "circ": 48})
		var wheel: SimRigidBody = rig["wheel"]
		var s = rig["support"]
		var label: String = "camber %s" % camber
		var p0: Vector3 = wheel.position
		var positions: Dictionary = oracle_positions(wheel, 9, 48)
		var u: Vector3 = (wheel.orientation * Vector3.RIGHT).normalized()
		s.prepare(DT, rig["world"].config)
		var report: Array = s.sample_report()
		var with_axle_component: int = 0
		for k in s.rows.size():
			var row = s.rows[k]
			var moment: Vector3 = (positions[report[k]["index"]] - p0).cross(Vector3.UP)
			var velocity_side = row.get("velocity_angular_b")
			check(velocity_side != null, "%s: the row has no velocity-side angular Jacobian" % label)
			if velocity_side == null:
				return
			check_vec3_near(row.angular_b, moment, 1.0e-6, "%s sample %d: the impulse Jacobian is not the full r x n" % [label, k])
			check_vec3_near(velocity_side, moment - u * u.dot(moment), 1.0e-6, "%s sample %d: the velocity-side Jacobian is not (r x n) minus its axle component" % [label, k])
			check_near(velocity_side.dot(u), 0.0, 1.0e-6, "%s sample %d: the velocity-side Jacobian has an axle component" % [label, k])
			check_vec3_near(row.linear_b, Vector3.UP, 1.0e-6, "%s sample %d: the linear Jacobian is shared" % [label, k])
			if absf(moment.dot(u)) > 0.05:
				with_axle_component += 1
		check(with_axle_component >= 3, "%s: fixture error: several samples must have an axle-parallel moment (got %d)" % [label, with_axle_component])


## Split correction: it is a POSE correction along the velocity-side Jacobian and never the physical wrench. One sequential split pass over
## a bottomed footprint is fore-aft asymmetric (ordering), so a physical wrench would leave an axle component in the split angular velocity;
## the velocity-side Jacobian leaves none. The physical velocities are untouched by the split pass.
func test_n_split_correction_does_not_apply_the_axle_moment_and_leaves_physical_velocity_alone() -> void:
	var rig: Dictionary = make_rig({"mass": 5.0, "height": R - 0.07, "dmax": 0.05, "k": STIFF_K, "c": STIFF_C, "lat": 9, "circ": 48,
			"angular_velocity": Vector3(-20.0, 0.0, 0.0)})
	var wheel: SimRigidBody = rig["wheel"]
	var s = rig["support"]
	s.prepare(DT, rig["world"].config)
	check(s.bottomed_sample_count() > 10, "fixture error: many samples must be bottomed (got %d)" % s.bottomed_sample_count())
	var linear0: Vector3 = wheel.linear_velocity
	var angular0: Vector3 = wheel.angular_velocity
	s.solve_split_iteration()
	var split_perp: float = (wheel.split_angular_velocity - Vector3.RIGHT * wheel.split_angular_velocity.x).length()
	var wrench_axle: float = 0.0
	for k in s.rows.size():
		if s.rows[k].split_enabled:
			wrench_axle += s.rows[k].angular_b.x * s.rows[k].split_accumulated_impulse
	check(wheel.split_linear_velocity.y > 0.0, "the split correction must push the wheel out of the plane")
	check(absf(wrench_axle) > 1.0e-3 * wheel.split_linear_velocity.y * wheel.mass or split_perp > 0.0, "fixture error: the split pass must be asymmetric or non-trivial")
	check_near(wheel.split_angular_velocity.x, 0.0, 1.0e-6 * maxf(1.0, wheel.split_angular_velocity.length()), "the split correction spun the wheel about its axle")
	check_vec3_near(wheel.linear_velocity, linear0, 0.0, "the split pass changed the physical linear velocity")
	check_vec3_near(wheel.angular_velocity, angular0, 0.0, "the split pass changed the physical angular velocity")


# ---------------------------------------------------------------- O: principal-axis precondition
##
## The velocity-side effective mass J_v M^-1 J_v^T equals the exact response J_v M^-1 J_i^T only if the wheel axle u (body-local +X) is a principal
## axis of the wheel inertia, which SimMassProperties does not imply (it accepts full tensors). validate() therefore rejects a wheel whose BODY-FRAME
## tensor couples the axle to the transverse axes (I_xy, I_xz). The tests establish: the accepted and the rejected tensors are valid inertias on
## their own (SimMassProperties.validate_inertia), the verdict depends on the body-frame tensor and never on the world orientation (a rotated principal
## wheel is accepted although its WORLD tensor is non-diagonal; a non-principal wheel posed so that its WORLD tensor is diagonal is rejected), float32
## roundoff of an assembled principal tensor is not rejected, the check runs in SimWorld's ordinary registration and preflight lifecycle, and nothing is
## repaired, projected or skipped. Wheel of the fixture: m = 100 kg, axial moment A = 7.53 and transverse moments B = 4.02 kg m^2 (MotorcycleConfig.wheel_inertia).

## Products of inertia of the rejected cases: 0.5 kg m^2 is 6.6 % of the axial moment and tilts the true principal axes of the axle/transverse
## plane by 0.5 atan(2 * 0.5 / (A - B)) = 0.139 rad (8 degrees), a physically meaningful skew (a wheel whose mass is distributed off its axle plane).
const COUPLING: float = 0.5
## The documented tolerance of the precondition (PHYSICS_CONVENTIONS.md): a coupling above this fraction of the largest tensor element is rejected.
## It equals SimMassProperties' symmetry tolerance (about 84 float32 ulps). Written here as a literal so the tests do not read it from the constraint.
const PRINCIPAL_REL_TOL: float = 1.0e-5


## The fixture wheel's tensor with the products of inertia I_xy, I_xz (axle coupling) and I_yz (transverse coupling) added. Symmetric by construction.
func coupled_inertia(xy: float, xz: float, yz: float = 0.0) -> Basis:
	var diagonal: Basis = MotorcycleConfig.wheel_inertia(100.0, R, 0.75 * R, W)
	var a: float = diagonal.x.x
	var b: float = diagonal.y.y
	var c: float = diagonal.z.z
	return Basis(Vector3(a, xy, xz), Vector3(xy, b, yz), Vector3(xz, yz, c))


func wheel_with(inertia: Basis, orientation: Quaternion = Quaternion.IDENTITY) -> SimRigidBody:
	return make_body(100.0, inertia, Vector3(0.0, R, 0.0), orientation)


## The patch exactly as create() would build it but WITHOUT the validation gate, so validate() can be read for a wheel that create() refuses.
func unchecked_support(ground: SimRigidBody, wheel: SimRigidBody):
	return TirePatchNormalSupport.new(ground, wheel, geo(), R, Vector3.ZERO, Vector3.UP, K_A, C_A, D_MAX, 9, 48)


## True when validate() of a patch on a wheel with this body-frame tensor reports no error (the only public route; no private helper is read).
func accepted(tensor: Basis) -> bool:
	return unchecked_support(make_static(), wheel_with(tensor)).validate().is_empty()


func max_off_diagonal(m: Basis) -> float:
	return maxf(maxf(absf(m.x.y), absf(m.x.z)), absf(m.y.z))


## O1. A wheel whose axle is a principal axis stays valid: the diagonal fixture wheel, other diagonal and isotropic tensors, with the full lifecycle.
func test_o_a_diagonal_wheel_with_the_axle_principal_axis_stays_valid() -> void:
	var diagonal: Basis = MotorcycleConfig.wheel_inertia(100.0, R, 0.75 * R, W)
	check(diagonal.x.y == 0.0 and diagonal.x.z == 0.0 and diagonal.y.z == 0.0, "fixture error: the reference wheel tensor must be exactly diagonal")
	var tensors: Array = [
		["fixture wheel (axial > transverse)", diagonal],
		["axial < transverse", LIGHT_INERTIA],
		["three distinct principal moments", Basis.from_scale(Vector3(5.0, 3.0, 4.0))],
		["isotropic (every axis principal)", SimMassProperties.sphere_inertia(100.0, R)],
	]
	for t in tensors:
		var tensor: Basis = t[1]
		check(SimMassProperties.validate_inertia(tensor) == "", "fixture error: %s must be a valid inertia" % t[0])
		var wheel: SimRigidBody = wheel_with(tensor)
		var support = unchecked_support(make_static(), wheel)
		check(support.validate().is_empty(), "%s: a principal-axis wheel was reported invalid: %s" % [t[0], "; ".join(support.validate())])
		check(try_create({"wheel": wheel}) != null, "%s: create() refused a principal-axis wheel" % t[0])
	# The whole lifecycle with the reference wheel: registered, preflight passes, the patch supports the wheel.
	var rig: Dictionary = make_rig({"height": R - DELTA})
	check(rig["support"].is_valid(), "the reference patch must be valid")
	check(rig["world"].constraints.size() == 1 and rig["world"].constraints[0] == rig["support"], "fixture error: the patch must be registered in the world")
	check(rig["world"].preflight(), "preflight must accept the reference patch (%s)" % rig["world"].last_error)
	check(rig["world"].step(), "the reference world must step")
	check(rig["support"].contact_active and rig["support"].normal_load() > 0.0, "a valid patch must support the compressed wheel")


## O2. A valid, non-diagonal tensor whose axle is NOT a principal axis is rejected: a physically meaningful product of inertia on the axle row.
func test_o_a_non_principal_axle_with_a_valid_non_diagonal_tensor_is_rejected() -> void:
	var cases: Array = [
		["I_xy only", coupled_inertia(COUPLING, 0.0)],
		["I_xz only (negative)", coupled_inertia(0.0, -COUPLING)],
		["I_xy and I_xz", coupled_inertia(0.3, 0.4)],
		["I_xy with a transverse I_yz", coupled_inertia(COUPLING, 0.0, 0.2)],
	]
	for c in cases:
		var tensor: Basis = c[1]
		var label: String = c[0]
		# The tensor is a valid inertia on its own and the body is an ordinary valid dynamic body: only the axle condition is at fault.
		check(SimMassProperties.validate_inertia(tensor) == "", "%s: fixture error: the tensor must pass validate_inertia() on its own" % label)
		var props: SimMassProperties = SimMassProperties.new(100.0, tensor)
		check(props.valid and not props.is_static(), "%s: fixture error: the mass properties must be valid and dynamic (%s)" % [label, props.error_message])
		check(max_off_diagonal(tensor) >= 0.2, "%s: fixture error: the coupling must be physically meaningful" % label)
		var wheel: SimRigidBody = wheel_with(tensor)
		var ground: SimRigidBody = make_static()
		var support = unchecked_support(ground, wheel)
		var errors: PackedStringArray = support.validate()
		check(errors.size() == 1, "%s: exactly one error expected, got %d: %s" % [label, errors.size(), "; ".join(errors)])
		check(not support.is_valid(), "%s: is_valid() must be false" % label)
		if errors.size() >= 1:
			for phrase in ["tire patch:", "normal-support model requires", "wheel axle (body-local +X)", "principal axis", "I_xy =", "I_xz =", "Define the wheel inertia"]:
				check(errors[0].contains(phrase), "%s: the message lacks '%s': %s" % [label, phrase, errors[0]])
		check(try_create({"wheel": wheel}) == null, "%s: create() accepted a non-principal-axis wheel" % label)
		# SimWorld refuses it at registration (both bodies registered, so the refusal can only be the validation), and accepts the principal control.
		var w: SimWorld = make_world()
		w.add_body(ground)
		w.add_body(wheel)
		check(w.add_constraint(unchecked_support(ground, wheel)) == null, "%s: SimWorld registered a non-principal-axis patch" % label)
		check(w.constraints.is_empty(), "%s: the refused constraint is in the world" % label)
		var control_wheel: SimRigidBody = wheel_with(MotorcycleConfig.wheel_inertia(100.0, R, 0.75 * R, W))
		var cw: SimWorld = make_world()
		var cground: SimRigidBody = make_static()
		cw.add_body(cground)
		cw.add_body(control_wheel)
		check(cw.add_constraint(unchecked_support(cground, control_wheel)) != null, "%s: control error: the principal-axis control must register" % label)
		# Nothing was repaired: the wheel keeps exactly the tensor it was given (and its inverse).
		check_basis_near(wheel.local_inertia, tensor, 0.0, "%s: the wheel inertia was modified" % label)
		check_basis_near(wheel.inverse_local_inertia, props.inverse_local_inertia, 0.0, "%s: the wheel inverse inertia was modified" % label)


## O3. A rotated body whose axle remains a principal axis is accepted, and the verdict follows the body-frame tensor, not the world orientation.
func test_o_the_verdict_follows_the_body_frame_tensor_not_the_world_orientation() -> void:
	var diagonal: Basis = MotorcycleConfig.wheel_inertia(100.0, R, 0.75 * R, W)
	var yaw_camber: Quaternion = Quaternion(Vector3.UP, 0.9) * Quaternion(Vector3(0.0, 0.0, 1.0), 0.3)
	var tilted: Quaternion = Quaternion(Vector3(1.0, 2.0, 0.5).normalized(), 1.1)
	# (a) a diagonal body-frame tensor in rotated bodies: the WORLD tensor R I R^T is non-diagonal, the axle is still principal, accepted.
	for q in [yaw_camber, tilted]:
		var wheel: SimRigidBody = wheel_with(diagonal, q)
		var u: Vector3 = q * Vector3.RIGHT
		check(max_off_diagonal(wheel.inertia_world) > 0.5, "fixture error: the world tensor of the rotated wheel must be clearly non-diagonal (got %s)" % max_off_diagonal(wheel.inertia_world))
		check((wheel.inertia_world * u).cross(u).length() < 1.0e-5 * diagonal.x.x, "fixture error: the world axle must still be a principal axis of the world tensor")
		check(unchecked_support(make_static(), wheel).validate().is_empty(), "a rotated wheel with a principal axle was rejected (orientation %s)" % q)
	# (b) a NON-DIAGONAL body-frame tensor whose axle is principal: the transverse axes are rotated about the axle inside the body frame (I_yz != 0,
	# I_xy = I_xz = 0 up to float32 rounding). Valid, accepted in every orientation.
	var spin_about_axle: Basis = Basis(Vector3.RIGHT, 0.6)
	var skewed_transverse: Basis = spin_about_axle * Basis.from_scale(Vector3(7.5, 5.0, 3.0)) * spin_about_axle.transposed()
	check(absf(skewed_transverse.y.z) > 0.5, "fixture error: the body-frame tensor must have a clear transverse product of inertia (got %s)" % skewed_transverse.y.z)
	check(SimMassProperties.validate_inertia(skewed_transverse) == "", "fixture error: the skewed-transverse tensor must be a valid inertia")
	check_vec3_near(skewed_transverse * Vector3.RIGHT, Vector3(7.5, 0.0, 0.0), 1.0e-5, "fixture error: the axle must be a principal axis of the skewed tensor")
	for q in [Quaternion.IDENTITY, yaw_camber, tilted]:
		var wheel_skewed: SimRigidBody = wheel_with(skewed_transverse, q)
		check(unchecked_support(make_static(), wheel_skewed).validate().is_empty(), "a non-diagonal tensor with a principal axle was rejected (orientation %s)" % q)
	# (c) the trap: a NON-principal body-frame tensor posed so that its WORLD tensor is exactly diagonal. A world-frame "is the tensor diagonal"
	# test would accept it; the axle is not principal and it must be rejected. T = V D V^T with V = Rz(phi); the body orientation R = V^T gives R T R^T = D.
	var tensor: Basis = coupled_inertia(COUPLING, 0.0)
	var phi: float = 0.5 * atan2(2.0 * COUPLING, tensor.x.x - tensor.y.y)
	var trap_orientation: Quaternion = Quaternion(Vector3(0.0, 0.0, 1.0), -phi)
	var trap: SimRigidBody = wheel_with(tensor, trap_orientation)
	check(max_off_diagonal(trap.inertia_world) < 5.0e-5, "fixture error: the world tensor of the trap wheel must be diagonal (got %s)" % max_off_diagonal(trap.inertia_world))
	var u_trap: Vector3 = trap_orientation * Vector3.RIGHT
	check((trap.inertia_world * u_trap).cross(u_trap).length() > 0.1, "fixture error: the axle of the trap wheel must NOT be a principal axis")
	var trap_errors: PackedStringArray = unchecked_support(make_static(), trap).validate()
	check(trap_errors.size() == 1, "a non-principal wheel whose world tensor is diagonal must be rejected (got %d errors)" % trap_errors.size())
	# (d) the verdict and the message of one non-principal tensor are the same in every orientation.
	var reference: PackedStringArray = unchecked_support(make_static(), wheel_with(tensor)).validate()
	for q in [yaw_camber, tilted, trap_orientation]:
		check(unchecked_support(make_static(), wheel_with(tensor, q)).validate() == reference, "the verdict or message depends on the world orientation (%s)" % q)


## O4. The validation result and message are deterministic, and carry the measured couplings.
func test_o_validation_result_and_message_are_deterministic() -> void:
	var tensor: Basis = coupled_inertia(COUPLING, 0.2)
	var first: PackedStringArray = unchecked_support(make_static(), wheel_with(tensor)).validate()
	var support = unchecked_support(make_static(), wheel_with(tensor))
	var repeated: Array = []
	for i in 5:
		repeated.append(support.validate())
	check(first.size() == 1, "exactly one error expected (got %d)" % first.size())
	for r in repeated:
		check(r == first, "validate() of one patch is not repeatable")
	# Everything below reads the message; a missing one is already a failure above and must not become a script error.
	var message: String = first[0] if first.size() == 1 else ""
	# The message names the measured products of inertia (the tensor holds them exactly, x.y == y.x) and the rule that was applied.
	var scale: float = tensor.x.x  # the largest element of this tensor
	var coupling: float = sqrt(tensor.x.y * tensor.x.y + tensor.x.z * tensor.x.z)
	check(message.contains("I_xy = %s, I_xz = %s kg m^2" % [tensor.x.y, tensor.x.z]), "the message must report the products of inertia: %s" % message)
	check(message.contains("coupling %s kg m^2 exceeds the tolerance %s kg m^2" % [coupling, PRINCIPAL_REL_TOL * scale]),
			"the message must report the coupling and the tolerance: %s" % message)
	# A different coupling gives a different message, an equal one an equal message.
	var other: PackedStringArray = unchecked_support(make_static(), wheel_with(coupled_inertia(0.3, 0.2))).validate()
	check(other.size() == 1 and message != "" and other[0] != message, "the message must depend on the coupling")
	var equal: PackedStringArray = unchecked_support(make_static(), wheel_with(coupled_inertia(COUPLING, 0.2))).validate()
	check(equal == first, "equal tensors must give equal results")


## O5. Float32 roundoff of an assembled principal tensor is not rejected, a coupling just inside the tolerance passes and one just outside fails, at every
## physical scale. The roundoff tensors are assembled through GENERAL rotations, T = Q^T (Q D Q^T) Q, which is mathematically diag(D) but numerically carries
## float32 products of inertia: measured over the 75 cases below, the symmetrised I_xy / I_xz are nonzero in every case and at most 2.2e-7 of the largest
## element (about two float32 epsilons), 45 times below the tolerance. (A rotation about the axle alone would not do: its first row and column are exactly
## (1, 0, 0), so I_xy = I_xz = 0 with no rounding at all and the test would be vacuous; such tensors are covered by O3 (b).)
func test_o_roundoff_is_accepted_and_the_tolerance_is_scale_independent() -> void:
	var worst: float = 0.0
	var nonzero: int = 0
	var cases: int = 0
	for scale in [1.0e-3, 1.0, 1.0e4]:
		for axis in [Vector3(1.0, 2.0, 3.0), Vector3(-2.0, 1.0, 0.5), Vector3(0.3, -1.0, 2.0), Vector3(1.0, 1.0, 1.0), Vector3(0.0, 3.0, -1.0)]:
			for angle in [0.3, 0.7, 1.3, 2.1, 2.9]:
				var q: Basis = Basis(axis.normalized(), angle)
				var principal: Basis = Basis.from_scale(Vector3(7.5, 5.0, 3.0) * scale)
				var tensor: Basis = q.transposed() * (q * principal * q.transposed()) * q
				check(SimMassProperties.validate_inertia(tensor) == "", "fixture error: the assembled tensor must be a valid inertia (scale %s, axis %s, angle %s)" % [scale, axis, angle])
				var relative: float = 0.5 * sqrt(pow(tensor.x.y + tensor.y.x, 2) + pow(tensor.x.z + tensor.z.x, 2)) / (7.5 * scale)
				worst = maxf(worst, relative)
				cases += 1
				if relative > 0.0:
					nonzero += 1
				check(accepted(tensor), "roundoff of an assembled principal tensor was rejected (scale %s, axis %s, angle %s, relative coupling %s)" % [scale, axis, angle, relative])
	check(nonzero == cases, "fixture error: every assembled tensor must carry some float32 roundoff in I_xy / I_xz (%d of %d do)" % [nonzero, cases])
	check(worst < 0.1 * PRINCIPAL_REL_TOL, "the roundoff of the assembled tensors (%s) is not far below the tolerance" % worst)
	# The tolerance boundary: half the tolerance passes, twice the tolerance fails, for both products of inertia, both signs and every scale.
	for scale in [1.0e-3, 1.0, 1.0e4]:
		var base: Vector3 = Vector3(0.6, 0.3, 0.3) * scale
		var largest: float = 0.6 * scale
		var inside: float = 0.5 * PRINCIPAL_REL_TOL * largest
		var outside: float = 2.0 * PRINCIPAL_REL_TOL * largest
		for direction in [1.0, -1.0]:
			var in_xy: Basis = Basis(Vector3(base.x, direction * inside, 0.0), Vector3(direction * inside, base.y, 0.0), Vector3(0.0, 0.0, base.z))
			var in_xz: Basis = Basis(Vector3(base.x, 0.0, direction * inside), Vector3(0.0, base.y, 0.0), Vector3(direction * inside, 0.0, base.z))
			var out_xy: Basis = Basis(Vector3(base.x, direction * outside, 0.0), Vector3(direction * outside, base.y, 0.0), Vector3(0.0, 0.0, base.z))
			var out_xz: Basis = Basis(Vector3(base.x, 0.0, direction * outside), Vector3(0.0, base.y, 0.0), Vector3(direction * outside, 0.0, base.z))
			check(accepted(in_xy) and accepted(in_xz), "a coupling at half the tolerance was rejected (scale %s)" % scale)
			check(not accepted(out_xy) and not accepted(out_xz), "a coupling at twice the tolerance was accepted (scale %s)" % scale)


## O6. The check belongs to the ordinary constraint validation lifecycle: a wheel whose mass properties are replaced AFTER registration is refused by
## preflight() and step() with no state change, nothing is repaired or skipped, and stepping resumes when the wheel is valid again.
func test_o_the_check_runs_in_the_preflight_lifecycle_and_nothing_is_repaired_or_skipped() -> void:
	var rig: Dictionary = make_rig({"height": R - DELTA})
	var w: SimWorld = rig["world"]
	var wheel: SimRigidBody = rig["wheel"]
	var support = rig["support"]
	check(w.step() and w.step(), "fixture error: the valid world must step")
	check(support.carried_impulse_total() > 0.0, "fixture error: the patch must carry a warm-start impulse")
	var good: SimMassProperties = wheel.mass_properties
	var tensor: Basis = coupled_inertia(COUPLING, 0.0)
	check(wheel.set_mass_properties(SimMassProperties.new(100.0, tensor)), "fixture error: the body must accept the valid non-principal mass properties")
	var position: Vector3 = wheel.position
	var orientation: Quaternion = wheel.orientation
	var linear: Vector3 = wheel.linear_velocity
	var angular: Vector3 = wheel.angular_velocity
	var steps: int = w.step_count
	var carried: float = support.carried_impulse_total()
	var rejected: int = w.rejected_step_count
	check(not support.is_valid(), "the patch must report the non-principal wheel")
	check(not w.preflight(), "preflight must refuse a non-principal-axis wheel")
	check(w.last_error.contains("principal axis"), "preflight must report the reason: %s" % w.last_error)
	check(not w.step(), "step() must be refused")
	check(w.advance(3.0 * DT) == 0, "advance() must be refused")
	check(w.rejected_step_count == rejected + 2, "the refused step and advance must be counted")
	check(w.step_count == steps, "a refused step must not advance the step counter")
	check(wheel.position == position and wheel.orientation == orientation and wheel.linear_velocity == linear and wheel.angular_velocity == angular,
			"a refused step must not change the wheel state")
	check(support.carried_impulse_total() == carried, "a refused step must not touch the carried impulses")
	check_basis_near(wheel.local_inertia, tensor, 0.0, "the wheel inertia was repaired or projected")
	check(not w.constraints.is_empty() and w.constraints[0] == support, "the patch must stay registered (it is refused, not skipped)")
	# Valid again: stepping resumes and the patch supports the wheel.
	check(wheel.set_mass_properties(good), "fixture error: the valid properties must be accepted again")
	check(w.preflight(), "preflight must accept the wheel again (%s)" % w.last_error)
	check(w.step(), "stepping must resume")
	check(support.normal_load() > 0.0, "the patch must support the wheel again")


## O7. Existing invalid-body and constraint validation is intact: an invalid body still reports only the dynamic-body rule, null and static wheels
## keep their messages, other rules still report next to the new one in a fixed order, and the reference patch stays valid.
func test_o_existing_invalid_body_and_constraint_validation_is_intact() -> void:
	# An indefinite tensor makes the mass properties invalid: zeroed, static-looking. The axle rule must not add a misleading second message.
	var indefinite: SimMassProperties = SimMassProperties.new(100.0, Basis(Vector3(1.0, 2.0, 0.0), Vector3(2.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0)))
	check(not indefinite.valid, "fixture error: the indefinite tensor must be an invalid inertia")
	var invalid_wheel: SimRigidBody = SimRigidBody.new(indefinite, Vector3(0.0, R, 0.0))
	var invalid_errors: PackedStringArray = unchecked_support(make_static(), invalid_wheel).validate()
	check(invalid_errors == PackedStringArray(["tire patch: the wheel body must be dynamic"]), "an invalid wheel body must report only the dynamic-body rule (got %s)" % "; ".join(invalid_errors))
	check(try_create({"wheel": invalid_wheel}) == null, "create() accepted an invalid wheel body")
	# Static wheel and null bodies: unchanged messages.
	check(unchecked_support(make_static(), make_static()).validate() == PackedStringArray(["tire patch: the wheel body must be dynamic"]),
			"a static wheel must report only the dynamic-body rule")
	check(unchecked_support(make_static(), null).validate() == PackedStringArray(["tire patch: bodies must not be null"]), "a null wheel must report the null-body rule")
	check(unchecked_support(null, wheel_with(MotorcycleConfig.wheel_inertia(100.0, R, 0.75 * R, W))).validate() == PackedStringArray(["tire patch: bodies must not be null"]),
			"a null ground must report the null-body rule")
	# Several faults: each rule still reports, the axle rule keeps its fixed place (after the body rules, before the parameter rules).
	var both = unchecked_support(make_static(), wheel_with(coupled_inertia(COUPLING, 0.0)))
	both.stiffness_density = 0.0
	both.max_compression = -1.0
	var errors: PackedStringArray = both.validate()
	check(errors.size() == 3, "three faults must give three errors (got %d: %s)" % [errors.size(), "; ".join(errors)])
	if errors.size() == 3:
		check(errors[0].contains("principal axis") and errors[1].contains("stiffness density") and errors[2].contains("maximum compression"),
				"the errors must come in the fixed order axle, stiffness, compression (got %s)" % "; ".join(errors))
	# Dynamic ground still rejected next to a valid wheel, and the valid reference patch is valid.
	var dynamic_ground: SimRigidBody = wheel_with(MotorcycleConfig.wheel_inertia(100.0, R, 0.75 * R, W))
	check(unchecked_support(dynamic_ground, wheel_with(MotorcycleConfig.wheel_inertia(100.0, R, 0.75 * R, W))).validate() == PackedStringArray(["tire patch: the ground body must be static"]),
			"a dynamic ground must keep its message")
	check(try_create({}) != null and try_create({}).validate().is_empty(), "the reference patch must stay valid")


# ---------------------------------------------------------------- M: determinism

func test_m_runs_are_bit_identical() -> void:
	var finals: Array = []
	for run in 2:
		var rig: Dictionary = make_rig({"gravity": G, "fixture": true, "velocity": Vector3(0.0, -8.0, 0.0)})
		for i in 90:
			rig["world"].step()
		finals.append([rig["wheel"].position, rig["wheel"].linear_velocity, rig["support"].normal_load(), rig["support"].active_sample_count()])
	check(finals[0][0] == finals[1][0] and finals[0][1] == finals[1][1] and finals[0][2] == finals[1][2] and finals[0][3] == finals[1][3],
			"two identical runs differ")
