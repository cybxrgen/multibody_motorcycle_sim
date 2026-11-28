extends "res://Tests/sim_test_case.gd"
## M2.5.3 Stage C: tyre-force integration on the distributed patch (TirePatchNormalSupport with a tyre,
## Simulation/Motorcycle/tire_patch_normal_support.gd; shared mathematics in tire_model.gd). The patch is built through create_with_tire() and
## is NEVER registered in a motorcycle assembly (that is Stage D); every fixture is a single wheel on a static ground plane.
##
## Expectations are computed IN THIS FILE from independent formulas: the aggregate load, point and velocity from sample_report() (Stage B
## per-sample loads and Stage A positions) with plain vector algebra; the tyre axes from the cross products of the definition; the slip from
## TireModel.slip (tested on its own in test_tire_slip.gd) evaluated at the independently derived effective point; the tangential force law from
## TireModel.force (test_tire_forces.gd); the rolling resistance from TireModel.rolling_resistance_force; impulse/momentum budgets from the velocity
## change of the wheel. No expectation reads the constraint's own tangential code.
##
## Fixtures. HEAVY rig: a 1e7 kg wheel without gravity (as in the Stage B file): its velocity changes by ~1e-6 m/s in one step, so the implicit
## tangential response reduces to the explicit law at the initial state, lambda = dt F(sigma), and the closed-form law can be the oracle.
## LIGHT rig: the 15 kg rear prototype wheel (hollow cylinder, inner radius 0.75 R) where the implicit coupling matters. Direct-driving
## helper solve_in_place() calls prepare / solve_iteration / finish on the constraint exactly like ConstraintSolver (no split pass, no gravity,
## no integration), so velocities and impulses can be compared exactly. Tyre: MotorcyclePrototype.tire_spec (illustrative values: k_x 15,
## k_y 10, grip 1, c_rr 0.015, v_rr 0.2 m/s, v_ref 0.5 m/s), surface friction 0.8. Patch: width 0.18 m, crown radius 0.10 m, wheel radius 0.31 m,
## k_A 4e6 N/m^3, c_A 1e5 N s/m^3, bottoming 5 cm (the Stage B test values, not tyre data).
## Tolerances: loads and forces of the heavy rig 2e-4 relative (the response changes the state by < 1e-6 relative); float32 positions 2e-6 m.

const TireGeometry = preload("res://Simulation/Motorcycle/tire_geometry.gd")
const TirePatchNormalSupport = preload("res://Simulation/Motorcycle/tire_patch_normal_support.gd")
const TireModel = preload("res://Simulation/Motorcycle/tire_model.gd")
const TireGroundContact = preload("res://Simulation/Motorcycle/tire_ground_contact.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")
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
const DELTA: float = 0.02
const MU_S: float = 0.8
const LIGHT: float = 15.0


# ---------------------------------------------------------------- fixtures

func geo() -> TireGeometry:
	return TireGeometry.new(W, RHO)


## A fresh prototype tyre of this wheel with fields overridden: {"field": value}.
func spec_with(overrides: Dictionary = {}):
	var s = MotorcyclePrototype.tire_spec(R)
	for key in overrides:
		s.set(key, overrides[key])
	return s


## Rig options: mass (15 kg), height (wheel centre, R), camber (rad about +Z), yaw (rad about +Y), forward_speed / lateral_speed (m/s along the tyre's
## own e_x / e_y), velocity (extra world vector), spin (rad/s, positive = forward rolling about -u), omega (extra world angular velocity), gravity,
## iterations, k, c, dmax, lat (9), circ (48), spec_overrides, spec (a ready spec, wins), friction (0.8), no_tire (builds the Stage B patch).
## Returns {world, ground, wheel, support, spec, u, e_x, e_y}.
func make_rig(opts: Dictionary = {}) -> Dictionary:
	var mass: float = opts.get("mass", LIGHT)
	var q: Quaternion = Quaternion(Vector3.UP, opts.get("yaw", 0.0)) * Quaternion(Vector3(0.0, 0.0, 1.0), opts.get("camber", 0.0))
	var w: SimWorld = make_world(Vector3(0.0, -opts.get("gravity", 0.0), 0.0))
	w.config.solver_iterations = opts.get("iterations", 20)
	var ground: SimRigidBody = make_static()
	var wheel: SimRigidBody = make_body(mass, MotorcycleConfig.wheel_inertia(mass, R, 0.75 * R, W), Vector3(0.0, opts.get("height", R), 0.0), q)
	var u: Vector3 = q * Vector3.RIGHT
	var tyre_axes: Dictionary = TireModel.axes(u, Vector3.UP)
	var e_x: Vector3 = tyre_axes["longitudinal"]
	var e_y: Vector3 = tyre_axes["lateral"]
	wheel.linear_velocity = e_x * opts.get("forward_speed", 0.0) + e_y * opts.get("lateral_speed", 0.0) + opts.get("velocity", Vector3.ZERO)
	wheel.angular_velocity = -u * opts.get("spin", 0.0) + opts.get("omega", Vector3.ZERO)
	w.add_body(ground)
	w.add_body(wheel)
	var spec = opts.get("spec", spec_with(opts.get("spec_overrides", {})))
	var support = null
	if opts.get("no_tire", false):
		support = TirePatchNormalSupport.create(ground, wheel, geo(), R, Vector3.ZERO, Vector3.UP, opts.get("k", K_A), opts.get("c", C_A),
				opts.get("dmax", D_MAX), opts.get("lat", 9), opts.get("circ", 48))
	else:
		support = TirePatchNormalSupport.create_with_tire(ground, wheel, geo(), R, Vector3.ZERO, Vector3.UP, opts.get("k", K_A), opts.get("c", C_A),
				opts.get("dmax", D_MAX), opts.get("lat", 9), opts.get("circ", 48), spec, opts.get("friction", MU_S))
	w.add_constraint(support)
	return {"world": w, "ground": ground, "wheel": wheel, "support": support, "spec": spec, "u": u, "e_x": e_x, "e_y": e_y}


## HEAVY one-step rig with the crown compressed by `delta`.
func heavy_rig(delta: float = DELTA, extra: Dictionary = {}) -> Dictionary:
	var camber: float = extra.get("camber", 0.0)
	var opts: Dictionary = {"mass": HEAVY, "height": R - delta * cos(camber)}
	for key in extra:
		opts[key] = extra[key]
	return make_rig(opts)


## prepare / `iterations` solver iterations / finish on the constraint alone (ConstraintSolver's order, without the split pass).
func solve_in_place(rig: Dictionary, iterations: int = 20) -> void:
	var s = rig["support"]
	s.prepare(DT, rig["world"].config)
	for i in iterations:
		s.solve_iteration()
	s.finish()


# ---------------------------------------------------------------- independent oracles

## {load, point}: the sum of the sample loads and their load-weighted position, from sample_report().
func oracle_weighted(report: Array) -> Dictionary:
	var total: float = 0.0
	var wp: Vector3 = Vector3.ZERO
	for e in report:
		total += e["load"]
		wp += e["position"] * e["load"]
	return {"load": total, "point": wp / total if total > 0.0 else Vector3.ZERO}


## Closed-form load (N) of the UPRIGHT wheel at centre height h on y = 0 (the Stage B oracle: circumferential integral in closed form,
## lateral midpoint rule with 2000 nodes).
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


## Velocity of the wheel material at world point p (rigid body).
func point_velocity(wheel: SimRigidBody, p: Vector3) -> Vector3:
	return wheel.linear_velocity + wheel.angular_velocity.cross(p - wheel.position)


## Replaces the accumulated impulse of every active row by f(position), keeping the rows' order (row k is sample_report()[k]).
func prescribe(support, impulse_of: Callable) -> void:
	var report: Array = support.sample_report()
	for k in support.rows.size():
		support.rows[k].accumulated_impulse = impulse_of.call(report[k]["position"])


# ---------------------------------------------------------------- A: aggregate contact state

func test_a_aggregate_load_impulse_and_effective_point_follow_the_sample_loads() -> void:
	var rig: Dictionary = heavy_rig(DELTA)
	solve_in_place(rig)
	var s = rig["support"]
	var report: Array = s.sample_report()
	check(report.size() > 20, "fixture error: the footprint must hold many samples")
	var o: Dictionary = oracle_weighted(report)
	check_near(s.aggregate_normal_load(), o["load"], 1.0e-6 * o["load"], "aggregate load = sum of the sample loads")
	check_near(s.aggregate_normal_impulse(), o["load"] * DT, 1.0e-6 * o["load"] * DT, "aggregate impulse = sum of the sample impulses")
	check_near(s.aggregate_normal_load(), s.aggregate_normal_impulse() / DT, 1.0e-9 * o["load"], "N = Lambda / dt")
	check_near(s.normal_load(), s.aggregate_normal_load(), 1.0e-6 * o["load"], "the Stage B load and the aggregate agree after finish()")
	check(s.has_aggregate_contact(), "a loaded patch has an aggregate contact")
	check_vec3_near(s.effective_contact_point(), o["point"], 2.0e-6, "effective point = normal-impulse weighted sample position")
	# Upright and fore-aft symmetric: the point is on the wheel's mid plane (x = 0) at about z = 0; it lies on the tread, below the centre.
	check_near(s.effective_contact_point().x, 0.0, 2.0e-6, "upright: the effective point is on the wheel's mid plane")
	check_near(s.effective_contact_point().z, 0.0, 1.0e-3, "upright quasi-static: the load is fore-aft symmetric to the sampling ripple")
	check(s.effective_contact_point().y < rig["wheel"].position.y - 0.25 and s.effective_contact_point().y > -1.0e-6 - DELTA, "the point is on the tread under the centre")
	# Camber moves the point across the tread (towards the side of the wheel that is lower) and the mirror image mirrors it.
	var lean_plus: Dictionary = heavy_rig(DELTA, {"camber": 0.12})
	var lean_minus: Dictionary = heavy_rig(DELTA, {"camber": -0.12})
	solve_in_place(lean_plus)
	solve_in_place(lean_minus)
	var o_plus: Dictionary = oracle_weighted(lean_plus["support"].sample_report())
	check_vec3_near(lean_plus["support"].effective_contact_point(), o_plus["point"], 2.0e-6, "cambered: effective point = weighted sample position")
	var x_plus: float = lean_plus["support"].effective_contact_point().x - lean_plus["wheel"].position.x
	var x_minus: float = lean_minus["support"].effective_contact_point().x - lean_minus["wheel"].position.x
	check(absf(x_plus) > 5.0e-3, "camber must move the effective point across the tread (%s m)" % x_plus)
	check_near(x_plus, -x_minus, 2.0e-6, "mirror camber mirrors the effective point")


func test_a_the_effective_point_migrates_with_the_load_distribution() -> void:
	var rig: Dictionary = heavy_rig(DELTA)
	var s = rig["support"]
	s.prepare(DT, rig["world"].config)
	var base: float = 0.05
	var gradients: Array = [Vector3.ZERO, Vector3(0.0, 0.0, 30.0), Vector3(0.0, 0.0, -30.0), Vector3(25.0, 0.0, 0.0), Vector3(-25.0, 0.0, 0.0)]
	var points: Array = []
	for gradient in gradients:
		prescribe(s, func(p: Vector3) -> float: return base * (1.0 + gradient.dot(p - rig["wheel"].position)))
		var report: Array = s.sample_report()
		var total: float = 0.0
		var wp: Vector3 = Vector3.ZERO
		for k in report.size():
			var l: float = s.rows[k].accumulated_impulse
			total += l
			wp += report[k]["position"] * l
		var expected: Vector3 = wp / total
		check_vec3_near(s.effective_contact_point(), expected, 2.0e-6, "effective point for load gradient %s" % gradient)
		check_near(s.aggregate_normal_impulse(), total, 1.0e-9 * total, "aggregate impulse for gradient %s" % gradient)
		points.append(s.effective_contact_point())
	check(points[1].z > points[0].z + 1.0e-3 and points[2].z < points[0].z - 1.0e-3, "a fore-aft load gradient moves the point along z")
	check(points[3].x > points[0].x + 1.0e-3 and points[4].x < points[0].x - 1.0e-3, "a lateral load gradient moves the point across the tread")
	check_near(points[1].z - points[0].z, points[0].z - points[2].z, 1.0e-4, "opposite gradients move the point by opposite amounts")


func test_a_the_effective_velocity_is_the_load_weighted_velocity_of_the_samples() -> void:
	var rig: Dictionary = make_rig({"height": R - DELTA, "forward_speed": 6.0, "lateral_speed": -1.5, "spin": 15.0,
			"omega": Vector3(0.3, 2.0, 0.8), "camber": 0.1})
	var s = rig["support"]
	var wheel: SimRigidBody = rig["wheel"]
	s.prepare(DT, rig["world"].config)
	prescribe(s, func(p: Vector3) -> float: return 0.04 * (1.0 + 18.0 * (p.z - wheel.position.z) - 9.0 * (p.x - wheel.position.x) + 2.0 * sin(40.0 * p.z)))
	var report: Array = s.sample_report()
	var total: float = 0.0
	var weighted: Vector3 = Vector3.ZERO
	for k in report.size():
		var l: float = s.rows[k].accumulated_impulse
		total += l
		weighted += point_velocity(wheel, report[k]["position"]) * l
	var expected: Vector3 = weighted / total
	check_vec3_near(s.effective_contact_velocity(), expected, 2.0e-5, "v_eff = weighted mean of the sample velocities of a rigid wheel")
	# It is not the velocity of any single privileged point: the first sample, the heaviest sample and the centre-line point all differ.
	var heaviest: int = 0
	for k in report.size():
		if s.rows[k].accumulated_impulse > s.rows[heaviest].accumulated_impulse:
			heaviest = k
	check((s.effective_contact_velocity() - point_velocity(wheel, report[0]["position"])).length() > 0.05, "not the velocity of the first sample")
	check((s.effective_contact_velocity() - point_velocity(wheel, report[heaviest]["position"])).length() > 1.0e-3, "not the velocity of the heaviest sample")
	check((s.effective_contact_velocity() - wheel.linear_velocity).length() > 0.5, "not the hub velocity")
	# It uses the CURRENT velocities: change them and the result follows (no cached value).
	wheel.linear_velocity += Vector3(0.0, 0.0, -2.0)
	wheel.angular_velocity += Vector3(1.0, 0.0, 0.0)
	var current: Vector3 = Vector3.ZERO
	for k in report.size():
		current += point_velocity(wheel, report[k]["position"]) * s.rows[k].accumulated_impulse
	check_vec3_near(s.effective_contact_velocity(), current / total, 2.0e-5, "v_eff follows the current velocities")


func test_a_zero_load_and_invalid_states_give_a_safe_aggregate_never_nan_never_stale() -> void:
	# A fresh patch before any step.
	var fresh: Dictionary = make_rig({"height": R - DELTA})
	var f = fresh["support"]
	check(f.aggregate_normal_impulse() == 0.0 and f.aggregate_normal_load() == 0.0 and not f.has_aggregate_contact(), "no aggregate before the first step")
	check(f.effective_contact_point() == Vector3.ZERO and f.effective_contact_velocity() == Vector3.ZERO, "safe no-load point and velocity")
	check(not f.current_slip()["valid"], "no slip without an aggregate contact")
	# Wheel clear of the ground.
	var air: Dictionary = make_rig({"height": R + 0.05, "forward_speed": 3.0, "spin": 5.0})
	solve_in_place(air)
	var a = air["support"]
	check(not a.has_aggregate_contact() and a.effective_contact_point() == Vector3.ZERO and a.longitudinal_axis() == Vector3.ZERO, "airborne: no aggregate, no axes")
	# Compressed rows with zero impulse (a state inside an iteration): no division by zero, no tyre response.
	var zero: Dictionary = make_rig({"height": R - DELTA, "forward_speed": 4.0, "spin": 3.0})
	var z = zero["support"]
	var wheel: SimRigidBody = zero["wheel"]
	z.prepare(DT, zero["world"].config)
	prescribe(z, func(_p: Vector3) -> float: return 0.0)
	var v0: Vector3 = wheel.linear_velocity
	var w0: Vector3 = wheel.angular_velocity
	check(not z.has_aggregate_contact() and z.effective_contact_point() == Vector3.ZERO and z.aggregate_normal_impulse() == 0.0, "zero impulse: safe result")
	z._solve_tire_iteration()  # the tyre step alone, on rows that carry no impulse
	check(wheel.linear_velocity.is_finite() and wheel.angular_velocity.is_finite(), "no NaN from a zero aggregate")
	check_vec3_near(wheel.linear_velocity, v0, 0.0, "a zero aggregate must not change the linear velocity")
	check_vec3_near(wheel.angular_velocity, w0, 0.0, "a zero aggregate must not change the angular velocity")
	check(z.longitudinal_force() == 0.0 and z.lateral_force() == 0.0 and z.rolling_resistance_torque() == 0.0, "zero aggregate: zero tyre response")
		# A non-finite aggregate impulse is reported as invalid, withdraws the response in that iteration and clears the patch in finish().
	var bad: Dictionary = make_rig({"height": R - DELTA, "forward_speed": 4.0, "spin": 3.0})
	var b = bad["support"]
	b.prepare(DT, bad["world"].config)
	b.rows[0].accumulated_impulse = NAN
	check(b.aggregate_normal_impulse() == 0.0 and not b.has_aggregate_contact() and b.effective_contact_point() == Vector3.ZERO, "NaN impulse: safe aggregate")
	b._solve_tire_iteration()
	check(bad["wheel"].linear_velocity.is_finite() and bad["wheel"].angular_velocity.is_finite(), "the non-finite aggregate must not reach the wheel")
	b.finish()
	check(not b.contact_active and b.rows.is_empty() and b.last_reason.contains("non-finite"), "a non-finite aggregate invalidates the whole patch with a reason (%s)" % b.last_reason)
	check(b.longitudinal_force() == 0.0 and b.rolling_resistance_torque() == 0.0 and b.carried_tire_impulse() == Vector3.ZERO, "and leaves no tyre state")


func test_a_the_aggregate_follows_the_solver_iterations_not_only_finish() -> void:
	# A light wheel pressed into the ground: the sequential normal rows change the aggregate impulse from iteration to iteration.
	var rig: Dictionary = make_rig({"height": R - 0.025, "velocity": Vector3(0.0, -1.0, 0.0)})
	var s = rig["support"]
	s.prepare(DT, rig["world"].config)
	var history: Array = []
	for i in 6:
		s.solve_iteration()
		var rows_sum: float = 0.0
		for row in s.rows:
			rows_sum += row.accumulated_impulse
		check_near(s.aggregate_normal_impulse(), rows_sum, 1.0e-9 * (rows_sum + 1.0), "iteration %d: aggregate = current row impulses" % i)
		check_near(s.normal_load(), 0.0, 0.0, "iteration %d: the finish()-based load is not yet available" % i)
		history.append(rows_sum)
	check(absf(history[1] - history[0]) > 1.0e-3 * history[0] and absf(history[3] - history[2]) > 1.0e-4 * history[0],
			"the aggregate must still change between iterations (%s)" % [history])
	s.finish()
	check_near(s.normal_load(), history[5] / DT, 1.0e-6 * history[5] / DT, "after finish() the load is the last iteration's aggregate / dt")


# ---------------------------------------------------------------- B: tyre axes and slip

func hand_axes(u: Vector3, n: Vector3) -> Array:
	var e_x: Vector3 = n.cross(u).normalized()
	return [e_x, e_x.cross(n)]


func test_b_axes_follow_the_tyre_model_conventions_in_the_ground_plane() -> void:
	var cases: Array = [[0.0, 0.0], [0.0, 0.3], [0.0, -0.3], [0.5, 0.0], [0.5, 0.3], [-1.2, -0.25], [2.5, 0.2]]
	for c in cases:
		var rig: Dictionary = heavy_rig(DELTA, {"yaw": c[0], "camber": c[1]})
		solve_in_place(rig)
		var s = rig["support"]
		var e_x: Vector3 = s.longitudinal_axis()
		var e_y: Vector3 = s.lateral_axis()
		var by_hand: Array = hand_axes(rig["u"], Vector3.UP)
		var label: String = "yaw %s camber %s" % [c[0], c[1]]
		check_vec3_near(e_x, by_hand[0], 2.0e-6, "longitudinal axis (%s)" % label)
		check_vec3_near(e_y, by_hand[1], 2.0e-6, "lateral axis (%s)" % label)
		check_near(e_x.length(), 1.0, 2.0e-6, "unit longitudinal axis (%s)" % label)
		check_near(e_y.length(), 1.0, 2.0e-6, "unit lateral axis (%s)" % label)
		check_near(e_x.dot(Vector3.UP), 0.0, 2.0e-6, "longitudinal axis in the ground plane (%s)" % label)
		check_near(e_y.dot(Vector3.UP), 0.0, 2.0e-6, "lateral axis in the ground plane (%s)" % label)
		check_near(e_x.dot(e_y), 0.0, 2.0e-6, "orthogonal axes (%s)" % label)
		# Sign conventions of TireModel: forward is -Z for the wheel with yaw 0, the lateral axis points to the wheel's right (u.e_y > 0).
		check(e_y.dot(rig["u"]) > 0.9, "the lateral axis points to the wheel's right (%s)" % label)
		var forward: Vector3 = Quaternion(Vector3.UP, c[0]) * Vector3(0.0, 0.0, -1.0)
		check_vec3_near(e_x, forward, 2.0e-6, "forward = the yawed -Z (%s): camber does not rotate the longitudinal axis" % label)
		check_vec3_near(e_y, Quaternion(Vector3.UP, c[0]) * Vector3.RIGHT, 2.0e-6, "lateral = the yawed +X: camber does not change the horizontal projection (%s)" % label)
		var model: Dictionary = TireModel.axes(rig["u"], Vector3.UP)
		check_vec3_near(e_x, model["longitudinal"], 2.0e-6, "agrees with TireModel.axes (%s)" % label)
	# A tilted ground plane: the axes are those of the plane (in the plane, built from the plane normal), not of the world.
	var tilt: Quaternion = Quaternion(Vector3(0.0, 0.0, 1.0), 0.08)
	var n: Vector3 = tilt * Vector3.UP
	var w: SimWorld = make_world()
	var ground: SimRigidBody = make_static()
	var wheel: SimRigidBody = make_body(HEAVY, MotorcycleConfig.wheel_inertia(HEAVY, R, 0.75 * R, W), n * (R - DELTA), tilt)
	w.add_body(ground)
	w.add_body(wheel)
	var support = TirePatchNormalSupport.create_with_tire(ground, wheel, geo(), R, Vector3.ZERO, n, K_A, C_A, D_MAX, 9, 48, spec_with(), MU_S)
	w.add_constraint(support)
	solve_in_place({"world": w, "support": support})
	var tilted: Array = hand_axes(tilt * Vector3.RIGHT, n)
	check(support.contact_active, "fixture error: the tilted plane must be loaded")
	check_vec3_near(support.longitudinal_axis(), tilted[0], 3.0e-6, "tilted plane: longitudinal axis")
	check_near(support.longitudinal_axis().dot(n), 0.0, 3.0e-6, "tilted plane: longitudinal axis in the plane")
	check_near(support.lateral_axis().dot(n), 0.0, 3.0e-6, "tilted plane: lateral axis in the plane")


func test_b_a_degenerate_axle_gives_no_tyre_force_but_keeps_the_normal_support() -> void:
	# Wheel lying flat (axle = plane normal): |n x u| = 0, the tyre axes are undefined. The tread still touches (its lowest side).
	var rig: Dictionary = make_rig({"height": 0.075, "camber": PI * 0.5, "forward_speed": 5.0, "spin": 20.0})
	var s = rig["support"]
	var wheel: SimRigidBody = rig["wheel"]
	var horizontal_before: Vector3 = Vector3(wheel.linear_velocity.x, 0.0, wheel.linear_velocity.z)
	solve_in_place(rig)
	check(s.contact_active and s.normal_load() > 0.0, "fixture error: a flat wheel must still carry normal support")
	check(s.tangential_response_count() == 0 and s.longitudinal_axis() == Vector3.ZERO, "degenerate axes: no tangential response")
	check(s.longitudinal_force() == 0.0 and s.lateral_force() == 0.0 and s.rolling_resistance_torque() == 0.0, "degenerate axes: no force, no rolling resistance")
	check_vec3_near(Vector3(wheel.linear_velocity.x, 0.0, wheel.linear_velocity.z), horizontal_before, 0.0, "degenerate axes: no tangential impulse on the wheel")
	check(s.carried_tire_impulse() == Vector3.ZERO, "degenerate axes: nothing carried")
	check(s.tangential_force_world() == Vector3.ZERO and s.tangential_moment_world() == Vector3.ZERO, "degenerate axes: no tangential wrench")


func test_b_slip_is_evaluated_at_the_effective_point_with_the_current_velocities() -> void:
	var spec = spec_with({"rolling_radius": 0.95 * R})
	var rig: Dictionary = make_rig({"height": R - DELTA, "forward_speed": 7.0, "lateral_speed": -0.8, "spin": 20.0, "omega": Vector3(0.2, 1.5, 0.5),
			"camber": 0.15, "yaw": 0.4, "spec": spec})
	var s = rig["support"]
	var wheel: SimRigidBody = rig["wheel"]
	s.prepare(DT, rig["world"].config)
	prescribe(s, func(p: Vector3) -> float: return 0.05 * (1.0 + 12.0 * (p.z - wheel.position.z) + 6.0 * (p.x - wheel.position.x)))
	var report: Array = s.sample_report()
	var total: float = 0.0
	var wp: Vector3 = Vector3.ZERO
	var vw: Vector3 = Vector3.ZERO
	for k in report.size():
		var l: float = s.rows[k].accumulated_impulse
		total += l
		wp += report[k]["position"] * l
		vw += point_velocity(wheel, report[k]["position"]) * l
	var p_eff: Vector3 = wp / total
	var v_eff: Vector3 = vw / total
	var u: Vector3 = rig["u"]
	var e_x: Vector3 = rig["e_x"]
	var e_y: Vector3 = rig["e_y"]
	var spin: float = -wheel.angular_velocity.dot(u)
	var s_x: float = -v_eff.dot(e_x) + (spec.rolling_radius - R) * spin
	var s_y: float = -v_eff.dot(e_y)
	var hub: float = wheel.linear_velocity.dot(e_x)
	var d: float = sqrt(hub * hub + spec.low_speed_reference * spec.low_speed_reference)
	var slip: Dictionary = s.current_slip()
	check(slip["valid"], "slip must be valid with an aggregate contact")
	check_vec3_near(slip["contact_point"], p_eff, 2.0e-6, "slip is evaluated at the effective contact point")
	check_vec3_near(slip["contact_velocity"], v_eff, 2.0e-5, "slip uses the weighted contact velocity")
	check_near(slip["slip_velocity_x"], s_x, 2.0e-5, "longitudinal slip velocity (with the rolling-radius correction)")
	check_near(slip["slip_velocity_y"], s_y, 2.0e-5, "lateral slip velocity")
	check_near(slip["slip_x"], s_x / d, 2.0e-5, "longitudinal slip with the low-speed reference")
	check_near(slip["slip_y"], s_y / d, 2.0e-5, "lateral slip")
	check(absf((spec.rolling_radius - R) * spin) > 0.1, "fixture: the rolling-radius correction must matter")
	# Current velocities, not cached: change the wheel's state and the slip follows.
	wheel.linear_velocity += e_x * 2.0
	check(absf(s.current_slip()["slip_velocity_x"] - (s_x - 2.0)) < 3.0e-5 and s.current_slip()["hub_speed"] > hub + 1.9, "slip follows the current velocities")
	# Low-speed regularisation: at rest the slip is finite and equals s / v_ref.
	var rest: Dictionary = heavy_rig(DELTA, {"lateral_speed": 0.05})
	solve_in_place(rest)
	var rest_slip: Dictionary = rest["support"].current_slip()
	check(rest_slip["valid"] and is_finite(rest_slip["slip_y"]), "finite slip at rest")
	check_near(rest_slip["reference_speed"], rest["spec"].low_speed_reference, 1.0e-9, "at rest the slip denominator is the low-speed reference")
	check_near(rest_slip["slip_y"], -0.05 / rest["spec"].low_speed_reference, 1.0e-5, "lateral slip at rest = s_y / v_ref (the wheel moves to the right, the force must point left)")


# ---------------------------------------------------------------- C: tangential response

## Force law at the aggregate state of a HEAVY rig after solve_in_place(): slip from TireModel.slip at the independently derived effective point,
## load from the sum of the sample loads, the combined-slip law TireModel.force.
func law_at_aggregate(rig: Dictionary) -> Dictionary:
	var s = rig["support"]
	var wheel: SimRigidBody = rig["wheel"]
	var spec = rig["spec"]
	var o: Dictionary = oracle_weighted(s.sample_report())
	var slip: Dictionary = TireModel.slip(wheel.position, wheel.orientation, wheel.linear_velocity, wheel.angular_velocity, R, spec.rolling_radius,
			Vector3.ZERO, Vector3.UP, spec.low_speed_reference, o["point"])
	var law: Dictionary = TireModel.force(slip["slip_x"], slip["slip_y"], o["load"], spec.grip_longitudinal * MU_S, spec.grip_lateral * MU_S,
			spec.longitudinal_stiffness, spec.lateral_stiffness)
	law["load"] = o["load"]
	law["point"] = o["point"]
	law["slip"] = slip
	return law


func test_c_longitudinal_force_follows_the_tyre_law_at_the_aggregate_load() -> void:
	for ratio in [1.15, 1.03, 0.8, 0.0]:
		var rig: Dictionary = heavy_rig(DELTA, {"forward_speed": 12.0, "spin": ratio * 12.0 / R})
		solve_in_place(rig)
		var s = rig["support"]
		var law: Dictionary = law_at_aggregate(rig)
		var tol: float = 2.0e-4 * (absf(law["fx"]) + 1.0)
		check_near(s.longitudinal_force(), law["fx"], tol, "longitudinal force, spin ratio %s" % ratio)
		check_near(s.lateral_force(), law["fy"], 2.0e-4 * (absf(law["fy"]) + 1.0), "lateral force, spin ratio %s" % ratio)
		check_vec3_near(s.tangential_force_world(), rig["e_x"] * law["fx"] + rig["e_y"] * law["fy"], tol + 1.0e-3, "world force, spin ratio %s" % ratio)
		if ratio > 1.0:
			check(s.longitudinal_force() > 10.0, "driving: the force points forward (+e_x)")
		else:
			check(s.longitudinal_force() < -10.0, "braking or locked: the force points backward (-e_x)")
		if ratio == 0.0:
			check_near(absf(s.longitudinal_force()), MU_S * law["load"], 1.0e-3 * law["load"], "a locked wheel carries mu N")
			check_near(s.friction_utilization(), 1.0, 1.0e-4, "locked wheel: the ellipse is saturated")


func test_c_lateral_force_direction_and_magnitude_follow_the_tyre_law() -> void:
	for lateral in [-3.0, -0.4, 0.4, 3.0]:
		var rig: Dictionary = heavy_rig(DELTA, {"forward_speed": 8.0, "spin": 8.0 / R, "lateral_speed": lateral, "yaw": 0.5, "camber": 0.2})
		solve_in_place(rig)
		var s = rig["support"]
		var law: Dictionary = law_at_aggregate(rig)
		check_near(s.lateral_force(), law["fy"], 2.0e-4 * (absf(law["fy"]) + 1.0), "lateral force for lateral speed %s" % lateral)
		check_near(s.longitudinal_force(), law["fx"], 2.0e-4 * (absf(law["fx"]) + 1.0), "longitudinal force for lateral speed %s" % lateral)
		# The wheel moving to the LEFT (-e_y) is pushed to the right (+e_y) and vice versa.
		check(s.lateral_force() * lateral < -5.0, "the lateral force opposes the sideways motion (%s N for %s m/s)" % [s.lateral_force(), lateral])


func test_c_combined_slip_is_one_ellipse_with_one_common_factor_on_the_aggregate_load() -> void:
	var spec = spec_with({"grip_longitudinal": 1.2, "grip_lateral": 0.6})
	var rig: Dictionary = heavy_rig(DELTA, {"forward_speed": 10.0, "spin": 0.0, "lateral_speed": 6.0, "spec": spec, "camber": 0.1})
	solve_in_place(rig)
	var s = rig["support"]
	var law: Dictionary = law_at_aggregate(rig)
	var n: float = law["load"]
	var ux: float = s.longitudinal_force() / (1.2 * MU_S * n)
	var uy: float = s.lateral_force() / (0.6 * MU_S * n)
	check(ux * ux + uy * uy <= 1.0 + 1.0e-6, "the aggregate force must lie inside the ellipse of the aggregate load (%s)" % (ux * ux + uy * uy))
	check_near(sqrt(ux * ux + uy * uy), law["utilization"], 2.0e-4, "utilisation equals the law's g(rho)")
	check_near(s.friction_utilization(), law["utilization"], 2.0e-4, "reported utilisation")
	check_near(s.longitudinal_force(), law["fx"], 2.0e-4 * absf(law["fx"]), "longitudinal component")
	check_near(s.lateral_force(), law["fy"], 2.0e-4 * absf(law["fy"]), "lateral component")
	var slip: Dictionary = law["slip"]
	var phi_x: float = s.longitudinal_force() / (spec.longitudinal_stiffness * n * slip["slip_x"])
	var phi_y: float = s.lateral_force() / (spec.lateral_stiffness * n * slip["slip_y"])
	check_near(phi_x, phi_y, 2.0e-4, "one common reduction factor on both axes (no per-axis clamp)")
	check(phi_x < 0.9, "fixture: the slip must be strongly saturating (phi = %s)" % phi_x)


func test_c_the_implicit_coupled_response_is_preserved() -> void:
	# Light wheel, strong combined slip, normal rows frozen at prescribed impulses so that only the tangential response iterates.
	var rig: Dictionary = make_rig({"height": R - DELTA, "forward_speed": 5.0, "spin": 1.05 * 5.0 / R, "lateral_speed": 0.3, "yaw": 0.3, "camber": 0.25})
	var s = rig["support"]
	var wheel: SimRigidBody = rig["wheel"]
	var spec = rig["spec"]
	s.prepare(DT, rig["world"].config)
	prescribe(s, func(_p: Vector3) -> float: return 0.3)
	var hub0: float = wheel.linear_velocity.dot(rig["e_x"])
	var d0: float = sqrt(hub0 * hub0 + spec.low_speed_reference * spec.low_speed_reference)
	var p_eff: Vector3 = s.effective_contact_point()
	var slip0: Dictionary = TireModel.slip(wheel.position, wheel.orientation, wheel.linear_velocity, wheel.angular_velocity, R, spec.rolling_radius,
			Vector3.ZERO, Vector3.UP, spec.low_speed_reference, p_eff)
	var ln: float = s.aggregate_normal_impulse()
	var mu_x: float = spec.grip_longitudinal * MU_S
	var mu_y: float = spec.grip_lateral * MU_S
	var explicit: Vector2 = Vector2.ZERO
	var e0: float = spec.longitudinal_stiffness * slip0["slip_velocity_x"] / d0 / mu_x
	var e1: float = spec.lateral_stiffness * slip0["slip_velocity_y"] / d0 / mu_y
	var phi0: float = TireModel.reduction(sqrt(e0 * e0 + e1 * e1))
	explicit = Vector2(phi0 * spec.longitudinal_stiffness * ln * slip0["slip_velocity_x"] / d0, phi0 * spec.lateral_stiffness * ln * slip0["slip_velocity_y"] / d0)
	for i in 60:
		s._solve_tire_iteration()
	var lam: Vector2 = Vector2(s.longitudinal_force() * DT, s.lateral_force() * DT)
	var slip1: Dictionary = TireModel.slip(wheel.position, wheel.orientation, wheel.linear_velocity, wheel.angular_velocity, R, spec.rolling_radius,
			Vector3.ZERO, Vector3.UP, spec.low_speed_reference, p_eff)
	var sx: float = slip1["slip_velocity_x"] / d0
	var sy: float = slip1["slip_velocity_y"] / d0
	var rho: float = sqrt(pow(spec.longitudinal_stiffness * sx / mu_x, 2.0) + pow(spec.lateral_stiffness * sy / mu_y, 2.0))
	var phi: float = TireModel.reduction(rho)
	# Backward Euler: lambda = A(sigma_end) sigma_end with the slip AFTER the response (independent of the constraint's response matrix).
	var implicit_x: float = phi * spec.longitudinal_stiffness * ln * sx
	var implicit_y: float = phi * spec.lateral_stiffness * ln * sy
	check_near(lam.x, implicit_x, 2.0e-3 * absf(implicit_x) + 1.0e-6, "lambda_x satisfies the implicit relation")
	check_near(lam.y, implicit_y, 2.0e-3 * absf(implicit_y) + 1.0e-6, "lambda_y satisfies the implicit relation")
	check(absf(lam.x - explicit.x) > 0.2 * absf(explicit.x), "fixture: the implicit result must differ from the explicit one (%s vs %s)" % [lam.x, explicit.x])
	check(absf(slip1["slip_velocity_x"]) < 0.5 * absf(slip0["slip_velocity_x"]), "the response removes slip within the step")
	# Pure helper: the coupled 2x2 system (off-diagonal terms) against an explicit solution, and the difference to a decoupled solve.
	var g: Array = [0.15, 0.09, -0.07, 0.2]
	var target: Vector2 = TireModel.implicit_tangential_impulse(2.0, -1.0, 3.0, 0.0, 0.0, g[0], g[1], g[2], g[3], 15.0, 10.0, 0.8, 0.8, 0.5)
	var sxh: float = 2.0
	var syh: float = -1.0
	var ph: float = TireModel.reduction(sqrt(pow(15.0 * sxh / 0.5 / 0.8, 2.0) + pow(10.0 * syh / 0.5 / 0.8, 2.0)))
	var ax: float = ph * 15.0 * 3.0
	var ay: float = ph * 10.0 * 3.0
	var m: Array = [1.0 - ax * g[0] / 0.5, -ax * g[1] / 0.5, -ay * g[2] / 0.5, 1.0 - ay * g[3] / 0.5]
	var det: float = m[0] * m[3] - m[1] * m[2]
	var bx: float = ax * sxh / 0.5
	var by: float = ay * syh / 0.5
	var ex: float = (m[3] * bx - m[1] * by) / det
	var ey: float = (-m[2] * bx + m[0] * by) / det
	var projected: Vector2 = TireModel.into_ellipse(ex, ey, 3.0, 0.8, 0.8)
	check_near(target.x, projected.x, 1.0e-5, "helper: coupled solve x")
	check_near(target.y, projected.y, 1.0e-5, "helper: coupled solve y")
	var decoupled_x: float = (ax * sxh / 0.5) / m[0]
	check(absf(decoupled_x - ex) > 0.02 * absf(ex), "fixture: the coupling must matter (%s vs %s)" % [decoupled_x, ex])


func test_c_the_friction_limit_follows_the_aggregate_normal_impulse_in_the_same_iteration() -> void:
	var rig: Dictionary = make_rig({"height": R - 0.02, "forward_speed": 8.0, "spin": 0.0, "velocity": Vector3(0.0, -0.5, 0.0)})
	var s = rig["support"]
	s.prepare(DT, rig["world"].config)
	var previous: float = 0.0
	var worst_current: float = 0.0
	var worst_stale: float = 0.0
	for i in 12:
		s.solve_iteration()
		var ln: float = 0.0
		for row in s.rows:
			ln += row.accumulated_impulse
		var lx: float = s.longitudinal_force() * DT
		var ly: float = s.lateral_force() * DT
		var util: float = sqrt(pow(lx / (MU_S * ln), 2.0) + pow(ly / (MU_S * ln), 2.0))
		# Iterations 0 and 1 are not saturated yet: the implicit relation (the wheel's slip drops within the step) needs the normal rows to
		# settle (measured: utilisation 0.875 at iteration 0). From iteration 2 on the locked wheel sits on the ellipse of the CURRENT load.
		if i >= 2:
			worst_current = maxf(worst_current, absf(util - 1.0))
			worst_stale = maxf(worst_stale, absf(sqrt(pow(lx / (MU_S * previous), 2.0) + pow(ly / (MU_S * previous), 2.0)) - 1.0))
		previous = ln
	check(worst_current < 2.0e-3, "the saturated tangential impulse equals mu * the CURRENT aggregate impulse in every iteration from the third on (worst %s; the lagged phi leaves ~5e-4)" % worst_current)
	check(worst_stale > 5.0e-3, "fixture: the aggregate impulse must change between iterations, otherwise this test cannot see a stale load (%s)" % worst_stale)


func test_c_the_tangential_impulse_is_applied_once_at_the_effective_point() -> void:
	var rig: Dictionary = make_rig({"height": R - DELTA, "forward_speed": 5.0, "spin": 1.2 * 5.0 / R, "lateral_speed": 0.7, "camber": 0.2,
			"omega": Vector3(0.0, 1.0, 0.0), "spec_overrides": {"rolling_resistance": 0.0}})
	var s = rig["support"]
	var wheel: SimRigidBody = rig["wheel"]
	var v0: Vector3 = wheel.linear_velocity
	var w0: Vector3 = wheel.angular_velocity
	solve_in_place(rig)
	var o: Dictionary = oracle_weighted(s.sample_report())
	var lam_n: float = o["load"] * DT
	var linear_change: Vector3 = wheel.mass * (wheel.linear_velocity - v0)
	var tangential: Vector3 = linear_change - Vector3.UP * lam_n
	var reported: Vector3 = s.tangential_force_world() * DT
	check(reported.length() > 0.3, "fixture: a non-trivial tangential impulse is expected (%s)" % reported.length())
	check_vec3_near(tangential, reported, 2.0e-4 * (reported.length() + lam_n), "m dv - n Lambda = the reported tangential impulse (applied ONCE, not per sample)")
	check_near(tangential.dot(Vector3.UP), 0.0, 2.0e-4 * (lam_n + 1.0), "no normal component in the tangential impulse")
	check_vec3_near(reported, rig["e_x"] * s.longitudinal_force() * DT + rig["e_y"] * s.lateral_force() * DT, 1.0e-6, "the impulse is e_x lx + e_y ly")
	var normal_moment: Vector3 = Vector3.ZERO
	for e in s.sample_report():
		normal_moment += (e["position"] - wheel.position).cross(Vector3.UP) * (e["load"] * DT)
	var lever: Vector3 = o["point"] - wheel.position
	var tangential_moment: Vector3 = lever.cross(reported)
	var angular_change: Vector3 = wheel.inertia_world * (wheel.angular_velocity - w0)
	var tol: float = 2.0e-3 * (angular_change.length() + 0.1)
	check_vec3_near(angular_change, normal_moment + tangential_moment, tol, "I d(omega) = normal moments + (p_eff - x) x L_tangential")
	check_vec3_near(s.tangential_moment_world() * DT, tangential_moment, 2.0e-3 * (tangential_moment.length() + 0.05), "the reported tangential moment is the resultant at the effective point")
	check((angular_change - normal_moment).distance_to(Vector3.ZERO) > 0.0 and tangential_moment.length() > 20.0 * tol * 0.1 and tangential_moment.length() > 0.05,
			"fixture: the lever arm must matter (a force at the wheel centre would give no moment)")


func test_c_no_generic_friction_exists_next_to_the_tyre() -> void:
	var enabled: Dictionary = make_rig({"gravity": G, "height": R - 0.005, "forward_speed": 10.0, "spin": 0.0})
	var disabled: Dictionary = make_rig({"gravity": G, "height": R - 0.005, "forward_speed": 10.0, "spin": 0.0})
	disabled["support"].tire_forces_enabled = false
	var stage_b: Dictionary = make_rig({"gravity": G, "height": R - 0.005, "forward_speed": 10.0, "spin": 0.0, "no_tire": true})
	for i in 5:
		enabled["world"].step()
		disabled["world"].step()
		stage_b["world"].step()
	check(enabled["wheel"].linear_velocity.dot(enabled["e_x"]) < 10.0 - 0.1, "negative control: the tyre decelerates the sliding wheel")
	for rig in [disabled, stage_b]:
		var v: Vector3 = rig["wheel"].linear_velocity
		check(v.dot(rig["e_x"]) == 10.0 and v.dot(rig["e_y"]) == 0.0, "no tyre: no tangential impulse of any kind (generic friction would slow the wheel)")
		check(rig["support"].normal_load() > 50.0, "no tyre: the normal support stays")
		check(rig["support"].tangential_response_count() == 0 and rig["support"].longitudinal_force() == 0.0, "no tyre: no tangential response")
	for row in enabled["support"].rows:
		check(row.linear_b == Vector3.UP, "every row of the patch is a pure normal row")
	check(enabled["support"].tangential_response_count() == 1 and enabled["support"].rows.size() == enabled["support"].active_sample_count(),
			"one tangential response per wheel and one row per physical sample")


# ---------------------------------------------------------------- D: rolling resistance

func rolling_oracle(rig: Dictionary, spin: float) -> float:
	var spec = rig["spec"]
	var n: float = oracle_weighted(rig["support"].sample_report())["load"]
	return -TireModel.rolling_resistance_force(spec.rolling_radius * spin, n, spec.rolling_resistance, spec.rolling_resistance_speed) * spec.rolling_radius


func test_d_rolling_resistance_follows_the_aggregate_load_and_the_spin_convention() -> void:
	var torques: Dictionary = {}
	for spin in [-30.0, -5.0, -1.0, -0.2, -0.02, 0.0, 0.02, 0.2, 1.0, 5.0, 30.0]:
		var rig: Dictionary = heavy_rig(DELTA, {"spin": spin})
		solve_in_place(rig)
		var m: float = rig["support"].rolling_resistance_torque()
		var expected: float = rolling_oracle(rig, spin)
		check_near(m, expected, 2.0e-4 * absf(expected) + 1.0e-9, "rolling-resistance moment for spin %s" % spin)
		check(m * spin >= 0.0, "the moment about +u opposes the spin (a forward spin is about -u): spin %s, M %s" % [spin, m])
		torques[spin] = m
	check(torques[0.0] == 0.0, "exactly zero at zero spin")
	check(absf(torques[0.02]) < absf(torques[0.2]) and absf(torques[0.2]) < absf(torques[1.0]) and absf(torques[1.0]) < absf(torques[30.0]), "monotone growth with the spin")
	var psi_ratio: float = 0.1 * sqrt(1.0 + pow(0.2 * R / 0.2, 2.0)) / sqrt(1.0 + pow(0.02 * R / 0.2, 2.0))
	check_near(torques[0.02] / torques[0.2], psi_ratio, 2.0e-3, "psi(x) = x / sqrt(1 + x^2): linear (smooth) near zero spin")
	check_near(torques[-5.0], -torques[5.0], 1.0e-6 * absf(torques[5.0]), "odd in the spin")
	# Proportional to the aggregate physical load.
	var shallow: Dictionary = heavy_rig(0.01, {"spin": 10.0})
	var deep: Dictionary = heavy_rig(0.025, {"spin": 10.0})
	solve_in_place(shallow)
	solve_in_place(deep)
	var load_ratio: float = deep["support"].aggregate_normal_load() / shallow["support"].aggregate_normal_load()
	check(load_ratio > 2.0, "fixture: the two loads must differ clearly")
	check_near(deep["support"].rolling_resistance_torque() / shallow["support"].rolling_resistance_torque(), load_ratio, 1.0e-4 * load_ratio, "rolling resistance is proportional to the aggregate load")


func test_d_rolling_resistance_follows_the_load_within_the_solve_and_is_applied_once() -> void:
	# Tangential response made negligible (stiffness 1e-9) so that the axle spin changes only through the rolling resistance.
	var weak: Dictionary = {"longitudinal_stiffness": 1.0e-9, "lateral_stiffness": 1.0e-9}
	var per_resolution: Array = []
	for res in [[9, 48], [17, 96]]:
		var rig: Dictionary = make_rig({"height": R - DELTA, "spin": 25.0, "forward_speed": 0.0, "spec_overrides": weak, "lat": res[0], "circ": res[1]})
		var s = rig["support"]
		var wheel: SimRigidBody = rig["wheel"]
		var u: Vector3 = rig["u"]
		var axial: float = wheel.local_inertia.x.x
		s.prepare(DT, rig["world"].config)
		var base_spin: float = wheel.angular_velocity.dot(u)
		prescribe(s, func(_p: Vector3) -> float: return 0.01)
		s._solve_tire_iteration()
		var m1: float = s.rolling_resistance_torque()
		var change1: float = wheel.angular_velocity.dot(u) - base_spin
		prescribe(s, func(_p: Vector3) -> float: return 0.02)
		s._solve_tire_iteration()
		var m2: float = s.rolling_resistance_torque()
		check_near(m2, 2.0 * m1, 1.0e-5 * m2, "%dx%d: the moment follows the doubled aggregate impulse in the same solve" % [res[0], res[1]])
		var change2: float = wheel.angular_velocity.dot(u) - base_spin  # (float32 resolution of a 25 rad/s spin is 2e-6: absolute term 6e-6 below)
		check(change1 > 0.0, "the moment slows the forward spin (omega.u rises towards zero)")
		check_near(change2, m2 * DT / axial, 1.0e-4 * change2 + 6.0e-6, "total spin change = ONE moment impulse of the current load, not their sum")
		for k in 4:
			s._solve_tire_iteration()
		check_near(wheel.angular_velocity.dot(u) - base_spin, change2, 1.0e-4 * change2 + 6.0e-6, "more iterations at the same load do not add more")
		per_resolution.append([m2 / s.aggregate_normal_load(), s.rows.size()])
	check_near(per_resolution[0][0], per_resolution[1][0], 1.0e-5 * per_resolution[0][0], "moment per unit aggregate load does not depend on the sample count")
	check(per_resolution[1][1] > 2 * per_resolution[0][1], "fixture: the finer grid must hold many more samples")


func test_d_rolling_resistance_is_outside_the_friction_ellipse_and_zero_without_load() -> void:
	var low: Dictionary = heavy_rig(DELTA, {"spin": 40.0, "forward_speed": 0.0, "spec_overrides": {"rolling_resistance": 0.0}})
	var high: Dictionary = heavy_rig(DELTA, {"spin": 40.0, "forward_speed": 0.0, "spec_overrides": {"rolling_resistance": 0.1}})
	solve_in_place(low)
	solve_in_place(high)
	check_near(low["support"].friction_utilization(), 1.0, 1.0e-4, "fixture: spinning in place saturates the ellipse")
	check_near(high["support"].longitudinal_force(), low["support"].longitudinal_force(), 1.0e-4 * absf(low["support"].longitudinal_force()), "the ellipse limit does not depend on the rolling-resistance coefficient")
	check(low["support"].rolling_resistance_torque() == 0.0 and high["support"].rolling_resistance_torque() > 1.0, "rolling resistance is a separate moment")
	# Free rolling: almost no slip, no ellipse use, but rolling resistance is present.
	var probe: Dictionary = heavy_rig(DELTA, {"forward_speed": 10.0})
	solve_in_place(probe)
	var lever: float = probe["wheel"].position.y - probe["support"].effective_contact_point().y  # loaded rolling radius under the effective point
	var rolling: Dictionary = heavy_rig(DELTA, {"forward_speed": 10.0, "spin": 10.0 / lever})
	solve_in_place(rolling)
	check(rolling["support"].rolling_resistance_torque() > 1.0, "a rolling wheel has rolling resistance")
	check(rolling["support"].friction_utilization() < 0.05, "and it does not occupy the friction ellipse (%s)" % rolling["support"].friction_utilization())
	# Without physical load: exactly zero.
	var air: Dictionary = make_rig({"height": R + 0.03, "spin": 40.0, "forward_speed": 5.0})
	solve_in_place(air)
	check(air["support"].rolling_resistance_torque() == 0.0 and air["support"].rolling_resistance_force() == 0.0, "no load: no rolling resistance")
	var off: Dictionary = heavy_rig(DELTA, {"spin": 40.0})
	off["support"].tire_forces_enabled = false
	solve_in_place(off)
	check(off["support"].rolling_resistance_torque() == 0.0 and off["support"].normal_load() > 50.0, "tyre forces disabled: no rolling resistance, normal support intact")


# ---------------------------------------------------------------- E: sample density

func test_e_increasing_the_sample_count_does_not_multiply_traction_or_rolling_resistance() -> void:
	var results: Dictionary = {}
	for res in [[9, 48], [17, 96]]:
		var key: String = "%dx%d" % [res[0], res[1]]
		var small: Dictionary = heavy_rig(DELTA, {"forward_speed": 10.0, "spin": 1.10 * 10.0 / R, "lat": res[0], "circ": res[1]})
		var locked: Dictionary = heavy_rig(DELTA, {"forward_speed": 10.0, "spin": 0.0, "lat": res[0], "circ": res[1]})
		var roll: Dictionary = heavy_rig(DELTA, {"spin": 30.0, "forward_speed": 0.0, "lat": res[0], "circ": res[1]})
		solve_in_place(small)
		solve_in_place(locked)
		solve_in_place(roll)
		var law: Dictionary = law_at_aggregate(small)
		# Per resolution the force is held to the tight tolerance against ITS OWN aggregate load and point (no loosening for quadrature).
		check_near(small["support"].longitudinal_force(), law["fx"], 2.0e-4 * absf(law["fx"]), "%s: small-slip force against the law at its own load" % key)
		var n_locked: float = locked["support"].aggregate_normal_load()
		check_near(absf(locked["support"].longitudinal_force()), MU_S * n_locked, 1.0e-4 * n_locked, "%s: the friction limit is mu N of its own aggregate load" % key)
		check_near(roll["support"].rolling_resistance_torque() / roll["support"].aggregate_normal_load(), rolling_oracle(roll, 30.0) / oracle_weighted(roll["support"].sample_report())["load"],
				1.0e-4 * absf(rolling_oracle(roll, 30.0) / oracle_weighted(roll["support"].sample_report())["load"]), "%s: rolling resistance per unit load" % key)
		results[key] = {"n": n_locked, "f_sat": absf(locked["support"].longitudinal_force()), "f_small": small["support"].longitudinal_force(),
				"m": roll["support"].rolling_resistance_torque(), "samples": locked["support"].active_sample_count(), "n_roll": roll["support"].aggregate_normal_load()}
	var a: Dictionary = results["9x48"]
	var b: Dictionary = results["17x96"]
	check(b["samples"] > 3 * a["samples"], "fixture: the fine grid has %d samples against %d" % [b["samples"], a["samples"]])
	# The limit scales with the aggregate normal load only: the ratio of the saturated forces is the ratio of the loads (not of the sample counts).
	check_near(b["f_sat"] / a["f_sat"], b["n"] / a["n"], 1.0e-4, "saturated traction scales with the aggregate load")
	check_near(b["m"] / a["m"], b["n_roll"] / a["n_roll"], 1.0e-4, "rolling resistance scales with the aggregate load")
	# Convergence of the normal-load quadrature itself (Stage B, measured here for the same tyre and compression): the two grids agree with the
	# closed-form load to the stated bounds, and so do the tyre responses derived from them.
	var exact: float = oracle_upright_load(R - DELTA, K_A)
	check(absf(a["n"] - exact) / exact < 0.05 and absf(b["n"] - exact) / exact < 0.005, "quadrature error of the normal load: %s (9x48), %s (17x96) against %s" % [a["n"], b["n"], exact])
	check(absf(b["n"] - exact) < absf(a["n"] - exact) + 1.0e-6 * exact, "the finer grid is closer to the closed form")
	check(absf(b["f_sat"] - MU_S * exact) / (MU_S * exact) < 0.005, "the limit converges to mu times the closed-form load")
	check(b["f_sat"] / a["f_sat"] < 1.05 and b["f_sat"] / a["f_sat"] > 0.95, "3.8x more samples, the same traction within the load quadrature error (%s)" % [b["f_sat"] / a["f_sat"]])
	# The small-slip force differs by the quadrature error of the load plus the small shift of the effective radius (about 2e-4 m of effective lever arm at 35 rad/s, measured 0.8 % of a 0.8 m/s slip).
	check(absf(b["f_small"] / a["f_small"] - 1.0) < 0.05, "small-slip force across resolutions (%s)" % [b["f_small"] / a["f_small"]])


# ---------------------------------------------------------------- F: lifecycle

func test_f_no_positive_physical_load_means_no_tyre_response() -> void:
	var rig: Dictionary = make_rig({"height": R + 0.05, "forward_speed": 6.0, "lateral_speed": 1.0, "spin": 30.0})
	var wheel: SimRigidBody = rig["wheel"]
	var v0: Vector3 = wheel.linear_velocity
	var w0: Vector3 = wheel.angular_velocity
	for i in 5:
		rig["world"].step()
	var s = rig["support"]
	check(s.longitudinal_force() == 0.0 and s.lateral_force() == 0.0 and s.rolling_resistance_torque() == 0.0, "no force without load")
	check(s.tangential_force_world() == Vector3.ZERO and s.tangential_moment_world() == Vector3.ZERO and s.carried_tire_impulse() == Vector3.ZERO, "no wrench, nothing carried")
	check(s.tangential_response_count() == 0 and not s.has_aggregate_contact() and s.effective_contact_point() == Vector3.ZERO, "no response, no aggregate point")
	check_vec3_near(wheel.linear_velocity, v0, 0.0, "airborne: linear velocity unchanged")
	check_vec3_near(wheel.angular_velocity, w0, 0.0, "airborne: angular velocity unchanged")


func test_f_separation_clears_every_tyre_state_and_recontact_starts_cold() -> void:
	var opts: Dictionary = {"gravity": G, "height": R - 0.01, "forward_speed": 6.0, "spin": 0.9 * 6.0 / R}
	var rig: Dictionary = make_rig(opts)
	var s = rig["support"]
	var wheel: SimRigidBody = rig["wheel"]
	for i in 12:
		rig["world"].step()
	check(absf(s.longitudinal_force()) > 5.0 and s.carried_tire_impulse().length() > 0.01 and s.tangential_response_count() == 1, "fixture: a loaded, slipping patch carries tyre state")
	wheel.position.y += 0.1
	wheel.linear_velocity = Vector3(0.0, 1.0, 0.0) + rig["e_x"] * 6.0
	var spin_before: Vector3 = wheel.angular_velocity
	rig["world"].step()
	check(s.longitudinal_force() == 0.0 and s.lateral_force() == 0.0 and s.rolling_resistance_torque() == 0.0, "separated: no force, no rolling resistance")
	check(s.tangential_force_world() == Vector3.ZERO and s.tangential_moment_world() == Vector3.ZERO, "separated: no tangential wrench")
	check(s.carried_tire_impulse() == Vector3.ZERO and s.carried_row_count() == 0 and s.tangential_response_count() == 0, "separated: warm-start state cleared")
	check_near(wheel.linear_velocity.dot(rig["e_x"]), 6.0, 1.0e-6, "separated: no stale tangential impulse reaches the wheel")
	check_vec3_near(wheel.angular_velocity, spin_before, 1.0e-6, "separated: no stale rolling or tangential moment")
	for i in 4:
		rig["world"].step()
	check(s.carried_tire_impulse() == Vector3.ZERO and s.longitudinal_force() == 0.0, "the separated patch stays clear")
	# Re-contact equals a freshly built patch in the same state.
	var twin: Dictionary = make_rig(opts)
	wheel.position = twin["wheel"].position
	wheel.orientation = twin["wheel"].orientation
	wheel.linear_velocity = twin["wheel"].linear_velocity
	wheel.angular_velocity = twin["wheel"].angular_velocity
	rig["world"].step()
	twin["world"].step()
	check(absf(s.longitudinal_force()) > 1.0, "fixture: the re-contact must produce tyre force")
	check_near(s.longitudinal_force(), twin["support"].longitudinal_force(), 1.0e-9 * absf(s.longitudinal_force()), "re-contact: longitudinal force equals a fresh patch")
	check_near(s.lateral_force(), twin["support"].lateral_force(), 1.0e-6, "re-contact: lateral force equals a fresh patch")
	check_near(s.rolling_resistance_torque(), twin["support"].rolling_resistance_torque(), 1.0e-9 * absf(s.rolling_resistance_torque()), "re-contact: rolling resistance equals a fresh patch")
	check_vec3_near(wheel.linear_velocity, twin["wheel"].linear_velocity, 1.0e-9, "re-contact: velocity equals a fresh patch")
	check_vec3_near(wheel.angular_velocity, twin["wheel"].angular_velocity, 1.0e-9, "re-contact: angular velocity equals a fresh patch")


func test_f_invalid_state_leaves_no_stale_tyre_response() -> void:
	var rig: Dictionary = make_rig({"gravity": G, "height": R - 0.01, "forward_speed": 6.0, "spin": 0.9 * 6.0 / R})
	var s = rig["support"]
	var wheel: SimRigidBody = rig["wheel"]
	for i in 10:
		rig["world"].step()
	check(s.carried_tire_impulse().length() > 0.01, "fixture: carried tyre impulse expected")
	var config = rig["world"].config
	for case in [["NaN position", Vector3(NAN, R, 0.0)], ["INF position", Vector3(INF, R, 0.0)]]:
		var pose: Vector3 = wheel.position
		var v: Vector3 = wheel.linear_velocity
		wheel.position = case[1]
		s.prepare(DT, config)
		s.solve_iteration()
		s.finish()
		check(not s.contact_active and s.last_reason != "", "%s: the patch is inactive with a reason" % case[0])
		check(s.carried_tire_impulse() == Vector3.ZERO and s.longitudinal_force() == 0.0 and s.lateral_force() == 0.0 and s.rolling_resistance_torque() == 0.0, "%s: no tyre state" % case[0])
		check(s.tangential_force_world() == Vector3.ZERO and s.effective_contact_point() == Vector3.ZERO and s.effective_contact_velocity() == Vector3.ZERO, "%s: no stale point or velocity" % case[0])
		check_vec3_near(wheel.linear_velocity, v, 0.0, "%s: the wheel is untouched" % case[0])
		wheel.position = pose
		rig["world"].step()
	# A non-finite tyre parameter is invalid configuration: SimWorld refuses to step it.
	s.tire.lateral_stiffness = NAN
	check(not s.is_valid() and not rig["world"].step(), "an invalid tyre specification makes the patch invalid and the world refuses to step")
	check(rig["world"].last_error.contains("tire"), "the refusal names the tyre (%s)" % rig["world"].last_error)


func test_f_reset_and_aborted_steps_clear_every_stage_c_state() -> void:
	var opts: Dictionary = {"gravity": G, "height": R - 0.01, "forward_speed": 6.0, "spin": 0.9 * 6.0 / R}
	var rig: Dictionary = make_rig(opts)
	var s = rig["support"]
	for i in 10:
		rig["world"].step()
	check(s.carried_tire_impulse().length() > 0.01 and absf(s.longitudinal_force()) > 1.0, "fixture: tyre state expected")
	s.reset_warm_start()
	check(s.carried_tire_impulse() == Vector3.ZERO and s.longitudinal_force() == 0.0 and s.lateral_force() == 0.0 and s.rolling_resistance_torque() == 0.0, "reset_warm_start clears the tyre state")
	check(s.tangential_force_world() == Vector3.ZERO and s.tangential_moment_world() == Vector3.ZERO and s.tangential_response_count() == 0, "and the applied wrench")
	# A real aborted step: a non-finite velocity is detected before the solve.
	var fresh: Dictionary = make_rig(opts)
	for i in 6:
		fresh["world"].step()
	var good_v: Vector3 = fresh["wheel"].linear_velocity
	var good_w: Vector3 = fresh["wheel"].angular_velocity
	var good_p: Vector3 = fresh["wheel"].position
	check(fresh["support"].carried_tire_impulse().length() > 0.01, "fixture: carried impulse before the abort")
	fresh["wheel"].linear_velocity = Vector3(NAN, 0.0, 0.0)
	check(not fresh["world"].step(), "the step with a non-finite velocity is aborted")
	check(fresh["support"].carried_tire_impulse() == Vector3.ZERO and fresh["support"].longitudinal_force() == 0.0, "aborted step: tyre state cleared")
	# Recovery equals a cold twin from the same state.
	var twin: Dictionary = make_rig(opts)
	twin["wheel"].position = good_p
	twin["wheel"].orientation = fresh["wheel"].orientation
	twin["wheel"].linear_velocity = good_v
	twin["wheel"].angular_velocity = good_w
	fresh["wheel"].linear_velocity = good_v
	fresh["world"].clear_numerical_fault()
	fresh["world"].step()
	twin["world"].step()
	check_near(fresh["support"].longitudinal_force(), twin["support"].longitudinal_force(), 1.0e-5 * (absf(twin["support"].longitudinal_force()) + 1.0), "after recovery (float32 state restore: 1.3e-6 measured) the response equals a cold twin")


func test_f_disabling_the_tyre_keeps_the_normal_support_bit_identical_to_stage_b() -> void:
	var opts: Dictionary = {"gravity": G, "height": R - 0.01, "forward_speed": 4.0, "spin": 3.0, "lateral_speed": 0.5}
	var plain: Dictionary = make_rig({"gravity": G, "height": R - 0.01, "forward_speed": 4.0, "spin": 3.0, "lateral_speed": 0.5, "no_tire": true})
	var off: Dictionary = make_rig(opts)
	off["support"].tire_forces_enabled = false
	for i in 25:
		plain["world"].step()
		off["world"].step()
	check(plain["wheel"].position == off["wheel"].position and plain["wheel"].linear_velocity == off["wheel"].linear_velocity and plain["wheel"].angular_velocity == off["wheel"].angular_velocity,
			"a patch with disabled tyre forces behaves bit-identically to the Stage B normal support")
	check(plain["support"].normal_load() == off["support"].normal_load() and off["support"].normal_load() > 50.0, "normal support available and identical")
	check(off["support"].longitudinal_force() == 0.0 and off["support"].lateral_force() == 0.0 and off["support"].rolling_resistance_torque() == 0.0, "no tangential response, no rolling resistance")
	# Re-enabling starts a clean tyre response.
	off["support"].tire_forces_enabled = true
	off["world"].step()
	check(absf(off["support"].longitudinal_force()) + absf(off["support"].lateral_force()) > 1.0, "re-enabled: the tyre acts again")


func test_f_the_tangential_warm_start_is_carried_projected_and_optional() -> void:
	var rig: Dictionary = make_rig({"gravity": G, "height": R - 0.01, "forward_speed": 6.0, "spin": 0.9 * 6.0 / R})
	var s = rig["support"]
	for i in 15:
		rig["world"].step()
	var carried: Vector3 = s.carried_tire_impulse()
	check_vec3_near(carried, rig["e_x"] * s.longitudinal_force() * DT + rig["e_y"] * s.lateral_force() * DT, 1.0e-6, "the carried impulse is the converged tyre impulse")
	check(carried.length() > 0.01, "fixture: non-zero carried impulse")
	var config = rig["world"].config
	var factor: float = config.warm_start_factor
	s.prepare(DT, config)
	check(absf(s.longitudinal_force() * DT - carried.dot(rig["e_x"]) * factor) < 0.05 * carried.length() + 1.0e-6, "warm start re-applies the carried impulse in the tyre axes (factor %s)" % factor)
	config.warm_starting = false
	s.prepare(DT, config)
	check(s.longitudinal_force() == 0.0 and s.lateral_force() == 0.0, "without warm starting the tyre response starts from zero")
	config.warm_starting = true


# ---------------------------------------------------------------- G: configuration

func test_g_configuration_is_validated_and_copied() -> void:
	var ground: SimRigidBody = make_static()
	var wheel: SimRigidBody = make_body(LIGHT, MotorcycleConfig.wheel_inertia(LIGHT, R, 0.75 * R, W), Vector3(0.0, R, 0.0))
	var build: Callable = func(spec, friction: float): return TirePatchNormalSupport.create_with_tire(ground, wheel, geo(), R, Vector3.ZERO, Vector3.UP, K_A, C_A, D_MAX, 9, 48, spec, friction)
	check(build.call(spec_with(), MU_S) != null, "the reference configuration is valid")
	check(build.call(null, MU_S) == null, "null tyre specification rejected")
	check(build.call(spec_with(), 0.0) == null and build.call(spec_with(), -0.1) == null and build.call(spec_with(), NAN) == null and build.call(spec_with(), INF) == null, "invalid surface friction rejected")
	for case in [["rolling radius above the wheel radius", {"rolling_radius": 1.1 * R}], ["rolling radius below half", {"rolling_radius": 0.4 * R}], ["NaN stiffness", {"longitudinal_stiffness": NAN}],
			["zero lateral stiffness", {"lateral_stiffness": 0.0}], ["zero grip", {"grip_longitudinal": 0.0}], ["negative rolling resistance", {"rolling_resistance": -0.1}],
			["zero reference speed", {"low_speed_reference": 0.0}], ["rolling resistance above half the grip", {"rolling_resistance": 0.6}]]:
		check(build.call(spec_with(case[1]), MU_S) == null, "invalid tyre rejected: %s" % case[0])
	check(TirePatchNormalSupport.create_with_tire(null, wheel, geo(), R, Vector3.ZERO, Vector3.UP, K_A, C_A, D_MAX, 9, 48, spec_with(), MU_S) == null, "null ground rejected")
	check(TirePatchNormalSupport.create_with_tire(ground, wheel, geo(), R, Vector3.ZERO, Vector3.UP, 0.0, C_A, D_MAX, 9, 48, spec_with(), MU_S) == null, "the Stage B rules still apply")
	# The specification is COPIED: later edits of the caller's object never reach the patch.
	var original = spec_with()
	var patch = build.call(original, MU_S)
	check(patch.tire != original, "the patch owns its own copy")
	original.longitudinal_stiffness = -5.0
	original.rolling_resistance = 0.3
	check(patch.tire.longitudinal_stiffness == 15.0 and patch.tire.rolling_resistance == 0.015 and patch.is_valid(), "mutating the caller's spec does not mutate the patch")
	# Independent tyres per patch even when built from one source object.
	var source = spec_with()
	var front = build.call(source, MU_S)
	var rear = build.call(source, MU_S)
	front.tire.grip_lateral = 0.5
	check(rear.tire.grip_lateral == 1.0 and front.tire != rear.tire, "two patches never share a tyre object")
	# The Stage B constructor yields a patch without a tyre.
	var stage_b = TirePatchNormalSupport.create(ground, wheel, geo(), R, Vector3.ZERO, Vector3.UP, K_A, C_A, D_MAX, 9, 48)
	check(stage_b != null and stage_b.tire == null and stage_b.tangential_response_count() == 0 and stage_b.is_valid(), "create() keeps the normal-support-only configuration")


func test_g_tyre_helpers_project_into_one_ellipse() -> void:
	var inside: Vector2 = TireModel.into_ellipse(0.3, -0.2, 1.0, 0.8, 0.8)
	check(inside == Vector2(0.3, -0.2), "inside the ellipse: unchanged")
	var outside: Vector2 = TireModel.into_ellipse(3.0, 4.0, 2.0, 1.0, 0.5)
	var u: float = sqrt(pow(outside.x / (1.0 * 2.0), 2.0) + pow(outside.y / (0.5 * 2.0), 2.0))
	check_near(u, 1.0, 1.0e-6, "outside: projected onto the boundary")
	check_near(outside.x / outside.y, 3.0 / 4.0, 1.0e-6, "radial: one common factor, the direction is kept")
	check(TireModel.into_ellipse(1.0, 1.0, 0.0, 1.0, 1.0) == Vector2.ZERO and TireModel.into_ellipse(1.0, 1.0, 1.0, 0.0, 1.0) == Vector2.ZERO, "no load or no limit: zero")
	var free: Vector2 = TireModel.implicit_tangential_impulse(0.5, -0.2, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0, 10.0, 0.8, 0.8, 0.5)
	var phi: float = TireModel.reduction(sqrt(pow(15.0 * 0.5 / 0.5 / 0.8, 2.0) + pow(10.0 * -0.2 / 0.5 / 0.8, 2.0)))
	check_near(free.x, phi * 15.0 * 2.0 * 0.5 / 0.5, 1.0e-5, "G = 0: the explicit law lambda = phi k lambda_n sigma")
	var singular: Vector2 = TireModel.implicit_tangential_impulse(0.5, 0.5, 2.0, 0.1, 0.1, 10.0, 0.0, 0.0, 10.0, 15.0, 10.0, 0.8, 0.8, 0.5)
	check(singular.is_finite(), "a singular 2x2 system stays finite")


# ---------------------------------------------------------------- H: determinism and sustained run

func test_h_runs_are_bit_identical() -> void:
	var results: Array = []
	for r in 2:
		var rig: Dictionary = make_rig({"gravity": G, "height": R - 0.01, "forward_speed": 5.0, "spin": 0.8 * 5.0 / R, "lateral_speed": 0.4, "camber": 0.1})
		for i in 40:
			rig["world"].step()
		results.append([rig["wheel"].position, rig["wheel"].linear_velocity, rig["wheel"].angular_velocity, rig["support"].longitudinal_force(), rig["support"].lateral_force(), rig["support"].rolling_resistance_torque()])
	check(results[0] == results[1], "two identical runs must be bit-identical")


func test_h_a_driven_wheel_on_a_hub_fixture_stays_finite_inside_the_ellipse_and_balances_torque() -> void:
	# TEST-ONLY fixture: a hub on a vertical slide (anchor + prismatic joint) carries the wheel on a hinge about the axle, so the wheel can spin
	# but not translate horizontally or roll. External drive torque on the wheel is a test input, not an engine.
	var pos: Vector3 = Vector3(0.0, R - 0.004, 0.0)
	var w: SimWorld = make_world(Vector3(0.0, -G, 0.0))
	w.config.solver_iterations = 50
	var ground: SimRigidBody = make_static()
	var anchor: SimRigidBody = make_static(pos)
	var hub: SimRigidBody = make_body(20.0, SimMassProperties.sphere_inertia(20.0, 0.1), pos)
	var wheel: SimRigidBody = make_body(LIGHT, MotorcycleConfig.wheel_inertia(LIGHT, R, 0.75 * R, W), pos)
	for b in [ground, anchor, hub, wheel]:
		w.add_body(b)
	var s = TirePatchNormalSupport.create_with_tire(ground, wheel, geo(), R, Vector3.ZERO, Vector3.UP, K_A, C_A, D_MAX, 9, 48, spec_with(), MU_S)
	w.add_constraint(s)
	w.add_constraint(PrismaticJoint.from_world(anchor, hub, pos, Vector3.UP, Vector3.RIGHT))
	w.add_constraint(HingeJoint.from_world(hub, wheel, pos, Vector3.RIGHT, Vector3.UP))
	var torque: float = 60.0
	var worst_util: float = 0.0
	var finite: bool = true
	var min_load: float = INF
	for i in 360:
		wheel.apply_torque(-Vector3.RIGHT * torque * minf(1.0, float(i) / 60.0))
		finite = finite and w.step() and wheel.position.is_finite() and wheel.linear_velocity.is_finite() and wheel.angular_velocity.is_finite()
		worst_util = maxf(worst_util, s.friction_utilization())
		if i > 100:
			min_load = minf(min_load, s.normal_load())
	check(finite, "the driven wheel stays finite for 3 s")
	check(worst_util <= 1.0 + 1.0e-6, "the ellipse is never exceeded (%s)" % worst_util)
	check(min_load > 0.0, "the patch keeps carrying load")
	var lever: float = wheel.position.y - s.effective_contact_point().y
	var balance: float = s.longitudinal_force() * lever + s.rolling_resistance_torque()
	check_near(balance, torque, 0.03 * torque, "steady axle torque balance: F_x * lever + M_rr = drive torque (%s)" % balance)
	check(s.longitudinal_force() > 100.0 and s.longitudinal_force() < MU_S * s.normal_load(), "driving force forward and below the friction limit (%s N)" % s.longitudinal_force())


# ---------------------------------------------------------------- I: tyre failure handling (review corrections of Stage C)
##
## Two properties of the tyre response of one solve, checked through prepare() / solve_iteration() / finish() exactly as ConstraintSolver drives them.
## (1) Every derived rolling-resistance value is validated BEFORE any angular velocity changes; an invalid value is a tyre failure (never a skipped
## sample, never a silent zero, never a NaN angular impulse on the wheel). (2) A tyre failure is terminal for the rest of that solve: no later
## iteration may apply a tangential or rolling-resistance impulse that finish() would then hide by clearing the bookkeeping.
## The oracle is a TWIN of the same rig with tire_forces_enabled = false (bit-identical to Stage B, tested above): while the tyre has failed, the
## wheel must follow the twin, which has only the normal rows. The tyre response of the fixtures is of order 0.1 m/s and 0.1 rad/s per iteration,
## so a tolerance of 1e-5 relative (float32 rounding of the apply-then-withdraw sequence, ~1e-7) cannot hide a resumed response.

## {p: linear momentum (kg m/s), l: angular momentum about the centre of mass (kg m^2/s)} of the wheel.
func wheel_momenta(wheel: SimRigidBody) -> Dictionary:
	return {"p": wheel.linear_velocity / wheel.inverse_mass, "l": wheel.inverse_inertia_world.inverse() * wheel.angular_velocity}


## The wheel's momenta equal those of the tyre-less twin (the tyre contributes nothing) within 1e-5 relative.
func check_wheel_equals_twin(wheel: SimRigidBody, twin: SimRigidBody, message: String) -> void:
	var a: Dictionary = wheel_momenta(wheel)
	var b: Dictionary = wheel_momenta(twin)
	check(wheel.linear_velocity.is_finite() and wheel.angular_velocity.is_finite(), "%s: the wheel state is finite" % message)
	check_vec3_near(a["p"], b["p"], 1.0e-5 * (b["p"].length() + 1.0), "%s: linear momentum equals the tyre-less twin" % message)
	check_vec3_near(a["l"], b["l"], 1.0e-5 * (b["l"].length() + 1.0), "%s: angular momentum equals the tyre-less twin" % message)


## No tyre state of any kind is left (queries of the patch).
func check_no_tire_state(s, message: String) -> void:
	check(s.longitudinal_force() == 0.0 and s.lateral_force() == 0.0 and s.rolling_resistance_torque() == 0.0, "%s: no tyre force or rolling moment" % message)
	check(s.tangential_force_world() == Vector3.ZERO and s.tangential_moment_world() == Vector3.ZERO and s.carried_tire_impulse() == Vector3.ZERO, "%s: no applied or carried wrench" % message)


func test_i_a_non_finite_rolling_resistance_is_a_tyre_failure_and_never_reaches_the_wheel() -> void:
	var base: Dictionary = {"height": R - DELTA, "forward_speed": 6.0}
	# Both cases use a VALID tyre specification and finite state; only the derived rolling-resistance quantities overflow.
	# A: rolling_resistance_speed 1e-306 with a 1e4 rad/s spin: x = v / v_rr = 3e309 overflows to INF and x / sqrt(1 + x x) is NaN.
	# B: grip 1e40 allows c_rr 1e39: the float64 target ~1e39 is finite but exceeds the float32 range of the angular-velocity update (3.4e38).
	var cases: Array = [
		{"name": "NaN target", "spin": 1.0e4, "spec": {"rolling_resistance_speed": 1.0e-306}, "fixed": {"rolling_resistance_speed": 0.2}, "target_finite": false},
		{"name": "target beyond the float32 range", "spin": 30.0, "spec": {"grip_longitudinal": 1.0e40, "rolling_resistance": 1.0e39},
			"fixed": {"grip_longitudinal": 1.0, "rolling_resistance": 0.015}, "target_finite": true},
	]
	for c in cases:
		var name: String = c["name"]
		var opts: Dictionary = base.duplicate()
		opts["spin"] = c["spin"]
		opts["spec_overrides"] = c["spec"]
		var rig: Dictionary = make_rig(opts)
		var twin_opts: Dictionary = opts.duplicate()
		var twin: Dictionary = make_rig(twin_opts)
		twin["support"].tire_forces_enabled = false
		var s = rig["support"]
		var wheel: SimRigidBody = rig["wheel"]
		var config = rig["world"].config
		s.prepare(DT, config)
		twin["support"].prepare(DT, twin["world"].config)
		s.solve_iteration()
		twin["support"].solve_iteration()
		# Fixture honesty: the rolling-resistance law at the aggregate load of this iteration really gives the quantity named in the case.
		var ln: float = s.aggregate_normal_impulse()
		check(ln > 0.0, "%s: fixture: the patch carries load" % name)
		var target: float = -TireModel.rolling_resistance_force(s.tire.rolling_radius * c["spin"], ln, s.tire.rolling_resistance, s.tire.rolling_resistance_speed) * s.tire.rolling_radius
		if c["target_finite"]:
			check(is_finite(target) and absf(target) > 3.4e38, "%s: fixture: finite float64 target %s beyond float32" % [name, target])
		else:
			check(not is_finite(target), "%s: fixture: the target is not finite (%s)" % [name, target])
		# The failure is detected in THIS iteration, before the wheel is touched: the wheel equals the tyre-less twin and nothing is left applied.
		check(s._tire_failed, "%s: the failure is detected in the iteration that derives the value" % name)
		check_wheel_equals_twin(wheel, twin["wheel"], "%s after the failing iteration" % name)
		check_no_tire_state(s, "%s after the failing iteration" % name)
		# The failure is terminal for the rest of the solve (see the next test); the wheel stays finite and equal to the twin.
		for i in 3:
			s.solve_iteration()
			twin["support"].solve_iteration()
			check_wheel_equals_twin(wheel, twin["wheel"], "%s, later iteration %d" % [name, i])
		s.finish()
		twin["support"].finish()
		check(not s.contact_active and s.last_reason.contains("non-finite"), "%s: the patch reports the failure (%s)" % [name, s.last_reason])
		check_no_tire_state(s, "%s after finish" % name)
		check(s.effective_contact_point() == Vector3.ZERO and s.aggregate_normal_load() == 0.0, "%s: no stale aggregate" % name)
		check_wheel_equals_twin(wheel, twin["wheel"], "%s after finish" % name)
		# Recovery: with valid tyre values the next solve (the next prepare) works again and equals a cold solve of a fresh rig from the same state.
		for key in c["fixed"]:
			s.tire.set(key, c["fixed"][key])
		var good: Dictionary = {"height": R - DELTA, "forward_speed": 6.0, "spin": 30.0}
		var fresh: Dictionary = make_rig(good)
		solve_in_place(fresh)
		wheel.linear_velocity = rig["e_x"] * 6.0
		wheel.angular_velocity = -rig["u"] * 30.0
		solve_in_place(rig)
		check(s.contact_active and s.last_reason == "", "%s: a later valid solve is accepted (%s)" % [name, s.last_reason])
		check(absf(s.longitudinal_force()) > 1.0 and s.rolling_resistance_torque() != 0.0, "%s: and the tyre acts again" % name)
		check_near(s.longitudinal_force(), fresh["support"].longitudinal_force(), 1.0e-5 * (absf(fresh["support"].longitudinal_force()) + 1.0), "%s: recovered longitudinal force equals a cold solve" % name)
		check_near(s.rolling_resistance_torque(), fresh["support"].rolling_resistance_torque(), 1.0e-5 * (absf(fresh["support"].rolling_resistance_torque()) + 1.0e-3), "%s: recovered rolling moment equals a cold solve" % name)
		check_vec3_near(wheel.angular_velocity, fresh["wheel"].angular_velocity, 1.0e-5 * (fresh["wheel"].angular_velocity.length() + 1.0), "%s: recovered wheel spin equals a cold solve" % name)


func test_i_b_a_failed_tyre_response_stays_failed_for_the_rest_of_the_solve_and_recovers_in_the_next() -> void:
	# Valid solve; the tyre fails in the FIRST iteration only (tyre rolling radius infinite for that iteration: the slip velocity is not finite, which the
	# tangential response detects and withdraws exactly as before the correction); the cause is gone from the second iteration on. The reviewed defect:
	# iteration 2 then applied a new tangential and rolling-resistance impulse that finish() hid by clearing the bookkeeping.
	for mode in ["finish", "reset"]:
		var opts: Dictionary = {"height": R - DELTA, "forward_speed": 6.0, "spin": 5.0}
		var rig: Dictionary = make_rig(opts)
		var twin: Dictionary = make_rig(opts)
		twin["support"].tire_forces_enabled = false
		var s = rig["support"]
		var wheel: SimRigidBody = rig["wheel"]
		var start: Dictionary = {"v": wheel.linear_velocity, "w": wheel.angular_velocity}
		s.prepare(DT, rig["world"].config)
		twin["support"].prepare(DT, twin["world"].config)
		# Fixture honesty: without the fault the tyre response of iteration 2 is far above the tolerance (0.1 m/s class, measured 0.16 m/s).
		var healthy: Dictionary = make_rig(opts)
		healthy["support"].prepare(DT, healthy["world"].config)
		healthy["support"].solve_iteration()
		healthy["support"].solve_iteration()
		var reference: Dictionary = make_rig(opts)
		reference["support"].tire_forces_enabled = false
		reference["support"].prepare(DT, reference["world"].config)
		reference["support"].solve_iteration()
		reference["support"].solve_iteration()
		check((healthy["wheel"].linear_velocity - reference["wheel"].linear_velocity).length() > 0.05, "fixture: a working tyre changes the wheel velocity by > 0.05 m/s within two iterations")
		var good_radius: float = s.tire.rolling_radius
		s.tire.rolling_radius = INF
		s.solve_iteration()
		s.tire.rolling_radius = good_radius
		twin["support"].solve_iteration()
		check(s._tire_failed, "%s: the first iteration fails" % mode)
		check_wheel_equals_twin(wheel, twin["wheel"], "%s: after the failing iteration (the response is withdrawn)" % mode)
		# The world keeps iterating: no later iteration may apply any tyre wrench, tangential or rolling.
		for i in 6:
			s.solve_iteration()
			twin["support"].solve_iteration()
			check_wheel_equals_twin(wheel, twin["wheel"], "%s: iteration %d after the failure" % [mode, i + 2])
			check_no_tire_state(s, "%s: iteration %d after the failure" % [mode, i + 2])
		if mode == "finish":
			s.finish()
			twin["support"].finish()
			check(not s.contact_active and s.last_reason.contains("non-finite"), "finish: the patch reports the failure (%s)" % s.last_reason)
			# Physical state and bookkeeping agree: the wheel carries only what the normal rows gave it; nothing tyre-related is recorded or hidden.
			check_wheel_equals_twin(wheel, twin["wheel"], "finish: final physical state")
		else:
			# SimWorld's aborted-step path: reset_warm_start() discards the solve; a failure is not carried into the next one.
			s.reset_warm_start()
			check(not s.contact_active, "reset: the patch is inactive")
		check_no_tire_state(s, "%s: final bookkeeping" % mode)
		check(not s._tire_failed, "%s: the failure flag does not outlive the solve" % mode)
		# Recovery: the same patch, valid tyre, the same start state: a new solve applies the tyre again and equals a cold solve of a fresh rig.
		wheel.linear_velocity = start["v"]
		wheel.angular_velocity = start["w"]
		var fresh: Dictionary = make_rig(opts)
		solve_in_place(fresh)
		var tyre_free: Dictionary = make_rig(opts)
		tyre_free["support"].tire_forces_enabled = false
		solve_in_place(tyre_free)
		solve_in_place(rig)
		check(s.contact_active and s.last_reason == "" and not s._tire_failed, "%s: the next valid solve is accepted" % mode)
		check((wheel.linear_velocity - tyre_free["wheel"].linear_velocity).length() > 0.05, "%s: and the tyre changes the wheel velocity again (by %s m/s)" % [mode, (wheel.linear_velocity - tyre_free["wheel"].linear_velocity).length()])
		check_near(s.longitudinal_force(), fresh["support"].longitudinal_force(), 1.0e-5 * (absf(fresh["support"].longitudinal_force()) + 1.0), "%s: recovered longitudinal force equals a cold solve" % mode)
		check_vec3_near(wheel.linear_velocity, fresh["wheel"].linear_velocity, 1.0e-5 * (fresh["wheel"].linear_velocity.length() + 1.0), "%s: recovered wheel velocity equals a cold solve" % mode)


func test_i_c_finish_never_clears_the_record_of_a_response_that_the_wheel_still_carries() -> void:
	# finish() has two paths that discard the tyre bookkeeping: a failed tyre response and a non-finite aggregate load. Neither may leave the impulse
	# in the wheel while its record is gone. One iteration only, so the normal rows of the patch and of the tyre-less twin are still identical and
	# the withdrawn wheel can be compared with the twin exactly (float32 rounding of apply-then-withdraw).
	for path in ["failed flag", "non-finite load"]:
		var opts: Dictionary = {"height": R - DELTA, "forward_speed": 6.0, "spin": 5.0}
		var rig: Dictionary = make_rig(opts)
		var twin: Dictionary = make_rig(opts)
		twin["support"].tire_forces_enabled = false
		var s = rig["support"]
		s.prepare(DT, rig["world"].config)
		twin["support"].prepare(DT, twin["world"].config)
		s.solve_iteration()
		twin["support"].solve_iteration()
		check((rig["wheel"].linear_velocity - twin["wheel"].linear_velocity).length() > 0.05 and s.rolling_resistance_torque() != 0.0, "%s: fixture: the tyre response is applied to the wheel" % path)
		if path == "failed flag":
			s._tire_failed = true
		else:
			prescribe(s, func(_p: Vector3) -> float: return NAN)
		s.finish()
		twin["support"].finish()
		check(not s.contact_active, "%s: the patch is inactive" % path)
		check_no_tire_state(s, "%s: the record is cleared" % path)
		check_wheel_equals_twin(rig["wheel"], twin["wheel"], "%s: and the wheel no longer carries the response" % path)


## The candidates that _apply_tangential_wrench() derives for a wrench (the same formulas, evaluated by the test), to prove that a fixture really makes a
## DERIVED quantity non-finite while the inputs are finite.
func tangential_candidates(s, wheel: SimRigidBody, lambda_x: float, lambda_y: float, point: Vector3) -> Dictionary:
	var linear: Vector3 = s._e_x * lambda_x + s._e_y * lambda_y
	var angular: Vector3 = (point - wheel.position).cross(linear)
	var d_velocity: Vector3 = (linear - s._applied_linear) * wheel.inverse_mass
	var d_spin: Vector3 = wheel.inverse_inertia_world * (angular - s._applied_angular)
	return {"linear": linear, "angular": angular, "d_velocity": d_velocity, "d_spin": d_spin,
			"velocity": wheel.linear_velocity + d_velocity, "spin": wheel.angular_velocity + d_spin}


func test_i_d_a_tangential_wrench_with_a_derived_overflow_is_rejected_atomically() -> void:
	# The reviewed defect: finite scalar impulses (or a finite lever arm) can still make the derived impulse or velocity update overflow float32; the
	# old code then wrote INF/NaN into the wheel and half updated the record. Each case calls the tangential application of the solver with FINITE
	# arguments. "first wrench": nothing was applied before, so the wheel must be bit-identical afterwards. "after a response": an earlier tangential
	# and rolling-resistance response of the same solve must be withdrawn, the later iterations must not re-apply anything, finish() must agree with
	# the wheel, and the next solve must equal a cold solve.
	var cases: Array = [
		{"name": "linear impulse overflows", "lx": 1.0e300, "ly": 0.0, "lever": Vector3.ZERO, "stage": "impulse"},
		{"name": "angular impulse overflows through a finite lever arm", "lx": 1.0e30, "ly": 0.0, "lever": Vector3(1.0e30, 1.0e30, 1.0e30), "stage": "impulse"},
		{"name": "finite impulses, velocity update overflows", "lx": 3.0e8, "ly": 0.0, "lever": Vector3(1.0e30, 0.0, 0.0), "stage": "update"},
		{"name": "non-finite scalar impulse", "lx": NAN, "ly": 0.0, "lever": Vector3.ZERO, "stage": "input"},
	]
	for c in cases:
		for mode in ["first wrench", "after a response"]:
			var label: String = "%s / %s" % [c["name"], mode]
			var opts: Dictionary = {"height": R - DELTA, "forward_speed": 6.0, "spin": 5.0}
			var rig: Dictionary = make_rig(opts)
			var twin: Dictionary = make_rig(opts)
			twin["support"].tire_forces_enabled = false
			var s = rig["support"]
			var wheel: SimRigidBody = rig["wheel"]
			var start: Dictionary = {"v": wheel.linear_velocity, "w": wheel.angular_velocity}
			s.prepare(DT, rig["world"].config)
			twin["support"].prepare(DT, twin["world"].config)
			if mode == "after a response":
				s.solve_iteration()
				twin["support"].solve_iteration()
				check((wheel.linear_velocity - twin["wheel"].linear_velocity).length() > 0.05 and s.rolling_resistance_torque() != 0.0 and s.tangential_force_world() != Vector3.ZERO,
						"%s: fixture: a tangential and a rolling-resistance response are applied before the fault" % label)
			else:
				check_no_tire_state(s, "%s: fixture: nothing applied yet" % label)
			# Fixture honesty: finite and valid before the triggering call, and the derived quantity of the case really is not finite.
			var point: Vector3 = wheel.position + c["lever"]
			var lx: float = c["lx"]
			var ly: float = c["ly"]
			check(not s._tire_failed and s._applied_linear.is_finite() and s._applied_angular.is_finite() and wheel.linear_velocity.is_finite() and wheel.angular_velocity.is_finite() and point.is_finite(),
					"%s: fixture: finite, valid state before the call" % label)
			if c["stage"] == "input":
				check(not is_finite(lx), "%s: fixture: the scalar impulse is not finite" % label)
			else:
				check(is_finite(lx) and is_finite(ly), "%s: fixture: the scalar impulses are finite" % label)
				var cand: Dictionary = tangential_candidates(s, wheel, lx, ly, point)
				if c["stage"] == "impulse":
					check(not (cand["linear"].is_finite() and cand["angular"].is_finite()), "%s: fixture: a derived impulse is not finite" % label)
				else:
					check(cand["linear"].is_finite() and cand["angular"].is_finite(), "%s: fixture: the derived impulses are finite" % label)
					check(not (cand["d_velocity"].is_finite() and cand["d_spin"].is_finite() and cand["velocity"].is_finite() and cand["spin"].is_finite()),
							"%s: fixture: the velocity update or a resulting velocity is not finite" % label)
			var before_v: Vector3 = wheel.linear_velocity
			var before_w: Vector3 = wheel.angular_velocity
			s._apply_tangential_wrench(lx, ly, point)
			check(s._tire_failed, "%s: the failure is detected" % label)
			check(wheel.linear_velocity.is_finite() and wheel.angular_velocity.is_finite(), "%s: the wheel state is finite" % label)
			check_no_tire_state(s, "%s: no tangential or rolling-resistance state is left" % label)
			check(s._applied_linear == Vector3.ZERO and s._applied_angular == Vector3.ZERO and s._rolling_impulse == 0.0, "%s: the record is zero" % label)
			if mode == "first wrench":
				check(wheel.linear_velocity == before_v and wheel.angular_velocity == before_w, "%s: the wheel is bit-identical (nothing of the invalid wrench was applied)" % label)
			check_wheel_equals_twin(wheel, twin["wheel"], "%s: the wheel equals the tyre-less twin" % label)
			# The world keeps iterating: nothing may be applied again.
			for i in 5:
				s.solve_iteration()
				twin["support"].solve_iteration()
				check_wheel_equals_twin(wheel, twin["wheel"], "%s: iteration %d after the failure" % [label, i + 2])
				check_no_tire_state(s, "%s: iteration %d after the failure" % [label, i + 2])
			s.finish()
			twin["support"].finish()
			check(not s.contact_active and s.last_reason.contains("non-finite"), "%s: finish reports the failure (%s)" % [label, s.last_reason])
			check_no_tire_state(s, "%s: bookkeeping after finish" % label)
			check_wheel_equals_twin(wheel, twin["wheel"], "%s: physical state agrees with the bookkeeping after finish" % label)
			# Recovery: a new valid solve from the same start state equals a cold solve of a fresh rig.
			wheel.linear_velocity = start["v"]
			wheel.angular_velocity = start["w"]
			var fresh: Dictionary = make_rig(opts)
			solve_in_place(fresh)
			solve_in_place(rig)
			check(s.contact_active and s.last_reason == "" and not s._tire_failed, "%s: the next valid solve is accepted" % label)
			check(absf(s.longitudinal_force()) > 1.0, "%s: and the tyre acts again" % label)
			check_near(s.longitudinal_force(), fresh["support"].longitudinal_force(), 1.0e-5 * (absf(fresh["support"].longitudinal_force()) + 1.0), "%s: recovered longitudinal force equals a cold solve" % label)
			check_vec3_near(wheel.linear_velocity, fresh["wheel"].linear_velocity, 1.0e-5 * (fresh["wheel"].linear_velocity.length() + 1.0), "%s: recovered wheel velocity equals a cold solve" % label)
			check_vec3_near(wheel.angular_velocity, fresh["wheel"].angular_velocity, 1.0e-5 * (fresh["wheel"].angular_velocity.length() + 1.0), "%s: recovered wheel spin equals a cold solve" % label)


## A finite float32 just below its maximum (3.4028235e38): "a wheel state near the representational limit".
const FLOAT32_NEAR_MAX: float = 3.4028e38


## What _withdraw_tire_response() has to produce, derived by the test from the record (the same inverse, evaluated independently; the angular
## rollback in the reverse order of the application: rolling first, then tangential).
func withdrawal_candidates(s, wheel: SimRigidBody) -> Dictionary:
	var d_roll: Vector3 = wheel.inverse_inertia_world * (s._axle * s._rolling_impulse)
	var d_spin: Vector3 = wheel.inverse_inertia_world * s._applied_angular
	return {"velocity": wheel.linear_velocity - s._applied_linear * wheel.inverse_mass, "spin": (wheel.angular_velocity - d_roll) - d_spin,
			"d_velocity": s._applied_linear * wheel.inverse_mass, "d_spin": d_spin, "d_roll": d_roll}


## A rig whose tyre response is genuinely applied and recorded (iteration 1), then enlarged by a further VALID application so that the record is
## large but finite: "linear" (huge linear impulse), "angular" (huge lever arm, so a huge angular impulse) or "rolling" (a huge rolling-resistance
## impulse, applied to the wheel and the record exactly as _solve_rolling_resistance() does). With `near_limit` the wheel velocity is then moved to
## just below the float32 limit, on the side that makes the rollback overflow (a state that the solver itself never rejected: it is finite).
func withdrawal_fixture(kind: String, near_limit: bool) -> Dictionary:
	var rig: Dictionary = make_rig({"height": R - DELTA, "forward_speed": 6.0, "spin": 5.0})
	var s = rig["support"]
	var wheel: SimRigidBody = rig["wheel"]
	rig["start"] = {"v": wheel.linear_velocity, "w": wheel.angular_velocity}
	s.prepare(DT, rig["world"].config)
	s.solve_iteration()
	match kind:
		"linear":
			s._apply_tangential_wrench(1.0e36, 0.0, wheel.position)
		"angular":
			s._apply_tangential_wrench(1.0e20, 0.0, wheel.position + Vector3(0.0, 1.0e17, 0.0))
		"rolling":
			var extra: float = 1.0e36
			wheel.angular_velocity += wheel.inverse_inertia_world * (s._axle * extra)
			s._rolling_impulse += extra
	var cand: Dictionary = withdrawal_candidates(s, wheel)
	if near_limit:
		match kind:
			"linear":
				wheel.linear_velocity = -cand["d_velocity"].sign() * FLOAT32_NEAR_MAX
			"angular":
				wheel.angular_velocity = -cand["d_spin"].sign() * FLOAT32_NEAR_MAX
			"rolling":
				wheel.angular_velocity = -cand["d_roll"].sign() * FLOAT32_NEAR_MAX
	rig["candidates"] = withdrawal_candidates(s, wheel)
	return rig


func record_of(s) -> Array:
	return [s._applied_linear, s._applied_angular, s._rolling_impulse, s._tire_lx, s._tire_ly]


func test_i_d_b_an_unrepresentable_withdrawal_is_a_terminal_numerical_fault_that_keeps_the_record() -> void:
	# The reviewed defect: a failure after an applied response withdrew it by subtracting the recorded impulses WITHOUT validating the result. A finite
	# wheel state near the float32 limit then became INF/NaN (or was half updated) and the record was cleared as if the rollback had succeeded. Now the
	# rollback is validated and atomic; when it is not representable nothing is written or cleared, the failure is terminal and reported as a numerical
	# fault (last_reason, validate() and with it SimWorld's preflight) until reset_warm_start().
	for kind in ["linear", "angular", "rolling"]:
		# Control: the same large record with the wheel NOT near the limit is withdrawn completely, with no fault.
		var control: Dictionary = withdrawal_fixture(kind, false)
		var cs = control["support"]
		check(control["candidates"]["velocity"].is_finite() and control["candidates"]["spin"].is_finite(), "%s control: fixture: the rollback is representable" % kind)
		cs._apply_tangential_wrench(NAN, 0.0, control["wheel"].position)
		check(cs._tire_failed and cs._withdrawal_fault == "" and cs.validate().is_empty(), "%s control: a representable rollback is an ordinary failure, not a fault" % kind)
		check(cs._applied_linear == Vector3.ZERO and cs._applied_angular == Vector3.ZERO and cs._rolling_impulse == 0.0, "%s control: the record is cleared because the response was withdrawn" % kind)
		check(control["wheel"].linear_velocity.is_finite() and control["wheel"].angular_velocity.is_finite(), "%s control: the wheel is finite" % kind)

		for trigger in ["failing wrench", "finish with failed flag"]:
			var label: String = "%s / %s" % [kind, trigger]
			var rig: Dictionary = withdrawal_fixture(kind, true)
			var s = rig["support"]
			var wheel: SimRigidBody = rig["wheel"]
			var cand: Dictionary = rig["candidates"]
			# Fixture honesty: a valid applied response, a finite wheel state, and an inverse that is NOT representable.
			check(not s._tire_failed and s._withdrawal_fault == "" and s._applied_linear.is_finite() and s._applied_angular.is_finite() and is_finite(s._rolling_impulse), "%s: fixture: valid, finite record before the failure" % label)
			check(s._applied_linear != Vector3.ZERO or s._applied_angular != Vector3.ZERO or s._rolling_impulse != 0.0, "%s: fixture: a response is recorded" % label)
			check(wheel.linear_velocity.is_finite() and wheel.angular_velocity.is_finite(), "%s: fixture: the wheel state is finite (near the float32 limit)" % label)
			check(not (cand["velocity"].is_finite() and cand["spin"].is_finite()), "%s: fixture: the rollback velocity is not finite" % label)
			var before_v: Vector3 = wheel.linear_velocity
			var before_w: Vector3 = wheel.angular_velocity
			var before_record: Array = record_of(s)
			if trigger == "failing wrench":
				s._apply_tangential_wrench(NAN, 0.0, wheel.position)
			else:
				s._tire_failed = true
				s.finish()
			check(s._tire_failed and s._withdrawal_fault != "", "%s: the failure is a terminal numerical fault" % label)
			check(wheel.linear_velocity == before_v and wheel.angular_velocity == before_w and wheel.linear_velocity.is_finite() and wheel.angular_velocity.is_finite(),
					"%s: the wheel is finite and bit-identical (no partial or clamped rollback)" % label)
			check(record_of(s) == before_record and (s._applied_linear != Vector3.ZERO or s._applied_angular != Vector3.ZERO or s._rolling_impulse != 0.0),
					"%s: the record is not cleared as if the rollback had succeeded" % label)
			check(s.validate().size() == 1 and String(s.validate()[0]).contains("could not be withdrawn"), "%s: validate() reports the fault" % label)
			if trigger == "failing wrench":
				# Later iterations apply nothing and withdraw nothing again: the record stays exactly as it was. (The NORMAL rows of the patch keep solving, as
				# after any tyre failure, so the wheel velocity itself may move by their impulses; it must stay finite.)
				for i in 6:
					s.solve_iteration()
					check(wheel.linear_velocity.is_finite() and wheel.angular_velocity.is_finite(), "%s: iteration %d after the fault: the wheel stays finite" % [label, i + 2])
					check(record_of(s) == before_record and s._tire_failed, "%s: iteration %d after the fault leaves the record untouched and the failure terminal" % [label, i + 2])
				before_v = wheel.linear_velocity
				before_w = wheel.angular_velocity
				s.finish()
			check(not s.contact_active and s.last_reason.contains("could not be withdrawn"), "%s: finish() reports the numerical fault (%s)" % [label, s.last_reason])
			check(wheel.linear_velocity == before_v and wheel.angular_velocity == before_w, "%s: finish() leaves the wheel untouched" % label)
			check(record_of(s) == before_record and s._tire_failed and s.carried_tire_impulse() == Vector3.ZERO,
					"%s: finish() holds the record, keeps the failure terminal and carries no warm start" % label)
			# The world refuses to step while the constraint reports the fault (the existing preflight channel).
			check(not rig["world"].preflight() and rig["world"].last_error.contains("could not be withdrawn"), "%s: SimWorld refuses to step (%s)" % [label, rig["world"].last_error])
			# A solve that is started anyway applies no tyre response and keeps the record.
			s.prepare(DT, rig["world"].config)
			check(s._tire_failed and record_of(s) == before_record, "%s: a later prepare() keeps the failure terminal and the record" % label)
			# Release: reset_warm_start() (aborted step / repaired wheel) clears everything, and a valid solve then equals a cold solve.
			s.reset_warm_start()
			check(s._withdrawal_fault == "" and not s._tire_failed and s.validate().is_empty(), "%s: reset_warm_start() releases the fault" % label)
			check_no_tire_state(s, "%s: after the release" % label)
			wheel.linear_velocity = rig["start"]["v"]
			wheel.angular_velocity = rig["start"]["w"]
			var fresh: Dictionary = make_rig({"height": R - DELTA, "forward_speed": 6.0, "spin": 5.0})
			solve_in_place(fresh)
			solve_in_place(rig)
			check(s.contact_active and s.last_reason == "" and not s._tire_failed, "%s: the next valid solve is accepted" % label)
			check_near(s.longitudinal_force(), fresh["support"].longitudinal_force(), 1.0e-5 * (absf(fresh["support"].longitudinal_force()) + 1.0), "%s: recovered longitudinal force equals a cold solve" % label)


func test_i_d_c_angular_withdrawal_reverses_the_application_order_so_a_rounded_rolling_term_leaves_no_residual() -> void:
	# float32 sums are not associative. The tyre applies the tangential angular impulse first and the rolling-resistance impulse second. When the
	# tangential contribution is large, (w + d_tan) + d_roll loses a small d_roll (the spacing of float32 there exceeds 2 d_roll), although the
	# rolling impulse is still recorded. Withdrawing in the SAME order, (w - d_tan) - d_roll, then leaves a residual of d_roll; the reverse order,
	# (w - d_roll) - d_tan, loses d_roll again at the same place and returns the pre-response spin. Everything here is applied through the real
	# paths (_apply_tangential_wrench() and _solve_rolling_resistance()) after a real prepare(), in the order of a solver iteration.
	var spin0: float = pow(2.0, 30.0)  # a multiple of the float32 spacing (2^27) around the tangential spin change below, so the sum is exact
	var rig: Dictionary = make_rig({"height": R - DELTA, "forward_speed": 6.0, "spin": spin0})
	var s = rig["support"]
	var wheel: SimRigidBody = rig["wheel"]
	var world: SimWorld = rig["world"]
	var w0: Vector3 = wheel.angular_velocity
	var v0: Vector3 = wheel.linear_velocity
	s.prepare(DT, world.config)
	check(w0.is_finite() and v0.is_finite() and s._axle == Vector3.RIGHT and is_equal_approx(s._spin, spin0), "fixture: finite wheel state, axle +X, frozen spin = the start spin")
	check(not s._tire_failed and s._withdrawal_fault == "" and record_of(s) == [Vector3.ZERO, Vector3.ZERO, 0.0, 0.0, 0.0], "fixture: nothing recorded yet")
	# 1. Tangential application with a long lever arm: a modest longitudinal impulse (linear velocity change of 67 m/s) whose moment about the axle
	# changes the spin by 1.5 * 2^50 rad/s (valid: finite, no overflow; it is the lever that makes it large).
	var inv_xx: float = wheel.inverse_inertia_world.x.x
	var lx: float = 1.0e3
	var lever: float = (1.5 * pow(2.0, 50.0) / inv_xx) / lx
	s._apply_tangential_wrench(lx, 0.0, wheel.position + Vector3(0.0, lever, 0.0))
	check(not s._tire_failed and s._applied_angular.x != 0.0 and s._applied_linear != Vector3.ZERO, "the tangential wrench is applied and recorded")
	var w1: Vector3 = wheel.angular_velocity
	var d_spin: Vector3 = wheel.inverse_inertia_world * s._applied_angular
	check(is_equal_approx(w1.x, w0.x + d_spin.x) and absf(w1.x) > 1.0e15, "the tangential angular contribution is applied (about 1.7e15 rad/s)")
	# 2. Rolling-resistance application (saturated at the frozen spin) with a huge aggregate normal impulse, so that its contribution is large in
	# absolute terms (about 4e7 rad/s) but still below half the float32 spacing (6.7e7) at the tangential spin.
	s._solve_rolling_resistance(1.0e10)
	var w2: Vector3 = wheel.angular_velocity
	var d_roll: Vector3 = wheel.inverse_inertia_world * (s._axle * s._rolling_impulse)
	check(not s._tire_failed and s._rolling_impulse != 0.0 and d_roll.x != 0.0, "the rolling impulse is applied and recorded")
	# Fixture honesty: the rounding-sensitive relationship, computed independently of the implementation.
	var spacing: float = pow(2.0, floor(log(absf(w1.x)) / log(2.0)) - 23.0)
	check(absf(d_roll.x) > 1.0e7 and absf(d_roll.x) < 0.5 * spacing, "fixture: the rolling contribution (%s) is large but below half the float32 spacing (%s) of the tangential spin" % [d_roll.x, spacing])
	check(w2 == w1, "fixture: the forward accumulation order lost the whole rolling contribution (w after rolling == w after tangential)")
	var old_order: Vector3 = (w2 - d_spin) - d_roll
	var new_order: Vector3 = (w2 - d_roll) - d_spin
	# Tolerances of the VECTOR comparisons below (TESTING.md tolerance policy: float32 Vector3 quantities use >= 1e-6 relative to the magnitude of the
	# compared vector). Spin: 1e-6 * |w0| = 1.07e3 rad/s for the pre-response spin of 2^30 rad/s. The fixture makes the spin sum exact (w0 is a multiple of
	# the float32 spacing at the tangential spin), so the reverse order is expected to return w0 with no error at all; the tolerance covers the float32
	# representation of the result, not a rounding of the sum. The old order leaves d_roll = 4.1e7 rad/s, about 38000 times this tolerance, which is what
	# separates the two orders (the margin is asserted below, as a factor of 1000). Linear velocity: 1e-6 * |v0| = 6e-6 m/s for 6 m/s.
	# The scalar facts that establish the rounding mechanism above (the spacing, the half spacing, the lost term) stay exact float64 scalar checks.
	var tol_w: float = 1.0e-6 * w0.length()
	var tol_v: float = 1.0e-6 * v0.length()
	check((old_order - w0).length() > 1000.0 * tol_w, "fixture: the old (forward) order leaves a residual of %s rad/s, over 1000 times the tolerance %s" % [(old_order - w0).length(), tol_w])
	check((new_order - w0).length() <= tol_w, "fixture: the reverse order returns the pre-response spin within the tolerance")
	var w_before: Vector3 = wheel.angular_velocity
	var applied_angular: Vector3 = s._applied_angular
	# Ordinary tyre failure after both responses were applied: the withdrawal runs.
	s._apply_tangential_wrench(NAN, 0.0, wheel.position)
	check(s._tire_failed and s._withdrawal_fault == "" and s.validate().is_empty(), "the withdrawal succeeds: an ordinary terminal failure, no numerical fault")
	check(wheel.angular_velocity.is_finite() and wheel.linear_velocity.is_finite(), "the wheel state is finite")
	check_vec3_near(wheel.angular_velocity, w0, tol_w, "the spin returns to the pre-response value (all components, 1e-6 relative)")
	check((wheel.angular_velocity - old_order).length() > 1000.0 * tol_w, "and differs from what the forward-order withdrawal would have left by over 1000 times the tolerance")
	check_vec3_near(wheel.linear_velocity, v0, tol_v, "the linear velocity returns to the pre-response value (1e-6 relative)")
	check(record_of(s) == [Vector3.ZERO, Vector3.ZERO, 0.0, 0.0, 0.0], "the response bookkeeping is cleared after the successful withdrawal")
	check(w_before != wheel.angular_velocity and applied_angular != Vector3.ZERO, "fixture: the record was real and the withdrawal changed the wheel")
	# Later iterations apply no tyre response; finish() ends the failed contact without stale bookkeeping.
	for i in 4:
		s.solve_iteration()
		check(record_of(s) == [Vector3.ZERO, Vector3.ZERO, 0.0, 0.0, 0.0] and s._tire_failed and s._withdrawal_fault == "", "iteration %d after the failure applies no tyre response and records nothing" % (i + 2))
	s.finish()
	check(not s.contact_active and s.last_reason != "" and not s.last_reason.contains("could not be withdrawn"), "finish() ends the failed contact with the ordinary failure reason (%s)" % s.last_reason)
	check(s._withdrawal_fault == "" and s.validate().is_empty(), "finish(): no withdrawal fault")
	check_no_tire_state(s, "after finish()")


# ---------------------------------------------------------------- J: low-speed behaviour of the patch (reference-speed crossover, rest, creep)
##
## Low-speed behaviour of the DISTRIBUTED patch around the reference speed v_ref of the tyre (0.5 m/s for the prototype tyre). Every test drives the
## real patch (create_with_tire(); prepare() / solve_iteration() / finish(); SimWorld.step() for the rest test) on the HEAVY or LIGHT rig of this
## file. Neither the one-point TireGroundContact nor a TireModel helper supplies an expectation: the expectations are written out here from the
## definitions (the oracles below):
##   slip velocity  s = -(v + w x r) . e at the load-weighted effective point (r = p_eff - x; the fixtures use R_eff = R, so the rolling-radius term
##                  vanishes), with e = e_x, e_y the tyre axes of the rig;
##   denominator    D = sqrt(v_hub^2 + v_ref^2), v_hub = v . e_x of the wheel CENTRE (neither the contact-point velocity nor the spin);
##   brush law      rho = |(k_x s_x / (mu_x D), k_y s_y / (mu_y D))|, F / N = phi k s / D, phi = g(rho) / rho, g = rho - rho^2/3 + rho^3/27 for rho < 3
##                  and 1 from 3 on (g' = (1 - rho/3)^2 >= 0: g increases and reaches 1 at rho = 3);
##   creep bound    3 mu v_ref / k (PHYSICS_CONVENTIONS.md, "Approximations"): for sliding along ONE axis at v_hub = 0 (D = v_ref) rho = k s / (mu v_ref),
##                  so g reaches 1 (the friction limit) at s = 3 mu v_ref / k, and every tangential load below mu N is carried at a sliding speed below it.
##                  For v_hub != 0 the bound is 3 mu D / k; for combined slip rho combines both axes. The tests below use exactly the conditions
##                  in which the bound applies (zero hub speed, one axis at a time).
## Tolerances: the heavy-rig value of this file, 2e-4 relative to the expected force (the response changes the state of the 1e7 kg wheel by < 1e-6
## relative; measured agreement with the law is 1e-7 to 5e-5). Quantities that are zero in exact arithmetic (rest) are bounded by 1e-6 of the load
## (the TESTING.md minimum relative tolerance of float32 quantities) and by the sliding speed that this force corresponds to in the linear law.

## g(rho) written out (not TireModel.saturation).
func brush_g(rho: float) -> float:
	if rho >= 3.0:
		return 1.0
	return rho - rho * rho / 3.0 + rho * rho * rho / 27.0


## The rho in [0, 3] with g(rho) = f for 0 <= f <= 1, by bisection (g is strictly increasing on [0, 3]).
func brush_g_inverse(f: float) -> float:
	var lo: float = 0.0
	var hi: float = 3.0
	for i in 100:
		var mid: float = 0.5 * (lo + hi)
		if brush_g(mid) < f:
			lo = mid
		else:
			hi = mid
	return 0.5 * (lo + hi)


## Force per unit load from the brush law by hand: {fx, fy, rho} for the slip velocities s_x, s_y (m/s) and the denominator d (m/s).
func hand_law(spec, s_x: float, s_y: float, d: float) -> Dictionary:
	var mu_x: float = spec.grip_longitudinal * MU_S
	var mu_y: float = spec.grip_lateral * MU_S
	var ex: float = spec.longitudinal_stiffness * s_x / (d * mu_x)
	var ey: float = spec.lateral_stiffness * s_y / (d * mu_y)
	var rho: float = sqrt(ex * ex + ey * ey)
	var phi: float = brush_g(rho) / rho if rho > 1.0e-12 else 1.0
	return {"fx": phi * spec.longitudinal_stiffness * s_x / d, "fy": phi * spec.lateral_stiffness * s_y / d, "rho": rho}


## First-principles state of a SOLVED rig (after finish()): {s_x, s_y (m/s), hub (m/s), load (N), point}, v_c = v + w x (p_eff - x) with the
## load-weighted effective point of sample_report().
func slip_state(rig: Dictionary) -> Dictionary:
	var wheel: SimRigidBody = rig["wheel"]
	var o: Dictionary = oracle_weighted(rig["support"].sample_report())
	var v_c: Vector3 = wheel.linear_velocity + wheel.angular_velocity.cross(o["point"] - wheel.position)
	return {"s_x": -v_c.dot(rig["e_x"]), "s_y": -v_c.dot(rig["e_y"]), "hub": wheel.linear_velocity.dot(rig["e_x"]), "load": o["load"], "point": o["point"]}


## Height of the wheel centre above the effective point of the resting heavy rig (m): only used to choose the spin that gives an intended tread speed
## (s_x = -v + spin * lever); every expectation uses the slip measured from the state after the solve.
func resting_lever() -> float:
	var calm: Dictionary = heavy_rig()
	solve_in_place(calm)
	return calm["wheel"].position.y - oracle_weighted(calm["support"].sample_report())["point"].y


## One solved heavy rig with the hub speed `hub` along e_x, a lateral speed of 0.03 m/s (s_y = -0.03 m/s) and the spin that makes the longitudinal
## tread slip about +0.03 m/s (driving). `mirrored` reverses the hub speed and the spin (s_x changes sign, s_y does not).
func crossover_case(hub: float, mirrored: bool, lever: float) -> Dictionary:
	var sgn: float = -1.0 if mirrored else 1.0
	var rig: Dictionary = heavy_rig(DELTA, {"forward_speed": sgn * hub, "lateral_speed": 0.03, "spin": sgn * (hub + 0.03) / lever})
	solve_in_place(rig)
	var s = rig["support"]
	var st: Dictionary = slip_state(rig)
	return {"st": st, "fx": s.longitudinal_force() / st["load"], "fy": s.lateral_force() / st["load"], "slip": s.current_slip()}


func test_j_a_the_patch_crosses_the_reference_speed_with_the_smooth_denominator_and_without_a_jump() -> void:
	var spec = spec_with()
	var v_ref: float = spec.low_speed_reference
	var mu_x: float = spec.grip_longitudinal * MU_S
	var mu_y: float = spec.grip_lateral * MU_S
	check_near(spec.rolling_radius, R, 0.0, "fixture: R_eff = R (no rolling-radius term in the slip velocity)")
	var lever: float = resting_lever()
	# Hub speeds in units of v_ref: far below, below, a dense run through the crossover v_hub = v_ref, and above.
	var multiples: Array = [0.0, 0.25, 0.5, 0.8, 0.9, 0.92, 0.94, 0.96, 0.98, 0.99, 1.0, 1.01, 1.02, 1.04, 1.06, 1.08, 1.1, 1.25, 2.0, 4.0, 10.0]
	var previous: Dictionary = {}
	for m in multiples:
		var hub: float = m * v_ref
		var up: Dictionary = crossover_case(hub, false, lever)
		var down: Dictionary = crossover_case(hub, true, lever)
		var st: Dictionary = up["st"]
		var d: float = sqrt(st["hub"] * st["hub"] + v_ref * v_ref)
		var law: Dictionary = hand_law(spec, st["s_x"], st["s_y"], d)
		var label: String = "hub speed %s v_ref" % m
		check(absf(st["hub"] - hub) < 1.0e-5 and st["s_x"] > 0.0 and st["s_y"] < 0.0 and law["rho"] < 3.0, "fixture: %s, driving with a leftward contact slide, not saturated (rho %s)" % [label, law["rho"]])
		# The force of the patch against the law written out, with the smooth denominator.
		check_near(up["fx"], law["fx"], 2.0e-4 * absf(law["fx"]) + 1.0e-9, "%s: longitudinal force per load" % label)
		check_near(up["fy"], law["fy"], 2.0e-4 * absf(law["fy"]) + 1.0e-9, "%s: lateral force per load" % label)
		# The patch's own slip evaluation (the path the solve uses) reports the same denominator and slips.
		var slip: Dictionary = up["slip"]
		check(slip["valid"], "%s: valid slip" % label)
		check_near(slip["reference_speed"], d, 2.0e-6 * d, "%s: slip denominator sqrt(v_hub^2 + v_ref^2)" % label)
		check_near(slip["slip_x"], st["s_x"] / d, 2.0e-5, "%s: longitudinal slip s_x / D" % label)
		check_near(slip["slip_y"], st["s_y"] / d, 2.0e-5, "%s: lateral slip s_y / D" % label)
		# Mirror symmetry: D is even in the hub speed, so the lateral force is even and the longitudinal force odd (same load, same slip velocities).
		var mirror: Dictionary = down["st"]
		check_near(mirror["hub"], -st["hub"], 1.0e-6 * (absf(st["hub"]) + 1.0), "%s: mirrored hub speed" % label)
		check_near(mirror["load"], st["load"], 1.0e-4 * st["load"], "%s: mirrored load" % label)
		check_near(down["fy"], up["fy"], 2.0e-4 * absf(up["fy"]), "%s: the lateral force is even in the hub speed" % label)
		check_near(down["fx"], -up["fx"], 2.0e-4 * absf(up["fx"]) + 1.0e-9, "%s: the longitudinal force is odd in the hub speed" % label)
		# No jump: D(v) is smooth and rho = c / D, so |d(rho)/dv| = rho v / D^2 <= rho / (2 v_ref) (maximum at v = v_ref), and |d g / d rho| <= 1. The
		# ellipse-normalised force u = (F_x / mu_x, F_y / mu_y) has |u| = g(rho) and a constant direction at fixed slip velocities, so between two hub
		# speeds |u_2 - u_1| <= rho_max dv / (2 v_ref) (rho is largest at the smaller |v|), plus the agreement with the law (2e-4 per point) twice.
		var u: Vector2 = Vector2(up["fx"] / mu_x, up["fy"] / mu_y)
		if not previous.is_empty():
			var dv: float = (m - previous["m"]) * v_ref
			var bound: float = maxf(law["rho"], previous["rho"]) * dv / (2.0 * v_ref) + 4.0e-4 * maxf(u.length(), previous["u"].length())
			check((u - previous["u"]).length() <= bound, "%s: no jump from %s v_ref (change %s, bound %s)" % [label, previous["m"], (u - previous["u"]).length(), bound])
		previous = {"m": m, "rho": law["rho"], "u": u}
		if m == 1.0:
			# The fixture can tell a wrong denominator: at the crossover each alternative changes the force by far more than the tolerance.
			var alternatives: Dictionary = {"D = v_ref (no hub dependence, or max(|v|, v_ref))": v_ref, "D = |v| + v_ref": absf(st["hub"]) + v_ref,
					"sqrt(v^2 + (v_ref / 2)^2)": sqrt(st["hub"] * st["hub"] + 0.25 * v_ref * v_ref)}
			for name in alternatives:
				var alt: Dictionary = hand_law(spec, st["s_x"], st["s_y"], alternatives[name])
				check(absf(alt["fx"] - law["fx"]) > 0.05 * absf(law["fx"]) and absf(alt["fy"] - law["fy"]) > 0.05 * absf(law["fy"]), "fixture: the wrong denominator %s is %s / %s away" % [name, absf(alt["fx"] - law["fx"]) / absf(law["fx"]), absf(alt["fy"] - law["fy"]) / absf(law["fy"])])


func test_j_b_the_reference_speed_comes_from_the_hub_velocity_and_is_frozen_for_the_whole_solve() -> void:
	var spec = spec_with()
	var v_ref: float = spec.low_speed_reference
	# (1) SELECTION: D uses the component of the wheel CENTRE's velocity along e_x only. The lateral speed, the spin, a yaw rate and the tyre's yaw and
	# camber change the contact-point velocity or the world-frame velocity, but not v . e_x.
	var hub: float = 0.3
	var expected: float = sqrt(hub * hub + v_ref * v_ref)
	var variants: Array = [{}, {"lateral_speed": 2.0}, {"spin": 12.0}, {"spin": -12.0}, {"lateral_speed": -1.5, "spin": -3.0, "omega": Vector3(0.0, 2.0, 0.0), "yaw": 0.5, "camber": 0.2}]
	for variant in variants:
		var opts: Dictionary = variant.duplicate()
		opts["forward_speed"] = hub
		var rig: Dictionary = heavy_rig(DELTA, opts)
		var s = rig["support"]
		s.prepare(DT, rig["world"].config)
		prescribe(s, func(_p: Vector3) -> float: return 0.1)
		check_near(s._reference_speed, expected, 1.0e-6 * expected, "frozen reference speed for %s" % [variant])
		var slip: Dictionary = s.current_slip()
		check(slip["valid"], "valid slip for %s" % [variant])
		check_near(slip["reference_speed"], expected, 2.0e-6 * expected, "the patch's slip denominator for %s" % [variant])
	# (2) FROZEN: with the normal impulses held (so that only the tangential response iterates, as in the implicit test above) the response removes
	# part of the hub speed within the solve. The converged impulse must satisfy lambda = phi k ln s_end / D with D of the START hub speed, not with the
	# D of the changed hub speed: the two differ by more than 2 % here. The second case starts above v_ref and ends below it.
	var lever: float = resting_lever()
	for cfg in [{"v0": 0.4, "s_x0": -0.4, "lateral": 0.0, "ln": 3.0, "crosses": false}, {"v0": 0.55, "s_x0": -0.35, "lateral": 0.1, "ln": 2.5, "crosses": true}]:
		var label: String = "start hub speed %s m/s" % cfg["v0"]
		var rig: Dictionary = make_rig({"height": R - DELTA, "forward_speed": cfg["v0"], "lateral_speed": cfg["lateral"], "spin": (cfg["v0"] + cfg["s_x0"]) / lever})
		var s = rig["support"]
		var wheel: SimRigidBody = rig["wheel"]
		s.prepare(DT, rig["world"].config)
		var count: int = s.rows.size()
		prescribe(s, func(_p: Vector3) -> float: return cfg["ln"] / float(count))
		var hub0: float = wheel.linear_velocity.dot(rig["e_x"])
		var d0: float = sqrt(hub0 * hub0 + v_ref * v_ref)
		var frozen: float = s._reference_speed
		check_near(frozen, d0, 1.0e-6 * d0, "%s: the reference speed of the solve is sqrt(v_hub0^2 + v_ref^2)" % label)
		var ln: float = s.aggregate_normal_impulse()
		var constant: bool = true
		for i in 60:
			s._solve_tire_iteration()
			constant = constant and s._reference_speed == frozen
		check(constant, "%s: the reference speed is the same value in every iteration of the solve" % label)
		var hub1: float = wheel.linear_velocity.dot(rig["e_x"])
		var d1: float = sqrt(hub1 * hub1 + v_ref * v_ref)
		check(hub0 - hub1 > 0.1 * hub0 and (hub1 < v_ref and hub0 > v_ref if cfg["crosses"] else hub1 < hub0 and hub0 < v_ref), "fixture: %s: the response changes the hub speed from %s to %s m/s (v_ref %s)" % [label, hub0, hub1, v_ref])
		check_near(s.current_slip()["reference_speed"], d1, 2.0e-6 * d1, "%s: the live slip follows the CURRENT hub speed (the frozen D does not)" % label)
		var p_eff: Vector3 = s.effective_contact_point()
		var v_c: Vector3 = wheel.linear_velocity + wheel.angular_velocity.cross(p_eff - wheel.position)
		var s_x: float = -v_c.dot(rig["e_x"])
		var s_y: float = -v_c.dot(rig["e_y"])
		var law0: Dictionary = hand_law(spec, s_x, s_y, d0)
		var law1: Dictionary = hand_law(spec, s_x, s_y, d1)
		var lam_x: float = s.longitudinal_force() * DT
		var lam_y: float = s.lateral_force() * DT
		check(law0["rho"] < 3.0 and law1["rho"] > law0["rho"], "fixture: %s: unsaturated at the end of the solve with the frozen D (rho %s, with the live D %s)" % [label, law0["rho"], law1["rho"]])
		# 2e-3: the same tolerance as the implicit test above (phi is lagged by one iteration; measured agreement is below 1e-6 here).
		check_near(lam_x, ln * law0["fx"], 2.0e-3 * absf(ln * law0["fx"]) + 1.0e-6, "%s: lambda_x uses the frozen D" % label)
		check_near(lam_y, ln * law0["fy"], 2.0e-3 * absf(ln * law0["fy"]) + 1.0e-6, "%s: lambda_y uses the frozen D" % label)
		check(absf(ln * law1["fx"] - lam_x) > 0.02 * absf(lam_x), "fixture: %s: a live D would change lambda_x by %s of its value" % [label, absf(ln * law1["fx"] - lam_x) / absf(lam_x)])
		# (3) It is a property of the SOLVE: the next prepare() takes the hub speed of that moment.
		s.finish()
		s.prepare(DT, rig["world"].config)
		check_near(s._reference_speed, d1, 1.0e-6 * d1, "%s: the next solve freezes the new hub speed" % label)
		check(absf(s._reference_speed - frozen) > 0.02 * frozen, "fixture: %s: the new reference speed differs from the old one" % label)


func test_j_c_a_loaded_wheel_at_rest_develops_no_tangential_response_and_accumulates_nothing() -> void:
	var spec = spec_with()
	var k_max: float = maxf(spec.longitudinal_stiffness, spec.lateral_stiffness)
	# A force of 1e-6 of the load is the tolerance; in the linear law F / N = k s / v_ref it corresponds to the sliding speed v_tol.
	var rel_tol: float = 1.0e-6
	var v_tol: float = rel_tol * spec.low_speed_reference / k_max
	# (1) A loaded wheel exactly at rest (HEAVY rig, no gravity, zero velocities, crown compressed by DELTA): after 1, 20 and 200 iterations of the
	# same solve the tangential impulse, the rolling-resistance impulse and the horizontal wheel velocity are still zero (not growing with the iterations).
	var rig: Dictionary = heavy_rig()
	var s = rig["support"]
	var wheel: SimRigidBody = rig["wheel"]
	check(wheel.linear_velocity == Vector3.ZERO and wheel.angular_velocity == Vector3.ZERO, "fixture: the wheel starts exactly at rest")
	s.prepare(DT, rig["world"].config)
	var done: int = 0
	for target in [1, 20, 200]:
		while done < target:
			s.solve_iteration()
			done += 1
		var ln: float = s.aggregate_normal_impulse()
		check(ln > 0.0, "fixture: the patch is loaded after %d iterations" % target)
		check(absf(s.longitudinal_force()) <= rel_tol * ln / DT and absf(s.lateral_force()) <= rel_tol * ln / DT, "%d iterations at rest: no tangential force (%s, %s N for a load of %s N)" % [target, s.longitudinal_force(), s.lateral_force(), ln / DT])
		check(absf(s.rolling_resistance_torque()) <= rel_tol * spec.rolling_resistance * spec.rolling_radius * ln / DT, "%d iterations at rest: no rolling-resistance moment (%s N m)" % [target, s.rolling_resistance_torque()])
		check(Vector2(wheel.linear_velocity.dot(rig["e_x"]), wheel.linear_velocity.dot(rig["e_y"])).length() <= v_tol, "%d iterations at rest: no horizontal motion (%s m/s)" % [target, wheel.linear_velocity])
	s.finish()
	check(s.carried_tire_impulse().length() <= rel_tol * s.aggregate_normal_load() * DT, "no tangential impulse is carried to the next step")
	# (2) The real step path (ConstraintSolver with the split pass, warm starting, integration, gravity): a 15 kg wheel settles on its patch and rests
	# there for 120 steps (1 s). A free upright wheel on a patch has no roll stability of its own: from float32 noise a roll motion of the whole wheel,
	# with or without a tyre (checked on a tyre-less twin), grows with a time constant of a few tenths of a second, and after ~2 s it is not at rest any
	# more. The horizon is therefore the part of the motion that IS at rest; there the tyre must stay silent: it creates no force and no carried impulse.
	var lrig: Dictionary = make_rig({"height": R - DELTA, "gravity": G})
	var ls = lrig["support"]
	var lwheel: SimRigidBody = lrig["wheel"]
	var world: SimWorld = lrig["world"]
	var worst_force: float = 0.0
	var worst_carried: float = 0.0
	var worst_roll: float = 0.0
	var worst_speed: float = 0.0
	var stepped: bool = true
	for i in 120:
		stepped = world.step() and stepped
		var n: float = ls.normal_load()
		worst_force = maxf(worst_force, ls.tangential_force_world().length() / maxf(n, 1.0))
		worst_carried = maxf(worst_carried, ls.carried_tire_impulse().length() / maxf(n * DT, 1.0e-9))
		worst_roll = maxf(worst_roll, absf(ls.rolling_resistance_torque()) / (spec.rolling_resistance * spec.rolling_radius * maxf(n, 1.0)))
		worst_speed = maxf(worst_speed, Vector2(lwheel.linear_velocity.dot(lrig["e_x"]), lwheel.linear_velocity.dot(lrig["e_y"])).length())
	check(stepped, "all 120 steps complete")
	check_near(ls.normal_load(), LIGHT * G, 1.0e-4 * LIGHT * G, "fixture: the wheel rests on the patch with its weight")
	check(worst_force <= rel_tol, "no tangential force at rest (worst %s of the load)" % worst_force)
	check(worst_carried <= rel_tol, "no carried tangential impulse at rest (worst %s of the normal impulse)" % worst_carried)
	check(worst_roll <= rel_tol, "no rolling-resistance moment at rest (worst %s of c_rr N R)" % worst_roll)
	check(worst_speed <= v_tol, "no horizontal creep at rest (worst %s m/s, bound %s m/s)" % [worst_speed, v_tol])


func test_j_d_creep_at_rest_is_bounded_by_3_mu_vref_over_k_and_does_not_depend_on_the_sample_count() -> void:
	var spec = spec_with()
	var v_ref: float = spec.low_speed_reference
	var mu_x: float = spec.grip_longitudinal * MU_S
	var mu_y: float = spec.grip_lateral * MU_S
	var bound_x: float = 3.0 * mu_x * v_ref / spec.longitudinal_stiffness
	var bound_y: float = 3.0 * mu_y * v_ref / spec.lateral_stiffness
	check_near(bound_x, 0.08, 1.0e-12, "fixture: the documented 0.08 m/s of the prototype tyre (longitudinal)")
	var lever: float = resting_lever()
	var fractions: Array = [0.1, 0.5, 0.9, 0.99]  # tangential load as a fraction of the friction limit mu N
	var lateral_by_grid: Dictionary = {}
	for grid in [[9, 48], [17, 96]]:
		var key: String = "%dx%d" % [grid[0], grid[1]]
		var lateral: Array = []
		var previous_speed: float = 0.0
		for f in fractions:
			# The sliding speed that carries the load f mu N, from the inverse of the brush law (hub speed zero, one axis).
			var rho: float = brush_g_inverse(f)
			var s_lat: float = rho * mu_y * v_ref / spec.lateral_stiffness
			var s_lon: float = rho * mu_x * v_ref / spec.longitudinal_stiffness
			check(s_lat < bound_y and s_lon < bound_x and s_lat > previous_speed, "%s: load %s mu N is carried below the creep bound (%s of %s m/s lateral, %s of %s m/s longitudinal)" % [key, f, s_lat, bound_y, s_lon, bound_x])
			previous_speed = s_lat
			for sgn in [1.0, -1.0]:
				var label: String = "%s, load %s mu N, %s" % [key, f, "right/driving" if sgn > 0.0 else "left/braking"]
				# Lateral: the wheel slides sideways at rest. s_y = -v_y, so the force points against the sliding.
				var rig: Dictionary = heavy_rig(DELTA, {"lateral_speed": sgn * s_lat, "lat": grid[0], "circ": grid[1]})
				solve_in_place(rig)
				var s = rig["support"]
				var st: Dictionary = slip_state(rig)
				check(absf(st["hub"]) < 1.0e-5, "fixture: %s: zero hub speed, so D = v_ref" % label)
				check_near(s.lateral_force() / st["load"], -sgn * f * mu_y, 2.0e-4 * f * mu_y, "%s: lateral sliding carries exactly the load that the inverse law gives" % label)
				check(s.lateral_force() * sgn < 0.0 and absf(s.longitudinal_force()) <= 1.0e-6 * st["load"], "%s: the lateral force opposes the sliding, no longitudinal force" % label)
				check(s.friction_utilization() < 1.0 and absf(s.lateral_force()) < mu_y * st["load"], "%s: bounded below the friction limit" % label)
				if sgn > 0.0:
					lateral.append(s.lateral_force() / st["load"])
				# Longitudinal: the wheel spins in place at zero hub speed; positive spin = driving = force forward.
				var spinning: Dictionary = heavy_rig(DELTA, {"spin": sgn * s_lon / lever, "lat": grid[0], "circ": grid[1]})
				solve_in_place(spinning)
				var sp = spinning["support"]
				var sst: Dictionary = slip_state(spinning)
				var law: Dictionary = hand_law(spec, sst["s_x"], sst["s_y"], sqrt(sst["hub"] * sst["hub"] + v_ref * v_ref))
				check(absf(sst["hub"]) < 1.0e-5 and absf(sst["s_x"]) < bound_x and absf(sst["s_x"] - sgn * s_lon) < 0.05 * s_lon, "fixture: %s: spinning in place, s_x = %s below the bound %s" % [label, sst["s_x"], bound_x])
				check_near(sp.longitudinal_force() / sst["load"], law["fx"], 2.0e-4 * absf(law["fx"]), "%s: longitudinal creep against the law with D = v_ref" % label)
				check_near(sp.longitudinal_force() / sst["load"], sgn * f * mu_x, 0.05 * f * mu_x, "%s: and close to the load that the inverse law gives (the slip is measured at the effective point, hence the looser bound)" % label)
				check(sp.longitudinal_force() * sgn > 0.0 and sp.friction_utilization() < 1.0 and absf(sp.lateral_force()) <= 1.0e-6 * sst["load"], "%s: driving/braking direction, below the limit, no lateral force" % label)
		lateral_by_grid[key] = lateral
		# The creep bound is where the force saturates: at it and above it the force is the friction limit and no longer grows.
		for factor in [1.0, 1.5]:
			var rig: Dictionary = heavy_rig(DELTA, {"lateral_speed": factor * bound_y, "lat": grid[0], "circ": grid[1]})
			solve_in_place(rig)
			var st: Dictionary = slip_state(rig)
			check_near(absf(rig["support"].lateral_force()) / st["load"], mu_y, 2.0e-4 * mu_y, "%s: at %s times the creep bound the lateral force is mu N" % [key, factor])
			check_near(rig["support"].friction_utilization(), 1.0, 1.0e-4, "%s: and the ellipse is saturated" % key)
	# The response per unit load does not depend on the number of samples (pure lateral sliding: the slip velocity is the same at every sample).
	for i in fractions.size():
		check_near(lateral_by_grid["17x96"][i], lateral_by_grid["9x48"][i], 2.0e-4 * absf(lateral_by_grid["9x48"][i]), "load %s mu N: the same force per unit load with 3.8 times more samples" % fractions[i])


func test_j_e_repeated_solver_iterations_converge_to_the_bounded_response_and_apply_it_once() -> void:
	var spec = spec_with()
	var mu_y: float = spec.grip_lateral * MU_S
	# A 15 kg wheel (the implicit coupling matters) slides sideways at 0.05 m/s (unsaturated) or is at rest, for 1 to 80 iterations of ONE solve.
	# The tyre responds to the slip of the CURRENT iterate. After ONE iteration the normal rows (Gauss-Seidel, row by row) have not converged and
	# leave the wheel in an asymmetric motion even though it started at rest (the tyre-less twin shows the same motion), so the tyre legitimately sees a
	# slip: measured 2 % of the normal impulse after 1 iteration, 3e-4 after 2, 1e-5 after 5 and 1e-9 from 10 iterations on (float32 noise). This is the
	# transient of the normal solve, not an accumulation. Hence the budget and the friction limit are checked for every count, and the zero response at
	# rest and the real response while sliding only from 10 iterations on (the default is 20, SimConfig.solver_iterations).
	var tangential_by_count: Dictionary = {}
	for iterations in [1, 10, 20, 80]:
		for sliding in [true, false]:
			var label: String = "%d iterations, %s" % [iterations, "sliding at 0.05 m/s" if sliding else "at rest"]
			var rig: Dictionary = make_rig({"height": R - DELTA, "lateral_speed": 0.05 if sliding else 0.0, "iterations": iterations, "spec_overrides": {"rolling_resistance": 0.0}})
			var s = rig["support"]
			var wheel: SimRigidBody = rig["wheel"]
			var v0: Vector3 = wheel.linear_velocity
			solve_in_place(rig, iterations)
			var lam_n: float = oracle_weighted(s.sample_report())["load"] * DT
			var impulse: Vector3 = s.tangential_force_world() * DT
			# The horizontal momentum change of the wheel is the tangential impulse, once (the normal rows act along n only).
			var dv: Vector3 = wheel.linear_velocity - v0
			var horizontal: Vector3 = wheel.mass * (dv - Vector3.UP * dv.dot(Vector3.UP))
			check_vec3_near(horizontal, impulse, 1.0e-5 * (impulse.length() + lam_n * 1.0e-3), "%s: the momentum change is the reported tangential impulse, applied once" % label)
			check(impulse.length() <= mu_y * lam_n * (1.0 + 1.0e-6), "%s: the impulse stays inside the friction limit (%s of %s)" % [label, impulse.length(), mu_y * lam_n])
			if iterations < 10:
				continue
			if sliding:
				check(impulse.length() > 0.05 * mu_y * lam_n and s.lateral_force() < 0.0, "fixture: %s: a real response against the sliding to the right" % label)
				tangential_by_count[iterations] = impulse.length() / lam_n
			else:
				check(impulse.length() <= 1.0e-6 * lam_n, "%s: no tangential impulse accumulates at rest (%s of the normal impulse %s)" % [label, impulse.length(), lam_n])
	# Converged: the response per unit normal impulse of 20 and 80 iterations agree (the implicit lag of the friction factor has died out), so
	# further iterations do not add impulse.
	check_near(tangential_by_count[80], tangential_by_count[20], 2.0e-3 * tangential_by_count[80], "the response per unit normal impulse is converged (20 against 80 iterations: %s, %s)" % [tangential_by_count[20], tangential_by_count[80]])
