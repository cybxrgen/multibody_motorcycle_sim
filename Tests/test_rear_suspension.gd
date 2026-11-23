extends "res://Tests/sim_test_case.gd"
## M2.3 rear spring-damper (Simulation/Motorcycle/rear_suspension.gd) and its use of the generic per-step force-provider hook.
##
## A spring-damper physics (analytic cases, direction following the instantaneous line, rotation, torques, momentum, power, energy)
## B dynamics of a chassis/swingarm pair (exact discrete reference, equilibrium, static/dynamic participation, energy, sustained run)
## C validation, degenerate geometry and provider lifecycle (preflight, once per fixed step)
## E regression sensitivity: one battery of physical criteria that must accept the real component and reject each in-suite mutant
##
## Conventions under test (PHYSICS_CONVENTIONS.md, "Rear suspension spring-damper"): n = (p_s - p_c) / l points from the chassis
## attachment toward the swingarm attachment; l_dot = n . (v(p_s) - v(p_c)) with v(p) = v_com + w x (p - x_com);
## F = -k (l - l_0) - c l_dot acts on the swingarm as +F n at p_s and on the chassis as -F n at p_c.
##
## Methodology / tolerances:
## * EVERY expectation is computed here from the bodies' own pose and velocity with test-owned local attachment points and plain
##   vector algebra (Oracle below); the component's query methods are compared WITH it, never used as the oracle.
## * Static quantities (forces up to ~10^2 N, torques, powers) come from float32 poses and velocities of order 1: the tolerance is
##   2e-5 relative to the magnitude of the expected value, floored at 1e-4 absolute (policy: >= 1e-6 relative for float32 vectors).
## * Dynamics are compared with the EXACT discrete scheme the world implements (force from the start-of-step state, semi-implicit
##   Euler), computed in float64. The continuous-time trajectory is never demanded from the first-order scheme.

const RearSuspension = preload("res://Simulation/Motorcycle/rear_suspension.gd")
const SimForceProvider = preload("res://Simulation/Core/sim_force_provider.gd")

const DT: float = 1.0 / 120.0

## Reference attachment points (world, reference pose) of the standard rig: an INCLINED line, neither vertical nor along any
## body axis. Reference length |P_S - P_C| = sqrt(0.2^2 + 0.4^2 + 0.35^2) = 0.567891...
const P_C: Vector3 = Vector3(0.15, 0.8, 0.2)
const P_S: Vector3 = Vector3(-0.05, 0.4, 0.55)


static func ref_length() -> float:
	return (P_S - P_C).length()


# =====================================================================================================================
# Fixtures
# =====================================================================================================================

## Standard two-body rig: a rotated chassis (30 kg, box inertia) and a rotated swingarm (6 kg, box inertia), both with COMs OFF the
## attachment points. Options: k (2000), c (50), rest (reference length), static_chassis (false), gravity (zero), sphere (false:
## isotropic inertia so that no gyroscopic term exists), register (true). Returns {world, cb, sb, susp, lc, ls, k, c, l0}.
## lc / ls are TEST-OWNED local attachment points computed from the reference geometry (never read back from the component).
func _rig(options: Dictionary = {}) -> Dictionary:
	var w: SimWorld = make_world(options.get("gravity", Vector3.ZERO))
	var qc: Quaternion = Quaternion(Vector3(0.3, 1.0, 0.2).normalized(), 0.4)
	var qs: Quaternion = Quaternion(Vector3(1.0, -0.4, 0.5).normalized(), 0.3)
	var pos_c: Vector3 = Vector3(0.1, 0.9, 0.05)
	var pos_s: Vector3 = Vector3(0.0, 0.2, 0.9)
	var sphere: bool = options.get("sphere", false)
	var cb: SimRigidBody
	if options.get("static_chassis", false):
		cb = make_static(pos_c)
		cb.orientation = qc
	else:
		cb = make_body(30.0, SimMassProperties.sphere_inertia(30.0, 0.3) if sphere else SimMassProperties.box_inertia(30.0, Vector3(0.3, 0.5, 1.0)), pos_c, qc)
	var sb: SimRigidBody = make_body(6.0, SimMassProperties.sphere_inertia(6.0, 0.15) if sphere else SimMassProperties.box_inertia(6.0, Vector3(0.2, 0.1, 0.6)), pos_s, qs)
	w.add_body(cb)
	w.add_body(sb)
	var k: float = options.get("k", 2000.0)
	var c: float = options.get("c", 50.0)
	var l0: float = options.get("rest", ref_length())
	var lc: Vector3 = Basis(qc).inverse() * (P_C - pos_c)
	var ls: Vector3 = Basis(qs).inverse() * (P_S - pos_s)
	var susp = RearSuspension.from_world(cb, sb, P_C, P_S, k, c, l0)
	check(susp != null, "the rear suspension builds")
	if susp != null and options.get("register", true):
		w.add_force_provider(susp)
	return {"world": w, "cb": cb, "sb": sb, "susp": susp, "lc": lc, "ls": ls, "k": k, "c": c, "l0": l0}


## Independent oracle: everything derived from the bodies' own state and the test-owned local points.
func _oracle(r: Dictionary) -> Dictionary:
	var cb: SimRigidBody = r.cb
	var sb: SimRigidBody = r.sb
	var pc: Vector3 = cb.position + Basis(cb.orientation) * r.lc
	var ps: Vector3 = sb.position + Basis(sb.orientation) * r.ls
	var d: Vector3 = ps - pc
	var l: float = d.length()
	var n: Vector3 = d / l
	var vc: Vector3 = cb.linear_velocity + cb.angular_velocity.cross(pc - cb.position)
	var vs: Vector3 = sb.linear_velocity + sb.angular_velocity.cross(ps - sb.position)
	var ldot: float = n.dot(vs - vc)
	var f_spring: float = -r.k * (l - r.l0)
	var f_damp: float = -r.c * ldot
	var f: float = f_spring + f_damp
	var fs: Vector3 = n * f
	var fc: Vector3 = -n * f
	return {"pc": pc, "ps": ps, "l": l, "n": n, "ldot": ldot, "f_spring": f_spring, "f_damp": f_damp, "F": f, "Fs": fs, "Fc": fc,
		"tau_s": (ps - sb.position).cross(fs), "tau_c": (pc - cb.position).cross(fc), "U": 0.5 * r.k * (l - r.l0) * (l - r.l0)}


func _tol(expected: float) -> float:
	return maxf(1.0e-4, 2.0e-5 * absf(expected))


func _vtol(expected: Vector3) -> float:
	return maxf(1.0e-4, 2.0e-5 * expected.length())


## Applies the provider once and compares force and torque on both bodies with the oracle.
func _check_applied(r: Dictionary, label: String) -> void:
	var o: Dictionary = _oracle(r)
	r.cb.clear_accumulators()
	r.sb.clear_accumulators()
	r.susp.apply_forces(DT, r.world.config)
	if not r.cb.is_static():
		check_vec3_near(r.cb.force_accumulator, o.Fc, _vtol(o.Fc), "%s: chassis force = -F n" % label)
		check_vec3_near(r.cb.torque_accumulator, o.tau_c, _vtol(o.tau_c), "%s: chassis torque = (p_c - x_c) x F_c" % label)
	check_vec3_near(r.sb.force_accumulator, o.Fs, _vtol(o.Fs), "%s: swingarm force = +F n" % label)
	check_vec3_near(r.sb.torque_accumulator, o.tau_s, _vtol(o.tau_s), "%s: swingarm torque = (p_s - x_s) x F_s" % label)


# =====================================================================================================================
# A. Spring-damper physics
# =====================================================================================================================

func test_a1_zero_force_at_rest_length_and_zero_relative_rate() -> void:
	var r: Dictionary = _rig()
	var s = r.susp
	var o: Dictionary = _oracle(r)
	check_near(o.l, ref_length(), 1.0e-6, "the rig starts at the configured rest length")
	check_near(s.length(), r.l0, 1.0e-6, "length() = l_0")
	check_near(s.extension(), 0.0, 1.0e-6, "zero extension")
	check_near(s.axial_force(), 0.0, 1.0e-3, "zero scalar force at rest length with zero rate")
	s.apply_forces(DT, r.world.config)
	check_vec3_near(r.sb.force_accumulator, Vector3.ZERO, 1.0e-3, "no force on the swingarm")
	check_vec3_near(r.cb.force_accumulator, Vector3.ZERO, 1.0e-3, "no force on the chassis")
	check_vec3_near(r.sb.torque_accumulator, Vector3.ZERO, 1.0e-3, "no torque on the swingarm")
	# A RIGID motion of the pair (common w about a point O, v_i = w x (x_i - O)) leaves the length unchanged: l_dot = 0 exactly,
	# although both attachment points move and rotate. Only a correct rotational velocity contribution gives zero here.
	var w_common: Vector3 = Vector3(0.7, -1.1, 0.5)
	var origin: Vector3 = Vector3(0.3, 0.2, -0.1)
	for body in [r.cb, r.sb]:
		body.angular_velocity = w_common
		body.linear_velocity = w_common.cross(body.position - origin)
	check_near(_oracle(r).ldot, 0.0, 1.0e-5, "oracle: a rigid motion of the pair has l_dot = 0")
	check_near(s.relative_extension_rate(), 0.0, 2.0e-5, "a rigid motion of the pair (both rotating and translating) has zero extension rate")
	check_near(s.damping_force(), 0.0, 2.0e-3, "and no damping force")


func test_a2_compression_gives_a_restoring_force_along_the_line() -> void:
	var k: float = 2000.0
	for compress in [0.01, 0.05, 0.12]:
		# Rest length LONGER than the geometric length by `compress`: the shock is compressed by that much.
		var r: Dictionary = _rig({"k": k, "c": 0.0, "rest": ref_length() + compress})
		var o: Dictionary = _oracle(r)
		check_near(o.F, k * compress, 1.0e-3, "compression %s: F = -k (l - l_0) = +k x is positive" % compress)
		check_near(r.susp.extension(), -compress, 1.0e-6, "extension = l - l_0 = -%s" % compress)
		check_near(r.susp.spring_force(), k * compress, _tol(k * compress), "compression %s: spring force" % compress)
		_check_applied(r, "compression %s" % compress)
		# Restoring: the swingarm is pushed AWAY from the chassis (along +n), the chassis the other way.
		check(r.sb.force_accumulator.dot(o.n) > 0.0, "compression %s: the swingarm is pushed away from the chassis" % compress)
		check(r.cb.force_accumulator.dot(o.n) < 0.0, "compression %s: the chassis is pushed away from the swingarm" % compress)


func test_a3_extension_gives_a_restoring_force_along_the_line() -> void:
	var k: float = 2000.0
	for stretch in [0.01, 0.05, 0.12]:
		var r: Dictionary = _rig({"k": k, "c": 0.0, "rest": ref_length() - stretch})
		var o: Dictionary = _oracle(r)
		check_near(o.F, -k * stretch, 1.0e-3, "extension %s: F = -k (l - l_0) = -k x is negative" % stretch)
		check_near(r.susp.extension(), stretch, 1.0e-6, "extension = l - l_0 = +%s" % stretch)
		_check_applied(r, "extension %s" % stretch)
		check(r.sb.force_accumulator.dot(o.n) < 0.0, "extension %s: the swingarm is pulled toward the chassis" % stretch)
		check(r.cb.force_accumulator.dot(o.n) > 0.0, "extension %s: the chassis is pulled toward the swingarm" % stretch)


func test_a4_damping_opposes_extension_and_compression_velocity() -> void:
	var c: float = 80.0
	for v in [0.9, -0.9, 0.05, -2.0]:
		# Swingarm moves along +n at v (v > 0 lengthens the shock). Chassis at rest, no rotation: l_dot = v.
		var r: Dictionary = _rig({"k": 0.0, "c": c})
		var n: Vector3 = _oracle(r).n
		r.sb.linear_velocity = n * v
		var o: Dictionary = _oracle(r)
		check_near(o.ldot, v, 1.0e-5, "v = %s: the rig realises l_dot = v" % v)
		check_near(r.susp.relative_extension_rate(), v, 2.0e-5, "v = %s: relative extension rate" % v)
		check_near(r.susp.damping_force(), -c * v, _tol(c * v), "v = %s: damping force = -c l_dot" % v)
		check_near(r.susp.spring_force(), 0.0, 0.0, "v = %s: no spring force with k = 0" % v)
		_check_applied(r, "v = %s" % v)
		check(r.sb.force_accumulator.dot(n) * v < 0.0, "v = %s: the force on the swingarm opposes its relative motion" % v)
		check(r.cb.force_accumulator.dot(n) * v > 0.0, "v = %s: the reaction on the chassis points along the extension" % v)
	# Chassis moving away from the swingarm (along -n) lengthens the shock as well: the relative rate is what counts.
	var r2: Dictionary = _rig({"k": 0.0, "c": c})
	r2.cb.linear_velocity = -_oracle(r2).n * 0.5
	check_near(r2.susp.relative_extension_rate(), 0.5, 2.0e-5, "chassis moving along -n is an extension")
	check_near(r2.susp.damping_force(), -c * 0.5, _tol(c * 0.5), "and is damped")
	# A common translation is not damped.
	var r3: Dictionary = _rig({"k": 0.0, "c": c})
	var common: Vector3 = Vector3(1.0, -2.0, 0.7)
	r3.cb.linear_velocity = common
	r3.sb.linear_velocity = common
	check_near(r3.susp.damping_force(), 0.0, 1.0e-3, "a common translation produces no damping force")


func test_a5_relative_rate_includes_the_angular_velocity_of_either_or_both_bodies() -> void:
	# Angular velocities along r x n of each attachment (n . (w x r) = w . (r x n)), so each rotation shortens or lengthens the shock
	# at ~|r x n| * |w| and the case cannot pass by accident.
	var probe: Dictionary = _rig({"k": 0.0, "c": 60.0})
	var po: Dictionary = _oracle(probe)
	var w_s: Vector3 = 2.0 * (po.ps - probe.sb.position).cross(po.n).normalized()
	var w_c: Vector3 = -3.0 * (po.pc - probe.cb.position).cross(po.n).normalized()
	var cases: Array = [
		["swingarm only", Vector3.ZERO, w_s],
		["chassis only", w_c, Vector3.ZERO],
		["both", w_c, w_s],
	]
	for case in cases:
		var r: Dictionary = _rig({"k": 0.0, "c": 60.0})
		r.cb.angular_velocity = case[1]
		r.sb.angular_velocity = case[2]
		var o: Dictionary = _oracle(r)
		# Independent re-derivation of l_dot from the cross products (not the oracle's own variables).
		var by_hand: float = o.n.dot(case[2].cross(o.ps - r.sb.position)) - o.n.dot(case[1].cross(o.pc - r.cb.position))
		check_near(o.ldot, by_hand, 1.0e-6, "%s: oracle l_dot = n.(w_s x r_s) - n.(w_c x r_c)" % case[0])
		check(absf(o.ldot) > 0.2, "%s: the case really exercises the rotational contribution (l_dot = %s)" % [case[0], o.ldot])
		check_near(r.susp.relative_extension_rate(), o.ldot, 2.0e-5, "%s: relative extension rate counts the rotation" % case[0])
		check_near(r.susp.damping_force(), o.f_damp, _tol(o.f_damp), "%s: damping force from the rotation-induced rate" % case[0])
		check_near(r.susp.damping_power(), -r.c * o.ldot * o.ldot, 1.0e-3, "%s: damping power = -c l_dot^2" % case[0])
		_check_applied(r, case[0])
		# Translation of the COMs alone would give a different (here: zero) rate: the whole effect is rotational.
		check_near(o.n.dot(r.sb.linear_velocity - r.cb.linear_velocity), 0.0, 1.0e-6, "%s: the COM velocities contribute nothing" % case[0])


func test_a6_force_follows_the_instantaneous_line_not_a_fixed_axis() -> void:
	var directions: Array = []
	for angle in [-0.5, -0.2, 0.0, 0.3, 0.6]:
		var r: Dictionary = _rig({"k": 1500.0, "c": 0.0, "rest": 0.40})
		# Rotate the swingarm about its own x axis: the attachment point swings, the line to the chassis point changes.
		r.sb.orientation = Quaternion(Vector3.RIGHT, angle) * r.sb.orientation
		var o: Dictionary = _oracle(r)
		directions.append(o.n)
		_check_applied(r, "swingarm angle %s" % angle)
		var f_dir: Vector3 = r.sb.force_accumulator.normalized()
		check(f_dir.dot(o.n) > 1.0 - 1.0e-6 or f_dir.dot(o.n) < -1.0 + 1.0e-6, "angle %s: the force on the swingarm is parallel to the instantaneous line p_s - p_c" % angle)
		check((r.sb.force_accumulator.cross(o.ps - o.pc)).length() <= 1.0e-3, "angle %s: F x (p_s - p_c) = 0" % angle)
		check(absf(o.n.y) < 0.97 and absf(o.n.z) < 0.97 and absf(o.n.x) < 0.97, "angle %s: the shock is not along a world axis" % angle)
		for local_axis in [Vector3.RIGHT, Vector3.UP, Vector3.BACK]:
			var world_axis: Vector3 = Basis(r.sb.orientation) * local_axis
			check(absf(world_axis.dot(o.n)) < 0.995, "angle %s: nor along a swingarm axis" % angle)
	# The direction really changes with the pose (otherwise the test would not distinguish a fixed direction).
	check(acos(clampf(directions[0].dot(directions[4]), -1.0, 1.0)) > 0.25, "the line direction turned by more than 0.25 rad across the poses")
	# The same holds for a CHASSIS rotation (e.g. pitching): the force direction follows the chassis attachment too.
	var r2: Dictionary = _rig({"k": 1500.0, "c": 0.0, "rest": 0.40})
	var n_before: Vector3 = _oracle(r2).n
	r2.cb.orientation = Quaternion(Vector3.RIGHT, 0.5) * r2.cb.orientation
	check(acos(clampf(_oracle(r2).n.dot(n_before), -1.0, 1.0)) > 0.1, "a chassis rotation also turns the line")
	_check_applied(r2, "rotated chassis")


func test_a7_forces_are_equal_and_opposite_and_the_net_force_is_zero() -> void:
	var r: Dictionary = _rig({"k": 1800.0, "c": 70.0, "rest": 0.5})
	r.cb.linear_velocity = Vector3(0.3, -0.2, 0.5)
	r.sb.linear_velocity = Vector3(-0.4, 0.6, 0.1)
	r.cb.angular_velocity = Vector3(0.2, 0.5, -0.3)
	r.sb.angular_velocity = Vector3(-0.9, 0.4, 0.6)
	var o: Dictionary = _oracle(r)
	check(absf(o.F) > 10.0, "the state carries a substantial force (F = %s N)" % o.F)
	r.susp.apply_forces(DT, r.world.config)
	check(r.cb.force_accumulator + r.sb.force_accumulator == Vector3.ZERO, "net applied force is exactly zero (equal and opposite)")
	check_vec3_near(r.sb.force_accumulator, -r.cb.force_accumulator, 1.0e-6, "F_s = -F_c")
	check_near(r.sb.force_accumulator.length(), absf(o.F), _tol(o.F), "|F_s| = |F|")
	_check_applied(r, "general state")


func test_a8_torques_come_from_the_attachment_lever_arms() -> void:
	var r: Dictionary = _rig({"k": 1800.0, "c": 70.0, "rest": 0.5})
	r.sb.angular_velocity = Vector3(-0.9, 0.4, 0.6)
	var o: Dictionary = _oracle(r)
	r.susp.apply_forces(DT, r.world.config)
	var lever_s: Vector3 = o.ps - r.sb.position
	var lever_c: Vector3 = o.pc - r.cb.position
	check(lever_s.length() > 0.05 and lever_c.length() > 0.05, "both attachments are off their COMs (levers %s, %s m)" % [lever_s.length(), lever_c.length()])
	check_vec3_near(r.sb.torque_accumulator, lever_s.cross(o.Fs), _vtol(o.tau_s), "swingarm torque = r_s x F_s")
	check_vec3_near(r.cb.torque_accumulator, lever_c.cross(o.Fc), _vtol(o.tau_c), "chassis torque = r_c x F_c")
	check(r.sb.torque_accumulator.length() > 1.0 and r.cb.torque_accumulator.length() > 1.0, "the torques are substantial, not zero by accident")
	# Torque is perpendicular to both the lever and the force.
	check(absf(r.sb.torque_accumulator.dot(lever_s)) <= 1.0e-3 and absf(r.sb.torque_accumulator.dot(o.Fs)) <= 1.0e-3, "torque is perpendicular to lever and force")
	# Net torque about the ORIGIN is zero: sum(x_i x F_i + tau_i) = (p_s - p_c) x F_s = 0 because the pair is collinear.
	var about_origin: Vector3 = r.sb.position.cross(r.sb.force_accumulator) + r.cb.position.cross(r.cb.force_accumulator) \
		+ r.sb.torque_accumulator + r.cb.torque_accumulator
	check(about_origin.length() <= 2.0e-3, "net torque about the origin is zero for the collinear pair (%s N m of ~%s)" % [about_origin.length(), o.F * 0.6])
	# A COM attachment has no lever: no torque on that body.
	var w: SimWorld = make_world()
	var b1: SimRigidBody = make_sphere(5.0, 0.2, Vector3(0.0, 1.0, 0.0))
	var b2: SimRigidBody = make_sphere(2.0, 0.2, Vector3(0.3, 0.4, 0.2))
	w.add_body(b1)
	w.add_body(b2)
	var s = RearSuspension.from_world(b1, b2, b1.position, b2.position, 1000.0, 0.0, 0.5)
	s.apply_forces(DT, w.config)
	check(b1.torque_accumulator == Vector3.ZERO and b2.torque_accumulator == Vector3.ZERO, "attachments at the COMs: no torque")


func test_a9_isolated_pair_conserves_linear_momentum_and_angular_momentum() -> void:
	# Spherical bodies: no gyroscopic term, so any change of L would come from the force pair itself. Zero gravity.
	var r: Dictionary = _rig({"k": 1800.0, "c": 40.0, "rest": 0.5, "sphere": true})
	r.cb.linear_velocity = Vector3(0.4, -0.1, 0.2)
	r.sb.linear_velocity = Vector3(-0.3, 0.5, 0.1)
	r.cb.angular_velocity = Vector3(0.2, 0.5, -0.3)
	r.sb.angular_velocity = Vector3(-0.9, 0.4, 0.6)
	var w: SimWorld = r.world
	var p0: Vector3 = w.total_linear_momentum()
	var l0: Vector3 = w.total_angular_momentum()
	var worst_p: float = 0.0
	var worst_l: float = 0.0
	var excursion: float = 0.0
	var peak_rate: float = 0.0
	for i in 480:
		w.step()
		worst_p = maxf(worst_p, (w.total_linear_momentum() - p0).length())
		worst_l = maxf(worst_l, (w.total_angular_momentum() - l0).length())
		excursion = maxf(excursion, absf(r.susp.extension()))
		peak_rate = maxf(peak_rate, absf(r.susp.relative_extension_rate()))
	check(w.numerical_fault == "", "no fault")
	# Float32 random walk of a ~30 kg m/s momentum over 480 steps: 2e-3 absolute; L (including r x p of order 10) 1e-2.
	check(worst_p <= 2.0e-3, "total linear momentum is preserved by the internal pair (worst deviation %s kg m/s of |P| = %s)" % [worst_p, p0.length()])
	check(worst_l <= 1.0e-2, "total angular momentum about the origin is preserved by the collinear pair (worst deviation %s of |L| = %s)" % [worst_l, l0.length()])
	# And the shock really worked: the relative state changed substantially, so the conservation is not vacuous.
	check(excursion > 0.02 and peak_rate > 0.3, "the shock was exercised (peak extension %s m, peak rate %s m/s)" % [excursion, peak_rate])


func test_a10_damping_power_is_never_positive_and_matches_the_applied_pair() -> void:
	var rng: RandomNumberGenerator = RandomNumberGenerator.new()
	rng.seed = 20260611
	var worst_positive: float = -INF
	var worst_mismatch: float = 0.0
	var spring_power_error: float = 0.0
	for i in 60:
		var r: Dictionary = _rig({"k": 0.0, "c": rng.randf_range(5.0, 200.0), "rest": 0.3})
		var tilt: Quaternion = Quaternion(Vector3(rng.randf_range(-1, 1), rng.randf_range(-1, 1), rng.randf_range(-1, 1)).normalized(), rng.randf_range(-0.6, 0.6))
		r.sb.orientation = tilt * r.sb.orientation
		for body in [r.cb, r.sb]:
			body.linear_velocity = Vector3(rng.randf_range(-2, 2), rng.randf_range(-2, 2), rng.randf_range(-2, 2))
			body.angular_velocity = Vector3(rng.randf_range(-3, 3), rng.randf_range(-3, 3), rng.randf_range(-3, 3))
		var o: Dictionary = _oracle(r)
		r.susp.apply_forces(DT, r.world.config)
		# Power of the applied pair from the bodies' own point velocities (independent of l_dot): F_s . v(p_s) + F_c . v(p_c).
		var vs: Vector3 = r.sb.linear_velocity + r.sb.angular_velocity.cross(o.ps - r.sb.position)
		var vc: Vector3 = r.cb.linear_velocity + r.cb.angular_velocity.cross(o.pc - r.cb.position)
		var power: float = r.sb.force_accumulator.dot(vs) + r.cb.force_accumulator.dot(vc)
		worst_positive = maxf(worst_positive, power)
		worst_mismatch = maxf(worst_mismatch, absf(power - (-r.c * o.ldot * o.ldot)))
		check_near(r.susp.damping_power(), -r.c * o.ldot * o.ldot, 1.0e-3 + 2.0e-5 * r.c * o.ldot * o.ldot, "sample %d: damping_power() = -c l_dot^2" % i)
	# Rounding of F.v for |F| ~ 10^2 N and |v| ~ 4 m/s: ~1e-4 W.
	check(worst_positive <= 1.0e-3, "the damping pair never does positive work (largest power %s W)" % worst_positive)
	check(worst_mismatch <= 2.0e-2, "the applied pair's power equals -c l_dot^2 (worst mismatch %s W)" % worst_mismatch)
	# With a spring too, the pair's power is F l_dot and its spring part is -dU/dt (checked by a small kinematic displacement).
	var r2: Dictionary = _rig({"k": 2500.0, "c": 0.0, "rest": 0.5})
	r2.sb.linear_velocity = _oracle(r2).n * 0.8
	var u_before: float = _oracle(r2).U
	var h: float = 1.0e-4
	r2.sb.position += r2.sb.linear_velocity * h
	var u_after: float = _oracle(r2).U
	var spring_power: float = r2.susp.spring_force() * r2.susp.relative_extension_rate()
	check_near(spring_power, -(u_after - u_before) / h, 0.05 * absf(spring_power) + 0.5, "spring power F_spring l_dot = -dU/dt (finite difference over 1e-4 s)")


func test_a11_spring_energy_follows_the_configured_rest_length() -> void:
	var k: float = 2400.0
	for delta in [0.0, 0.02, -0.02, 0.1, -0.1]:
		var r: Dictionary = _rig({"k": k, "c": 0.0, "rest": ref_length() - delta})
		# l - l_0 = delta.
		check_near(r.susp.spring_energy(), 0.5 * k * delta * delta, 1.0e-3 + 1.0e-5 * k * delta * delta, "l - l_0 = %s: U = k delta^2 / 2" % delta)
		check_near(_oracle(r).U, 0.5 * k * delta * delta, 1.0e-3 + 1.0e-5 * k * delta * delta, "oracle agrees")
	check_near(_rig({"k": k, "c": 0.0}).susp.spring_energy(), 0.0, 1.0e-6, "zero energy at the rest length")
	# The energy depends on the CONFIGURED rest length, not on the reference pose: same pose, other l_0.
	var a: Dictionary = _rig({"k": k, "c": 0.0, "rest": 0.5})
	var b: Dictionary = _rig({"k": k, "c": 0.0, "rest": 0.6})
	check(absf(a.susp.spring_energy() - b.susp.spring_energy()) > 1.0, "a different rest length changes the stored energy at the same pose")


func test_a12_zero_stiffness_and_zero_damping_limiting_cases() -> void:
	var rest_stretch: float = 0.07
	var r0: Dictionary = _rig({"k": 0.0, "c": 0.0, "rest": ref_length() - rest_stretch})
	r0.sb.linear_velocity = _oracle(r0).n * 1.5
	r0.sb.angular_velocity = Vector3(1.0, 2.0, -1.0)
	check(r0.susp.is_valid(), "k = c = 0 is valid")
	check(r0.susp.axial_force() == 0.0 and r0.susp.spring_energy() == 0.0 and r0.susp.damping_power() == 0.0, "k = c = 0: no force, no energy, no power")
	r0.susp.apply_forces(DT, r0.world.config)
	check(r0.sb.force_accumulator == Vector3.ZERO and r0.cb.force_accumulator == Vector3.ZERO, "k = c = 0 applies exactly zero force")
	var rk: Dictionary = _rig({"k": 2000.0, "c": 0.0, "rest": ref_length() - rest_stretch})
	rk.sb.linear_velocity = _oracle(rk).n * 1.5
	check_near(rk.susp.damping_force(), 0.0, 0.0, "c = 0: no damping force even while moving")
	check_near(rk.susp.axial_force(), -2000.0 * rest_stretch, _tol(2000.0 * rest_stretch), "c = 0: the spring alone")
	var rc: Dictionary = _rig({"k": 0.0, "c": 60.0, "rest": ref_length() - rest_stretch})
	check_near(rc.susp.spring_force(), 0.0, 0.0, "k = 0: no spring force although the length differs from l_0")
	check_near(rc.susp.axial_force(), 0.0, 1.0e-6, "k = 0 at rest: zero force")
	check_near(rc.susp.spring_energy(), 0.0, 0.0, "k = 0: no stored energy")


# =====================================================================================================================
# B. Dynamics of a chassis / swingarm pair
# =====================================================================================================================

## Two spherical bodies whose attachment points ARE their COMs, on an INCLINED line (no rotation is ever excited, the motion stays
## on the line). Relative length x = l - l_0 obeys, with the reduced mass mu = m1 m2 / (m1 + m2), the scheme the world implements:
##   F_n = -k x_n - c v_n;  v_{n+1} = v_n + h F_n / mu;  x_{n+1} = x_n + h v_{n+1}   (semi-implicit Euler, force from the START state).
## `static_chassis`: the chassis is static, the swingarm mass is mu.
func _line_pair(k: float, c: float, x0: float, static_chassis: bool, gravity: Vector3 = Vector3.ZERO) -> Dictionary:
	var w: SimWorld = make_world(gravity)
	var dir: Vector3 = Vector3(0.36, -0.8, 0.48).normalized()
	var l0: float = 0.5
	var pos_c: Vector3 = Vector3(0.0, 1.0, 0.0)
	var cb: SimRigidBody = make_static(pos_c) if static_chassis else make_sphere(40.0, 0.3, pos_c)
	var sb: SimRigidBody = make_sphere(8.0, 0.15, pos_c + dir * (l0 + x0))
	w.add_body(cb)
	w.add_body(sb)
	var susp = RearSuspension.from_world(cb, sb, cb.position, sb.position, k, c, l0)
	w.add_force_provider(susp)
	var mu: float = 8.0 if static_chassis else 40.0 * 8.0 / 48.0
	return {"world": w, "cb": cb, "sb": sb, "susp": susp, "n": dir, "mu": mu, "l0": l0}


func _discrete_line_run(k: float, c: float, mu: float, x0: float, steps: int) -> PackedFloat64Array:
	var xs: PackedFloat64Array = PackedFloat64Array()
	var x: float = x0
	var v: float = 0.0
	xs.append(x)
	for i in steps:
		var f: float = -k * x - c * v
		v += DT * f / mu
		x += DT * v
		xs.append(x)
	return xs


func test_b1_compression_and_extension_responses_match_the_exact_discrete_scheme() -> void:
	# k = 3000 N/m, mu = 6.67 kg: w = 21.2 rad/s, w h = 0.177; c = 60: zeta = 0.21 (c h / mu = 0.075 < 2: explicit damping dissipates).
	for case in [[3000.0, 0.0, 0.04, false], [3000.0, 60.0, 0.04, false], [3000.0, 60.0, -0.04, false], [3000.0, 60.0, 0.04, true]]:
		var rig: Dictionary = _line_pair(case[0], case[1], case[2], case[3])
		var expect: PackedFloat64Array = _discrete_line_run(case[0], case[1], rig.mu, case[2], 240)
		var worst: float = 0.0
		var excursion: float = 0.0
		for i in 240:
			rig.world.step()
			var x: float = (rig.sb.position - rig.cb.position).length() - rig.l0
			worst = maxf(worst, absf(x - expect[i + 1]))
			excursion = maxf(excursion, absf(expect[i + 1]))
		var label: String = "k=%s c=%s x0=%s static_chassis=%s" % [case[0], case[1], case[2], case[3]]
		# Float32 poses of metre size give ~1e-7 m per position; 5e-6 m is 50x that on a 0.04 m excursion (measured ~1e-7).
		check(worst <= 5.0e-6, "%s: the simulated length equals the exact discrete solution (worst %s m, excursion %s m)" % [label, worst, excursion])
		check(excursion > 0.03, "%s: the run exercised the shock" % label)
		# Direction never left the configured line (the motion is pure translation along it).
		check((rig.sb.position - rig.cb.position).normalized().dot(rig.n) > 1.0 - 1.0e-6, "%s: the pair stayed on the line" % label)
		check(rig.sb.angular_velocity.length() < 1.0e-6, "%s: no rotation was excited by a force through the COMs" % label)
		if case[3]:
			check(rig.cb.position == Vector3(0.0, 1.0, 0.0) and rig.cb.linear_velocity == Vector3.ZERO, "static chassis untouched")
	# The first-order scheme is NOT asked to follow the continuous solution exactly; its frequency error (w h)^2 / 24 is bounded here.
	var cont: Dictionary = _line_pair(3000.0, 0.0, 0.04, false)
	var w_n: float = sqrt(3000.0 / cont.mu)
	var worst_cont: float = 0.0
	for i in 120:
		cont.world.step()
		var t: float = (i + 1) * DT
		var x: float = (cont.sb.position - cont.cb.position).length() - cont.l0
		worst_cont = maxf(worst_cont, absf(x - 0.04 * cos(w_n * t)))
	# Two analytic error terms of the symplectic-Euler map started from rest: (i) the velocity is updated first, which is the exact
	# solution shifted by h/2 in time (phase w h / 2 = 0.088 rad); (ii) the frequency shift (w h)^2 / 24 accumulates as w t (w h)^2 / 24
	# (0.028 rad at t = 1 s). Sum times the amplitude: 0.04 * (0.088 + 0.028) = 4.6e-3 m; the bound allows 20 % on top.
	var phase_bound: float = 0.04 * (0.5 * w_n * DT + w_n * 1.0 * pow(w_n * DT, 2.0) / 24.0) * 1.2
	check(worst_cont <= phase_bound, "undamped: within the analytic first-order error of the continuous solution after 1 s (worst %s m, bound %s m)" % [worst_cont, phase_bound])
	check(worst_cont >= 0.5 * phase_bound / 1.2, "and the scheme's known h/2 phase shift is actually present (%s m): the comparison is sensitive" % worst_cont)


func test_b2_undamped_energy_stays_inside_the_discrete_bound_and_damping_dissipates() -> void:
	var k: float = 3000.0
	var rig: Dictionary = _line_pair(k, 0.0, 0.04, false)
	var mu: float = rig.mu
	var e0: float = 0.5 * k * 0.04 * 0.04
	var wh_half: float = 0.5 * sqrt(k / mu) * DT
	var lo: float = INF
	var hi: float = -INF
	for i in 600:
		rig.world.step()
		var x: float = (rig.sb.position - rig.cb.position).length() - rig.l0
		var v: float = rig.n.dot(rig.sb.linear_velocity - rig.cb.linear_velocity)
		var e: float = 0.5 * mu * v * v + 0.5 * k * x * x
		lo = minf(lo, e)
		hi = maxf(hi, e)
	# Symplectic Euler: E0 / (1 + w h / 2) <= E <= E0 / (1 - w h / 2) (see test_front_suspension.gd for the derivation); 1e-4 slack for float32.
	check(lo >= e0 / (1.0 + wh_half) * (1.0 - 1.0e-4), "no loss beyond the discretisation bound (min %s of E0 = %s)" % [lo, e0])
	check(hi <= e0 / (1.0 - wh_half) * (1.0 + 1.0e-4), "no creation beyond the discretisation bound (max %s of E0 = %s)" % [hi, e0])
	check(hi - lo > 0.0, "the energy oscillates inside the bound (it is not constant: the bound is attained from both sides)")
	# With the damper: energy ends far below the start and never exceeds the undamped upper bound.
	var damped: Dictionary = _line_pair(k, 60.0, 0.04, false)
	var e_max: float = 0.0
	for i in 600:
		damped.world.step()
		var x: float = (damped.sb.position - damped.cb.position).length() - damped.l0
		var v: float = damped.n.dot(damped.sb.linear_velocity - damped.cb.linear_velocity)
		e_max = maxf(e_max, 0.5 * mu * v * v + 0.5 * k * x * x)
	var x_end: float = (damped.sb.position - damped.cb.position).length() - damped.l0
	var v_end: float = damped.n.dot(damped.sb.linear_velocity - damped.cb.linear_velocity)
	var e_end: float = 0.5 * mu * v_end * v_end + 0.5 * k * x_end * x_end
	check(e_max <= e0 / (1.0 - wh_half) * (1.0 + 1.0e-4), "damped: the energy never exceeds the start (max %s)" % e_max)
	# zeta w = 0.21 * 21.2 = 4.5 /s: after 5 s the amplitude factor is e^-22 -> energy ~1e-19 E0; float32 floor ~1e-12 J.
	check(e_end <= 1.0e-6 * e0, "damped: after 5 s the shock has come to rest (E = %s J of %s J)" % [e_end, e0])


func test_b3_static_equilibrium_under_gravity_with_a_static_chassis() -> void:
	# Static chassis above, swingarm hanging on a vertical shock: l_eq = l_0 + m g / k (F = -k (l_eq - l_0) must carry m g).
	var k: float = 2000.0
	var m: float = 5.0
	var g: float = 9.81
	var l0: float = 0.5
	var l_eq: float = l0 + m * g / k
	for start in ["at equilibrium", "released from the rest length"]:
		var w: SimWorld = make_world(Vector3(0.0, -g, 0.0))
		var cb: SimRigidBody = make_static(Vector3(0.0, 1.0, 0.0))
		var y0: float = 1.0 - (l_eq if start == "at equilibrium" else l0)
		var sb: SimRigidBody = make_sphere(m, 0.1, Vector3(0.0, y0, 0.0))
		w.add_body(cb)
		w.add_body(sb)
		var susp = RearSuspension.from_world(cb, sb, cb.position, sb.position, k, 80.0, l0)
		w.add_force_provider(susp)
		if start == "at equilibrium":
			susp.apply_forces(DT, w.config)
			check_vec3_near(sb.force_accumulator, Vector3(0.0, m * g, 0.0), 2.0e-3, "at l_eq the shock carries exactly m g upward")
			check(cb.force_accumulator == Vector3.ZERO, "the static chassis receives no force")
			sb.clear_accumulators()
		var worst: float = 0.0
		for i in 1200:
			w.step()
			worst = maxf(worst, absf((cb.position - sb.position).length() - l_eq))
		var final_error: float = absf((cb.position - sb.position).length() - l_eq)
		if start == "at equilibrium":
			# Float32: k * eps(l) ~ 2000 * 6e-8 = 1e-4 N, a/w^2 = 1e-4 / 5 / 400 ~ 1e-8 m of drift; 1e-6 m bound.
			check(worst <= 1.0e-6, "at equilibrium nothing moves (worst deviation %s m)" % worst)
		else:
			# zeta = 80 / (2 sqrt(k m)) = 0.4, w = 20 rad/s: e-folding 0.125 s; after 10 s only the float32 floor is left.
			check(final_error <= 2.0e-6, "released from l_0 the shock settles at l_eq = l_0 + m g / k (%s m, error %s)" % [l_eq, final_error])
			check(worst >= 0.0 and absf((cb.position - sb.position).length() - l0) > 0.0, "and it really moved away from l_0")
		check(cb.position == Vector3(0.0, 1.0, 0.0) and cb.linear_velocity == Vector3.ZERO, "%s: the static chassis never moved" % start)
		check(w.numerical_fault == "", "%s: no fault" % start)


func test_b4_free_fall_of_both_bodies_leaves_the_shock_unloaded() -> void:
	var rig: Dictionary = _line_pair(3000.0, 60.0, 0.0, false, Vector3(0.0, -9.81, 0.0))
	var worst: float = 0.0
	for i in 120:
		rig.world.step()
		worst = maxf(worst, absf((rig.sb.position - rig.cb.position).length() - rig.l0))
		check(rig.susp.axial_force() < 1.0e-2 and rig.susp.axial_force() > -1.0e-2, "no force while both fall together")
	# Both bodies receive the same gravity increments, so the separation is constant up to float32 position rounding after 1 s of fall (~5 m).
	check(worst <= 5.0e-6, "the separation stays l_0 in free fall (worst %s m): the provider adds no hidden force" % worst)
	check(rig.sb.position.y < -3.0, "the pair really fell")


func test_b5_static_and_dynamic_participation() -> void:
	# Spherical swingarm inertia: the world integrates rotation with a midpoint gyroscopic term, so an anisotropic body would add a
	# (w x I w) correction of a few percent after a step that spins it at ~6 rad/s; a sphere has none and h I^-1 tau is then exact.
	var rig: Dictionary = _rig({"static_chassis": true, "k": 2000.0, "c": 50.0, "rest": 0.45, "sphere": true})
	var o: Dictionary = _oracle(rig)
	rig.susp.apply_forces(DT, rig.world.config)
	check(rig.cb.force_accumulator == Vector3.ZERO and rig.cb.torque_accumulator == Vector3.ZERO, "the static chassis gets neither force nor torque")
	check_vec3_near(rig.sb.force_accumulator, o.Fs, _vtol(o.Fs), "the dynamic swingarm gets the FULL force F n (not half)")
	check_vec3_near(rig.sb.torque_accumulator, o.tau_s, _vtol(o.tau_s), "and the lever-arm torque")
	# One step from rest, no gravity, no constraint: v = h F / m and w = h I_world^-1 tau exactly (a hand-computed expectation).
	var w: SimWorld = rig.world
	var sb: SimRigidBody = rig.sb
	var inv_i: Basis = sb.inverse_inertia_world
	rig.cb.clear_accumulators()
	sb.clear_accumulators()
	w.step()
	check_vec3_near(sb.linear_velocity, o.Fs * DT / 6.0, 1.0e-5 + 2.0e-5 * (o.Fs * DT / 6.0).length(), "v = h F / m")
	check_vec3_near(sb.angular_velocity, inv_i * o.tau_s * DT, 1.0e-5 + 2.0e-5 * (inv_i * o.tau_s * DT).length(), "w = h I_world^-1 tau (off-COM attachment spins the body)")
	check(rig.cb.linear_velocity == Vector3.ZERO and rig.cb.angular_velocity == Vector3.ZERO, "the static chassis is exactly unchanged")
	# Two dynamic bodies: each gets its own response; total momentum is zero.
	var both: Dictionary = _rig({"k": 2000.0, "c": 50.0, "rest": 0.45, "sphere": true})
	var ob: Dictionary = _oracle(both)
	both.world.step()
	check_vec3_near(both.cb.linear_velocity, ob.Fc * DT / 30.0, 1.0e-5 + 2.0e-5 * (ob.Fc * DT / 30.0).length(), "chassis: v = h F_c / m_c")
	check_vec3_near(both.sb.linear_velocity, ob.Fs * DT / 6.0, 1.0e-5 + 2.0e-5 * (ob.Fs * DT / 6.0).length(), "swingarm: v = h F_s / m_s")
	check_vec3_near(both.world.total_linear_momentum(), Vector3.ZERO, 1.0e-5, "zero total momentum from rest")


func test_b6_energy_of_an_isolated_pair_with_off_com_attachments() -> void:
	# General geometry (rotating bodies, off-COM seats, spheres so that there is no gyroscopic term). Spring only: K + U is
	# conserved by the continuous system; the first-order scheme keeps it within O(h). Measured below at two step sizes.
	var deviations: Array = []
	for h_divisor in [1, 2, 4]:
		var r: Dictionary = _rig({"k": 1800.0, "c": 0.0, "rest": 0.5, "sphere": true})
		r.cb.linear_velocity = Vector3(0.4, -0.1, 0.2)
		r.sb.linear_velocity = Vector3(-0.3, 0.5, 0.1)
		r.sb.angular_velocity = Vector3(-0.9, 0.4, 0.6)
		var w: SimWorld = r.world
		w.config.fixed_timestep = DT / h_divisor
		var e0: float = w.total_kinetic_energy() + _oracle(r).U
		var worst: float = 0.0
		var steps: int = 480 * h_divisor
		for i in steps:
			w.step()
			worst = maxf(worst, absf(w.total_kinetic_energy() + _oracle(r).U - e0))
		deviations.append(worst / e0)
		check(w.numerical_fault == "", "no fault")
	# Measured (this rig, 4 s): 0.107, 0.062, 0.032 of E0 at h = 1/120, 1/240, 1/480 s: first order in h (ratios 0.58 and 0.52, tending to
	# 0.5 as w h shrinks; the threshold 0.65 leaves margin above the pre-asymptotic 0.58). The
	# shock is stiff in rotation (the swingarm is light: effective w ~ 70 rad/s, w h ~ 0.6), hence the large absolute value at 1/120 s;
	# this is the documented time-step limitation of the explicit force, not a defect. Bounds: 0.15 (measured x 1.4) and the scaling.
	check(deviations[0] <= 0.15, "energy deviation over 4 s at h = 1/120 s is %s of E0 (bound 0.15)" % deviations[0])
	check(deviations[1] <= 0.65 * deviations[0] and deviations[2] <= 0.65 * deviations[1], "halving the step roughly halves the deviation (%s -> %s -> %s): the error is the first-order discretisation, not a hidden source" % deviations)
	# With damping the pair may only lose energy beyond that discretisation error.
	var rd: Dictionary = _rig({"k": 1800.0, "c": 60.0, "rest": 0.5, "sphere": true})
	rd.cb.linear_velocity = Vector3(0.4, -0.1, 0.2)
	rd.sb.linear_velocity = Vector3(-0.3, 0.5, 0.1)
	rd.sb.angular_velocity = Vector3(-0.9, 0.4, 0.6)
	# The centre-of-mass kinetic energy P^2 / (2 M) is conserved by any internal force (it is not dissipated), so the decay is judged
	# on the internal energy E - P^2 / (2 M). Measured: 6.33 J -> 2.52 J after 4 s.
	var kcm: float = rd.world.total_linear_momentum().length_squared() / (2.0 * 36.0)
	var e_start: float = rd.world.total_kinetic_energy() + _oracle(rd).U
	var e_peak: float = e_start
	for i in 480:
		rd.world.step()
		e_peak = maxf(e_peak, rd.world.total_kinetic_energy() + _oracle(rd).U)
	var e_final: float = rd.world.total_kinetic_energy() + _oracle(rd).U
	check(e_peak <= e_start * (1.0 + deviations[0] + 1.0e-3), "damped: the energy never exceeds the start beyond the discretisation error")
	check(e_final - kcm < 0.5 * (e_start - kcm), "damped: the internal energy decayed by more than half (%s of %s J)" % [e_final - kcm, e_start - kcm])


func test_b7_sustained_excitation_stays_finite_and_bounded() -> void:
	# Two dynamic bodies in gravity, a sinusoidal external force on the swingarm (near the shock's resonance) for 30 s.
	var r: Dictionary = _rig({"k": 2500.0, "c": 120.0, "rest": 0.5, "gravity": Vector3(0.0, -9.81, 0.0)})
	var w: SimWorld = r.world
	var worst_l: float = 0.0
	var best_l: float = INF
	var worst_rel_v: float = 0.0
	var worst_w: float = 0.0
	var finite: bool = true
	for i in 3600:
		r.sb.apply_force(Vector3(0.0, 200.0 * sin(2.0 * PI * 3.0 * i * DT), 60.0 * cos(2.0 * PI * 3.0 * i * DT)))
		w.step()
		var l: float = r.susp.length()
		worst_l = maxf(worst_l, l)
		best_l = minf(best_l, l)
		worst_rel_v = maxf(worst_rel_v, (r.sb.linear_velocity - r.cb.linear_velocity).length())
		worst_w = maxf(worst_w, maxf(r.sb.angular_velocity.length(), r.cb.angular_velocity.length()))
		finite = finite and is_finite(l) and r.sb.position.is_finite() and r.cb.position.is_finite() \
			and r.sb.linear_velocity.is_finite() and r.sb.angular_velocity.is_finite() and r.cb.angular_velocity.is_finite()
	check(finite, "all state stays finite")
	check(w.numerical_fault == "", "no numerical fault")
	check(best_l > 0.1 and worst_l < 2.0, "the shock length stays physical (%s .. %s m)" % [best_l, worst_l])
	# There is no ground, so the pair free-falls together (about 4.4 km in 30 s): only RELATIVE motion and rotation are meaningful.
	# Measured: 5.3 m/s relative and 43 rad/s peak (a 6 kg swingarm driven by 200 N); the bounds 20 m/s and 100 rad/s only exclude runaway.
	check(worst_rel_v < 20.0 and worst_w < 100.0, "relative velocity and rotation stay bounded (%s m/s, %s rad/s)" % [worst_rel_v, worst_w])


# =====================================================================================================================
# C. Validation, degenerate geometry and provider lifecycle
# =====================================================================================================================

func test_c1_invalid_parameters_are_rejected_safely() -> void:
	var w: SimWorld = make_world()
	var cb: SimRigidBody = make_sphere(10.0, 0.2, Vector3(0.0, 1.0, 0.0))
	var sb: SimRigidBody = make_sphere(5.0, 0.2, Vector3(0.0, 0.5, 0.4))
	w.add_body(cb)
	w.add_body(sb)
	var pc: Vector3 = Vector3(0.0, 1.0, 0.1)
	var ps: Vector3 = Vector3(0.0, 0.5, 0.35)
	var good = RearSuspension.from_world(cb, sb, pc, ps, 1000.0, 10.0, 0.5)
	check(good != null and good.is_valid(), "the baseline is valid")
	var nan: float = NAN
	var inf: float = INF
	var cases: Array = [
		["null chassis", null, sb, pc, ps, 1000.0, 10.0, 0.5],
		["null swingarm", cb, null, pc, ps, 1000.0, 10.0, 0.5],
		["same body", cb, cb, pc, ps, 1000.0, 10.0, 0.5],
		["NaN chassis attachment", cb, sb, Vector3(nan, 1.0, 0.0), ps, 1000.0, 10.0, 0.5],
		["infinite swingarm attachment", cb, sb, pc, Vector3(inf, 0.0, 0.0), 1000.0, 10.0, 0.5],
		["negative stiffness", cb, sb, pc, ps, -1.0, 10.0, 0.5],
		["NaN stiffness", cb, sb, pc, ps, nan, 10.0, 0.5],
		["infinite stiffness", cb, sb, pc, ps, inf, 10.0, 0.5],
		["negative damping", cb, sb, pc, ps, 1000.0, -0.1, 0.5],
		["NaN damping", cb, sb, pc, ps, 1000.0, nan, 0.5],
		["infinite damping", cb, sb, pc, ps, 1000.0, inf, 0.5],
		["zero rest length", cb, sb, pc, ps, 1000.0, 10.0, 0.0],
		["negative rest length", cb, sb, pc, ps, 1000.0, 10.0, -0.3],
		["NaN rest length", cb, sb, pc, ps, 1000.0, 10.0, nan],
		["infinite rest length", cb, sb, pc, ps, 1000.0, 10.0, inf],
		["coincident attachments", cb, sb, pc, pc, 1000.0, 10.0, 0.5],
		["attachments 0.5 mm apart", cb, sb, pc, pc + Vector3(0.0, 0.0005, 0.0), 1000.0, 10.0, 0.5],
	]
	for case in cases:
		var before: Array = [cb.position, sb.position, cb.force_accumulator, sb.force_accumulator, w.force_providers.size()]
		var s = RearSuspension.from_world(case[1], case[2], case[3], case[4], case[5], case[6], case[7])
		check(s == null, "%s: from_world refuses" % case[0])
		check(before == [cb.position, sb.position, cb.force_accumulator, sb.force_accumulator, w.force_providers.size()], "%s: nothing was touched" % case[0])
	# Both bodies static: nothing to act on.
	var s1: SimRigidBody = make_static(Vector3(0.0, 1.0, 0.0))
	var s2: SimRigidBody = make_static(Vector3(0.0, 0.0, 0.4))
	w.add_body(s1)
	w.add_body(s2)
	check(RearSuspension.from_world(s1, s2, Vector3(0.0, 1.0, 0.1), Vector3(0.0, 0.0, 0.35), 1000.0, 10.0, 0.5) == null, "two static bodies: refused")
	# Direct construction with invalid data is reported by validate() and refused by the world.
	var direct = RearSuspension.new(cb, sb, Vector3.ZERO, Vector3(0.0, -0.5, 0.4), -5.0, nan, 0.0)
	var errors: PackedStringArray = direct.validate()
	check(errors.size() == 3, "stiffness, damping and rest length each reported once (%d messages: %s)" % [errors.size(), "; ".join(errors)])
	check(w.add_force_provider(direct) == null and w.force_providers.is_empty(), "the world refuses to register it")
	# The shortest accepted separation works and produces finite forces.
	var short = RearSuspension.from_world(cb, sb, pc, pc + Vector3(0.0, 0.002, 0.0), 1000.0, 10.0, 0.5)
	check(short != null, "a 2 mm separation (> MIN_LENGTH) is accepted")
	if short != null:
		short.apply_forces(DT, w.config)
		check(cb.force_accumulator.is_finite() and sb.force_accumulator.is_finite(), "and its forces are finite")


func test_c2_degenerate_geometry_never_yields_a_non_finite_direction_or_force() -> void:
	var r: Dictionary = _rig({"k": 2000.0, "c": 50.0, "rest": 0.5})
	var s = r.susp
	# Collapse the shock: move the swingarm so that the two attachment points coincide exactly.
	r.sb.position += _oracle(r).pc - _oracle(r).ps
	r.sb.reset_published_state()
	check(s.length() < 1.0e-5, "the attachments coincide (%s m)" % s.length())
	check(s.direction() == Vector3.ZERO, "direction() is zero where it is undefined (never NaN)")
	check(s.direction().is_finite() and is_finite(s.relative_extension_rate()) and is_finite(s.axial_force()) and is_finite(s.spring_energy()) and is_finite(s.damping_power()),
		"every query is finite")
	check(not s.has_direction(), "has_direction() is false")
	var errors: PackedStringArray = s.validate()
	check(errors.size() == 1 and "undefined" in errors[0], "validate() reports the collapse once (%s)" % "; ".join(errors))
	s.apply_forces(DT, r.world.config)
	check(r.sb.force_accumulator == Vector3.ZERO and r.cb.force_accumulator == Vector3.ZERO, "apply_forces() applies nothing (no non-finite force)")
	# The world refuses to step while the geometry is degenerate: nothing changes, nothing is repaired.
	var pos_s: Vector3 = r.sb.position
	var vel_s: Vector3 = r.sb.linear_velocity
	check(not r.world.step(), "step() is refused")
	check(r.sb.position == pos_s and r.sb.linear_velocity == vel_s, "state is exactly unchanged by the refused step")
	check("rear" in r.world.last_error and "apart" in r.world.last_error, "the refusal names the cause (%s)" % r.world.last_error)
	# Just below and just above the threshold.
	for sep in [0.0009, 0.0011]:
		var r2: Dictionary = _rig({"k": 2000.0, "c": 50.0, "rest": 0.5})
		var o: Dictionary = _oracle(r2)
		r2.sb.position += (o.pc + o.n * sep) - o.ps
		check_near(r2.susp.length(), sep, 2.0e-6, "separation %s m realised" % sep)
		check(r2.susp.is_valid() == (sep >= RearSuspension.MIN_LENGTH), "separation %s m: valid iff >= MIN_LENGTH" % sep)
	# Non-finite pose (corrupt body): the component does not manufacture a NaN force; the world's own pose check owns the fault.
	var r3: Dictionary = _rig({"k": 2000.0, "c": 50.0, "rest": 0.5})
	r3.sb.position = Vector3(NAN, 0.0, 0.0)
	r3.susp.apply_forces(DT, r3.world.config)
	check(r3.sb.force_accumulator == Vector3.ZERO and r3.cb.force_accumulator == Vector3.ZERO, "a non-finite pose yields no force from the component")


func test_c3_invalid_or_mutated_providers_are_refused_by_the_preflight_rules() -> void:
	# Registration of a foreign / null / unregistered-body provider.
	var r: Dictionary = _rig({"register": false})
	var w: SimWorld = r.world
	var other: SimWorld = make_world()
	check(other.add_force_provider(r.susp) == null and other.force_providers.is_empty(), "a provider of bodies that belong to another world is refused")
	check(w.add_force_provider(null) == null, "null is refused")
	var stray: SimRigidBody = make_sphere(3.0, 0.1, Vector3(0.0, 0.3, 0.3))
	var with_stray = RearSuspension.from_world(r.cb, stray, P_C, Vector3(0.0, 0.4, 0.4), 1000.0, 5.0, 0.5)
	check(with_stray != null and w.add_force_provider(with_stray) == null, "a provider involving an unregistered body is refused")
	check(w.add_force_provider(r.susp) != null and w.preflight(), "the valid provider registers and preflight passes")
	var before: Vector3 = r.sb.position
	# Each mutation after registration must make the world refuse to step, with the state exactly unchanged, and recover when undone.
	var mutations: Array = [
		["negative stiffness", func(s): s.stiffness = -1.0, func(s): s.stiffness = 2000.0],
		["NaN damping", func(s): s.damping = NAN, func(s): s.damping = 50.0],
		["zero rest length", func(s): s.rest_length = 0.0, func(s): s.rest_length = ref_length()],
		["infinite local attachment", func(s): s.local_attachment_swingarm = Vector3(INF, 0.0, 0.0), func(s): s.local_attachment_swingarm = r.ls],
		["null swingarm", func(s): s.swingarm_body = null, func(s): s.swingarm_body = r.sb],
	]
	for m in mutations:
		m[1].call(r.susp)
		check(not r.world.preflight(), "%s: preflight refuses" % m[0])
		check(not w.step() and r.sb.position == before, "%s: step refused, state untouched" % m[0])
		m[2].call(r.susp)
		check(r.world.preflight(), "%s: undone, preflight passes again" % m[0])
	check(w.step(), "stepping resumes")
	# A body erased behind the world's back.
	w.bodies.erase(r.sb)
	check(not w.preflight() and "membership" in w.last_error, "a body erased behind the world's back is caught (%s)" % w.last_error)
	w.bodies.append(r.sb)
	check(w.preflight(), "restored")
	# remove_body() drops the provider of that body.
	w.remove_body(r.sb)
	check(w.force_providers.is_empty(), "remove_body() drops the providers involving that body")


class CountingRear extends "res://Simulation/Motorcycle/rear_suspension.gd":
	var calls: int = 0
	var last_dt: float = 0.0

	func _init(a, b, c, d, e, f, g) -> void:
		super(a, b, c, d, e, f, g)

	func apply_forces(dt: float, config) -> void:
		calls += 1
		last_dt = dt
		super(dt, config)


func test_c4_evaluated_exactly_once_per_fixed_step_never_per_frame() -> void:
	var w: SimWorld = make_world()
	var cb: SimRigidBody = make_sphere(30.0, 0.3, Vector3(0.0, 1.0, 0.0))
	var sb: SimRigidBody = make_sphere(6.0, 0.15, Vector3(0.0, 0.5, 0.4))
	w.add_body(cb)
	w.add_body(sb)
	var s = CountingRear.new(cb, sb, Vector3.ZERO, Vector3.ZERO, 1500.0, 30.0, 0.6)
	check(w.add_force_provider(s) != null, "registered")
	var steps: int = w.advance(0.05)  # 0.05 s at 1/120 s = 6 fixed steps in ONE frame
	check(steps == 6 and s.calls == 6, "6 fixed steps in a 0.05 s frame -> 6 evaluations, not 1 (steps %d, calls %d)" % [steps, s.calls])
	check_near(s.last_dt, w.config.fixed_timestep, 1.0e-12, "called with the fixed step")
	s.enabled = false
	w.advance(0.05)
	check(s.calls == 6, "a disabled provider is not evaluated")
	s.enabled = true
	w.advance(DT * 0.5)
	check(s.calls == 6, "a partial step (no fixed step completed) does not evaluate it")
	w.advance(DT * 0.5)
	check(s.calls == 7, "the completed step does")
	w.remove_force_provider(s)
	w.advance(0.05)
	check(s.calls == 7, "a removed provider is not evaluated")
	# The force is re-evaluated from the CURRENT state at every one of the six steps of a frame: compare one frame of 6 steps with six
	# one-step frames (bit-identical), which fails if the force were applied once per frame.
	var a: Dictionary = _rig({"k": 2000.0, "c": 40.0, "rest": 0.5})
	var b: Dictionary = _rig({"k": 2000.0, "c": 40.0, "rest": 0.5})
	a.world.advance(6 * DT + 1.0e-9)
	for i in 6:
		b.world.advance(DT + 1.0e-9)
	check(a.sb.position == b.sb.position and a.sb.linear_velocity == b.sb.linear_velocity and a.cb.angular_velocity == b.cb.angular_velocity,
		"one 6-step frame equals six 1-step frames exactly")
	check(a.sb.linear_velocity.length() > 1.0e-3, "and the shock acted")


# =====================================================================================================================
# E. Regression sensitivity (in-suite mutants of the REAL component)
# =====================================================================================================================
# Each mutant subclasses the real RearSuspension and breaks exactly one thing. One battery of physical criteria, with every
# expectation from the independent oracle above, must accept the real component (and an unmodified subclass: the control) and
# reject every mutant. Defects covered (the complete list of what was actually tested): reversed spring direction, reversed damping
# sign, missing rotational contribution to the attachment-point velocity, force on one body only, force at the COM instead of the
# attachment point, force direction fixed in the world frame, force direction fixed in the chassis frame (turns with the chassis
# but not with the swingarm), wrong rest-length handling (reference length used instead of the configured rest length).

class Unmodified extends "res://Simulation/Motorcycle/rear_suspension.gd":
	func _init(a, b, c, d, e, f, g) -> void:
		super(a, b, c, d, e, f, g)


class ReversedSpring extends "res://Simulation/Motorcycle/rear_suspension.gd":
	func _init(a, b, c, d, e, f, g) -> void:
		super(a, b, c, d, e, f, g)

	func spring_force() -> float:
		return -super()


class ReversedDamping extends "res://Simulation/Motorcycle/rear_suspension.gd":
	func _init(a, b, c, d, e, f, g) -> void:
		super(a, b, c, d, e, f, g)

	func damping_force() -> float:
		return -super()


class NoRotation extends "res://Simulation/Motorcycle/rear_suspension.gd":
	func _init(a, b, c, d, e, f, g) -> void:
		super(a, b, c, d, e, f, g)

	func relative_extension_rate() -> float:
		return direction().dot(swingarm_body.linear_velocity - chassis_body.linear_velocity)


class OneBodyOnly extends "res://Simulation/Motorcycle/rear_suspension.gd":
	func _init(a, b, c, d, e, f, g) -> void:
		super(a, b, c, d, e, f, g)

	func apply_forces(_dt: float, _config) -> void:
		swingarm_body.apply_force_at_point(direction() * axial_force(), attachment_swingarm_world())


class AtCenterOfMass extends "res://Simulation/Motorcycle/rear_suspension.gd":
	func _init(a, b, c, d, e, f, g) -> void:
		super(a, b, c, d, e, f, g)

	func apply_forces(_dt: float, _config) -> void:
		var force: Vector3 = direction() * axial_force()
		swingarm_body.apply_force(force)
		chassis_body.apply_force(-force)


class WorldFixedDirection extends "res://Simulation/Motorcycle/rear_suspension.gd":
	var frozen: Vector3 = Vector3.ZERO

	func _init(a, b, c, d, e, f, g) -> void:
		super(a, b, c, d, e, f, g)

	func direction() -> Vector3:
		return frozen


class ChassisFixedDirection extends "res://Simulation/Motorcycle/rear_suspension.gd":
	var local_frozen: Vector3 = Vector3.ZERO

	func _init(a, b, c, d, e, f, g) -> void:
		super(a, b, c, d, e, f, g)

	func direction() -> Vector3:
		return chassis_body.orientation * local_frozen


class WrongRest extends "res://Simulation/Motorcycle/rear_suspension.gd":
	var reference: float = 0.0

	func _init(a, b, c, d, e, f, g) -> void:
		super(a, b, c, d, e, f, g)

	func extension() -> float:
		return length() - reference


## Builds the rig with a provider produced by `factory`, then rotates the swingarm (so that a line fixed at construction is wrong),
## gives both bodies translation AND rotation, and returns the names of every violated criterion.
func _battery(kind: String) -> PackedStringArray:
	var failed: PackedStringArray = PackedStringArray()
	var w: SimWorld = make_world()
	var qc: Quaternion = Quaternion(Vector3(0.3, 1.0, 0.2).normalized(), 0.4)
	var qs: Quaternion = Quaternion(Vector3(1.0, -0.4, 0.5).normalized(), 0.3)
	var cb: SimRigidBody = make_body(30.0, SimMassProperties.box_inertia(30.0, Vector3(0.3, 0.5, 1.0)), Vector3(0.1, 0.9, 0.05), qc)
	var sb: SimRigidBody = make_body(6.0, SimMassProperties.box_inertia(6.0, Vector3(0.2, 0.1, 0.6)), Vector3(0.0, 0.45, 0.5), qs)
	w.add_body(cb)
	w.add_body(sb)
	var k: float = 2000.0
	var c: float = 50.0
	var l0: float = 0.47   # != the reference length 0.5679
	var args: Array = [cb, sb, Basis(qc).inverse() * (P_C - cb.position), Basis(qs).inverse() * (P_S - sb.position), k, c, l0]
	var s = null
	match kind:
		"real": s = RearSuspension.new(cb, sb, args[2], args[3], k, c, l0)
		"control": s = Unmodified.new(cb, sb, args[2], args[3], k, c, l0)
		"reversed_spring": s = ReversedSpring.new(cb, sb, args[2], args[3], k, c, l0)
		"reversed_damping": s = ReversedDamping.new(cb, sb, args[2], args[3], k, c, l0)
		"no_rotation": s = NoRotation.new(cb, sb, args[2], args[3], k, c, l0)
		"one_body_only": s = OneBodyOnly.new(cb, sb, args[2], args[3], k, c, l0)
		"at_com": s = AtCenterOfMass.new(cb, sb, args[2], args[3], k, c, l0)
		"world_fixed_direction":
			s = WorldFixedDirection.new(cb, sb, args[2], args[3], k, c, l0)
			s.frozen = (P_S - P_C).normalized()
		"chassis_fixed_direction":
			s = ChassisFixedDirection.new(cb, sb, args[2], args[3], k, c, l0)
			s.local_frozen = Basis(qc).inverse() * (P_S - P_C).normalized()
		"wrong_rest":
			s = WrongRest.new(cb, sb, args[2], args[3], k, c, l0)
			s.reference = (P_S - P_C).length()
	# Move away from the construction pose: the swingarm turns (the line changes), both bodies move and spin.
	sb.orientation = Quaternion(Vector3.RIGHT, 0.45) * sb.orientation
	cb.orientation = Quaternion(Vector3.FORWARD, 0.25) * cb.orientation
	cb.linear_velocity = Vector3(0.3, -0.2, 0.5)
	sb.linear_velocity = Vector3(-0.4, 0.6, 0.1)
	cb.angular_velocity = Vector3(0.2, 0.5, -0.3)
	sb.angular_velocity = Vector3(-0.9, 0.4, 0.6)
	var r: Dictionary = {"cb": cb, "sb": sb, "lc": args[2], "ls": args[3], "k": k, "c": c, "l0": l0}
	var o: Dictionary = _oracle(r)
	s.apply_forces(DT, w.config)
	var ftol: float = _vtol(o.Fs)
	if (sb.force_accumulator - o.Fs).length() > ftol:
		failed.append("swingarm force")
	if (cb.force_accumulator - o.Fc).length() > ftol:
		failed.append("chassis force")
	if (sb.torque_accumulator - o.tau_s).length() > _vtol(o.tau_s):
		failed.append("swingarm torque")
	if (cb.torque_accumulator - o.tau_c).length() > _vtol(o.tau_c):
		failed.append("chassis torque")
	if (cb.force_accumulator + sb.force_accumulator).length() > 1.0e-3:
		failed.append("net force")
	if absf(s.relative_extension_rate() - o.ldot) > 2.0e-5:
		failed.append("relative extension rate")
	if absf(s.spring_force() - o.f_spring) > _tol(o.f_spring):
		failed.append("spring force")
	if absf(s.damping_force() - o.f_damp) > _tol(o.f_damp):
		failed.append("damping force")
	if sb.force_accumulator.length() > 1.0e-3 and (sb.force_accumulator.normalized().dot(o.n) * signf(o.F)) < 1.0 - 1.0e-5:
		failed.append("direction along the instantaneous line")
	return failed


func test_e1_battery_accepts_the_real_component_and_the_unmodified_control() -> void:
	var real: PackedStringArray = _battery("real")
	check(real.is_empty(), "the real component passes every criterion (failed: %s)" % "; ".join(real))
	var control: PackedStringArray = _battery("control")
	check(control.is_empty(), "an unmodified subclass (control) passes every criterion (failed: %s)" % "; ".join(control))


func test_e2_battery_rejects_every_listed_defect() -> void:
	var mutants: Array = ["reversed_spring", "reversed_damping", "no_rotation", "one_body_only", "at_com", "world_fixed_direction",
		"chassis_fixed_direction", "wrong_rest"]
	for m in mutants:
		var failed: PackedStringArray = _battery(m)
		check(not failed.is_empty(), "mutant '%s' is rejected (criteria failed: %s)" % [m, "; ".join(failed)])
