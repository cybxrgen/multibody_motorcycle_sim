extends "res://Tests/sim_test_case.gd"
## M2.2 front suspension spring-damper (Simulation/Motorcycle/front_suspension.gd) and the generic per-step force-provider hook it
## uses (Simulation/Core/sim_force_provider.gd + SimWorld).
##
## D spring-damper physics (analytic cases, energy, torques, validation, oscillation vs a discrete and a continuous reference)
## G generic force-provider lifecycle in SimWorld (once per FIXED step, membership, fault containment)
##
## Conventions under test (PHYSICS_CONVENTIONS.md, "Front suspension spring-damper"): u = fork axis from body A (steering assembly)
## toward body B (slider); x = (p_b - p_a).u - l0 (positive = extension); v = u.(v(p_b) - v(p_a)); F = -k (x - x_rest) - c v is the
## scalar force on B along +u and on A along -u, applied at the seats.
##
## Methodology / tolerances:
## * Static quantities (force, torque, power) come from float32 poses and velocities of order 1: 1e-5 relative (>= 1e-6 policy).
## * Dynamics are compared with the EXACT discrete scheme the world implements (force from the start-of-step state, semi-implicit
##   Euler), computed in float64, and only separately with the continuous solution, where the tolerance is the analytic O(omega dt)
##   error of that scheme: the frequency of symplectic Euler is shifted by (omega dt)^2/24, and the explicit damping adds O(zeta
##   omega dt) per period (derivations below at the use).
## * Energy: for symplectic Euler the quantity I = v^2 + w^2 x^2 - w^2 h x v is conserved exactly (verified to 4e-15 in float64);
##   with |w^2 h x v| <= (w h / 2)(v^2 + w^2 x^2) the true energy E obeys E0 / (1 + w h / 2) <= E <= E0 / (1 - w h / 2) for a release
##   from rest. That is the bound asserted (no secular growth, no creation beyond the bounded discretisation oscillation).

const Rig = preload("res://Tests/fork_rig.gd")
const FrontSuspension = preload("res://Simulation/Motorcycle/front_suspension.gd")
const PrismaticJoint = preload("res://Simulation/Solver/prismatic_joint_constraint.gd")
const SimForceProvider = preload("res://Simulation/Core/sim_force_provider.gd")

const DT: float = 1.0 / 120.0


## Rig + suspension registered in the world. Seats lie ON the joint axis line (0.1 m before and 0.2 m after the joint point), so
## l0 = 0.3 m. Returns the rig dictionary with "susp".
func _rig(options: Dictionary, k: float, c: float, rest: float = 0.0) -> Dictionary:
	var r: Dictionary = Rig.make(options)
	var u: Vector3 = r.u
	var s = FrontSuspension.from_world(r.joint, r.point - u * 0.1, r.point + u * 0.2, k, c, rest)
	check(s != null, "the suspension builds")
	if s != null:
		r.world.add_force_provider(s)
	r["susp"] = s
	return r


func _displace(r: Dictionary, x: float) -> void:
	r.b.position += r.u * x
	r.b.reset_published_state()


# --- Subclasses (mutants and probes) ---
class Counting extends "res://Simulation/Core/sim_force_provider.gd":
	var body = null
	var calls: int = 0
	var last_dt: float = 0.0
	var force: Vector3 = Vector3.ZERO
	func involved_bodies() -> Array:
		return [body]
	func apply_forces(dt, _config) -> void:
		calls += 1
		last_dt = dt
		body.apply_force(force)


# =====================================================================================================================
# D. Spring-damper physics
# =====================================================================================================================

func test_d1_spring_force_vanishes_at_the_rest_displacement() -> void:
	var r: Dictionary = _rig({}, 1000.0, 0.0, 0.02)
	var s = r.susp
	check_near(s.reference_separation, 0.3, 1.0e-6, "l0 = 0.3 m (seat separation along the axis at the reference pose)")
	check_near(s.displacement(), 0.0, 2.0e-6, "x = 0 at the reference pose")
	check_near(s.spring_force(), 1000.0 * 0.02, 1.0e-2, "at x = 0 with x_rest = 0.02 the spring pushes toward extension: F = +k x_rest")
	_displace(r, 0.02)
	check_near(s.displacement(), 0.02, 2.0e-6, "x follows the slider displacement along the axis")
	check_near(s.displacement_from_rest(), 0.0, 2.0e-6, "x - x_rest = 0")
	check_near(s.spring_force(), 0.0, 2.0e-2, "the spring force is zero at the rest displacement")
	check_near(s.spring_energy(), 0.0, 1.0e-3, "no stored energy at the rest displacement")
	r.world.step()
	check(r.b.force_accumulator == Vector3.ZERO, "stepping consumed the accumulators")


func test_d2_compression_and_extension_give_opposite_restoring_forces() -> void:
	var k: float = 1000.0
	for x in [0.03, -0.03, 0.08, -0.08]:
		var r: Dictionary = _rig({"on_axis": true}, k, 0.0)
		_displace(r, x)
		var s = r.susp
		var f_expected: float = -k * x
		check_near(s.axial_force(), f_expected, 1.0e-2, "x = %s: F = -k x = %s N" % [x, f_expected])
		s.apply_forces(DT, r.world.config)
		check_vec3_near(r.b.force_accumulator, r.u * f_expected, 1.0e-2, "x = %s: the force on B is F u" % x)
		# Restoring: it points back toward the reference (against the displacement along u).
		check(r.b.force_accumulator.dot(r.u) * x < 0.0, "x = %s: the force on B opposes the displacement (restoring)" % x)
		check(r.a.force_accumulator.dot(r.u) * x > 0.0, "x = %s: the reaction on A is the other way" % x)
		check_near(s.spring_energy(), 0.5 * k * x * x, 1.0e-3, "x = %s: stored energy k x^2 / 2" % x)


func test_d3_damping_opposes_positive_and_negative_axial_velocity() -> void:
	var c: float = 40.0
	for v in [0.7, -0.7, 2.0, -0.05]:
		var r: Dictionary = _rig({"on_axis": true}, 0.0, c)
		r.b.linear_velocity = r.u * v
		var s = r.susp
		check_near(s.relative_velocity(), v, 1.0e-6, "v = %s: relative axial velocity of the seats" % v)
		check_near(s.damping_force(), -c * v, 1.0e-4, "v = %s: F_damping = -c v" % v)
		check(s.damping_force() * v < 0.0, "v = %s: the damping force opposes the motion" % v)
		check_near(s.spring_force(), 0.0, 0.0, "v = %s: no spring force with zero stiffness" % v)
		s.apply_forces(DT, r.world.config)
		check(r.b.force_accumulator.dot(r.u) * v < 0.0, "v = %s: the force on B opposes the motion of B" % v)
	# Moving A instead of B: the relative velocity is what counts.
	var r2: Dictionary = _rig({"on_axis": true}, 0.0, c)
	r2.a.linear_velocity = -r2.u * 0.5
	check_near(r2.susp.relative_velocity(), 0.5, 1.0e-6, "A moving away along -u is a positive (extension) relative velocity")
	check_near(r2.susp.damping_force(), -c * 0.5, 1.0e-4, "and is damped")
	# Equal velocities of both bodies: no relative velocity, no damping.
	var r3: Dictionary = _rig({"on_axis": true}, 0.0, c)
	r3.a.linear_velocity = r3.u * 3.0
	r3.b.linear_velocity = r3.u * 3.0
	check_near(r3.susp.damping_force(), 0.0, 1.0e-6, "a common velocity is not damped")


func test_d4_zero_damping_and_zero_stiffness_produce_no_force() -> void:
	var r: Dictionary = _rig({"on_axis": true}, 1000.0, 0.0)
	_displace(r, 0.04)
	r.b.linear_velocity = r.u * 1.5
	check(r.susp.damping_force() == 0.0 or absf(r.susp.damping_force()) == 0.0, "c = 0: no damping force (even at 1.5 m/s)")
	check_near(r.susp.axial_force(), -1000.0 * 0.04, 1.0e-2, "c = 0: the total force is the spring force only")
	var r2: Dictionary = _rig({"on_axis": true}, 0.0, 40.0)
	_displace(r2, 0.04)
	check(absf(r2.susp.spring_force()) == 0.0, "k = 0: no spring force at 4 cm displacement")
	check_near(r2.susp.axial_force(), 0.0, 1.0e-6, "k = 0, v = 0: the total force is zero")
	var r3: Dictionary = _rig({"on_axis": true}, 0.0, 0.0)
	_displace(r3, 0.04)
	r3.b.linear_velocity = r3.u * 1.0
	r3.susp.apply_forces(DT, r3.world.config)
	check(r3.b.force_accumulator == Vector3.ZERO and r3.a.force_accumulator == Vector3.ZERO, "k = 0 and c = 0: nothing is applied to either body")


func test_d5_forces_on_the_two_bodies_are_equal_and_opposite() -> void:
	var r: Dictionary = _rig({}, 1500.0, 60.0)
	_displace(r, -0.025)
	r.b.linear_velocity = Vector3(0.3, -0.2, 0.4)
	r.a.linear_velocity = Vector3(-0.1, 0.5, 0.2)
	r.b.angular_velocity = Vector3(0.7, 0.1, -0.4)
	r.susp.apply_forces(DT, r.world.config)
	check(r.a.force_accumulator.length() > 1.0, "a non-trivial force is applied (%s N)" % r.a.force_accumulator.length())
	check_vec3_near(r.a.force_accumulator + r.b.force_accumulator, Vector3.ZERO, 1.0e-4, "the two forces sum to zero")
	var along: float = r.b.force_accumulator.dot(r.u)
	check_near(along, r.susp.axial_force(), 1.0e-3, "the force on B along u equals the scalar force F")
	check_near(r.a.force_accumulator.dot(r.u), -r.susp.axial_force(), 1.0e-3, "the force on A along u equals -F")


func test_d6_forces_at_the_seats_produce_the_expected_torques_about_each_com() -> void:
	var r: Dictionary = _rig({}, 1500.0, 60.0)
	_displace(r, 0.035)
	var s = r.susp
	var p_a: Vector3 = r.point - r.u * 0.1 + Vector3.ZERO
	var p_b: Vector3 = r.point + r.u * 0.2 + r.u * 0.035   # seat B moved with the slider
	var f: float = s.axial_force()
	s.apply_forces(DT, r.world.config)
	# Independent expectation from the configured geometry: tau = (p - x_com) x F.
	var fb: Vector3 = r.u * f
	check_vec3_near(r.b.torque_accumulator, (p_b - r.b.position).cross(fb), 1.0e-3, "torque on B = (p_b - x_b) x F u")
	check_vec3_near(r.a.torque_accumulator, (p_a - r.a.position).cross(-fb), 1.0e-3, "torque on A = (p_a - x_a) x (-F u)")
	check(r.b.torque_accumulator.length() > 0.5 and r.a.torque_accumulator.length() > 0.5, "the COMs are off the line of action, so real torques arise")
	# Central pair: the seats are collinear along u, so the pair adds no net torque about ANY point (angular momentum safe).
	var origin_torque: Vector3 = r.a.torque_accumulator + r.b.torque_accumulator + r.a.position.cross(r.a.force_accumulator) + r.b.position.cross(r.b.force_accumulator)
	check_vec3_near(origin_torque, Vector3.ZERO, 2.0e-3, "net torque about the origin is zero (collinear equal and opposite forces)")
	# The applied force has no component perpendicular to the axis.
	check(r.b.force_accumulator.cross(r.u).length() <= 1.0e-5 * maxf(1.0, r.b.force_accumulator.length()), "no force perpendicular to the fork axis on B")
	check(r.a.force_accumulator.cross(r.u).length() <= 1.0e-5 * maxf(1.0, r.a.force_accumulator.length()), "no force perpendicular to the fork axis on A")


func test_d7_damping_power_is_never_positive() -> void:
	var c: float = 55.0
	var states: Array = [
		[Vector3(0.3, 0.1, -0.2), Vector3(-0.4, 0.2, 0.5)],
		[Vector3(1.0, -1.0, 2.0), Vector3(1.0, -1.0, 2.0)],
		[Vector3(-0.2, 0.9, 0.0), Vector3(0.6, -0.3, 0.1)],
		[Vector3.ZERO, Vector3(0.0, 0.0, 0.7)],
	]
	for st in states:
		var r: Dictionary = _rig({}, 0.0, c)
		r.a.linear_velocity = st[0]
		r.b.linear_velocity = st[1]
		r.b.angular_velocity = Vector3(0.4, -0.3, 0.2)
		var s = r.susp
		check(s.damping_power() <= 0.0, "damping power is non-positive (%s W)" % s.damping_power())
		# Independent power of the applied pair: F_b . v(p_b) + F_a . v(p_a) with c only (k = 0), at the seats.
		s.apply_forces(DT, r.world.config)
		var power: float = r.b.force_accumulator.dot(r.b.velocity_at_point(s.attachment_b_world())) + r.a.force_accumulator.dot(r.a.velocity_at_point(s.attachment_a_world()))
		check_near(power, s.damping_power(), 1.0e-4, "the power delivered by the applied pair equals -c v^2")
		check(power <= 1.0e-6, "the applied forces never inject power (%s W)" % power)


func test_d8_the_spring_stores_and_releases_energy_but_never_creates_it() -> void:
	# Two dynamic bodies on the axis, zero damping, released from rest at 6 cm: E = KE + k x^2 / 2. The energy of the discrete scheme
	# oscillates inside the exact bound of the header and does not grow; at the turning points it returns to its starting value.
	var k: float = 3000.0
	var r: Dictionary = _rig({"on_axis": true}, k, 0.0)
	var mu: float = r.a.mass * r.b.mass / (r.a.mass + r.b.mass)
	var w: float = sqrt(k / mu)
	_displace(r, 0.06)
	var e0: float = r.susp.spring_energy()
	check_near(e0, 0.5 * k * 0.06 * 0.06, 1.0e-2, "initial stored energy")
	var lo: float = e0 / (1.0 + w * DT / 2.0)
	var hi: float = e0 / (1.0 - w * DT / 2.0)
	var e_min: float = INF
	var e_max: float = -INF
	var max_ke_gain_total: float = 0.0
	for i in 480:
		r.world.step()
		var e: float = r.world.total_kinetic_energy() + r.susp.spring_energy()
		e_min = minf(e_min, e)
		e_max = maxf(e_max, e)
	check(e_min >= lo * (1.0 - 1.0e-4), "the total energy never falls below the discrete bound (min %s, bound %s)" % [e_min, lo])
	check(e_max <= hi * (1.0 + 1.0e-4), "the total energy never exceeds the discrete bound: nothing is created (max %s, bound %s, E0 %s)" % [e_max, hi, e0])
	# It really exchanges energy: kinetic energy reaches a large fraction of the stored energy.
	var r2: Dictionary = _rig({"on_axis": true}, k, 0.0)
	_displace(r2, 0.06)
	var ke_peak: float = 0.0
	var spring_min: float = INF
	for i in 120:
		r2.world.step()
		ke_peak = maxf(ke_peak, r2.world.total_kinetic_energy())
		spring_min = minf(spring_min, r2.susp.spring_energy())
	check(ke_peak >= 0.8 * e0, "the spring releases its energy into motion (peak KE %s of %s J)" % [ke_peak, e0])
	check(spring_min <= 0.1 * e0, "and is nearly empty at mid-travel (min %s J)" % spring_min)
	check_vec3_near(r.world.total_linear_momentum(), Vector3.ZERO, 1.0e-4, "momentum stays zero")


func test_d9_static_and_dynamic_participation_follow_the_body_api() -> void:
	# Static A (like gravity, a static body receives no load; its accumulators stay zero and its pose never changes).
	var r: Dictionary = _rig({"on_axis": true, "static_a": true}, 800.0, 10.0)
	_displace(r, 0.05)
	var pos_a: Vector3 = r.a.position
	r.susp.apply_forces(DT, r.world.config)
	check(r.a.force_accumulator == Vector3.ZERO and r.a.torque_accumulator == Vector3.ZERO, "a static body A receives no force or torque")
	check_vec3_near(r.b.force_accumulator, -r.u * 800.0 * 0.05, 1.0e-2, "the dynamic body B receives the full force")
	r.b.clear_accumulators()
	var moved: float = 0.0
	for i in 120:
		r.world.step()
		moved = maxf(moved, absf(Rig.measure(r).s))
	check(r.a.position == pos_a and r.a.linear_velocity == Vector3.ZERO, "static A never moves")
	check(moved > 0.04 and moved <= 0.0501, "B oscillates against the static body (excursion %s)" % moved)
	# Static B: the dynamic A is loaded with -F u.
	var rb: Dictionary = Rig.make({"on_axis": true})
	rb.world.remove_body(rb.b)
	var sb: SimRigidBody = SimRigidBody.new(SimMassProperties.new(0.0), rb.b.position, rb.b.orientation)
	rb.world.add_body(sb)
	var jb = PrismaticJoint.from_world(rb.a, sb, rb.point, rb.u, Rig.perp_reference(rb.u))
	rb.world.add_constraint(jb)
	var susp_b = FrontSuspension.from_world(jb, rb.point - rb.u * 0.1, rb.point + rb.u * 0.2, 500.0, 0.0, 0.0)
	check(susp_b != null and rb.world.add_force_provider(susp_b) == susp_b, "a static body on the B side is allowed")
	sb.position += rb.u * 0.04
	susp_b.apply_forces(DT, rb.world.config)
	check(sb.force_accumulator == Vector3.ZERO, "a static body B receives nothing")
	check_vec3_near(rb.a.force_accumulator, rb.u * 500.0 * 0.04, 1.0e-2, "dynamic A receives -F u (F = -k x)")


func test_d10_invalid_parameters_are_rejected_never_repaired() -> void:
	var r: Dictionary = Rig.make({"on_axis": true})
	var u: Vector3 = r.u
	var pa: Vector3 = r.point - u * 0.1
	var pb: Vector3 = r.point + u * 0.2
	var cases: Array = [
		["negative stiffness", func(): return FrontSuspension.from_world(r.joint, pa, pb, -1.0, 10.0, 0.0)],
		["NaN stiffness", func(): return FrontSuspension.from_world(r.joint, pa, pb, NAN, 10.0, 0.0)],
		["infinite stiffness", func(): return FrontSuspension.from_world(r.joint, pa, pb, INF, 10.0, 0.0)],
		["negative damping", func(): return FrontSuspension.from_world(r.joint, pa, pb, 100.0, -0.1, 0.0)],
		["NaN damping", func(): return FrontSuspension.from_world(r.joint, pa, pb, 100.0, NAN, 0.0)],
		["infinite damping", func(): return FrontSuspension.from_world(r.joint, pa, pb, 100.0, INF, 0.0)],
		["NaN rest displacement", func(): return FrontSuspension.from_world(r.joint, pa, pb, 100.0, 1.0, NAN)],
		["infinite rest displacement", func(): return FrontSuspension.from_world(r.joint, pa, pb, 100.0, 1.0, INF)],
		["non-finite seat", func(): return FrontSuspension.from_world(r.joint, Vector3(NAN, 0, 0), pb, 100.0, 1.0, 0.0)],
		["seats off one line parallel to the axis", func(): return FrontSuspension.from_world(r.joint, pa, pb + Vector3(0.0, 0.0, 0.01) - u * u.dot(Vector3(0.0, 0.0, 0.01)), 100.0, 1.0, 0.0)],
		["null joint", func(): return FrontSuspension.from_world(null, pa, pb, 100.0, 1.0, 0.0)],
	]
	for c in cases:
		check(c[1].call() == null, "%s: from_world returns null" % c[0])
	# Zero values are valid (explicit tests above rely on them).
	check(FrontSuspension.from_world(r.joint, pa, pb, 0.0, 0.0, 0.0) != null, "k = 0, c = 0 is valid")
	# Direct construction + registration: rejected by validate() and by add_force_provider(), world untouched.
	var bad = FrontSuspension.new(r.joint, Vector3.ZERO, Vector3.ZERO, -5.0, 1.0, 0.0)
	check(not bad.is_valid() and not bad.validate().is_empty(), "validate() reports a negative stiffness")
	check(r.world.add_force_provider(bad) == null and r.world.force_providers.is_empty(), "add_force_provider refuses it and registers nothing")
	# Mutated after registration: the next step is refused with the state untouched, and resumes when fixed.
	var good = FrontSuspension.from_world(r.joint, pa, pb, 100.0, 1.0, 0.0)
	r.world.add_force_provider(good)
	good.stiffness = NAN
	var before: Vector3 = r.b.position
	check(not r.world.step() and r.world.rejected_step_count == 1 and r.world.last_error.contains("force provider"), "a provider turned invalid is caught by preflight (%s)" % r.world.last_error.left(70))
	check(r.b.position == before and r.world.step_count == 0, "the refused step changed nothing")
	good.stiffness = 100.0
	check(r.world.step(), "fixing the stiffness resumes stepping")
	# Two static bodies have nothing to act on.
	var w: SimWorld = make_world()
	var s1: SimRigidBody = w.add_body(make_static())
	var s2: SimRigidBody = w.add_body(make_static(Vector3(0, 0, 1)))
	var probe_joint = PrismaticJoint.new(s1, s2, Vector3.ZERO, Vector3.ZERO, Vector3.BACK, Vector3.BACK, Vector3.UP, Vector3.UP)
	var two_static = FrontSuspension.new(probe_joint, Vector3.ZERO, Vector3.ZERO, 1.0, 1.0, 0.0)
	check(not two_static.is_valid(), "a suspension between two static bodies is invalid")


## float64 reference of the exact discrete scheme: F from the start-of-step state, v' = v + h F / m, x' = x + h v'.
func _discrete_reference(x0: float, v0: float, k: float, c: float, mass: float, rest: float, steps: int) -> PackedFloat64Array:
	var x: float = x0
	var v: float = v0
	var out: PackedFloat64Array = PackedFloat64Array()
	for i in steps:
		var f: float = -k * (x - rest) - c * v
		v += DT * f / mass
		x += DT * v
		out.append(x)
	return out


func test_d11_free_oscillation_matches_the_discrete_scheme_and_the_continuous_solution() -> void:
	# (a) Static A, body B (5 kg) with COM on the axis: a pure scalar oscillator. k = 1200 N/m, c = 20 N s/m, x_rest = 0.02 m,
	# released from x0 = 0.07 m at rest. omega = 15.49 rad/s, zeta = 0.129, omega dt = 0.129.
	var k: float = 1200.0
	var c: float = 20.0
	var rest: float = 0.02
	var steps: int = 360
	var r: Dictionary = _rig({"on_axis": true, "static_a": true}, k, c, rest)
	var m: float = r.b.mass
	_displace(r, 0.07)
	var ref: PackedFloat64Array = _discrete_reference(0.07, 0.0, k, c, m, rest, steps)
	var worst_discrete: float = 0.0
	var worst_continuous: float = 0.0
	var w0: float = sqrt(k / m)
	var zeta: float = c / (2.0 * sqrt(k * m))
	var wd: float = w0 * sqrt(1.0 - zeta * zeta)
	for i in steps:
		r.world.step()
		var x_sim: float = Rig.measure(r).s
		worst_discrete = maxf(worst_discrete, absf(x_sim - ref[i]))
		var t: float = (i + 1) * DT
		var x0: float = 0.07 - rest
		var x_cont: float = rest + exp(-zeta * w0 * t) * (x0 * cos(wd * t) + (zeta * w0 * x0 / wd) * sin(wd * t))
		worst_continuous = maxf(worst_continuous, absf(x_sim - x_cont))
	# Same arithmetic in float64 vs float32 state: the discrete reference must agree to float32 round-off accumulated over 360 steps
	# (x ~ 0.07: ~1e-8 per step, random walk). Measured 1.6e-7 m (30x margin).
	check(worst_discrete <= 5.0e-6, "the simulation equals the exact discrete scheme (worst |x_sim - x_ref| = %s m)" % worst_discrete)
	# Continuous solution: the explicit-damping, symplectic-Euler scheme differs by a frequency shift (w dt)^2 / 24 (phase error
	# 2 pi n (w dt)^2/24 over n periods ~ 0.036 rad over 3 periods at amplitude 0.05 -> 1.8e-3 m) plus the O(zeta w dt) decay error
	# (~0.1 * 0.129 * 0.05 = 6e-4 m): estimate ~2.4e-3 m. Measured 3.2e-3 m; bound 4e-3 m = 8 % of the amplitude.
	check(worst_continuous <= 4.0e-3, "the simulation follows the continuous damped solution to the O(omega dt) error of the scheme (worst %s m)" % worst_continuous)
	# The oscillation is about x_rest and decays: late amplitude well below the initial one.
	var late: float = 0.0
	for i in 120:
		r.world.step()
		late = maxf(late, absf(Rig.measure(r).s - rest))
	check(late < 0.05 * 0.2, "the oscillation about x_rest has decayed (late amplitude %s m)" % late)
	check(r.world.numerical_fault == "", "no numerical fault")

	# (b) Two dynamic bodies (3 kg and 5 kg): the relative coordinate obeys the same scalar law with the REDUCED mass, and total momentum
	# stays zero (equal and opposite forces).
	var r2: Dictionary = _rig({"on_axis": true}, k, c, rest)
	var mu: float = r2.a.mass * r2.b.mass / (r2.a.mass + r2.b.mass)
	_displace(r2, 0.07)
	var ref2: PackedFloat64Array = _discrete_reference(0.07, 0.0, k, c, mu, rest, steps)
	var worst2: float = 0.0
	var worst_p: float = 0.0
	for i in steps:
		r2.world.step()
		worst2 = maxf(worst2, absf(Rig.measure(r2).s - ref2[i]))
		worst_p = maxf(worst_p, r2.world.total_linear_momentum().length())
	check(worst2 <= 5.0e-6, "two dynamic bodies follow the reduced-mass discrete scheme (worst %s m)" % worst2)
	check(worst_p <= 1.0e-4, "total momentum stays zero (worst %s N s)" % worst_p)


func test_d12_the_force_never_acts_along_the_constrained_directions_in_a_full_run() -> void:
	# Off-axis, rotated bodies; excited in all directions; every step the applied force (read through the accumulators before the
	# step consumes them) is parallel to the CURRENT fork axis.
	var r: Dictionary = _rig({}, 2500.0, 50.0)
	r.b.apply_impulse(Vector3(1.0, -2.0, 0.5))
	r.b.apply_angular_impulse(Vector3(0.1, 0.2, -0.1))
	var worst: float = 0.0
	for i in 120:
		r.susp.apply_forces(DT, r.world.config)
		var f: Vector3 = r.b.force_accumulator
		worst = maxf(worst, f.cross(r.susp.axis()).length() / maxf(f.length(), 1.0))
		r.b.clear_accumulators()
		r.a.clear_accumulators()
		r.world.step()
	check(worst <= 2.0e-6, "the suspension force stays parallel to the fork axis (worst relative perpendicular part %s)" % worst)


# =====================================================================================================================
# G. Generic force-provider lifecycle in SimWorld
# =====================================================================================================================

func test_g1_providers_are_evaluated_once_per_fixed_step_not_per_frame() -> void:
	var w: SimWorld = make_world()
	var b: SimRigidBody = w.add_body(make_sphere(2.0, 0.1))
	var p: Counting = Counting.new()
	p.body = b
	p.force = Vector3(4.0, 0.0, 0.0)
	check(w.add_force_provider(p) == p and w.force_providers.size() == 1, "registration")
	# 0.05 s of frame time = 6 fixed steps of 1/120 s: the provider is asked 6 times and the force acts on EVERY step.
	var steps: int = w.advance(0.05)
	check(steps == 6 and p.calls == 6, "six fixed steps, six evaluations (steps %d, calls %d)" % [steps, p.calls])
	check_near(p.last_dt, DT, 1.0e-12, "the provider receives the fixed timestep")
	check_near(b.linear_velocity.x, 4.0 / 2.0 * 6.0 * DT, 1.0e-6, "the force acted for all six steps: v = F t / m")
	check(b.force_accumulator == Vector3.ZERO, "forces are consumed by each step")
	p.enabled = false
	w.advance(0.05)
	check(p.calls == 6, "a disabled provider is skipped")
	p.enabled = true
	w.step()
	check(p.calls == 7, "re-enabled, it is asked again")
	w.remove_force_provider(p)
	w.step()
	check(p.calls == 7 and w.force_providers.is_empty(), "a removed provider is no longer asked")


func test_g2_membership_and_body_removal() -> void:
	var w: SimWorld = make_world()
	var b: SimRigidBody = w.add_body(make_sphere(1.0, 0.1))
	var other: SimWorld = make_world()
	var foreign: SimRigidBody = other.add_body(make_sphere(1.0, 0.1))
	var p_foreign: Counting = Counting.new()
	p_foreign.body = foreign
	check(w.add_force_provider(p_foreign) == null and w.force_providers.is_empty(), "a provider of a foreign body is refused")
	check(w.add_force_provider(null) == null, "a null provider is refused")
	var p_null: Counting = Counting.new()
	p_null.body = null
	check(w.add_force_provider(p_null) == null, "a provider with a null body is refused")
	var p: Counting = Counting.new()
	p.body = b
	w.add_force_provider(p)
	var keeper: SimRigidBody = w.add_body(make_sphere(1.0, 0.1, Vector3(1, 0, 0)))
	var p2: Counting = Counting.new()
	p2.body = keeper
	w.add_force_provider(p2)
	w.remove_body(b)
	check(w.force_providers == [p2], "removing a body removes the providers that involve it and keeps the others in order")
	# A body erased from the public array behind the world's back is caught by preflight.
	w.bodies.erase(keeper)
	check(not w.step() and w.last_error.contains("force provider"), "a provider whose body left the world refuses the step (%s)" % w.last_error.left(60))


func test_g3_a_non_finite_provider_force_is_contained_by_the_fault_path() -> void:
	var w: SimWorld = make_world()
	var b: SimRigidBody = w.add_body(make_sphere(1.0, 0.1))
	var p: Counting = Counting.new()
	p.body = b
	p.force = Vector3(NAN, 0.0, 0.0)
	w.add_force_provider(p)
	var before: Vector3 = b.position
	check(not w.step(), "a non-finite provider force aborts the step")
	check(w.numerical_fault.contains("linear velocity"), "the fault names the non-finite velocity (%s)" % w.numerical_fault.left(70))
	check(b.position == before and b.force_accumulator == Vector3.ZERO and b.linear_velocity.is_finite(), "rolled back, accumulators discarded, state finite")
	check(w.step_count == 0 and w.rejected_step_count == 0, "an abort is not a rejection and not a step")
