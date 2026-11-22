extends "res://Tests/sim_test_case.gd"
## M2.2 generic PrismaticJointConstraint (Simulation/Solver/prismatic_joint_constraint.gd). Generic physics: no motorcycle code.
##
## A construction and validation     B kinematics and degrees of freedom     C travel limits and stabilization
## F regression sensitivity (in-suite mutants: every defect the assignment names that belongs to the joint)
##
## Methodology notes (numbers are measurements of this implementation; the mechanism is stated where one is known):
## * Behaviour is judged against the CONFIGURED geometry (world point, world axis, world reference direction) through test-owned
##   copies of the local frames (fork_rig.gd), never against the constraint's cached rows or local vectors.
## * Solver: 200 iterations (the converged setting of the hinge/motorcycle tests). Velocity probes start from rest, run ONE step and
##   evaluate lever arms at the start-of-step poses, the frames the solver linearized (a residual measured at the moved poses
##   would show a fake error of order w_rel * w_carrier * dt, see test_motorcycle_assembly.gd).
## * Exact identity used for the free axis: the joint's lateral rows have Jv perpendicular to the axis and the rotation rows have
##   Jv = 0, so the constraint never changes a body's COM velocity ALONG the axis; the relative axial COM velocity after an equal
##   and opposite axial impulse pair J is therefore exactly J (1/m_a + 1/m_b), whatever the orientations and COM offsets.
## * float32: Vector3/Basis/Quaternion carry ~1.2e-7 relative error, so pose and identity tolerances are >= 1e-6 (never 1e-9).
##   Constraint-solver ERROR output from deliberately invalid input in section A is expected (TESTING.md).

const Rig = preload("res://Tests/fork_rig.gd")
const PrismaticJoint = preload("res://Simulation/Solver/prismatic_joint_constraint.gd")

const DT: float = 1.0 / 120.0
## float32 construction tolerance (coordinates <= ~1 m, 4 roundings of ~1.2e-7 doubled).
const POSE_TOL: float = 2.0e-6


# --- Mutants (in-suite controlled defects; each overrides only _update_rows of the real joint) ---
class NoTwist extends "res://Simulation/Solver/prismatic_joint_constraint.gd":
	func _update_rows(dt, config) -> void:
		super(dt, config)
		_deactivate(rows[ROW_TWIST])


class NoAxisRows extends "res://Simulation/Solver/prismatic_joint_constraint.gd":
	func _update_rows(dt, config) -> void:
		super(dt, config)
		_deactivate(rows[ROW_AXIS_P])
		_deactivate(rows[ROW_AXIS_Q])


class ReversedLimitSign extends "res://Simulation/Solver/prismatic_joint_constraint.gd":
	func _update_rows(dt, config) -> void:
		super(dt, config)
		for row in [rows[ROW_LOWER], rows[ROW_UPPER]]:
			row.linear_a = -row.linear_a
			row.angular_a = -row.angular_a
			row.linear_b = -row.linear_b
			row.angular_b = -row.angular_b


## Violated limit corrected with a velocity-level (Baumgarte) bias on the PHYSICAL row instead of the split channel.
class SplitOnPhysicalVelocity extends "res://Simulation/Solver/prismatic_joint_constraint.gd":
	func _update_rows(dt, config) -> void:
		super(dt, config)
		for row in [rows[ROW_LOWER], rows[ROW_UPPER]]:
			if row.split_enabled:
				row.bias = row.split_bias
				row.split_enabled = false


## Replaces the rig's registered joint by `script.new(...)` with identical data (the mutant).
func _swap(r: Dictionary, script) -> Dictionary:
	var old = r.joint
	r.world.remove_constraint(old)
	var m = script.new(old.body_a, old.body_b, old.local_anchor_a, old.local_anchor_b, old.local_axis_a, old.local_axis_b,
		old.local_reference_a, old.local_reference_b)
	m.limit_enabled = old.limit_enabled
	m.lower_limit = old.lower_limit
	m.upper_limit = old.upper_limit
	r.world.add_constraint(m)
	r.joint = m
	return r


# =====================================================================================================================
# A. Construction and validation
# =====================================================================================================================

func test_a1_valid_construction_and_registration() -> void:
	var r: Dictionary = Rig.make()
	var j = r.joint
	check(j != null and j.is_valid(), "from_world builds a valid joint")
	if j == null:
		return
	check(r.world.constraints.size() == 1 and r.world.constraints[0] == j, "the joint is registered exactly once")
	check(r.world.preflight(), "a world holding the joint passes preflight")
	check(j.rows.size() == PrismaticJoint.ROW_COUNT and PrismaticJoint.ROW_COUNT == 7, "seven rows: 2 lateral, 2 axis, 1 twist, 2 limits")
	check_near(j.translation(), 0.0, POSE_TOL, "translation coordinate is 0 at the reference configuration")
	check(j.lateral_error().length() <= POSE_TOL, "no lateral error at the reference configuration")
	check(j.axis_error_angle() <= POSE_TOL, "aligned axes at the reference configuration")
	check(absf(j.twist_error_angle()) <= POSE_TOL, "zero twist at the reference configuration")
	check(not j.limit_enabled, "limits are disabled by default")
	var m: Dictionary = Rig.measure(r)
	check_near(m.s, 0.0, POSE_TOL, "the test-owned measurement agrees: s = 0")
	check(m.lateral <= POSE_TOL and m.axis <= POSE_TOL and absf(m.twist) <= POSE_TOL, "the test-owned measurement agrees: lateral, axis and twist errors are 0")
	# A static body is allowed on either side (not both).
	var rs: Dictionary = Rig.make({"static_a": true})
	check(rs.joint != null and rs.world.preflight(), "static body A + dynamic body B is a valid joint")
	var w: SimWorld = make_world()
	var sb: SimRigidBody = w.add_body(make_static())
	var db: SimRigidBody = w.add_body(make_sphere(1.0, 0.1, Vector3(0.0, 0.0, 1.0)))
	var jb = PrismaticJoint.from_world(db, sb, Vector3(0.0, 0.0, 0.5), Vector3.BACK, Vector3.UP)
	check(jb != null and w.add_constraint(jb) == jb, "dynamic body A + static body B is a valid joint")


func test_a2_invalid_input_is_rejected_and_never_repaired() -> void:
	var w: SimWorld = make_world()
	var a: SimRigidBody = w.add_body(make_sphere(1.0, 0.1))
	var b: SimRigidBody = w.add_body(make_sphere(1.0, 0.1, Vector3(0, 0, 1)))
	var s1: SimRigidBody = w.add_body(make_static(Vector3(5, 0, 0)))
	var s2: SimRigidBody = w.add_body(make_static(Vector3(6, 0, 0)))
	var u: Vector3 = Vector3.BACK
	var ref: Vector3 = Vector3.UP
	var cases: Array = [
		["null body A", func(): return PrismaticJoint.from_world(null, b, Vector3.ZERO, u, ref)],
		["null body B", func(): return PrismaticJoint.from_world(a, null, Vector3.ZERO, u, ref)],
		["same body twice", func(): return PrismaticJoint.from_world(a, a, Vector3.ZERO, u, ref)],
		["two static bodies", func(): return PrismaticJoint.from_world(s1, s2, Vector3.ZERO, u, ref)],
		["non-finite anchor", func(): return PrismaticJoint.from_world(a, b, Vector3(NAN, 0, 0), u, ref)],
		["infinite anchor", func(): return PrismaticJoint.from_world(a, b, Vector3(INF, 0, 0), u, ref)],
		["zero axis", func(): return PrismaticJoint.from_world(a, b, Vector3.ZERO, Vector3.ZERO, ref)],
		["non-unit axis", func(): return PrismaticJoint.from_world(a, b, Vector3.ZERO, u * 1.001, ref)],
		["NaN axis", func(): return PrismaticJoint.from_world(a, b, Vector3.ZERO, Vector3(NAN, 0, 1), ref)],
		["non-unit reference", func(): return PrismaticJoint.from_world(a, b, Vector3.ZERO, u, ref * 0.9)],
		["reference parallel to the axis", func(): return PrismaticJoint.from_world(a, b, Vector3.ZERO, u, u)],
		["reference not perpendicular (5 deg)", func(): return PrismaticJoint.from_world(a, b, Vector3.ZERO, u, Quaternion(Vector3.RIGHT, deg_to_rad(5.0)) * ref)],
		["NaN reference", func(): return PrismaticJoint.from_world(a, b, Vector3.ZERO, u, Vector3(0, NAN, 0))],
	]
	for c in cases:
		var count_before: int = w.constraints.size()
		check(c[1].call() == null, "%s: from_world returns null" % c[0])
		check(w.constraints.size() == count_before, "%s: nothing was registered" % c[0])
	# Direct construction with invalid limits: rejected by validate() and by add_constraint(); the world is untouched.
	var limit_cases: Array = [
		["lower > upper (enabled)", true, 0.1, -0.1],
		["NaN lower", true, NAN, 0.1],
		["infinite upper", true, -0.1, INF],
		["NaN lower while disabled", false, NAN, 0.1],
	]
	for c in limit_cases:
		var joint = PrismaticJoint.from_world(a, b, Vector3(0, 0, 0.5), u, ref)
		joint.limit_enabled = c[1]
		joint.lower_limit = c[2]
		joint.upper_limit = c[3]
		check(not joint.is_valid() and not joint.validate().is_empty(), "%s: validate() reports it" % c[0])
		var constraints_before: Array = w.constraints.duplicate()
		var bodies_before: Array = w.bodies.duplicate()
		check(w.add_constraint(joint) == null, "%s: add_constraint refuses it" % c[0])
		check(w.constraints == constraints_before and w.bodies == bodies_before, "%s: the world is exactly as before" % c[0])
	var ok = PrismaticJoint.from_world(a, b, Vector3(0, 0, 0.5), u, ref)
	ok.limit_enabled = true
	ok.lower_limit = 0.0
	ok.upper_limit = 0.0
	check(ok.is_valid(), "lower == upper is valid (a locked slider)")
	# Constraint data mutated AFTER registration: the next step is refused with the state untouched.
	var r: Dictionary = Rig.make()
	r.joint.limit_enabled = true
	r.joint.lower_limit = 0.5
	r.joint.upper_limit = -0.5
	var position_before: Vector3 = r.b.position
	check(not r.world.step() and r.world.rejected_step_count == 1, "invalid limits after registration: the step is refused")
	check(r.b.position == position_before and r.world.step_count == 0, "the refused step changed nothing")
	r.joint.lower_limit = -0.5
	r.joint.upper_limit = 0.5
	check(r.world.step(), "fixing the limits resumes stepping")
	# Membership: a body of another world is refused.
	var other: SimWorld = make_world()
	var foreign: SimRigidBody = other.add_body(make_sphere(1.0, 0.1))
	var joint_foreign = PrismaticJoint.from_world(a, foreign, Vector3(0, 0, 0.5), u, ref)
	check(w.add_constraint(joint_foreign) == null, "a body that is not a member of the world is refused")


func test_a3_local_frames_derive_from_world_geometry_for_differently_oriented_bodies() -> void:
	var r: Dictionary = Rig.make()
	var j = r.joint
	var u: Vector3 = r.u
	# Both anchors are the same world point at the reference pose; the world axes both equal the configured axis.
	check_vec3_near(j.world_anchor_a(), r.point, POSE_TOL, "anchor A maps onto the configured world point")
	check_vec3_near(j.world_anchor_b(), r.point, POSE_TOL, "anchor B maps onto the configured world point")
	check_vec3_near(j.world_axis_a(), u, POSE_TOL, "world axis of A equals the configured axis")
	check_vec3_near(j.world_axis_b(), u, POSE_TOL, "world axis of B equals the configured axis")
	check_vec3_near(r.a.orientation * j.local_reference_a, r.ref, POSE_TOL, "reference direction of A maps onto the configured one")
	check_vec3_near(r.b.orientation * j.local_reference_b, r.ref, POSE_TOL, "reference direction of B maps onto the configured one")
	# Different orientations => different local vectors (a shared local frame would be wrong for one of the bodies).
	check((j.local_axis_a - j.local_axis_b).length() > 0.5, "the two local axes differ (bodies are oriented differently)")
	check((j.local_reference_a - j.local_reference_b).length() > 0.5, "the two local reference vectors differ")
	check((j.local_anchor_a - j.local_anchor_b).length() > 0.3, "the two local anchors differ")
	# Local frames equal the inverse-rotated world data, and are exactly unit and perpendicular.
	check_vec3_near(j.local_axis_a, r.a.orientation.inverse() * u, POSE_TOL, "local axis A = R_a^-1 u")
	check_vec3_near(j.local_axis_b, r.b.orientation.inverse() * u, POSE_TOL, "local axis B = R_b^-1 u")
	check_vec3_near(j.local_anchor_a, r.a.orientation.inverse() * (r.point - r.a.position), POSE_TOL, "local anchor A = R_a^-1 (P - x_a)")
	check_vec3_near(j.local_anchor_b, r.b.orientation.inverse() * (r.point - r.b.position), POSE_TOL, "local anchor B = R_b^-1 (P - x_b)")
	check_near(j.local_axis_a.length(), 1.0, POSE_TOL, "local axis A is unit")
	check_near(j.local_axis_a.dot(j.local_reference_a), 0.0, POSE_TOL, "reference A is perpendicular to axis A")
	check_near(j.local_axis_b.dot(j.local_reference_b), 0.0, POSE_TOL, "reference B is perpendicular to axis B")
	# Rotating BOTH bodies rigidly (same world rotation about a point) keeps every error at zero: the frames follow the bodies.
	var dq: Quaternion = Quaternion(Vector3(0.2, -1.0, 0.4).normalized(), 0.9)
	var centre: Vector3 = Vector3(0.7, -0.3, 0.2)
	for body in [r.a, r.b]:
		body.position = centre + dq * (body.position - centre)
		body.orientation = dq * body.orientation
	check_near(j.translation(), 0.0, 4.0e-6, "rigid rotation of both bodies: translation stays 0")
	check(j.lateral_error().length() <= 4.0e-6 and j.axis_error_angle() <= 4.0e-6 and absf(j.twist_error_angle()) <= 4.0e-6, "rigid rotation of both bodies: no constraint error")
	check_vec3_near(j.world_axis_a(), dq * u, 4.0e-6, "the world axis rotated with the bodies")


# =====================================================================================================================
# B. Kinematics and degrees of freedom
# =====================================================================================================================
# One step from rest, 200 iterations, standard rig (raked axis, bodies rotated differently, COMs off the axis). Judged against the
# CONFIGURED axis u and point P with start-of-step lever arms. Measured on the correct joint (this file's first run):
# see the thresholds below; the mutants of section F exceed every threshold by >= 100x.

const PROBE_J: float = 6.0
const PROBE_L: float = 3.0
## Residual of a constrained velocity row after one step (measured values in the comment of each use; 200 iterations).
const RESIDUAL_TOL: float = 5.0e-3

func _step_probe(r: Dictionary, apply: Callable) -> Dictionary:
	var a: SimRigidBody = r.a
	var b: SimRigidBody = r.b
	var lever_a: Vector3 = r.point - a.position
	var lever_b: Vector3 = r.point - b.position
	apply.call(r)
	r.world.step()
	var va: Vector3 = a.linear_velocity + a.angular_velocity.cross(lever_a)
	var vb: Vector3 = b.linear_velocity + b.angular_velocity.cross(lever_b)
	return {"v_rel_point": vb - va, "w_rel": b.angular_velocity - a.angular_velocity,
		"axial_com": (b.linear_velocity - a.linear_velocity).dot(r.u)}


func test_b1_axial_translation_is_free_in_both_directions() -> void:
	for sign in [1.0, -1.0]:
		var r: Dictionary = Rig.make()
		var u: Vector3 = r.u
		var m: Dictionary = _step_probe(r, func(rig):
			rig.b.apply_impulse_at_point(u * PROBE_J * sign, rig.point)
			rig.a.apply_impulse_at_point(-u * PROBE_J * sign, rig.point))
		var expected: float = PROBE_J * sign * (1.0 / r.b.mass + 1.0 / r.a.mass)
		# Exact identity (header): the constraint adds no axial COM velocity. float32 over 7 rows x 200 iterations: 1e-5.
		check_near(m.axial_com, expected, 1.0e-5, "relative axial COM velocity = J (1/m_a + 1/m_b) for sign %s" % sign)
		check_near(r.joint.lower_limit_impulse() + r.joint.upper_limit_impulse(), 0.0, 0.0, "no limit impulse with limits disabled")
		# The pair really slides: after 30 steps the coordinate moved in the pushed direction by v t (no restoring force).
	var r2: Dictionary = Rig.make({"on_axis": true})
	r2.b.linear_velocity = r2.u * 0.5
	for i in 60:
		r2.world.step()
	check_near(Rig.measure(r2).s, 0.5 * 60.0 * DT, 1.0e-5, "free translation: s = v t after 0.5 s (on-axis bodies, no hidden force)")
	check_near(r2.b.linear_velocity.dot(r2.u), 0.5, 1.0e-6, "the axial velocity is not reduced (no hidden axial force)")
	r2.b.linear_velocity = -r2.u * 0.5
	for i in 120:
		r2.world.step()
	check_near(Rig.measure(r2).s, 0.5 * 60.0 * DT - 0.5 * 120.0 * DT, 2.0e-5, "free translation in the negative direction")


func test_b2_perpendicular_translation_is_constrained() -> void:
	var u: Vector3 = Rig.axis()
	var ref: Vector3 = Rig.perp_reference(u)
	var e2: Vector3 = u.cross(ref)
	for e in [ref, e2, -ref, -e2]:
		var r: Dictionary = Rig.make()
		var m: Dictionary = _step_probe(r, func(rig): rig.b.apply_impulse(e * PROBE_J))
		# Relative velocity of the two bodies at the joint point, evaluated at the start-of-step frames. Measured <= 1e-4 m/s.
		check((m.v_rel_point - u * m.v_rel_point.dot(u)).length() <= RESIDUAL_TOL, "lateral relative velocity at the joint point stays ~0 for impulse %s (got %s)" % [e, (m.v_rel_point - u * m.v_rel_point.dot(u)).length()])
		# The pair moves as one: the whole impulse stays in the pair (linear momentum), and A was set moving by the constraint.
		check_vec3_near(r.a.linear_momentum() + r.b.linear_momentum(), e * PROBE_J, 2.0e-4, "momentum of the pair equals the impulse")
		check(r.a.linear_velocity.length() + r.a.angular_velocity.length() > 0.05, "the lateral impulse reaches body A through the constraint")


func test_b3_all_three_relative_rotations_are_constrained() -> void:
	var u: Vector3 = Rig.axis()
	var ref: Vector3 = Rig.perp_reference(u)
	var e2: Vector3 = u.cross(ref)
	var names: Array = ["about the axis (twist)", "about the reference direction", "about the third direction"]
	var dirs: Array = [u, ref, e2]
	for k in 3:
		for sign in [1.0, -1.0]:
			var r: Dictionary = Rig.make()
			var m: Dictionary = _step_probe(r, func(rig): rig.b.apply_angular_impulse(dirs[k] * PROBE_L * sign))
			# Measured <= 2e-4 rad/s. An unconstrained rotation would give L / I >= 20 rad/s here.
			check(m.w_rel.length() <= RESIDUAL_TOL, "relative angular velocity stays ~0 for an angular impulse %s (got %s rad/s)" % [names[k], m.w_rel.length()])
			check(r.a.angular_velocity.length() > 0.05, "the angular impulse %s reaches body A (the bodies rotate together)" % names[k])


func test_b4_the_constraint_stays_correct_for_a_raked_axis_with_both_bodies_rotating() -> void:
	# Both bodies spin about a world axis through a point away from them (a rigid rotation of the pair) while sliding relative to
	# each other: the lateral offset, axis and twist errors must stay at the frozen-Jacobian level and the sliding coordinate must
	# follow the slide. Judged by the test-owned measurement.
	var r: Dictionary = Rig.make()
	var omega: Vector3 = Vector3(0.6, -1.1, 0.8)
	var centre: Vector3 = Vector3(-0.4, 0.2, 0.3)
	for body in [r.a, r.b]:
		body.angular_velocity = omega
		body.linear_velocity = omega.cross(body.position - centre)
	var worst: Vector3 = Vector3.ZERO
	for i in 90:
		r.world.step()
		var m: Dictionary = Rig.measure(r)
		worst = Vector3(maxf(worst.x, m.lateral), maxf(worst.y, m.axis), maxf(worst.z, absf(m.twist)))
	# Steady state of the split correction: e_step / beta, e_step ~ r (w dt)^2 / 2 (frozen Jacobian). With |w| = 1.5 rad/s, r <= 1 m:
	# 1 * (1.5/120)^2 / 2 / 0.2 = 3.9e-4 m. Measured: lateral 9e-5 m, axis and twist <= 6e-5 rad.
	check(worst.x <= 4.0e-4, "lateral error stays at the frozen-Jacobian level (worst %s m)" % worst.x)
	check(worst.y <= 4.0e-4, "axis error stays small (worst %s rad)" % worst.y)
	check(worst.z <= 4.0e-4, "twist error stays small (worst %s rad)" % worst.z)
	# Spin of the pair about the joint axis with the COMs ON the axis and isotropic inertia: the motion is a torque-free rotation, so
	# the free coordinate must stay 0 (measured 2e-8). With COMs OFF the axis the same initial state is NOT force-free (the pair's
	# centre of mass would need an external centripetal force), so s legitimately drifts there; that case is not asserted.
	var r0: Dictionary = Rig.make({"isotropic": true, "on_axis": true})
	for body in [r0.a, r0.b]:
		body.angular_velocity = r0.u * 1.5
	var worst0: float = 0.0
	for i in 90:
		r0.world.step()
		worst0 = maxf(worst0, absf(Rig.measure(r0).s))
	check(worst0 <= 1.0e-5, "spin about the joint axis does not move the free coordinate (worst |s| = %s)" % worst0)
	check(r.world.numerical_fault == "", "no numerical fault")
	# With an added relative slide the free axis stays free: the pair's centrifugal/Coriolis dynamics move s, the five constrained
	# quantities still stay at the frozen-Jacobian level (a sliding mass in a rotating frame is NOT s = v t).
	var r2: Dictionary = Rig.make()
	for body in [r2.a, r2.b]:
		body.angular_velocity = omega
		body.linear_velocity = omega.cross(body.position - centre)
	r2.b.linear_velocity += r2.u * 0.4
	var worst2: float = 0.0
	for i in 90:
		r2.world.step()
		var m2: Dictionary = Rig.measure(r2)
		worst2 = maxf(worst2, maxf(m2.lateral, maxf(m2.axis, absf(m2.twist))))
	check(worst2 <= 1.0e-3, "sliding in a rotating pair: constrained errors stay small (worst %s)" % worst2)
	check(absf(Rig.measure(r2).s) > 0.1, "the free coordinate did move (s = %s)" % Rig.measure(r2).s)


func test_b5_no_hidden_axial_force_without_limits() -> void:
	# Dynamic pair (both bodies free, COMs on the axis), opposite axial velocities: the relative axial velocity is exactly
	# constant and the centre of mass moves uniformly (no spring, damper, motor or limit acts along a free axis).
	var r: Dictionary = Rig.make({"on_axis": true})
	r.a.linear_velocity = -r.u * 0.3
	r.b.linear_velocity = r.u * 0.9
	var p0: Vector3 = r.world.total_linear_momentum()
	var rel0: float = (r.b.linear_velocity - r.a.linear_velocity).dot(r.u)
	var ke0: float = r.world.total_kinetic_energy()
	var worst: float = 0.0
	for i in 240:
		r.world.step()
		worst = maxf(worst, absf((r.b.linear_velocity - r.a.linear_velocity).dot(r.u) - rel0))
	check(worst <= 1.0e-6, "relative axial velocity unchanged over 2 s (worst deviation %s m/s)" % worst)
	check_near(r.world.total_kinetic_energy(), ke0, 1.0e-5 * ke0, "kinetic energy unchanged (nothing dissipates along the free axis)")
	check_vec3_near(r.world.total_linear_momentum(), p0, 1.0e-5, "total momentum unchanged")
	check_near(r.joint.lower_limit_impulse(), 0.0, 0.0, "no limit impulse")
	check_near(Rig.measure(r).s, rel0 * 240.0 * DT, 1.0e-4, "s = relative velocity * t (float32 over 240 steps)")


# =====================================================================================================================
# C. Travel limits and stabilization
# =====================================================================================================================
# On-axis rig (COMs on the axis, bodies unrotated) so each limit is a single exactly solvable scalar row; the off-axis case is
# covered separately. Limits [-0.05, +0.08] m unless stated. 200 iterations.

const LIM: Vector2 = Vector2(-0.05, 0.08)

func test_c1_lower_limit_prevents_crossing_and_pushes_the_right_way() -> void:
	var r: Dictionary = Rig.make({"on_axis": true, "static_a": true, "limits": LIM})
	r.b.linear_velocity = -r.u * 1.0
	var min_gap: float = INF
	var total_impulse: float = 0.0
	var upper_total: float = 0.0
	var max_speed_after: float = 0.0
	var saw_contact: bool = false
	for i in 40:
		r.world.step()
		var s: float = Rig.measure(r).s
		min_gap = minf(min_gap, s - LIM.x)
		total_impulse += r.joint.lower_limit_impulse()
		upper_total += r.joint.upper_limit_impulse()
		if r.joint.lower_limit_impulse() > 0.0:
			saw_contact = true
	# Speculative bias: the row engages one step before contact so the coordinate arrives exactly at the limit. Measured 0 .. 1.5e-7.
	check(min_gap >= -1.0e-6, "the coordinate never crosses the lower limit (min gap %s m)" % min_gap)
	check(saw_contact, "the lower limit row engaged")
	# Inelastic stop: the total pushing impulse equals the momentum removed, m v = 5 N s, along +u on B.
	check_near(total_impulse, r.b.mass * 1.0, 1.0e-3, "total lower-limit impulse = m v")
	check_near(upper_total, 0.0, 0.0, "the upper limit row never acts at the lower limit")
	check(r.b.linear_velocity.dot(r.u) >= -1.0e-6, "the velocity into the limit is removed (no bounce, no penetration)")
	check_near(r.b.linear_velocity.dot(r.u), 0.0, 1.0e-5, "inelastic stop: resting at the limit")
	check_near(Rig.measure(r).s, LIM.x, 1.0e-5, "the slider rests at the lower limit")


func test_c2_upper_limit_prevents_crossing_and_pushes_the_right_way() -> void:
	var r: Dictionary = Rig.make({"on_axis": true, "static_a": true, "limits": LIM})
	r.b.linear_velocity = r.u * 1.0
	var max_s: float = -INF
	var lower_total: float = 0.0
	var upper_total: float = 0.0
	var first_dv: float = 0.0
	for i in 40:
		var before: float = r.b.linear_velocity.dot(r.u)
		r.world.step()
		max_s = maxf(max_s, Rig.measure(r).s)
		lower_total += r.joint.lower_limit_impulse()
		upper_total += r.joint.upper_limit_impulse()
		if first_dv == 0.0 and r.joint.upper_limit_impulse() > 0.0:
			first_dv = r.b.linear_velocity.dot(r.u) - before
	check(max_s <= LIM.y + 1.0e-6, "the coordinate never crosses the upper limit (max %s, limit %s)" % [max_s, LIM.y])
	check_near(upper_total, r.b.mass * 1.0, 1.0e-3, "total upper-limit impulse = m v")
	check_near(lower_total, 0.0, 0.0, "the lower limit row never acts at the upper limit")
	check(first_dv < 0.0, "the upper limit pushes B along -u (velocity change %s)" % first_dv)
	check_near(Rig.measure(r).s, LIM.y, 1.0e-5, "the slider rests at the upper limit")


func test_c3_limits_hold_for_differently_oriented_bodies_with_offset_coms() -> void:
	# Standard rig: raked axis, rotated bodies, COMs off the axis. Equal and opposite axial impulses at the joint point drive the
	# slider into each limit. The limit row's Jacobian includes the (r_a + d) x a lever term; the translation coordinate measured
	# independently must stop at the limit.
	for direction in [-1.0, 1.0]:
		var r: Dictionary = Rig.make({"limits": LIM})
		var u: Vector3 = r.u
		r.b.apply_impulse_at_point(u * 4.0 * direction, r.point)
		r.a.apply_impulse_at_point(-u * 4.0 * direction, r.point)
		var excursion: float = 0.0
		for i in 120:
			r.world.step()
			excursion = maxf(excursion, Rig.measure(r).s * direction)
		var limit: float = LIM.y if direction > 0.0 else -LIM.x
		# Position-level slack: the constraint solves the anchor-based rate; the independent s adds the second-order lever term.
		check(excursion <= limit + 2.0e-4, "direction %s: the independently measured coordinate stops at the limit (excursion %s, limit %s)" % [direction, excursion, limit])
		check(excursion >= limit - 5.0e-3, "direction %s: the slider actually reached the limit (excursion %s)" % [direction, excursion])
		var m: Dictionary = Rig.measure(r)
		check(m.lateral <= 3.0e-4 and m.axis <= 3.0e-4 and absf(m.twist) <= 3.0e-4, "direction %s: the five constrained DOF stay satisfied at the stop" % direction)


func test_c4_a_violated_limit_is_corrected_by_the_split_channel_only() -> void:
	for side in [-1.0, 1.0]:
		var r: Dictionary = Rig.make({"on_axis": true, "static_a": true, "limits": LIM})
		var limit: float = LIM.x if side < 0.0 else LIM.y
		var violation: float = 0.02
		r.b.position += r.u * (limit + side * violation - Rig.measure(r).s)
		r.b.reset_published_state()
		check_near(Rig.measure(r).s, limit + side * violation, 1.0e-6, "setup: the limit is violated by %s m" % violation)
		var beta: float = r.world.config.baumgarte_factor
		r.world.step()
		# Physical channel: a body at rest keeps v = w = 0 EXACTLY and gains no kinetic energy; the pose moves by beta * violation.
		check(r.b.linear_velocity == Vector3.ZERO and r.b.angular_velocity == Vector3.ZERO, "side %s: physical velocity stays exactly zero" % side)
		check(r.world.total_kinetic_energy() == 0.0, "side %s: the split correction creates no kinetic energy (KE = %s)" % [side, r.world.total_kinetic_energy()])
		check_near(Rig.measure(r).s, limit + side * violation * (1.0 - beta), 2.0e-6, "side %s: one step removes the fraction beta of the violation" % side)
		check(r.b.split_linear_velocity == Vector3.ZERO, "side %s: split velocity is cleared after the step" % side)
		# Geometric convergence (1 - beta)^n, never overshooting, still no velocity.
		var gap: float = violation * (1.0 - beta)
		for n in 6:
			r.world.step()
			gap *= (1.0 - beta)
			check_near(absf(Rig.measure(r).s - limit), gap, 3.0e-6, "side %s: gap after %d more steps follows (1 - beta)^n" % [side, n + 1])
			check(r.b.linear_velocity == Vector3.ZERO, "side %s: still no physical velocity after %d more steps" % [side, n + 1])
		check((Rig.measure(r).s - limit) * side >= 0.0, "side %s: still on the violated side (no overshoot)" % side)


func test_c5_violated_limit_moving_further_out_is_stopped_physically_and_corrected_by_split() -> void:
	var r: Dictionary = Rig.make({"on_axis": true, "static_a": true, "limits": LIM})
	r.b.position += r.u * (LIM.x - 0.02 - Rig.measure(r).s)
	r.b.reset_published_state()
	r.b.linear_velocity = -r.u * 0.5   # moving further out
	var beta: float = r.world.config.baumgarte_factor
	r.world.step()
	# Physical row: bias 0 => Cdot >= 0 (no further outward motion, no outward push-out velocity injected).
	check_near(r.b.linear_velocity.dot(r.u), 0.0, 1.0e-6, "the outward velocity is stopped; no velocity is injected to push the slider back")
	check_near(Rig.measure(r).s, LIM.x - 0.02 * (1.0 - beta), 2.0e-6, "the position moved back by beta * violation through the split channel only")


func test_c6_no_attractive_limit_impulse_when_moving_away() -> void:
	for side in [-1.0, 1.0]:
		var r: Dictionary = Rig.make({"on_axis": true, "static_a": true, "limits": LIM})
		var limit: float = LIM.x if side < 0.0 else LIM.y
		r.b.position += r.u * (limit - Rig.measure(r).s)   # exactly at the limit
		r.b.reset_published_state()
		r.b.linear_velocity = -r.u * 0.7 * side   # moving AWAY from the limit
		var before: Vector3 = r.b.linear_velocity
		for i in 10:
			r.world.step()
			check(r.joint.lower_limit_impulse() == 0.0 and r.joint.upper_limit_impulse() == 0.0, "side %s: no limit impulse while moving away (step %d)" % [side, i])
		check_vec3_near(r.b.linear_velocity, before, 1.0e-6, "side %s: the velocity away from the limit is not reduced (no attraction)" % side)
		# The same inside the range with room to spare: the speculative row stays inactive.
		var r2: Dictionary = Rig.make({"on_axis": true, "static_a": true, "limits": LIM})
		r2.b.linear_velocity = r2.u * 0.2
		r2.world.step()
		check(r2.joint.lower_limit_impulse() == 0.0 and r2.joint.upper_limit_impulse() == 0.0, "side %s: no impulse far from both limits" % side)


func test_c7_disabled_limits_leave_the_axis_free() -> void:
	var r: Dictionary = Rig.make({"on_axis": true, "static_a": true})
	r.joint.lower_limit = -0.05   # stored but not enabled
	r.joint.upper_limit = 0.08
	check(not r.joint.limit_enabled, "limits are not enabled")
	r.b.linear_velocity = -r.u * 1.0
	for i in 120:
		r.world.step()
	check_near(Rig.measure(r).s, -1.0, 2.0e-5, "the slider travels 1 m through the stored lower value")
	check(r.joint.lower_limit_impulse() == 0.0 and r.joint.upper_limit_impulse() == 0.0, "no limit impulse")
	check_near(r.b.linear_velocity.dot(r.u), -1.0, 1.0e-6, "velocity unchanged")
	r.b.linear_velocity = r.u * 1.0
	for i in 240:
		r.world.step()
	check_near(Rig.measure(r).s, 1.0, 5.0e-5, "and 2 m back through the stored upper value")
	# Enabling the limits while violated starts the correction; disabling releases it again.
	r.joint.limit_enabled = true
	r.b.linear_velocity = Vector3.ZERO
	r.world.step()
	check(Rig.measure(r).s < 1.0, "enabling the limits while violated pulls the slider back (split correction)")
	r.joint.limit_enabled = false
	var s_free: float = Rig.measure(r).s
	r.world.step()
	check_near(Rig.measure(r).s, s_free, 1.0e-6, "disabling the limits releases the correction")


func test_c8_resting_load_warm_start_and_reset() -> void:
	# Gravity pushes the slider (5 kg) into the lower limit along -u: holding impulse per step = m g dt, as for the hinge limit.
	var g: float = 9.81
	var final_states: Array = []
	for warm in [true, false]:
		var r: Dictionary = Rig.make({"on_axis": true, "static_a": true, "limits": LIM, "gravity": -Rig.axis() * g})
		r.world.config.warm_starting = warm
		for i in 90:
			r.world.step()
		check_near(r.joint.lower_limit_impulse(), r.b.mass * g * DT, 1.0e-4, "warm_starting=%s: holding impulse = m g dt" % warm)
		check_near(Rig.measure(r).s, LIM.x, 2.0e-5, "warm_starting=%s: the slider rests on the lower limit" % warm)
		check(r.b.linear_velocity.length() <= 1.0e-5, "warm_starting=%s: no residual velocity" % warm)
		final_states.append(Rig.measure(r).s)
		if warm:
			# reset_warm_start() invalidates the carried impulses; the next step rebuilds the holding impulse from cold.
			r.joint.reset_warm_start()
			for row in r.joint.rows:
				check(row.accumulated_impulse == 0.0 and row.split_accumulated_impulse == 0.0, "reset_warm_start zeroes every carried impulse")
			r.world.step()
			check_near(r.joint.lower_limit_impulse(), r.b.mass * g * DT, 1.0e-4, "after a reset the holding impulse is rebuilt in one step")
			# Disabling the limits clears the stale impulse in the next prepare and releases the slider.
			r.joint.limit_enabled = false
			r.world.step()
			check(r.joint.lower_limit_impulse() == 0.0, "a disabled limit carries no stale impulse")
			check(r.b.linear_velocity.dot(r.u) < -0.05, "the freed slider accelerates under gravity")
	check_near(final_states[0], final_states[1], 1.0e-5, "warm and cold solver agree on the resting position")


func test_c9_dynamic_pair_at_a_limit_conserves_momentum_and_never_gains_energy() -> void:
	var r: Dictionary = Rig.make({"on_axis": true, "limits": LIM})
	r.a.linear_velocity = r.u * 0.4
	r.b.linear_velocity = -r.u * 0.9
	var p0: Vector3 = r.world.total_linear_momentum()
	var ke0: float = r.world.total_kinetic_energy()
	var ke_prev: float = ke0
	var worst_rise: float = 0.0
	for i in 120:
		r.world.step()
		worst_rise = maxf(worst_rise, r.world.total_kinetic_energy() - ke_prev)
		ke_prev = r.world.total_kinetic_energy()
		var s: float = Rig.measure(r).s
		check(s >= LIM.x - 1.0e-5 and s <= LIM.y + 1.0e-5, "step %d: the coordinate stays inside the limits (s = %s)" % [i, s])
	check_vec3_near(r.world.total_linear_momentum(), p0, 1.0e-5, "the equal and opposite limit impulses conserve linear momentum")
	check(worst_rise <= 1.0e-5, "kinetic energy never rises (the limit is inelastic; worst rise %s J)" % worst_rise)
	check(r.world.total_kinetic_energy() < ke0, "the impact dissipated kinetic energy")


func test_c10_invalid_and_non_finite_state_is_contained_by_the_world() -> void:
	# Non-finite velocity injected into the dynamic body: aborted step, rollback, fault named, carried impulses invalidated.
	var r: Dictionary = Rig.make({"on_axis": true, "static_a": true, "limits": LIM})
	r.b.linear_velocity = -r.u * 1.0
	for i in 3:
		r.world.step()
	var position_before: Vector3 = r.b.position
	r.b.linear_velocity = Vector3(INF, 0.0, 0.0)
	check(not r.world.step(), "a non-finite velocity aborts the step")
	check(r.world.numerical_fault != "" and r.world.numerical_fault.contains("linear velocity"), "the fault is recorded (%s)" % r.world.numerical_fault.left(80))
	check(r.b.position == position_before, "the body is restored to the pre-step pose")
	for row in r.joint.rows:
		check(is_finite(row.accumulated_impulse) and row.accumulated_impulse == 0.0, "the carried impulse is invalidated, not left non-finite")
	check(not r.world.step() and r.world.rejected_step_count == 1, "further steps are refused until the fault is cleared")
	r.b.linear_velocity = Vector3.ZERO
	r.world.clear_numerical_fault()
	check(r.world.step(), "after clearing the fault and repairing the body, stepping resumes")
	# A non-finite pose of the static body is caught before the solver (D-series of test_fault_handling, here with this joint).
	var r2: Dictionary = Rig.make({"on_axis": true, "static_a": true})
	r2.a.position = Vector3(NAN, 0.0, 0.0)
	check(not r2.world.step() and r2.world.numerical_fault.contains("position"), "a non-finite static pose is detected before the constraint solve")
	check(r2.b.position.is_finite(), "the dynamic body stays finite")
	# Warm-start state never leaks NaN into a later valid world.
	var r3: Dictionary = Rig.make({"on_axis": true, "limits": LIM})
	r3.b.angular_velocity = Vector3(0.0, NAN, 0.0)
	check(not r3.world.step(), "a non-finite angular velocity aborts the step")
	check(r3.world.numerical_fault != "", "the fault is recorded (non-finite angular velocity fails the gyroscopic solve or the state check)")


# =====================================================================================================================
# F. Regression sensitivity: the probe criteria must accept the real joint and reject each named defect
# =====================================================================================================================

## Messages for every acceptance criterion the joint violates (empty = behaves exactly as configured).
func _dof_violations(mutate: Callable) -> PackedStringArray:
	var out: PackedStringArray = PackedStringArray()
	var u: Vector3 = Rig.axis()
	var ref: Vector3 = Rig.perp_reference(u)
	var e2: Vector3 = u.cross(ref)
	# Free axis: exact axial identity.
	var r: Dictionary = mutate.call(Rig.make())
	var m: Dictionary = _step_probe(r, func(rig):
		rig.b.apply_impulse_at_point(u * PROBE_J, rig.point)
		rig.a.apply_impulse_at_point(-u * PROBE_J, rig.point))
	var expected: float = PROBE_J * (1.0 / r.b.mass + 1.0 / r.a.mass)
	if absf(m.axial_com - expected) > 1.0e-4:
		out.append("axial translation is not free (relative axial velocity %s, expected %s)" % [m.axial_com, expected])
	# Lateral translation constrained.
	for e in [ref, e2]:
		var rl: Dictionary = mutate.call(Rig.make())
		var ml: Dictionary = _step_probe(rl, func(rig): rig.b.apply_impulse(e * PROBE_J))
		var lat: float = (ml.v_rel_point - u * ml.v_rel_point.dot(u)).length()
		if lat > RESIDUAL_TOL:
			out.append("lateral translation not constrained (%s m/s)" % lat)
	# All three rotations constrained.
	for e in [u, ref, e2]:
		var rr: Dictionary = mutate.call(Rig.make())
		var mr: Dictionary = _step_probe(rr, func(rig): rig.b.apply_angular_impulse(e * PROBE_L))
		if mr.w_rel.length() > RESIDUAL_TOL:
			out.append("a relative rotation is not constrained (%s rad/s)" % mr.w_rel.length())
	return out


## Messages for the limit behaviour of an on-axis rig built by `mutate`.
func _limit_violations(mutate: Callable) -> PackedStringArray:
	var out: PackedStringArray = PackedStringArray()
	# Lower limit stop: no crossing, pushes toward +u.
	var r: Dictionary = mutate.call(Rig.make({"on_axis": true, "static_a": true, "limits": LIM}))
	r.b.linear_velocity = -r.u * 1.0
	var min_s: float = INF
	for i in 40:
		r.world.step()
		min_s = minf(min_s, Rig.measure(r).s)
	if min_s < LIM.x - 1.0e-4:
		out.append("the lower limit is crossed (min s %s)" % min_s)
	# Upper limit stop.
	var ru: Dictionary = mutate.call(Rig.make({"on_axis": true, "static_a": true, "limits": LIM}))
	ru.b.linear_velocity = ru.u * 1.0
	var max_s: float = -INF
	for i in 40:
		ru.world.step()
		max_s = maxf(max_s, Rig.measure(ru).s)
	if max_s > LIM.y + 1.0e-4:
		out.append("the upper limit is crossed (max s %s)" % max_s)
	# Violated limit: physical velocity must stay exactly zero and the pose must move back by beta * violation.
	var rv: Dictionary = mutate.call(Rig.make({"on_axis": true, "static_a": true, "limits": LIM}))
	rv.b.position += rv.u * (LIM.x - 0.02 - Rig.measure(rv).s)
	rv.b.reset_published_state()
	rv.world.step()
	if rv.b.linear_velocity.length() > 1.0e-6:
		out.append("the violated limit injected physical velocity (%s m/s)" % rv.b.linear_velocity.length())
	if absf(Rig.measure(rv).s - (LIM.x - 0.02 * (1.0 - rv.world.config.baumgarte_factor))) > 1.0e-5:
		out.append("the violated limit was not corrected by beta * violation (s = %s)" % Rig.measure(rv).s)
	return out


func test_f1_the_criteria_accept_the_real_joint() -> void:
	var v: PackedStringArray = _dof_violations(func(rig): return rig)
	check(v.is_empty(), "the real joint satisfies every degree-of-freedom criterion: %s" % v)
	var vl: PackedStringArray = _limit_violations(func(rig): return rig)
	check(vl.is_empty(), "the real joint satisfies every limit criterion: %s" % vl)


func test_f2_a_missing_prismatic_constraint_is_detected() -> void:
	var v: PackedStringArray = _dof_violations(func(rig):
		rig.world.remove_constraint(rig.joint)
		return rig)
	check(v.size() >= 5, "a missing joint violates the lateral and rotational criteria (%d messages: %s)" % [v.size(), v])


func test_f3_a_prismatic_axis_tilted_away_from_the_configured_axis_is_detected() -> void:
	var v: PackedStringArray = _dof_violations(func(rig):
		var old = rig.joint
		rig.world.remove_constraint(old)
		var tilted: Vector3 = Quaternion(rig.ref, deg_to_rad(8.0)) * rig.u
		var j = PrismaticJoint.from_world(old.body_a, old.body_b, rig.point, tilted, Rig.perp_reference(tilted))
		rig.world.add_constraint(j)
		rig.joint = j
		return rig)
	check(not v.is_empty(), "an axis tilted by 8 degrees from the configured fork axis is detected (%s)" % v)
	var found_axial: bool = false
	for m in v:
		if m.contains("axial translation is not free") or m.contains("lateral translation"):
			found_axial = true
	check(found_axial, "...through the axial/lateral criteria judged against the CONFIGURED axis")


func test_f4_a_prismatic_joint_that_permits_rotation_is_detected() -> void:
	var v_twist: PackedStringArray = _dof_violations(func(rig): return _swap(rig, NoTwist))
	check(not v_twist.is_empty() and " ".join(v_twist).contains("rotation"), "a joint without the twist row is detected (%s)" % v_twist)
	var v_axis: PackedStringArray = _dof_violations(func(rig): return _swap(rig, NoAxisRows))
	check(not v_axis.is_empty() and " ".join(v_axis).contains("rotation"), "a joint without the axis-alignment rows is detected (%s)" % v_axis)
	# Control: the swap helper itself, with the real script, passes (the detection is not an artifact of the swap).
	var v_ok: PackedStringArray = _dof_violations(func(rig): return _swap(rig, PrismaticJoint))
	check(v_ok.is_empty(), "the swap helper with the unmodified script is accepted (%s)" % v_ok)


func test_f5_a_limit_row_with_reversed_sign_is_detected() -> void:
	var v: PackedStringArray = _limit_violations(func(rig): return _swap(rig, ReversedLimitSign))
	check(v.size() >= 2, "reversed limit rows fail to hold the limits (%d messages: %s)" % [v.size(), v])


func test_f6_a_split_limit_correction_applied_to_physical_velocity_is_detected() -> void:
	var v: PackedStringArray = _limit_violations(func(rig): return _swap(rig, SplitOnPhysicalVelocity))
	var text: String = " ".join(v)
	check(text.contains("injected physical velocity"), "the physical-velocity push-out is detected (%s)" % v)
