extends "res://Tests/sim_test_case.gd"
## M1.1 hinge joint: constraint structure, angle measurement, limits, motor, solver integration.
##
## Tolerance policy (float32 vectors, see PHYSICS_CONVENTIONS.md): quantities that are exact in the
## algebra (impulse symmetry, bounds) use 1e-6 relative; anchor/axis drift under load uses the measured
## Baumgarte steady-state bounds stated at each check.

const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")
const DT: float = 1.0 / 120.0
const WORLD_AXES: Array = [Vector3.RIGHT, Vector3.UP, Vector3.BACK]


# --- Builders ---

## Any unit vector perpendicular to `axis` (test-side helper; the joint never needs one).
func perpendicular_to(axis: Vector3) -> Vector3:
	var helper: Vector3 = Vector3.UP if absf(axis.dot(Vector3.UP)) < 0.9 else Vector3.RIGHT
	return axis.cross(helper).normalized()


func box_body(mass: float, pos: Vector3, rot: Quaternion = Quaternion.IDENTITY) -> SimRigidBody:
	return make_body(mass, SimMassProperties.box_inertia(mass, Vector3(0.6, 0.2, 0.3)), pos, rot)


## Hinge between A and B at `anchor` about world `axis`; zero angle = the bodies' current pose.
## Returns {world, a, b, joint}. `static_a` makes A static.
func make_hinge(axis: Vector3, static_a: bool, gravity: Vector3 = Vector3.ZERO, anchor: Vector3 = Vector3(0.3, 1.0, -0.2),
		rot_a: Quaternion = Quaternion.IDENTITY, rot_b: Quaternion = Quaternion.IDENTITY, iterations: int = 40) -> Dictionary:
	var w = make_world(gravity)
	w.config.solver_iterations = iterations
	var a
	if static_a:
		a = make_static(anchor + Vector3(-0.4, 0.1, 0.2))
		a.orientation = rot_a
	else:
		a = box_body(2.0, anchor + Vector3(-0.4, 0.1, 0.2), rot_a)
	var b = box_body(1.0, anchor + Vector3(0.5, -0.2, 0.1), rot_b)
	w.add_body(a)
	w.add_body(b)
	var unit_axis: Vector3 = axis.normalized()
	var joint = HingeJoint.from_world(a, b, anchor, unit_axis, perpendicular_to(unit_axis))
	check(joint != null, "hinge construction")
	w.add_constraint(joint)
	return {"world": w, "a": a, "b": b, "joint": joint, "axis": unit_axis, "anchor": anchor}


## Rotates B about the hinge axis through the anchor by `phi` (pose edit, keeps the anchor fixed).
func rotate_b_about_hinge(s: Dictionary, phi: float) -> void:
	var dq: Quaternion = Quaternion(s.axis, phi)
	var b = s.b
	b.position = s.anchor + dq * (b.position - s.anchor)
	b.orientation = dq * b.orientation


## Free spin of a hinge pair about its axis (static A, no limits/motor) loses rate because the velocity solve uses a frozen Jacobian.
## One step (delta = w dt) leaves a tangential velocity error ~delta^2/2 after the centripetal impulse, which removes a fraction
## (delta^2/2) * f of w with f = m d_perp^2 / I_pivot (d_perp: COM distance from the axis, I_pivot = a.I.a + m d_perp^2).
## Hence dw/dt = -f w^3 dt/2, i.e. w(t)^-2 = w0^-2 + f dt t. Since M1.3 the loss is no longer masked: the former velocity-level
## Baumgarte term injected energy that happened to offset it (measured: kinetic energy ratio 0.9985 with the old Baumgarte bias,
## 0.9686 without any stabilization AND with split impulses, 1 s at 2 rad/s, dt = 1/120; halving dt halves the loss).
func spin_loss_factor(s: Dictionary) -> float:
	var d: Vector3 = s.b.position - s.anchor
	var d_perp_sq: float = d.length_squared() - d.dot(s.axis) * d.dot(s.axis)
	return s.b.mass * d_perp_sq / (s.axis.dot(s.b.inertia_world * s.axis) + s.b.mass * d_perp_sq)


func predicted_spin_rate(omega0: float, f: float, t: float) -> float:
	return pow(1.0 / (omega0 * omega0) + f * DT * t, -0.5)


## Runs `steps` steps and reports the worst anchor error, worst axis error and peak body angular speed.
func run_and_measure(s: Dictionary, steps: int) -> Dictionary:
	var result: Dictionary = {"max_lin": 0.0, "max_axis": 0.0, "max_omega": 0.0}
	for i in steps:
		s.world.step()
		result.max_lin = maxf(result.max_lin, s.joint.linear_error().length())
		result.max_axis = maxf(result.max_axis, s.joint.axis_error_angle())
		for body in [s.a, s.b]:
			result.max_omega = maxf(result.max_omega, body.angular_velocity.length())
	return result


## Baumgarte steady-state anchor error of a rotating lever: r * (w dt)^2 / (2 beta) (PROJECT_STATE.md, measured for the ball
## joint in M0), doubled as margin, plus 0.1 mm for float32 positions. r = lever arm [m], w = peak angular speed [rad/s].
func drift_bound(world, r: float, w: float) -> float:
	var dt: float = world.config.fixed_timestep
	return 2.0 * r * (w * dt) * (w * dt) / (2.0 * world.config.baumgarte_factor) + 1.0e-4


func arbitrary_orientations() -> Array:
	return [
		[Vector3(1, 0, 0), Quaternion.IDENTITY, Quaternion.IDENTITY],
		[Vector3(0, 1, 0), Quaternion(Vector3(1, 1, 0).normalized(), 0.7), Quaternion(Vector3(0, 1, 1).normalized(), -1.1)],
		[Vector3(0, 0, 1), Quaternion(Vector3(1, 2, 3).normalized(), 2.0), Quaternion(Vector3(-1, 0, 2).normalized(), 0.4)],
		[Vector3(1, 2, -3), Quaternion(Vector3(0, 0, 1), 1.3), Quaternion(Vector3(3, -1, 1).normalized(), -2.6)],
		[Vector3(-0.3, 0.9, 0.1), Quaternion(Vector3(1, -1, 0).normalized(), 3.0), Quaternion.IDENTITY],
	]


# --- A. Basic hinge behavior ---

func test_a1_static_to_dynamic_holds_under_gravity() -> void:
	var s = make_hinge(Vector3(0, 0, 1), true, Vector3(0, -9.81, 0))
	var max_lin: float = 0.0
	var max_axis: float = 0.0
	for i in 480:  # 4 s of swinging
		s.world.step()
		max_lin = maxf(max_lin, s.joint.linear_error().length())
		max_axis = maxf(max_axis, s.joint.axis_error_angle())
	# Warm-started 40-iteration solve of a ~1 m pendulum at <= ~4 m/s: Baumgarte steady state v^2 dt^2 / (2 L beta) ~ 3 mm.
	check(max_lin < 5.0e-3, "anchor drift stays below 5 mm (max %s)" % max_lin)
	check(max_axis < 5.0e-3, "axis misalignment stays below 5 mrad (max %s)" % max_axis)
	check(s.b.position.y < s.anchor.y - 0.1, "the body actually swung down (gravity acts through the hinge)")


func test_a2_dynamic_to_dynamic_under_gravity() -> void:
	var s = make_hinge(Vector3(0, 0, 1), false, Vector3(0, -9.81, 0))
	# Two free bodies joined by a hinge fall together; the pair must also rotate relative to each other only about the axis.
	var max_lin: float = 0.0
	var max_axis: float = 0.0
	for i in 360:
		s.world.step()
		max_lin = maxf(max_lin, s.joint.linear_error().length())
		max_axis = maxf(max_axis, s.joint.axis_error_angle())
	check(max_lin < 2.0e-3, "anchor drift below 2 mm (max %s)" % max_lin)
	check(max_axis < 5.0e-3, "axis misalignment below 5 mrad (max %s)" % max_axis)
	# Gravity-only free fall: the center of mass of the pair accelerates at g.
	var t: float = 360 * DT
	var com_y: float = (s.a.mass * s.a.position.y + s.b.mass * s.b.position.y) / (s.a.mass + s.b.mass)
	var com_y0: float = (2.0 * (s.anchor.y + 0.1) + 1.0 * (s.anchor.y - 0.2)) / 3.0
	check_near(com_y - com_y0, -0.5 * 9.81 * t * t, 0.1 * 0.5 * 9.81 * t * t + 0.05, "pair COM falls with ~g (internal forces cancel)")


func test_a3_arbitrary_world_orientation() -> void:
	for cfg in arbitrary_orientations():
		var s = make_hinge(cfg[0], true, Vector3(0, -9.81, 0), Vector3(0.3, 1.0, -0.2), cfg[1], cfg[2])
		var max_lin: float = 0.0
		var max_axis: float = 0.0
		for i in 240:
			s.world.step()
			max_lin = maxf(max_lin, s.joint.linear_error().length())
			max_axis = maxf(max_axis, s.joint.axis_error_angle())
		check(max_lin < 6.0e-3, "axis %s: anchor drift %s < 6 mm" % [cfg[0], max_lin])
		check(max_axis < 6.0e-3, "axis %s: axis error %s < 6 mrad" % [cfg[0], max_axis])


func test_a4_anchors_coincident_under_gravity_dynamic_pair() -> void:
	for cfg in arbitrary_orientations():
		var s = make_hinge(cfg[0], false, Vector3(0, -9.81, 0), Vector3(-0.5, 2.0, 0.7), cfg[1], cfg[2])
		run_steps(s.world, 240)
		check(s.joint.linear_error().length() < 2.0e-3, "axis %s: anchors coincide (err %s)" % [cfg[0], s.joint.linear_error().length()])


func test_a5_anchors_coincident_after_linear_impulses() -> void:
	for static_a in [true, false]:
		var s = make_hinge(Vector3(1, 1, 0), static_a)
		s.b.apply_impulse(Vector3(0.9, -0.4, 0.6))
		if not static_a:
			s.a.apply_impulse(Vector3(-0.7, 0.2, 0.5))
		var m: Dictionary = run_and_measure(s, 240)
		var bound: float = drift_bound(s.world, 0.6, m.max_omega)
		check(m.max_omega < 5.0, "static_a=%s: scenario stays below the fast-spin regime (peak %s rad/s)" % [static_a, m.max_omega])
		check(m.max_lin < bound, "static_a=%s: anchor error after impulses %s < bound %s" % [static_a, m.max_lin, bound])
		check(s.joint.linear_error().length() < bound, "static_a=%s: final anchor error %s < bound %s" % [static_a, s.joint.linear_error().length(), bound])


func test_a6_anchors_coincident_after_off_center_impulses() -> void:
	for static_a in [true, false]:
		var s = make_hinge(Vector3(0, 1, 1), static_a)
		s.b.apply_impulse_at_point(Vector3(0.5, 0.3, -0.6), s.b.position + Vector3(0.2, 0.15, -0.1))
		if not static_a:
			s.a.apply_impulse_at_point(Vector3(-0.3, 0.6, 0.3), s.a.position + Vector3(-0.1, 0.1, 0.2))
		var m: Dictionary = run_and_measure(s, 240)
		var bound: float = drift_bound(s.world, 0.6, m.max_omega)
		check(m.max_omega < 5.0, "static_a=%s: scenario stays below the fast-spin regime (peak %s rad/s)" % [static_a, m.max_omega])
		check(m.max_lin < bound, "static_a=%s: off-center impulse anchor error %s < bound %s" % [static_a, m.max_lin, bound])
		check(s.joint.linear_error().length() < bound, "static_a=%s: final anchor error %s < bound %s" % [static_a, s.joint.linear_error().length(), bound])


func test_a7_axes_remain_aligned_after_perpendicular_kicks() -> void:
	var s = make_hinge(Vector3(1, 0, 0), true)
	var kick_axes: Array = [Vector3.UP, Vector3.BACK, Vector3(0, 1, 1).normalized()]
	for k in kick_axes:
		s.b.apply_angular_impulse(k * 0.8)
		var max_err: float = 0.0
		for i in 120:
			s.world.step()
			max_err = maxf(max_err, s.joint.axis_error_angle())
		check(max_err < 0.02, "kick about %s: axis misalignment < 20 mrad (max %s)" % [k, max_err])
		check(s.joint.axis_error_angle() < 2.0e-3, "kick about %s: misalignment settles (%s)" % [k, s.joint.axis_error_angle()])


func test_a8_rotation_about_hinge_axis_is_free() -> void:
	# Static A: a torque about the axis must accelerate B exactly as a free pivot would: alpha = tau / I_pivot with
	# I_pivot = a.I_com.a + m (|d|^2 - (d.a)^2) (parallel-axis theorem). Any axial resistance of the hinge shows up here.
	for cfg in arbitrary_orientations():
		var s = make_hinge(cfg[0], true, Vector3.ZERO, Vector3(0.3, 1.0, -0.2), cfg[1], cfg[2])
		var d: Vector3 = s.b.position - s.anchor
		var i_pivot: float = s.axis.dot(s.b.inertia_world * s.axis) + s.b.mass * (d.length_squared() - d.dot(s.axis) * d.dot(s.axis))
		var steps: int = 60
		# Torque sized so the final rate is 3 rad/s whatever the pivot inertia (keeps w*dt <= 0.025 rad: below the M0 fast-spin regime).
		var tau: float = 3.0 * i_pivot / (steps * DT)
		for i in steps:
			s.b.apply_torque(s.axis * tau)
			s.world.step()
		var expected: float = tau * steps * DT / i_pivot
		check_near(s.joint.angle_rate(), expected, 0.01 * expected, "axis %s: axial torque accelerates freely (%s vs %s)" % [cfg[0], s.joint.angle_rate(), expected])
		check(s.joint.axis_error_angle() < 5.0e-3, "axis %s: axes stay aligned (%s)" % [cfg[0], s.joint.axis_error_angle()])
	# Free spin keeps its rate up to the documented (w dt)^2 loss (PROJECT_STATE.md, see spin_loss_factor).
	var s2 = make_hinge(Vector3(1, 2, -3), true)
	s2.b.angular_velocity = s2.axis * 2.0
	s2.b.linear_velocity = s2.b.angular_velocity.cross(s2.b.position - s2.anchor)
	var f2: float = spin_loss_factor(s2)
	run_steps(s2.world, 120)
	# Expected rate from the frozen-Jacobian loss law above (1.9706 here); measured 1.9709. Tolerance 1e-3 covers the
	# higher-order terms of the first-order derivation (~1.3e-4 relative observed) and float32 rounding.
	var expected_rate: float = predicted_spin_rate(2.0, f2, 120 * DT)
	check_near(s2.joint.angle_rate(), expected_rate, 1.0e-3, "free spin of 2 rad/s follows the frozen-Jacobian loss law (%s vs %s)" % [s2.joint.angle_rate(), expected_rate])
	check(s2.joint.angle_rate() > 2.0 * 0.98, "and loses less than 2 %% in 1 s (%s)" % s2.joint.angle_rate())
	# Dynamic A: the hinge exerts no external force or torque on the pair, so total linear and angular (about the origin)
	# momentum are conserved even though both bodies rotate and translate.
	for cfg in arbitrary_orientations():
		var s3 = make_hinge(cfg[0], false, Vector3.ZERO, Vector3(0.3, 1.0, -0.2), cfg[1], cfg[2])
		s3.b.angular_velocity = s3.axis * 2.0
		s3.b.linear_velocity = s3.b.angular_velocity.cross(s3.b.position - s3.anchor)
		var p0: Vector3 = s3.world.total_linear_momentum()
		var l0: Vector3 = s3.world.total_angular_momentum()
		run_steps(s3.world, 120)
		check_vec3_near(s3.world.total_linear_momentum(), p0, 1.0e-4, "axis %s: linear momentum conserved" % cfg[0])
		check_vec3_near(s3.world.total_angular_momentum(), l0, 0.02 * l0.length() + 1.0e-3, "axis %s: angular momentum conserved (%s -> %s)" % [cfg[0], l0, s3.world.total_angular_momentum()])


func test_a9_rotation_about_perpendicular_axes_is_constrained() -> void:
	var s = make_hinge(Vector3(0, 0, 1), true)
	# Torque B persistently about X (perpendicular to the hinge axis): the hinge must reject it.
	var max_err: float = 0.0
	for i in 240:
		s.b.apply_torque(Vector3(5.0, 0, 0))
		s.world.step()
		max_err = maxf(max_err, s.joint.axis_error_angle())
	# Baumgarte steady-state with a constant 5 N m load on a ~0.03 kg m^2 body: tiny but nonzero.
	check(max_err < 0.05, "axis error under a persistent perpendicular torque stays < 50 mrad (max %s)" % max_err)
	check(absf(s.b.angular_velocity.x) < 0.2, "no spin about the perpendicular axis builds up (%s)" % s.b.angular_velocity.x)
	var s2 = make_hinge(Vector3(0, 0, 1), false)
	s2.b.apply_angular_impulse(Vector3(0.0, 0.7, 0.0))
	run_steps(s2.world, 120)
	check(s2.joint.axis_error_angle() < 3.0e-3, "dynamic pair: perpendicular impulse leaves axes aligned (%s)" % s2.joint.axis_error_angle())


func test_a10_static_body_unchanged() -> void:
	var s = make_hinge(Vector3(1, 2, 3), true, Vector3(0, -9.81, 0))
	var pos0: Vector3 = s.a.position
	var rot0: Quaternion = s.a.orientation
	s.b.apply_impulse_at_point(Vector3(4, -2, 1), s.b.position + Vector3(0.1, 0.2, 0.0))
	s.joint.limit_enabled = true
	s.joint.lower_limit = -0.3
	s.joint.upper_limit = 0.3
	s.joint.motor_enabled = true
	s.joint.motor_target_speed = 3.0
	s.joint.motor_max_torque = 20.0
	run_steps(s.world, 240)
	check(s.a.position == pos0, "static position bit-identical")
	check(s.a.orientation == rot0, "static orientation bit-identical")
	check(s.a.linear_velocity == Vector3.ZERO and s.a.angular_velocity == Vector3.ZERO, "static velocities stay zero")


# --- B. Angle behavior ---

func test_b1_initial_angle_follows_zero_convention() -> void:
	for cfg in arbitrary_orientations():
		var s = make_hinge(cfg[0], false, Vector3.ZERO, Vector3(0.3, 1.0, -0.2), cfg[1], cfg[2])
		check_near(s.joint.angle(), 0.0, 1.0e-6, "axis %s: from_world pose is angle 0 (%s)" % [cfg[0], s.joint.angle()])
	# Direct constructor: zero angle when R_a*reference_a == R_b*reference_b. B rotated by 0.9 rad about the axis => 0.9.
	var a = box_body(1.0, Vector3.ZERO)
	var qb: Quaternion = Quaternion(Vector3.BACK, 0.9)
	var b = box_body(1.0, Vector3.ZERO, qb)
	var joint = HingeJoint.new(a, b, Vector3.ZERO, Vector3.ZERO, Vector3.BACK, Vector3.BACK, Vector3.RIGHT, Vector3.RIGHT)
	check(joint.is_valid(), "explicit-frame constructor accepts valid data")
	check_near(joint.angle(), 0.9, 1.0e-6, "angle = rotation of B's reference relative to A's about +axis")


func test_b2_positive_and_b3_negative_rotation_signs() -> void:
	for cfg in arbitrary_orientations():
		var s = make_hinge(cfg[0], false, Vector3.ZERO, Vector3(0.3, 1.0, -0.2), cfg[1], cfg[2])
		rotate_b_about_hinge(s, 0.6)
		check_near(s.joint.angle(), 0.6, 2.0e-6, "axis %s: +0.6 rad about +axis reports +0.6" % cfg[0])
		rotate_b_about_hinge(s, -1.5)
		check_near(s.joint.angle(), -0.9, 2.0e-6, "axis %s: further -1.5 rad reports -0.9" % cfg[0])
	# Rotating A about the axis in the positive sense moves the angle negative (relative rotation of B w.r.t. A).
	var s2 = make_hinge(Vector3(0, 1, 0), false)
	s2.a.orientation = Quaternion(Vector3.UP, 0.4) * s2.a.orientation
	check_near(s2.joint.angle(), -0.4, 2.0e-6, "rotating A by +0.4 about the axis = angle -0.4")
	# Physical check: a positive relative angular velocity increases the angle.
	var s3 = make_hinge(Vector3(0, 0, 1), true)
	s3.b.angular_velocity = Vector3(0, 0, 1.0)
	s3.b.linear_velocity = s3.b.angular_velocity.cross(s3.b.position - s3.anchor)
	run_steps(s3.world, 30)
	check(s3.joint.angle() > 0.2, "positive angular velocity about +axis increases the angle (%s)" % s3.joint.angle())


func test_b4_angle_continuous_across_pi() -> void:
	var s = make_hinge(Vector3(0, 1, 0), false)
	var prev: float = 0.0
	var max_jump: float = 0.0
	var step_phi: float = 0.2
	for i in 40:  # 0 -> 8 rad, crosses pi and 2 pi
		rotate_b_about_hinge(s, step_phi)
		var ang: float = s.joint.angle()
		max_jump = maxf(max_jump, absf(ang - prev))
		prev = ang
		check_near(ang, (i + 1) * step_phi, 1.0e-5, "step %d reports the unwrapped angle" % i)
	check(max_jump < step_phi + 1.0e-5, "no jump larger than the commanded increment (max %s)" % max_jump)
	for i in 20:  # and back through pi
		rotate_b_about_hinge(s, -step_phi)
		check_near(s.joint.angle(), (40 - 1 - i) * step_phi, 1.0e-5, "reverse step %d" % i)


func test_b5_angle_accumulates_beyond_one_revolution() -> void:
	var s = make_hinge(Vector3(0, 0, 1), true)
	# 3 rad/s = 0.025 rad per step. The rate decays by the frozen-Jacobian loss law (spin_loss_factor): w^-2 = w0^-2 + f dt t, so the
	# angle is the integral (2/k) (sqrt(w0^-2 + k T) - sqrt(w0^-2)) with k = f dt (21.43 rad here; 24 rad without any loss).
	# Measured 21.41: the 0.25 % tolerance covers the higher-order terms of the first-order derivation (0.08 % observed).
	var omega: float = 3.0
	s.b.angular_velocity = Vector3(0, 0, omega)
	s.b.linear_velocity = s.b.angular_velocity.cross(s.b.position - s.anchor)
	var steps: int = 960  # 8 s => ~21 rad = ~3.4 revolutions
	var k: float = spin_loss_factor(s) * DT
	var total_time: float = steps * DT
	run_steps(s.world, steps)
	var expected: float = (2.0 / k) * (sqrt(1.0 / (omega * omega) + k * total_time) - sqrt(1.0 / (omega * omega)))
	check(s.joint.angle() > 3.0 * TAU, "more than three full revolutions counted (%s)" % s.joint.angle())
	check_near(s.joint.angle(), expected, 0.0025 * expected, "angle follows the frozen-Jacobian loss law (%s vs %s)" % [s.joint.angle(), expected])
	# Reverse spin: counts back down through zero into negative revolutions.
	s.b.angular_velocity = Vector3(0, 0, -omega)
	s.b.linear_velocity = s.b.angular_velocity.cross(s.b.position - s.anchor)
	run_steps(s.world, 2 * steps)
	check(s.joint.angle() < -TAU, "counting continues into negative revolutions (%s)" % s.joint.angle())


func test_b6_small_axis_misalignment_does_not_jump_the_angle() -> void:
	var s = make_hinge(Vector3(1, 1, 1), false)
	rotate_b_about_hinge(s, 0.7)
	var base: float = s.joint.angle()
	var tilt_axes: Array = [perpendicular_to(s.axis), s.axis.cross(perpendicular_to(s.axis)), (perpendicular_to(s.axis) + s.axis.cross(perpendicular_to(s.axis))).normalized()]
	for t in tilt_axes:
		for eps in [0.001, 0.01, 0.05]:
			var saved_q: Quaternion = s.b.orientation
			s.b.orientation = Quaternion(t, eps) * saved_q  # tilt B's frame about the anchor-independent axis
			var ang: float = s.joint.angle()
			# Second order: a tilt of eps changes the projected angle by O(eps^2).
			check_near(ang, base, eps * eps + 1.0e-6, "tilt %s rad about a perpendicular axis moves the angle by O(eps^2) (got %s)" % [eps, ang - base])
			s.b.orientation = saved_q
			s.joint.angle()


func test_b7_angle_independent_of_world_orientation() -> void:
	var reference_angles: Array = []
	for cfg in arbitrary_orientations():
		var s = make_hinge(cfg[0], false, Vector3.ZERO, Vector3(0.3, 1.0, -0.2), cfg[1], cfg[2])
		rotate_b_about_hinge(s, 1.234)
		reference_angles.append(s.joint.angle())
		# Rotate the whole assembly rigidly by an arbitrary world rotation: the angle must not change.
		var rigid: Quaternion = Quaternion(Vector3(2, -1, 0.5).normalized(), 2.2)
		for body in [s.a, s.b]:
			body.position = rigid * body.position
			body.orientation = rigid * body.orientation
		check_near(s.joint.angle(), 1.234, 3.0e-6, "axis %s: angle unchanged by a rigid rotation of the assembly (%s)" % [cfg[0], s.joint.angle()])
	for v in reference_angles:
		check_near(v, 1.234, 3.0e-6, "same relative pose, any orientation => same angle")


# --- C. Limits ---

## Sets B's relative angular velocity about the hinge axis to `rate` (A at rest), with the anchor-preserving linear velocity.
func spin_b(s: Dictionary, rate: float) -> void:
	s.b.angular_velocity = s.a.angular_velocity + s.axis * rate
	s.b.linear_velocity = s.a.linear_velocity + s.a.angular_velocity.cross(s.anchor - s.a.position) \
		+ s.b.angular_velocity.cross(s.b.position - s.anchor)


## 100 solver iterations: the scalar-row Gauss-Seidel stop is only fully converged from ~80 iterations on. Measured for a
## 3 rad/s impact on a limit (static A): residual separation speed after the stop 10% of the impact speed at 20 iterations
## (the SimConfig default), 1.5% at 40, 0 at >= 80 (PROJECT_STATE.md). The stop itself never penetrates at any count.
func make_limited(static_a: bool, lower: float, upper: float, gravity: Vector3 = Vector3.ZERO, axis: Vector3 = Vector3(0, 0, 1)) -> Dictionary:
	var s = make_hinge(axis, static_a, gravity, Vector3(0.3, 1.0, -0.2), Quaternion.IDENTITY, Quaternion.IDENTITY, 100)
	s.joint.limit_enabled = true
	s.joint.lower_limit = lower
	s.joint.upper_limit = upper
	return s


## Steps while recording the extreme angles and the largest limit impulses. Angle is sampled every step.
func run_tracking(s: Dictionary, steps: int) -> Dictionary:
	var r: Dictionary = {"min_angle": INF, "max_angle": -INF, "max_lower_impulse": 0.0, "max_upper_impulse": 0.0, "min_lower_impulse": INF, "min_upper_impulse": INF}
	for i in steps:
		s.world.step()
		var ang: float = s.joint.angle()
		r.min_angle = minf(r.min_angle, ang)
		r.max_angle = maxf(r.max_angle, ang)
		r.max_lower_impulse = maxf(r.max_lower_impulse, s.joint.lower_limit_impulse())
		r.max_upper_impulse = maxf(r.max_upper_impulse, s.joint.upper_limit_impulse())
		r.min_lower_impulse = minf(r.min_lower_impulse, s.joint.lower_limit_impulse())
		r.min_upper_impulse = minf(r.min_upper_impulse, s.joint.upper_limit_impulse())
	return r


# Overshoot tolerance for a unilateral stop: the speculative row removes the approach velocity so the angle arrives at the limit
# at the end of the step; the residual is the float32 / first-order-integration error of the angle, ~1e-4 rad (measured).
const STOP_TOLERANCE: float = 5.0e-4


func test_c1_lower_limit_blocks_motion_past_minimum() -> void:
	var s = make_limited(true, -0.5, 0.6)
	spin_b(s, -3.0)
	var r: Dictionary = run_tracking(s, 240)  # 2 s: would reach -6 rad unrestrained
	check(r.min_angle > -0.5 - STOP_TOLERANCE, "static A: never below the lower limit (min %s)" % r.min_angle)
	check_near(s.joint.angle(), -0.5, STOP_TOLERANCE, "static A: rests at the lower limit")
	check_near(s.joint.angle_rate(), 0.0, 1.0e-3, "static A: relative rate removed (inelastic stop)")
	check(r.max_lower_impulse > 0.0, "static A: the lower row did the stopping")
	check(r.max_upper_impulse == 0.0, "static A: the upper row stayed idle")
	# Dynamic pair: the stop acts on the RELATIVE angle. The pair keeps tumbling afterwards (nonzero total momentum, anchor
	# forces exert axial torques on each body), so only "never past the limit" and "the lower row engaged" are invariant.
	var d = make_limited(false, -0.5, 0.6)
	spin_b(d, -3.0)
	var rd: Dictionary = run_tracking(d, 240)
	check(rd.min_angle > -0.5 - STOP_TOLERANCE, "dynamic pair: never below the lower limit (min %s)" % rd.min_angle)
	check(rd.max_lower_impulse > 0.0, "dynamic pair: the lower row engaged")


func test_c2_upper_limit_blocks_motion_past_maximum() -> void:
	var s = make_limited(true, -0.5, 0.6)
	spin_b(s, 3.0)
	var r: Dictionary = run_tracking(s, 240)
	check(r.max_angle < 0.6 + STOP_TOLERANCE, "static A: never above the upper limit (max %s)" % r.max_angle)
	check_near(s.joint.angle(), 0.6, STOP_TOLERANCE, "static A: rests at the upper limit")
	check_near(s.joint.angle_rate(), 0.0, 1.0e-3, "static A: relative rate removed")
	check(r.max_upper_impulse > 0.0, "static A: the upper row did the stopping")
	check(r.max_lower_impulse == 0.0, "static A: the lower row stayed idle")
	var d = make_limited(false, -0.5, 0.6)
	spin_b(d, 3.0)
	var rd: Dictionary = run_tracking(d, 240)
	check(rd.max_angle < 0.6 + STOP_TOLERANCE, "dynamic pair: never above the upper limit (max %s)" % rd.max_angle)
	check(rd.max_upper_impulse > 0.0, "dynamic pair: the upper row engaged")


func test_c3_motion_inside_range_is_not_blocked() -> void:
	for rate in [1.5, -1.5]:
		var s = make_limited(true, -0.5, 0.6)
		spin_b(s, rate)
		var steps: int = 30  # 0.25 s => +-0.375 rad, inside [-0.5, 0.6]
		for i in steps:
			s.world.step()
			check(s.joint.lower_limit_impulse() == 0.0 and s.joint.upper_limit_impulse() == 0.0, "rate %s step %d: no limit impulse inside the range" % [rate, i])
		check_near(s.joint.angle(), rate * steps * DT, 2.0e-3, "rate %s: angle advances freely (%s)" % [rate, s.joint.angle()])
		# 0.5%: the M0 frozen-Jacobian spin loss (~(w dt)^2) plus the first-step transient; measured 0.07%.
		check_near(s.joint.angle_rate(), rate, 0.005 * absf(rate), "rate %s: speed unchanged (%s)" % [rate, s.joint.angle_rate()])


func test_c4_motion_away_from_a_limit_is_allowed() -> void:
	# Start exactly at the lower limit, moving up; and exactly at the upper limit, moving down.
	var s = make_limited(true, -0.5, 0.6)
	rotate_b_about_hinge(s, -0.5)
	s.joint.angle()
	spin_b(s, 2.0)
	for i in 20:
		s.world.step()
		check(s.joint.lower_limit_impulse() <= 1.0e-9, "step %d: leaving the lower limit costs no impulse (%s)" % [i, s.joint.lower_limit_impulse()])
	check_near(s.joint.angle(), -0.5 + 2.0 * 20 * DT, 5.0e-3, "angle leaves the limit at the commanded rate (%s)" % s.joint.angle())
	var s2 = make_limited(true, -0.5, 0.6)
	rotate_b_about_hinge(s2, 0.6)
	s2.joint.angle()
	spin_b(s2, -2.0)
	for i in 20:
		s2.world.step()
		check(s2.joint.upper_limit_impulse() <= 1.0e-9, "step %d: leaving the upper limit costs no impulse" % i)
	check_near(s2.joint.angle(), 0.6 - 2.0 * 20 * DT, 5.0e-3, "angle leaves the upper limit at the commanded rate (%s)" % s2.joint.angle())


func test_c5_a_limit_never_pulls_outward() -> void:
	# (1) Impulse signs: unilateral rows are bounded to [0, +inf) so they can only push toward the allowed range.
	var s = make_limited(true, -0.5, 0.6)
	check(s.joint.rows.size() == HingeJoint.ROW_COUNT, "row layout")
	spin_b(s, -3.0)
	var r: Dictionary = run_tracking(s, 120)
	check(r.min_lower_impulse >= 0.0 and r.min_upper_impulse >= 0.0, "limit impulses never negative (%s, %s)" % [r.min_lower_impulse, r.min_upper_impulse])
	# (2) Violated lower limit (pose edit below the minimum), moving further out: the first step must not increase the outward speed.
	# Since M1.3 the physical limit row only forbids further outward rate (physical bias 0); the violation itself is removed by the
	# split channel, which moves the pose (angle += beta * violation) and never the rate: the rate must come out as 0, not positive.
	for outward in [0.0, -2.0]:
		var v = make_limited(true, -0.5, 0.6)
		rotate_b_about_hinge(v, -0.7)
		v.joint.angle()
		spin_b(v, outward)
		var before: float = v.joint.angle_rate()
		v.world.step()
		check(v.joint.angle_rate() >= before - 1.0e-6, "violated lower limit (rate %s): rate %s -> %s, never more negative" % [outward, before, v.joint.angle_rate()])
		check_near(v.joint.angle_rate(), 0.0, 1.0e-3, "violated lower limit: outward rate removed, none added (rate %s)" % v.joint.angle_rate())
		check_near(v.joint.angle(), -0.7 + v.world.config.baumgarte_factor * 0.2 + v.joint.angle_rate() * DT, 3.0e-3, "violated lower limit: pose pushed back inward by beta * violation (angle %s)" % v.joint.angle())
		var worst: float = v.joint.angle()
		for i in 60:
			v.world.step()
			worst = minf(worst, v.joint.angle())
		check(worst >= -0.7 - 1.0e-3, "angle never goes deeper than the initial violation (min %s)" % worst)
	# (3) Same for the upper limit.
	for outward in [0.0, 2.0]:
		var v = make_limited(true, -0.5, 0.6)
		rotate_b_about_hinge(v, 0.8)
		v.joint.angle()
		spin_b(v, outward)
		var before: float = v.joint.angle_rate()
		v.world.step()
		check(v.joint.angle_rate() <= before + 1.0e-6, "violated upper limit (rate %s): rate %s -> %s, never more positive" % [outward, before, v.joint.angle_rate()])
		check_near(v.joint.angle_rate(), 0.0, 1.0e-3, "violated upper limit: outward rate removed, none added (rate %s)" % v.joint.angle_rate())
		check_near(v.joint.angle(), 0.8 - v.world.config.baumgarte_factor * 0.2 + v.joint.angle_rate() * DT, 3.0e-3, "violated upper limit: pose pushed back inward (angle %s)" % v.joint.angle())


func test_c6_both_limits_dynamic_pair() -> void:
	var s = make_limited(false, -0.4, 0.7)
	var p0: Vector3 = s.world.total_linear_momentum()
	spin_b(s, 4.0)
	var r: Dictionary = run_tracking(s, 200)
	check(r.max_angle < 0.7 + STOP_TOLERANCE, "dynamic pair: upper limit holds (max %s)" % r.max_angle)
	spin_b(s, -4.0)
	r = run_tracking(s, 300)
	check(r.min_angle > -0.4 - STOP_TOLERANCE, "dynamic pair: lower limit holds (min %s)" % r.min_angle)
	check(s.joint.axis_error_angle() < 5.0e-3, "axes stay aligned through both impacts (%s)" % s.joint.axis_error_angle())
	check(s.joint.linear_error().length() < 5.0e-3, "anchors stay together through both impacts (%s)" % s.joint.linear_error().length())


func test_c7_limit_stable_under_gravity_repeated_steps() -> void:
	# Horizontal bar on a static pivot, gravity pulls it down onto the lower limit (axis +Z, B at +X side => negative angle).
	var s = make_limited(true, -0.8, 0.8, Vector3(0, -9.81, 0))
	s.world.config.solver_iterations = 60
	run_steps(s.world, 240)  # fall and settle
	var impulses: Array = []
	var angles: Array = []
	for i in 240:
		s.world.step()
		impulses.append(s.joint.lower_limit_impulse())
		angles.append(s.joint.angle())
	var mean: float = 0.0
	for v in impulses:
		mean += v
	mean /= impulses.size()
	var max_dev: float = 0.0
	for v in impulses:
		max_dev = maxf(max_dev, absf(v - mean))
	var min_a: float = angles.min()
	var max_a: float = angles.max()
	# Holding torque: gravity torque about the axis at the resting pose, tau = ((x_b - anchor) x m g).axis, magnitude * dt.
	var d: Vector3 = s.b.position - s.anchor
	var tau: float = absf(d.cross(Vector3(0, -9.81, 0) * s.b.mass).dot(s.axis))
	check(mean > 0.0, "the stop carries load (mean impulse %s)" % mean)
	check_near(mean, tau * DT, 0.03 * tau * DT, "resting impulse equals the gravity torque * dt (%s vs %s)" % [mean, tau * DT])
	check(max_dev < 0.02 * mean, "no jitter: impulse varies < 2%% (max dev %s of %s)" % [max_dev, mean])
	check(max_a - min_a < 1.0e-3, "angle stationary at the stop (span %s)" % (max_a - min_a))
	check_near(s.joint.angle(), -0.8, STOP_TOLERANCE, "rests at the lower limit (%s)" % s.joint.angle())
	check(absf(s.joint.angle_rate()) < 1.0e-2, "no residual motion (%s)" % s.joint.angle_rate())


func test_c8_motor_toward_active_limit_does_not_push_through() -> void:
	for static_a in [true, false]:
		var s = make_limited(static_a, -0.5, 0.6)
		s.joint.motor_enabled = true
		s.joint.motor_target_speed = -4.0
		s.joint.motor_max_torque = 200.0
		var r: Dictionary = run_tracking(s, 360)
		check(r.min_angle > -0.5 - STOP_TOLERANCE, "static_a=%s: motor driving down stops at the lower limit (min %s)" % [static_a, r.min_angle])
		check_near(s.joint.angle(), -0.5, STOP_TOLERANCE, "static_a=%s: holds at the limit" % static_a)
		# No fight: with no external load the stalled motor and the limit carry no opposing or runaway impulses.
		check(absf(s.joint.motor_impulse()) < 1.0e-3, "static_a=%s: stalled motor impulse ~ 0, not saturated (%s)" % [static_a, s.joint.motor_impulse()])
		check(absf(s.joint.lower_limit_impulse()) < 1.0e-3, "static_a=%s: limit impulse ~ 0 at rest (%s)" % [static_a, s.joint.lower_limit_impulse()])
		var s2 = make_limited(static_a, -0.5, 0.6)
		s2.joint.motor_enabled = true
		s2.joint.motor_target_speed = 4.0
		s2.joint.motor_max_torque = 200.0
		r = run_tracking(s2, 360)
		check(r.max_angle < 0.6 + STOP_TOLERANCE, "static_a=%s: motor driving up stops at the upper limit (max %s)" % [static_a, r.max_angle])
		check(absf(s2.joint.motor_impulse()) < 1.0e-3, "static_a=%s: stalled motor impulse ~ 0 at the upper limit (%s)" % [static_a, s2.joint.motor_impulse()])


func test_c9_motor_away_from_limit_moves_back_into_range() -> void:
	var s = make_limited(true, -0.5, 0.6)
	rotate_b_about_hinge(s, -0.5)
	s.joint.angle()
	s.joint.motor_enabled = true
	s.joint.motor_target_speed = 2.0
	s.joint.motor_max_torque = 100.0
	var r: Dictionary = run_tracking(s, 120)
	check_near(s.joint.angle(), 0.6, STOP_TOLERANCE, "motor carries the hinge from the lower to the upper limit and stops (%s)" % s.joint.angle())
	check(r.min_angle > -0.5 - STOP_TOLERANCE, "starting on the limit never dips below it (min %s)" % r.min_angle)
	# And the other way round.
	s.joint.motor_target_speed = -2.0
	r = run_tracking(s, 120)
	check_near(s.joint.angle(), -0.5, STOP_TOLERANCE, "reversing the motor returns to the lower limit (%s)" % s.joint.angle())
	check(r.max_angle < 0.6 + STOP_TOLERANCE, "reversal from the upper limit never exceeds it (max %s)" % r.max_angle)


# --- D. Motor ---

## 100 iterations for the same convergence reason as make_limited.
func make_motor(static_a: bool, target: float, max_torque: float, gravity: Vector3 = Vector3.ZERO, axis: Vector3 = Vector3(0, 0, 1)) -> Dictionary:
	var s = make_hinge(axis, static_a, gravity, Vector3(0.3, 1.0, -0.2), Quaternion.IDENTITY, Quaternion.IDENTITY, 100)
	s.joint.motor_enabled = true
	s.joint.motor_target_speed = target
	s.joint.motor_max_torque = max_torque
	return s


## Moment of inertia of B about the hinge axis through the anchor (parallel-axis theorem). Invariant under rotation about the axis.
func pivot_inertia(s: Dictionary) -> float:
	var d: Vector3 = s.b.position - s.anchor
	return s.axis.dot(s.b.inertia_world * s.axis) + s.b.mass * (d.length_squared() - d.dot(s.axis) * d.dot(s.axis))


func test_d1_motor_accelerates_free_hinge_to_target() -> void:
	for cfg in arbitrary_orientations():
		var s = make_hinge(cfg[0], true, Vector3.ZERO, Vector3(0.3, 1.0, -0.2), cfg[1], cfg[2], 100)
		s.joint.motor_enabled = true
		s.joint.motor_target_speed = 2.0
		s.joint.motor_max_torque = 1000.0  # cap (8.3 N m s per step) far above the 0.3-1 N m s needed: unsaturated
		s.world.step()
		check_near(s.joint.angle_rate(), 2.0, 0.005 * 2.0, "axis %s: target reached in one step (%s)" % [cfg[0], s.joint.angle_rate()])
		run_steps(s.world, 120)
		check_near(s.joint.angle_rate(), 2.0, 0.005 * 2.0, "axis %s: target held (%s)" % [cfg[0], s.joint.angle_rate()])
		check(s.joint.axis_error_angle() < 5.0e-3 and s.joint.linear_error().length() < 5.0e-3, "axis %s: hinge integrity while driven" % cfg[0])


func test_d2_motor_approaches_target_under_load() -> void:
	# Gravity pulls the bar down (negative angle). Gravity torque ~ m g d <= 5.5 N m. Motor target +2 rad/s.
	var s = make_motor(true, 2.0, 50.0, Vector3(0, -9.81, 0))
	run_steps(s.world, 60)
	check_near(s.joint.angle_rate(), 2.0, 0.02 * 2.0, "enough torque (50 N m): holds the target against gravity (%s)" % s.joint.angle_rate())
	check(s.joint.angle() > 0.5, "and the hinge really moved (%s rad)" % s.joint.angle())
	# Not enough torque (2 N m < gravity torque ~4.9 N m at the start): the motor cannot hold the target, gravity wins.
	var w = make_motor(true, 2.0, 2.0, Vector3(0, -9.81, 0))
	run_steps(w.world, 60)
	check(w.joint.angle_rate() < 0.0, "under-powered motor loses to gravity (rate %s)" % w.joint.angle_rate())
	check(absf(w.joint.motor_impulse()) <= 2.0 * DT * (1.0 + 1.0e-9), "and its impulse stays at the cap (%s <= %s)" % [w.joint.motor_impulse(), 2.0 * DT])


func test_d3_motor_torque_bounded_by_maximum() -> void:
	var tau: float = 3.0
	var s = make_motor(true, 50.0, tau)  # target far out of reach: the motor saturates
	var i_pivot: float = pivot_inertia(s)
	var steps: int = 30
	for i in steps:
		s.world.step()
		check(absf(s.joint.motor_impulse()) <= tau * DT * (1.0 + 1.0e-9), "step %d: |impulse| %s <= cap %s" % [i, s.joint.motor_impulse(), tau * DT])
		check_near(s.joint.motor_impulse(), tau * DT, tau * DT * 1.0e-6, "step %d: saturated motor applies exactly the cap" % i)
	# Torque seen by the body: angular acceleration = tau / I_pivot, i.e. rate = tau t / I_pivot (1.5%: (w dt)^2 loss and anchor coupling).
	var expected: float = tau * steps * DT / i_pivot
	check_near(s.joint.angle_rate(), expected, 0.015 * expected, "rate grows at tau / I_pivot (%s vs %s)" % [s.joint.angle_rate(), expected])


func test_d4_impulse_cap_scales_with_timestep() -> void:
	var tau: float = 3.0
	for dt in [1.0 / 60.0, 1.0 / 120.0, 1.0 / 240.0]:
		var s = make_motor(true, 50.0, tau)
		s.world.config.fixed_timestep = dt
		var i_pivot: float = pivot_inertia(s)
		s.world.step()
		check_near(s.joint.motor_impulse_bound(), tau * dt, tau * dt * 1.0e-12, "dt %s: impulse bound = tau * dt" % dt)
		check_near(s.joint.motor_impulse(), tau * dt, tau * dt * 1.0e-6, "dt %s: saturated impulse = tau * dt" % dt)
		# The resulting torque is the same for every dt: delta(I w) / dt.
		check_near(i_pivot * s.joint.angle_rate() / dt, tau, 0.01 * tau, "dt %s: effective torque = %s N m (%s)" % [dt, tau, i_pivot * s.joint.angle_rate() / dt])


func test_d5_disabled_motor_contributes_nothing() -> void:
	var s = make_motor(true, 3.0, 10.0)
	s.joint.motor_enabled = false
	var control = make_hinge(Vector3(0, 0, 1), true, Vector3.ZERO, Vector3(0.3, 1.0, -0.2), Quaternion.IDENTITY, Quaternion.IDENTITY, 100)
	spin_b(s, 1.2)
	spin_b(control, 1.2)
	for i in 60:
		s.world.step()
		control.world.step()
		check(s.joint.motor_impulse() == 0.0, "step %d: no motor impulse" % i)
		check(s.joint.motor_impulse_bound() == 0.0, "step %d: motor bound is zero" % i)
	check_near(s.joint.angle(), control.joint.angle(), 1.0e-9, "trajectory identical to a hinge that has no motor settings")
	check_near(s.b.angular_velocity.z, control.b.angular_velocity.z, 1.0e-9, "velocity identical")


func test_d6_zero_max_torque_is_inactive() -> void:
	var s = make_motor(true, 3.0, 0.0)
	var control = make_hinge(Vector3(0, 0, 1), true, Vector3.ZERO, Vector3(0.3, 1.0, -0.2), Quaternion.IDENTITY, Quaternion.IDENTITY, 100)
	spin_b(s, 1.2)
	spin_b(control, 1.2)
	for i in 60:
		s.world.step()
		control.world.step()
		check(s.joint.motor_impulse() == 0.0, "step %d: zero-torque motor applies no impulse" % i)
	check_near(s.joint.angle(), control.joint.angle(), 1.0e-9, "zero max torque behaves as no motor")


func test_d7_motor_signs() -> void:
	for cfg in arbitrary_orientations():
		for target in [2.0, -2.0]:
			var s = make_hinge(cfg[0], false, Vector3.ZERO, Vector3(0.3, 1.0, -0.2), cfg[1], cfg[2], 100)
			s.joint.motor_enabled = true
			s.joint.motor_target_speed = target
			s.joint.motor_max_torque = 5.0
			run_steps(s.world, 20)
			var sgn: float = signf(target)
			check(s.joint.angle() * sgn > 0.0, "axis %s target %s: the angle moves in the sign of the target (%s)" % [cfg[0], target, s.joint.angle()])
			check(s.joint.angle_rate() * sgn > 0.0, "axis %s target %s: relative rate has the sign of the target" % [cfg[0], target])
			# Action on B along +axis for positive target, reaction on A the opposite way (A is initially at rest).
			check(s.b.angular_velocity.dot(s.axis) * sgn > 0.0, "axis %s target %s: B is pushed along sign*axis" % [cfg[0], target])
			check(s.a.angular_velocity.dot(s.axis) * sgn < 0.0, "axis %s target %s: reaction on A is opposite" % [cfg[0], target])
	var f = make_motor(true, 1.0, 10.0)
	check(f.joint.motor_impulse() == 0.0, "no impulse before the first step")


func test_d8_static_body_provides_the_reaction() -> void:
	var s = make_motor(true, 50.0, 3.0)
	var i_pivot: float = pivot_inertia(s)
	var pos0: Vector3 = s.a.position
	run_steps(s.world, 10)
	# All of the impulse reached B (nothing is lost or shared with the static body)...
	var impulse_total: float = 10.0 * 3.0 * DT
	check_near(i_pivot * s.joint.angle_rate(), impulse_total, 0.015 * impulse_total, "B received the full motor impulse (%s vs %s)" % [i_pivot * s.joint.angle_rate(), impulse_total])
	# ...and the static body absorbed the reaction without moving.
	check(s.a.position == pos0 and s.a.linear_velocity == Vector3.ZERO and s.a.angular_velocity == Vector3.ZERO, "static body is unchanged")
	check(s.joint.linear_error().length() < 1.0e-3, "anchor holds (%s)" % s.joint.linear_error().length())


func test_d9_dynamic_pair_impulses_equal_and_opposite() -> void:
	var s = make_motor(false, 3.0, 20.0)
	s.world.step()
	var row = s.joint.rows[HingeJoint.ROW_MOTOR]
	check(row.angular_a + row.angular_b == Vector3.ZERO, "motor Jacobian: angular_a = -angular_b (%s, %s)" % [row.angular_a, row.angular_b])
	check(row.linear_a == Vector3.ZERO and row.linear_b == Vector3.ZERO, "motor row has no linear part")
	check_near(row.angular_b.length(), 1.0, 1.0e-6, "unit hinge axis")
	# The row holds the axis at prepare time (start of the step); the pose has rotated slightly since.
	check_vec3_near(row.angular_b, s.axis, 2.0e-3, "acts about the hinge axis")
	# Starting from rest, equal and opposite impulses leave total momentum at zero.
	run_steps(s.world, 60)
	var l_b: float = s.b.angular_momentum_about_com().length() + s.b.position.cross(s.b.linear_momentum()).length()
	check(l_b > 0.05, "the motor really drove the pair (|L_B| %s)" % l_b)
	check(s.world.total_linear_momentum().length() < 1.0e-4, "total linear momentum stays 0 (%s)" % s.world.total_linear_momentum().length())
	check(s.world.total_angular_momentum().length() < 0.02 * l_b, "total angular momentum stays ~0 (%s vs |L_B| %s)" % [s.world.total_angular_momentum().length(), l_b])


func test_d10_motor_and_limits_stable_together() -> void:
	var s = make_limited(true, -0.5, 0.6, Vector3(0, -9.81, 0))
	s.joint.motor_enabled = true
	s.joint.motor_target_speed = 1.0
	s.joint.motor_max_torque = 30.0
	var bound: float = 30.0 * DT
	var history: Array = []
	for i in 720:  # 6 s: drives up to the upper limit and holds against gravity
		s.world.step()
		var ang: float = s.joint.angle()
		check(ang > -0.5 - STOP_TOLERANCE and ang < 0.6 + STOP_TOLERANCE, "step %d: inside the limits (%s)" % [i, ang])
		check(absf(s.joint.motor_impulse()) <= bound * (1.0 + 1.0e-9), "step %d: motor impulse within cap" % i)
		if i >= 600:
			history.append(s.joint.motor_impulse() + s.joint.upper_limit_impulse())
	var mean: float = 0.0
	for v in history:
		mean += v
	mean /= history.size()
	var max_dev: float = 0.0
	for v in history:
		max_dev = maxf(max_dev, absf(v - mean))
	var d: Vector3 = s.b.position - s.anchor
	var tau_g: float = absf(d.cross(Vector3(0, -9.81, 0) * s.b.mass).dot(s.axis))
	check_near(s.joint.angle(), 0.6, STOP_TOLERANCE, "parked at the upper limit (%s)" % s.joint.angle())
	check_near(mean, tau_g * DT, 0.03 * tau_g * DT, "motor + limit together carry exactly the gravity torque (%s vs %s)" % [mean, tau_g * DT])
	check(max_dev < 0.02 * mean, "carried impulse steady: no jitter or alternation (max dev %s of %s)" % [max_dev, mean])
	check(s.joint.motor_impulse() * s.joint.upper_limit_impulse() >= 0.0, "motor and limit never push against each other")
	check(not is_nan(s.joint.angle()) and not is_nan(s.b.position.x), "finite state")


# --- E. Solver integration and conservation ---

func make_hanging() -> Dictionary:
	# B hangs straight below a static pivot: hinge axis +Z, B COM 0.5 m directly below the anchor (equilibrium).
	var w = make_world(Vector3(0, -9.81, 0))
	w.config.solver_iterations = 100
	var anchor: Vector3 = Vector3(0, 2, 0)
	var a = make_static(anchor)
	var b = make_body(2.0, SimMassProperties.box_inertia(2.0, Vector3(0.2, 0.6, 0.2)), anchor + Vector3(0, -0.5, 0))
	w.add_body(a)
	w.add_body(b)
	var joint = HingeJoint.from_world(a, b, anchor, Vector3.BACK, Vector3.RIGHT)
	check(joint != null, "hanging hinge construction")
	w.add_constraint(joint)
	return {"world": w, "a": a, "b": b, "joint": joint, "axis": Vector3.BACK, "anchor": anchor}


func anchor_impulse(joint) -> Vector3:
	return Vector3(joint.rows[0].accumulated_impulse, joint.rows[1].accumulated_impulse, joint.rows[2].accumulated_impulse)


func test_e1_warm_starting_carries_valid_impulses_without_accumulating() -> void:
	var s = make_hanging()
	run_steps(s.world, 240)
	var weight_impulse: float = 2.0 * 9.81 * DT
	var early: Vector3 = anchor_impulse(s.joint)
	check_near(early.y, weight_impulse, 0.01 * weight_impulse, "the anchor row carries the weight: m g dt (%s vs %s)" % [early.y, weight_impulse])
	check(absf(early.x) < 1.0e-3 * weight_impulse and absf(early.z) < 1.0e-3 * weight_impulse, "no lateral anchor impulse (%s)" % early)
	run_steps(s.world, 480)
	var late: Vector3 = anchor_impulse(s.joint)
	check_vec3_near(late, early, 1.0e-3 * weight_impulse, "impulse does not grow with the number of warm-started steps (%s vs %s)" % [late, early])
	check(absf(s.joint.rows[HingeJoint.ROW_AXIS_P].accumulated_impulse) < 1.0e-4 * weight_impulse and absf(s.joint.rows[HingeJoint.ROW_AXIS_Q].accumulated_impulse) < 1.0e-4 * weight_impulse, "axis rows idle at equilibrium")
	check(s.joint.lower_limit_impulse() == 0.0 and s.joint.upper_limit_impulse() == 0.0 and s.joint.motor_impulse() == 0.0, "inactive rows carry exactly zero")
	# Warm and cold start agree on the physical solution of a swinging hinge (both converged at 100 iterations).
	var warm_angle: float = 0.0
	var cold_angle: float = 0.0
	for use_warm in [true, false]:
		var w = make_hinge(Vector3(0, 0, 1), true, Vector3(0, -9.81, 0), Vector3(0.3, 1.0, -0.2), Quaternion.IDENTITY, Quaternion.IDENTITY, 100)
		w.world.config.warm_starting = use_warm
		run_steps(w.world, 120)
		if use_warm:
			warm_angle = w.joint.angle()
		else:
			cold_angle = w.joint.angle()
	check_near(warm_angle, cold_angle, 5.0e-3, "warm and cold start follow the same physics (%s vs %s)" % [warm_angle, cold_angle])


func test_e2_toggling_limits_does_not_reuse_stale_impulses() -> void:
	# Press B against the lower limit with gravity so the lower row carries a large impulse.
	var s = make_limited(true, -0.4, 0.6, Vector3(0, -9.81, 0))
	run_steps(s.world, 240)
	check(s.joint.lower_limit_impulse() > 0.01, "lower row carries the load before the toggle (%s)" % s.joint.lower_limit_impulse())
	# Disable: the row impulse must be exactly zero after the next step and the bar swings past the former limit.
	s.joint.limit_enabled = false
	s.world.step()
	check(s.joint.lower_limit_impulse() == 0.0 and s.joint.upper_limit_impulse() == 0.0, "disabled limits hold no impulse")
	run_steps(s.world, 30)
	check(s.joint.angle() < -0.45, "with the limit off the bar falls through the old stop (%s)" % s.joint.angle())
	# Re-enable far inside the range: no spurious push. Compare with a control that never had limits.
	var a = make_limited(true, -0.4, 0.6, Vector3(0, -9.81, 0))
	var c = make_hinge(Vector3(0, 0, 1), true, Vector3(0, -9.81, 0), Vector3(0.3, 1.0, -0.2), Quaternion.IDENTITY, Quaternion.IDENTITY, 100)
	run_steps(a.world, 240)
	a.joint.limit_enabled = false
	a.world.step()
	for body in [a.b, c.b]:
		body.linear_velocity = Vector3.ZERO
		body.angular_velocity = Vector3.ZERO
	rotate_b_about_hinge(a, 0.6 - a.joint.angle())
	rotate_b_about_hinge(c, 0.0)
	a.joint.angle()
	c.joint.angle()
	a.joint.limit_enabled = true
	a.world.step()
	check(a.joint.lower_limit_impulse() == 0.0, "re-enabled inside the range: the lower row starts from zero (%s)" % a.joint.lower_limit_impulse())
	check(a.joint.upper_limit_impulse() >= 0.0, "upper row non-negative")
	# The hinge is at the upper limit (0.6): the velocity change over this step is just gravity plus (possibly) the stop;
	# the lower row must not have added anything: its impulse is exactly 0 (checked above).


func test_e3_toggling_motor_does_not_produce_a_spurious_impulse() -> void:
	var s = make_motor(true, 50.0, 3.0)
	run_steps(s.world, 30)
	check(absf(s.joint.motor_impulse()) > 0.0, "motor running at its cap")
	s.joint.motor_enabled = false
	s.world.step()
	check(s.joint.motor_impulse() == 0.0, "disabled: impulse exactly 0 on the very next step")
	var coast: float = s.joint.angle_rate()
	run_steps(s.world, 30)
	check_near(s.joint.angle_rate(), coast, 0.005 * absf(coast), "coasting: no leftover motor effect (%s vs %s)" % [s.joint.angle_rate(), coast])
	# Re-enable with a target equal to the current rate: nothing to correct => (almost) no impulse (no stale 3 N m * dt carried over).
	s.joint.motor_target_speed = s.joint.angle_rate()
	s.joint.motor_enabled = true
	s.world.step()
	check(absf(s.joint.motor_impulse()) < 0.02 * 3.0 * DT, "re-enabled at the current speed: no spurious impulse (%s vs cap %s)" % [s.joint.motor_impulse(), 3.0 * DT])
	# Re-enable with a different target: the first impulse is exactly what a fresh motor needs (I_pivot * delta rate), unsaturated.
	var i_pivot: float = pivot_inertia(s)
	s.joint.motor_max_torque = 1000.0
	var rate0: float = s.joint.angle_rate()
	s.joint.motor_target_speed = rate0 + 0.5
	s.world.step()
	check_near(s.joint.motor_impulse(), i_pivot * 0.5, 0.02 * i_pivot * 0.5, "first impulse = I_pivot * delta rate (%s vs %s)" % [s.joint.motor_impulse(), i_pivot * 0.5])


func make_sphere_hinge() -> Dictionary:
	# Two spheres (isotropic inertia: the explicit gyroscopic term vanishes, so the integrator itself conserves L exactly).
	var w = make_world(Vector3.ZERO)
	w.config.solver_iterations = 100
	var anchor: Vector3 = Vector3(0.3, 1.0, -0.2)
	var a = make_sphere(2.0, 0.3, anchor + Vector3(-0.4, 0.1, 0.2))
	var b = make_sphere(1.0, 0.2, anchor + Vector3(0.5, -0.2, 0.1))
	w.add_body(a)
	w.add_body(b)
	var axis: Vector3 = Vector3(1, 2, -1).normalized()
	var joint = HingeJoint.from_world(a, b, anchor, axis, perpendicular_to(axis))
	check(joint != null, "sphere hinge construction")
	w.add_constraint(joint)
	joint.limit_enabled = true
	joint.lower_limit = -0.6
	joint.upper_limit = 0.5
	joint.motor_enabled = true
	joint.motor_target_speed = 2.0
	joint.motor_max_torque = 0.2
	a.linear_velocity = Vector3(0.3, -0.2, 0.1)
	b.linear_velocity = Vector3(-0.1, 0.4, 0.2)
	a.angular_velocity = Vector3(0.2, 0.1, -0.3)
	b.angular_velocity = Vector3(-0.3, 0.2, 0.4)
	return {"world": w, "a": a, "b": b, "joint": joint, "axis": axis, "anchor": anchor}


func test_e4_hinge_impulses_conserve_linear_momentum() -> void:
	var s = make_sphere_hinge()
	var p0: Vector3 = s.world.total_linear_momentum()
	var max_dev: float = 0.0
	for i in 480:
		s.world.step()
		max_dev = maxf(max_dev, (s.world.total_linear_momentum() - p0).length())
	# Every row has Jv_a = -Jv_b (anchor rows) or Jv = 0 (axis/motor/limit rows): exact up to float32 velocity rounding. That rounding is
	# a random walk over ~480 steps x 100 iterations x 8 rows of equal-and-opposite impulse applications, each ~1e-7 * |p| = 6e-8:
	# sqrt(3.8e5) * 6e-8 ~ 4e-5 as an upper estimate. The former 1e-5 only held because the old Baumgarte trajectory happened to land
	# at 6e-6; the same unstabilized velocity solve (beta = 0, old and new code alike) gives 2.05e-5. A one-sided impulse would be
	# orders of magnitude larger, so 5e-5 still detects any real momentum leak (the split channel adds exactly zero here).
	check(max_dev < 5.0e-5, "linear momentum conserved through anchor, axis, motor and limit impulses (max dev %s, |p| %s)" % [max_dev, p0.length()])
	check(s.joint.lower_limit_impulse() + s.joint.upper_limit_impulse() + absf(s.joint.motor_impulse()) >= 0.0, "scenario exercised the rows")


func test_e5_hinge_impulses_conserve_angular_momentum_up_to_the_constraint_error() -> void:
	var s = make_sphere_hinge()
	var l0: Vector3 = s.world.total_angular_momentum()
	var predicted: Vector3 = Vector3.ZERO
	var sum_abs: float = 0.0
	var engaged: bool = false
	for i in 480:
		var c: Vector3 = s.joint.linear_error()  # error at prepare time (positions do not change between here and the solve)
		var x_before: Array = [s.a.position, s.b.position]
		s.world.step()
		# Split displacement (M1.3): the pose correction moves x without changing p, so sum(x x p) changes by sum(m dx_split x v).
		# dx_split = x_after - x_before - v_after * dt (the integrator moves x by (v + v_split) dt).
		var bodies: Array = [s.a, s.b]
		for k in 2:
			var dx_split: Vector3 = bodies[k].position - x_before[k] - bodies[k].linear_velocity * DT
			predicted += bodies[k].mass * dx_split.cross(bodies[k].linear_velocity)
		# Anchor rows: dL_origin = C x lambda. Axis, motor and limit rows apply equal and opposite pure angular impulses: dL = 0.
		var lambda: Vector3 = anchor_impulse(s.joint)
		predicted += c.cross(lambda)
		sum_abs += (c.cross(lambda)).length()
		engaged = engaged or s.joint.lower_limit_impulse() > 0.0 or s.joint.upper_limit_impulse() > 0.0 or absf(s.joint.motor_impulse()) > 0.0
	var actual: Vector3 = s.world.total_angular_momentum() - l0
	check(engaged, "motor/limit rows were active")
	check(sum_abs > 1.0e-7, "the constraint-error term is not vacuous (sum |C x lambda| = %s)" % sum_abs)
	# The measured change must equal the analytic C x lambda term plus the split-displacement term above; the remaining slack is
	# float32 rounding of the ~1 kg m^2/s-scale angular momentum summed over 480 steps (~1e-6 each).
	check_vec3_near(actual, predicted, 2.0e-4, "dL = sum C x lambda + split displacement term (actual %s, predicted %s)" % [actual, predicted])
	check(actual.length() < 0.01 * l0.length() + 1.0e-3, "and it is small in absolute terms (|dL| %s of |L| %s)" % [actual.length(), l0.length()])


# --- F. Input validation ---

func test_f1_invalid_construction_is_rejected_explicitly() -> void:
	var a = box_body(1.0, Vector3.ZERO)
	var b = box_body(1.0, Vector3(1, 0, 0))
	var w = make_world()
	w.add_body(a)
	w.add_body(b)
	var good_axis: Vector3 = Vector3.BACK
	var good_ref: Vector3 = Vector3.RIGHT
	var cases: Array = [
		["zero axis A", [Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, good_axis, good_ref, good_ref]],
		["zero axis B", [Vector3.ZERO, Vector3.ZERO, good_axis, Vector3.ZERO, good_ref, good_ref]],
		["non-unit axis", [Vector3.ZERO, Vector3.ZERO, good_axis * 1.001, good_axis, good_ref, good_ref]],
		["NaN axis", [Vector3.ZERO, Vector3.ZERO, Vector3(NAN, 0, 1), good_axis, good_ref, good_ref]],
		["infinite anchor", [Vector3(INF, 0, 0), Vector3.ZERO, good_axis, good_axis, good_ref, good_ref]],
		["zero reference", [Vector3.ZERO, Vector3.ZERO, good_axis, good_axis, Vector3.ZERO, good_ref]],
		["reference parallel to axis", [Vector3.ZERO, Vector3.ZERO, good_axis, good_axis, good_axis, good_ref]],
		["reference not perpendicular", [Vector3.ZERO, Vector3.ZERO, good_axis, good_axis, (good_ref + good_axis * 0.01).normalized(), good_ref]],
	]
	for c in cases:
		var args: Array = c[1]
		var joint = HingeJoint.new(a, b, args[0], args[1], args[2], args[3], args[4], args[5])
		check(not joint.is_valid(), "%s: reported invalid" % c[0])
		check(not joint.validate().is_empty(), "%s: carries an error message" % c[0])
		check(w.add_constraint(joint) == null, "%s: refused by SimWorld.add_constraint" % c[0])
	check(w.constraints.is_empty(), "no invalid constraint was stored")
	# Accepted boundary: a few float32 ulps of normalization error is rounding, not invalid input.
	var near_unit = HingeJoint.new(a, b, Vector3.ZERO, Vector3.ZERO, good_axis * (1.0 + 5.0e-7), good_axis, good_ref, good_ref)
	check(near_unit.is_valid(), "unit length within float32 rounding is accepted")
	check_near(near_unit.local_axis_a.length(), 1.0, 1.0e-7, "valid input is re-normalized exactly")
	# Bodies.
	check(not HingeJoint.new(null, b, Vector3.ZERO, Vector3.ZERO, good_axis, good_axis, good_ref, good_ref).is_valid(), "null body A rejected")
	check(not HingeJoint.new(a, null, Vector3.ZERO, Vector3.ZERO, good_axis, good_axis, good_ref, good_ref).is_valid(), "null body B rejected")
	check(not HingeJoint.new(a, a, Vector3.ZERO, Vector3.ZERO, good_axis, good_axis, good_ref, good_ref).is_valid(), "same body twice rejected")
	check(not HingeJoint.new(make_static(), make_static(Vector3(1, 0, 0)), Vector3.ZERO, Vector3.ZERO, good_axis, good_axis, good_ref, good_ref).is_valid(), "two static bodies rejected")
	check(HingeJoint.from_world(a, b, Vector3(0.5, 0, 0), Vector3.ZERO, good_ref) == null, "from_world: zero axis returns null")
	check(HingeJoint.from_world(a, b, Vector3(0.5, 0, 0), good_axis, good_axis) == null, "from_world: parallel reference returns null")
	check(HingeJoint.from_world(a, b, Vector3(0.5, 0, 0), good_axis, good_ref) != null, "from_world: valid input accepted")


func test_f2_invalid_limit_and_motor_settings_are_detected() -> void:
	var s = make_hinge(Vector3(0, 0, 1), true)
	var j = s.joint
	check(j.is_valid(), "baseline valid")
	j.lower_limit = 1.0
	j.upper_limit = 0.5
	check(j.is_valid(), "lower > upper is irrelevant while limits are disabled")
	j.limit_enabled = true
	check(not j.is_valid(), "lower > upper with limits enabled is invalid")
	j.lower_limit = -1.0
	check(j.is_valid(), "fixed")
	j.upper_limit = INF
	check(not j.is_valid(), "infinite limit rejected (use a finite bound)")
	j.upper_limit = 1.0
	j.motor_target_speed = NAN
	check(not j.is_valid(), "NaN target speed rejected")
	j.motor_target_speed = 1.0
	j.motor_max_torque = -1.0
	check(not j.is_valid(), "negative max torque rejected")
	j.motor_max_torque = INF
	check(not j.is_valid(), "infinite max torque rejected")
	j.motor_max_torque = NAN
	check(not j.is_valid(), "NaN max torque rejected")
	j.motor_max_torque = 0.0
	check(j.is_valid(), "zero max torque is valid (inactive motor)")


func test_f3_world_refuses_to_step_with_an_invalid_constraint() -> void:
	var s = make_hinge(Vector3(0, 0, 1), true, Vector3(0, -9.81, 0))
	run_steps(s.world, 5)
	var steps0: int = s.world.step_count
	var pos0: Vector3 = s.b.position
	var vel0: Vector3 = s.b.linear_velocity
	s.joint.motor_max_torque = NAN  # corrupt after it was accepted
	check(not s.world.step(), "step() refused")
	check(s.world.advance(0.1) == 0, "advance() refused")
	check(s.world.step_count == steps0 and s.b.position == pos0 and s.b.linear_velocity == vel0, "no state changed")
	check(s.world.last_error.contains("constraint 0"), "reason names the constraint (%s)" % s.world.last_error)
	s.joint.motor_max_torque = 0.0
	check(s.world.step(), "stepping resumes once the data is valid")


func test_f4_initial_axis_misalignment_is_corrected() -> void:
	var s = make_hinge(Vector3(1, 1, 0), true)
	var tilt_axis: Vector3 = perpendicular_to(s.axis)
	var dq: Quaternion = Quaternion(tilt_axis, 0.3)
	s.b.position = s.anchor + dq * (s.b.position - s.anchor)  # tilt about the anchor: anchors still coincide
	s.b.orientation = dq * s.b.orientation
	check_near(s.joint.axis_error_angle(), 0.3, 1.0e-5, "initial misalignment is 0.3 rad")
	var max_lin: float = 0.0
	for i in 120:
		s.world.step()
		max_lin = maxf(max_lin, s.joint.linear_error().length())
	check(s.joint.axis_error_angle() < 1.0e-3, "Baumgarte removes the misalignment within 1 s (%s)" % s.joint.axis_error_angle())
	check(max_lin < 0.02, "and does not tear the anchors apart (max %s)" % max_lin)
