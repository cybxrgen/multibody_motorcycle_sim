extends "res://Tests/sim_test_case.gd"
## Constraint framework and ball joint: coincidence, static coupling, stability, recovery, unilateral limits.

const DT: float = 1.0 / 120.0


## Static pivot at (0,5,0); sphere bob hanging at angle theta0 from straight down, joined at the pivot.
func make_pendulum(theta0: float, length: float, radius: float, warm_start: bool = true) -> Dictionary:
	var w = make_world(Vector3(0, -9.81, 0))
	w.config.solver_iterations = 50
	w.config.warm_starting = warm_start
	var pivot: Vector3 = Vector3(0, 5, 0)
	var offset: Vector3 = Vector3(length * sin(theta0), -length * cos(theta0), 0)
	var anchor = make_static(pivot)
	var bob = make_sphere(1.0, radius, pivot + offset)
	w.add_body(anchor)
	w.add_body(bob)
	var joint = BallJoint.new(anchor, bob, Vector3.ZERO, -offset)
	w.add_constraint(joint)
	return {"world": w, "anchor": anchor, "bob": bob, "joint": joint, "pivot": pivot}


func swing_angle(s: Dictionary) -> float:
	var d: Vector3 = s.bob.position - s.pivot
	return atan2(d.x, -d.y)


func test_ball_joint_anchor_coincidence_initial() -> void:
	var q: Quaternion = Quaternion(Vector3(1, 1, 0).normalized(), 0.9)
	var a = make_sphere(1.0, 0.5, Vector3(0, 0, 0))
	var pivot: Vector3 = Vector3(1.0, 0.2, -0.3)
	var b = make_body(1.0, SimMassProperties.sphere_inertia(1.0, 0.5), Vector3(2, 1, 0), q)
	var la: Vector3 = pivot - a.position
	var lb: Vector3 = q.inverse() * (pivot - b.position)
	var joint = BallJoint.new(a, b, la, lb)
	check_vec3_near(joint.world_anchor_a(), pivot, 1e-12, "world anchor A")
	check_vec3_near(joint.world_anchor_b(), pivot, 1e-6, "world anchor B (rotated body)")
	check_vec3_near(joint.linear_error(), Vector3.ZERO, 1e-6, "no initial constraint error")
	check_vec3_near(joint.relative_anchor_velocity(), Vector3.ZERO, 1e-12, "no initial relative velocity")
	b.angular_velocity = Vector3(0, 0, 2)
	var r_b: Vector3 = q * lb
	check_vec3_near(joint.relative_anchor_velocity(), Vector3(0, 0, 2).cross(r_b), 1e-12, "relative anchor velocity = w x r")


func test_static_to_dynamic_coupling_holds_weight() -> void:
	var s = make_pendulum(0.0, 1.0, 0.1)
	var w = s.world
	run_steps(w, 240)  # 2 s
	check_vec3_near(s.anchor.position, s.pivot, 0.0, "static body never moves")
	check_vec3_near(s.anchor.linear_velocity, Vector3.ZERO, 0.0, "static body velocity stays zero")
	check_near(s.bob.position.y, 4.0, 1e-4, "hanging bob stays at rest below pivot")
	check_near(s.bob.linear_velocity.length(), 0.0, 1e-3, "hanging bob has no velocity")
	check_near(s.joint.accumulated_impulse_on_b().y / DT, 9.81, 0.1, "joint force on bob equals its weight m*g")
	check_near(s.joint.linear_error().length(), 0.0, 1e-4, "no joint drift")


func test_pendulum_period_matches_theory() -> void:
	# Physical pendulum: I_pivot = 2/5 m r^2 + m L^2, T = 2 pi sqrt(I_pivot / (m g L)).
	var s = make_pendulum(0.1, 1.0, 0.1)
	var period: float = 2.0 * PI * sqrt((0.4 * 0.01 + 1.0) / (9.81 * 1.0))
	var w = s.world
	run_steps(w, 30)  # t = 0.25 s
	var expected_eighth: float = 0.1 * cos(2.0 * PI * 0.25 / period)
	check_near(swing_angle(s), expected_eighth, 0.01, "angle at t=0.25 s follows small-angle solution")
	run_steps(w, 30)  # t = 0.5 s ~ T/4
	check_near(swing_angle(s), 0.1 * cos(2.0 * PI * 0.5 / period), 0.01, "angle near zero at ~T/4")
	check_near(s.joint.linear_error().length(), 0.0, 1e-3, "joint holds while swinging")


func test_stability_under_gravity_large_swing() -> void:
	for warm in [true, false]:
		var s = make_pendulum(PI / 2.0, 1.0, 0.1, warm)
		var w = s.world
		# Cold start needs more iterations: the 0.1 m sphere has tiny inertia, so the three scalar rows are
		# strongly coupled and Gauss-Seidel converges slowly (measured: 35 mm @20, 11 mm @50, 4.7 mm @100, 3.3 mm @200).
		# 3.3 mm is the converged Baumgarte steady state v^2 dt^2 / (2 L beta).
		w.config.solver_iterations = 50 if warm else 200
		var max_err: float = 0.0
		var max_speed: float = 0.0
		var max_len_dev: float = 0.0
		for i in 600:  # 5 s
			w.step()
			max_err = maxf(max_err, s.joint.linear_error().length())
			max_speed = maxf(max_speed, s.bob.linear_velocity.length())
			max_len_dev = maxf(max_len_dev, absf((s.bob.position - s.pivot).length() - 1.0))
		check(not is_nan(s.bob.position.length()), "no NaN (warm_start=%s)" % warm)
		check(max_err < 1e-2, "anchor error stays below 1 cm (warm_start=%s, max=%s)" % [warm, max_err])
		check(max_len_dev < 1e-2, "rod length preserved within 1 cm (warm_start=%s, dev=%s)" % [warm, max_len_dev])
		check(max_speed < 6.0, "speed bounded; theoretical max ~4.5 m/s (warm_start=%s, max=%s)" % [warm, max_speed])


func test_two_link_chain_stability() -> void:
	var w = make_world(Vector3(0, -9.81, 0))
	w.config.solver_iterations = 50
	var pivot: Vector3 = Vector3(0, 5, 0)
	var anchor = make_static(pivot)
	var inertia: Basis = SimMassProperties.box_inertia(1.0, Vector3(1, 0.1, 0.1))
	var l1 = make_body(1.0, inertia, pivot + Vector3(0.5, 0, 0))
	var l2 = make_body(1.0, inertia, pivot + Vector3(1.5, 0, 0))
	w.add_body(anchor)
	w.add_body(l1)
	w.add_body(l2)
	var j1 = BallJoint.new(anchor, l1, Vector3.ZERO, Vector3(-0.5, 0, 0))
	var j2 = BallJoint.new(l1, l2, Vector3(0.5, 0, 0), Vector3(-0.5, 0, 0))
	w.add_constraint(j1)
	w.add_constraint(j2)
	var max_err: float = 0.0
	for i in 600:
		w.step()
		max_err = maxf(max_err, maxf(j1.linear_error().length(), j2.linear_error().length()))
	check(not is_nan(l2.position.length()), "no NaN in chain")
	check(max_err < 2e-2, "both joints stay within 2 cm over 5 s (max=%s)" % max_err)
	check(l2.position.y < 5.0, "chain fell under gravity")


func test_recovery_from_external_impulses() -> void:
	# Impulse along the rod is absorbed by the joint within one step.
	var s = make_pendulum(0.0, 1.0, 0.1)
	var w = s.world
	s.bob.apply_impulse(Vector3(0, -5, 0))
	w.step()
	check_near(s.bob.linear_velocity.y, 0.0, 1e-6, "axial impulse fully absorbed by one solve")
	check_near(s.joint.linear_error().length(), 0.0, 1e-3, "no drift after axial impulse")
	# Lateral impulse starts a swing; the joint must hold throughout.
	var s2 = make_pendulum(0.0, 1.0, 0.1)
	s2.bob.apply_impulse(Vector3(3, 0, 0))
	var max_err: float = 0.0
	for i in 360:
		s2.world.step()
		max_err = maxf(max_err, s2.joint.linear_error().length())
	check(max_err < 1e-2, "joint holds after lateral impulse (max=%s)" % max_err)
	check(s2.bob.position.x > 0.2 or s2.bob.linear_velocity.length() > 0.1, "pendulum actually swings")
	# Off-center impulse that also spins the body (~5 rad/s). Much faster spin degrades the joint: the Jacobian is
	# frozen per step, so anchor error grows ~ r*sin(phi)*(|w| dt)^2/2 (measured 17 mm at 50 rad/s, see PROJECT_STATE.md).
	var s3 = make_pendulum(0.0, 1.0, 0.1)
	s3.bob.apply_impulse_at_point(Vector3(0, 0, 2), s3.bob.position + Vector3(0.01, 0, 0))
	max_err = 0.0
	for i in 240:
		s3.world.step()
		max_err = maxf(max_err, s3.joint.linear_error().length())
	check(max_err < 1e-2, "joint holds after off-center impulse (max=%s)" % max_err)


func test_two_dynamic_bodies_stay_joined() -> void:
	var w = make_world(Vector3.ZERO)
	var a = make_sphere(1.0, 0.5, Vector3(0, 0, 0))
	var b = make_sphere(1.0, 0.5, Vector3(1, 0, 0))
	w.add_body(a)
	w.add_body(b)
	var joint = BallJoint.new(a, b, Vector3(0.5, 0, 0), Vector3(-0.5, 0, 0))
	w.add_constraint(joint)
	a.linear_velocity = Vector3(0, 2, 0)
	var max_err: float = 0.0
	for i in 240:
		w.step()
		max_err = maxf(max_err, joint.linear_error().length())
	check(max_err < 2e-3, "dynamic-dynamic joint drift small (max=%s)" % max_err)
	# Velocity-level satisfaction is a property of the SOLVED state. After the position integration the bodies have rotated, so a
	# measurement then carries the frozen-Jacobian lag ~ w^2 r dt (0.017 m/s peak here, 0.0166 at the last step), which depends on the
	# phase of the swing and not on the solver. Evaluate it right after a solve, without integrating.
	joint.prepare(w.config.fixed_timestep, w.config)
	for i in w.config.solver_iterations:
		joint.solve_iteration()
	check_near(joint.relative_anchor_velocity().length(), 0.0, 1e-2, "anchors co-move after the velocity solve")
	check(a.angular_velocity.length() + b.angular_velocity.length() > 0.1, "off-center coupling induced rotation")


func test_unilateral_and_clamped_rows() -> void:
	# One-sided (contact-like) row: min_impulse = 0 pushes but never pulls.
	var w = make_world(Vector3.ZERO)
	var ground = make_static()
	var falling = make_sphere(1.0, 0.5, Vector3(0, 1, 0))
	var rising = make_sphere(1.0, 0.5, Vector3(5, 1, 0))
	var capped = make_sphere(1.0, 0.5, Vector3(10, 1, 0))
	falling.linear_velocity = Vector3(0, -5, 0)
	rising.linear_velocity = Vector3(0, 5, 0)
	capped.linear_velocity = Vector3(0, -5, 0)
	for b in [ground, falling, rising, capped]:
		w.add_body(b)
	var specs: Array = [[falling, 0.0, INF], [rising, 0.0, INF], [capped, 0.0, 2.0]]
	for spec in specs:
		var c = SimConstraint.new(ground, spec[0])
		var row = c.add_row()
		row.linear_b = Vector3.UP
		row.min_impulse = spec[1]
		row.max_impulse = spec[2]
		w.add_constraint(c)
	w.step()
	check_near(falling.linear_velocity.y, 0.0, 1e-9, "unilateral row stops approach")
	check_near(rising.linear_velocity.y, 5.0, 1e-9, "unilateral row does not pull (impulse clamped at 0)")
	check_near(capped.linear_velocity.y, -3.0, 1e-9, "max_impulse clamps the correction to 2 N*s")
