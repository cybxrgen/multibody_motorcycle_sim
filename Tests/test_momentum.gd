extends "res://Tests/sim_test_case.gd"
## Conservation laws and Newton's third law for impulses and constraints.


func angular_momentum_about_origin(b) -> Vector3:
	return b.position.cross(b.linear_momentum()) + b.angular_momentum_about_com()


func make_joined_pair():
	var w = make_world(Vector3.ZERO)
	var a = make_sphere(2.0, 0.4, Vector3(0, 0, 0))
	var b = make_sphere(1.0, 0.3, Vector3(1.2, 0.3, -0.2))
	w.add_body(a)
	w.add_body(b)
	var joint = BallJoint.new(a, b, Vector3(0.6, 0.1, 0.0), Vector3(-0.6, -0.2, 0.2))
	# Re-seat B so the anchors coincide exactly.
	b.position = a.position + a.orientation * joint.local_anchor_a - b.orientation * joint.local_anchor_b
	b.reset_published_state()
	w.add_constraint(joint)
	a.linear_velocity = Vector3(1.0, -0.5, 0.3)
	b.linear_velocity = Vector3(-0.4, 0.8, 0.0)
	a.angular_velocity = Vector3(0.5, 1.0, -0.7)
	b.angular_velocity = Vector3(-1.0, 0.2, 0.9)
	return {"world": w, "a": a, "b": b, "joint": joint}


func test_equal_and_opposite_impulses() -> void:
	var a = make_sphere(2.0, 0.5, Vector3(0, 0, 0))
	var b = make_body(3.0, SimMassProperties.box_inertia(3.0, Vector3(1, 2, 3)), Vector3(2, 1, 0))
	var p: Vector3 = Vector3(1, 0.5, 0.2)
	var j: Vector3 = Vector3(3, -2, 5)
	a.apply_impulse_at_point(j, p)
	b.apply_impulse_at_point(-j, p)
	check_vec3_near(a.linear_momentum() + b.linear_momentum(), Vector3.ZERO, 1e-5, "linear momentum change sums to zero")
	var l_a: Vector3 = angular_momentum_about_origin(a)
	var l_b: Vector3 = angular_momentum_about_origin(b)
	check_vec3_near(l_a, p.cross(j), 1e-5, "A gains p x J about the origin")
	check_vec3_near(l_b, -p.cross(j), 1e-5, "B gains -p x J about the origin")
	check_vec3_near(l_a + l_b, Vector3.ZERO, 1e-5, "angular momentum change sums to zero")


func test_constraint_impulses_are_equal_and_opposite() -> void:
	var s = make_joined_pair()
	var joint = s.joint
	s.world.step()
	for row in joint.rows:
		check_vec3_near(row.linear_a + row.linear_b, Vector3.ZERO, 1e-12, "row linear Jacobians are opposite")
	check_vec3_near(s.a.linear_momentum() + s.b.linear_momentum(), Vector3(2.0 * 1.0 + 1.0 * -0.4, 2.0 * -0.5 + 0.8, 2.0 * 0.3), 1e-5, "joint impulses conserve total linear momentum in one step")


func test_linear_momentum_conserved_in_isolated_system() -> void:
	var s = make_joined_pair()
	var w = s.world
	var p0: Vector3 = w.total_linear_momentum()
	var total_mass: float = s.a.mass + s.b.mass
	var com0: Vector3 = (s.a.position * s.a.mass + s.b.position * s.b.mass) / total_mass
	run_steps(w, 240)
	check_vec3_near(w.total_linear_momentum(), p0, 1e-4, "total linear momentum constant over 2 s")
	var com: Vector3 = (s.a.position * s.a.mass + s.b.position * s.b.mass) / total_mass
	check_vec3_near(com, com0 + p0 / total_mass * w.simulation_time, 1e-4, "system COM moves uniformly")
	check_near(s.joint.linear_error().length(), 0.0, 2e-3, "joint held during the run")


func test_angular_momentum_conserved_without_external_torque() -> void:
	var s = make_joined_pair()
	var w = s.world
	w.config.solver_iterations = 50
	var dt: float = w.config.fixed_timestep
	var l0: Vector3 = w.total_angular_momentum()
	# No external torque: the change of angular momentum about the origin has exactly two sources (spheres: no gyroscopic term).
	#  (1) the anchor impulses: dL = C x lambda (equal and opposite impulses applied C apart), C = anchor error at prepare time;
	#  (2) the split-impulse pose correction (M1.3), which moves x without changing p: dL = sum m dx_split x v with
	#      dx_split = x_after - x_before - v_after * dt (the integrator moves x by (v + v_split) dt).
	# Before M1.3 only (1) existed and the drift stayed below 1e-3 (|L0| + 1) because the Baumgarte term kept C tiny; with the
	# split channel off (beta = 0) C grows and (1) alone gives a drift of 0.017. The decomposition is the honest invariant.
	var predicted: Vector3 = Vector3.ZERO
	var term_split: float = 0.0
	for i in 240:
		var c: Vector3 = s.joint.linear_error()
		var x_before: Array = [s.a.position, s.b.position]
		w.step()
		predicted += c.cross(s.joint.accumulated_impulse_on_b())
		for k in 2:
			var body = [s.a, s.b][k]
			var dx_split: Vector3 = body.position - x_before[k] - body.linear_velocity * dt
			var term: Vector3 = body.mass * dx_split.cross(body.linear_velocity)
			predicted += term
			term_split += term.length()
	var actual: Vector3 = w.total_angular_momentum() - l0
	check(term_split > 1.0e-4, "the split-displacement term is not vacuous (sum |m dx x v| = %s)" % term_split)
	# Tolerance: float32 rounding of values of order 1 summed over 240 steps (~1e-6 each) = 2e-4, as in the hinge test e5.
	check_vec3_near(actual, predicted, 2.0e-4, "dL = sum C x lambda + split displacement term (actual %s, predicted %s)" % [actual, predicted])
	check(actual.length() < 0.02 * (l0.length() + 1.0), "and the drift stays small (%s of |L0| = %s)" % [actual.length(), l0.length()])
