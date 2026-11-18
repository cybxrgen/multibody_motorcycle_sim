extends "res://Tests/sim_test_case.gd"
## Rigid-body dynamics, fixed timestep and interpolation.
## Reference solutions are the exact DISCRETE solutions of semi-implicit Euler
## (v_k = v_0 + a*k*dt, x_n = x_0 + sum v_k*dt) and, loosely, the continuous ones.

const DT: float = 1.0 / 120.0


func test_static_body_does_not_move() -> void:
	var w = make_world(Vector3(0, -9.81, 0))
	var b = make_static(Vector3(1, 2, 3))
	w.add_body(b)
	b.apply_force(Vector3(100, 0, 0))
	b.apply_torque(Vector3(0, 50, 0))
	b.apply_impulse(Vector3(10, 10, 10))
	run_steps(w, 120)
	check_vec3_near(b.position, Vector3(1, 2, 3), 1e-12, "static position unchanged")
	check_vec3_near(b.linear_velocity, Vector3.ZERO, 1e-12, "static linear velocity zero")
	check_vec3_near(b.angular_velocity, Vector3.ZERO, 1e-12, "static angular velocity zero")


func test_free_body_moves_at_constant_velocity() -> void:
	var w = make_world(Vector3.ZERO)
	var b = make_sphere(1.0, 0.5)
	b.linear_velocity = Vector3(1, 2, 3)
	w.add_body(b)
	run_steps(w, 120)
	check_vec3_near(b.position, Vector3(1, 2, 3), 1e-5, "x = v*t after 1 s")
	check_vec3_near(b.linear_velocity, Vector3(1, 2, 3), 1e-12, "velocity unchanged without forces")


func test_gravity_integration() -> void:
	var w = make_world(Vector3(0, -9.81, 0))
	var b = make_sphere(3.0, 0.5, Vector3(0, 100, 0))
	w.add_body(b)
	run_steps(w, 120)
	var n: float = 120.0
	var discrete_y: float = 100.0 - 9.81 * DT * DT * n * (n + 1.0) / 2.0
	check_near(b.linear_velocity.y, -9.81, 5e-5, "v = g*t after 1 s")
	check_near(b.position.y, discrete_y, 5e-5, "matches exact semi-implicit Euler solution")
	check_near(b.position.y, 100.0 - 0.5 * 9.81, 0.05, "close to continuous 1/2 g t^2 (O(dt) difference)")
	check_near(b.position.x, 0.0, 1e-12, "no sideways drift")
	# Gravity must be independent of mass.
	var heavy = make_sphere(300.0, 0.5, Vector3(0, 100, 0))
	var w2 = make_world(Vector3(0, -9.81, 0))
	w2.add_body(heavy)
	run_steps(w2, 120)
	check_near(heavy.position.y, b.position.y, 5e-5, "fall is mass independent")


func test_constant_force_acceleration() -> void:
	var w = make_world(Vector3.ZERO)
	var b = make_sphere(2.0, 0.5)
	w.add_body(b)
	for i in 120:
		b.apply_force(Vector3(10, 0, 0))  # forces must be re-applied every step
		w.step()
	var n: float = 120.0
	check_near(b.linear_velocity.x, 5.0, 1e-5, "a = F/m = 5 -> v = 5 m/s after 1 s")
	check_near(b.position.x, 5.0 * DT * DT * n * (n + 1.0) / 2.0, 1e-5, "exact discrete displacement")
	check_vec3_near(b.force_accumulator, Vector3.ZERO, 1e-12, "accumulator cleared by the step")


func test_forces_are_consumed_each_step() -> void:
	var w = make_world(Vector3.ZERO)
	var b = make_sphere(1.0, 0.5)
	w.add_body(b)
	b.apply_force(Vector3(60, 0, 0))  # 60 N for ONE step -> impulse 0.5 N*s
	run_steps(w, 100)
	check_near(b.linear_velocity.x, 60.0 * DT, 1e-5, "one-step force gives impulse F*dt, not continuing acceleration")


func test_torque_angular_acceleration() -> void:
	var w = make_world(Vector3.ZERO)
	var inertia: Basis = SimMassProperties.diagonal_inertia(Vector3(1, 2, 3))
	var b = make_body(1.0, inertia)
	w.add_body(b)
	for i in 120:
		b.apply_torque(Vector3(0, 4, 0))
		w.step()
	check_near(b.angular_velocity.y, 2.0, 1e-5, "alpha = tau/I_yy = 2 -> omega_y = 2 after 1 s")
	check_near(b.angular_velocity.x, 0.0, 1e-12, "no gyroscopic leak into x")
	check_near(b.angular_velocity.z, 0.0, 1e-12, "no gyroscopic leak into z")
	var n: float = 120.0
	var angle: float = 2.0 * DT * DT * n * (n + 1.0) / 2.0  # sum of omega_k*dt
	check_near(b.orientation.get_angle(), angle, 1e-6, "accumulated rotation angle")
	check_vec3_near(b.orientation.get_axis(), Vector3.UP, 1e-6, "rotation about +Y")


func test_fixed_timestep_decoupled_from_frame_time() -> void:
	var w = make_world(Vector3.ZERO)
	check_near(w.config.fixed_timestep, DT, 1e-15, "default timestep is 1/120 s")
	var steps: int = w.advance(0.05)
	check(steps == 6, "0.05 s of frame time yields 6 fixed steps (got %d)" % steps)
	check_near(w.simulation_time, 6.0 * DT, 1e-12, "simulation time = steps * dt")
	# Many small frames sum to the same number of steps.
	var w2 = make_world(Vector3.ZERO)
	var total: int = 0
	for i in 60:
		total += w2.advance(1.0 / 60.0)
	check(total >= 119 and total <= 120, "60 frames of 1/60 s give ~120 steps (got %d)" % total)


func test_backlog_is_capped() -> void:
	var w = make_world(Vector3.ZERO)
	w.config.max_frame_time = 1.0  # let the step cap, not the frame clamp, be the limiting guard
	var steps: int = w.advance(10.0)
	check(steps == w.config.max_steps_per_frame, "step count capped at max_steps_per_frame (got %d)" % steps)
	check(w.accumulator < w.config.fixed_timestep, "backlog discarded, accumulator below one step")
	check(w.dropped_time > 0.0, "dropped time recorded")


func test_render_interpolation() -> void:
	var w = make_world(Vector3.ZERO)
	var b = make_sphere(1.0, 0.5)
	b.linear_velocity = Vector3(1, 0, 0)
	w.add_body(b)
	var steps: int = w.advance(1.5 * DT)
	check(steps == 1, "one step for 1.5 dt")
	var alpha: float = w.interpolation_alpha()
	check_near(alpha, 0.5, 1e-9, "alpha is the sub-step fraction")
	var t: Transform3D = SimBodyState.interpolate_transform(b.previous_state, b.published_state, alpha)
	check_near(t.origin.x, 0.5 * DT, 1e-8, "interpolated position is halfway between states")
	check_near(b.published_state.position.x, DT, 1e-8, "published state is the latest step")
	check_near(b.previous_state.position.x, 0.0, 1e-12, "previous state is the step start")
