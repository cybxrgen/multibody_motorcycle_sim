extends "res://Tests/sim_test_case.gd"
## Quaternion orientation integration and rotational dynamics (incl. gyroscopic term).


func test_quaternion_stays_normalized() -> void:
	var w = make_world(Vector3.ZERO)
	var b = make_body(1.0, SimMassProperties.diagonal_inertia(Vector3(1, 2, 3)))
	b.angular_velocity = Vector3(1.5, 1.0, 0.5)
	w.add_body(b)
	run_steps(w, 5000)
	check_near(b.orientation.length(), 1.0, 1e-9, "|q| = 1 after 5000 steps")
	var r: Basis = Basis(b.orientation)
	check_basis_near(r * r.transposed(), Basis.IDENTITY, 1e-6, "rotation matrix stays orthonormal")
	check(not is_nan(b.angular_velocity.length()), "no NaN in angular velocity")


func test_constant_spin_matches_analytic_rotation() -> void:
	var w = make_world(Vector3.ZERO)
	var b = make_sphere(1.0, 0.5)
	b.angular_velocity = Vector3(0, 0, 1.0)  # 1 rad/s about +Z
	w.add_body(b)
	run_steps(w, 120)  # 1 s
	check_vec3_near(b.orientation * Vector3.RIGHT, Vector3(cos(1.0), sin(1.0), 0.0), 1e-5, "+X rotates to (cos1, sin1, 0)")
	check_vec3_near(b.angular_velocity, Vector3(0, 0, 1), 1e-12, "isotropic body: omega constant")


func test_angular_velocity_is_world_frame() -> void:
	# Body already rotated +90 deg about Z. World omega = +X must rotate about WORLD X:
	# q' = Rx(0.5) * q0. A local-frame interpretation (q0 * Rx) would give a different result.
	var q0: Quaternion = Quaternion(Vector3.BACK, PI / 2.0)
	var w = make_world(Vector3.ZERO)
	var b = make_body(1.0, SimMassProperties.sphere_inertia(1.0, 0.5), Vector3.ZERO, q0)
	b.angular_velocity = Vector3(1, 0, 0)
	w.add_body(b)
	run_steps(w, 60)  # 0.5 s -> 0.5 rad
	var expected: Vector3 = Vector3(0, cos(0.5), sin(0.5))
	check_vec3_near(b.orientation * Vector3.RIGHT, expected, 1e-6, "world-frame increment is pre-multiplied")
	var local_frame_result: Vector3 = (q0 * Quaternion(Vector3.RIGHT, 0.5)) * Vector3.RIGHT
	check((local_frame_result - expected).length() > 0.1, "local-frame interpretation would differ (test is discriminating)")


func test_principal_axis_spin_is_stationary() -> void:
	# Torque-free spin about a principal axis: I w is parallel to w, gyroscopic term is zero,
	# so omega must stay constant even for an asymmetric body in an arbitrary orientation.
	var q0: Quaternion = Quaternion(Vector3(1, 1, 0).normalized(), 0.7)
	var w = make_world(Vector3.ZERO)
	var b = make_body(1.0, SimMassProperties.diagonal_inertia(Vector3(1, 2, 3)), Vector3.ZERO, q0)
	var omega0: Vector3 = (q0 * Vector3(0, 2.0, 0))  # about local principal Y axis
	b.angular_velocity = omega0
	w.add_body(b)
	run_steps(w, 240)
	check_vec3_near(b.angular_velocity, omega0, 1e-5, "omega constant for principal-axis spin")


func test_gyroscopic_dynamics_sanity() -> void:
	# Torque-free asymmetric body, omega not on a principal axis. Body-frame Euler equations give
	# domega/dt = (-0.15, 0.9, -0.5) rad/s^2 initially, so omega MUST change; while angular momentum
	# and kinetic energy (the invariants of torque-free motion) must be conserved to O(dt).
	var w = make_world(Vector3.ZERO)
	w.config.fixed_timestep = 1.0 / 240.0
	var b = make_body(1.0, SimMassProperties.diagonal_inertia(Vector3(1, 2, 3)))
	b.angular_velocity = Vector3(3.0, 0.5, 0.3)
	w.add_body(b)
	var l0: Vector3 = b.angular_momentum_about_com()
	var e0: float = b.kinetic_energy()
	var omega0: Vector3 = b.angular_velocity
	run_steps(w, 120)  # 0.5 s
	check((b.angular_velocity - omega0).length() > 0.05, "gyroscopic term changes omega (|d omega| = %s)" % (b.angular_velocity - omega0).length())
	check_near((b.angular_momentum_about_com() - l0).length() / l0.length(), 0.0, 0.02, "relative |L| drift below 2% (O(dt))")
	check_near(b.kinetic_energy() / e0, 1.0, 0.01, "kinetic energy conserved within 1% (O(dt))")
