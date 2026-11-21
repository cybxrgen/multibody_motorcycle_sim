extends "res://Tests/sim_test_case.gd"
## Rotational integration stability (implicit-midpoint gyroscopic update) and numerical failure containment.
## Regression for the long-duration runaway of the torque-free asymmetric spinner of Scenes/simulation_test.gd: the former explicit
## gyroscopic update raised kinetic energy and |L| every step and overflowed after ~85.7 s at 120 Hz. Assertions are invariants and
## bounded-error checks, never exact trajectories: the intermediate-axis motion is exponentially sensitive to rounding.

const SimIntegrator = preload("res://Simulation/Core/sim_integrator.gd")

const SPIN_SIZE: Vector3 = Vector3(0.6, 0.3, 0.15)
const SPIN_OMEGA: Vector3 = Vector3(0.2, 6.0, 0.2)


func _spinner_world(hz: float = 120.0) -> Array:
	var w: SimWorld = make_world(Vector3.ZERO)
	w.config.fixed_timestep = 1.0 / hz
	var b: SimRigidBody = w.add_body(make_body(1.0, SimMassProperties.box_inertia(1.0, SPIN_SIZE)))
	b.angular_velocity = SPIN_OMEGA
	return [w, b]


func _inertia_consistency_error(b: SimRigidBody) -> float:
	var r: Basis = Basis(b.orientation)
	var iw: Basis = r * b.local_inertia * r.transposed()
	return maxf((iw.x - b.inertia_world.x).length(), maxf((iw.y - b.inertia_world.y).length(), (iw.z - b.inertia_world.z).length()))


func _finite_body(b: SimRigidBody) -> bool:
	for v in [b.position, b.linear_velocity, b.angular_velocity, b.inertia_world.x, b.inertia_world.y, b.inertia_world.z]:
		if not (is_finite(v.x) and is_finite(v.y) and is_finite(v.z)):
			return false
	return is_finite(b.orientation.x) and is_finite(b.orientation.y) and is_finite(b.orientation.z) and is_finite(b.orientation.w)


# --- A. Long-duration torque-free asymmetric spinner (the inspection scene's body) ---

func test_a1_long_duration_spinner_stays_bounded_and_conservative() -> void:
	# 150 s (18000 steps) is longer than the 85.7 s at which the explicit scheme overflowed. Principal moments of the box:
	# 0.009375 < 0.031875 < 0.0375 (spin about the intermediate axis).
	var pair: Array = _spinner_world()
	var w: SimWorld = pair[0]
	var b: SimRigidBody = pair[1]
	var l0: Vector3 = b.angular_momentum_about_com()
	var t0: float = b.kinetic_energy()
	var i_min: float = 0.009375
	var i_max: float = 0.0375
	var all_finite: bool = true
	var max_q_err: float = 0.0
	var max_dl: float = 0.0
	var max_dt: float = 0.0
	var max_dir: float = 0.0
	var max_incons: float = 0.0
	var max_w: float = 0.0
	var min_w: float = INF
	var flips: int = 0
	var max_wbx: float = 0.0
	var prev_y: float = 1.0
	var steps: int = 18000
	for i in steps:
		w.step()
		all_finite = all_finite and _finite_body(b)
		max_q_err = maxf(max_q_err, absf(b.orientation.length() - 1.0))
		var l: Vector3 = b.angular_momentum_about_com()
		max_dl = maxf(max_dl, absf(l.length() / l0.length() - 1.0))
		max_dt = maxf(max_dt, absf(b.kinetic_energy() / t0 - 1.0))
		max_dir = maxf(max_dir, l.angle_to(l0))
		max_incons = maxf(max_incons, _inertia_consistency_error(b))
		var wl: float = b.angular_velocity.length()
		max_w = maxf(max_w, wl)
		min_w = minf(min_w, wl)
		var wb: Vector3 = Basis(b.orientation).transposed() * b.angular_velocity
		if wb.y * prev_y < 0.0:
			flips += 1
		prev_y = wb.y
		max_wbx = maxf(max_wbx, absf(wb.x))
	check(w.numerical_fault == "" and w.step_count == steps, "no numerical fault, all %d steps executed" % steps)
	check(all_finite, "all body state finite for 150 s")
	check_near(max_q_err, 0.0, 1e-6, "orientation stays normalized (float32 rounding ~6e-8)")
	check_near(max_incons, 0.0, 1e-6, "inertia_world == R I R^T at every step")
	# Exact invariants of the midpoint rule; measured 2.5e-6 (L) and 4.6e-6 (T) over 150 s, i.e. float32 round-off of the stored
	# vectors accumulated as a random walk. 1e-4 leaves a 20x margin while the former scheme drifted by +1.9e0.
	check_near(max_dl, 0.0, 1e-4, "|L| conserved (relative)")
	check_near(max_dt, 0.0, 1e-4, "kinetic energy conserved (relative)")
	# |w| is confined by the invariants: |L|/I_max <= |w| <= |L|/I_min (T = L.I^-1.L/2). Analytic range for this state: [6.0049, 7.0029].
	check(max_w <= l0.length() / i_min * (1.0 + 1e-4), "|w| below the invariant bound |L|/I_min (max %s)" % max_w)
	check(min_w >= l0.length() / i_max * (1.0 - 1e-4), "|w| above |L|/I_max (min %s)" % min_w)
	check(max_w < 7.1 and max_w > 6.9, "|w| peak matches the analytic excursion 7.003 (got %s)" % max_w)
	# World-frame direction of L: O(dt) bounded error of the explicit-orientation coupling; measured 0.0157 rad (halves with dt).
	check(max_dir < 0.03, "L direction error bounded (max %s rad)" % max_dir)
	# Tumbling remains: the body-frame spin component about the intermediate axis flips sign repeatedly and the perturbation grows
	# from 0.2 rad/s to several rad/s (measured: 25 flips in 120 s, |w_b.x| up to 4.95).
	check(flips >= 10, "intermediate-axis tumbling: body-frame w_y changes sign %d times in 150 s" % flips)
	check(max_wbx > 3.0, "perturbation grows to tumbling amplitude (max |w_b.x| = %s)" % max_wbx)


func test_a2_explicit_gyroscopic_scheme_is_unstable_on_the_same_problem() -> void:
	# Discrimination: the former update, replayed through the same position integrator, gains energy without bound on the same body
	# (measured T/T0 = 1.88 at 40 s, 16 at 80 s, overflow at 85.7 s). The test proves the problem is what it was.
	var b: SimRigidBody = make_body(1.0, SimMassProperties.box_inertia(1.0, SPIN_SIZE))
	b.angular_velocity = SPIN_OMEGA
	var t0: float = b.kinetic_energy()
	var dt: float = 1.0 / 120.0
	for i in 4800:
		var wv: Vector3 = b.angular_velocity
		b.angular_velocity += b.inverse_inertia_world * ((-wv.cross(b.inertia_world * wv)) * dt)
		SimIntegrator.integrate_positions(b, dt)
	check(b.kinetic_energy() / t0 > 1.5, "explicit scheme: T/T0 = %s after 40 s (new scheme stays 1 +- 1e-4)" % (b.kinetic_energy() / t0))


func test_a3_spinner_error_scales_with_dt_and_stays_bounded() -> void:
	# O(dt) direction error: halving dt halves the measured bound; invariants stay exact at every rate.
	var dirs: Array = []
	for hz in [60.0, 120.0, 240.0]:
		var pair: Array = _spinner_world(hz)
		var w: SimWorld = pair[0]
		var b: SimRigidBody = pair[1]
		var l0: Vector3 = b.angular_momentum_about_com()
		var t0: float = b.kinetic_energy()
		var max_dir: float = 0.0
		var max_dt: float = 0.0
		for i in int(40.0 * hz):
			w.step()
			max_dir = maxf(max_dir, b.angular_momentum_about_com().angle_to(l0))
			max_dt = maxf(max_dt, absf(b.kinetic_energy() / t0 - 1.0))
		dirs.append(max_dir)
		check_near(max_dt, 0.0, 1e-4, "energy conserved at %s Hz" % hz)
	check(dirs[1] < 0.6 * dirs[0] and dirs[2] < 0.6 * dirs[1], "direction error decreases ~1/2 per dt halving (%s)" % [dirs])


# --- B. Rotational integration cases ---

func test_b1_zero_angular_velocity_is_exactly_preserved() -> void:
	var w: SimWorld = make_world(Vector3.ZERO)
	var b: SimRigidBody = w.add_body(make_body(1.0, SimMassProperties.box_inertia(1.0, SPIN_SIZE)))
	run_steps(w, 1200)
	check(b.angular_velocity == Vector3.ZERO and b.orientation == Quaternion.IDENTITY, "no spurious rotation from rest")
	check(w.numerical_fault == "", "no fault")


func test_b2_tiny_angular_velocity_stays_finite_and_conservative() -> void:
	for scale in [1e-12, 1e-8, 1e-4]:
		var w: SimWorld = make_world(Vector3.ZERO)
		var b: SimRigidBody = w.add_body(make_body(1.0, SimMassProperties.box_inertia(1.0, SPIN_SIZE)))
		b.angular_velocity = Vector3(0.3, 1.0, 0.2) * scale
		var l0: float = b.angular_momentum_about_com().length()
		run_steps(w, 2400)
		check(_finite_body(b) and w.numerical_fault == "", "finite for |w| ~ %s" % scale)
		check_near(b.angular_momentum_about_com().length() / l0, 1.0, 1e-4, "|L| conserved for |w| ~ %s" % scale)


func test_b3_symmetric_bodies() -> void:
	# Sphere: no gyroscopic effect, angular velocity bit-identical.
	var w: SimWorld = make_world(Vector3.ZERO)
	var s: SimRigidBody = w.add_body(make_sphere(2.0, 0.4))
	s.angular_velocity = Vector3(7.0, -3.0, 11.0)
	var w0: Vector3 = s.angular_velocity
	run_steps(w, 1200)
	check(s.angular_velocity == w0, "isotropic body keeps omega exactly")
	# Symmetric top I = (1, 2, 1), w = (1, 3, 0): body-frame (w_x, w_z) rotate about y at (I_y - I_x)/I_x * w_y = 3 rad/s,
	# w_y and |w| constant (Euler's equations, analytic).
	var w2: SimWorld = make_world(Vector3.ZERO)
	var t: SimRigidBody = w2.add_body(make_body(1.0, SimMassProperties.diagonal_inertia(Vector3(1, 2, 1))))
	t.angular_velocity = Vector3(1, 3, 0)
	run_steps(w2, 120)  # 1 s
	var wb: Vector3 = Basis(t.orientation).transposed() * t.angular_velocity
	# measured error 1.5e-4 (O(dt) phase lag of the semi-implicit orientation update); 2e-3 = ~13x margin
	check_near(wb.x, cos(3.0), 2e-3, "precession x")
	check_near(wb.z, -sin(3.0), 2e-3, "precession z")
	check_near(wb.y, 3.0, 1e-5, "w_y constant")
	check_near(t.angular_velocity.length(), sqrt(10.0), 1e-5, "|w| constant")


func test_b4_asymmetric_torque_free_short_run() -> void:
	var w: SimWorld = make_world(Vector3.ZERO)
	w.config.fixed_timestep = 1.0 / 240.0
	var b: SimRigidBody = w.add_body(make_body(1.0, SimMassProperties.diagonal_inertia(Vector3(1, 2, 3))))
	b.angular_velocity = Vector3(3.0, 0.5, 0.3)
	var l0: float = b.angular_momentum_about_com().length()
	var e0: float = b.kinetic_energy()
	var om0: Vector3 = b.angular_velocity
	run_steps(w, 240)
	check((b.angular_velocity - om0).length() > 0.05, "gyroscopic motion present")
	check_near(b.angular_momentum_about_com().length() / l0, 1.0, 1e-5, "|L| conserved (exact invariant, float32 rounding)")
	check_near(b.kinetic_energy() / e0, 1.0, 1e-5, "energy conserved (exact invariant, float32 rounding)")


func test_b5_applied_torque_is_integrated_like_before() -> void:
	# Torque on a principal axis (no gyroscopic term): omega = tau t / I exactly, symmetric and asymmetric bodies.
	var w: SimWorld = make_world(Vector3.ZERO)
	var s: SimRigidBody = w.add_body(make_sphere(1.0, 0.5))
	var a: SimRigidBody = w.add_body(make_body(1.0, SimMassProperties.diagonal_inertia(Vector3(1, 2, 3))))
	for i in 1200:
		s.apply_torque(Vector3(0.0, 0.4, 0.0))
		a.apply_torque(Vector3(0.0, 0.0, 6.0))
		w.step()
	var i_s: float = 0.4 * 1.0 * 0.25
	check_vec3_near(s.angular_velocity, Vector3(0.0, 0.4 * 10.0 / i_s, 0.0), 1e-3, "sphere: omega = tau t / I")
	check_vec3_near(a.angular_velocity, Vector3(0.0, 0.0, 6.0 * 10.0 / 3.0), 1e-3, "box: omega = tau t / I about a principal axis")


func test_b6_torque_on_asymmetric_body_bounded_over_long_run() -> void:
	# A short torque pulse on the spinner, then torque-free for 120 s: after the pulse the invariants must hold exactly again.
	var pair: Array = _spinner_world()
	var w: SimWorld = pair[0]
	var b: SimRigidBody = pair[1]
	for i in 60:
		b.apply_torque(Vector3(0.002, 0.0, -0.003))
		w.step()
	var l1: float = b.angular_momentum_about_com().length()
	var e1: float = b.kinetic_energy()
	run_steps(w, 14400)
	check(_finite_body(b) and w.numerical_fault == "", "finite after pulse + 120 s")
	check_near(b.angular_momentum_about_com().length() / l1, 1.0, 1e-4, "|L| conserved after the torque pulse")
	check_near(b.kinetic_energy() / e1, 1.0, 1e-4, "energy conserved after the torque pulse")


func test_b7_free_rotation_solver_invariants_and_convergence() -> void:
	var inv: Basis = Basis.from_scale(Vector3(1.0 / 0.009375, 1.0 / 0.031875, 1.0 / 0.0375))
	var seed_vals: Array = [Vector3(0.0019, 0.19, 0.0075), Vector3(0.05, 0.05, 0.05), Vector3(0.3, -0.1, 0.2)]
	for l in seed_vals:
		for dt in [1.0 / 480.0, 1.0 / 120.0, 1.0 / 60.0]:
			var d: PackedFloat64Array = SimIntegrator.solve_free_rotation(l, inv, dt)
			check(d.size() == 3, "converged for L=%s dt=%s" % [l, dt])
			if d.size() == 3:
				var lp: Vector3 = Vector3(l.x + d[0], l.y + d[1], l.z + d[2])
				var t_old: float = 0.5 * l.dot(inv * l)
				check_near((lp.length_squared()) / l.length_squared(), 1.0, 1e-6, "|L|^2 conserved (L stored as float32 here)")
				check_near(0.5 * lp.dot(inv * lp) / t_old, 1.0, 1e-6, "T conserved")
	check(SimIntegrator.solve_free_rotation(Vector3(NAN, 0, 0), inv, 0.01).is_empty(), "non-finite input rejected")
	check(SimIntegrator.solve_free_rotation(Vector3(1, 2, 3), inv, INF).is_empty(), "non-finite dt rejected")
	check(SimIntegrator.solve_free_rotation(Vector3.ZERO, inv, 0.01).size() == 3, "zero L: zero increment")


# --- C. Failure containment ---

func test_c1_non_finite_velocity_is_contained() -> void:
	var w: SimWorld = make_world(Vector3(0, -9.81, 0))
	var good: SimRigidBody = w.add_body(make_body(1.0, SimMassProperties.box_inertia(1.0, SPIN_SIZE), Vector3(5, 0, 0)))
	good.angular_velocity = Vector3(0.1, 0.2, 0.3)
	var bad: SimRigidBody = w.add_body(make_body(1.0, SimMassProperties.box_inertia(1.0, SPIN_SIZE), Vector3(0, 3, 0)))
	bad.body_name = "bad_spinner"
	bad.angular_velocity = Vector3(1, 2, 3)
	var joint: SimConstraint = w.add_constraint(BallJoint.new(good, bad, Vector3.ZERO, Vector3(-5, 0, 0)))
	check(joint != null, "joint registered")
	run_steps(w, 5)
	var good_pos: Vector3 = good.position
	var good_rot: Quaternion = good.orientation
	var good_lv: Vector3 = good.linear_velocity
	var good_av: Vector3 = good.angular_velocity
	var pub_pos: Vector3 = bad.published_state.position
	var steps_before: int = w.step_count
	bad.angular_velocity = Vector3(INF, 0, NAN)
	var ok: bool = w.step()
	check(not ok, "step() reports the aborted step as failed")
	check(w.numerical_fault != "" and w.numerical_fault.contains("body %d" % bad.id) and w.numerical_fault.contains("bad_spinner"), "fault names the body: %s" % w.numerical_fault)
	check(w.step_count == steps_before, "aborted step is not counted")
	check(good.position == good_pos and good.orientation == good_rot and good.linear_velocity == good_lv and good.angular_velocity == good_av, "unrelated (jointed) body untouched, not contaminated")
	check(bad.published_state.position == pub_pos and _finite_body(good), "published state of the faulty body is the last valid one")
	check(not w.step() and w.last_error.contains("numerical fault"), "further steps refused with a diagnostic")
	check(w.advance(0.1) == 0, "advance() refused as well")
	bad.angular_velocity = Vector3(1, 2, 3)
	w.clear_numerical_fault()
	check(w.step() and w.numerical_fault == "", "stepping resumes after clear_numerical_fault()")


func test_c2_non_finite_position_after_integration_is_not_published() -> void:
	var w: SimWorld = make_world(Vector3.ZERO)
	var start: Vector3 = Vector3(1, 2, 3)
	var b: SimRigidBody = w.add_body(make_sphere(1.0, 0.5, start))
	b.body_name = "ball"
	run_steps(w, 3)
	var pub: Vector3 = b.published_state.position
	var steps_before: int = w.step_count
	# Finite velocity, but position + v dt exceeds the float32 range (3.4028e38) and becomes inf.
	b.position = Vector3(3.4e38, 0, 0)
	b.linear_velocity = Vector3(3.0e38, 0, 0)
	w.step()
	check(w.numerical_fault.contains("body %d" % b.id) and w.numerical_fault.contains("ball") and w.numerical_fault.contains("position"), "overflowing position detected: %s" % w.numerical_fault)
	check(w.step_count == steps_before, "aborted step not counted")
	check(b.position == Vector3(3.4e38, 0, 0) and b.linear_velocity == Vector3(3.0e38, 0, 0), "body restored to its pre-step state")
	check(b.published_state.position == pub and b.published_state.position.is_finite(), "last valid state remains published")
