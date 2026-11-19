extends "res://Tests/sim_test_case.gd"
## ContactConstraint (M1.2): unilateral normal contact + isotropic Coulomb friction between two bodies.
## Tolerance policy (TESTING.md): float32 vector quantities >= 1e-6 relative; a single-row/single-contact solve is exact after
## one iteration, so single-step expectations use 1e-5 (absolute, for values of order 1). Each looser tolerance is justified inline.

const ContactConstraint = preload("res://Simulation/Solver/contact_constraint.gd")
const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")

const DT: float = 1.0 / 120.0
const G: float = 9.81
## Exact single-contact solves carry only float32 rounding of a few ulps on O(1) values.
const TOL: float = 1.0e-5


# --- Builders ---

## Static ground (plane y = 0 through the origin, normal +Y from ground toward the ball) and a solid sphere whose lowest
## point is the contact point. Contact data is given in world coordinates (from_world).
func ground_ball(gravity: Vector3, center_y: float, velocity: Vector3, mu: float, mass: float = 1.0, radius: float = 0.5) -> Dictionary:
	var w = make_world(gravity)
	var ground = make_static()
	var ball = make_sphere(mass, radius, Vector3(0, center_y, 0))
	ball.linear_velocity = velocity
	w.add_body(ground)
	w.add_body(ball)
	var c = ContactConstraint.from_world(ground, ball, Vector3.ZERO, Vector3(0, center_y - radius, 0), Vector3.UP, mu)
	w.add_constraint(c)
	return {"world": w, "ground": ground, "ball": ball, "contact": c, "radius": radius}


## The caller's duty: contact anchors are fixed to the bodies, so when the contact patch moves, re-seat them.
func refresh_ground_ball(s: Dictionary) -> void:
	var ball = s.ball
	s.contact.local_anchor_a = Vector3(ball.position.x, 0.0, ball.position.z)
	s.contact.local_anchor_b = ball.orientation.inverse() * Vector3(0, -s.radius, 0)


func step_refreshing(s: Dictionary, count: int) -> void:
	for i in count:
		refresh_ground_ball(s)
		s.world.step()


## Two oblique bodies touching at `point` with normal n (A -> B), arbitrary orientations and inertias.
func oblique_pair(mu: float, tangent_rotation: float = 0.0) -> Dictionary:
	var w = make_world(Vector3.ZERO)
	var a = make_body(2.0, SimMassProperties.box_inertia(2.0, Vector3(1.0, 0.6, 0.4)), Vector3(-0.4, -0.5, 0.3),
		Quaternion(Vector3(1, 2, 3).normalized(), 0.8))
	var b = make_body(3.0, SimMassProperties.box_inertia(3.0, Vector3(0.5, 1.2, 0.9)), Vector3(0.5, 0.6, -0.4),
		Quaternion(Vector3(-2, 1, 1).normalized(), -1.1))
	w.add_body(a)
	w.add_body(b)
	var n: Vector3 = Vector3(1, 2, -2) / 3.0
	var p: Vector3 = Vector3(0.1, 0.0, 0.2)
	var c = ContactConstraint.from_world(a, b, p, p, n, mu)
	c.tangent_rotation = tangent_rotation
	w.add_constraint(c)
	a.linear_velocity = Vector3(0.5, 0.1, -0.3)
	b.linear_velocity = Vector3(-0.6, -0.9, 0.4)
	a.angular_velocity = Vector3(0.3, -0.4, 0.2)
	b.angular_velocity = Vector3(-0.2, 0.5, -0.6)
	return {"world": w, "a": a, "b": b, "contact": c, "n": n, "p": p}


## Independent derivation of K = J M^-1 J^T for a unit direction d at lever arms r_a, r_b (not using the constraint's code).
func k_for(a, b, d: Vector3, r_a: Vector3, r_b: Vector3) -> float:
	return a.inverse_mass + b.inverse_mass \
		+ r_a.cross(d).dot(a.inverse_inertia_world * r_a.cross(d)) \
		+ r_b.cross(d).dot(b.inverse_inertia_world * r_b.cross(d))


## Runs the constraint's solver cycle (prepare, config.solver_iterations iterations, finish) on the CURRENT pose and velocities,
## without the integrator. World.step() would first apply the explicit gyroscopic term and then move the bodies, which changes the
## quantities an exactness check wants to evaluate; this isolates the constraint.
func solve_in_place(c, config) -> void:
	c.prepare(DT, config)
	for i in config.solver_iterations:
		c.solve_iteration()
	c.finish()


func angular_momentum_about_origin(b) -> Vector3:
	return b.position.cross(b.linear_momentum()) + b.angular_momentum_about_com()


# =====================================================================================================================
# Normal contact
# =====================================================================================================================

func test_static_ground_stops_approaching_body() -> void:
	var s = ground_ball(Vector3.ZERO, 0.5, Vector3(0, -3, 0), 0.0)
	s.world.step()
	check_near(s.ball.linear_velocity.y, 0.0, TOL, "approach velocity removed")
	check_near(s.contact.normal_impulse(), 3.0, TOL, "normal impulse = m * approach speed")
	check_near(s.ball.position.y, 0.5, TOL, "ball stays on the ground plane")
	check(s.contact.normal_velocity() > -TOL, "no residual approach at the contact")


func test_separating_bodies_receive_no_attractive_impulse() -> void:
	var s = ground_ball(Vector3.ZERO, 0.5, Vector3(0, 2, 0), 0.0)
	s.world.step()
	check(s.contact.normal_impulse() == 0.0, "separating at gap 0: zero impulse")
	check_near(s.ball.linear_velocity.y, 2.0, 1e-6, "separating velocity untouched (never pulled back)")
	# Positive gap, separating.
	s = ground_ball(Vector3.ZERO, 0.6, Vector3(0, 2, 0), 0.0)
	s.world.step()
	check(s.contact.normal_impulse() == 0.0, "separating with positive gap: zero impulse")
	# Penetrating but separating faster than the Baumgarte correction needs (beta/dt*|g| = 0.24 m/s): no impulse.
	s = ground_ball(Vector3.ZERO, 0.49, Vector3(0, 5, 0), 0.0)
	s.world.step()
	check(s.contact.normal_impulse() == 0.0, "penetrating but separating fast: zero impulse")
	check_near(s.ball.linear_velocity.y, 5.0, 1e-6, "fast separation untouched")


func test_normal_impulse_is_never_negative() -> void:
	# Ball thrown up, falls back, lands, rests: contact is active, inactive and active again.
	var s = ground_ball(Vector3(0, -G, 0), 0.8, Vector3(0, 1, 0), 0.0)
	var min_impulse: float = 0.0
	var min_gap: float = 0.0
	var saw_active: bool = false
	var saw_inactive: bool = false
	for i in 360:
		s.world.step()
		var lam: float = s.contact.normal_impulse()
		min_impulse = minf(min_impulse, lam)
		min_gap = minf(min_gap, s.contact.gap())
		if lam > 0.0:
			saw_active = true
		else:
			saw_inactive = true
	check(min_impulse >= 0.0, "accumulated normal impulse never negative (min %s)" % min_impulse)
	check(saw_active and saw_inactive, "contact was both active and inactive during the run")
	check(min_gap > -1.0e-4, "speculative landing does not penetrate (min gap %s)" % min_gap)
	check_near(s.contact.normal_impulse(), G * DT, 1e-5, "comes to rest supported by m g dt")


func test_row_bounds_are_unilateral() -> void:
	var s = ground_ball(Vector3.ZERO, 0.5, Vector3(0, -1, 0), 0.5)
	s.world.step()
	var row = s.contact.rows[ContactConstraint.ROW_NORMAL]
	check(row.min_impulse == 0.0 and row.max_impulse == INF, "normal row bounds are [0, +INF)")
	check(s.contact.rows.size() == 3, "contact has one normal row and two tangent rows")


func test_dynamic_dynamic_contact() -> void:
	# Head-on along X, masses 1 and 3, centered contact: perfectly inelastic, common velocity = momentum / total mass.
	var w = make_world(Vector3.ZERO)
	var a = make_sphere(1.0, 0.5, Vector3(0, 0, 0))
	var b = make_sphere(3.0, 0.5, Vector3(1.0, 0, 0))
	w.add_body(a)
	w.add_body(b)
	a.linear_velocity = Vector3(1, 0, 0)
	b.linear_velocity = Vector3(-1, 0, 0)
	var c = ContactConstraint.from_world(a, b, Vector3(0.5, 0, 0), Vector3(0.5, 0, 0), Vector3.RIGHT, 0.0)
	w.add_constraint(c)
	w.step()
	var expected: float = (1.0 * 1.0 + 3.0 * -1.0) / 4.0
	check_near(a.linear_velocity.x, expected, TOL, "A leaves with the common velocity")
	check_near(b.linear_velocity.x, expected, TOL, "B leaves with the common velocity")
	check_near(c.normal_impulse(), 1.0 * (1.0 - expected), TOL, "impulse = m_A * (v_A before - common)")
	check_vec3_near(a.linear_momentum() + b.linear_momentum(), Vector3(-2, 0, 0), TOL, "total momentum conserved")


func test_arbitrary_orientation_matches_independent_derivation() -> void:
	var s = oblique_pair(0.0)
	var a = s.a
	var b = s.b
	var n: Vector3 = s.n
	var p: Vector3 = s.p
	var vn_before: float = (b.velocity_at_point(p) - a.velocity_at_point(p)).dot(n)
	check(vn_before < -0.1, "precondition: bodies approach along the oblique normal (vn = %s)" % vn_before)
	var k: float = k_for(a, b, n, p - a.position, p - b.position)
	var c = s.contact
	check_vec3_near(c.world_normal(), n, 1e-6, "world normal follows body A's frame")
	check_vec3_near(c.world_contact_point(), p, 1e-6, "shared contact point")
	solve_in_place(c, s.world.config)
	check_near(c.normal_impulse(), -vn_before / k, 1e-5, "impulse = -vn / K with K derived independently")
	check_near((b.velocity_at_point(p) - a.velocity_at_point(p)).dot(n), 0.0, 1e-5, "normal velocity at the contact point removed")
	check_vec3_near(a.linear_momentum() + b.linear_momentum(), Vector3(2.0 * 0.5 + 3.0 * -0.6, 2.0 * 0.1 + 3.0 * -0.9, 2.0 * -0.3 + 3.0 * 0.4), 1e-5, "momentum conserved")


func test_off_center_contact_angular_response() -> void:
	# Box-like body, contact point 0.3 m beside the COM-vertical, normal +Y; torque arm r x n = (0, 0, 0.3).
	var w = make_world(Vector3.ZERO)
	var ground = make_static()
	var inertia: Basis = Basis.from_scale(Vector3(0.4, 0.5, 0.6))
	var body = make_body(2.0, inertia, Vector3(0, 1, 0))
	w.add_body(ground)
	w.add_body(body)
	body.linear_velocity = Vector3(0, -2, 0)
	var c = ContactConstraint.from_world(ground, body, Vector3(0.3, 0, 0), Vector3(0.3, 0, 0), Vector3.UP, 0.0)
	w.add_constraint(c)
	solve_in_place(c, w.config)
	var k: float = 1.0 / 2.0 + 0.3 * 0.3 / 0.6
	var lambda: float = 2.0 / k
	check_near(c.normal_impulse(), lambda, TOL, "impulse = v / (1/m + (r x n)^2 / I_zz)")
	check_near(body.linear_velocity.y, -2.0 + lambda / 2.0, TOL, "linear response lambda/m")
	check_near(body.angular_velocity.z, 0.3 * lambda / 0.6, TOL, "angular response about +Z (positive: contact is at +X)")
	check_near(body.angular_velocity.x, 0.0, TOL, "no spurious X rotation")
	check_near(body.angular_velocity.y, 0.0, TOL, "no spurious Y rotation")
	check_near(c.normal_velocity(), 0.0, TOL, "contact point no longer approaches")


func test_resting_body_is_supported_with_weight_impulse() -> void:
	var s = ground_ball(Vector3(0, -G, 0), 0.5, Vector3.ZERO, 0.5, 2.5)
	s.world.step()
	check_near(s.contact.normal_impulse(), 2.5 * G * DT, 1e-5, "first step: impulse = m g dt")
	run_steps(s.world, 239)
	check_near(s.contact.normal_impulse(), 2.5 * G * DT, 1e-5, "after 2 s: impulse = m g dt")
	check_near(s.ball.position.y, 0.5, 1e-5, "body stays at rest on the ground")
	check_near(s.ball.linear_velocity.length(), 0.0, 1e-5, "velocity stays zero")
	# Same without warm starting: the row is exact in one iteration.
	s = ground_ball(Vector3(0, -G, 0), 0.5, Vector3.ZERO, 0.5, 2.5)
	s.world.config.warm_starting = false
	run_steps(s.world, 120)
	check_near(s.contact.normal_impulse(), 2.5 * G * DT, 1e-5, "cold start: impulse = m g dt")
	check_near(s.ball.position.y, 0.5, 1e-5, "cold start: stays supported")


func test_speculative_contact_prevents_crossing() -> void:
	# Gap 0.05 m, approach 20 m/s: free flight would travel 0.1667 m per step and tunnel 0.117 m deep.
	var s = ground_ball(Vector3.ZERO, 0.55, Vector3(0, -20, 0), 0.0)
	s.world.step()
	check_near(s.ball.linear_velocity.y, -0.05 / DT, 1e-4, "velocity limited to gap/dt")
	check_near(s.contact.gap(), 0.0, 1e-6, "gap closes to exactly zero, no penetration")
	s.world.step()
	check_near(s.ball.linear_velocity.y, 0.0, 1e-4, "next step: stopped at the surface")
	check(s.contact.gap() > -1e-6, "still no penetration")
	# A gap that will not close within the step must not brake the body at all.
	s = ground_ball(Vector3.ZERO, 1.5, Vector3(0, -2, 0), 0.0)
	s.world.step()
	check(s.contact.normal_impulse() == 0.0, "gap 1 m, travel 0.0167 m: contact inactive")
	check_near(s.ball.linear_velocity.y, -2.0, 1e-6, "no early braking")
	# Exactly closing speed is still free.
	s = ground_ball(Vector3.ZERO, 0.55, Vector3(0, -0.05 / DT * 0.99, 0), 0.0)
	s.world.step()
	check(s.contact.normal_impulse() == 0.0, "approach slower than gap/dt: inactive")


func test_penetration_correction_is_bounded_and_uses_baumgarte_factor() -> void:
	# Pure Baumgarte: at rest with 2 cm penetration the contact pushes apart at beta*|g|/dt.
	var s = ground_ball(Vector3.ZERO, 0.48, Vector3.ZERO, 0.0)
	var beta: float = s.world.config.baumgarte_factor
	s.world.step()
	check_near(s.ball.linear_velocity.y, beta * 0.02 / DT, 1e-4, "push-out speed beta*|g|/dt")
	check_near(s.contact.gap(), -0.02 * (1.0 - beta), 1e-6, "a fraction beta of the penetration is removed in one step")
	# Scales linearly with the configured factor (no separate stabilization parameter).
	var s_low = ground_ball(Vector3.ZERO, 0.48, Vector3.ZERO, 0.0)
	s_low.world.config.baumgarte_factor = 0.1
	s_low.world.step()
	var s_high = ground_ball(Vector3.ZERO, 0.48, Vector3.ZERO, 0.0)
	s_high.world.config.baumgarte_factor = 0.4
	s_high.world.step()
	check_near(s_high.ball.linear_velocity.y / s_low.ball.linear_velocity.y, 4.0, 1e-4, "push-out speed proportional to beta")
	var s_zero = ground_ball(Vector3.ZERO, 0.48, Vector3.ZERO, 0.0)
	s_zero.world.config.baumgarte_factor = 0.0
	s_zero.world.step()
	check(s_zero.ball.linear_velocity.y == 0.0, "beta = 0: no positional correction (only the velocity constraint)")
	# Under gravity the penetration never grows beyond what one correction step leaves and converges to contact.
	var g = ground_ball(Vector3(0, -G, 0), 0.48, Vector3.ZERO, 0.0)
	var deepest: float = 0.0
	for i in 480:
		g.world.step()
		deepest = minf(deepest, g.contact.gap())
	check(deepest >= -0.02 * (1.0 - beta) - 1e-6, "penetration bounded by the first-step residual (deepest %s)" % deepest)
	check_near(g.contact.gap(), 0.0, 1e-5, "converges to zero gap")
	check_near(g.contact.normal_impulse(), G * DT, 1e-5, "and carries the weight")


# =====================================================================================================================
# Friction
# =====================================================================================================================

func test_zero_friction_is_frictionless() -> void:
	var s = ground_ball(Vector3.ZERO, 0.5, Vector3(3, -1, 4), 0.0)
	s.world.step()
	check(s.contact.friction_impulse_magnitude() == 0.0, "no friction impulse at mu = 0")
	check_vec3_near(s.ball.linear_velocity, Vector3(3, 0, 4), TOL, "tangential velocity untouched")
	check_vec3_near(s.ball.angular_velocity, Vector3.ZERO, TOL, "no spin generated")
	check_near(s.contact.normal_impulse(), 1.0, TOL, "normal impulse unaffected")


func test_tangential_motion_resisted_with_normal_force() -> void:
	var s = ground_ball(Vector3.ZERO, 0.5, Vector3(3, -1, 0), 0.3)
	var free = ground_ball(Vector3.ZERO, 0.5, Vector3(3, -1, 0), 0.0)
	s.world.step()
	free.world.step()
	check(s.ball.linear_velocity.x < free.ball.linear_velocity.x - 0.1, "friction reduces tangential speed")
	check(s.contact.friction_impulse_world().x < 0.0, "friction impulse on B opposes the slip direction")


func test_static_friction_holds_small_load() -> void:
	# Slow slip, large mu: sticking. Sphere K_t = 1/m + r^2/I = 1 + 0.25/0.1 = 3.5, so lambda_t = 0.1/3.5.
	var s = ground_ball(Vector3.ZERO, 0.5, Vector3(0.1, -1, 0), 1.0)
	s.world.step()
	check_near(s.contact.friction_impulse_magnitude(), 0.1 / 3.5, TOL, "sticking impulse = slip / K_t")
	check(s.contact.friction_impulse_magnitude() < s.contact.friction_limit() - 0.5, "well inside the Coulomb disk")
	check_near(s.contact.tangential_velocity().length(), 0.0, TOL, "slip removed")
	# Held against a sustained tangential load (huge inertia so the body cannot roll away): F = 0.3 m g < mu m g.
	var w = make_world(Vector3(0, -G, 0))
	var ground = make_static()
	var body = make_body(1.0, Basis.from_scale(Vector3(100, 100, 100)), Vector3(0, 0.5, 0))
	w.add_body(ground)
	w.add_body(body)
	var c = ContactConstraint.from_world(ground, body, Vector3.ZERO, Vector3.ZERO, Vector3.UP, 0.5)
	w.add_constraint(c)
	var load: float = 0.3 * G
	for i in 120:
		body.apply_force(Vector3(load, 0, 0))
		w.step()
		check(c.friction_impulse_magnitude() <= c.friction_limit() + 1e-6, "step %d: inside the Coulomb bound" % i)
	check_near(c.tangential_velocity().length(), 0.0, 1e-4, "no slip after 1 s of tangential load")
	check_near(c.friction_impulse_magnitude(), load * DT, load * DT * 0.01, "friction impulse = load * dt (1%: residual rotation, I = 100)")
	check(body.position.x < 0.01, "body displaced < 1 cm in 1 s under the held load (x = %s)" % body.position.x)
	check(c.friction_impulse_magnitude() < 0.9 * c.friction_limit(), "static friction, not saturated")


func test_sliding_friction_respects_coulomb_bound() -> void:
	# Impact 1 m/s onto the ground with 3 m/s slip: lambda_n = 1, static would need 3/3.5 = 0.857 > mu*lambda_n = 0.3.
	var s = ground_ball(Vector3.ZERO, 0.5, Vector3(3, -1, 0), 0.3)
	s.world.step()
	var c = s.contact
	check_near(c.normal_impulse(), 1.0, TOL, "normal impulse")
	check_near(c.friction_impulse_magnitude(), 0.3, TOL, "sliding: friction saturates at mu * lambda_n")
	check(c.friction_impulse_magnitude() <= c.friction_limit() * (1.0 + 1e-6), "never above the bound")
	check_vec3_near(c.friction_impulse_world(), Vector3(-0.3, 0, 0), TOL, "impulse opposes the slip direction")
	check_near(s.ball.linear_velocity.x, 2.7, TOL, "vx = 3 - mu*lambda_n/m")
	check_near(s.ball.angular_velocity.z, -1.5, 1e-4, "spin = r x F / I = -0.15/0.1")
	check(c.tangential_velocity().length() > 1.0, "still sliding afterwards (slip %s)" % c.tangential_velocity().length())


func test_sliding_then_rolling_under_gravity() -> void:
	# Solid sphere launched sliding at 2 m/s: sticks into pure rolling at 5/7 of the launch speed (angular momentum about the
	# contact line is conserved), friction saturated while sliding and strictly inside the disk once rolling.
	var s = ground_ball(Vector3(0, -G, 0), 0.5, Vector3(2, 0, 0), 0.5)
	var saw_saturated: bool = false
	var max_excess: float = 0.0
	for i in 120:
		refresh_ground_ball(s)
		s.world.step()
		var c = s.contact
		max_excess = maxf(max_excess, c.friction_impulse_magnitude() - c.friction_limit())
		if i == 0:
			saw_saturated = absf(c.friction_impulse_magnitude() - c.friction_limit()) < 1e-6
	check(saw_saturated, "first step slides: friction on the Coulomb boundary")
	check(max_excess <= 1e-6, "friction never exceeds mu*lambda_n (max excess %s)" % max_excess)
	check_near(s.ball.linear_velocity.x, 2.0 * 5.0 / 7.0, 2e-3, "rolls at 5/7 v0 (O(dt) bookkeeping of gravity impulses)")
	check_near(s.ball.angular_velocity.z, -s.ball.linear_velocity.x / 0.5, 5e-3, "rolling: omega = v / r")
	check_near(s.contact.tangential_velocity().length(), 0.0, 1e-3, "no slip once rolling")
	check(s.contact.friction_impulse_magnitude() < 0.1 * s.contact.friction_limit(), "rolling: friction well inside the disk")


func test_friction_limit_is_isotropic_for_diagonal_motion() -> void:
	# Slip (3, 4): a square (per-axis) clamp would allow |lambda_t| = 0.3*sqrt(2) = 0.424; the disk allows 0.3 along the slip.
	var s = ground_ball(Vector3.ZERO, 0.5, Vector3(3, -1, 4), 0.3)
	s.world.step()
	var c = s.contact
	check_near(c.friction_impulse_magnitude(), 0.3, TOL, "diagonal motion: |lambda_t| = mu*lambda_n exactly")
	check_vec3_near(c.friction_impulse_world(), Vector3(-0.18, 0, -0.24), TOL, "impulse points along the slip direction (0.6, 0.8)")
	check_vec3_near(s.ball.linear_velocity, Vector3(2.82, 0, 3.76), TOL, "velocity change along the slip direction")
	# Same magnitude for every direction in the plane, including with a rotated tangent basis.
	for angle in [0.0, 0.4, 1.0, 2.2, 3.9]:
		for rotation in [0.0, 0.6, -2.0]:
			var t = ground_ball(Vector3.ZERO, 0.5, Vector3(3.0 * cos(angle), -1, 3.0 * sin(angle)), 0.3)
			t.contact.tangent_rotation = rotation
			t.world.step()
			check_near(t.contact.friction_impulse_magnitude(), 0.3, TOL, "direction %s rotation %s: magnitude mu*lambda_n" % [angle, rotation])
			var expected_dir: Vector3 = -Vector3(cos(angle), 0, sin(angle))
			check_vec3_near(t.contact.friction_impulse_world(), expected_dir * 0.3, TOL, "direction %s rotation %s: opposes slip" % [angle, rotation])


func test_friction_cannot_act_without_normal_impulse() -> void:
	# Separating: no normal impulse, so no friction regardless of tangential speed or mu.
	var s = ground_ball(Vector3.ZERO, 0.5, Vector3(3, 2, 1), 5.0)
	s.world.step()
	check(s.contact.normal_impulse() == 0.0, "no normal impulse")
	check(s.contact.friction_impulse_magnitude() == 0.0, "no friction impulse")
	check_vec3_near(s.ball.linear_velocity, Vector3(3, 2, 1), 1e-6, "velocity untouched")
	check_vec3_near(s.ball.angular_velocity, Vector3.ZERO, 1e-6, "no spin")
	# Not touching (1 m gap): sliding free.
	s = ground_ball(Vector3.ZERO, 1.5, Vector3(3, -1, 0), 5.0)
	s.world.step()
	check(s.contact.friction_impulse_magnitude() == 0.0, "gap > 0, inactive: no friction")
	check_vec3_near(s.ball.linear_velocity, Vector3(3, -1, 0), 1e-6, "free flight")


func test_friction_has_no_normal_component() -> void:
	var s = ground_ball(Vector3.ZERO, 0.5, Vector3(3, -1, 4), 0.7)
	s.world.step()
	check_near(s.contact.friction_impulse_world().dot(Vector3.UP), 0.0, 1e-6, "friction impulse perpendicular to the normal (flat contact)")
	for mu in [0.2, 0.7, 5.0]:
		var o = oblique_pair(mu)
		var c = o.contact
		solve_in_place(c, o.world.config)
		check_near(c.friction_impulse_world().dot(o.n), 0.0, 1e-6, "mu %s: oblique friction impulse has no normal component" % mu)
		check_near(c.normal_velocity(), 0.0, 1e-4, "mu %s: friction does not leave a normal velocity (solver converged)" % mu)
		check(c.normal_impulse() >= 0.0, "mu %s: normal impulse non-negative" % mu)
		check(c.friction_impulse_magnitude() <= c.friction_limit() * (1.0 + 1e-6) + 1e-9, "mu %s: Coulomb bound" % mu)


func test_tangent_basis_orientation_does_not_change_physics() -> void:
	# Anisotropic K (two boxes with different inertias, oblique normal): sliding (mu 0.3) and sticking (mu 4) regimes.
	for mu in [0.3, 4.0]:
		var ref = oblique_pair(mu, 0.0)
		ref.world.step()
		for rotation in [0.7, 2.1, -1.3, 4.0]:
			var o = oblique_pair(mu, rotation)
			o.world.step()
			var label: String = "mu %s rotation %s" % [mu, rotation]
			check_vec3_near(o.a.linear_velocity, ref.a.linear_velocity, 2e-5, label + ": A linear velocity")
			check_vec3_near(o.b.linear_velocity, ref.b.linear_velocity, 2e-5, label + ": B linear velocity")
			check_vec3_near(o.a.angular_velocity, ref.a.angular_velocity, 2e-5, label + ": A angular velocity")
			check_vec3_near(o.b.angular_velocity, ref.b.angular_velocity, 2e-5, label + ": B angular velocity")
			check_vec3_near(o.contact.friction_impulse_world(), ref.contact.friction_impulse_world(), 2e-5, label + ": friction impulse vector")
			check_near(o.contact.normal_impulse(), ref.contact.normal_impulse(), 2e-5, label + ": normal impulse")
	# The two regimes really differ (the test exercises both).
	var slide = oblique_pair(0.3)
	slide.world.step()
	check(absf(slide.contact.friction_impulse_magnitude() - slide.contact.friction_limit()) < 1e-6, "mu 0.3 is sliding (on the boundary)")
	var stick = oblique_pair(4.0)
	solve_in_place(stick.contact, stick.world.config)
	check(stick.contact.friction_impulse_magnitude() < 0.9 * stick.contact.friction_limit(), "mu 4 is sticking (inside the disk)")
	check(stick.contact.tangential_velocity().length() < 1e-4, "sticking removes the slip (%s)" % stick.contact.tangential_velocity().length())


func test_friction_uses_current_normal_impulse_within_each_iteration() -> void:
	# One iteration, cold start: the normal row is solved first, so the friction bound already sees lambda_n = 1 in the same
	# iteration. Solving friction first would see lambda_n = 0 and apply no friction at all.
	var s = ground_ball(Vector3.ZERO, 0.5, Vector3(3, -1, 0), 0.3)
	s.world.config.solver_iterations = 1
	s.world.config.warm_starting = false
	s.world.step()
	check_near(s.contact.normal_impulse(), 1.0, TOL, "normal impulse after one iteration")
	check_near(s.contact.friction_impulse_magnitude(), 0.3, TOL, "friction already saturated at mu * lambda_n after one iteration")


func test_sliding_friction_opposes_residual_slip_with_anisotropic_block() -> void:
	# Maximum dissipation: at a converged sliding contact the friction impulse is antiparallel to the remaining slip velocity.
	# With two different boxes and an oblique normal the effective-mass block K is anisotropic; this holds only for the exact
	# K-metric projection (a Euclidean radial scaling would leave v_slip along -K*lambda_t instead).
	var o = oblique_pair(0.3)
	o.world.config.solver_iterations = 400
	solve_in_place(o.contact, o.world.config)
	var c = o.contact
	check(absf(c.friction_impulse_magnitude() - c.friction_limit()) < 1e-6, "sliding: on the boundary")
	var slip: Vector3 = c.tangential_velocity()
	check(slip.length() > 0.05, "residual slip exists (%s)" % slip.length())
	var f: Vector3 = c.friction_impulse_world()
	check(f.normalized().dot(slip.normalized()) < -0.99999, "friction impulse antiparallel to the slip (cos = %s)" % f.normalized().dot(slip.normalized()))


func test_warm_start_projected_into_current_friction_region() -> void:
	# Step 1: impact with sliding (static friction would need 20/3.5 = 5.7 > mu*lambda_n = 2.5); friction saturates at 0.5 * 5.
	var s = ground_ball(Vector3.ZERO, 0.5, Vector3(20, -5, 0), 0.5)
	s.world.step()
	var c = s.contact
	check_near(c.friction_impulse_magnitude(), 0.5 * 5.0, 1e-4, "step 1: saturated at mu * lambda_n")
	check_near(c.normal_impulse(), 5.0, 1e-4, "step 1: normal impulse")
	# The friction limit drops from 0.5 to 0.1: the carried impulse must be clipped to the new disk.
	c.friction = 0.1
	c.prepare(DT, s.world.config)
	var limit: float = c.friction * c.normal_impulse()
	check(c.normal_impulse() > 0.0, "normal impulse was warm started")
	check(c.friction_impulse_magnitude() <= limit + 1e-9, "after prepare: |lambda_t| %s <= new limit %s" % [c.friction_impulse_magnitude(), limit])
	check_near(c.friction_impulse_magnitude(), limit, 1e-6, "clipped radially onto the new boundary")
	# Friction coefficient raised again: the (now smaller) carried impulse stays admissible, nothing is amplified.
	c.friction = 5.0
	var before: float = c.friction_impulse_magnitude()
	s.world.step()
	check(c.friction_impulse_magnitude() <= c.friction_limit() + 1e-6, "next step: inside the bound")
	check(before <= limit + 1e-9, "carried value was not amplified by the larger mu")


func test_warm_start_with_changing_normal_load() -> void:
	# Sliding while the support is unloaded: an upward force cancels gravity linearly over 1 s, so lambda_n -> 0.
	var s = ground_ball(Vector3(0, -G, 0), 0.5, Vector3(2, 0, 0), 0.5)
	var c = s.contact
	var worst_excess: float = -INF
	var last_normal: float = 1.0
	for i in 120:
		var unload: float = float(i) / 119.0
		s.ball.apply_force(Vector3(0, G * unload, 0))
		refresh_ground_ball(s)
		s.world.step()
		worst_excess = maxf(worst_excess, c.friction_impulse_magnitude() - c.friction_limit())
		last_normal = c.normal_impulse()
	check(worst_excess <= 1e-6, "Coulomb bound holds at every step while the load changes (worst excess %s)" % worst_excess)
	check_near(last_normal, 0.0, 1e-5, "support fully unloaded at the end")
	check_near(c.friction_impulse_magnitude(), 0.0, 1e-5, "no friction without normal load")
	# Warm starting disabled: same bound, impulses start from zero each step.
	var cold = ground_ball(Vector3(0, -G, 0), 0.5, Vector3(2, 0, 0), 0.5)
	cold.world.config.warm_starting = false
	for i in 60:
		refresh_ground_ball(cold)
		cold.world.step()
		check(cold.contact.friction_impulse_magnitude() <= cold.contact.friction_limit() + 1e-6, "cold start: bound holds")


func test_multiple_contacts_in_one_world() -> void:
	# (a) A box on four corner contacts carries its weight.
	var w = make_world(Vector3(0, -G, 0))
	var ground = make_static()
	var box = make_body(4.0, SimMassProperties.box_inertia(4.0, Vector3(1.0, 0.5, 1.0)), Vector3(0, 0.25, 0))
	w.add_body(ground)
	w.add_body(box)
	var contacts: Array = []
	for sx in [-0.5, 0.5]:
		for sz in [-0.5, 0.5]:
			var c = ContactConstraint.from_world(ground, box, Vector3(sx, 0, sz), Vector3(sx, 0, sz), Vector3.UP, 0.5)
			w.add_constraint(c)
			contacts.append(c)
	run_steps(w, 240)
	var total: float = 0.0
	for c in contacts:
		check(c.normal_impulse() >= 0.0, "corner impulse non-negative")
		total += c.normal_impulse()
	check_near(total, 4.0 * G * DT, 4.0 * G * DT * 0.02, "sum of corner impulses = m g dt (2%: GS on a statically indeterminate support)")
	check_near(box.position.y, 0.25, 1e-3, "box stays on its corners")
	check(box.angular_velocity.length() < 1e-2, "box does not tip (|w| = %s)" % box.angular_velocity.length())
	check(box.linear_velocity.length() < 1e-2, "box at rest")
	# (b) Independent contacts in one world do not corrupt each other: identical to running them alone.
	var combined = make_world(Vector3(0, -G, 0))
	var ground2 = make_static()
	combined.add_body(ground2)
	var specs: Array = [
		{"x": 0.0, "v": Vector3(3, -1, 0), "mu": 0.3},
		{"x": 5.0, "v": Vector3(-1, -2, 1), "mu": 0.8},
	]
	var balls: Array = []
	var cons: Array = []
	for spec in specs:
		var ball = make_sphere(1.0, 0.5, Vector3(spec.x, 0.5, 0))
		ball.linear_velocity = spec.v
		combined.add_body(ball)
		var cc = ContactConstraint.from_world(ground2, ball, Vector3(spec.x, 0, 0), Vector3(spec.x, 0, 0), Vector3.UP, spec.mu)
		combined.add_constraint(cc)
		balls.append(ball)
		cons.append(cc)
	run_steps(combined, 30)
	for i in specs.size():
		var spec = specs[i]
		var alone = make_world(Vector3(0, -G, 0))
		var g1 = make_static()
		var b1 = make_sphere(1.0, 0.5, Vector3(spec.x, 0.5, 0))
		b1.linear_velocity = spec.v
		alone.add_body(g1)
		alone.add_body(b1)
		var c1 = ContactConstraint.from_world(g1, b1, Vector3(spec.x, 0, 0), Vector3(spec.x, 0, 0), Vector3.UP, spec.mu)
		alone.add_constraint(c1)
		run_steps(alone, 30)
		check_vec3_near(balls[i].linear_velocity, b1.linear_velocity, 1e-6, "ball %d velocity identical to the solo run" % i)
		check_vec3_near(balls[i].position, b1.position, 1e-6, "ball %d position identical to the solo run" % i)
		check_vec3_near(cons[i].friction_impulse_world(), c1.friction_impulse_world(), 1e-6, "ball %d friction state identical" % i)
		check_near(cons[i].normal_impulse(), c1.normal_impulse(), 1e-6, "ball %d normal state identical" % i)


# =====================================================================================================================
# Conservation and regression
# =====================================================================================================================

func isolated_pair() -> Dictionary:
	var w = make_world(Vector3.ZERO)
	var n: Vector3 = Vector3(0.6, 0.8, 0.0)
	var a = make_sphere(2.0, 0.4, Vector3(0.2, -0.3, 0.1))
	var b = make_sphere(1.0, 0.3, a.position + n * 0.7)
	w.add_body(a)
	w.add_body(b)
	a.linear_velocity = Vector3(1.0, 0.5, 0.0)
	b.linear_velocity = Vector3(-0.5, -1.0, 0.3)
	a.angular_velocity = Vector3(0.3, -0.7, 1.0)
	b.angular_velocity = Vector3(-1.0, 0.4, 0.2)
	var c = ContactConstraint.from_world(a, b, a.position + n * 0.4, b.position - n * 0.3, n, 0.6)
	w.add_constraint(c)
	return {"world": w, "a": a, "b": b, "contact": c}


func test_linear_momentum_conserved() -> void:
	var s = isolated_pair()
	var p0: Vector3 = s.world.total_linear_momentum()
	s.world.step()
	check(s.contact.normal_impulse() > 0.1, "the contact acted (impulse %s)" % s.contact.normal_impulse())
	check(s.contact.friction_impulse_magnitude() > 0.01, "friction acted")
	check_vec3_near(s.world.total_linear_momentum(), p0, 1e-5, "total linear momentum unchanged by one contact step")
	run_steps(s.world, 60)
	check_vec3_near(s.world.total_linear_momentum(), p0, 1e-5, "and over 0.5 s")
	for row in s.contact.rows:
		check_vec3_near(row.linear_a + row.linear_b, Vector3.ZERO, 1e-12, "row linear Jacobians are equal and opposite")


func test_angular_momentum_about_origin_conserved() -> void:
	# Impulses at one shared point: change of total angular momentum about the origin is p x lambda - p x lambda = 0.
	# Spheres (isotropic inertia) in free flight conserve it exactly under semi-implicit Euler (x += v dt, no gyroscopic term),
	# so the tolerance only has to absorb float32 rounding of values of order 5 (1e-4 absolute ~ 2e-5 relative).
	var s = isolated_pair()
	var l0: Vector3 = angular_momentum_about_origin(s.a) + angular_momentum_about_origin(s.b)
	for i in 60:
		s.world.step()
		var l: Vector3 = angular_momentum_about_origin(s.a) + angular_momentum_about_origin(s.b)
		check_vec3_near(l, l0, 1e-4, "step %d: angular momentum about the origin" % i)
	# Same with a gap (anchors not coincident): the shared midpoint keeps the impulses collinear.
	var w = make_world(Vector3.ZERO)
	var a = make_sphere(2.0, 0.4, Vector3(0, 0, 0))
	var b = make_sphere(1.0, 0.3, Vector3(0.0, 0.69, 0.0))
	w.add_body(a)
	w.add_body(b)
	b.linear_velocity = Vector3(0.5, -2.0, 0.0)
	var c = ContactConstraint.from_world(a, b, Vector3(0, 0.4, 0), Vector3(0, 0.39, 0), Vector3.UP, 0.4)
	w.add_constraint(c)
	var l1: Vector3 = angular_momentum_about_origin(a) + angular_momentum_about_origin(b)
	run_steps(w, 20)
	check(c.normal_impulse() >= 0.0, "contact solved")
	check_vec3_near(angular_momentum_about_origin(a) + angular_momentum_about_origin(b), l1, 1e-4, "with a 1 cm penetrating gap")


func test_static_body_is_unchanged() -> void:
	var s = ground_ball(Vector3(0, -G, 0), 0.8, Vector3(4, -6, 2), 0.7, 5.0)
	var ground = s.ground
	var position_before: Vector3 = ground.position
	var orientation_before: Quaternion = ground.orientation
	for i in 240:
		refresh_ground_ball(s)
		s.world.step()
	check(ground.position == position_before, "static position exactly unchanged")
	check(ground.orientation == orientation_before, "static orientation exactly unchanged")
	check(ground.linear_velocity == Vector3.ZERO, "static linear velocity exactly zero")
	check(ground.angular_velocity == Vector3.ZERO, "static angular velocity exactly zero")


func test_disabled_contact_has_no_effect() -> void:
	var s = ground_ball(Vector3(0, -G, 0), 0.48, Vector3(2, -3, 0), 0.5)
	s.contact.enabled = false
	var reference = make_world(Vector3(0, -G, 0))
	var ref_ball = make_sphere(1.0, 0.5, Vector3(0, 0.48, 0))
	ref_ball.linear_velocity = Vector3(2, -3, 0)
	reference.add_body(make_static())
	reference.add_body(ref_ball)
	run_steps(s.world, 60)
	run_steps(reference, 60)
	check(s.ball.position == ref_ball.position, "position identical to a world without the contact")
	check(s.ball.linear_velocity == ref_ball.linear_velocity, "velocity identical to a world without the contact")
	check(s.ball.position.y < -1.0, "the ball fell through the (disabled) ground")
	# Re-enabling restores the effect.
	s.contact.enabled = true
	var z = ground_ball(Vector3(0, -G, 0), 0.5, Vector3(0, -3, 0), 0.0)
	z.world.step()
	check(z.contact.normal_impulse() > 0.0, "an enabled contact acts")


func test_mixed_with_ball_joint_pendulum() -> void:
	# Existing joints keep working next to contacts: a 1.2 m pendulum whose bob would dip 0.2 m below the ground plane.
	var w = make_world(Vector3(0, -G, 0))
	var pivot = make_static(Vector3(0, 1.5, 0))
	var ground = make_static()
	var bob = make_sphere(1.0, 0.1, Vector3(1.2, 1.5, 0))
	w.add_body(pivot)
	w.add_body(ground)
	w.add_body(bob)
	var joint = BallJoint.new(pivot, bob, Vector3.ZERO, Vector3(-1.2, 0, 0))
	w.add_constraint(joint)
	# Contact anchors are re-seated every step below (the caller's duty); the initial bob bottom is at (1.2, 1.4, 0).
	var contact = ContactConstraint.from_world(ground, bob, Vector3(1.2, 0, 0), Vector3(1.2, 1.4, 0), Vector3.UP, 0.3)
	w.add_constraint(contact)
	var min_gap: float = INF
	var max_joint_error: float = 0.0
	var min_lambda: float = INF
	for i in 480:
		contact.local_anchor_a = Vector3(bob.position.x, 0, bob.position.z)
		contact.local_anchor_b = bob.orientation.inverse() * Vector3(0, -0.1, 0)
		w.step()
		min_gap = minf(min_gap, contact.gap())
		min_lambda = minf(min_lambda, contact.normal_impulse())
		max_joint_error = maxf(max_joint_error, joint.linear_error().length())
	check(is_finite(bob.position.x) and is_finite(bob.position.y), "state finite")
	check(min_lambda >= 0.0, "contact impulse non-negative alongside a joint")
	check(min_gap > -0.01, "bob does not sink into the ground (min gap %s)" % min_gap)
	check(max_joint_error < 0.01, "ball joint stays closed (max anchor error %s)" % max_joint_error)
	check(bob.position.y > 0.1 - 0.01, "bob is held above the ground plane")


func test_hinge_and_contact_coexist() -> void:
	# A door-like hinge body resting on a stop (contact) under gravity: hinge limit/axis rows and the contact row cooperate.
	var w = make_world(Vector3(0, -G, 0))
	var frame = make_static()
	var stop = make_static()
	var door = make_body(2.0, SimMassProperties.box_inertia(2.0, Vector3(1.0, 0.1, 0.4)), Vector3(0.5, 0, 0))
	w.add_body(frame)
	w.add_body(stop)
	w.add_body(door)
	var hinge = HingeJoint.from_world(frame, door, Vector3.ZERO, Vector3.BACK, Vector3.RIGHT)
	check(hinge != null, "hinge built")
	w.add_constraint(hinge)
	# Door tip at (1, -0.05, 0) rests on a stop below it.
	var contact = ContactConstraint.from_world(stop, door, Vector3(1, -0.05, 0), Vector3(1, -0.05, 0), Vector3.UP, 0.4)
	w.add_constraint(contact)
	run_steps(w, 240)
	check_near(contact.normal_impulse(), 0.5 * 2.0 * G * DT, 0.5 * 2.0 * G * DT * 0.05, "the stop carries half the door's weight (hinge carries the rest)")
	check(hinge.linear_error().length() < 1e-3, "hinge anchor stays closed (%s)" % hinge.linear_error().length())
	check(hinge.axis_error_angle() < 1e-3, "hinge axis stays aligned")
	check(absf(door.angular_velocity.z) < 0.05, "door at rest on the stop (w_z = %s)" % door.angular_velocity.z)


# =====================================================================================================================
# Validation
# =====================================================================================================================

func test_invalid_input_rejected_at_registration() -> void:
	var w = make_world()
	var a = make_sphere(1.0, 0.5, Vector3(0, 1, 0))
	var b = make_sphere(1.0, 0.5, Vector3(0, 2, 0))
	var st1 = make_static()
	var st2 = make_static(Vector3(3, 0, 0))
	for body in [a, b, st1, st2]:
		w.add_body(body)
	var good = ContactConstraint.new(a, b, Vector3(0, 0.5, 0), Vector3(0, -0.5, 0), Vector3.UP, 0.5)
	check(good.is_valid(), "baseline contact is valid")
	var cases: Array = [
		{"label": "NaN anchor A", "make": func(): return ContactConstraint.new(a, b, Vector3(NAN, 0, 0), Vector3.ZERO, Vector3.UP, 0.5)},
		{"label": "infinite anchor B", "make": func(): return ContactConstraint.new(a, b, Vector3.ZERO, Vector3(INF, 0, 0), Vector3.UP, 0.5)},
		{"label": "zero normal", "make": func(): return ContactConstraint.new(a, b, Vector3.ZERO, Vector3.ZERO, Vector3.ZERO, 0.5)},
		{"label": "NaN normal", "make": func(): return ContactConstraint.new(a, b, Vector3.ZERO, Vector3.ZERO, Vector3(NAN, 1, 0), 0.5)},
		{"label": "non-unit normal", "make": func(): return ContactConstraint.new(a, b, Vector3.ZERO, Vector3.ZERO, Vector3(0, 1.1, 0), 0.5)},
		{"label": "short normal", "make": func(): return ContactConstraint.new(a, b, Vector3.ZERO, Vector3.ZERO, Vector3(0, 0.9, 0), 0.5)},
		{"label": "negative friction", "make": func(): return ContactConstraint.new(a, b, Vector3.ZERO, Vector3.ZERO, Vector3.UP, -0.1)},
		{"label": "NaN friction", "make": func(): return ContactConstraint.new(a, b, Vector3.ZERO, Vector3.ZERO, Vector3.UP, NAN)},
		{"label": "infinite friction", "make": func(): return ContactConstraint.new(a, b, Vector3.ZERO, Vector3.ZERO, Vector3.UP, INF)},
		{"label": "null body B", "make": func(): return ContactConstraint.new(a, null, Vector3.ZERO, Vector3.ZERO, Vector3.UP, 0.5)},
		{"label": "null body A", "make": func(): return ContactConstraint.new(null, b, Vector3.ZERO, Vector3.ZERO, Vector3.UP, 0.5)},
		{"label": "same body twice", "make": func(): return ContactConstraint.new(a, a, Vector3.ZERO, Vector3.ZERO, Vector3.UP, 0.5)},
		{"label": "two static bodies", "make": func(): return ContactConstraint.new(st1, st2, Vector3.ZERO, Vector3.ZERO, Vector3.UP, 0.5)},
	]
	for case in cases:
		var c = case.make.call()
		check(not c.is_valid(), "%s reports errors" % case.label)
		check(w.add_constraint(c) == null, "%s rejected by add_constraint" % case.label)
	var nan_rotation = ContactConstraint.new(a, b, Vector3.ZERO, Vector3.ZERO, Vector3.UP, 0.5)
	nan_rotation.tangent_rotation = NAN
	check(not nan_rotation.is_valid() and w.add_constraint(nan_rotation) == null, "NaN tangent_rotation rejected")
	check(w.constraints.is_empty(), "nothing invalid was registered")
	check(w.add_constraint(good) == good, "the valid contact is accepted")
	check(ContactConstraint.from_world(a, b, Vector3.ZERO, Vector3.ZERO, Vector3(0, 2, 0), 0.5) == null, "from_world rejects a non-unit normal")
	check(ContactConstraint.from_world(a, b, Vector3.ZERO, Vector3.ZERO, Vector3.UP, -1.0) == null, "from_world rejects negative friction")


func test_invalid_data_rejected_by_runtime_preflight() -> void:
	var mutations: Array = [
		{"label": "anchor A", "name": "local_anchor_a", "bad": Vector3(NAN, 0, 0)},
		{"label": "anchor B", "name": "local_anchor_b", "bad": Vector3(0, INF, 0)},
		{"label": "zero normal", "name": "local_normal_a", "bad": Vector3.ZERO},
		{"label": "non-unit normal", "name": "local_normal_a", "bad": Vector3(0, 2, 0)},
		{"label": "NaN normal", "name": "local_normal_a", "bad": Vector3(0, NAN, 0)},
		{"label": "negative friction", "name": "friction", "bad": -0.5},
		{"label": "NaN friction", "name": "friction", "bad": NAN},
		{"label": "tangent rotation", "name": "tangent_rotation", "bad": INF},
	]
	for m in mutations:
		var s = ground_ball(Vector3.ZERO, 0.5, Vector3.ZERO, 0.5)
		var w = s.world
		check(w.step(), "%s: valid contact steps" % m.label)
		var position_before: Vector3 = s.ball.position
		var time_before: float = w.simulation_time
		var good_value = s.contact.get(m.name)
		s.contact.set(m.name, m.bad)
		check(not w.step(), "%s: step refused after the contact became invalid" % m.label)
		check(w.advance(DT) == 0, "%s: advance refused" % m.label)
		check(w.last_error.contains("constraint 0 is invalid"), "%s: last_error names the constraint (%s)" % [m.label, w.last_error])
		check(s.ball.position == position_before and w.simulation_time == time_before, "%s: no state change while refused" % m.label)
		s.contact.set(m.name, good_value)
		check(w.step(), "%s: resumes once valid again" % m.label)


func test_membership_still_enforced_for_contacts() -> void:
	var w = make_world()
	var a = make_sphere(1.0, 0.5, Vector3(0, 1, 0))
	var b = make_sphere(1.0, 0.5, Vector3(0, 2, 0))
	w.add_body(a)
	var c = ContactConstraint.new(a, b, Vector3.ZERO, Vector3.ZERO, Vector3.UP, 0.5)
	check(w.add_constraint(c) == null, "a contact whose body B is not in the world is rejected (world-membership invariant)")
	w.add_body(b)
	check(w.add_constraint(c) == c, "accepted once both bodies are members")
	w.remove_body(b)
	check(w.constraints.is_empty(), "removing a body removes its contacts")
