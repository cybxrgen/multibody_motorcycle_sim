extends "res://Tests/sim_test_case.gd"
## Split-impulse stabilization (M1.3): positional constraint error is corrected by a separate solve on temporary "split"
## velocities that move the pose but never become physical velocity (PHYSICS_CONVENTIONS.md, "Split-impulse stabilization").
##
## Tolerance policy (TESTING.md): float32 vector quantities >= 1e-6 relative. Where a quantity must be EXACTLY unchanged by the
## correction (a body at rest whose constraint velocities are all zero) the test compares with == : the physical solve then adds
## literally zero impulse, so any nonzero value is a leak of the split channel into the physical channel.
## Where the expectation is the analytic result of one correction step (anchor/gap error becomes (1 - beta) * error), the
## tolerance covers the second-order term r * theta^2 / 2 of the rotation the correction itself produces (stated per test).
## Counterfactual: with the former velocity-level Baumgarte bias the first step of A1/A2/A3 leaves a physical velocity of
## beta * |error| / dt (0.48 m/s for the 2 cm contact below), i.e. 0.115 J of kinetic energy out of nothing; these tests
## compare against zero and therefore fail for that implementation.

const ContactConstraint = preload("res://Simulation/Solver/contact_constraint.gd")
const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")
const ConstraintSolver = preload("res://Simulation/Solver/constraint_solver.gd")

const DT: float = 1.0 / 120.0
const G: float = 9.81
## Single-contact solves are exact after one iteration: float32 rounding of a few ulps on O(1) values.
const TOL: float = 1.0e-5
## Anchor/gap error after one correction step: (1 - beta) * error, up to the second-order rotation term r * theta^2 / 2.
const ERR_TOL: float = 1.0e-4
## Hinge limit stops need ~80 iterations to converge (PROJECT_STATE.md); split passes are cold started every step.
const ITERATIONS: int = 100


# =====================================================================================================================
# Builders
# =====================================================================================================================

## Static ground (plane y = 0, normal +Y from ground toward the ball) and a solid sphere whose lowest point is the contact point.
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


## Static pivot at the origin, a sphere bob 1 m along +X whose anchor is displaced by `error` from the pivot.
func ball_pendulum(error: Vector3, gravity: Vector3 = Vector3.ZERO, static_pivot: bool = true) -> Dictionary:
	var w = make_world(gravity)
	w.config.solver_iterations = ITERATIONS
	var pivot = make_static(Vector3.ZERO) if static_pivot else make_sphere(1.0, 0.5, Vector3.ZERO)
	var bob = make_sphere(1.0, 0.5, Vector3(1.0, 0.0, 0.0) + error)
	w.add_body(pivot)
	w.add_body(bob)
	var joint = BallJoint.new(pivot, bob, Vector3.ZERO, Vector3(-1, 0, 0))
	w.add_constraint(joint)
	return {"world": w, "pivot": pivot, "bob": bob, "joint": joint}


## Hinge about world +Z at `anchor`: A (static or dynamic box) and a dynamic box B offset from the anchor.
func hinge_pair(static_a: bool = true, gravity: Vector3 = Vector3.ZERO) -> Dictionary:
	var w = make_world(gravity)
	w.config.solver_iterations = ITERATIONS
	var anchor: Vector3 = Vector3(0.3, 1.0, -0.2)
	var a
	if static_a:
		a = make_static(anchor + Vector3(-0.4, 0.1, 0.2))
	else:
		a = make_body(2.0, SimMassProperties.box_inertia(2.0, Vector3(0.6, 0.4, 0.5)), anchor + Vector3(-0.4, 0.1, 0.2))
	var b = make_body(1.0, SimMassProperties.box_inertia(1.0, Vector3(0.5, 0.3, 0.4)), anchor + Vector3(0.5, -0.2, 0.1))
	w.add_body(a)
	w.add_body(b)
	var joint = HingeJoint.from_world(a, b, anchor, Vector3.BACK, Vector3.RIGHT)
	check(joint != null, "hinge construction")
	w.add_constraint(joint)
	return {"world": w, "a": a, "b": b, "joint": joint, "anchor": anchor, "axis": Vector3.BACK}


## Pose edit: rotate B about the world axis through the hinge anchor (keeps the anchor satisfied).
func rotate_b_about(s: Dictionary, axis: Vector3, phi: float) -> void:
	var dq: Quaternion = Quaternion(axis, phi)
	s.b.position = s.anchor + dq * (s.b.position - s.anchor)
	s.b.orientation = dq * s.b.orientation
	s.joint.angle()  # sample the tracker so the continuous angle follows the edit


## prepare + physical pass + split pass of the world's constraints WITHOUT integrating (inspect the mid-step state).
func solve_only(world) -> ConstraintSolver:
	var solver: ConstraintSolver = ConstraintSolver.new()
	solver.solve(world.constraints, world.config.fixed_timestep, world.config)
	return solver


func kinetic_energy_of(world) -> float:
	return world.total_kinetic_energy()


## Hinge about world +Z at `anchor`: static A and a dynamic body B whose COM sits on the anchor and whose local Z axis (a PRINCIPAL axis
## of `inertia`) is the hinge axis. B is then tilted by `tilt` about world X (axis-alignment error, sine-type C = -sin(tilt)) and its
## COM is displaced by `anchor_error` (anchor error), after which its physical angular velocity is `spin` about its own hinge axis.
## That spin satisfies the physical axis rows (it is parallel to B's hinge axis) and, being along a principal axis, has no gyroscopic
## torque, so the physical solve adds nothing and the split pass is the only thing that moves the pose. `beta` is set on the config.
func tilted_hinge_spinner(beta: float, inertia: Basis, tilt: float, anchor_error: Vector3, spin: float) -> Dictionary:
	var w = make_world(Vector3.ZERO)
	w.config.baumgarte_factor = beta
	w.config.solver_iterations = ITERATIONS
	var anchor: Vector3 = Vector3(0.3, 1.0, -0.2)
	var a = make_static(anchor)
	var b = make_body(2.0, inertia, anchor)
	w.add_body(a)
	w.add_body(b)
	var joint = HingeJoint.from_world(a, b, anchor, Vector3.BACK, Vector3.RIGHT)  # built satisfied, perturbed below
	check(joint != null, "hinge construction")
	w.add_constraint(joint)
	b.orientation = Quaternion(Vector3.RIGHT, tilt) * b.orientation
	b.position = anchor + anchor_error
	b.angular_velocity = spin * (b.orientation * Vector3.BACK)
	return {"world": w, "a": a, "b": b, "joint": joint}


# =====================================================================================================================
# A. Separation of physical and split velocities
# =====================================================================================================================

func test_a1_penetrating_contact_is_corrected_without_physical_velocity() -> void:
	var beta: float = 0.2  # SimConfig default, asserted below
	# At rest, 2 cm deep, no gravity: nothing physical happens, only the pose is corrected.
	var s = ground_ball(Vector3.ZERO, 0.48, Vector3.ZERO, 0.0)
	check_near(s.world.config.baumgarte_factor, beta, 0.0, "default beta")
	s.world.step()
	check(s.ball.linear_velocity == Vector3.ZERO, "physical linear velocity exactly zero (%s)" % s.ball.linear_velocity)
	check(s.ball.angular_velocity == Vector3.ZERO, "physical angular velocity exactly zero")
	check(s.contact.normal_impulse() == 0.0, "no physical normal impulse is needed")
	check_near(s.ball.position.y, 0.48 + beta * 0.02, 1.0e-6, "pose corrected by beta * |g|")
	check_near(s.contact.gap(), -0.02 * (1.0 - beta), 1.0e-6, "a fraction beta of the penetration removed in one step")
	# Penetrating AND approaching at 1 m/s: the physical impulse is exactly m * v (stop), the correction stays out of it.
	var t = ground_ball(Vector3.ZERO, 0.48, Vector3(0, -1.0, 0), 0.0)
	t.world.step()
	check_near(t.contact.normal_impulse(), 1.0, TOL, "physical impulse = m * v")
	check_near(t.ball.linear_velocity.y, 0.0, TOL, "approach stopped, nothing more (old Baumgarte: +%s)" % (beta * 0.02 / DT))
	check_near(t.ball.position.y, 0.48 + beta * 0.02, 1.0e-6, "pose still corrected by the split velocity")
	# Beta changes only the pose, never the physical result while the contact stays penetrating.
	var u = ground_ball(Vector3.ZERO, 0.48, Vector3(0, -1.0, 0), 0.0)
	u.world.config.baumgarte_factor = 0.0
	u.world.step()
	check(u.ball.linear_velocity == t.ball.linear_velocity, "physical velocity independent of beta")
	check_near(u.ball.position.y, 0.48, 1.0e-7, "beta = 0: no positional correction")


func test_a2_contact_correction_injects_no_kinetic_energy() -> void:
	var s = ground_ball(Vector3.ZERO, 0.48, Vector3.ZERO, 0.0)
	var max_ke: float = 0.0
	for i in 240:
		s.world.step()
		max_ke = maxf(max_ke, s.world.total_kinetic_energy())
	check(max_ke == 0.0, "zero gravity, at rest: kinetic energy stays exactly zero (max %s; old Baumgarte 0.1152 J)" % max_ke)
	check_near(s.contact.gap(), 0.0, 1.0e-6, "the penetration is nevertheless gone (0.8^240 of 2 cm)")
	# Under gravity, resting on the ground from a 2 cm penetration: still no energy beyond what gravity physically provides.
	var g = ground_ball(Vector3(0, -G, 0), 0.48, Vector3.ZERO, 0.0)
	var max_speed: float = 0.0
	for i in 240:
		g.world.step()
		max_speed = maxf(max_speed, g.ball.linear_velocity.length())
	check(max_speed < 1.0e-5, "resting contact never acquires a physical velocity (max %s)" % max_speed)
	check_near(g.contact.normal_impulse(), G * DT, TOL, "and carries the weight")


func test_a3_ball_joint_correction_changes_pose_not_velocity() -> void:
	var error: Vector3 = Vector3(0.03, -0.02, 0.01)
	var s = ball_pendulum(error)
	var c0: Vector3 = s.joint.linear_error()
	check_vec3_near(c0, error, 1.0e-6, "initial anchor error")
	s.world.step()
	check(s.bob.linear_velocity == Vector3.ZERO, "physical linear velocity exactly zero (%s)" % s.bob.linear_velocity)
	check(s.bob.angular_velocity == Vector3.ZERO, "physical angular velocity exactly zero (%s)" % s.bob.angular_velocity)
	# The 3x3 anchor block is diagonal here (lever arm along X), so one split step removes exactly beta of the error, up to the
	# second-order rotation r*theta^2/2 ~ 3e-5.
	check_vec3_near(s.joint.linear_error(), error * 0.8, ERR_TOL, "anchor error becomes (1 - beta) * error")
	for i in 60:
		s.world.step()
	check(s.bob.linear_velocity == Vector3.ZERO and s.bob.angular_velocity == Vector3.ZERO, "still no physical velocity after 60 steps")
	check(s.joint.linear_error().length() < 1.0e-4 * error.length() * 100.0, "error keeps decreasing (%s)" % s.joint.linear_error().length())
	check(s.world.total_kinetic_energy() == 0.0, "no kinetic energy injected")


func test_a4_hinge_anchor_and_axis_correction_leave_no_physical_velocity() -> void:
	var s = hinge_pair()
	s.b.position += Vector3(0.03, -0.02, 0.01)  # anchor error
	rotate_b_about(s, Vector3.RIGHT, 0.06)         # axis error 0.06 rad about the anchor
	var anchor_error0: float = s.joint.linear_error().length()
	var axis_error0: float = s.joint.axis_error_angle()
	check(anchor_error0 > 0.03 and axis_error0 > 0.05, "both errors present")
	s.world.step()
	check(s.b.linear_velocity == Vector3.ZERO, "physical linear velocity exactly zero (%s)" % s.b.linear_velocity)
	check(s.b.angular_velocity == Vector3.ZERO, "physical angular velocity exactly zero (%s)" % s.b.angular_velocity)
	check(s.joint.linear_error().length() < 0.9 * anchor_error0, "anchor error decreased (%s -> %s)" % [anchor_error0, s.joint.linear_error().length()])
	check(s.joint.axis_error_angle() < 0.9 * axis_error0, "axis error decreased (%s -> %s)" % [axis_error0, s.joint.axis_error_angle()])
	for i in 30:
		s.world.step()
	check(s.b.linear_velocity == Vector3.ZERO and s.b.angular_velocity == Vector3.ZERO, "no physical velocity after 31 steps")
	check(s.joint.axis_error_angle() < 0.1 * axis_error0, "axis error keeps decreasing (%s)" % s.joint.axis_error_angle())


func test_a5_violated_hinge_limit_is_corrected_without_physical_velocity() -> void:
	var beta: float = 0.2
	# Lower limit -0.5, B pushed to -0.7 (violation 0.2 rad).
	var s = hinge_pair()
	s.joint.limit_enabled = true
	s.joint.lower_limit = -0.5
	s.joint.upper_limit = 0.6
	rotate_b_about(s, s.axis, -0.7)
	s.world.step()
	check(s.b.angular_velocity == Vector3.ZERO, "lower: physical angular velocity exactly zero (%s)" % s.b.angular_velocity)
	check(s.b.linear_velocity == Vector3.ZERO, "lower: physical linear velocity exactly zero")
	check_near(s.joint.angle(), -0.7 + beta * 0.2, 2.0e-3, "lower: one step moves the angle inward by beta * violation")
	for i in 100:
		s.world.step()
	check(s.joint.angle() >= -0.5 - 1.0e-4 and s.joint.angle() < -0.5 + 1.0e-3, "lower: converges onto the limit without overshoot (%s)" % s.joint.angle())
	check(s.b.angular_velocity == Vector3.ZERO, "lower: no physical angular velocity left behind")
	# Upper limit 0.6, B pushed to 0.8.
	var u = hinge_pair()
	u.joint.limit_enabled = true
	u.joint.lower_limit = -0.5
	u.joint.upper_limit = 0.6
	rotate_b_about(u, u.axis, 0.8)
	u.world.step()
	check(u.b.angular_velocity == Vector3.ZERO, "upper: physical angular velocity exactly zero")
	check_near(u.joint.angle(), 0.8 - beta * 0.2, 2.0e-3, "upper: one step moves the angle inward")
	for i in 100:
		u.world.step()
	check(u.joint.angle() <= 0.6 + 1.0e-4 and u.joint.angle() > 0.6 - 1.0e-3, "upper: converges onto the limit without overshoot (%s)" % u.joint.angle())
	check(u.b.angular_velocity == Vector3.ZERO, "upper: no physical angular velocity left behind")


func test_a6_physical_effects_still_change_physical_velocity() -> void:
	# Gravity: free fall v = g t.
	var w = make_world(Vector3(0, -G, 0))
	var free = make_sphere(1.0, 0.5, Vector3(0, 10, 0))
	w.add_body(free)
	run_steps(w, 12)
	check_near(free.linear_velocity.y, -G * 12 * DT, 1.0e-5, "gravity changes physical velocity")
	# Normal contact: impulse = m v (m = 1.5, v = 2).
	var n = ground_ball(Vector3.ZERO, 0.5, Vector3(0, -2.0, 0), 0.0, 1.5)
	n.world.step()
	check_near(n.contact.normal_impulse(), 3.0, TOL, "normal impulse m * v")
	check_near(n.ball.linear_velocity.y, 0.0, TOL, "approach stopped")
	# Friction: sliding sphere (m = 1, I = 0.1), v = (3, -1, 0), mu = 0.3. The normal impulse is 1, the stop impulse 3/3.5 is above
	# mu * 1 = 0.3, so friction saturates: dv_x = -0.3.
	var f = ground_ball(Vector3.ZERO, 0.5, Vector3(3, -1, 0), 0.3)
	f.world.step()
	check_near(f.contact.friction_impulse_magnitude(), 0.3, TOL, "friction at the Coulomb bound")
	check_near(f.ball.linear_velocity.x, 2.7, TOL, "friction changes the physical tangential velocity")
	check(f.ball.angular_velocity.length() > 0.1, "and spins the body (physical angular velocity)")
	# Hinge motor: target 2 rad/s against an anchor error (split channel active): the physical rate is the same as without it.
	var clean = hinge_pair()
	var dirty = hinge_pair()
	for h in [clean, dirty]:
		h.joint.motor_enabled = true
		h.joint.motor_target_speed = 2.0
		h.joint.motor_max_torque = 1000.0
	dirty.b.position += Vector3(0.03, -0.02, 0.0)
	clean.world.step()
	dirty.world.step()
	check_near(clean.joint.angle_rate(), 2.0, 2.0e-3, "motor reaches its target rate (%s)" % clean.joint.angle_rate())
	check_near(dirty.joint.angle_rate(), clean.joint.angle_rate(), 2.0e-3, "unchanged by a simultaneous positional correction (%s)" % dirty.joint.angle_rate())


func test_a7_split_pass_changes_only_split_velocities() -> void:
	# Constraint-level view: after the physical pass the physical velocities are fixed; the split pass must not touch them.
	var s = ground_ball(Vector3.ZERO, 0.48, Vector3(0.5, -1.0, 0.0), 0.2)
	var c = s.contact
	var cfg = s.world.config
	c.prepare(DT, cfg)
	for i in cfg.solver_iterations:
		c.solve_iteration()
	var v_phys: Vector3 = s.ball.linear_velocity
	var w_phys: Vector3 = s.ball.angular_velocity
	var lambda_n: float = c.normal_impulse()
	var lambda_t: float = c.friction_impulse_magnitude()
	check(s.ball.split_linear_velocity == Vector3.ZERO, "split velocity untouched by the physical pass")
	for i in cfg.solver_iterations:
		c.solve_split_iteration()
	check(s.ball.linear_velocity == v_phys, "split pass leaves the physical linear velocity bit-identical")
	check(s.ball.angular_velocity == w_phys, "split pass leaves the physical angular velocity bit-identical")
	check(c.normal_impulse() == lambda_n and c.friction_impulse_magnitude() == lambda_t, "and the physical accumulated impulses")
	check_near(s.ball.split_linear_velocity.y, 0.2 * 0.02 / DT, TOL, "split velocity = beta * |g| / dt along the normal")
	check_near(c.rows[ContactConstraint.ROW_NORMAL].split_accumulated_impulse, 0.2 * 0.02 / DT, TOL, "split impulse = m * beta * |g| / dt")
	# Diagnostics report physical state only.
	check_near(s.ball.kinetic_energy(), 0.5 * s.ball.mass * v_phys.length_squared() + 0.5 * w_phys.dot(s.ball.inertia_world * w_phys), 1.0e-9, "kinetic energy ignores the split velocity")
	check_vec3_near(s.ball.velocity_at_point(Vector3(0, 0.0, 0)), v_phys + w_phys.cross(Vector3(0, 0.0, 0) - s.ball.position), 1.0e-9, "point velocity ignores the split velocity")
	check_vec3_near(s.world.total_linear_momentum(), v_phys * s.ball.mass, 1.0e-9, "momentum ignores the split velocity")


# =====================================================================================================================
# B. Contact behavior
# =====================================================================================================================

func test_b1_penetration_shrinks_geometrically_through_the_split_correction() -> void:
	var s = ground_ball(Vector3.ZERO, 0.48, Vector3.ZERO, 0.0)
	var gap: float = -0.02
	for i in 12:
		s.world.step()
		gap *= 0.8
		check_near(s.contact.gap(), gap, 1.0e-6, "step %d: gap follows (1 - beta)^n * g0" % (i + 1))
	# Deeper penetration under gravity with a descending ball: never grows beyond the initial penetration, ends in contact.
	var d = ground_ball(Vector3(0, -G, 0), 0.45, Vector3(0, -3.0, 0), 0.0)
	var deepest: float = 0.0
	for i in 360:
		d.world.step()
		deepest = minf(deepest, d.contact.gap())
	check(deepest >= -0.05 - 1.0e-6, "penetration never exceeds the initial 5 cm (deepest %s)" % deepest)
	check_near(d.contact.gap(), 0.0, 1.0e-5, "converges to zero gap")
	check_near(d.contact.normal_impulse(), G * DT, TOL, "carrying the weight")


func test_b2_speculative_contact_still_prevents_crossing_through_physical_impulses() -> void:
	# Gap 5 cm, closing at 20 m/s: 0.33 m per step would cross. The PHYSICAL speculative row brakes to gap/dt.
	var s = ground_ball(Vector3.ZERO, 0.55, Vector3(0, -20.0, 0), 0.0)
	var row = s.contact.rows[ContactConstraint.ROW_NORMAL]
	s.world.step()
	check(s.contact.normal_impulse() > 0.0, "physical impulse engaged")
	check_near(s.ball.linear_velocity.y, -0.05 / DT, 1.0e-3, "braked to gap/dt (physical velocity)")
	check(s.contact.gap() > -1.0e-6, "no crossing (gap %s)" % s.contact.gap())
	check(not row.split_enabled and row.split_accumulated_impulse == 0.0, "split channel inactive for a positive gap")
	# Next step: now touching, stopped.
	s.world.step()
	check_near(s.ball.linear_velocity.y, 0.0, 1.0e-3, "stopped at the surface")
	check(s.contact.gap() > -1.0e-6, "still no penetration")
	# Slow approach: inactive, no braking, no split impulse either.
	var t = ground_ball(Vector3.ZERO, 1.5, Vector3(0, -2.0, 0), 0.0)
	t.world.step()
	check(t.contact.normal_impulse() == 0.0, "gap 1 m: contact inactive")
	check_near(t.ball.linear_velocity.y, -2.0, 1.0e-6, "no early braking")
	check(t.ball.position.y == 1.5 - 2.0 * DT or absf(t.ball.position.y - (1.5 - 2.0 * DT)) < 1.0e-6, "pose is pure physical motion")


func test_b3_separating_penetrating_contact_gets_no_attractive_physical_impulse() -> void:
	# 1 cm deep, separating at 0.1 m/s, below the old Baumgarte push-out speed beta * 0.01 / dt = 0.24 m/s.
	var s = ground_ball(Vector3.ZERO, 0.49, Vector3(0, 0.1, 0), 0.0)
	s.world.step()
	check(s.contact.normal_impulse() == 0.0, "no physical normal impulse")
	check_near(s.ball.linear_velocity.y, 0.1, 1.0e-7, "physical velocity exactly the separating velocity (old Baumgarte raised it to 0.24)")
	check_near(s.ball.position.y, 0.49 + 0.1 * DT + 0.2 * 0.01, 1.0e-6, "pose = physical motion + split correction")
	# Fast separation: also untouched.
	var f = ground_ball(Vector3.ZERO, 0.49, Vector3(0, 5.0, 0), 0.0)
	f.world.step()
	check(f.contact.normal_impulse() == 0.0 and absf(f.ball.linear_velocity.y - 5.0) < 1.0e-7, "fast separation untouched")


func test_b4_friction_uses_the_physical_normal_impulse() -> void:
	# 2 cm deep, approaching at 2 m/s, sliding at 5 m/s, mu = 0.5, m = 1.
	# Physical normal impulse 2 (stops the approach); split normal impulse m*beta*|g|/dt = 0.48, a different number.
	# Friction limit mu * 2 = 1 < the stopping impulse 5/3.5: friction saturates at exactly mu * physical lambda_n = 1.
	var s = ground_ball(Vector3.ZERO, 0.48, Vector3(5, -2, 0), 0.5)
	var c = s.contact
	solve_only(s.world)  # the real solver: prepare, physical pass, split pass
	var split_impulse: float = c.rows[ContactConstraint.ROW_NORMAL].split_accumulated_impulse
	check_near(c.normal_impulse(), 2.0, TOL, "physical normal impulse")
	check_near(split_impulse, 0.48, TOL, "split normal impulse is a different quantity")
	check_near(c.friction_limit(), 0.5 * c.normal_impulse(), 1.0e-9, "limit is mu * physical normal impulse")
	check_near(c.friction_impulse_magnitude(), 1.0, TOL, "friction saturates at mu * physical lambda_n, not mu * (physical + split) = 1.24")
	check(c.friction_impulse_magnitude() <= c.friction_limit() + 1.0e-6, "inside the Coulomb disk")
	# The physical channel does not depend on the split channel at all: with beta = 0 (no split impulse) friction is identical.
	var z = ground_ball(Vector3.ZERO, 0.48, Vector3(5, -2, 0), 0.5)
	z.world.config.baumgarte_factor = 0.0
	solve_only(z.world)
	check(z.contact.rows[ContactConstraint.ROW_NORMAL].split_accumulated_impulse == 0.0, "beta = 0: no split impulse")
	check(z.contact.friction_impulse_magnitude() == c.friction_impulse_magnitude() and z.ball.linear_velocity == s.ball.linear_velocity, "friction and physical velocity independent of the split channel")
	# Through the world as well, over many steps (friction stays inside the disk while the contact is also being pushed out).
	var w = ground_ball(Vector3(0, -G, 0), 0.47, Vector3(4, -1, 2), 0.4, 2.0)
	var worst: float = -INF
	for i in 120:
		w.world.step()
		worst = maxf(worst, w.contact.friction_impulse_magnitude() - w.contact.friction_limit())
	check(worst <= 1.0e-6, "|lambda_t| <= mu * lambda_n at every step (worst excess %s)" % worst)


func test_b5_frictionless_contact() -> void:
	var s = ground_ball(Vector3.ZERO, 0.48, Vector3(3, -1, 4), 0.0)
	s.world.step()
	check(s.contact.friction_impulse_magnitude() == 0.0, "no friction impulse at mu = 0")
	check_vec3_near(s.ball.linear_velocity, Vector3(3, 0, 4), TOL, "tangential velocity untouched, approach stopped")
	check_vec3_near(s.ball.angular_velocity, Vector3.ZERO, TOL, "no spin")
	check_near(s.contact.normal_impulse(), 1.0, TOL, "normal impulse m * v")
	# The physical impulse removes the 1 m/s approach, so the pose moves only by the split correction beta * |g|.
	check_near(s.ball.position.y, 0.48 + 0.2 * 0.02, 1.0e-6, "penetration still corrected")


func test_b6_static_bodies_are_exactly_unchanged() -> void:
	var s = ground_ball(Vector3(0, -G, 0), 0.4, Vector3(4, -6, 2), 0.7, 5.0)
	var ground = s.ground
	var position_before: Vector3 = ground.position
	var orientation_before: Quaternion = ground.orientation
	for i in 240:
		s.contact.local_anchor_a = Vector3(s.ball.position.x, 0.0, s.ball.position.z)
		s.contact.local_anchor_b = s.ball.orientation.inverse() * Vector3(0, -s.radius, 0)
		s.world.step()
	check(ground.position == position_before and ground.orientation == orientation_before, "static pose exactly unchanged")
	check(ground.linear_velocity == Vector3.ZERO and ground.angular_velocity == Vector3.ZERO, "static physical velocity exactly zero")
	check(ground.split_linear_velocity == Vector3.ZERO and ground.split_angular_velocity == Vector3.ZERO, "static split velocity exactly zero")
	# Mid-step too: the split pass on a static body never writes anything.
	var t = ground_ball(Vector3.ZERO, 0.45, Vector3.ZERO, 0.5)
	solve_only(t.world)
	check(t.ground.split_linear_velocity == Vector3.ZERO and t.ground.split_angular_velocity == Vector3.ZERO, "static split velocity zero mid-step")
	check(t.ball.split_linear_velocity.y > 0.0, "while the dynamic body does receive the correction")


# =====================================================================================================================
# C. Joint behavior
# =====================================================================================================================

func test_c1_ball_joint_error_decreases_monotonically() -> void:
	var s = ball_pendulum(Vector3(0.05, 0.03, -0.04), Vector3.ZERO, false)  # dynamic-dynamic
	var previous: float = s.joint.linear_error().length()
	var first: float = previous
	for i in 40:
		s.world.step()
		var e: float = s.joint.linear_error().length()
		check(e < previous, "step %d: error decreases (%s -> %s)" % [i, previous, e])
		previous = e
	check(previous < 0.01 * first, "after 40 steps the error is below 1 %% of the start (%s)" % previous)
	# Static pivot with a swinging pendulum under gravity: the error stays small (Baumgarte steady state, as before).
	var p = ball_pendulum(Vector3.ZERO, Vector3(0, -G, 0))
	var peak: float = 0.0
	for i in 480:
		p.world.step()
		peak = maxf(peak, p.joint.linear_error().length())
	check(peak < 5.0e-3, "pendulum anchor error bounded (peak %s)" % peak)


func test_c2_hinge_anchor_error_decreases() -> void:
	var s = hinge_pair()
	s.b.position += Vector3(0.04, -0.03, 0.02)
	var previous: float = s.joint.linear_error().length()
	for i in 30:
		s.world.step()
		var e: float = s.joint.linear_error().length()
		check(e < previous, "step %d: anchor error decreases (%s -> %s)" % [i, previous, e])
		previous = e
	check(previous < 0.01, "and ends small (%s)" % previous)


func test_c3_hinge_axis_error_decreases() -> void:
	var s = hinge_pair()
	rotate_b_about(s, Vector3.RIGHT, 0.1)
	var previous: float = s.joint.axis_error_angle()
	check_near(previous, 0.1, 1.0e-5, "initial axis error")
	for i in 30:
		s.world.step()
		var e: float = s.joint.axis_error_angle()
		check(e < previous, "step %d: axis error decreases (%s -> %s)" % [i, previous, e])
		previous = e
	check(previous < 0.01, "and ends small (%s)" % previous)


func test_c4_violated_limits_are_corrected_in_the_right_direction() -> void:
	for spec in [[-0.7, 1.0], [0.8, -1.0]]:  # [initial angle, expected sign of the correction]
		var s = hinge_pair()
		s.joint.limit_enabled = true
		s.joint.lower_limit = -0.5
		s.joint.upper_limit = 0.6
		rotate_b_about(s, s.axis, spec[0])
		var previous: float = s.joint.angle()
		for i in 5:
			s.world.step()
			var a: float = s.joint.angle()
			check((a - previous) * spec[1] > 0.0, "angle %s: step %d moves toward the allowed range (%s -> %s)" % [spec[0], i, previous, a])
			previous = a
	# Inside the range nothing is applied.
	var inside = hinge_pair()
	inside.joint.limit_enabled = true
	inside.joint.lower_limit = -0.5
	inside.joint.upper_limit = 0.6
	rotate_b_about(inside, inside.axis, 0.2)
	solve_only(inside.world)
	check(inside.b.split_angular_velocity == Vector3.ZERO and inside.b.split_linear_velocity == Vector3.ZERO, "no split velocity for a satisfied limit")
	check(not inside.joint.rows[HingeJoint.ROW_LOWER].split_enabled and not inside.joint.rows[HingeJoint.ROW_UPPER].split_enabled, "split rows inactive inside the limits")


func test_c5_hinge_speculative_limit_stays_physical() -> void:
	# B approaches the lower limit at 3 rad/s from 0.01 rad away: the physical speculative row must stop it before it crosses.
	var s = hinge_pair()
	s.joint.limit_enabled = true
	s.joint.lower_limit = -0.5
	s.joint.upper_limit = 0.6
	rotate_b_about(s, s.axis, -0.49)
	s.b.angular_velocity = Vector3(0, 0, -3.0)
	s.b.linear_velocity = s.b.angular_velocity.cross(s.b.position - s.anchor)
	var lowest: float = s.joint.angle()
	for i in 20:
		s.world.step()
		lowest = minf(lowest, s.joint.angle())
	check(lowest >= -0.5 - 5.0e-4, "never crosses the limit (lowest %s)" % lowest)
	check(s.joint.lower_limit_impulse() >= 0.0, "physical limit impulse engaged without going negative")
	var s2 = hinge_pair()
	s2.joint.limit_enabled = true
	s2.joint.lower_limit = -0.5
	s2.joint.upper_limit = 0.6
	rotate_b_about(s2, s2.axis, -0.49)
	s2.b.angular_velocity = Vector3(0, 0, -3.0)
	s2.b.linear_velocity = s2.b.angular_velocity.cross(s2.b.position - s2.anchor)
	solve_only(s2.world)
	check(not s2.joint.rows[HingeJoint.ROW_LOWER].split_enabled, "split channel inactive while the limit is not violated")
	check(s2.joint.lower_limit_impulse() > 0.0, "physical limit impulse engaged by the speculative bias")


func test_c6_motor_produces_the_expected_physical_rate_and_hinge_rotates_freely() -> void:
	var s = hinge_pair()
	s.joint.motor_enabled = true
	s.joint.motor_target_speed = 1.5
	s.joint.motor_max_torque = 1000.0
	s.world.step()
	check_near(s.joint.angle_rate(), 1.5, 2.0e-3, "motor target reached after one step (%s)" % s.joint.angle_rate())
	# The pose follows the PHYSICAL rate only: no positional-correction velocity is added by a motor row (angle = rate * dt, up to
	# the O(theta^3) sine error of the 0.0125 rad step and the 2e-3 rate tolerance above).
	check_near(s.joint.angle(), s.joint.angle_rate() * DT, 5.0e-5, "motor row adds no split displacement (angle %s)" % s.joint.angle())
	run_steps(s.world, 4)
	check_near(s.joint.angle_rate(), 1.5, 2.0e-3, "motor target held (%s)" % s.joint.angle_rate())
	# Free rotation: no motor, spinning at 1 rad/s keeps its rate except for the documented frozen-Jacobian loss (w dt)^2/2 per step.
	var f = hinge_pair()
	f.b.angular_velocity = Vector3(0, 0, 1.0)
	f.b.linear_velocity = f.b.angular_velocity.cross(f.b.position - f.anchor)
	run_steps(f.world, 12)
	check_near(f.joint.angle_rate(), 1.0, 1.0 * 12 * 0.5 * (DT * DT) + 1.0e-4, "free rotation stays free (%s)" % f.joint.angle_rate())
	# An axial torque accelerates B as an unconstrained pivot: alpha = tau / I_pivot.
	var t = hinge_pair()
	var d: Vector3 = t.b.position - t.anchor
	var i_pivot: float = t.axis.dot(t.b.inertia_world * t.axis) + t.b.mass * (d.length_squared() - d.dot(t.axis) * d.dot(t.axis))
	for i in 30:
		t.b.apply_torque(t.axis * 0.4)
		t.world.step()
	check_near(t.joint.angle_rate(), 0.4 * 30 * DT / i_pivot, 0.01 * 0.4 * 30 * DT / i_pivot, "axial torque: alpha = tau / I_pivot")


# =====================================================================================================================
# D. State lifecycle and regression
# =====================================================================================================================

func test_d1_split_velocities_start_at_zero_and_never_leak() -> void:
	# A stale split velocity on a body (as a leak would leave it) must not move the pose: SimWorld zeroes it before the solve.
	var w = make_world(Vector3.ZERO)
	var lone = make_sphere(1.0, 0.5, Vector3(1, 2, 3))
	w.add_body(lone)
	lone.split_linear_velocity = Vector3(5, 5, 5)
	lone.split_angular_velocity = Vector3(1, 1, 1)
	w.step()
	check(lone.position == Vector3(1, 2, 3) and lone.orientation == Quaternion.IDENTITY, "stale split velocity did not move the body")
	check(lone.split_linear_velocity == Vector3.ZERO and lone.split_angular_velocity == Vector3.ZERO, "zero after the step")
	# Through a constrained body: the solver's split pass starts from zero too.
	var s = ball_pendulum(Vector3(0.03, -0.02, 0.01))
	var ref = ball_pendulum(Vector3(0.03, -0.02, 0.01))
	s.bob.split_linear_velocity = Vector3(9, 9, 9)
	solve_only(s.world)
	solve_only(ref.world)
	check(s.bob.split_linear_velocity == ref.bob.split_linear_velocity, "stale split velocity discarded by the solver's split pass")
	# Every step ends with zero split velocity.
	var c = ground_ball(Vector3(0, -G, 0), 0.4, Vector3(1, -2, 0), 0.5)
	for i in 100:
		c.world.step()
		check(c.ball.split_linear_velocity == Vector3.ZERO and c.ball.split_angular_velocity == Vector3.ZERO, "step %d: split velocity zero after the step" % i)
		if not c.ball.split_linear_velocity == Vector3.ZERO:
			break
	# Nothing carried to the next step: correct a penetration, then put the ball exactly in contact and step again.
	var l = ground_ball(Vector3.ZERO, 0.48, Vector3.ZERO, 0.0)
	l.world.step()
	check(l.ball.position.y > 0.48, "first step corrected the pose")
	l.ball.position = Vector3(0, 0.5, 0)
	var before: Vector3 = l.ball.position
	l.world.step()
	check(l.ball.position == before, "second step: no residual motion from the first step's correction (%s)" % l.ball.position)
	check(l.ball.linear_velocity == Vector3.ZERO, "and no persistent velocity")
	# Split impulses are not warm started: the accumulated split impulse is rebuilt from zero every step.
	var m = ground_ball(Vector3.ZERO, 0.48, Vector3.ZERO, 0.0)
	var row = m.contact.rows[ContactConstraint.ROW_NORMAL]
	m.world.step()
	var first_impulse: float = row.split_accumulated_impulse
	m.ball.position = Vector3(0, 0.48, 0)  # same penetration again
	m.world.step()
	check_near(row.split_accumulated_impulse, first_impulse, 1.0e-6, "same error -> same split impulse (not accumulated across steps)")


func test_d2_multiple_constraints_remain_stable_and_inject_no_energy() -> void:
	# Horizontal chain, no gravity: static pivot -ball joint- sphere b1 -hinge (axis Z)- box b2 whose far-bottom edge penetrates a
	# floor by 2 cm. b1 is displaced by (0.03, 0.02, 0.01) so BOTH joints start with an anchor error; the floor penetration is
	# removable by rotating b2 about the hinge, so all three errors can vanish together. All velocities are zero, so every
	# physical constraint velocity is zero and the physical channel adds nothing: kinetic energy must stay exactly zero while
	# every error shrinks. (The old velocity-level Baumgarte bias produces kinetic energy in the first step.)
	var w = make_world(Vector3.ZERO)
	w.config.solver_iterations = ITERATIONS
	var pivot = make_static(Vector3.ZERO)
	var b1 = make_sphere(1.0, 0.3, Vector3(1.0, 0.0, 0.0))
	var b2 = make_body(1.0, SimMassProperties.box_inertia(1.0, Vector3(1.0, 0.4, 0.4)), Vector3(2.0, 0.0, 0.0))
	var floor_body = make_static()
	for b in [pivot, b1, b2, floor_body]:
		w.add_body(b)
	var ball = BallJoint.new(pivot, b1, Vector3.ZERO, Vector3(-1, 0, 0))
	var hinge = HingeJoint.from_world(b1, b2, Vector3(1.5, 0, 0), Vector3.BACK, Vector3.UP)
	# Floor plane y = -0.18 (anchor on the static body), contact point on b2 at its far-bottom edge (2.5, -0.2, 0): gap -0.02.
	var contact = ContactConstraint.new(floor_body, b2, Vector3(2.5, -0.18, 0), Vector3(0.5, -0.2, 0), Vector3.UP, 0.4)
	b1.position += Vector3(0.03, 0.02, 0.01)  # both joints now have an error
	for c in [ball, hinge, contact]:
		check(w.add_constraint(c) != null, "constraint accepted")
	var e0: Array = [ball.linear_error().length(), hinge.linear_error().length(), contact.gap()]
	check(e0[0] > 0.03 and e0[1] > 0.03 and e0[2] < -0.019, "all three constraints start with an error (%s)" % [e0])
	var max_ke: float = 0.0
	for i in 300:
		w.step()
		max_ke = maxf(max_ke, w.total_kinetic_energy())
	check(max_ke == 0.0, "no kinetic energy injected (max %s)" % max_ke)
	check(ball.linear_error().length() < 0.01 * e0[0], "ball joint error shrinks (%s -> %s)" % [e0[0], ball.linear_error().length()])
	check(hinge.linear_error().length() < 0.01 * e0[1], "hinge anchor error shrinks (%s -> %s)" % [e0[1], hinge.linear_error().length()])
	check(contact.gap() > -1.0e-3, "contact penetration removed (%s)" % contact.gap())
	check(hinge.axis_error_angle() < 1.0e-3, "hinge axis stays aligned (%s)" % hinge.axis_error_angle())
	for b in [b1, b2]:
		check(is_finite(b.position.x) and is_finite(b.position.y) and is_finite(b.position.z), "state finite")


func test_d3_dynamic_dynamic_and_static_dynamic_configurations() -> void:
	# Dynamic-dynamic contact: the two split impulses are equal and opposite, so the center of mass of the pair does not move
	# and the relative gap closes by beta per step. Masses 2 and 1, 2 cm deep, at rest.
	var w = make_world(Vector3.ZERO)
	var a = make_sphere(2.0, 0.4, Vector3(0, 0, 0))
	var b = make_sphere(1.0, 0.3, Vector3(0, 0.68, 0))  # gap = 0.68 - 0.4 - 0.3 = -0.02
	w.add_body(a)
	w.add_body(b)
	var c = ContactConstraint.from_world(a, b, Vector3(0, 0.4, 0), Vector3(0, 0.38, 0), Vector3.UP, 0.3)
	w.add_constraint(c)
	var com0: Vector3 = (a.position * 2.0 + b.position * 1.0) / 3.0
	var gap0: float = c.gap()
	w.step()
	check_near(gap0, -0.02, 1.0e-6, "initial gap")
	check_near(c.gap(), gap0 * 0.8, ERR_TOL, "gap closes by beta")
	check_vec3_near((a.position * 2.0 + b.position * 1.0) / 3.0, com0, 1.0e-6, "center of mass unmoved by the split impulses")
	check(a.linear_velocity == Vector3.ZERO and b.linear_velocity == Vector3.ZERO, "no physical velocity")
	check_near((b.position.y - 0.68) * 1.0, -(a.position.y - 0.0) * 2.0, 1.0e-6, "displacements inversely proportional to mass (a moves down by beta*0.02/3)")
	# Dynamic-dynamic ball joint: center of mass unmoved as well.
	var p = ball_pendulum(Vector3(0.03, -0.02, 0.01), Vector3.ZERO, false)
	var com1: Vector3 = (p.pivot.position + p.bob.position) * 0.5
	p.world.step()
	check_vec3_near((p.pivot.position + p.bob.position) * 0.5, com1, 1.0e-6, "ball joint: center of mass unmoved")
	check(p.pivot.linear_velocity == Vector3.ZERO and p.bob.linear_velocity == Vector3.ZERO, "ball joint: no physical velocity")
	# Dynamic-dynamic hinge.
	var h = hinge_pair(false)
	h.b.position += Vector3(0.03, -0.02, 0.01)
	var e0: float = h.joint.linear_error().length()
	h.world.step()
	check(h.joint.linear_error().length() < 0.9 * e0, "dynamic-dynamic hinge: anchor error decreases")
	check(h.a.linear_velocity == Vector3.ZERO and h.b.linear_velocity == Vector3.ZERO, "dynamic-dynamic hinge: no physical velocity")


func test_d4_disabled_constraints_generate_no_split_impulses() -> void:
	var s = ground_ball(Vector3.ZERO, 0.48, Vector3.ZERO, 0.0)
	s.contact.enabled = false
	var p = ball_pendulum(Vector3(0.03, -0.02, 0.01))
	p.joint.enabled = false
	var h = hinge_pair()
	h.b.position += Vector3(0.03, 0.0, 0.0)
	h.joint.enabled = false
	for world in [s.world, p.world, h.world]:
		solve_only(world)
	check(s.ball.split_linear_velocity == Vector3.ZERO, "disabled contact: no split velocity")
	check(p.bob.split_linear_velocity == Vector3.ZERO and p.bob.split_angular_velocity == Vector3.ZERO, "disabled ball joint: no split velocity")
	check(h.b.split_linear_velocity == Vector3.ZERO and h.b.split_angular_velocity == Vector3.ZERO, "disabled hinge: no split velocity")
	var before: Vector3 = s.ball.position
	s.world.step()
	check(s.ball.position == before, "disabled contact: pose exactly unchanged")
	for r in s.contact.rows:
		check(r.split_accumulated_impulse == 0.0, "disabled contact: split impulse zero")


func test_d5_zero_error_constraints_generate_no_split_impulses() -> void:
	# A satisfied ball joint holding a hanging weight, a contact with a positive gap, a hinge inside its limits.
	var p = ball_pendulum(Vector3.ZERO, Vector3(0, -G, 0))
	# ball_pendulum(Vector3.ZERO): bob at (1, 0, 0), anchor (-1, 0, 0) local -> the anchors coincide exactly in float32.
	solve_only(p.world)
	check(p.joint.linear_error().length() == 0.0, "satisfied ball joint has exactly zero error")
	for r in p.joint.rows:
		check(r.split_accumulated_impulse == 0.0, "ball joint row: no split impulse")
	check(p.bob.split_linear_velocity == Vector3.ZERO and p.bob.split_angular_velocity == Vector3.ZERO, "ball joint: no split velocity")
	var c = ground_ball(Vector3(0, -G, 0), 0.6, Vector3.ZERO, 0.5)
	solve_only(c.world)
	check(c.ball.split_linear_velocity == Vector3.ZERO, "positive-gap contact: no split velocity")
	check(not c.contact.rows[ContactConstraint.ROW_NORMAL].split_enabled, "positive-gap contact: split row inactive")
	var h = hinge_pair()
	h.joint.limit_enabled = true
	h.joint.lower_limit = -0.5
	h.joint.upper_limit = 0.6
	solve_only(h.world)
	check(h.b.split_linear_velocity == Vector3.ZERO and h.b.split_angular_velocity == Vector3.ZERO, "hinge inside limits: no split velocity")


func test_d6_invalid_constraints_are_still_rejected() -> void:
	var w = make_world(Vector3.ZERO)
	var ground = make_static()
	var ball = make_sphere(1.0, 0.5, Vector3(0, 0.5, 0))
	w.add_body(ground)
	w.add_body(ball)
	var bad = ContactConstraint.new(ground, ball, Vector3.ZERO, Vector3.ZERO, Vector3(0, 2, 0), 0.5)  # non-unit normal
	check(w.add_constraint(bad) == null, "invalid contact refused at registration")
	var good = ContactConstraint.from_world(ground, ball, Vector3.ZERO, Vector3(0, 0.0, 0), Vector3.UP, 0.5)
	check(w.add_constraint(good) != null, "valid contact accepted")
	good.friction = -1.0
	var position_before: Vector3 = ball.position
	check(not w.step(), "preflight refuses the step after the data became invalid")
	check(ball.position == position_before and ball.split_linear_velocity == Vector3.ZERO, "refused step changes nothing")
	good.friction = 0.5
	check(w.step(), "valid again: stepping resumes")
	# World membership.
	var other = make_sphere(1.0, 0.5, Vector3(5, 0, 0))
	check(w.add_constraint(BallJoint.new(ground, other, Vector3.ZERO, Vector3.ZERO)) == null, "constraint with a body outside the world refused")


func test_d7_stabilization_is_deterministic() -> void:
	var results: Array = []
	for run in 2:
		var s = hinge_pair(false, Vector3(0, -G, 0))
		s.joint.limit_enabled = true
		s.joint.lower_limit = -0.5
		s.joint.upper_limit = 0.6
		s.b.position += Vector3(0.02, 0.01, 0.0)
		s.b.angular_velocity = Vector3(0.3, -0.2, 2.0)
		for i in 120:
			s.world.step()
		results.append( [s.b.position, s.b.orientation, s.b.linear_velocity, s.b.angular_velocity])
	check(results[0][0] == results[1][0] and results[0][1] == results[1][1], "identical inputs give bit-identical poses")
	check(results[0][2] == results[1][2] and results[0][3] == results[1][3], "and bit-identical velocities")


func test_d8_split_displacement_is_the_only_angular_momentum_change_of_the_correction() -> void:
	# Split displacement moves x without changing p, so sum(x x p) changes by sum(dx x p) (and a rotation changes I w for a
	# non-spherical body; spheres here). Physical impulses conserve it (PHYSICS_CONVENTIONS.md). Verify the prediction with a
	# contact correction that is the only thing acting: ball moving sideways while 2 cm deep, no gravity, mu = 0.
	var s = ground_ball(Vector3.ZERO, 0.48, Vector3(1.0, 0.0, 0.0), 0.0)
	var l0: Vector3 = s.world.total_angular_momentum()
	s.world.step()
	var dx: Vector3 = Vector3(0, 0.2 * 0.02, 0)  # split displacement of the ball
	var predicted: Vector3 = l0 + dx.cross(s.ball.linear_momentum())
	check_vec3_near(s.world.total_angular_momentum(), predicted, 1.0e-6, "dL = dx_split x p")
	check(absf(s.world.total_angular_momentum().z - l0.z) > 1.0e-3, "and it is not vacuous (%s)" % (s.world.total_angular_momentum().z - l0.z))
	# With beta = 0 the same scene conserves L exactly.
	var u = ground_ball(Vector3.ZERO, 0.48, Vector3(1.0, 0.0, 0.0), 0.0)
	u.world.config.baumgarte_factor = 0.0
	var m0: Vector3 = u.world.total_angular_momentum()
	u.world.step()
	check_vec3_near(u.world.total_angular_momentum(), m0, 1.0e-6, "beta = 0: angular momentum conserved by the velocity solve")


func test_d9_beta_is_the_only_stabilization_parameter() -> void:
	# beta = 1 removes the whole error in one step, beta = 0 none; no other tuning value exists (SimConfig has no new parameter).
	var s = ball_pendulum(Vector3(0.03, 0.0, 0.0))
	s.world.config.baumgarte_factor = 1.0
	s.world.step()
	check(s.joint.linear_error().length() < 1.0e-5, "beta = 1: error removed in one step (%s)" % s.joint.linear_error().length())
	var z = ball_pendulum(Vector3(0.03, 0.0, 0.0))
	z.world.config.baumgarte_factor = 0.0
	z.world.step()
	check_near(z.joint.linear_error().length(), 0.03, 1.0e-6, "beta = 0: error untouched")
	var props: Array = []
	for p in z.world.config.get_property_list():
		if p.name.contains("split") or p.name.contains("stabil"):
			props.append(p.name)
	check(props.is_empty(), "no extra stabilization parameter in SimConfig (%s)" % [props])


func test_d10_split_angular_correction_changes_orientation_dependent_quantities_but_not_physical_velocity() -> void:
	# M1.3.1. The split channel never writes physical velocity, but the split ANGULAR velocity rotates the body, and for an
	# anisotropic body a different orientation means a different world inertia I_w = R I R^T. With the physical omega unchanged,
	# L_spin = I_w omega and T_rot = 1/2 omega . I_w omega therefore change. This is the expected consequence of pose correction,
	# not a leak of split velocity into physical velocity (PHYSICS_CONVENTIONS.md, "Stabilization: split impulses").
	#
	# Scene: tilted_hinge_spinner(). Box 0.2 x 0.3 x 0.8 (principal moments 0.1217, 0.1133, 0.0217), spinning at 3 rad/s about its
	# own hinge axis (principal axis Z, the smallest moment), tilted 0.4 rad about X, COM 1 cm off the anchor. A twin world with
	# beta = 0 (no split channel) is the control: same physical channel, no pose correction.
	var beta: float = 0.2
	var tilt: float = 0.4
	var spin: float = 3.0
	var anchor_error: Vector3 = Vector3(0.01, -0.005, 0.008)
	var inertia: Basis = SimMassProperties.box_inertia(2.0, Vector3(0.2, 0.3, 0.8))
	var s = tilted_hinge_spinner(beta, inertia, tilt, anchor_error, spin)
	var ctl = tilted_hinge_spinner(0.0, inertia, tilt, anchor_error, spin)
	var body = s.b
	var twin = ctl.b
	var w_before: Vector3 = body.angular_velocity
	var v_before: Vector3 = body.linear_velocity
	var q_before: Quaternion = body.orientation
	var i_before: Basis = body.inertia_world
	var l_before: Vector3 = body.angular_momentum_about_com()
	var t_before: float = body.kinetic_energy()
	check(inertia.x.x != inertia.y.y and inertia.y.y != inertia.z.z and inertia.x.x != inertia.z.z, "the body is anisotropic (three distinct principal moments)")

	# 1. The split pass really produces an angular correction velocity, and it stays out of the physical channel. Solve only (no
	# integration) on a scratch copy so the mid-step state can be read.
	var probe = tilted_hinge_spinner(beta, inertia, tilt, anchor_error, spin)
	solve_only(probe.world)
	check(probe.b.split_angular_velocity.length() > 5.0, "the split pass produced an angular correction velocity (%s rad/s)" % probe.b.split_angular_velocity.length())
	check(probe.b.split_linear_velocity.length() > 0.1, "and a linear one from the anchor error (%s m/s)" % probe.b.split_linear_velocity.length())
	check_vec3_near(probe.b.angular_velocity, w_before, TOL, "solve: physical angular velocity untouched by the split pass")
	check_vec3_near(probe.b.linear_velocity, v_before, TOL, "solve: physical linear velocity untouched by the split pass")

	# 2. The real pipeline: one World.step() (physical pass, split pass, position integration). No pose is edited by hand.
	s.world.step()
	ctl.world.step()

	# Physical velocities: unchanged from before the step and identical to the control that has no split channel. Tolerances: the
	# physical rows see (b x p).omega with omega parallel to b, i.e. float32 rounding of O(1e-7); TOL = 1e-5 covers it and is far
	# below the 10 rad/s (1 m/s scale) a leak of the split velocity would produce.
	check_vec3_near(body.angular_velocity, w_before, TOL, "physical angular velocity unchanged by the split correction")
	check_vec3_near(body.linear_velocity, v_before, TOL, "physical linear velocity unchanged by the split correction")
	check_vec3_near(body.angular_velocity, twin.angular_velocity, TOL, "physical angular velocity equals the beta = 0 control")
	check_vec3_near(body.linear_velocity, twin.linear_velocity, TOL, "physical linear velocity equals the beta = 0 control")
	check(body.split_angular_velocity == Vector3.ZERO and body.split_linear_velocity == Vector3.ZERO, "split velocities are scratch: zero after the step")
	# The linear correction did happen (so the linear check above is not vacuous): the pose moved by beta * anchor error.
	check_vec3_near(body.position - twin.position, -beta * anchor_error, 1.0e-6, "linear pose correction removed beta * anchor error without velocity")

	# Orientation: changed by the split angular correction. Relative rotation to the control isolates it from the physical spin about
	# the hinge axis. One row-exact step rotates by beta * tan(tilt) (the axis row's Jacobian has magnitude cos(tilt)); second-order
	# terms of the composed rotation vector are below 1e-4.
	var relative: Quaternion = body.orientation * twin.orientation.inverse()
	check_near(relative.get_angle(), beta * tan(tilt), 1.0e-4, "split correction rotated the body by beta * tan(tilt)")
	check(relative.get_angle() > 0.05, "the orientation change is substantial (%s rad)" % relative.get_angle())
	# The physical spin turns the body about its own hinge axis, so only the split rotation can move that axis in the world frame.
	var axis_before: Vector3 = q_before * Vector3.BACK
	check_near((body.orientation * Vector3.BACK).angle_to(axis_before), beta * tan(tilt), 1.0e-4, "the body's hinge axis was turned by the split correction")
	check_near((twin.orientation * Vector3.BACK).angle_to(axis_before), 0.0, 1.0e-4, "control: the hinge axis stays where it was (only the physical spin about it)")

	# World-space inertia reflects the corrected orientation: recomputed here from the quaternion, not read from the body's cache.
	var r_now: Basis = Basis(body.orientation)
	var i_now: Basis = r_now * inertia * r_now.transposed()
	check_basis_near(body.inertia_world, i_now, 1.0e-6, "inertia_world = R I R^T of the corrected orientation")
	check((body.inertia_world.x - i_before.x).length() + (body.inertia_world.y - i_before.y).length() + (body.inertia_world.z - i_before.z).length() > 0.01,
		"the world inertia tensor changed (it is orientation dependent for an anisotropic body)")

	# Spin angular momentum and rotational kinetic energy: computed from the UPDATED inertia and the UNCHANGED physical omega.
	var w_now: Vector3 = body.angular_velocity
	var l_now: Vector3 = body.angular_momentum_about_com()
	var t_now: float = body.kinetic_energy()
	check_vec3_near(l_now, i_now * w_now, 1.0e-6, "L_spin = I_w(new orientation) * omega")
	check_near(t_now, 0.5 * w_now.dot(i_now * w_now), 1.0e-6, "T_rot = 1/2 omega . I_w(new orientation) omega")
	# Independent principal-axis form of the same energy: T = 1/2 sum I_k (omega . e_k)^2 with e_k the body axes in the world frame.
	var t_principal: float = 0.0
	var principal: Array = [inertia.x.x, inertia.y.y, inertia.z.z]
	var axes: Array = [body.orientation * Vector3.RIGHT, body.orientation * Vector3.UP, body.orientation * Vector3.BACK]
	for k in 3:
		t_principal += 0.5 * principal[k] * pow(w_now.dot(axes[k]), 2.0)
	check_near(t_now, t_principal, 1.0e-6, "T_rot agrees with the principal-axis form")

	# The effect: both quantities changed although omega did not. The control, which has the same omega but no pose correction,
	# keeps its initial values (omega is along a principal axis, so the physical spin about it leaves I_w omega unchanged).
	check_vec3_near(twin.angular_momentum_about_com(), l_before, 1.0e-5, "control (no pose correction): L_spin unchanged")
	check_near(twin.kinetic_energy(), t_before, 1.0e-5, "control (no pose correction): T_rot unchanged")
	var dl: float = (l_now - l_before).length()
	var dt_rel: float = (t_now - t_before) / t_before
	# Measured for this configuration: |dL| = 0.0232 (|L| = 0.065), dT/T = +3.0 %. Thresholds are a factor ~2-3 below, far above float32 noise.
	check(dl > 0.01, "L_spin changed because the orientation changed (|dL| = %s, |L| = %s)" % [dl, l_before.length()])
	check(dt_rel > 0.01, "T_rot changed because the orientation changed (dT/T = %s)" % dt_rel)
	# Direction of the effect in THIS configuration (not a general rule): omega started on the smallest-moment axis and the correction
	# turned the body so that omega now has a component along larger-moment axes, hence T_rot = 1/2 omega . I_w omega rose.
	check(t_now > t_before, "T_rot rose at fixed omega (smallest-moment spin turned away from its spin axis)")

	# Counter-check, same scene with an isotropic body: I_w is orientation independent, so the same pose correction changes
	# neither L_spin nor T_rot. The change above is therefore a consequence of anisotropy, not of the correction injecting energy.
	var ball_inertia: Basis = SimMassProperties.sphere_inertia(2.0, 0.4)
	var iso = tilted_hinge_spinner(beta, ball_inertia, tilt, anchor_error, spin)
	var l_iso: Vector3 = iso.b.angular_momentum_about_com()
	var t_iso: float = iso.b.kinetic_energy()
	var iso_axis_before: Vector3 = iso.b.orientation * Vector3.BACK
	iso.world.step()
	check((iso.b.orientation * Vector3.BACK).angle_to(iso_axis_before) > 0.05, "isotropic body: the pose was corrected by the same split rotation")
	check_vec3_near(iso.b.angular_velocity, l_iso / ball_inertia.x.x, TOL, "isotropic body: physical angular velocity unchanged")
	check_vec3_near(iso.b.angular_momentum_about_com(), l_iso, 1.0e-5, "isotropic body: L_spin unchanged by the pose correction")
	check_near(iso.b.kinetic_energy(), t_iso, 1.0e-5, "isotropic body: T_rot unchanged by the pose correction")
