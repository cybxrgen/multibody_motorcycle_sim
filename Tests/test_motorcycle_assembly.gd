extends "res://Tests/sim_test_case.gd"
## M2.1 rigid motorcycle assembly (graph updated for M2.2: the fork is now two bodies, steering assembly and fork slider, joined by
## the generic prismatic joint; this file still tests the hinge structure and the rear assembly, the front fork/suspension tests
## are in test_prismatic_joint.gd, test_front_suspension.gd and test_motorcycle_front_suspension.gd). Six independent dynamic rigid
## bodies, four generic hinges and one prismatic joint (Simulation/Motorcycle/). Fully dynamic world, zero gravity unless a test
## says otherwise; no body is static. The front suspension spring-damper is part of every assembly built here (stiffness 12 kN/m,
## damping 400 N s/m, prototype values): it only loads the fork's axial direction, which the hinge probes below do not excite
## except through the shared bodies; where a test needs the hinge structure in isolation the spring is not involved because the
## probes act on the other joints or on the wheels.
##
## A construction and validity      B kinematic structure at the reference pose
## C degrees of freedom (probes)    D coupled dynamics and conservation
## E free fall / determinism        F regression sensitivity (missing hinge, wrongly configured axis)
##
## Methodology notes (numbers are from measurements of this assembly; the mechanism is stated where one is known):
## * Solver: 200 iterations (100 before M2.2, the value the hinge tests use for converged velocity rows). Residuals of this five-body chain were
##   measured to converge with the iteration count (worst velocity-row residual 0.045 rad/s at 20, 0.007 at 50, 2.3e-4 at 100).
## * Velocity-row residuals are evaluated with the frames the solver linearized (start of the step, i.e. `previous_state` after
##   a step) because the rows are frozen per step (PROJECT_STATE.md). Measured against the END-of-step axis instead, a wheel
##   spinning at 6.5 rad/s relative to a carrier that turned 0.021 rad in the step shows a fake 0.14 rad/s "violation"
##   (= 6.5 * 0.021); that is the measurement frame, not a constraint defect.
## * Position-level errors (anchor, axis) are NOT iteration limited: each step leaves a second-order frozen-Jacobian error
##   ~ 1/2 (w_rel dt)(w_perp dt) and the split correction removes a fraction beta per step, so the steady state is e_step / beta.
##   Measured worst case 2.6e-3 rad (relative spin 6.5 rad/s, carrier 2.5 rad/s: 1/2 (6.5/120)(2.5/120)/0.2 = 2.8e-3).
## * float32: Vector3/Basis/Quaternion carry ~1.2e-7 relative error; pose-construction tolerances are therefore a few 1e-7 per
##   metre, never 1e-9 (PHYSICS_CONVENTIONS.md, "Numerical precision"). Constraint-solver ERROR output from deliberately invalid
##   input in sections A is expected (TESTING.md).

const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const MotorcycleAssembly = preload("res://Simulation/Motorcycle/motorcycle_assembly.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")

const DT: float = 1.0 / 120.0
## 200 since M2.2 (was 100): with the fork split into steering assembly, prismatic joint and slider the front chain is longer and
## scalar Gauss-Seidel converges more slowly. Measured pivot-velocity residual of the front axle after a 12/-9/15 N s impulse on the
## front wheel: 5.9e-2 m/s at 20 iterations, 4.9e-3 at 100, 6.5e-4 at 200, 5.4e-4 at 400 and 1000 (the floor is the end-of-step
## measurement-frame artifact described above). 100 iterations left the unchanged 2e-3 threshold violated; no tolerance was changed.
const ITERATIONS: int = 200
## Pose-construction tolerances (float32, coordinates <= ~1 m): 4 roundings of ~1.2e-7 each, doubled for margin.
const POSE_TOL: float = 2.0e-6
## Names for the four hinges in the fixed order used by Assembly.joints() / MotorcycleConfig.joint_specs().
const JOINT_NAMES: Array = ["steering_head", "swingarm_pivot", "front_axle", "rear_axle"]


# --- World double that refuses the n-th add_body / add_constraint call (failure injection for the builder's rollback) ---
class FailingWorld extends "res://Simulation/Core/sim_world.gd":
	var fail_body_call: int = -1
	var fail_constraint_call: int = -1
	var body_calls: int = 0
	var constraint_calls: int = 0

	func add_body(body):
		body_calls += 1
		if body_calls == fail_body_call:
			return null
		return super(body)

	func add_constraint(constraint):
		constraint_calls += 1
		if constraint_calls == fail_constraint_call:
			return null
		return super(constraint)


# --- Builders and helpers ---

func _config() -> MotorcycleConfig:
	return MotorcyclePrototype.config()


## Zero-gravity world, converged solver, prototype assembly. Null (after recording a failure) if construction fails.
func _build(gravity: Vector3 = Vector3.ZERO, iterations: int = ITERATIONS) -> MotorcycleAssembly:
	var w: SimWorld = make_world(gravity)
	w.config.solver_iterations = iterations
	var a: MotorcycleAssembly = MotorcycleBuilder.build(_config(), w)
	check(a != null, "the prototype assembly builds")
	if a == null:
		return null
	for body in a.bodies():
		if body == null:
			check(false, "every named body exists")
			return null
	for joint in a.joints():
		if joint == null:
			check(false, "every named joint exists")
			return null
	return a


func _pair(a: MotorcycleAssembly, k: int) -> Array:
	var j = a.joints()[k]
	return [j.body_a, j.body_b]


## Bodies on the A side and on the B side of joint k when that joint is cut (the assembly is a tree).
func _sides(a: MotorcycleAssembly, k: int) -> Array:
	match k:
		0:
			return [[a.chassis, a.swingarm, a.rear_wheel], [a.steering, a.fork_slider, a.front_wheel]]
		1:
			return [[a.chassis, a.steering, a.fork_slider, a.front_wheel], [a.swingarm, a.rear_wheel]]
		2:
			return [[a.chassis, a.swingarm, a.rear_wheel, a.steering, a.fork_slider], [a.front_wheel]]
	return [[a.chassis, a.steering, a.fork_slider, a.front_wheel, a.swingarm], [a.rear_wheel]]


## Angular momentum of `bodies` about `point`, projected on `axis`: sum (x - p) x (m v) + I_world w.
func _h_axis(bodies: Array, point: Vector3, axis: Vector3) -> float:
	var h: Vector3 = Vector3.ZERO
	for b in bodies:
		h += (b.position - point).cross(b.linear_momentum()) + b.angular_momentum_about_com()
	return h.dot(axis)


## Velocity-row residual of a hinge, evaluated at the frames the solver linearized (start of the last step):
## x = |relative anchor velocity|, y = |component of the relative angular velocity perpendicular to B's axis|.
func _residual(j) -> Vector2:
	var qa: Quaternion = j.body_a.previous_state.orientation
	var qb: Quaternion = j.body_b.previous_state.orientation
	var ra: Vector3 = qa * j.local_anchor_a
	var rb: Vector3 = qb * j.local_anchor_b
	var cdot: Vector3 = (j.body_b.linear_velocity + j.body_b.angular_velocity.cross(rb)) - (j.body_a.linear_velocity + j.body_a.angular_velocity.cross(ra))
	var b: Vector3 = qb * j.local_axis_b
	var dw: Vector3 = j.body_b.angular_velocity - j.body_a.angular_velocity
	return Vector2(cdot.length(), (dw - b * b.dot(dw)).length())


## Rotate body B kinematically by `angle` about the world axis through `point` (pose edit, for kinematic tests).
func _rotate_about(body: SimRigidBody, point: Vector3, axis: Vector3, angle: float) -> void:
	var dq: Quaternion = Quaternion(axis, angle)
	body.position = point + dq * (body.position - point)
	body.orientation = dq * body.orientation


func _body_finite(b: SimRigidBody) -> bool:
	return b.position.is_finite() and b.orientation.is_finite() and b.linear_velocity.is_finite() and b.angular_velocity.is_finite() \
		and b.inertia_world.x.is_finite() and b.inertia_world.y.is_finite() and b.inertia_world.z.is_finite()


# =====================================================================================================================
# A. Construction and validity
# =====================================================================================================================

func test_a1_builds_six_dynamic_bodies_four_hinges_and_the_fork_joint() -> void:
	var w: SimWorld = make_world()
	var a: MotorcycleAssembly = MotorcycleBuilder.build(_config(), w)
	check(a != null, "build succeeds")
	if a == null:
		return
	check(a.world == w, "the assembly uses the world it was given")
	check(w.bodies.size() == 6, "exactly six bodies registered (got %d)" % w.bodies.size())
	check(w.constraints.size() == 5, "exactly five constraints registered: four hinges and the fork's prismatic joint (got %d)" % w.constraints.size())
	for body in a.bodies():
		check(w.has_body(body), "body '%s' is a member of the world" % body.body_name)
		check(not body.is_static(), "body '%s' is dynamic" % body.body_name)
		check(body.mass_properties.valid, "body '%s' has valid mass properties" % body.body_name)
	for joint in a.joints():
		if joint == null:
			check(false, "every named joint exists")
			continue
		check(w.constraints.has(joint), "a named joint is registered in the world")
		check(joint.get_script() == HingeJoint, "every joint is the generic M1 HingeJointConstraint")
		check(w.has_body(joint.body_a) and w.has_body(joint.body_b), "a joint references two bodies of the same world")
		check(joint.body_a != joint.body_b, "a joint connects two different bodies")
	# A world that already has content keeps it and gains exactly 5 bodies and 4 joints.
	var w2: SimWorld = make_world()
	var existing: SimRigidBody = w2.add_body(make_static())
	var a2: MotorcycleAssembly = MotorcycleBuilder.build(_config(), w2)
	check(a2 != null and w2.bodies.size() == 7 and w2.constraints.size() == 5, "building into a populated world adds exactly 6 bodies, 4 hinges and 1 prismatic joint")
	check(w2.bodies[0] == existing, "pre-existing content keeps its place")
	# Without a world argument the builder creates its own.
	var a3: MotorcycleAssembly = MotorcycleBuilder.build(_config())
	check(a3 != null and a3.world != null and a3.world.bodies.size() == 6, "a world is created when none is passed")


func test_a2_named_references_identify_the_correct_objects() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	var expected_names: Array = ["chassis", "steering", "fork_slider", "swingarm", "front_wheel", "rear_wheel"]
	var bodies: Array = a.bodies()
	for i in 6:
		check(bodies[i].body_name == expected_names[i], "bodies()[%d] is '%s' (got '%s')" % [i, expected_names[i], bodies[i].body_name])
	check(a.world.bodies[0] == a.chassis and a.world.bodies[1] == a.steering and a.world.bodies[2] == a.fork_slider \
		and a.world.bodies[3] == a.swingarm and a.world.bodies[4] == a.front_wheel and a.world.bodies[5] == a.rear_wheel, "world order equals the named order")
	var ids: Dictionary = {}
	for b in bodies:
		ids[b.id] = true
	check(ids.size() == 6, "six distinct body ids")
	check(a.steering_joint.body_a == a.chassis and a.steering_joint.body_b == a.steering, "steering joint: chassis <-> steering")
	check(a.swingarm_joint.body_a == a.chassis and a.swingarm_joint.body_b == a.swingarm, "swingarm joint: chassis <-> swingarm")
	check(a.front_axle_joint.body_a == a.fork_slider and a.front_axle_joint.body_b == a.front_wheel, "front axle joint: fork slider <-> front wheel (not the upper steering assembly)")
	check(a.rear_axle_joint.body_a == a.swingarm and a.rear_axle_joint.body_b == a.rear_wheel, "rear axle joint: swingarm <-> rear wheel")
	var joints: Array = a.joints()
	check(joints[0] == a.steering_joint and joints[1] == a.swingarm_joint and joints[2] == a.front_axle_joint and joints[3] == a.rear_axle_joint, "joints() order")
	check(a.world.constraints.size() == 5, "the world holds exactly the four named hinges and the prismatic fork joint")
	check(a.world.constraints[4] == a.fork_joint and a.fork_joint.body_a == a.steering and a.fork_joint.body_b == a.fork_slider, "constraint 4 is the fork joint: steering <-> fork slider")
	for i in mini(4, a.world.constraints.size()):
		check(a.world.constraints[i] == joints[i], "constraint %d in the world is the named %s joint" % [i, JOINT_NAMES[i]])


func test_a3_mass_properties_are_explicit_and_independent() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	var specs: Array = a.config.body_specs()
	var bodies: Array = a.bodies()
	var sum: float = 0.0
	for i in 6:
		check_near(bodies[i].mass, specs[i].mass, 0.0, "body %d mass equals its configured mass exactly" % i)
		check_basis_near(bodies[i].local_inertia, specs[i].local_inertia, 0.0, "body %d inertia equals its configured tensor" % i)
		check_vec3_near(bodies[i].position, specs[i].position, 0.0, "body %d COM equals its configured COM" % i)
		sum += specs[i].mass
	check_near(a.total_mass(), sum, 1.0e-9, "total_mass() is the sum of the six masses")
	# Nothing is merged into the chassis: no body carries the total, and the wheels/carriers have their own mass.
	check(a.chassis.mass < a.total_mass() - 1.0, "the chassis does not carry the whole mass (%s of %s kg)" % [a.chassis.mass, a.total_mass()])
	check(a.front_wheel.mass != a.rear_wheel.mass and a.steering.mass != a.swingarm.mass and a.fork_slider.mass != a.steering.mass, "masses are independent per body")


func test_a4_every_inertia_is_valid_symmetric_positive_definite_and_well_conditioned() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	for body in a.bodies():
		var label: String = body.body_name
		check(SimMassProperties.validate_inertia(body.local_inertia) == "", "%s: local inertia passes validate_inertia (finite, symmetric, positive definite, rcond >= 1e-5)" % label)
		check(SimMassProperties.validate_inertia(body.inertia_world) == "", "%s: world inertia passes validate_inertia" % label)
		var i: Basis = body.local_inertia
		check_basis_near(i, i.transposed(), 0.0, "%s: local inertia is exactly symmetric" % label)
		check(i.x.x > 0.0 and i.y.y > 0.0 and i.z.z > 0.0, "%s: positive principal moments" % label)
		# The prototype tensors are diagonal (principal frame): also a physically realizable body (triangle inequality).
		check(i.x.y == 0.0 and i.x.z == 0.0 and i.y.z == 0.0, "%s: prototype inertia is diagonal" % label)
		check(i.x.x + i.y.y >= i.z.z and i.y.y + i.z.z >= i.x.x and i.x.x + i.z.z >= i.y.y, "%s: principal moments satisfy the triangle inequality" % label)
		check_basis_near(body.inverse_local_inertia * i, Basis.IDENTITY, 1.0e-5, "%s: I^-1 I = 1 (float32, 1e-5)" % label)
		var r: Basis = Basis(body.orientation)
		var scale: float = maxf(i.x.x, maxf(i.y.y, i.z.z))
		check_basis_near(body.inertia_world, r * i * r.transposed(), 1.0e-6 * scale, "%s: cached world inertia equals R I R^T" % label)


func test_a5_primitive_inertias_match_their_documented_formulas() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	# Box primitives (independent of SimMassProperties.box_inertia: m/12 (b^2 + c^2)).
	var ch: Basis = a.chassis.local_inertia
	check_near(ch.x.x, 120.0 / 12.0 * (0.55 * 0.55 + 1.0 * 1.0), 1.0e-4, "chassis I_xx of the 0.30 x 0.55 x 1.00 m box")
	check_near(ch.y.y, 120.0 / 12.0 * (0.30 * 0.30 + 1.0 * 1.0), 1.0e-4, "chassis I_yy")
	check_near(ch.z.z, 120.0 / 12.0 * (0.30 * 0.30 + 0.55 * 0.55), 1.0e-4, "chassis I_zz")
	# Fork frame: local +Y is the steering axis, so the moment about the steering axis is the box's I_yy (the sides perpendicular to it).
	var steer_axis: Vector3 = a.config.steering_head.axis
	check_near(steer_axis.dot(a.steering.inertia_world * steer_axis), 14.0 / 12.0 * (0.60 * 0.60 + 0.16 * 0.16), 1.0e-4, "steering assembly: moment about the steering axis = box I_yy (frame rotated by the rake)")
	check_near(steer_axis.dot(a.fork_slider.inertia_world * steer_axis), 8.0 / 12.0 * (0.24 * 0.24 + 0.14 * 0.14), 1.0e-4, "fork slider: moment about the fork axis = box I_yy (frame rotated by the rake)")
	check_vec3_near(Basis(a.fork_slider.orientation) * Vector3.UP, steer_axis, POSE_TOL, "slider-local +Y maps onto the steering axis")
	check_vec3_near(Basis(a.steering.orientation) * Vector3.UP, steer_axis, POSE_TOL, "fork-local +Y maps onto the steering axis")
	# Swingarm frame: local +Z along the arm (pivot -> axle); moment about the arm direction is the box I_zz (0.20 x 0.10).
	var arm_dir: Vector3 = (a.config.rear_axle.point - a.config.swingarm_pivot.point).normalized()
	check_vec3_near(Basis(a.swingarm.orientation) * Vector3.BACK, arm_dir, POSE_TOL, "swingarm-local +Z maps onto the pivot-to-axle direction")
	check_near(arm_dir.dot(a.swingarm.inertia_world * arm_dir), 9.0 / 12.0 * (0.20 * 0.20 + 0.10 * 0.10), 1.0e-4, "swingarm: moment about the arm = box I_zz")
	# Wheels: hollow cylinder I_axle = m/2 (ro^2 + ri^2); the axle is a principal axis and the moment about the WORLD axle
	# direction equals it.
	var wheels: Array = [[a.front_wheel, a.config.front_wheel_radius, a.config.front_axle], [a.rear_wheel, a.config.rear_wheel_radius, a.config.rear_axle]]
	for w in wheels:
		var body: SimRigidBody = w[0]
		var ro: float = w[1]
		var ri: float = MotorcyclePrototype.WHEEL_INNER_FRACTION * ro
		var axle: Vector3 = w[2].axis
		var expected_axial: float = 0.5 * body.mass * (ro * ro + ri * ri)
		check_near(axle.dot(body.inertia_world * axle), expected_axial, 1.0e-5, "%s: moment about the world axle = m/2 (ro^2 + ri^2)" % body.body_name)
		var along: Vector3 = body.inertia_world * axle
		check_vec3_near(along, axle * expected_axial, 1.0e-5, "%s: the axle is an eigenvector of the world inertia (principal axis)" % body.body_name)
	# Limiting cases of the helper itself.
	var disc: Basis = MotorcycleConfig.wheel_inertia(2.0, 0.5, 0.0, 0.1)
	check_near(disc.x.x, 0.5 * 2.0 * 0.25, 1.0e-6, "solid disc: I_axle = m r^2 / 2")
	check_near(disc.y.y, 2.0 / 12.0 * (3.0 * 0.25 + 0.01), 1.0e-6, "solid cylinder: I_transverse = m (3 r^2 + w^2) / 12")
	var ring: Basis = MotorcycleConfig.wheel_inertia(2.0, 0.5, 0.5, 0.0)
	check_near(ring.x.x, 2.0 * 0.25, 1.0e-6, "thin ring: I_axle = m r^2")
	check_near(ring.y.y, 0.5 * 2.0 * 0.25, 1.0e-6, "thin ring: I_diameter = m r^2 / 2")


## Invalid configurations: (label, mutation of a fresh prototype config, substring that the validation message must contain).
func _invalid_cases() -> Array:
	return [
		["zero mass (would be static)", func(c): c.steering.mass = 0.0, "mass"],
		["negative mass", func(c): c.swingarm.mass = -3.0, "mass"],
		["NaN mass", func(c): c.chassis.mass = NAN, "mass"],
		["infinite mass", func(c): c.rear_wheel.mass = INF, "mass"],
		["asymmetric inertia", func(c): c.chassis.local_inertia = Basis(Vector3(10, 1, 0), Vector3(0, 10, 0), Vector3(0, 0, 10)), "symmetric"],
		["negative principal moment", func(c): c.swingarm.local_inertia = Basis.from_scale(Vector3(1.0, -1.0, 1.0)), "inertia"],
		["ill-conditioned inertia", func(c): c.front_wheel.local_inertia = Basis.from_scale(Vector3(1.0, 1.0, 1.0e-7)), "condition"],
		["NaN inertia", func(c): c.steering.local_inertia = Basis(Vector3(NAN, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)), "inertia"],
		["NaN position", func(c): c.rear_wheel.position = Vector3(0, NAN, 0), "position"],
		["non-unit orientation", func(c): c.steering.orientation = Quaternion(0, 0, 0, 2.0), "orientation"],
		["NaN orientation", func(c): c.chassis.orientation = Quaternion(NAN, 0, 0, 1), "orientation"],
		["zero steering axis", func(c): c.steering_head.axis = Vector3.ZERO, "axis"],
		["non-unit swingarm axis", func(c): c.swingarm_pivot.axis = Vector3(2, 0, 0), "unit"],
		["NaN axle axis", func(c): c.front_axle.axis = Vector3(NAN, 0, 0), "axis"],
		["infinite pivot", func(c): c.rear_axle.point = Vector3(INF, 0, 0), "point"],
		["wheel axle not a principal axis of the wheel inertia", func(c): c.front_wheel.orientation = Quaternion(Vector3.BACK, 0.5), "principal"],
		["duplicate body name", func(c): c.rear_wheel.body_name = "front_wheel", "duplicate"],
		["empty body name", func(c): c.chassis.body_name = "", "name"],
		["non-positive wheel radius", func(c): c.rear_wheel_radius = 0.0, "radius"],
		["zero box proxy size", func(c): c.swingarm.visual_size = Vector3(0.2, 0.0, 0.5), "visual_size"],
	]


func test_a6_validation_reports_every_invalid_rule_and_accepts_the_prototype() -> void:
	check(_config().validate().is_empty(), "the prototype configuration is valid")
	check(_config().is_valid(), "is_valid() agrees")
	for case in _invalid_cases():
		var c: MotorcycleConfig = _config()
		case[1].call(c)
		var errors: PackedStringArray = c.validate()
		check(not errors.is_empty(), "'%s' is reported" % case[0])
		check(" ".join(errors).containsn(case[2]), "'%s': a message mentions '%s' (got %s)" % [case[0], case[2], errors])
		check(not c.is_valid(), "'%s': is_valid() is false" % case[0])


func test_a7_invalid_configuration_is_rejected_without_partial_registration() -> void:
	# A populated world: the builder must neither add to it nor disturb it on failure.
	var w: SimWorld = make_world()
	var anchor: SimRigidBody = w.add_body(make_static())
	var ball: SimRigidBody = w.add_body(make_sphere(1.0, 0.1, Vector3(1, 0, 0)))
	var joint: SimConstraint = w.add_constraint(BallJoint.new(anchor, ball, Vector3.ZERO, Vector3(-1, 0, 0)))
	var bodies_before: Array = w.bodies.duplicate()
	var constraints_before: Array = w.constraints.duplicate()
	for case in _invalid_cases():
		var c: MotorcycleConfig = _config()
		case[1].call(c)
		var a: MotorcycleAssembly = MotorcycleBuilder.build(c, w)
		check(a == null, "'%s': build returns null" % case[0])
		check(w.bodies == bodies_before and w.constraints == constraints_before, "'%s': the world is exactly as before" % case[0])
	check(MotorcycleBuilder.build(null, w) == null, "a null configuration is rejected")
	check(w.bodies == bodies_before and w.constraints == constraints_before, "null configuration: world unchanged")
	check(w.constraints[0] == joint and w.preflight(), "the pre-existing world still passes preflight")
	# The same world accepts a valid assembly afterwards.
	var ok: MotorcycleAssembly = MotorcycleBuilder.build(_config(), w)
	check(ok != null and w.bodies.size() == 8 and w.constraints.size() == 6, "after rejected attempts a valid build still succeeds")


func test_a8_registration_failure_rolls_back_every_partial_registration() -> void:
	# Failure of the n-th add_body (first, middle, last) and the n-th add_constraint (first .. last): nothing may remain.
	var failures: Array = []
	for n in range(1, 7):
		failures.append([n, -1, "add_body #%d" % n])
	for n in range(1, 6):
		failures.append([-1, n, "add_constraint #%d" % n])
	for f in failures:
		var w: FailingWorld = FailingWorld.new()
		w.config.gravity = Vector3.ZERO
		var existing: SimRigidBody = w.add_body(make_static())
		w.body_calls = 0
		var bodies_before: Array = w.bodies.duplicate()
		w.fail_body_call = f[0]
		w.fail_constraint_call = f[1]
		var cfg: MotorcycleConfig = _config()
		var mass_before: float = cfg.chassis.mass
		var a: MotorcycleAssembly = MotorcycleBuilder.build(cfg, w)
		check(a == null, "%s refused: build returns null" % f[2])
		check(w.bodies == bodies_before, "%s refused: bodies rolled back (%d left)" % [f[2], w.bodies.size()])
		check(w.constraints.is_empty() and w.force_providers.is_empty(), "%s refused: no constraint or force provider left (%d, %d)" % [f[2], w.constraints.size(), w.force_providers.size()])
		check(w.has_body(existing) and cfg.chassis.mass == mass_before, "%s refused: pre-existing body and the configuration are untouched" % f[2])
		# No stale state: the same world builds a complete assembly once the injected failure is lifted.
		w.fail_body_call = -1
		w.fail_constraint_call = -1
		var retry: MotorcycleAssembly = MotorcycleBuilder.build(_config(), w)
		check(retry != null and w.bodies.size() == 7 and w.constraints.size() == 5 and w.force_providers.size() == 1, "%s refused: a retry on the same world succeeds completely" % f[2])
		if retry != null:
			check(w.step(), "%s refused: the retried assembly steps" % f[2])


func test_a9_construction_postcondition_detects_displacement_and_misalignment() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	check(MotorcycleBuilder.construction_error(a) == "", "a freshly built assembly satisfies the construction contract")
	# Displace the fork by 1 mm: the steering-head anchors no longer coincide.
	a.steering.position += Vector3(0.001, 0.0, 0.0)
	check(MotorcycleBuilder.construction_error(a).containsn("anchors"), "a 1 mm anchor separation is reported")
	a.steering.position -= Vector3(0.001, 0.0, 0.0)
	check(MotorcycleBuilder.construction_error(a) == "", "restoring the pose restores the contract")
	# Tilt the rear wheel (a leaf: no hinge hangs off it) by 0.001 rad about the vertical through its axle point: the anchors
	# still coincide but the axle axes are no longer aligned.
	_rotate_about(a.rear_wheel, a.config.rear_axle.point, Vector3.UP, 0.001)
	check(MotorcycleBuilder.construction_error(a).containsn("axes"), "a 1 mrad axis misalignment is reported")


# =====================================================================================================================
# B. Kinematic structure at the reference pose
# =====================================================================================================================

func test_b1_hinge_anchors_coincide_at_the_configured_pivots() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	var specs: Array = a.config.joint_specs()
	var joints: Array = a.joints()
	for k in 4:
		var j = joints[k]
		check_vec3_near(j.world_anchor_a(), specs[k].point, POSE_TOL, "%s: anchor on body A is the configured pivot" % JOINT_NAMES[k])
		check_vec3_near(j.world_anchor_b(), specs[k].point, POSE_TOL, "%s: anchor on body B is the configured pivot" % JOINT_NAMES[k])
		check(j.linear_error().length() <= POSE_TOL, "%s: anchors coincide (%s m)" % [JOINT_NAMES[k], j.linear_error().length()])
	check(a.max_anchor_error() <= POSE_TOL, "max_anchor_error() <= %s (got %s)" % [POSE_TOL, a.max_anchor_error()])
	# Each wheel's COM is on its axle, so a wheel has no lever arm about its own hinge.
	check_vec3_near(a.front_wheel.position, a.config.front_axle.point, POSE_TOL, "front wheel COM on the front axle")
	check_vec3_near(a.rear_wheel.position, a.config.rear_axle.point, POSE_TOL, "rear wheel COM on the rear axle")


func test_b2_hinge_axes_are_aligned_and_equal_the_configured_axes() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	var specs: Array = a.config.joint_specs()
	var joints: Array = a.joints()
	for k in 4:
		var j = joints[k]
		check_vec3_near(j.world_axis_a(), specs[k].axis, POSE_TOL, "%s: world axis seen from body A" % JOINT_NAMES[k])
		check_vec3_near(j.world_axis_b(), specs[k].axis, POSE_TOL, "%s: world axis seen from body B" % JOINT_NAMES[k])
		check(j.axis_error_angle() <= POSE_TOL, "%s: axes aligned (%s rad)" % [JOINT_NAMES[k], j.axis_error_angle()])
	check(a.max_axis_error() <= POSE_TOL, "max_axis_error() <= %s" % POSE_TOL)
	# The bodies do NOT share local orientations: the steering axis is the rake-tilted vector in the chassis frame but
	# +Y in the fork frame; the builder works from world data, not from equal local vectors.
	var s: HingeJoint = a.steering_joint
	var local_angle: float = atan2(s.local_axis_a.cross(s.local_axis_b).length(), s.local_axis_a.dot(s.local_axis_b))
	check_near(local_angle, deg_to_rad(MotorcyclePrototype.RAKE_DEGREES), 1.0e-6, "steering: local axes differ by the rake angle")
	check_vec3_near(s.local_axis_b, Vector3.UP, POSE_TOL, "steering: the axis is +Y in the fork frame")


func test_b3_zero_angle_and_per_body_reference_directions() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	var specs: Array = a.config.joint_specs()
	var joints: Array = a.joints()
	for k in 4:
		var j = joints[k]
		var ref_a: Vector3 = j.body_a.orientation * j.local_reference_a
		var ref_b: Vector3 = j.body_b.orientation * j.local_reference_b
		check_vec3_near(ref_a, ref_b, POSE_TOL, "%s: world reference directions coincide (zero angle)" % JOINT_NAMES[k])
		check_vec3_near(ref_a, MotorcycleBuilder.hinge_reference(specs[k].axis), POSE_TOL, "%s: reference is the documented deterministic direction" % JOINT_NAMES[k])
		check(absf(ref_a.dot(specs[k].axis)) <= POSE_TOL, "%s: reference is perpendicular to the axis" % JOINT_NAMES[k])
		check(absf(j.angle()) <= POSE_TOL, "%s: initial hinge angle is zero (%s)" % [JOINT_NAMES[k], j.angle()])
		# Different local frames: the SAME world reference is two different body-local vectors. The steering hinge is the one
		# exception: its reference is world +X and the rake is a rotation ABOUT X, so both locals are +X (its local AXES differ, B2).
		var differ: bool = (j.local_reference_a - j.local_reference_b).length() > 0.1
		check(differ == (k != 0), "%s: local reference vectors %s between the bodies" % [JOINT_NAMES[k], "differ" if k != 0 else "coincide"])


func test_b4_hinge_reference_is_unit_perpendicular_and_never_singular() -> void:
	var axes: Array = [Vector3.RIGHT, Vector3.UP, Vector3.BACK, -Vector3.RIGHT, Vector3(1, 1, 1).normalized(), Vector3(0, 0.898794, 0.438371).normalized(),
		Vector3(1, 0, 1).normalized(), Vector3(0.001, 1, 0.001).normalized()]
	for axis in axes:
		var r: Vector3 = MotorcycleBuilder.hinge_reference(axis)
		check_near(r.length(), 1.0, 1.0e-6, "reference of %s is a unit vector" % axis)
		check(absf(r.dot(axis)) <= 1.0e-6, "reference of %s is perpendicular to it" % axis)
		check(MotorcycleBuilder.hinge_reference(axis) == r, "deterministic for %s" % axis)


func test_b5_every_hinge_is_a_free_hinge_without_limits_or_motor() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	for k in 4:
		var j = a.joints()[k]
		check(j.rows.size() == HingeJoint.ROW_COUNT, "%s: the generic 8-row hinge" % JOINT_NAMES[k])
		check(j.enabled, "%s: enabled" % JOINT_NAMES[k])
		check(not j.limit_enabled and not j.motor_enabled and j.motor_max_torque == 0.0, "%s: no hidden limit, motor or torque" % JOINT_NAMES[k])
		check(j.is_valid(), "%s: valid constraint data" % JOINT_NAMES[k])


func test_b6_each_hinge_allows_rotation_about_its_axis_without_error() -> void:
	for k in 4:
		var a: MotorcycleAssembly = _build()
		if a == null:
			return
		var j = a.joints()[k]
		var spec = a.config.joint_specs()[k]
		_rotate_about(j.body_b, spec.point, spec.axis, 0.7)
		check_near(j.angle(), 0.7, 2.0e-6, "%s: rotating B by +0.7 rad about the axis reads +0.7" % JOINT_NAMES[k])
		check(j.linear_error().length() <= 2.0 * POSE_TOL, "%s: anchors still coincide after the rotation (%s)" % [JOINT_NAMES[k], j.linear_error().length()])
		check(j.axis_error_angle() <= 2.0 * POSE_TOL, "%s: axes still aligned after the rotation (%s)" % [JOINT_NAMES[k], j.axis_error_angle()])
		_rotate_about(j.body_b, spec.point, spec.axis, -1.9)
		check_near(j.angle(), -1.2, 4.0e-6, "%s: a further -1.9 rad reads -1.2 (continuous, signed)" % JOINT_NAMES[k])
		check(j.linear_error().length() <= 4.0 * POSE_TOL and j.axis_error_angle() <= 4.0 * POSE_TOL, "%s: still no constraint error" % JOINT_NAMES[k])


func test_b7_error_measures_respond_to_the_prohibited_relative_motions() -> void:
	# Negative control for B6: the same measures DO react to translation and to rotation about a perpendicular direction.
	for k in 4:
		var a: MotorcycleAssembly = _build()
		if a == null:
			return
		var j = a.joints()[k]
		var spec = a.config.joint_specs()[k]
		var perp: Vector3 = MotorcycleBuilder.hinge_reference(spec.axis)
		_rotate_about(j.body_b, spec.point, perp, 0.05)
		check_near(j.axis_error_angle(), 0.05, 1.0e-5, "%s: tilting B by 0.05 rad about a perpendicular reads 0.05 rad of axis error" % JOINT_NAMES[k])
		check(j.linear_error().length() <= 2.0 * POSE_TOL, "%s: the pivot did not move" % JOINT_NAMES[k])
		var b: MotorcycleAssembly = _build()
		var jb = b.joints()[k]
		jb.body_b.position += Vector3(0.0, 0.01, 0.0)
		check_near(jb.linear_error().length(), 0.01, 1.0e-6, "%s: translating B by 10 mm reads 10 mm of anchor error" % JOINT_NAMES[k])


# =====================================================================================================================
# C. Degrees of freedom, probed with impulses (one fixed step from rest each)
# =====================================================================================================================
# For hinge k with bodies A, B, the CONFIGURED geometry (config pivot and axis, never the constraint objects) defines:
#   AXIAL   angular impulse L about the axis on B: B-side angular momentum about the pivot along the axis must equal L and the
#           A side must carry 0 (a free axis transmits no axial torque); B must rotate relative to A about the axis.
#   PERP1/2 angular impulse L about each perpendicular direction on B: the relative angular velocity perpendicular to the axis
#           must stay ~0 (the two perpendicular rotations are constrained).
#   LINEAR  linear impulse J at B's COM: the relative velocity of the two bodies at the pivot must stay ~0.
# Measured on the correct assembly (100 iterations): |H_B.a - L| and |H_A.a| <= 1e-6, perpendicular residual <= 1.1e-3 rad/s (swingarm:
# the slowest-converging joint, it carries the rear-wheel subtree), pivot velocity residual <= 9e-6 m/s. Measured on the defects: an
# omitted hinge leaves perpendicular >= 1.0 rad/s and (for wheels) pivot 1.4-1.8 m/s; a hinge tilted 25 degrees transmits an axial
# torque |H_A.a| >= 0.10 (every joint).

const KIND_AXIAL: int = 0
const KIND_PERP1: int = 1
const KIND_PERP2: int = 2
const KIND_LINEAR: int = 3
const MUT_NONE: int = 0
const MUT_MISSING: int = 1
const MUT_WRONG_AXIS: int = 2

const PROBE_L: float = 3.0
const PROBE_J: Vector3 = Vector3(12.0, -9.0, 15.0)
## Axial angular-momentum identity: exact in exact arithmetic at the first step; float32 over 8 rows x 100 iterations. Measured
## |H_B.a - L| and |H_A.a| are <= 1e-6 for all four joints, so 2e-4 is 200x margin and still 3 orders below L = 3.
const H_TOL: float = 2.0e-4
## B must rotate about the axis: unconstrained L / I_B_axial is >= 1.4 rad/s for L = 3 here; a blocked axis gives ~0.
const RATE_MIN: float = 0.1
## Residual of a constrained velocity row after one step at 100 iterations: measured <= 1.1e-3 rad/s (swingarm, convergence limited),
## so 1e-2 is ~9x margin; an unconstrained rotation gives 1-6 rad/s (>= 100x above).
const PERP_TOL: float = 1.0e-2
const ANCHOR_V_TOL: float = 2.0e-3


func _set_named_joint(a: MotorcycleAssembly, k: int, j) -> void:
	match k:
		0:
			a.steering_joint = j
		1:
			a.swingarm_joint = j
		2:
			a.front_axle_joint = j
		_:
			a.rear_axle_joint = j


## Replace hinge k by a defective one. MISSING removes it; WRONG_AXIS builds a consistent hinge (zero initial error) about an
## axis tilted by 25 degrees from the configured one, about the configured reference direction.
func _apply_mutation(a: MotorcycleAssembly, k: int, mutation: int) -> void:
	var old = a.joints()[k]
	a.world.remove_constraint(old)
	if mutation == MUT_MISSING:
		return
	var spec = a.config.joint_specs()[k]
	var wrong: Vector3 = Quaternion(MotorcycleBuilder.hinge_reference(spec.axis), deg_to_rad(25.0)) * spec.axis
	var replacement = HingeJoint.from_world(old.body_a, old.body_b, spec.point, wrong, MotorcycleBuilder.hinge_reference(wrong))
	a.world.add_constraint(replacement)
	_set_named_joint(a, k, replacement)


## One probe on a fresh assembly (optionally with hinge k defective); returns the measurements.
func _probe(k: int, mutation: int, kind: int) -> Dictionary:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return {}
	if mutation != MUT_NONE:
		_apply_mutation(a, k, mutation)
	var spec = a.config.joint_specs()[k]
	var pair: Array = _pair(a, k)
	# `pair` comes from the (possibly replaced) named joint, which always joins the same two bodies.
	var body_a: SimRigidBody = pair[0]
	var body_b: SimRigidBody = pair[1]
	var axis: Vector3 = spec.axis
	var ref: Vector3 = MotorcycleBuilder.hinge_reference(axis)
	var pivot: Vector3 = spec.point
	var lever_a: Vector3 = pivot - body_a.position
	var lever_b: Vector3 = pivot - body_b.position
	match kind:
		KIND_AXIAL:
			body_b.apply_angular_impulse(axis * PROBE_L)
		KIND_PERP1:
			body_b.apply_angular_impulse(ref * PROBE_L)
		KIND_PERP2:
			body_b.apply_angular_impulse(axis.cross(ref) * PROBE_L)
		KIND_LINEAR:
			body_b.apply_impulse(PROBE_J)
	a.world.step()
	var dw: Vector3 = body_b.angular_velocity - body_a.angular_velocity
	var v_a: Vector3 = body_a.linear_velocity + body_a.angular_velocity.cross(lever_a)
	var v_b: Vector3 = body_b.linear_velocity + body_b.angular_velocity.cross(lever_b)
	var sides: Array = _sides(a, k)
	return {
		"rate": dw.dot(axis),
		"perp": (dw - axis * dw.dot(axis)).length(),
		"anchor_v": (v_b - v_a).length(),
		"h_a": _h_axis(sides[0], pivot, axis),
		"h_b": _h_axis(sides[1], pivot, axis),
	}


## Messages for every acceptance criterion hinge k violates (empty = behaves exactly as the configuration says).
func _violations(k: int, mutation: int) -> PackedStringArray:
	var out: PackedStringArray = PackedStringArray()
	var axial: Dictionary = _probe(k, mutation, KIND_AXIAL)
	if axial.is_empty():
		out.append("assembly not built")
		return out
	if absf(axial.h_b - PROBE_L) > H_TOL:
		out.append("axial impulse not carried by the B side (H_B.a = %s, expected %s)" % [axial.h_b, PROBE_L])
	if absf(axial.h_a) > H_TOL:
		out.append("axial torque transmitted to the A side (H_A.a = %s)" % axial.h_a)
	if axial.rate < RATE_MIN:
		out.append("no relative rotation about the axis (rate %s)" % axial.rate)
	for kind in [KIND_PERP1, KIND_PERP2]:
		var perp: Dictionary = _probe(k, mutation, kind)
		if perp.perp > PERP_TOL:
			out.append("perpendicular relative rotation not constrained (%s rad/s, direction %d)" % [perp.perp, kind])
	var linear: Dictionary = _probe(k, mutation, KIND_LINEAR)
	if linear.anchor_v > ANCHOR_V_TOL:
		out.append("pivot not held (relative pivot velocity %s m/s)" % linear.anchor_v)
	return out


func test_c1_each_hinge_constrains_exactly_the_intended_degrees_of_freedom() -> void:
	for k in 4:
		var v: PackedStringArray = _violations(k, MUT_NONE)
		check(v.is_empty(), "%s: %s" % [JOINT_NAMES[k], v])
		# Report the measured values so the thresholds stay auditable.
		var axial: Dictionary = _probe(k, MUT_NONE, KIND_AXIAL)
		check(axial.rate >= 0.3, "%s: the free axis responds strongly (rate %s rad/s)" % [JOINT_NAMES[k], axial.rate])


func test_c2_wheels_spin_freely_about_their_axles_without_disturbing_anything_else() -> void:
	# The hinge transmits no axial torque and the wheel COM lies on its axle: an axial angular impulse spins the wheel at
	# exactly L / I_axle (principal axis, so the gyroscopic update leaves it unchanged) and nothing else moves.
	for wheel_index in 2:
		var a: MotorcycleAssembly = _build()
		if a == null:
			return
		var wheel: SimRigidBody = a.front_wheel if wheel_index == 0 else a.rear_wheel
		var joint = a.front_axle_joint if wheel_index == 0 else a.rear_axle_joint
		var carrier: SimRigidBody = joint.body_a
		var impulse: float = 5.0
		var i_axle: float = wheel.local_inertia.x.x
		wheel.apply_angular_impulse(Vector3.RIGHT * impulse)
		for i in 120:
			a.world.step()
		check_vec3_near(wheel.angular_velocity, Vector3.RIGHT * impulse / i_axle, 1.0e-5 * impulse / i_axle, "%s: w = L / I_axle after 1 s (principal-axis spin is conserved)" % wheel.body_name)
		check_near(joint.angle(), impulse / i_axle * 120.0 * DT, 1.0e-4 * impulse / i_axle, "%s: hinge angle = w t" % wheel.body_name)
		for other in a.bodies():
			if other != wheel:
				check(other.linear_velocity.length() <= 1.0e-7 and other.angular_velocity.length() <= 1.0e-7, "%s: untouched by the wheel spin (v %s, w %s)" % [other.body_name, other.linear_velocity, other.angular_velocity])
		check(carrier.position.is_equal_approx(a.config.fork_slider.position) or carrier.position.is_equal_approx(a.config.swingarm.position), "the carrier did not move")
		check(joint.linear_error().length() <= POSE_TOL and joint.axis_error_angle() <= POSE_TOL, "%s: axle joint error stays at construction level" % wheel.body_name)


func test_c3_steering_and_swingarm_rotate_about_their_axes_and_stay_attached() -> void:
	# Position-level bounds below are the measured worst cases (steering 4 N m s: anchor 3.2e-5 m, axis 6.3e-5 rad;
	# swingarm 3 N m s: 2.2e-5 m, 3.9e-6 rad) with ~8x margin; both are orders below the multi-metre / radian separation of
	# an omitted or misaligned hinge (section F). The mechanism is the second-order frozen-Jacobian error divided by beta.
	var cases: Array = [[0, 4.0], [1, 3.0]]
	for case in cases:
		var k: int = case[0]
		var a: MotorcycleAssembly = _build()
		if a == null:
			return
		var j = a.joints()[k]
		var spec = a.config.joint_specs()[k]
		j.body_b.apply_angular_impulse(spec.axis * case[1])
		var last_angle: float = 0.0
		var monotonic: bool = true
		var max_anchor: float = 0.0
		var max_axis: float = 0.0
		var max_residual: Vector2 = Vector2.ZERO
		var finite: bool = true
		for i in 120:
			a.world.step()
			var angle: float = j.angle()
			monotonic = monotonic and angle > last_angle
			last_angle = angle
			max_anchor = maxf(max_anchor, j.linear_error().length())
			max_axis = maxf(max_axis, j.axis_error_angle())
			var r: Vector2 = _residual(j)
			max_residual = Vector2(maxf(max_residual.x, r.x), maxf(max_residual.y, r.y))
			for b in a.bodies():
				finite = finite and _body_finite(b)
		check(finite, "%s: finite state throughout" % JOINT_NAMES[k])
		check(monotonic, "%s: the angle increases every step for a positive impulse about the axis" % JOINT_NAMES[k])
		check(last_angle > 0.5, "%s: the permitted DOF moved (angle %s rad after 1 s)" % [JOINT_NAMES[k], last_angle])
		check(max_anchor <= 4.0e-4, "%s: anchor separation stays bounded (peak %s m)" % [JOINT_NAMES[k], max_anchor])
		check(max_axis <= 5.0e-4, "%s: axis misalignment stays bounded (peak %s rad)" % [JOINT_NAMES[k], max_axis])
		check(max_residual.x <= 4.0e-3 and max_residual.y <= 8.0e-3, "%s: velocity rows converged at the linearization frames (anchor %s m/s, perpendicular %s rad/s)" % [JOINT_NAMES[k], max_residual.x, max_residual.y])
		check(a.world.numerical_fault == "", "%s: no numerical fault" % JOINT_NAMES[k])


func test_c4_a_free_axis_transmits_no_axial_torque_and_the_reaction_is_exact() -> void:
	# Action / reaction through each hinge: the A side must receive no angular momentum about the pivot along the axis, and
	# the B side keeps all of it, for either sign and different magnitudes. See H_TOL.
	var impulses: Array = [3.0, -2.0, 0.5]
	for k in 4:
		for l in impulses:
			var a: MotorcycleAssembly = _build()
			if a == null:
				return
			var spec = a.config.joint_specs()[k]
			var pair: Array = _pair(a, k)
			pair[1].apply_angular_impulse(spec.axis * l)
			a.world.step()
			var sides: Array = _sides(a, k)
			check_near(_h_axis(sides[1], spec.point, spec.axis), l, H_TOL, "%s: B-side axial angular momentum = applied impulse %s" % [JOINT_NAMES[k], l])
			check_near(_h_axis(sides[0], spec.point, spec.axis), 0.0, H_TOL, "%s: A-side axial angular momentum is zero for impulse %s" % [JOINT_NAMES[k], l])


# =====================================================================================================================
# D. Coupled dynamics and conservation
# =====================================================================================================================

func _excite_combined(a: MotorcycleAssembly) -> void:
	a.steering.apply_angular_impulse(a.config.steering_head.axis * 3.0)
	a.swingarm.apply_angular_impulse(Vector3.RIGHT * 2.0)
	a.front_wheel.apply_angular_impulse(Vector3.RIGHT * -6.0)
	a.rear_wheel.apply_angular_impulse(Vector3.RIGHT * -8.0)
	a.chassis.apply_impulse_at_point(Vector3(5.0, 0.0, -10.0), a.chassis.position + Vector3(0.0, 0.2, 0.0))


func test_d1_a_linear_impulse_on_the_rear_wheel_propagates_through_the_whole_assembly() -> void:
	# Impulse J applied ONLY to the rear wheel. Total linear momentum equals J (constraint impulses are equal and opposite;
	# float32 random walk over 120 steps x 100 iterations x 8 rows, measured <= 3.1e-4 for |J| = 20, tolerance 2e-3), and every
	# other body, including the far-away front wheel, acquires motion only through the hinges.
	var impulses: Array = [Vector3(20, 0, 0), Vector3(0, 0, -20)]
	for j_impulse in impulses:
		var a: MotorcycleAssembly = _build()
		if a == null:
			return
		var l0: Vector3 = Vector3.ZERO
		a.rear_wheel.apply_impulse(j_impulse)
		l0 = a.world.total_angular_momentum()
		var max_anchor: float = 0.0
		for i in 120:
			a.world.step()
			max_anchor = maxf(max_anchor, a.max_anchor_error())
		check_vec3_near(a.world.total_linear_momentum(), j_impulse, 2.0e-3, "total linear momentum equals the applied impulse %s" % j_impulse)
		for b in a.bodies():
			var moved: float = b.linear_velocity.length() + b.angular_velocity.length()
			check(moved > 1.0e-3, "%s moved although only the rear wheel was struck (%s)" % [b.body_name, moved])
		check(a.front_wheel.linear_velocity.length() > 0.0 and a.steering.linear_velocity.length() > 0.0, "the front end received the impulse")
		check(max_anchor <= 2.0e-3, "anchors stay attached under a 20 N s impulse (peak %s m; measured <= 4.0e-5)" % max_anchor)
		# Angular momentum about the origin: constraint terms C x lambda and the split displacement are bounded by
		# (anchor error x impulse); measured <= 9.2e-4 of |L| in these cases, tolerance 5e-3.
		var l1: Vector3 = a.world.total_angular_momentum()
		check((l1 - l0).length() <= 5.0e-3 * maxf(l0.length(), 1.0), "angular momentum about the origin conserved (change %s of %s)" % [(l1 - l0).length(), l0.length()])


func test_d2_combined_excitation_run_stays_finite_valid_and_bounded() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	_excite_combined(a)
	var w: SimWorld = a.world
	var p0: Vector3 = w.total_linear_momentum()
	var ke_first: float = 0.0
	var ke_peak: float = 0.0
	var peak_anchor: float = 0.0
	var peak_axis: float = 0.0
	var peak_w: float = 0.0
	var peak_rel: float = 0.0
	var ok_steps: int = 0
	var valid: bool = true
	var normalized: bool = true
	var inertia_ok: bool = true
	for i in 240:
		if not w.step():
			break
		ok_steps += 1
		if i == 0:
			ke_first = w.total_kinetic_energy()
		ke_peak = maxf(ke_peak, w.total_kinetic_energy())
		peak_anchor = maxf(peak_anchor, a.max_anchor_error())
		peak_axis = maxf(peak_axis, a.max_axis_error())
		for j in a.joints():
			peak_rel = maxf(peak_rel, absf(j.angle_rate()))
		for b in a.bodies():
			valid = valid and _body_finite(b)
			peak_w = maxf(peak_w, b.angular_velocity.length())
			normalized = normalized and absf(b.orientation.length() - 1.0) <= 1.0e-6
			var r: Basis = Basis(b.orientation)
			inertia_ok = inertia_ok and SimMassProperties.validate_inertia(b.inertia_world) == "" \
				and (b.inertia_world.x - (r * b.local_inertia * r.transposed()).x).length() <= 1.0e-5
	check(ok_steps == 240 and w.step_count == 240 and w.numerical_fault == "", "240 steps completed without a numerical fault")
	check(valid, "all positions, orientations, velocities and world inertias stay finite")
	check(normalized, "orientations stay unit length (<= 1e-6)")
	check(inertia_ok, "world inertia stays valid and equal to R I R^T")
	check_vec3_near(w.total_linear_momentum(), p0, 2.0e-3, "total linear momentum conserved (float32 random walk; measured 7.6e-4)")
	# No energy injection: after the first (impulsive) step the kinetic energy never rises, and the O(dt) frozen-Jacobian
	# dissipation of the documented limitation (measured 3 % over 2 s with 6.5 rad/s relative wheel spin) stays small.
	check(ke_peak <= ke_first * (1.0 + 1.0e-3), "kinetic energy never exceeds its post-impulse value (peak/first = %s)" % (ke_peak / ke_first))
	check(w.total_kinetic_energy() >= 0.9 * ke_first, "kinetic energy loss bounded (end/first = %s)" % (w.total_kinetic_energy() / ke_first))
	# Mechanism bounds (steady state e_step / beta with e_step = 1/2 (w_rel dt)(w dt) for angles, r (w dt)^2 / 2 for anchors),
	# doubled, from the measured peak rates of this run. Measured: 6.8e-5 m and 2.6e-3 rad.
	var beta: float = w.config.baumgarte_factor
	var axis_bound: float = peak_rel * peak_w * DT * DT / beta
	var anchor_bound: float = 0.7 * peak_w * DT * peak_w * DT / beta
	check(peak_axis <= axis_bound, "axis misalignment within the Baumgarte steady-state bound (%s <= %s rad)" % [peak_axis, axis_bound])
	check(peak_anchor <= anchor_bound, "anchor error within the Baumgarte steady-state bound (%s <= %s m)" % [peak_anchor, anchor_bound])
	check(peak_axis < 0.05 and peak_anchor < 5.0e-3, "and both far below visible detachment (%s rad, %s m)" % [peak_axis, peak_anchor])


func test_d3_gyroscopic_coupling_front_wheel_spin_reaches_the_chassis_through_the_steering() -> void:
	# A spinning front wheel resists being steered: its angular momentum changes direction, and the reaction travels
	# wheel -> fork -> steering hinge -> chassis. Compared with an identical world where the wheel does not spin.
	var results: Array = []
	for spin in [0.0, 9.0]:
		var a: MotorcycleAssembly = _build()
		if a == null:
			return
		a.front_wheel.apply_angular_impulse(Vector3.RIGHT * spin)
		a.steering.apply_angular_impulse(a.config.steering_head.axis * 2.0)
		var spin_rate0: float = a.front_axle_joint.angle_rate()
		var max_axis: float = 0.0
		var max_anchor: float = 0.0
		for i in 120:
			a.world.step()
			max_axis = maxf(max_axis, a.front_axle_joint.axis_error_angle())
			max_anchor = maxf(max_anchor, a.front_axle_joint.linear_error().length())
		results.append({"chassis_w": a.chassis.angular_velocity, "rate": a.front_axle_joint.angle_rate(), "rate0": spin_rate0, "axis": max_axis, "anchor": max_anchor, "fault": a.world.numerical_fault})
	var free: Dictionary = results[0]
	var spun: Dictionary = results[1]
	check(spun.fault == "" and free.fault == "", "no numerical fault")
	check((spun.chassis_w - free.chassis_w).length() > 0.05, "the wheel's spin changes the chassis motion (difference %s rad/s)" % (spun.chassis_w - free.chassis_w).length())
	# The steering motion does not change the wheel's spin rate about its own axle (no axial coupling): the relative rate is
	# only changed by the O(dt) gyroscopic direction error; measured 0.3 % over 1 s (tolerance 2 %). Axis error measured 2.6e-3 rad (bound 1e-2), anchor 2.5e-5 m (bound 1e-3).
	check(absf(spun.rate - spun.rate0) <= 0.02 * absf(spun.rate0), "relative wheel spin rate preserved (%s -> %s rad/s)" % [spun.rate0, spun.rate])
	check(spun.axis <= 1.0e-2 and spun.anchor <= 1.0e-3, "front axle stays attached and aligned while spinning and steering (%s rad, %s m)" % [spun.axis, spun.anchor])


# =====================================================================================================================
# E. Free fall and determinism
# =====================================================================================================================

func test_e1_free_fall_has_no_hidden_forces_or_relative_motion() -> void:
	# Gravity on, no ground: every COM accelerates at g, there is no torque, so no constraint is loaded and nothing moves
	# relative to anything else.
	var a: MotorcycleAssembly = _build(Vector3(0.0, -9.81, 0.0), 20)
	if a == null:
		return
	for i in 60:
		a.world.step()
	var t: float = 60.0 * DT
	for b in a.bodies():
		check_vec3_near(b.linear_velocity, Vector3(0.0, -9.81 * t, 0.0), 1.0e-4, "%s: v = g t" % b.body_name)
		check(b.angular_velocity.length() <= 1.0e-5, "%s: no rotation appears (%s)" % [b.body_name, b.angular_velocity.length()])
	for k in 4:
		var j = a.joints()[k]
		check(absf(j.angle()) <= 1.0e-5, "%s: hinge angle stays zero (%s)" % [JOINT_NAMES[k], j.angle()])
	check(a.max_anchor_error() <= 1.0e-5 and a.max_axis_error() <= 1.0e-5, "no constraint error in free fall")


func test_e2_identical_builds_and_runs_are_bit_identical() -> void:
	var states: Array = []
	for n in 2:
		var a: MotorcycleAssembly = _build(Vector3.ZERO, 20)
		if a == null:
			return
		_excite_combined(a)
		for i in 60:
			a.world.step()
		var snapshot: Array = []
		for b in a.bodies():
			snapshot.append([b.position, b.orientation, b.linear_velocity, b.angular_velocity])
		states.append(snapshot)
	check(states[0] == states[1], "two identical builds produce bit-identical state after 60 steps")


# =====================================================================================================================
# F. Regression sensitivity: the acceptance criteria of C must reject each structural defect
# =====================================================================================================================

func test_f1_a_missing_hinge_is_detected_for_every_joint() -> void:
	for k in 4:
		var v: PackedStringArray = _violations(k, MUT_MISSING)
		check(not v.is_empty(), "%s omitted: detected" % JOINT_NAMES[k])
		var text: String = " ".join(v)
		check(text.contains("pivot not held") and text.contains("perpendicular"), "%s omitted: pivot and perpendicular criteria both fail (%s)" % [JOINT_NAMES[k], v])


func test_f2_a_wrongly_configured_axis_is_detected_for_every_joint() -> void:
	for k in 4:
		var v: PackedStringArray = _violations(k, MUT_WRONG_AXIS)
		check(not v.is_empty(), "%s with the axis tilted 25 degrees: detected (%s)" % [JOINT_NAMES[k], v])
		check(" ".join(v).contains("axial torque transmitted"), "%s wrong axis: the free-axis identity fails (%s)" % [JOINT_NAMES[k], v])


func test_f3_inconsistent_local_frames_are_caught_at_construction() -> void:
	# A hinge whose two local axes disagree (here B's axis 10 degrees off) starts misaligned; construction_error reports it
	# and the axis error equals the tilt.
	for k in 4:
		var a: MotorcycleAssembly = _build()
		if a == null:
			return
		var old = a.joints()[k]
		a.world.remove_constraint(old)
		var tilted: Vector3 = Quaternion(MotorcycleBuilder.hinge_reference(a.config.joint_specs()[k].axis), deg_to_rad(10.0)) * old.local_axis_b
		var ref_b: Vector3 = (old.local_reference_b - tilted * tilted.dot(old.local_reference_b)).normalized()
		var bad = HingeJoint.new(old.body_a, old.body_b, old.local_anchor_a, old.local_anchor_b, old.local_axis_a, tilted, old.local_reference_a, ref_b)
		a.world.add_constraint(bad)
		_set_named_joint(a, k, bad)
		check(MotorcycleBuilder.construction_error(a).containsn("axes"), "%s: misaligned local axes reported" % JOINT_NAMES[k])
		check_near(bad.axis_error_angle(), deg_to_rad(10.0), 1.0e-5, "%s: axis error equals the 10 degree tilt" % JOINT_NAMES[k])
