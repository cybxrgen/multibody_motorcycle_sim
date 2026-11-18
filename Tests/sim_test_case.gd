extends RefCounted
## Base class for test files.
## Every method named `test_*` in a subclass is run by Tests/run_tests.gd.
## Tests record failures through check*() helpers instead of aborting, so one
## test reports every violated expectation.

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")
const SimMassProperties = preload("res://Simulation/Core/sim_mass_properties.gd")
const SimBodyState = preload("res://Simulation/Core/sim_body_state.gd")
const SimConstraint = preload("res://Simulation/Solver/sim_constraint.gd")
const BallJoint = preload("res://Simulation/Solver/ball_joint_constraint.gd")

var checks_run: int = 0
var failures: Array[String] = []
var skip_reason: String = ""


func reset_for_test() -> void:
	checks_run = 0
	failures = []
	skip_reason = ""


func check(condition: bool, message: String) -> void:
	checks_run += 1
	if not condition:
		failures.append(message)


func check_near(actual: float, expected: float, tolerance: float, message: String) -> void:
	checks_run += 1
	if is_nan(actual) or absf(actual - expected) > tolerance:
		failures.append("%s (actual=%s expected=%s tol=%s)" % [message, actual, expected, tolerance])


func check_vec3_near(actual: Vector3, expected: Vector3, tolerance: float, message: String) -> void:
	checks_run += 1
	var err: float = (actual - expected).length()
	if is_nan(err) or err > tolerance:
		failures.append("%s (actual=%s expected=%s err=%s tol=%s)" % [message, actual, expected, err, tolerance])


func check_basis_near(actual: Basis, expected: Basis, tolerance: float, message: String) -> void:
	checks_run += 1
	var err: float = 0.0
	err = maxf(err, (actual.x - expected.x).length())
	err = maxf(err, (actual.y - expected.y).length())
	err = maxf(err, (actual.z - expected.z).length())
	if is_nan(err) or err > tolerance:
		failures.append("%s (err=%s tol=%s)" % [message, err, tolerance])


func skip(reason: String) -> void:
	skip_reason = reason


# --- Shared builders ---

func make_world(gravity: Vector3 = Vector3.ZERO) -> SimWorld:
	var w: SimWorld = SimWorld.new()
	w.config.gravity = gravity
	return w


## Dynamic body with an explicit inertia tensor.
func make_body(mass: float, inertia: Basis, pos: Vector3 = Vector3.ZERO, rot: Quaternion = Quaternion.IDENTITY) -> SimRigidBody:
	return SimRigidBody.new(SimMassProperties.new(mass, inertia), pos, rot)


func make_sphere(mass: float, radius: float, pos: Vector3 = Vector3.ZERO) -> SimRigidBody:
	return make_body(mass, SimMassProperties.sphere_inertia(mass, radius), pos)


func make_static(pos: Vector3 = Vector3.ZERO) -> SimRigidBody:
	return SimRigidBody.new(SimMassProperties.new(0.0), pos)


func run_steps(world: SimWorld, count: int) -> void:
	for i in count:
		world.step()
