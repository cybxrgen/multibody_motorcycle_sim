extends RefCounted
## The single authority over all physical state. Scene-tree independent.
##
## Step pipeline (differences from the conceptual pipeline are noted in ARCHITECTURE.md):
##  1. capture previous state (for render interpolation)
##  2. accumulate gravity into force accumulators
##  3. integrate velocities (forces, torques, implicit-midpoint gyroscopic update); validate the result (see below)
##  4. zero the split (pseudo) velocities
##  5. prepare + solve constraints: physical velocity pass, then split-impulse pass (positional correction, M1.3)
##  6. integrate positions / orientations with physical + split velocities
##  7. zero the split velocities again (they never persist; physical velocities are untouched by split impulses)
##  8. publish new state
##  9. clear force/torque accumulators
## Failure containment: the velocities are checked after step 3 (before the solver could spread a bad value to other bodies)
## and the whole state after step 6 (before step 8 publishes it). A non-finite value, a non-unit orientation or a failed
## gyroscopic solve aborts the step: every body is restored to its state at the start of the step, nothing is published, the
## fault is recorded in `numerical_fault` / `last_error` with the body id and name, reported once with push_error, and all
## further steps are refused until clear_numerical_fault(). The renderer therefore never receives a non-finite transform.

const SimConfig = preload("res://Simulation/Core/sim_config.gd")
const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")
const SimIntegrator = preload("res://Simulation/Core/sim_integrator.gd")
const ConstraintSolver = preload("res://Simulation/Solver/constraint_solver.gd")
const SimConstraint = preload("res://Simulation/Solver/sim_constraint.gd")

var config: SimConfig = SimConfig.new()
var bodies: Array = []
var constraints: Array = []
var step_count: int = 0
var simulation_time: float = 0.0
var accumulator: float = 0.0
## Total simulated time discarded by the backlog guard (diagnostic).
var dropped_time: float = 0.0

## Reason the last advance()/step() was refused, "" when the world is runnable.
var last_error: String = ""
## Number of advance()/step() calls refused because of invalid configuration or bodies.
var rejected_step_count: int = 0

## Description of a numerical failure detected during a step, "" when none. While set, preflight() refuses every step.
var numerical_fault: String = ""
## Tolerance of | |q| - 1 | for a stored orientation (float32 unit quaternion rounding is ~6e-8; the setter normalizes every update).
const ORIENTATION_NORM_TOLERANCE: float = 1.0e-5

var _reported_error: String = ""
var _next_body_id: int = 0
var _solver: ConstraintSolver = ConstraintSolver.new()


## Returns the body, or null (with an error) if it is null or has invalid mass properties.
func add_body(body: SimRigidBody) -> SimRigidBody:
	if body == null or not body.mass_properties.valid:
		push_error("SimWorld.add_body: rejected body with invalid mass properties.")
		return null
	body.id = _next_body_id
	_next_body_id += 1
	body.reset_published_state()
	bodies.append(body)
	return body


## Removes the body and, first, every registered constraint that references it (as body_a or body_b), so the world never
## holds a constraint whose body is absent. Chosen over refusing the removal because remove_body() has no failure channel
## (void, like remove_constraint) and a constraint is meaningless without both bodies. Constraints between other bodies
## and their order are untouched. Removing a body that is not a member of this world does nothing.
func remove_body(body: SimRigidBody) -> void:
	if not has_body(body):
		return
	for i in range(constraints.size() - 1, -1, -1):
		var c: SimConstraint = constraints[i]
		if c.body_a == body or c.body_b == body:
			constraints.remove_at(i)
	bodies.erase(body)


## True if `body` is a registered member of this world. Object identity only: never body ids or property equality.
func has_body(body: SimRigidBody) -> bool:
	return body != null and bodies.has(body)


## Returns the constraint, or null (with an error) if it is null, references a body that is null or not registered in this
## world (identity check, any SimConstraint subclass), or reports invalid data (SimConstraint.validate()).
func add_constraint(constraint: SimConstraint) -> SimConstraint:
	if constraint == null:
		push_error("SimWorld.add_constraint: rejected null constraint.")
		return null
	var membership_error: String = _membership_error(constraint)
	if membership_error != "":
		push_error("SimWorld.add_constraint: rejected constraint: %s." % membership_error)
		return null
	var errors: PackedStringArray = constraint.validate()
	if not errors.is_empty():
		push_error("SimWorld.add_constraint: rejected invalid constraint: %s" % "; ".join(errors))
		return null
	constraints.append(constraint)
	return constraint


func remove_constraint(constraint: SimConstraint) -> void:
	constraints.erase(constraint)


## "" if both bodies of the constraint are registered in this world, otherwise a description of the violation.
func _membership_error(constraint: SimConstraint) -> String:
	if constraint.body_a == null or constraint.body_b == null:
		return "body_a or body_b is null"
	if not has_body(constraint.body_a):
		return "body_a is not registered in this world"
	if not has_body(constraint.body_b):
		return "body_b is not registered in this world"
	return ""


## Checks that the world can be stepped: valid configuration, valid body mass properties, valid constraints, and every
## constraint's bodies still registered in this world (defensive: bodies/constraints are public and may be edited directly).
## Pure check except for diagnostics (last_error, one push_error per distinct failure message).
## SimConfig is a mutable Resource, so this runs at the start of every advance() and step().
func preflight() -> bool:
	var message: String = ""
	if numerical_fault != "":
		message = "numerical fault: " + numerical_fault
	elif config == null:
		message = "config is null"
	else:
		var errors: PackedStringArray = config.validate()
		if not errors.is_empty():
			message = "invalid SimConfig: " + "; ".join(errors)
		else:
			for body in bodies:
				if not body.mass_properties.valid:
					message = "body %d has invalid mass properties: %s" % [body.id, body.mass_properties.error_message]
					break
			if message == "":
				# Constraint data is mutable (limits, motor settings, anchors), so it is re-checked every step.
				for i in constraints.size():
					var membership_error: String = _membership_error(constraints[i])
					if membership_error != "":
						message = "constraint %d violates world membership: %s" % [i, membership_error]
						break
					var constraint_errors: PackedStringArray = constraints[i].validate()
					if not constraint_errors.is_empty():
						message = "constraint %d is invalid: %s" % [i, "; ".join(constraint_errors)]
						break
	last_error = message
	if message == "":
		_reported_error = ""
		return true
	if message != _reported_error:
		_reported_error = message
		push_error("SimWorld: %s. Steps are refused (no state changes) until this is fixed." % message)
	return false


## Slack when comparing the accumulator with one timestep, to absorb floating-point rounding in
## accumulated frame times. min(1e-12 s, 1e-9 * dt): identical to the former absolute 1e-12 for any
## dt >= 1e-3 s (including the 1/120 default), and never larger than a billionth of a step, so it can
## not make `accumulator >= dt - tol` true for an empty accumulator (the former absolute value did
## for dt <= 1e-12). Double rounding error per addition is ~1e-16 relative; 1e-9 leaves ~7 orders of margin.
static func step_tolerance(dt: float) -> float:
	return minf(1.0e-12, dt * 1.0e-9)


## Feed real frame time; runs zero or more fixed steps. Returns the number of steps that COMPLETED successfully.
## If preflight() fails, nothing changes (accumulator, state, time) and 0 is returned.
## Accumulator policy on a numerical fault: the first step that aborts ends the call immediately. The aborted step is not
## counted and its dt is NOT subtracted: the accumulator keeps the time of the aborted step and of every step not yet
## attempted (including this call's frame time, which was added before the loop), and the backlog guard below does not run,
## so no simulated time is silently discarded. The world is then in the numerical-fault state, in which preflight() refuses
## every further advance()/step() (counted in rejected_step_count, accumulator untouched, the refused frame time is not
## accumulated) until clear_numerical_fault(); no failed step is retried automatically.
func advance(frame_delta: float) -> int:
	if not preflight():
		rejected_step_count += 1
		return 0
	var dt: float = config.fixed_timestep
	accumulator += minf(maxf(frame_delta, 0.0), config.max_frame_time)
	var tol: float = step_tolerance(dt)
	var steps: int = 0
	while accumulator >= dt - tol and steps < config.max_steps_per_frame:
		if not _step_unchecked():
			return steps
		accumulator -= dt
		steps += 1
	if accumulator >= dt - tol:
		# Cap reached with backlog remaining: drop whole steps, keep the sub-step phase.
		var keep: float = fmod(accumulator, dt)
		if keep > dt - tol:
			keep = 0.0  # fmod rounding landed just below a whole step
		dropped_time += accumulator - keep
		accumulator = keep
	accumulator = maxf(accumulator, 0.0)
	return steps


## Fraction of a step the renderer is ahead of the last published state, in [0, 1).
func interpolation_alpha() -> float:
	if config == null or not (is_finite(config.fixed_timestep) and config.fixed_timestep > 0.0):
		return 0.0
	return clampf(accumulator / config.fixed_timestep, 0.0, 1.0)


## One fixed step. Returns true only if the complete step succeeded. Returns false if preflight() refuses the step (nothing
## changes, counted in rejected_step_count) or if the step was aborted by a numerical fault (rolled back, see _abort_step();
## the fault is in `numerical_fault`).
func step() -> bool:
	if not preflight():
		rejected_step_count += 1
		return false
	return _step_unchecked()


## Runs one fixed step without preflight. Returns true if it completed and was published, false if it was aborted by
## _abort_step() (every body restored, nothing published, step_count and simulation_time unchanged).
func _step_unchecked() -> bool:
	var dt: float = config.fixed_timestep
	for body in bodies:
		body.previous_state.capture_from(body)
	for body in bodies:
		if not body.is_static():
			body.apply_force(config.gravity * body.mass)
	for body in bodies:
		if not SimIntegrator.integrate_velocities(body, dt):
			_abort_step("body %d (%s): implicit gyroscopic solve failed (non-finite input or no convergence, |w| dt = %s)" % [body.id, body.body_name, body.angular_velocity.length() * dt])
			return false
		body.clear_split_velocity()
	var fault: String = _invalid_state_description(false)
	if fault != "":
		_abort_step(fault + " after velocity integration (before the constraint solve)")
		return false
	_solver.solve(constraints, dt, config)
	fault = _invalid_state_description(false)
	if fault != "":
		_abort_step(fault + " after the constraint solve")
		return false
	for body in bodies:
		SimIntegrator.integrate_positions(body, dt)
		body.clear_split_velocity()
	fault = _invalid_state_description(true)
	if fault != "":
		_abort_step(fault + " after position integration")
		return false
	for body in bodies:
		body.published_state.capture_from(body)
		body.clear_accumulators()
	step_count += 1
	simulation_time += dt
	return true


## "" if every dynamic body has finite velocities (and, when `check_pose`, finite position, orientation of unit length and
## finite world inertia); otherwise a description naming the first offending body and quantity.
func _invalid_state_description(check_pose: bool) -> String:
	for body in bodies:
		if body.is_static():
			continue
		var what: String = ""
		if not _vec_finite(body.linear_velocity):
			what = "linear velocity %s" % body.linear_velocity
		elif not _vec_finite(body.angular_velocity):
			what = "angular velocity %s" % body.angular_velocity
		elif not _vec_finite(body.split_linear_velocity) or not _vec_finite(body.split_angular_velocity):
			what = "split velocity"
		elif check_pose:
			var q: Quaternion = body.orientation
			if not _vec_finite(body.position):
				what = "position %s" % body.position
			elif not (is_finite(q.x) and is_finite(q.y) and is_finite(q.z) and is_finite(q.w)):
				what = "orientation %s" % q
			elif absf(q.length() - 1.0) > ORIENTATION_NORM_TOLERANCE:
				what = "orientation not normalized (|q| = %s)" % q.length()
			elif not (_vec_finite(body.inertia_world.x) and _vec_finite(body.inertia_world.y) and _vec_finite(body.inertia_world.z)):
				what = "world inertia"
		if what != "":
			return "body %d (%s): non-finite or invalid %s" % [body.id, body.body_name, what]
	return ""


static func _vec_finite(v: Vector3) -> bool:
	return is_finite(v.x) and is_finite(v.y) and is_finite(v.z)


## Aborts the current step: restores every body to its start-of-step state, discards forces and split velocities, publishes
## nothing, does not advance time, records the fault and reports it once.
## Constraint solver state is invalidated, not restored: the solver may already have rewritten the carried (warm-start)
## impulses of every constraint, possibly with non-finite values, and the pre-step values are not kept (keeping them would
## cost a copy per row in every normal step). reset_warm_start() zeroes them, so the first step after
## clear_numerical_fault() starts from a consistent cold solver state. Pose-derived tracker state (the hinge angle) is not
## touched by the solver before the bodies move, so it already agrees with the restored poses.
func _abort_step(message: String) -> void:
	for body in bodies:
		body.position = body.previous_state.position
		body.orientation = body.previous_state.orientation
		body.linear_velocity = body.previous_state.linear_velocity
		body.angular_velocity = body.previous_state.angular_velocity
		body.clear_split_velocity()
		body.clear_accumulators()
	for c in constraints:
		c.reset_warm_start()
	numerical_fault = "%s (step %d, t = %s s). The step was aborted and the world restored to the state before it; steps are refused until clear_numerical_fault()." % [message, step_count + 1, simulation_time]
	last_error = "numerical fault: " + numerical_fault
	_reported_error = last_error
	push_error("SimWorld: numerical fault: %s" % numerical_fault)


## Clears a recorded numerical fault so stepping may resume (the caller is responsible for having fixed the cause).
func clear_numerical_fault() -> void:
	numerical_fault = ""


# --- Diagnostics (read-only) ---

func total_linear_momentum() -> Vector3:
	var p: Vector3 = Vector3.ZERO
	for body in bodies:
		p += body.linear_momentum()
	return p


## Angular momentum about the world origin: sum of x_com x (m v) + I_world omega.
func total_angular_momentum() -> Vector3:
	var l: Vector3 = Vector3.ZERO
	for body in bodies:
		l += body.position.cross(body.linear_momentum()) + body.angular_momentum_about_com()
	return l


func total_kinetic_energy() -> float:
	var e: float = 0.0
	for body in bodies:
		e += body.kinetic_energy()
	return e


func total_potential_energy() -> float:
	var e: float = 0.0
	for body in bodies:
		e -= body.mass * config.gravity.dot(body.position)
	return e
