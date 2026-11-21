extends "res://Tests/sim_test_case.gd"
## Numerical-fault handling of SimWorld (PR #8 hardening): step() return semantics, advance() behaviour and accumulator policy after
## an aborted step, and rollback-safety of the constraint solver's carried (warm-start) state.
##
## Faults are injected INSIDE the step, through the real world-step / constraint-solver path: a probe constraint overrides
## prepare() and, on a chosen call, writes a non-finite linear velocity into one of its bodies after the velocity checks have
## passed. The solver then spreads it into the rows' accumulated impulses (non-finite warm-start state) and SimWorld detects it
## after the solve. The probe never edits cached impulses itself. The rotation, joint and contact mathematics are the real ones.
##
## Tolerances: accumulator/time values are float64 sums of a few terms of size dt (1e-12 s is ~1e-10 of a step); body states of
## the recovery comparisons are float32 results of identical operation sequences (1e-6, normally bit-identical).

const DT: float = 1.0 / 120.0


## Ball joint that counts how many steps it took part in (prepare() calls) and, on call `fault_on_call`, corrupts body B.
class ProbeBall extends "res://Simulation/Solver/ball_joint_constraint.gd":
	var prepare_calls: int = 0
	var fault_on_call: int = -1
	var impulses_at_finish: Array = []

	func prepare(dt: float, config) -> void:
		prepare_calls += 1
		if prepare_calls == fault_on_call:
			body_b.linear_velocity = Vector3(NAN, 0.0, 0.0)
		super(dt, config)

	func finish() -> void:
		impulses_at_finish = []
		for row in rows:
			impulses_at_finish.append(row.accumulated_impulse)
		super()


## Contact (normal + friction) with the same probe behaviour; its friction impulse is warm-start state outside the rows.
class ProbeContact extends "res://Simulation/Solver/contact_constraint.gd":
	var prepare_calls: int = 0
	var fault_on_call: int = -1

	func prepare(dt: float, config) -> void:
		prepare_calls += 1
		if prepare_calls == fault_on_call:
			body_b.linear_velocity = Vector3(NAN, 0.0, 0.0)
		super(dt, config)


# --- Builders and helpers ---

## Static anchor at the origin, 1 kg sphere on a 1 m horizontal arm (ball joint at the anchor), gravity on: the joint
## carries a non-trivial impulse from the first step.
func _pendulum(fault_on_call: int = -1) -> Dictionary:
	var w: SimWorld = make_world(Vector3(0, -9.81, 0))
	var anchor: SimRigidBody = w.add_body(make_static(Vector3.ZERO))
	var bob: SimRigidBody = w.add_body(make_sphere(1.0, 0.1, Vector3(1, 0, 0)))
	var joint = ProbeBall.new(anchor, bob, Vector3.ZERO, Vector3(-1, 0, 0))
	joint.fault_on_call = fault_on_call
	w.add_constraint(joint)
	return {"world": w, "anchor": anchor, "bob": bob, "joint": joint}


func _impulses(c) -> Array:
	var out: Array = []
	for row in c.rows:
		out.append(row.accumulated_impulse)
	return out


## True if every row's carried physical and split impulse is exactly zero (and therefore finite).
func _rows_are_cold(c) -> bool:
	for row in c.rows:
		if row.accumulated_impulse != 0.0 or row.split_accumulated_impulse != 0.0:
			return false
	return true


func _rows_are_finite(c) -> bool:
	for row in c.rows:
		if not (is_finite(row.accumulated_impulse) and is_finite(row.split_accumulated_impulse)):
			return false
	return true


func _copy_dynamics(from: SimRigidBody, to: SimRigidBody) -> void:
	to.position = from.position
	to.orientation = from.orientation
	to.linear_velocity = from.linear_velocity
	to.angular_velocity = from.angular_velocity


# --- A. step() return semantics ---

func test_a1_successful_step_returns_true() -> void:
	var s: Dictionary = _pendulum()
	var w: SimWorld = s.world
	check(w.step() == true, "a complete step returns true")
	check(w.step_count == 1 and w.numerical_fault == "" and w.rejected_step_count == 0, "counted, no fault, no rejection")
	check_near(w.simulation_time, DT, 1e-12, "simulation time advanced by one step")


func test_a2_aborted_step_returns_false() -> void:
	var s: Dictionary = _pendulum(3)
	var w: SimWorld = s.world
	var bob: SimRigidBody = s.bob
	check(w.step() and w.step(), "steps 1 and 2 succeed")
	var pos: Vector3 = bob.position
	var lv: Vector3 = bob.linear_velocity
	var av: Vector3 = bob.angular_velocity
	var published: Vector3 = bob.published_state.position
	var time: float = w.simulation_time
	var ok: bool = w.step()
	check(ok == false, "step() returns false when the step was aborted by a numerical fault")
	check(w.numerical_fault != "" and w.numerical_fault.contains("after the constraint solve"), "fault recorded with its stage: %s" % w.numerical_fault)
	check(w.step_count == 2 and w.simulation_time == time, "aborted step is neither counted nor timed")
	check(w.rejected_step_count == 0, "an abort is not a preflight rejection: rejected_step_count unchanged")
	check(bob.position == pos and bob.linear_velocity == lv and bob.angular_velocity == av, "body restored to the start of the step")
	check(bob.published_state.position == published, "nothing published by the aborted step")
	check(w.step() == false and w.rejected_step_count == 1, "while faulted step() is refused by preflight (false, counted once)")
	check(s.joint.prepare_calls == 3, "the refused call did not reach the solver")


func test_a3_preflight_rejection_semantics_unchanged() -> void:
	var s: Dictionary = _pendulum()
	var w: SimWorld = s.world
	w.config.fixed_timestep = -1.0
	check(w.step() == false, "invalid configuration: step() false")
	check(w.rejected_step_count == 1 and w.step_count == 0 and s.joint.prepare_calls == 0, "counted as rejection, nothing stepped")
	w.config.fixed_timestep = DT
	check(w.step() == true and w.rejected_step_count == 1, "recovers once the configuration is valid; counter unchanged")


# --- B. advance() after a fault ---

func test_b1_successful_multi_step_advance_counts_steps() -> void:
	var s: Dictionary = _pendulum()
	var w: SimWorld = s.world
	check(w.advance(5.0 * DT) == 5, "five steps of accumulated time run five steps")
	check(w.step_count == 5 and s.joint.prepare_calls == 5, "five steps executed")
	check_near(w.simulation_time, 5.0 * DT, 1e-12, "simulation time")
	check_near(w.accumulator, 0.0, 1e-12, "accumulator consumed")
	# The step cap is unchanged: 0.1 s is 12 steps, at most max_steps_per_frame (10) run, the whole-step remainder is dropped.
	var s2: Dictionary = _pendulum()
	var w2: SimWorld = s2.world
	check(w2.advance(0.1) == w2.config.max_steps_per_frame, "step cap still applies")
	check_near(w2.dropped_time, 0.1 - w2.config.max_steps_per_frame * DT, 1e-9, "backlog beyond the cap is still dropped on success")


func test_b2_first_step_fault_stops_immediately_and_keeps_the_time() -> void:
	var s: Dictionary = _pendulum(1)
	var w: SimWorld = s.world
	var bob: SimRigidBody = s.bob
	var joint = s.joint
	var start_pos: Vector3 = bob.position
	var steps: int = w.advance(6.0 * DT)  # enough time for six fixed steps
	check(steps == 0, "zero steps reported as executed (got %d)" % steps)
	check(joint.prepare_calls == 1, "exactly one step was attempted (got %d)" % joint.prepare_calls)
	check(w.step_count == 0 and w.simulation_time == 0.0, "no step counted, no time advanced")
	check(w.numerical_fault != "", "fault recorded")
	check(bob.position == start_pos and bob.linear_velocity == Vector3.ZERO and bob.angular_velocity == Vector3.ZERO, "no body integration remains: bob exactly at its initial state")
	check_near(w.accumulator, 6.0 * DT, 1e-12, "policy: the accumulator keeps the aborted step's time and all unprocessed time")
	check(w.dropped_time == 0.0, "no time dropped by the backlog guard")
	check(w.rejected_step_count == 0, "the abort itself is not a rejection")


func test_b3_repeated_calls_while_faulted_do_not_retry() -> void:
	var s: Dictionary = _pendulum(1)
	var w: SimWorld = s.world
	var joint = s.joint
	w.advance(6.0 * DT)
	var acc: float = w.accumulator
	for i in 3:
		check(w.advance(DT) == 0, "advance() while faulted returns 0 (call %d)" % i)
	check(w.advance(1.0) == 0, "also for a large frame time")
	check(w.step() == false, "step() while faulted returns false")
	check(joint.prepare_calls == 1, "no failed step was attempted again (prepare calls %d)" % joint.prepare_calls)
	check(w.accumulator == acc, "accumulator untouched while faulted (the refused frame time is not accumulated)")
	check(w.rejected_step_count == 5, "every refused call counted (got %d)" % w.rejected_step_count)
	check(w.step_count == 0 and w.dropped_time == 0.0, "nothing stepped, nothing dropped")


func test_b4_mid_call_fault_keeps_completed_steps_and_the_remaining_time() -> void:
	var s: Dictionary = _pendulum(3)
	var w: SimWorld = s.world
	var joint = s.joint
	var steps: int = w.advance(6.0 * DT)
	check(steps == 2, "two steps completed before the fault (got %d)" % steps)
	check(joint.prepare_calls == 3, "the third step was attempted once and the loop stopped (got %d)" % joint.prepare_calls)
	check(w.step_count == 2, "only the completed steps are counted")
	check_near(w.simulation_time, 2.0 * DT, 1e-12, "time advanced only by the completed steps")
	check_near(w.accumulator, 4.0 * DT, 1e-12, "accumulator keeps the aborted step's dt plus the three unattempted steps")
	check(w.dropped_time == 0.0, "no time dropped")
	check(w.advance(DT) == 0 and joint.prepare_calls == 3, "no retry while the fault is active")
	# Explicit recovery: the preserved time is then consumed (the cause was in the step, rollback removed it).
	w.clear_numerical_fault()
	var resumed: int = w.advance(0.0)
	check(resumed == 4, "after clear_numerical_fault() the preserved time runs four steps (got %d)" % resumed)
	check(w.step_count == 6 and w.numerical_fault == "", "all six steps now executed")
	check_near(w.simulation_time, 6.0 * DT, 1e-12, "simulation time complete")
	check_near(w.accumulator, 0.0, 1e-12, "accumulator consumed")


# --- C. Constraint solver rollback ---

func test_c1_joint_warm_start_is_not_left_contaminated_by_a_fault() -> void:
	var s: Dictionary = _pendulum(12)
	var w: SimWorld = s.world
	var bob: SimRigidBody = s.bob
	var joint = s.joint
	for i in 11:
		check(w.step(), "step %d succeeds" % (i + 1))
	var before: Array = _impulses(joint)
	var biggest: float = 0.0
	for v in before:
		biggest = maxf(biggest, absf(v))
	check(biggest > 1e-3, "the joint carries real warm-start impulses before the fault (max |lambda| = %s)" % biggest)
	var pos: Vector3 = bob.position
	var orientation: Quaternion = bob.orientation
	var lv: Vector3 = bob.linear_velocity
	var av: Vector3 = bob.angular_velocity
	var published: Vector3 = bob.published_state.position
	var steps_before: int = w.step_count

	var ok: bool = w.step()
	check(ok == false, "the failure is detected and reported by step()")
	check(joint.prepare_calls == 12, "the solver path ran (prepare called for the faulty step)")
	check(w.numerical_fault.contains("after the constraint solve"), "detected after the solve: %s" % w.numerical_fault)
	check(joint.impulses_at_finish != before, "the solver had rewritten the warm-start impulses before the fault was detected (at finish: %s, before: %s)" % [joint.impulses_at_finish, before])
	check(w.step_count == steps_before, "aborted step not counted")
	check(bob.position == pos and bob.orientation == orientation and bob.linear_velocity == lv and bob.angular_velocity == av, "body state rolled back exactly")
	check(bob.published_state.position == published, "published state is the last valid one")
	check(_rows_are_finite(joint), "no non-finite cached impulse remains")
	check(_rows_are_cold(joint), "carried impulses invalidated (reset to zero), not left partially updated")

	# Recovery: after the explicit clear, the next step must be the same as a step of a fresh world from the same state.
	var twin: Dictionary = _pendulum()
	_copy_dynamics(bob, twin.bob)
	w.clear_numerical_fault()
	check(w.step() == true, "a valid step after clear_numerical_fault() succeeds")
	check(twin.world.step() == true, "twin step succeeds")
	check(w.numerical_fault == "" and w.step_count == steps_before + 1, "counted, no new fault")
	check_vec3_near(bob.position, twin.bob.position, 1e-6, "position equals the cold-start twin")
	check_vec3_near(bob.linear_velocity, twin.bob.linear_velocity, 1e-6, "velocity equals the cold-start twin")
	check_vec3_near(bob.angular_velocity, twin.bob.angular_velocity, 1e-6, "angular velocity equals the cold-start twin")
	check(_rows_are_finite(joint), "impulses finite after recovery")
	for i in 5:
		check(w.step(), "stepping continues normally (%d)" % i)


func test_c2_contact_friction_warm_start_is_not_left_contaminated_by_a_fault() -> void:
	var w: SimWorld = make_world(Vector3(0, -9.81, 0))
	var ground: SimRigidBody = w.add_body(make_static())
	var ball: SimRigidBody = w.add_body(make_sphere(1.0, 0.5, Vector3(0, 0.5, 0)))
	ball.linear_velocity = Vector3(3.0, 0, 0)  # sliding: rolling is reached after ~21 steps, the fault comes at step 9
	var contact = ProbeContact.new(ground, ball, Vector3.ZERO, Vector3(0, -0.5, 0), Vector3.UP, 0.5)
	contact.fault_on_call = 9
	w.add_constraint(contact)
	for i in 8:
		check(w.step(), "step %d succeeds" % (i + 1))
	check(contact._friction_impulse.length() > 1e-3, "friction impulse is carried (warm-start state outside the rows): %s" % contact._friction_impulse)
	check(contact.normal_impulse() > 1e-3, "normal impulse is carried: %s" % contact.normal_impulse())
	var pos: Vector3 = ball.position
	var lv: Vector3 = ball.linear_velocity
	var av: Vector3 = ball.angular_velocity
	var steps_before: int = w.step_count

	check(w.step() == false, "the failure is detected and reported by step()")
	check(w.numerical_fault.contains("after the constraint solve"), "detected after the solve: %s" % w.numerical_fault)
	check(w.step_count == steps_before, "aborted step not counted")
	check(ball.position == pos and ball.linear_velocity == lv and ball.angular_velocity == av, "body state rolled back exactly")
	check(_rows_are_finite(contact) and _rows_are_cold(contact), "row impulses finite and invalidated")
	var f: Vector3 = contact._friction_impulse
	check(is_finite(f.x) and is_finite(f.y) and is_finite(f.z) and f == Vector3.ZERO, "carried friction impulse invalidated, not non-finite or stale: %s" % f)

	# Recovery equals a cold-start step from the same state.
	var tw: SimWorld = make_world(Vector3(0, -9.81, 0))
	var tg: SimRigidBody = tw.add_body(make_static())
	var tb: SimRigidBody = tw.add_body(make_sphere(1.0, 0.5, Vector3(0, 0.5, 0)))
	_copy_dynamics(ball, tb)
	tw.add_constraint(ProbeContact.new(tg, tb, Vector3.ZERO, Vector3(0, -0.5, 0), Vector3.UP, 0.5))
	w.clear_numerical_fault()
	check(w.step() == true and tw.step() == true, "valid steps after clear_numerical_fault()")
	check_vec3_near(ball.position, tb.position, 1e-6, "position equals the cold-start twin")
	check_vec3_near(ball.linear_velocity, tb.linear_velocity, 1e-6, "velocity equals the cold-start twin")
	check_vec3_near(ball.angular_velocity, tb.angular_velocity, 1e-6, "angular velocity equals the cold-start twin")
	check(_rows_are_finite(contact), "impulses finite after recovery")


func test_c3_successful_steps_keep_warm_starting() -> void:
	# The reset runs only on abort: a normal run must carry impulses from step to step exactly as before.
	var s: Dictionary = _pendulum()
	var w: SimWorld = s.world
	var joint = s.joint
	for i in 10:
		check(w.step(), "step %d" % (i + 1))
	var carried: Array = _impulses(joint)
	var biggest: float = 0.0
	for v in carried:
		biggest = maxf(biggest, absf(v))
	check(biggest > 1e-3 and not _rows_are_cold(joint), "impulses are carried between successful steps (max |lambda| = %s)" % biggest)
	check(joint.impulses_at_finish == carried, "carried impulses are exactly the solver's result of the last step")
