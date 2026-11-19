extends "res://Tests/sim_test_case.gd"
## M1.1.1: a constraint may only be registered and simulated while both of its bodies are registered members of the
## same SimWorld (object identity). Checked at add_constraint(), at preflight() every step, and kept true by remove_body().
## The invariant is generic: every test that matters runs for a ball joint, a hinge joint and a bare SimConstraint.
##
## Tolerances: nothing is integrated in the refusal tests, so state is compared for exact equality. The free-fall check
## uses the exact semi-implicit Euler solution (v_n = -g n dt, y_n = y0 - g dt^2 n(n+1)/2); float32 positions of
## magnitude ~10 accumulate ~1e-6 error over 60 steps, so 1e-4 m is a safe bound, far below the sideways/upward
## displacement an active pivot constraint would cause on the bob (tens of centimetres, see the test below).

const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")

const KINDS: Array = ["ball", "hinge", "base"]
const DT: float = 1.0 / 120.0


## Joint of the given kind between a (near x=0) and b (near x=1), anchored midway. Starts satisfied.
func _make_constraint(kind: String, a: SimRigidBody, b: SimRigidBody) -> SimConstraint:
	var c: SimConstraint = null
	if kind == "ball":
		c = BallJoint.new(a, b, Vector3(0.5, 0, 0), Vector3(-0.5, 0, 0))
	elif kind == "hinge":
		c = HingeJoint.from_world(a, b, Vector3(0.5, 0, 0), Vector3.BACK, Vector3.RIGHT)
	else:
		c = SimConstraint.new(a, b)
	check(c != null, "%s: constraint constructed" % kind)
	return c


func _dyn(x: float) -> SimRigidBody:
	return make_sphere(1.0, 0.3, Vector3(x, 5, 0))


func _snapshot(w: SimWorld, list: Array) -> Dictionary:
	var d: Dictionary = {
		"step_count": w.step_count, "simulation_time": w.simulation_time, "accumulator": w.accumulator,
		"dropped_time": w.dropped_time,
	}
	for i in list.size():
		var b: SimRigidBody = list[i]
		d["pos%d" % i] = b.position
		d["orient%d" % i] = b.orientation
		d["vel%d" % i] = b.linear_velocity
		d["omega%d" % i] = b.angular_velocity
		d["force%d" % i] = b.force_accumulator
		d["torque%d" % i] = b.torque_accumulator
		d["pub%d" % i] = b.published_state.position
		d["prev%d" % i] = b.previous_state.position
	return d


func _check_untouched(w: SimWorld, list: Array, before: Dictionary, label: String) -> void:
	var after: Dictionary = _snapshot(w, list)
	for key in before:
		check(before[key] == after[key], "%s: '%s' unchanged" % [label, key])


# --- Constraint registration ---

func test_add_accepts_constraint_between_registered_bodies() -> void:
	for kind in KINDS:
		var w: SimWorld = make_world()
		var a: SimRigidBody = w.add_body(_dyn(0.0))
		var b: SimRigidBody = w.add_body(_dyn(1.0))
		var c: SimConstraint = _make_constraint(kind, a, b)
		check(w.add_constraint(c) == c, "%s: dynamic-dynamic constraint accepted and returned" % kind)
		check(w.constraints.size() == 1 and w.constraints[0] == c, "%s: stored exactly once" % kind)
		check(w.preflight(), "%s: world with the constraint is runnable" % kind)
		# Static + dynamic (the existing valid use case) in both argument orders.
		var w2: SimWorld = make_world()
		var s: SimRigidBody = w2.add_body(make_static(Vector3(0, 5, 0)))
		var d: SimRigidBody = w2.add_body(_dyn(1.0))
		check(w2.add_constraint(_make_constraint(kind, s, d)) != null, "%s: static(A)-dynamic(B) accepted" % kind)
		check(w2.add_constraint(_make_constraint(kind, d, s)) != null, "%s: dynamic(A)-static(B) accepted" % kind)
		check(w2.constraints.size() == 2, "%s: both stored" % kind)


func test_add_rejects_body_a_registered_body_b_absent() -> void:
	for kind in KINDS:
		var w: SimWorld = make_world()
		var a: SimRigidBody = w.add_body(_dyn(0.0))
		var outsider: SimRigidBody = _dyn(1.0)
		check(w.add_constraint(_make_constraint(kind, a, outsider)) == null, "%s: body B absent -> rejected (null)" % kind)
		check(w.constraints.is_empty(), "%s: nothing stored" % kind)
		check(w.preflight(), "%s: world still runnable" % kind)


func test_add_rejects_body_b_registered_body_a_absent() -> void:
	for kind in KINDS:
		var w: SimWorld = make_world()
		var b: SimRigidBody = w.add_body(_dyn(1.0))
		var outsider: SimRigidBody = _dyn(0.0)
		check(w.add_constraint(_make_constraint(kind, outsider, b)) == null, "%s: body A absent -> rejected (null)" % kind)
		check(w.constraints.is_empty(), "%s: nothing stored" % kind)


func test_add_rejects_neither_body_registered() -> void:
	for kind in KINDS:
		var w: SimWorld = make_world()
		w.add_body(_dyn(5.0))  # an unrelated member must not satisfy the check
		check(w.add_constraint(_make_constraint(kind, _dyn(0.0), _dyn(1.0))) == null, "%s: neither registered -> rejected" % kind)
		check(w.constraints.is_empty(), "%s: nothing stored" % kind)


func test_add_rejects_null_constraint_and_null_bodies() -> void:
	var w: SimWorld = make_world()
	var a: SimRigidBody = w.add_body(_dyn(0.0))
	check(w.add_constraint(null) == null, "null constraint rejected")
	check(w.add_constraint(SimConstraint.new(a, null)) == null, "null body B rejected")
	check(w.add_constraint(SimConstraint.new(null, a)) == null, "null body A rejected")
	check(w.add_constraint(SimConstraint.new(null, null)) == null, "both bodies null rejected")
	check(w.constraints.is_empty(), "nothing stored")


func test_membership_is_object_identity_not_id() -> void:
	# Two worlds hand out the same body ids; a body of world 1 is NOT a member of world 2 even though its id (and
	# every property) matches a registered body there.
	var w1: SimWorld = make_world()
	var w2: SimWorld = make_world()
	var a1: SimRigidBody = w1.add_body(_dyn(0.0))
	var b1: SimRigidBody = w1.add_body(_dyn(1.0))
	var a2: SimRigidBody = w2.add_body(_dyn(0.0))
	var b2: SimRigidBody = w2.add_body(_dyn(1.0))
	check(a1.id == a2.id and b1.id == b2.id, "precondition: ids collide across worlds")
	for kind in KINDS:
		check(w2.add_constraint(_make_constraint(kind, a1, b1)) == null, "%s: bodies of another world rejected" % kind)
		check(w2.add_constraint(_make_constraint(kind, a1, b2)) == null, "%s: one foreign body rejected" % kind)
		check(w2.add_constraint(_make_constraint(kind, a2, b1)) == null, "%s: one foreign body (B) rejected" % kind)
		check(w2.add_constraint(_make_constraint(kind, a2, b2)) != null, "%s: own bodies accepted" % kind)
		w2.constraints.clear()
	check(not w2.has_body(a1) and w2.has_body(a2) and not w2.has_body(null), "has_body uses identity and tolerates null")


# --- Runtime preflight ---

func test_preflight_passes_for_valid_registered_constraints() -> void:
	for kind in KINDS:
		var w: SimWorld = make_world(Vector3(0, -9.81, 0))
		var a: SimRigidBody = w.add_body(_dyn(0.0))
		var b: SimRigidBody = w.add_body(_dyn(1.0))
		w.add_constraint(_make_constraint(kind, a, b))
		check(w.preflight(), "%s: preflight passes" % kind)
		check(w.last_error == "", "%s: no error recorded" % kind)
		check(w.step(), "%s: step runs" % kind)


func test_preflight_rejects_body_removed_after_registration() -> void:
	for kind in KINDS:
		for removed_is_a in [true, false]:
			var w: SimWorld = make_world()
			var a: SimRigidBody = w.add_body(_dyn(0.0))
			var b: SimRigidBody = w.add_body(_dyn(1.0))
			w.add_constraint(_make_constraint(kind, a, b))
			# Bypass remove_body(): public collections can be edited directly, preflight must still catch it.
			w.bodies.erase(a if removed_is_a else b)
			var label: String = "%s/%s" % [kind, "A removed" if removed_is_a else "B removed"]
			check(not w.preflight(), "%s: preflight fails" % label)
			check(w.last_error.contains("membership") and w.last_error.contains("constraint 0"),
				"%s: last_error explains the membership violation (got '%s')" % [label, w.last_error])
			check(w.last_error.contains("body_a" if removed_is_a else "body_b"), "%s: names the missing body (got '%s')" % [label, w.last_error])


func test_preflight_rejects_null_body_set_after_registration() -> void:
	for kind in KINDS:
		var w: SimWorld = make_world()
		var c: SimConstraint = _make_constraint(kind, w.add_body(_dyn(0.0)), w.add_body(_dyn(1.0)))
		w.add_constraint(c)
		c.body_b = null
		check(not w.preflight(), "%s: null body fails preflight" % kind)
		check(w.last_error.contains("membership"), "%s: reported as membership violation" % kind)
		check(w.step() == false, "%s: and step() refuses without a script error" % kind)


func test_preflight_rejects_disabled_constraint_with_absent_body() -> void:
	# `enabled` only controls solving; a stale reference is still an inconsistent world.
	var w: SimWorld = make_world()
	var c: SimConstraint = _make_constraint("ball", w.add_body(_dyn(0.0)), w.add_body(_dyn(1.0)))
	w.add_constraint(c)
	c.enabled = false
	w.bodies.erase(c.body_b)
	check(not w.preflight(), "disabled constraint with an absent body still fails preflight")


func test_step_refuses_when_membership_violated_and_preserves_state() -> void:
	for kind in KINDS:
		var w: SimWorld = make_world(Vector3(0, -9.81, 0))
		var a: SimRigidBody = w.add_body(_dyn(0.0))
		var b: SimRigidBody = w.add_body(_dyn(1.0))
		var other: SimRigidBody = w.add_body(_dyn(4.0))
		w.add_constraint(_make_constraint(kind, a, b))
		a.linear_velocity = Vector3(1, 2, 3)
		a.angular_velocity = Vector3(0.1, 0.2, 0.3)
		b.linear_velocity = Vector3(-1, 0.5, 0)
		other.linear_velocity = Vector3(0, 0, 1)
		a.apply_force(Vector3(4, 5, 6))
		w.accumulator = 0.25 * DT
		w.bodies.erase(b)
		var list: Array = [a, b, other]
		var before: Dictionary = _snapshot(w, list)
		var rejected_before: int = w.rejected_step_count
		check(w.step() == false, "%s: step() returns false" % kind)
		_check_untouched(w, list, before, "%s step()" % kind)
		check(w.rejected_step_count == rejected_before + 1, "%s: refusal counted" % kind)
		check(w.last_error.contains("membership"), "%s: last_error set" % kind)
		# Recovery: registering the body again makes the world runnable and stepping resumes.
		w.bodies.append(b)
		check(w.step(), "%s: stepping resumes once the invariant holds again" % kind)
		check(w.step_count == 1 and w.last_error == "", "%s: one step executed, error cleared" % kind)


func test_advance_refuses_when_membership_violated_and_preserves_state() -> void:
	for kind in KINDS:
		var w: SimWorld = make_world(Vector3(0, -9.81, 0))
		var a: SimRigidBody = w.add_body(_dyn(0.0))
		var b: SimRigidBody = w.add_body(_dyn(1.0))
		w.add_constraint(_make_constraint(kind, a, b))
		a.linear_velocity = Vector3(1, 2, 3)
		b.angular_velocity = Vector3(0.1, 0.2, 0.3)
		w.accumulator = 0.4 * DT
		w.bodies.erase(a)
		var list: Array = [a, b]
		var before: Dictionary = _snapshot(w, list)
		check(w.advance(10.0 * DT) == 0, "%s: advance() executes zero steps" % kind)
		_check_untouched(w, list, before, "%s advance()" % kind)
		check(w.last_error.contains("membership"), "%s: last_error set" % kind)
		check(w.rejected_step_count == 1, "%s: refusal counted" % kind)


# --- Body removal (policy: remove_body() also removes every constraint referencing the body) ---

func test_remove_body_removes_its_constraints_either_side() -> void:
	for kind in KINDS:
		var w: SimWorld = make_world()
		var a: SimRigidBody = w.add_body(_dyn(0.0))
		var b: SimRigidBody = w.add_body(_dyn(1.0))
		var c: SimRigidBody = w.add_body(_dyn(2.0))
		var ab: SimConstraint = w.add_constraint(_make_constraint(kind, a, b))  # b referenced as body_b
		var bc: SimConstraint = w.add_constraint(_make_constraint(kind, b, c))  # b referenced as body_a
		var ca: SimConstraint = w.add_constraint(_make_constraint(kind, c, a))  # unrelated to b
		w.remove_body(b)
		check(not w.has_body(b), "%s: body removed" % kind)
		check(w.bodies.size() == 2 and w.bodies[0] == a and w.bodies[1] == c, "%s: other bodies kept in order" % kind)
		check(w.constraints.size() == 1 and w.constraints[0] == ca, "%s: only the unrelated constraint remains" % kind)
		check(not w.constraints.has(ab) and not w.constraints.has(bc), "%s: no dangling constraint" % kind)
		check(w.preflight() and w.last_error == "", "%s: world is runnable after removal" % kind)
		check(w.constraints[0].body_a == c and w.constraints[0].body_b == a, "%s: surviving constraint still references members" % kind)


## Chain static - l1 - l2 - l3 of ball joints (swinging under gravity from horizontal). With `extra`, a fourth body l4
## joined to l3 by a hinge is also built. Returns {world, bodies (static, l1, l2, l3), joints (3), l4, hinge}.
func _chain(extra: bool) -> Dictionary:
	var w: SimWorld = make_world(Vector3(0, -9.81, 0))
	var inertia: Basis = SimMassProperties.box_inertia(1.0, Vector3(1, 0.1, 0.1))
	var s: SimRigidBody = w.add_body(make_static(Vector3(0, 5, 0)))
	var l1: SimRigidBody = w.add_body(make_body(1.0, inertia, Vector3(0.5, 5, 0)))
	var l2: SimRigidBody = w.add_body(make_body(1.0, inertia, Vector3(1.5, 5, 0)))
	var l3: SimRigidBody = w.add_body(make_body(1.0, inertia, Vector3(2.5, 5, 0)))
	var l4: SimRigidBody = null
	var hinge: SimConstraint = null
	if extra:
		l4 = w.add_body(make_body(1.0, inertia, Vector3(3.5, 5, 0)))
	var joints: Array = [
		w.add_constraint(BallJoint.new(s, l1, Vector3.ZERO, Vector3(-0.5, 0, 0))),
		w.add_constraint(BallJoint.new(l1, l2, Vector3(0.5, 0, 0), Vector3(-0.5, 0, 0))),
		w.add_constraint(BallJoint.new(l2, l3, Vector3(0.5, 0, 0), Vector3(-0.5, 0, 0))),
	]
	if extra:
		hinge = w.add_constraint(HingeJoint.from_world(l3, l4, Vector3(3.0, 5, 0), Vector3.BACK, Vector3.RIGHT))
	return {"world": w, "bodies": [s, l1, l2, l3], "joints": joints, "l4": l4, "hinge": hinge}


func test_remove_body_keeps_unrelated_bodies_and_constraints_valid() -> void:
	# Removing l4 (and with it the hinge) must leave everything else exactly as if l4 had never existed: the reference
	# world is built without l4 and both are stepped identically (deterministic, so no tuned tolerance is needed).
	var ref: Dictionary = _chain(false)
	var tst: Dictionary = _chain(true)
	var w: SimWorld = tst.world
	var joints: Array = tst.joints
	check(tst.hinge != null and not joints.has(null), "chain with hinge built")
	w.remove_body(tst.l4)
	check(w.constraints.size() == 3 and w.constraints[0] == joints[0] and w.constraints[1] == joints[1] and w.constraints[2] == joints[2],
		"hinge removed, the three ball joints kept in registration order")
	check(w.bodies.size() == 4, "four bodies remain")
	for body in tst.bodies:
		check(w.has_body(body), "unrelated body still registered")
	check(w.preflight(), "preflight passes")
	for i in 60:
		check(w.step() and ref.world.step(), "step %d runs in both worlds" % i)
	for i in 4:
		check_vec3_near(tst.bodies[i].position, ref.bodies[i].position, 1.0e-6, "body %d follows the reference trajectory" % i)
	check(tst.bodies[3].position.y < 5.0 - 0.01, "the chain really swung (the comparison is not trivially static)")
	check(tst.l4.position == Vector3(3.5, 5, 0) and tst.l4.linear_velocity == Vector3.ZERO, "removed body is not simulated")


func test_no_stale_constraint_reaches_the_solver_after_removal() -> void:
	for kind in KINDS:
		# Static pivot + bob hanging 1 m sideways: an active joint would swing the bob (it moves by centimetres in 60
		# steps). After the pivot is removed the bob must instead be in exact free fall.
		var w: SimWorld = make_world(Vector3(0, -9.81, 0))
		var pivot: SimRigidBody = w.add_body(make_static(Vector3(0, 10, 0)))
		var bob: SimRigidBody = w.add_body(make_sphere(1.0, 0.1, Vector3(1, 10, 0)))
		var c: SimConstraint = w.add_constraint(_make_constraint_at(kind, pivot, bob))
		check(c != null, "%s: constraint registered" % kind)
		w.remove_body(pivot)
		check(w.constraints.is_empty(), "%s: removal dropped the constraint" % kind)
		check(w.preflight(), "%s: runnable" % kind)
		var n: int = 60
		for i in n:
			w.step()
		check(w.step_count == n and w.rejected_step_count == 0, "%s: all steps ran, none refused" % kind)
		var expected_y: float = 10.0 - 9.81 * DT * DT * n * (n + 1) / 2.0
		check_near(bob.position.y, expected_y, 1.0e-4, "%s: bob in exact free fall (y)" % kind)
		check_near(bob.position.x, 1.0, 1.0e-6, "%s: bob not pulled sideways (x)" % kind)
		check_near(bob.linear_velocity.y, -9.81 * DT * n, 1.0e-4, "%s: free-fall velocity" % kind)


## Joint of `kind` between a static pivot at (0,10,0) and a body at (1,10,0), anchored at the pivot.
func _make_constraint_at(kind: String, a: SimRigidBody, b: SimRigidBody) -> SimConstraint:
	var c: SimConstraint = null
	if kind == "ball":
		c = BallJoint.new(a, b, Vector3.ZERO, Vector3(-1, 0, 0))
	elif kind == "hinge":
		c = HingeJoint.from_world(a, b, Vector3(0, 10, 0), Vector3.BACK, Vector3.RIGHT)
	else:
		c = SimConstraint.new(a, b)
	return c


func test_remove_body_that_is_not_a_member_changes_nothing() -> void:
	var w: SimWorld = make_world()
	var a: SimRigidBody = w.add_body(_dyn(0.0))
	var b: SimRigidBody = w.add_body(_dyn(1.0))
	var c: SimConstraint = w.add_constraint(_make_constraint("ball", a, b))
	w.remove_body(_dyn(0.0))
	w.remove_body(null)
	check(w.bodies.size() == 2 and w.constraints.size() == 1 and w.constraints[0] == c, "world unchanged by removing a non-member")
	check(w.preflight(), "still runnable")
	w.remove_body(a)
	w.remove_body(a)  # second removal is a no-op
	check(w.bodies.size() == 1 and w.constraints.is_empty(), "double removal is harmless")
