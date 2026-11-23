extends "res://Tests/sim_test_case.gd"
## M2.3 rear suspension in the six-body motorcycle assembly (Simulation/Motorcycle/, built from MotorcyclePrototype).
## The spring-damper's own physics is in test_rear_suspension.gd; here: integration, builder transaction, regression sensitivity.
##
## C assembly integration: topology unchanged, both suspensions registered and operational, momentum transfer, the response of
##   swingarm and chassis against an expectation derived from the CONFIGURED geometry, finite and deterministic, disabling/removing
## D builder transaction and configuration validation (validated before ANY mutation, rollback, retry, populated world)
## F regression sensitivity at assembly level (the real provider swapped for in-suite mutants)
##
## Methodology: world at 200 solver iterations as in the other motorcycle files (ITERATIONS) unless a criterion is position- or
## momentum-level (not iteration limited), which then uses BATTERY_ITERATIONS to keep the suite fast. Expectations come from the
## configured world geometry (MotorcycleConfig) with plain vector algebra, never from the provider's own query methods.

const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const MotorcycleAssembly = preload("res://Simulation/Motorcycle/motorcycle_assembly.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const RearSuspension = preload("res://Simulation/Motorcycle/rear_suspension.gd")
const FrontSuspension = preload("res://Simulation/Motorcycle/front_suspension.gd")
const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")
const PrismaticJoint = preload("res://Simulation/Solver/prismatic_joint_constraint.gd")
const RearTests = preload("res://Tests/test_rear_suspension.gd")

const DT: float = 1.0 / 120.0
const ITERATIONS: int = 200
const BATTERY_ITERATIONS: int = 100


class FailingWorld extends "res://Simulation/Core/sim_world.gd":
	var fail_body_call: int = -1
	var fail_constraint_call: int = -1
	var fail_provider_call: int = -1
	var body_calls: int = 0
	var constraint_calls: int = 0
	var provider_calls: int = 0

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

	func add_force_provider(provider):
		provider_calls += 1
		if provider_calls == fail_provider_call:
			return null
		return super(provider)


func _config() -> MotorcycleConfig:
	return MotorcyclePrototype.config()


## Prototype with the rear shock preloaded: rest length `fraction` of the reference length (0.9 = compressed by 10 % at the
## reference pose), stiffness 2000 N/m (a ~108 N force) so the response is a clear but moderate signal.
func _preloaded(fraction: float, k: float = 2000.0, c: float = 0.0) -> MotorcycleConfig:
	var cfg: MotorcycleConfig = _config()
	var ref: float = (cfg.rear_suspension.swingarm_attachment - cfg.rear_suspension.chassis_attachment).length()
	cfg.rear_suspension.rest_length = ref * fraction
	cfg.rear_suspension.stiffness = k
	cfg.rear_suspension.damping = c
	return cfg


func _build(gravity: Vector3 = Vector3.ZERO, config: MotorcycleConfig = null, iterations: int = ITERATIONS) -> MotorcycleAssembly:
	var w: SimWorld = make_world(gravity)
	w.config.solver_iterations = iterations
	var a: MotorcycleAssembly = MotorcycleBuilder.build(config if config != null else _config(), w)
	check(a != null, "the prototype assembly builds")
	return a


func _state(a: MotorcycleAssembly) -> Array:
	var out: Array = []
	for body in a.bodies():
		out.append([body.position, body.orientation, body.linear_velocity, body.angular_velocity])
	return out


## Independent expectation from the CONFIGURED geometry at the reference pose (bodies at their configured positions).
func _expected_pair(a: MotorcycleAssembly) -> Dictionary:
	var spec = a.config.rear_suspension
	var d: Vector3 = spec.swingarm_attachment - spec.chassis_attachment
	var l: float = d.length()
	var n: Vector3 = d / l
	var f: float = -spec.stiffness * (l - spec.rest_length)
	var fs: Vector3 = n * f
	return {"n": n, "l": l, "F": f, "Fs": fs, "Fc": -fs,
		"tau_s": (spec.swingarm_attachment - a.config.swingarm.position).cross(fs),
		"tau_c": (spec.chassis_attachment - a.config.chassis.position).cross(-fs)}


# =====================================================================================================================
# C. Assembly integration
# =====================================================================================================================

func test_c1_topology_is_unchanged_and_both_suspensions_are_registered() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	var w: SimWorld = a.world
	check(w.bodies.size() == 6 and a.bodies().size() == 6, "exactly six rigid bodies (no body for the shock): %d" % w.bodies.size())
	var ids: Dictionary = {}
	for body in w.bodies:
		ids[body.id] = true
		check(not body.is_static() and not body.body_name.containsn("shock") and not body.body_name.containsn("damper"), "'%s' is an independent dynamic body" % body.body_name)
	check(ids.size() == 6, "six distinct body ids")
	check(w.constraints.size() == 5, "exactly five constraints: four hinges and the prismatic fork joint (no joint for the shock): %d" % w.constraints.size())
	var hinge_count: int = 0
	var prismatic_count: int = 0
	for constraint in w.constraints:
		if constraint is HingeJoint:
			hinge_count += 1
		elif constraint is PrismaticJoint:
			prismatic_count += 1
	check(hinge_count == 4 and prismatic_count == 1, "four hinges, one prismatic (%d, %d)" % [hinge_count, prismatic_count])
	# The joint graph is exactly the M2.1 / M2.2 graph.
	var graph: Array = [
		[a.steering_joint, a.chassis, a.steering], [a.swingarm_joint, a.chassis, a.swingarm],
		[a.front_axle_joint, a.fork_slider, a.front_wheel], [a.rear_axle_joint, a.swingarm, a.rear_wheel],
		[a.fork_joint, a.steering, a.fork_slider]]
	for link in graph:
		check(link[0].body_a == link[1] and link[0].body_b == link[2], "joint pair %s - %s unchanged" % [link[1].body_name, link[2].body_name])
	check(w.constraints == a.constraints(), "constraints registered in the documented order")
	# Providers: front first (M2.2), rear second (M2.3), named references on the assembly.
	check(w.force_providers.size() == 2, "two force providers")
	check(a.force_providers() == [a.front_suspension, a.rear_suspension] and w.force_providers == a.force_providers(), "front then rear, same objects as the world holds")
	check(a.front_suspension is FrontSuspension and a.rear_suspension is RearSuspension, "types")
	check(a.rear_suspension.involved_bodies() == [a.chassis, a.swingarm], "the rear suspension references exactly the chassis and the swingarm")
	check(a.rear_suspension.chassis_body == a.chassis and a.rear_suspension.swingarm_body == a.swingarm, "named body references")
	check(a.front_suspension.involved_bodies() == [a.steering, a.fork_slider], "the front suspension still references steering and slider")
	check(a.rear_suspension.enabled and a.front_suspension.enabled, "both are enabled")
	# Configured parameters and geometry reach the provider.
	var spec = a.config.rear_suspension
	check(a.rear_suspension.stiffness == spec.stiffness and a.rear_suspension.damping == spec.damping and a.rear_suspension.rest_length == spec.rest_length, "k, c and l_0 as configured")
	check_vec3_near(a.rear_suspension.attachment_chassis_world(), spec.chassis_attachment, 2.0e-6, "chassis attachment at the configured world point")
	check_vec3_near(a.rear_suspension.attachment_swingarm_world(), spec.swingarm_attachment, 2.0e-6, "swingarm attachment at the configured world point")
	# The prototype is unloaded at the reference pose (rest length = reference length) and the shock is inclined.
	check_near(a.rear_suspension.extension(), 0.0, 2.0e-6, "prototype shock at its rest length at the reference pose")
	var n: Vector3 = a.rear_suspension.direction()
	check(absf(n.y) < 0.9 and absf(n.z) > 0.3, "the prototype shock is inclined (n = %s), not vertical" % n)
	check(w.preflight(), "preflight passes")


func test_c2_reference_pose_is_force_free_in_zero_gravity() -> void:
	# Both suspensions are unloaded at the reference pose (x = x_rest, l = l_0) and nothing else acts: the assembly must stay at rest.
	var a: MotorcycleAssembly = _build(Vector3.ZERO, null, BATTERY_ITERATIONS)
	if a == null:
		return
	var worst: float = 0.0
	for i in 120:
		a.world.step()
		for body in a.bodies():
			worst = maxf(worst, maxf(body.linear_velocity.length(), body.angular_velocity.length()))
	check(worst <= 1.0e-5, "no hidden force: the unloaded assembly stays at rest (largest velocity %s)" % worst)
	check(a.world.numerical_fault == "", "no fault")


func test_c3_the_shock_transfers_momentum_between_chassis_and_swingarm() -> void:
	var impulse: Vector3 = Vector3(0.0, -9.0, 3.0)
	var runs: Array = []
	for sprung in [false, true]:
		var cfg: MotorcycleConfig = _config()
		if not sprung:
			cfg.rear_suspension.stiffness = 0.0
			cfg.rear_suspension.damping = 0.0
		var a: MotorcycleAssembly = _build(Vector3.ZERO, cfg, BATTERY_ITERATIONS)
		if a == null:
			return
		a.swingarm.apply_impulse(impulse)
		var chassis_early: Vector3 = Vector3.ZERO
		var worst_total: float = 0.0
		for i in 120:
			a.world.step()
			if i == 5:
				chassis_early = a.chassis.linear_momentum()
			worst_total = maxf(worst_total, (a.world.total_linear_momentum() - impulse).length())
		check(a.world.numerical_fault == "", "no fault")
		# Total linear momentum is the injected impulse in both worlds (all forces internal; float32 random walk over 120 steps of a 100 kg+ system).
		check(worst_total <= 2.0e-3 * impulse.length(), "sprung=%s: total momentum stays the injected impulse (worst deviation %s of %s)" % [sprung, worst_total, impulse.length()])
		runs.append({"chassis_early": chassis_early, "chassis_end": a.chassis.linear_momentum(), "assembly": a})
	# The shock moved momentum into the chassis. The kick has J . n = +8.7 > 0 (n = configured chassis -> swingarm direction), so it
	# LENGTHENS the shock; the shock pulls the chassis along +n relative to the unsprung twin.
	var n: Vector3 = _expected_pair(runs[1].assembly).n
	var gained: Vector3 = runs[1].chassis_early - runs[0].chassis_early
	check(gained.length() > 0.05, "the chassis momentum after 6 steps differs from the unsprung twin by %s kg m/s: the shock transferred momentum" % gained.length())
	check(impulse.dot(n) > 0.0 and gained.dot(n) > 0.0, "and it points toward the swingarm (along +n): component %s" % gained.dot(n))
	# Provider-level equal and opposite check on the kicked assembly (nonzero velocities, damping active).
	var b: MotorcycleAssembly = _build(Vector3.ZERO, null, BATTERY_ITERATIONS)
	b.swingarm.apply_impulse(impulse)
	for i in 3:
		b.world.step()
	b.chassis.clear_accumulators()
	b.swingarm.clear_accumulators()
	b.rear_suspension.apply_forces(DT, b.world.config)
	check(b.chassis.force_accumulator + b.swingarm.force_accumulator == Vector3.ZERO and b.swingarm.force_accumulator.length() > 1.0, "equal and opposite non-zero forces on swingarm and chassis")
	var line: Vector3 = b.rear_suspension.attachment_swingarm_world() - b.rear_suspension.attachment_chassis_world()
	check(b.swingarm.force_accumulator.cross(line).length() <= 1.0e-3 * b.swingarm.force_accumulator.length(), "along the instantaneous attachment line of the moved assembly")


func test_c4_preloaded_shock_gives_the_expected_forces_torques_and_motion() -> void:
	var a: MotorcycleAssembly = _build(Vector3.ZERO, _preloaded(0.9), BATTERY_ITERATIONS)
	if a == null:
		return
	var e: Dictionary = _expected_pair(a)
	check_near(e.F, -2000.0 * 0.1 * e.l, 1.0e-3, "expectation: l_0 = 0.9 l so the shock is STRETCHED, F = -k (l - 0.9 l) = -0.1 k l")
	check(e.F < -50.0, "the stretched shock pulls the swingarm toward the chassis with %s N" % e.F)
	a.chassis.clear_accumulators()
	a.swingarm.clear_accumulators()
	a.rear_suspension.apply_forces(DT, a.world.config)
	check_vec3_near(a.swingarm.force_accumulator, e.Fs, 2.0e-5 * absf(e.F) + 1.0e-4, "swingarm force = +F n from the CONFIGURED geometry")
	check_vec3_near(a.chassis.force_accumulator, e.Fc, 2.0e-5 * absf(e.F) + 1.0e-4, "chassis force = -F n")
	check_vec3_near(a.swingarm.torque_accumulator, e.tau_s, 2.0e-5 * e.tau_s.length() + 1.0e-4, "swingarm torque from the attachment lever arm")
	check_vec3_near(a.chassis.torque_accumulator, e.tau_c, 2.0e-5 * e.tau_c.length() + 1.0e-4, "chassis torque from the attachment lever arm")
	check(e.tau_s.length() > 1.0 and e.tau_c.length() > 1.0, "both torques are substantial (%s, %s N m)" % [e.tau_s.length(), e.tau_c.length()])
	# Motion: the stretched shock pulls the swingarm toward the chassis; the swingarm hinge axis is +X, positive angle = rear end DOWN.
	# Expected sign of the swingarm's rotation about the pivot from the configured geometry: tau_x of F_s about the pivot point.
	var pivot: Vector3 = a.config.swingarm_pivot.point
	var tau_pivot_x: float = (a.config.rear_suspension.swingarm_attachment - pivot).cross(e.Fs).x
	check(absf(tau_pivot_x) > 5.0, "the shock loads the swingarm about its pivot (%s N m)" % tau_pivot_x)
	var angle_early: float = 0.0
	for i in 12:
		a.world.step()
	angle_early = a.swingarm_joint.angle()
	check(angle_early * tau_pivot_x > 0.0, "the swingarm rotates the way the shock torque about the pivot points (angle %s rad, torque %s N m)" % [angle_early, tau_pivot_x])
	check(absf(angle_early) > 1.0e-4, "by a measurable amount")
	check(a.max_anchor_error() <= 2.0e-4 and a.max_axis_error() <= 2.0e-3, "the hinges stay attached under the shock load (%s m, %s rad)" % [a.max_anchor_error(), a.max_axis_error()])
	# The chassis gets the reaction: pulled toward the swingarm, i.e. along +n.
	check(a.chassis.linear_momentum().dot(e.n) > 0.0, "the chassis momentum points toward the swingarm (%s)" % a.chassis.linear_momentum().dot(e.n))
	check_vec3_near(a.world.total_linear_momentum(), Vector3.ZERO, 2.0e-3, "total momentum stays zero (internal forces only)")
	# A second sign check: rest length 1.1 l makes the shock COMPRESSED (pushes the swingarm away) and reverses everything.
	var b: MotorcycleAssembly = _build(Vector3.ZERO, _preloaded(1.1), BATTERY_ITERATIONS)
	var eb: Dictionary = _expected_pair(b)
	check(eb.F > 50.0 and eb.F * e.F < 0.0, "a compressed shock has the opposite force sign (%s N)" % eb.F)
	for i in 12:
		b.world.step()
	check(b.swingarm_joint.angle() * a.swingarm_joint.angle() < 0.0, "and the swingarm rotates the opposite way")


func test_c5_front_suspension_stays_operational_next_to_the_rear() -> void:
	var a: MotorcycleAssembly = _build(Vector3.ZERO, null, BATTERY_ITERATIONS)
	if a == null:
		return
	var u: Vector3 = a.config.fork_travel.axis
	var kick: float = 15.0
	a.fork_slider.apply_impulse(u * -kick)   # compression
	var lowest: float = 0.0
	var returned: bool = false
	for i in 240:
		a.world.step()
		var s: float = a.fork_joint.translation()
		lowest = minf(lowest, s)
		if lowest < -0.004 and s > lowest + 0.002:
			returned = true
	check(lowest < -0.004, "the fork compresses under the kick (lowest %s m)" % lowest)
	check(lowest >= a.config.front_suspension.lower_travel_limit - 1.0e-3, "inside its travel limit")
	check(returned, "the front spring pushes it back (rebound)")
	check(absf(a.front_suspension.axial_force()) < 4000.0 and is_finite(a.front_suspension.axial_force()), "front suspension force is finite and moderate")
	check(a.world.total_linear_momentum().length() > 0.5 and (a.world.total_linear_momentum() - u * -kick).length() <= 2.0e-3 * kick, "total momentum is the injected impulse (internal suspension forces only)")
	check(a.world.numerical_fault == "", "no fault")


func test_c6_complete_motorcycle_is_finite_and_deterministic_under_suspension_excitation() -> void:
	var finals: Array = []
	for n in 2:
		var a: MotorcycleAssembly = _build(Vector3(0.0, -9.81, 0.0), null, ITERATIONS)
		if a == null:
			return
		a.swingarm.apply_impulse(Vector3(0.0, -45.0, 14.0))
		a.fork_slider.apply_impulse(a.config.fork_travel.axis * -10.0)
		a.rear_wheel.apply_angular_impulse(Vector3.RIGHT * 3.0)
		a.front_wheel.apply_angular_impulse(Vector3.RIGHT * 3.0)
		var finite: bool = true
		var worst_anchor: float = 0.0
		var worst_axis: float = 0.0
		var peak_ext: float = 0.0
		for i in 300:
			a.world.step()
			peak_ext = maxf(peak_ext, absf(a.rear_suspension.extension()))
			worst_anchor = maxf(worst_anchor, a.max_anchor_error())
			worst_axis = maxf(worst_axis, a.max_axis_error())
			for body in a.bodies():
				finite = finite and body.position.is_finite() and body.linear_velocity.is_finite() and body.angular_velocity.is_finite() and body.orientation.is_finite()
		check(finite, "run %d: all state finite" % n)
		check(a.world.numerical_fault == "", "run %d: no numerical fault" % n)
		check(absf(a.rear_suspension.length() - a.rear_suspension.rest_length) < 0.5 * a.rear_suspension.rest_length, "run %d: the shock length stays physical (%s m)" % [n, a.rear_suspension.length()])
		check(peak_ext > 0.01, "run %d: the rear shock was really excited (peak extension %s m)" % [n, peak_ext])
		check(worst_anchor <= 5.0e-3 and worst_axis <= 5.0e-2, "run %d: joints stay assembled (worst anchor %s m, axis %s rad)" % [n, worst_anchor, worst_axis])
		finals.append(_state(a))
	check(finals[0] == finals[1], "two identical runs are bit-identical")


func test_c7_disabling_or_removing_the_rear_provider_equals_a_zero_force_shock() -> void:
	var results: Dictionary = {}
	for mode in ["zero", "disabled", "removed", "active"]:
		var cfg: MotorcycleConfig = _config()
		if mode == "zero":
			cfg.rear_suspension.stiffness = 0.0
			cfg.rear_suspension.damping = 0.0
		var a: MotorcycleAssembly = _build(Vector3(0.0, -9.81, 0.0), cfg, BATTERY_ITERATIONS)
		if a == null:
			return
		if mode == "disabled":
			a.rear_suspension.enabled = false
		elif mode == "removed":
			a.world.remove_force_provider(a.rear_suspension)
			check(a.world.force_providers == [a.front_suspension], "only the front provider is left")
		a.swingarm.apply_impulse(Vector3(0.0, -14.0, 4.0))
		a.rear_wheel.apply_angular_impulse(Vector3.RIGHT * 3.0)
		for i in 150:
			a.world.step()
		results[mode] = _state(a)
		check(a.world.numerical_fault == "", "%s: no fault" % mode)
	check(results["disabled"] == results["zero"], "a disabled rear provider is bit-identical to a k = c = 0 shock (it applies exactly nothing)")
	check(results["removed"] == results["zero"], "a removed rear provider is bit-identical to a k = c = 0 shock")
	var sw_active: Vector3 = results["active"][3][0]
	var sw_zero: Vector3 = results["zero"][3][0]
	check((sw_active - sw_zero).length() > 5.0e-3, "the active shock changes the swingarm position by %s m: it has a real effect" % (sw_active - sw_zero).length())


# =====================================================================================================================
# D. Builder transaction and configuration validation
# =====================================================================================================================

func test_d1_invalid_rear_configurations_are_rejected_before_any_world_mutation() -> void:
	var cases: Array = [
		["negative stiffness", func(c): c.rear_suspension.stiffness = -1.0, "stiffness"],
		["NaN stiffness", func(c): c.rear_suspension.stiffness = NAN, "stiffness"],
		["infinite stiffness", func(c): c.rear_suspension.stiffness = INF, "stiffness"],
		["negative damping", func(c): c.rear_suspension.damping = -5.0, "damping"],
		["NaN damping", func(c): c.rear_suspension.damping = NAN, "damping"],
		["infinite damping", func(c): c.rear_suspension.damping = INF, "damping"],
		["zero rest length", func(c): c.rear_suspension.rest_length = 0.0, "rest length"],
		["negative rest length", func(c): c.rear_suspension.rest_length = -0.2, "rest length"],
		["NaN rest length", func(c): c.rear_suspension.rest_length = NAN, "rest length"],
		["infinite rest length", func(c): c.rear_suspension.rest_length = INF, "rest length"],
		["NaN chassis attachment", func(c): c.rear_suspension.chassis_attachment = Vector3(NAN, 0.8, 0.2), "attachment"],
		["infinite swingarm attachment", func(c): c.rear_suspension.swingarm_attachment = Vector3(0.0, INF, 0.5), "attachment"],
		["coincident attachments", func(c): c.rear_suspension.swingarm_attachment = c.rear_suspension.chassis_attachment, "apart"],
		["attachments 0.5 mm apart", func(c): c.rear_suspension.swingarm_attachment = c.rear_suspension.chassis_attachment + Vector3(0.0, 0.0, 0.0005), "apart"],
		["null rear suspension spec", func(c): c.rear_suspension = null, "rear suspension"],
	]
	var w: FailingWorld = FailingWorld.new()
	w.config.gravity = Vector3.ZERO
	var existing_a: SimRigidBody = w.add_body(make_sphere(5.0, 0.2, Vector3(5.0, 1.0, 0.0)))
	var existing_b: SimRigidBody = w.add_body(make_sphere(3.0, 0.2, Vector3(5.0, 0.4, 0.4)))
	var unrelated = RearSuspension.from_world(existing_a, existing_b, existing_a.position, existing_b.position, 800.0, 10.0, 0.7)
	w.add_force_provider(unrelated)
	var joint = BallJoint.new(existing_a, existing_b, Vector3(0.0, -0.3, 0.2), Vector3(0.0, 0.3, -0.2))
	w.add_constraint(joint)
	w.body_calls = 0
	w.constraint_calls = 0
	w.provider_calls = 0
	var bodies_before: Array = w.bodies.duplicate()
	var constraints_before: Array = w.constraints.duplicate()
	var providers_before: Array = w.force_providers.duplicate()
	for case in cases:
		var cfg: MotorcycleConfig = _config()
		case[1].call(cfg)
		var errors: PackedStringArray = cfg.validate()
		check(not errors.is_empty() and " ".join(errors).containsn(case[2]), "'%s' is reported by validate() (%s)" % [case[0], "; ".join(errors)])
		check(MotorcycleBuilder.build(cfg, w) == null, "'%s': build returns null" % case[0])
		check(w.body_calls == 0 and w.constraint_calls == 0 and w.provider_calls == 0, "'%s': the world was never touched (calls %d/%d/%d): validated before mutation" % [case[0], w.body_calls, w.constraint_calls, w.provider_calls])
		check(w.bodies == bodies_before and w.constraints == constraints_before and w.force_providers == providers_before, "'%s': bodies, constraints and providers are exactly as before" % case[0])
	var good: MotorcycleAssembly = MotorcycleBuilder.build(_config(), w)
	check(good != null and w.bodies.size() == 8 and w.constraints.size() == 6 and w.force_providers.size() == 3, "after the rejected attempts a valid build succeeds")
	# Boundary: the shortest accepted separation and zero k / c build.
	var edge: MotorcycleConfig = _config()
	edge.rear_suspension.swingarm_attachment = edge.rear_suspension.chassis_attachment + Vector3(0.0, 0.0, 0.002)
	edge.rear_suspension.rest_length = 0.002
	edge.rear_suspension.stiffness = 0.0
	edge.rear_suspension.damping = 0.0
	check(edge.validate().is_empty() and MotorcycleBuilder.build(edge, FailingWorld.new()) != null, "a 2 mm separation with k = c = 0 is valid and builds")


func test_d2_failure_at_rear_provider_registration_rolls_everything_back_and_a_retry_succeeds() -> void:
	# The existing strategy (FailingWorld refusing the n-th registration) extended to the rear provider (#2) in a populated world.
	var stages: Array = []
	for n in range(1, 7):
		stages.append([n, -1, -1, "add_body #%d" % n])
	for n in range(1, 6):
		stages.append([-1, n, -1, "add_constraint #%d" % n])
	stages.append([-1, -1, 1, "add_force_provider #1 (front)"])
	stages.append([-1, -1, 2, "add_force_provider #2 (rear)"])
	for st in stages:
		var w: FailingWorld = FailingWorld.new()
		w.config.gravity = Vector3.ZERO
		var a0: SimRigidBody = w.add_body(make_sphere(5.0, 0.2, Vector3(5.0, 1.0, 0.0)))
		var b0: SimRigidBody = w.add_body(make_sphere(3.0, 0.2, Vector3(5.0, 0.4, 0.4)))
		var unrelated = RearSuspension.from_world(a0, b0, a0.position, b0.position, 800.0, 10.0, 0.7)
		w.add_force_provider(unrelated)
		w.add_constraint(BallJoint.new(a0, b0, Vector3(0.0, -0.3, 0.2), Vector3(0.0, 0.3, -0.2)))
		w.body_calls = 0
		w.constraint_calls = 0
		w.provider_calls = 0
		var bodies_before: Array = w.bodies.duplicate()
		var constraints_before: Array = w.constraints.duplicate()
		var providers_before: Array = w.force_providers.duplicate()
		w.fail_body_call = st[0]
		w.fail_constraint_call = st[1]
		w.fail_provider_call = st[2]
		check(MotorcycleBuilder.build(_config(), w) == null, "%s refused: build returns null" % st[3])
		check(w.bodies == bodies_before and w.constraints == constraints_before and w.force_providers == providers_before,
			"%s refused: bodies, constraints and BOTH motorcycle providers fully rolled back, unrelated content intact (%d, %d, %d)" % [st[3], w.bodies.size(), w.constraints.size(), w.force_providers.size()])
		check(w.has_body(a0) and w.has_body(b0) and w.force_providers[0] == unrelated, "%s refused: the pre-existing content survives in order" % st[3])
		check(w.preflight(), "%s refused: the world is still valid and steppable" % st[3])
		w.fail_body_call = -1
		w.fail_constraint_call = -1
		w.fail_provider_call = -1
		var retry: MotorcycleAssembly = MotorcycleBuilder.build(_config(), w)
		check(retry != null and w.bodies.size() == 8 and w.constraints.size() == 6 and w.force_providers.size() == 3, "%s refused: a clean retry succeeds completely" % st[3])
		if retry != null:
			check(w.force_providers[1] == retry.front_suspension and w.force_providers[2] == retry.rear_suspension, "%s refused: the retried providers are registered in order" % st[3])
			check(w.step(), "%s refused: the retried world steps" % st[3])


func test_d3_unrelated_content_in_the_world_is_unaffected_and_the_motorcycle_does_not_depend_on_it() -> void:
	var states: Array = []
	for with_motorcycle in [false, true]:
		var w: SimWorld = make_world(Vector3(0.0, -9.81, 0.0))
		w.config.solver_iterations = BATTERY_ITERATIONS
		var a0: SimRigidBody = w.add_body(make_sphere(5.0, 0.2, Vector3(5.0, 1.0, 0.0)))
		var b0: SimRigidBody = w.add_body(make_sphere(3.0, 0.2, Vector3(5.0, 0.4, 0.4)))
		var unrelated = RearSuspension.from_world(a0, b0, a0.position, b0.position, 800.0, 10.0, 0.55)
		w.add_force_provider(unrelated)
		w.add_constraint(BallJoint.new(a0, b0, Vector3(0.0, -0.3, 0.2), Vector3(0.0, 0.3, -0.2)))
		if with_motorcycle:
			var m: MotorcycleAssembly = MotorcycleBuilder.build(_config(), w)
			check(m != null and w.bodies.size() == 8, "the motorcycle builds into the populated world")
			m.swingarm.apply_impulse(Vector3(0.0, -14.0, 4.0))
		for i in 120:
			w.step()
		states.append([a0.position, a0.linear_velocity, b0.position, b0.angular_velocity, a0.orientation])
		check(w.numerical_fault == "", "no fault")
	check(states[0] == states[1], "the unrelated pair behaves bit-identically with and without the motorcycle in the same world")


# =====================================================================================================================
# F. Regression sensitivity at assembly level (the real provider replaced by in-suite mutants of the real component)
# =====================================================================================================================
# Defects tested here (a subset of those of test_rear_suspension.gd, section E, now through the complete assembly): reversed spring
# direction, reversed damping sign, force applied to one body only. The battery's criteria use only the configured geometry and
# conserved/decaying quantities: (1) total linear momentum stays the injected impulse, (2) the swingarm turns the way the configured
# shock torque about the pivot points, (3) with the damper alone the shock's relative speed decays.

func _swap_rear(a: MotorcycleAssembly, script) -> void:
	var old = a.rear_suspension
	a.world.remove_force_provider(old)
	var m = script.new(old.chassis_body, old.swingarm_body, old.local_attachment_chassis, old.local_attachment_swingarm, old.stiffness, old.damping, old.rest_length)
	a.world.add_force_provider(m)
	a.rear_suspension = m


func _rear_violations(script) -> PackedStringArray:
	var out: PackedStringArray = PackedStringArray()
	# (1) + (2): preloaded shock from rest, zero gravity.
	var a: MotorcycleAssembly = _build(Vector3.ZERO, _preloaded(0.9), BATTERY_ITERATIONS)
	if script != null:
		_swap_rear(a, script)
	var e: Dictionary = _expected_pair(a)
	var tau_pivot_x: float = (a.config.rear_suspension.swingarm_attachment - a.config.swingarm_pivot.point).cross(e.Fs).x
	for i in 24:
		a.world.step()
	if a.world.total_linear_momentum().length() > 2.0e-3:
		out.append("momentum not conserved (%s)" % a.world.total_linear_momentum().length())
	if not (a.swingarm_joint.angle() * tau_pivot_x > 0.0 and absf(a.swingarm_joint.angle()) > 1.0e-4):
		out.append("swingarm turns against the shock torque (angle %s)" % a.swingarm_joint.angle())
	# (3): damper alone, the swingarm kicked: the shock's relative speed must decay.
	var d: MotorcycleAssembly = _build(Vector3.ZERO, _preloaded(1.0, 0.0, 1500.0), BATTERY_ITERATIONS)
	if script != null:
		_swap_rear(d, script)
	d.swingarm.apply_impulse(Vector3(0.0, -9.0, 3.0))
	d.world.step()
	var rate0: float = absf(d.rear_suspension.relative_extension_rate())
	for i in 40:
		d.world.step()
	var rate1: float = absf(d.rear_suspension.relative_extension_rate())
	if not (rate0 > 0.05 and rate1 < 0.7 * rate0):
		out.append("the damper does not remove the shock's relative speed (%s -> %s)" % [rate0, rate1])
	return out


func test_f1_assembly_battery_accepts_the_real_provider_and_an_unmodified_swap() -> void:
	var real: PackedStringArray = _rear_violations(null)
	check(real.is_empty(), "the real assembly passes every criterion (failed: %s)" % "; ".join(real))
	var control: PackedStringArray = _rear_violations(RearTests.Unmodified)
	check(control.is_empty(), "swapping in an unmodified subclass (control) passes every criterion (failed: %s)" % "; ".join(control))


func test_f2_assembly_battery_rejects_reversed_spring_reversed_damping_and_one_sided_force() -> void:
	for m in [["reversed spring", RearTests.ReversedSpring], ["reversed damping", RearTests.ReversedDamping], ["force on one body only", RearTests.OneBodyOnly]]:
		var failed: PackedStringArray = _rear_violations(m[1])
		check(not failed.is_empty(), "mutant '%s' is rejected (criteria failed: %s)" % [m[0], "; ".join(failed)])
