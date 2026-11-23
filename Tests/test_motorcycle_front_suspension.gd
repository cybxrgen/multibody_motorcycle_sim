extends "res://Tests/sim_test_case.gd"
## M2.2 integrated motorcycle tests: the six-body, five-joint assembly with the front telescopic suspension
## (Simulation/Motorcycle/, built by MotorcycleBuilder from MotorcyclePrototype; ILLUSTRATIVE values, not a real motorcycle).
##
## E integrated behaviour (graph, fork kinematics, steering, wheel spin, momentum, gravity, determinism, no hidden force, builder
##   transaction)
## F regression sensitivity: one battery of physical criteria that must accept the real assembly and reject each named defect
##   (missing prismatic constraint, tilted fork axis, a joint that permits rotation, reversed limit row, split correction applied to
##   physical velocity, reversed spring, energy-injecting damping, one-sided force, front axle on the upper steering assembly).
##
## Methodology notes:
## * Solver: 200 iterations (the converged setting of test_motorcycle_assembly.gd). Position-level errors are not iteration limited:
##   frozen-Jacobian second-order error per step divided by beta (PROJECT_STATE.md); bounds below state the value measured here.
## * The fork is judged against the CONFIGURED geometry (MotorcycleConfig world data at the reference pose) through test-owned local
##   frames (Rig.measure), never through the constraint's own cached values.
## * Zero gravity unless stated; the prototype spring is k = 12 kN/m, c = 400 N s/m, limits -0.12 / +0.03 m.
## * float32: pose tolerances >= 2e-6, momentum tolerances sized by the float32 random walk (TESTING.md).

const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const MotorcycleAssembly = preload("res://Simulation/Motorcycle/motorcycle_assembly.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const FrontSuspension = preload("res://Simulation/Motorcycle/front_suspension.gd")
const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")
const PrismaticJoint = preload("res://Simulation/Solver/prismatic_joint_constraint.gd")
const Rig = preload("res://Tests/fork_rig.gd")
const PJ = preload("res://Tests/test_prismatic_joint.gd")

const DT: float = 1.0 / 120.0
const ITERATIONS: int = 200
## The regression battery (section F) judges position-, energy- and momentum-level criteria, which are not iteration limited, so it
## runs at 100 iterations to keep the suite fast (the real assembly passes and every defect is still detected, see test_f1..f10).
const BATTERY_ITERATIONS: int = 100
const POSE_TOL: float = 2.0e-6


# --- Suspension mutants ---
class ReversedSpring extends "res://Simulation/Motorcycle/front_suspension.gd":
	func spring_force() -> float:
		return stiffness * displacement_from_rest()


class InjectingDamping extends "res://Simulation/Motorcycle/front_suspension.gd":
	func damping_force() -> float:
		return damping * relative_velocity()


class OneSidedForce extends "res://Simulation/Motorcycle/front_suspension.gd":
	func apply_forces(_dt, _config) -> void:
		joint.body_b.apply_force_at_point(axis() * axial_force(), attachment_b_world())


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


## M2.3: the rear spring-damper is zeroed here (k = c = 0: registered, exactly zero force) so these M2.2 tests keep isolating the
## front suspension and the M2.1 rear behaviour; the sprung rear coexisting with the front is in test_motorcycle_rear_suspension.gd.
func _config() -> MotorcycleConfig:
	var c: MotorcycleConfig = MotorcyclePrototype.config()
	c.rear_suspension.stiffness = 0.0
	c.rear_suspension.damping = 0.0
	return c


func _build(gravity: Vector3 = Vector3.ZERO, config: MotorcycleConfig = null, iterations: int = ITERATIONS) -> MotorcycleAssembly:
	var w: SimWorld = make_world(gravity)
	w.config.solver_iterations = iterations
	var a: MotorcycleAssembly = MotorcycleBuilder.build(config if config != null else _config(), w)
	check(a != null, "the prototype assembly builds")
	return a


## Test-owned frame data of the fork joint from the CONFIGURED geometry, in the dictionary layout of Rig.measure().
func _fork_frame(a: MotorcycleAssembly) -> Dictionary:
	var spec = a.config.fork_travel
	var ref: Vector3 = MotorcycleBuilder.hinge_reference(spec.axis)
	var s: SimRigidBody = a.steering
	var f: SimRigidBody = a.fork_slider
	return {"a": s, "b": f, "pa": s.orientation.inverse() * (spec.point - s.position), "pb": f.orientation.inverse() * (spec.point - f.position),
		"ua": s.orientation.inverse() * spec.axis, "ub": f.orientation.inverse() * spec.axis, "ra": s.orientation.inverse() * ref, "rb": f.orientation.inverse() * ref}


## Test-owned pivot data of the front axle hinge on the slider and on the wheel (configured front_axle point).
func _axle_frame(a: MotorcycleAssembly) -> Array:
	var p: Vector3 = a.config.front_axle.point
	return [a.fork_slider.orientation.inverse() * (p - a.fork_slider.position), a.front_wheel.orientation.inverse() * (p - a.front_wheel.position)]


func _axle_error(a: MotorcycleAssembly, frame: Array) -> float:
	return ((a.fork_slider.position + a.fork_slider.orientation * frame[0]) - (a.front_wheel.position + a.front_wheel.orientation * frame[1])).length()


func _lower_momentum(a: MotorcycleAssembly) -> Vector3:
	return a.fork_slider.linear_momentum() + a.front_wheel.linear_momentum()


# =====================================================================================================================
# E. Integrated behaviour
# =====================================================================================================================

func test_e1_six_bodies_five_joints_one_suspension_and_named_references() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	var w: SimWorld = a.world
	check(w.bodies.size() == 6 and a.bodies().size() == 6, "six bodies")
	check(w.constraints.size() == 5 and a.constraints().size() == 5, "five constraints")
	check(w.force_providers.size() == 2 and w.force_providers[0] == a.front_suspension and w.force_providers[1] == a.rear_suspension, "two registered force providers: the front suspension, then the rear suspension")
	var hinge_count: int = 0
	var prismatic_count: int = 0
	for c in w.constraints:
		if c.get_script() == HingeJoint:
			hinge_count += 1
		elif c.get_script() == PrismaticJoint:
			prismatic_count += 1
	check(hinge_count == 4 and prismatic_count == 1, "four generic hinges and one generic prismatic joint (%d, %d)" % [hinge_count, prismatic_count])
	var names: Array = ["chassis", "steering", "fork_slider", "swingarm", "front_wheel", "rear_wheel"]
	for i in 6:
		check(a.bodies()[i].body_name == names[i] and w.has_body(a.bodies()[i]) and not a.bodies()[i].is_static(), "body %d is the dynamic '%s'" % [i, names[i]])
	check(a.steering_joint.body_a == a.chassis and a.steering_joint.body_b == a.steering, "steering head: chassis <-> steering assembly")
	check(a.fork_joint.body_a == a.steering and a.fork_joint.body_b == a.fork_slider, "fork joint: steering assembly <-> fork slider")
	check(a.front_axle_joint.body_a == a.fork_slider and a.front_axle_joint.body_b == a.front_wheel, "front axle: fork slider <-> front wheel (NOT the upper steering assembly)")
	check(a.front_axle_joint.body_a != a.steering, "the front axle is not attached to the upper steering assembly")
	check(a.swingarm_joint.body_a == a.chassis and a.swingarm_joint.body_b == a.swingarm, "swingarm pivot: chassis <-> swingarm")
	check(a.rear_axle_joint.body_a == a.swingarm and a.rear_axle_joint.body_b == a.rear_wheel, "rear axle: swingarm <-> rear wheel")
	check(a.front_suspension.joint == a.fork_joint, "the suspension acts through the fork joint")
	# Independent mass properties, nothing merged.
	var specs: Array = a.config.body_specs()
	for i in 6:
		check(a.bodies()[i].mass == specs[i].mass and a.bodies()[i].local_inertia == specs[i].local_inertia, "body '%s' has its own configured mass and inertia" % names[i])
	check_near(a.fork_slider.mass + a.steering.mass, 22.0, 1.0e-6, "steering assembly + slider = the 22 kg fork of M2.1 (nothing lost)")
	check(a.fork_slider.mass > 0.0 and a.fork_slider.mass != a.front_wheel.mass, "the slider has its own mass, distinct from the wheel's")
	check(w.preflight(), "the world is valid")
	# Fork axis follows the rake: it is the steering axis (reversed), not world vertical.
	var u: Vector3 = a.fork_joint.world_axis_a()
	check_vec3_near(u, -a.config.steering_head.axis, POSE_TOL, "the fork axis is the (reversed) raked steering axis")
	check(absf(u.dot(Vector3.UP)) < 0.95 and absf(u.x) <= 1.0e-6, "it is raked, not vertical (|u.up| = %s)" % absf(u.dot(Vector3.UP)))
	check_vec3_near(a.fork_joint.world_axis_b(), u, POSE_TOL, "the slider's axis agrees")
	check_near(a.fork_joint.translation(), 0.0, POSE_TOL, "fork displacement 0 at the reference pose")
	check(MotorcycleBuilder.construction_error(a) == "", "the construction contract holds")


func test_e2_the_front_axle_follows_the_slider_not_the_upper_steering_assembly() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	var frame: Array = _axle_frame(a)
	var u: Vector3 = a.config.fork_travel.axis
	# Kinematic probe: translate slider + wheel 4 cm along the fork axis (a legal fork displacement).
	for body in [a.fork_slider, a.front_wheel]:
		body.position += u * 0.04
	check_near(_axle_error(a, frame), 0.0, POSE_TOL, "the front axle pivot still coincides on slider and wheel")
	check_near(a.front_axle_joint.linear_error().length(), 0.0, POSE_TOL, "the axle hinge reports no error")
	check_near(a.fork_joint.translation(), 0.04, POSE_TOL, "the fork coordinate is +4 cm (extension)")
	check(a.steering_joint.linear_error().length() <= POSE_TOL, "the steering head is unaffected")
	# Dynamic probe: an impulse on the slider alone moves the wheel with it (the axle holds) and does not move the steering assembly
	# by the same amount.
	var b: MotorcycleAssembly = _build()
	var frame_b: Array = _axle_frame(b)
	b.fork_slider.apply_impulse(b.config.fork_travel.axis * 4.0)
	for i in 30:
		b.world.step()
	check(_axle_error(b, frame_b) <= 2.0e-4, "the wheel stays on the slider's axle while the fork moves (error %s m)" % _axle_error(b, frame_b))
	check(b.front_wheel.linear_velocity.length() > 0.05, "the wheel follows the slider")
	check((b.front_wheel.position - b.config.front_wheel.position).length() > 0.5 * absf(b.fork_joint.translation()), "the wheel moved with the fork displacement")


func test_e3_steering_rotation_moves_the_fork_axis_and_slider_consistently() -> void:
	# Kinematic: rotate steering assembly, slider and front wheel rigidly about the steering axis through the head point.
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	var head: Vector3 = a.config.steering_head.point
	var steer_axis: Vector3 = a.config.steering_head.axis
	var fork_axis0: Vector3 = a.config.fork_travel.axis
	var theta: float = 0.3
	var dq: Quaternion = Quaternion(steer_axis, theta)
	for body in [a.steering, a.fork_slider, a.front_wheel]:
		body.position = head + dq * (body.position - head)
		body.orientation = dq * body.orientation
	check_near(a.steering_joint.angle(), theta, 2.0e-6, "the steering hinge angle equals the imposed rotation")
	check_vec3_near(a.fork_joint.world_axis_a(), dq * fork_axis0, 4.0e-6, "the world fork axis rotated about the steering axis")
	check_near(a.fork_joint.world_axis_a().dot(steer_axis), fork_axis0.dot(steer_axis), 4.0e-6, "the rake angle to the steering axis is preserved (the fork axis stays parallel to it)")
	check(a.fork_joint.lateral_error().length() <= 4.0e-6 and a.fork_joint.axis_error_angle() <= 4.0e-6, "the fork joint stays satisfied")
	check(a.front_axle_joint.linear_error().length() <= 4.0e-6, "the front axle stays satisfied")
	check_vec3_near(a.fork_slider.position, head + dq * (a.config.fork_slider.position - head), 4.0e-6, "the slider sits where the rotated reference puts it")
	# Dynamic: opposite angular impulses about the steering axis turn the steering assembly relative to the chassis; the fork axis
	# must follow: world fork axis = R(steering axis, angle) * configured fork axis, the slider stays on that line.
	var d: MotorcycleAssembly = _build()
	var frame: Dictionary = _fork_frame(d)
	var chassis_q0: Quaternion = d.chassis.orientation
	d.steering.apply_angular_impulse(steer_axis * 1.5)
	d.chassis.apply_angular_impulse(-steer_axis * 1.5)
	var worst_axis: float = 0.0
	var worst_lat: float = 0.0
	var angle_reached: float = 0.0
	for i in 60:
		d.world.step()
		var angle: float = d.steering_joint.angle()
		angle_reached = maxf(angle_reached, absf(angle))
		# The chassis recoils too (equal and opposite impulse): world fork axis = q_chassis * R(steer axis, angle) * f0, all in the
		# reference frame, because the steering axis is rigid in the chassis.
		var expected_axis: Vector3 = (d.chassis.orientation * chassis_q0.inverse()) * (Quaternion(steer_axis, angle) * fork_axis0)
		worst_axis = maxf(worst_axis, (d.fork_joint.world_axis_a() - expected_axis).length())
		worst_lat = maxf(worst_lat, Rig.measure(frame).lateral)
	check(angle_reached > 0.1, "the steering assembly really turned (%s rad)" % angle_reached)
	# The steering angle is a sine-type measure of a body whose axis error is bounded; the rake-preserving rotation holds to the
	# frozen-Jacobian level of the steering hinge (measured 6e-4).
	check(worst_axis <= 3.0e-3, "the fork axis stays on the rotated raked axis (worst deviation %s)" % worst_axis)
	check(worst_lat <= 3.0e-4, "the slider stays on the fork axis line while steering (worst lateral offset %s m)" % worst_lat)


func test_e4_the_slider_moves_only_along_the_fork_axis_under_excitation() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	var frame: Dictionary = _fork_frame(a)
	var u: Vector3 = a.config.fork_travel.axis
	var lateral: Vector3 = u.cross(Vector3.RIGHT).normalized()
	a.fork_slider.apply_impulse(u * -25.0 + lateral * 5.0 + Vector3.RIGHT * 4.0)
	a.front_wheel.apply_impulse(lateral * -3.0 + Vector3.RIGHT * 2.0)
	a.fork_slider.apply_angular_impulse(Vector3(0.3, -0.2, 0.4))
	var worst: Vector3 = Vector3.ZERO
	var s_min: float = 0.0
	var s_max: float = 0.0
	for i in 200:
		a.world.step()
		var m: Dictionary = Rig.measure(frame)
		worst = Vector3(maxf(worst.x, m.lateral), maxf(worst.y, m.axis), maxf(worst.z, absf(m.twist)))
		s_min = minf(s_min, m.s)
		s_max = maxf(s_max, m.s)
	# Measured worst values on this run: lateral 3e-4 m, axis 2e-3 rad, twist 2e-3 rad (frozen Jacobian / beta steady state with
	# the wheel and the steering assembly rotating). Bounds are ~3x those.
	check(worst.x <= 1.0e-3, "the slider's lateral offset from the fork axis stays small (worst %s m)" % worst.x)
	check(worst.y <= 6.0e-3, "its axis stays aligned with the steering assembly's (worst %s rad)" % worst.y)
	check(worst.z <= 6.0e-3, "it does not twist relative to the steering assembly (worst %s rad)" % worst.z)
	check(s_min >= -0.12 - 1.0e-3 and s_max <= 0.03 + 1.0e-3, "the travel stays inside the configured limits (s in [%s, %s])" % [s_min, s_max])
	check(s_min < -0.01 or s_max > 0.01, "the fork did move axially")
	check(a.world.numerical_fault == "", "no numerical fault")


func test_e5_the_front_wheel_spins_independently_about_its_axle() -> void:
	# Two runs, identical except for an axial angular impulse on the wheel; the fork is excited in both. Everything is judged by the
	# DIFFERENCE of the runs (an axial slider impulse acts off the fork axis and legitimately tilts the slider a little, 0.08 rad/s).
	var results: Array = []
	for spin in [false, true]:
		var a: MotorcycleAssembly = _build()
		if a == null:
			return
		var impulse: float = 5.0
		if spin:
			a.front_wheel.apply_angular_impulse(Vector3.RIGHT * impulse)
		a.fork_slider.apply_impulse(a.config.fork_travel.axis * -20.0)   # fork motion at the same time
		var s_track: PackedFloat64Array = PackedFloat64Array()
		for i in 90:
			a.world.step()
			s_track.append(a.fork_joint.translation())
		results.append({"s": s_track, "rate": a.front_axle_joint.angle_rate(), "a": a})
	var i_axle: float = results[1].a.front_wheel.local_inertia.x.x
	var expected: float = 5.0 / i_axle
	check_near(results[1].rate - results[0].rate, expected, 2.0e-3 * expected, "the spin adds L / I_axle to the relative wheel rate (%s rad/s)" % expected)
	check(absf(results[0].rate) < 0.5, "without the spin impulse the wheel's relative rate stays small (%s rad/s: only the slider's own tilt)" % results[0].rate)
	var worst: float = 0.0
	for i in 90:
		worst = maxf(worst, absf(results[1].s[i] - results[0].s[i]))
	# Measured 1e-4 m (a wheel spinning at 5.5 rad/s has gyroscopic reaction of order I w w_slider ~ 0.5 * 5.5 * 0.1 N m).
	check(worst <= 5.0e-4, "the spin does not change the fork travel (worst difference %s m)" % worst)
	var dw: float = (results[1].a.fork_slider.angular_velocity - results[0].a.fork_slider.angular_velocity).length()
	check(dw <= 0.05, "the slider does not pick up the wheel's spin (difference |dw| = %s rad/s)" % dw)


func test_e6_suspension_forces_transfer_momentum_without_net_external_momentum() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	var u: Vector3 = a.config.fork_travel.axis
	a.fork_slider.apply_impulse(u * 6.0)
	a.steering.apply_impulse(-u * 6.0)    # equal and opposite: total momentum zero
	var p_total0: Vector3 = a.world.total_linear_momentum()
	var lower_changed: float = 0.0
	var worst_total: float = 0.0
	var worst_pair: float = 0.0
	for i in 180:
		# Provider forces of this step, read before the step consumes them.
		a.front_suspension.apply_forces(DT, a.world.config)
		worst_pair = maxf(worst_pair, (a.steering.force_accumulator + a.fork_slider.force_accumulator).length())
		a.steering.clear_accumulators()
		a.fork_slider.clear_accumulators()
		a.world.step()
		worst_total = maxf(worst_total, (a.world.total_linear_momentum() - p_total0).length())
		lower_changed = maxf(lower_changed, (_lower_momentum(a) - (a.fork_slider.mass + a.front_wheel.mass) * Vector3.ZERO).length())
	check(worst_pair <= 1.0e-3, "at every step the two suspension forces are equal and opposite (worst sum %s N)" % worst_pair)
	# float32 random walk of |P| ~ 50 over 240 steps with 200 iterations. Measured 4e-4.
	check(worst_total <= 3.0e-3, "total linear momentum is unchanged: the suspension is internal (worst drift %s N s)" % worst_total)
	# The first step sees the artificially injected, constraint-inconsistent relative velocity (1.2 m/s instead of 0.4 m/s) and
	# damps it hard; the lower part keeps 2.1 N s of the 6 N s along the axis after step 1 and then exchanges it with the sprung part.
	check(lower_changed > 1.0, "momentum really moved between the sprung and unsprung parts (peak lower-part momentum %s N s)" % lower_changed)


func test_e7_stable_under_gravity_and_excitation_with_no_hidden_force() -> void:
	# Free fall (no ground): the fork must not move at all: every body accelerates identically and the spring is unloaded.
	var a: MotorcycleAssembly = _build(Vector3(0.0, -9.81, 0.0))
	if a == null:
		return
	var worst_s: float = 0.0
	for i in 240:
		a.world.step()
		worst_s = maxf(worst_s, absf(a.fork_joint.translation()))
	check(worst_s <= 2.0e-4, "free fall for 2 s leaves the fork at its reference position (worst |s| = %s m)" % worst_s)
	check(a.world.numerical_fault == "" and a.fork_slider.position.is_finite(), "finite, no fault")
	# Representative excitation under gravity: stays finite, inside the limits, energy never created.
	var b: MotorcycleAssembly = _build(Vector3(0.0, -9.81, 0.0))
	var u: Vector3 = b.config.fork_travel.axis
	b.fork_slider.apply_impulse(u * -40.0)    # toward compression
	b.front_wheel.apply_impulse(Vector3(0.0, 2.0, -1.0))
	b.steering.apply_angular_impulse(b.config.steering_head.axis * 0.5)
	var s_min: float = 0.0
	var s_max: float = 0.0
	var worst_rise: float = 0.0
	var e0: float = b.world.total_kinetic_energy() + b.world.total_potential_energy() + b.front_suspension.spring_energy()
	for i in 360:
		b.world.step()
		var s: float = b.fork_joint.translation()
		s_min = minf(s_min, s)
		s_max = maxf(s_max, s)
		var e: float = b.world.total_kinetic_energy() + b.world.total_potential_energy() + b.front_suspension.spring_energy()
		worst_rise = maxf(worst_rise, e - e0)
	check(b.world.numerical_fault == "", "no numerical fault")
	check(s_min >= -0.12 - 1.0e-3 and s_max <= 0.03 + 1.0e-3, "inside the travel limits (s in [%s, %s])" % [s_min, s_max])
	check(s_min < -0.015, "the excitation really compressed the fork (min s %s)" % s_min)
	# Energy: damping, the inelastic limit and the frozen-Jacobian loss only remove energy; the bounded symplectic-Euler spring
	# oscillation (w dt / 2 = 11 % of the spring energy, here <= 0.5 J of a 10 J-scale stored energy) and the split pose correction
	# (which moves bodies without momentum) can add a little. Measured worst rise: see message; bound 2 J on E0 of ~40 J.
	check(worst_rise <= 2.0, "total mechanical energy is not created beyond the discretisation bound (worst rise %s J, E0 = %s J)" % [worst_rise, e0])
	var e_end: float = b.world.total_kinetic_energy() + b.world.total_potential_energy() + b.front_suspension.spring_energy()
	check(e_end <= e0 + 2.0, "and ends at or below its starting value")
	check((b.fork_slider.linear_velocity - b.chassis.linear_velocity).length() < 10.0, "bounded velocities relative to the falling chassis")


func test_e8_the_rear_assembly_keeps_its_m21_behaviour_next_to_a_moving_fork() -> void:
	var a: MotorcycleAssembly = _build()
	if a == null:
		return
	var i_axle: float = a.rear_wheel.local_inertia.x.x
	a.rear_wheel.apply_angular_impulse(Vector3.RIGHT * 5.0)
	a.fork_slider.apply_impulse(a.config.fork_travel.axis * 4.0)
	var worst: Vector2 = Vector2.ZERO
	for i in 120:
		a.world.step()
		worst = Vector2(maxf(worst.x, a.rear_axle_joint.linear_error().length()), maxf(worst.y, a.rear_axle_joint.axis_error_angle()))
	check_near(a.rear_axle_joint.angle_rate(), 5.0 / i_axle, 2.0e-3 * 5.0 / i_axle, "the rear wheel spins at L / I_axle relative to the swingarm")
	check(worst.x <= 2.0e-4 and worst.y <= 2.0e-3, "the rear axle stays attached and aligned (worst %s m, %s rad)" % [worst.x, worst.y])
	check(a.swingarm_joint.linear_error().length() <= 2.0e-4, "the swingarm pivot stays attached")
	check(a.world.numerical_fault == "", "no fault")


func test_e9_identical_simulations_are_bit_identical() -> void:
	var runs: Array = []
	for n in 2:
		var a: MotorcycleAssembly = _build(Vector3(0.0, -9.81, 0.0))
		a.fork_slider.apply_impulse(a.config.fork_travel.axis * 5.0)
		a.front_wheel.apply_angular_impulse(Vector3.RIGHT * 3.0)
		for i in 150:
			a.world.step()
		var snap: Array = []
		for body in a.bodies():
			snap.append([body.position, body.orientation, body.linear_velocity, body.angular_velocity])
		runs.append(snap)
	check(runs[0] == runs[1], "two identical runs produce bit-identical state for all six bodies")


func test_e10_without_spring_damper_or_limits_the_fork_is_free_no_hidden_damping() -> void:
	var cfg: MotorcycleConfig = _config()
	cfg.front_suspension.stiffness = 0.0
	cfg.front_suspension.damping = 0.0
	cfg.front_suspension.travel_limits_enabled = false
	var a: MotorcycleAssembly = _build(Vector3.ZERO, cfg)
	if a == null:
		return
	var u: Vector3 = a.config.fork_travel.axis
	a.fork_slider.apply_impulse(u * 3.0)
	var p0: float = _lower_momentum(a).dot(u)
	var worst: float = 0.0
	for i in 160:
		a.world.step()
		worst = maxf(worst, absf(_lower_momentum(a).dot(u) - p0))
	# The slider + wheel pair receives no axial impulse from any row, so its axial momentum is exactly constant (identity of
	# test_prismatic_joint.gd); float32 over 240 steps.
	check(worst <= 2.0e-4 * absf(p0), "axial momentum of slider + wheel is constant over 1.3 s: no hidden damping or force (worst deviation %s of %s)" % [worst, p0])
	check(a.fork_joint.translation() > 0.2, "the slider travelled freely well beyond the default stroke (s = %s m)" % a.fork_joint.translation())
	check(a.fork_joint.lower_limit_impulse() == 0.0 and a.fork_joint.upper_limit_impulse() == 0.0, "no limit impulse")


func test_e11_builder_is_transactional_for_every_registration_stage_and_every_invalid_suspension() -> void:
	var stages: Array = []
	for n in range(1, 7):
		stages.append([n, -1, -1, "add_body #%d" % n])
	for n in range(1, 6):
		stages.append([-1, n, -1, "add_constraint #%d" % n])
	stages.append([-1, -1, 1, "add_force_provider #1"])
	stages.append([-1, -1, 2, "add_force_provider #2 (rear suspension, M2.3)"])
	for st in stages:
		var w: FailingWorld = FailingWorld.new()
		w.config.gravity = Vector3.ZERO
		var existing: SimRigidBody = w.add_body(make_static())
		w.body_calls = 0
		var bodies_before: Array = w.bodies.duplicate()
		var cfg: MotorcycleConfig = _config()
		w.fail_body_call = st[0]
		w.fail_constraint_call = st[1]
		w.fail_provider_call = st[2]
		check(MotorcycleBuilder.build(cfg, w) == null, "%s refused: build returns null" % st[3])
		check(w.bodies == bodies_before and w.constraints.is_empty() and w.force_providers.is_empty(), "%s refused: bodies, constraints and force providers fully rolled back (%d, %d, %d)" % [st[3], w.bodies.size(), w.constraints.size(), w.force_providers.size()])
		check(w.has_body(existing), "%s refused: the pre-existing body survives" % st[3])
		w.fail_body_call = -1
		w.fail_constraint_call = -1
		w.fail_provider_call = -1
		var retry: MotorcycleAssembly = MotorcycleBuilder.build(_config(), w)
		check(retry != null and w.bodies.size() == 7 and w.constraints.size() == 5 and w.force_providers.size() == 2, "%s refused: a retry succeeds completely" % st[3])
		if retry != null:
			check(w.step(), "%s refused: the retried assembly steps" % st[3])
	# Invalid configurations are rejected before anything is registered, in a populated world, and a valid build follows.
	var cases: Array = [
		["negative stiffness", func(c): c.front_suspension.stiffness = -1.0, "stiffness"],
		["NaN damping", func(c): c.front_suspension.damping = NAN, "damping"],
		["negative damping", func(c): c.front_suspension.damping = -5.0, "damping"],
		["infinite rest displacement", func(c): c.front_suspension.rest_displacement = INF, "rest"],
		["lower limit above zero", func(c): c.front_suspension.lower_travel_limit = 0.01, "lower <= 0 <= upper"],
		["upper limit below zero", func(c): c.front_suspension.upper_travel_limit = -0.01, "lower <= 0 <= upper"],
		["NaN limit", func(c): c.front_suspension.lower_travel_limit = NAN, "limits must be finite"],
		["seats off the fork line", func(c): c.front_suspension.lower_attachment += Vector3.RIGHT * 0.01, "attachment"],
		["fork axis tilted away from the steering axis", func(c): c.fork_travel.axis = Quaternion(Vector3.RIGHT, 0.05) * c.fork_travel.axis, "parallel"],
		["non-unit fork axis", func(c): c.fork_travel.axis *= 1.01, "unit"],
		["NaN fork point", func(c): c.fork_travel.point = Vector3(NAN, 0, 0), "point"],
		["zero slider mass", func(c): c.fork_slider.mass = 0.0, "mass"],
		["ill-conditioned slider inertia", func(c): c.fork_slider.local_inertia = Basis.from_scale(Vector3(1.0, 1.0, 1.0e-7)), "inertia"],
		["slider visual size zero", func(c): c.fork_slider.visual_size = Vector3.ZERO, "visual_size"],
		["null suspension spec", func(c): c.front_suspension = null, "suspension"],
	]
	var w2: SimWorld = make_world()
	var anchor: SimRigidBody = w2.add_body(make_static())
	var bodies_before: Array = w2.bodies.duplicate()
	for case in cases:
		var cfg: MotorcycleConfig = _config()
		case[1].call(cfg)
		var errors: PackedStringArray = cfg.validate()
		check(not errors.is_empty() and " ".join(errors).containsn(case[2]), "'%s' is reported (%s)" % [case[0], errors])
		check(MotorcycleBuilder.build(cfg, w2) == null, "'%s': build returns null" % case[0])
		check(w2.bodies == bodies_before and w2.constraints.is_empty() and w2.force_providers.is_empty(), "'%s': the world is exactly as before" % case[0])
	check(MotorcycleBuilder.build(_config(), w2) != null and w2.bodies.size() == 7, "after the rejected attempts a valid build succeeds")
	# The post-construction check also covers the fork.
	var a: MotorcycleAssembly = _build()
	a.fork_slider.position += Vector3.RIGHT * 0.001
	check(MotorcycleBuilder.construction_error(a).containsn("fork") or MotorcycleBuilder.construction_error(a).containsn("anchors"), "a 1 mm lateral displacement of the slider is reported (%s)" % MotorcycleBuilder.construction_error(a))


# =====================================================================================================================
# F. Regression sensitivity of the integrated tests
# =====================================================================================================================

## Replaces the fork's prismatic joint by `script.new(...)` with identical data and re-points the suspension at it.
func _swap_fork(a: MotorcycleAssembly, script) -> void:
	var old = a.fork_joint
	a.world.remove_constraint(old)
	var m = script.new(old.body_a, old.body_b, old.local_anchor_a, old.local_anchor_b, old.local_axis_a, old.local_axis_b, old.local_reference_a, old.local_reference_b)
	m.limit_enabled = old.limit_enabled
	m.lower_limit = old.lower_limit
	m.upper_limit = old.upper_limit
	a.world.add_constraint(m)
	a.fork_joint = m
	a.front_suspension.joint = m


func _swap_suspension(a: MotorcycleAssembly, script) -> void:
	var old = a.front_suspension
	a.world.remove_force_provider(old)
	var m = script.new(old.joint, old.local_attachment_a, old.local_attachment_b, old.stiffness, old.damping, old.rest_displacement, old.reference_separation)
	a.world.add_force_provider(m)
	a.front_suspension = m


## Battery of physical criteria on the front end (zero gravity, spring + limits as configured). Empty = behaves as configured.
func _front_violations(mutate: Callable) -> PackedStringArray:
	var out: PackedStringArray = PackedStringArray()
	# 1. The slider moves only along the CONFIGURED fork axis, and does not rotate relative to the steering assembly.
	var a: MotorcycleAssembly = _build(Vector3.ZERO, null, BATTERY_ITERATIONS)
	mutate.call(a)
	var frame: Dictionary = _fork_frame(a)
	var u: Vector3 = a.config.fork_travel.axis
	var side: Vector3 = u.cross(Vector3.RIGHT).normalized()
	a.fork_slider.apply_impulse(u * -25.0 + side * 3.0)
	a.fork_slider.apply_angular_impulse(u * 0.6 + side * 0.3)
	var worst: Vector3 = Vector3.ZERO
	for i in 80:
		a.world.step()
		var m: Dictionary = Rig.measure(frame)
		worst = Vector3(maxf(worst.x, m.lateral), maxf(worst.y, m.axis), maxf(worst.z, absf(m.twist)))
	# Real assembly: worst lateral 2.2e-5 m (measured). A fork axis tilted 6 / 10 / 20 degrees gives 1.2e-3 / 1.8e-3 / 5.4e-3 m.
	if worst.x > 3.0e-4:
		out.append("the slider leaves the configured fork axis (lateral %s m)" % worst.x)
	if worst.y > 1.0e-2 or worst.z > 1.0e-2:
		out.append("the slider rotates relative to the steering assembly (axis %s rad, twist %s rad)" % [worst.y, worst.z])
	# 2. The front wheel hangs on the slider: an axial slider impulse must move the wheel with it.
	var b: MotorcycleAssembly = _build(Vector3.ZERO, null, BATTERY_ITERATIONS)
	mutate.call(b)
	var axle: Array = _axle_frame(b)
	b.fork_slider.apply_impulse(b.config.fork_travel.axis * 4.0)
	var worst_axle: float = 0.0
	for i in 40:
		b.world.step()
		worst_axle = maxf(worst_axle, _axle_error(b, axle))
	if worst_axle > 2.0e-3:
		out.append("the front wheel is not held by the slider's axle (pivot separation %s m)" % worst_axle)
	# 3. Spring-damper: restoring, dissipative, internal. Release from 5 cm compression.
	var c: MotorcycleAssembly = _build(Vector3.ZERO, null, BATTERY_ITERATIONS)
	mutate.call(c)
	c.fork_slider.position += c.config.fork_travel.axis * -0.05
	c.front_wheel.position += c.config.fork_travel.axis * -0.05
	for body in [c.fork_slider, c.front_wheel]:
		body.reset_published_state()
	var e0: float = c.world.total_kinetic_energy() + c.front_suspension.spring_energy()
	var e_max: float = e0
	var p_max: float = 0.0
	for i in 160:
		c.world.step()
		e_max = maxf(e_max, c.world.total_kinetic_energy() + c.front_suspension.spring_energy())
		p_max = maxf(p_max, c.world.total_linear_momentum().length())
	var e_end: float = c.world.total_kinetic_energy() + c.front_suspension.spring_energy()
	# Discrete bound 1 / (1 - w dt / 2) with w = 26 rad/s: 1.12 ; margin to 1.25 (chassis coupling shifts w slightly).
	if e_max > 1.25 * e0:
		out.append("the suspension creates energy (peak %s J from %s J)" % [e_max, e0])
	if e_end > 0.5 * e0:
		out.append("the suspension does not dissipate (end %s J from %s J)" % [e_end, e0])
	if p_max > 3.0e-3:
		out.append("the suspension changes the total momentum (peak %s N s)" % p_max)
	# 4. Limit hold: a hard compression stays inside the travel (spring off so only the limit acts).
	var cfg: MotorcycleConfig = _config()
	cfg.front_suspension.stiffness = 0.0
	cfg.front_suspension.damping = 0.0
	var d: MotorcycleAssembly = _build(Vector3.ZERO, cfg, BATTERY_ITERATIONS)
	mutate.call(d)
	d.fork_slider.apply_impulse(d.config.fork_travel.axis * -12.0)
	var s_min: float = 0.0
	for i in 90:
		d.world.step()
		s_min = minf(s_min, Rig.measure(_fork_frame_now(d, frame)).s)
	if s_min < -0.12 - 2.0e-3:
		out.append("the compression limit is crossed (min s %s)" % s_min)
	# 5. A violated limit is corrected without injecting physical velocity (spring off).
	var e: MotorcycleAssembly = _build(Vector3.ZERO, cfg, BATTERY_ITERATIONS)
	mutate.call(e)
	var violation: float = 0.01
	for body in [e.fork_slider, e.front_wheel]:
		body.position += e.config.fork_travel.axis * (-0.12 - violation)
		body.reset_published_state()
	var ke_before: float = e.world.total_kinetic_energy()
	e.world.step()
	var v_slider: float = e.fork_slider.linear_velocity.length()
	if v_slider > 1.0e-3:
		out.append("a violated limit injected physical velocity (%s m/s)" % v_slider)
	return out


## Fork frame measured at the REFERENCE pose of a fresh assembly (the pose edit above makes the live one useless); the bodies are
## the live ones of `a`, the local data the test-owned ones from the reference `frame`.
func _fork_frame_now(a: MotorcycleAssembly, frame: Dictionary) -> Dictionary:
	var f: Dictionary = frame.duplicate()
	f["a"] = a.steering
	f["b"] = a.fork_slider
	return f


func test_f1_the_battery_accepts_the_real_assembly() -> void:
	var v: PackedStringArray = _front_violations(func(a): pass)
	check(v.is_empty(), "the real assembly satisfies every front-end criterion: %s" % v)


func test_f2_a_missing_prismatic_constraint_is_detected() -> void:
	var v: PackedStringArray = _front_violations(func(a): a.world.remove_constraint(a.fork_joint))
	check(" ".join(v).contains("leaves the configured fork axis") or " ".join(v).contains("rotates"), "missing fork joint: %s" % v)


func test_f3_a_tilted_fork_axis_is_detected() -> void:
	var v: PackedStringArray = _front_violations(func(a):
		var old = a.fork_joint
		a.world.remove_constraint(old)
		var spec = a.config.fork_travel
		var tilted: Vector3 = Quaternion(Vector3.RIGHT, deg_to_rad(10.0)) * spec.axis
		var j = PrismaticJoint.from_world(a.steering, a.fork_slider, spec.point, tilted, MotorcycleBuilder.hinge_reference(tilted))
		j.limit_enabled = old.limit_enabled
		j.lower_limit = old.lower_limit
		j.upper_limit = old.upper_limit
		a.world.add_constraint(j)
		a.fork_joint = j
		a.front_suspension.joint = j)
	check(" ".join(v).contains("leaves the configured fork axis"), "a fork axis tilted 10 degrees from the configured one: %s" % v)


func test_f4_a_prismatic_joint_that_permits_rotation_is_detected() -> void:
	var v: PackedStringArray = _front_violations(func(a): _swap_fork(a, PJ.NoTwist))
	check(" ".join(v).contains("rotates relative"), "no twist row: %s" % v)
	var v2: PackedStringArray = _front_violations(func(a): _swap_fork(a, PJ.NoAxisRows))
	check(" ".join(v2).contains("rotates relative"), "no axis-alignment rows: %s" % v2)
	var v3: PackedStringArray = _front_violations(func(a): _swap_fork(a, PrismaticJoint))
	check(v3.is_empty(), "control: swapping in the unmodified joint is accepted (%s)" % v3)


func test_f5_a_limit_row_with_reversed_sign_is_detected() -> void:
	var v: PackedStringArray = _front_violations(func(a): _swap_fork(a, PJ.ReversedLimitSign))
	check(" ".join(v).contains("compression limit is crossed"), "reversed limit rows: %s" % v)


func test_f6_a_split_limit_correction_applied_to_physical_velocity_is_detected() -> void:
	var v: PackedStringArray = _front_violations(func(a): _swap_fork(a, PJ.SplitOnPhysicalVelocity))
	check(" ".join(v).contains("injected physical velocity"), "velocity-level limit correction: %s" % v)


func test_f7_a_reversed_spring_is_detected() -> void:
	var v: PackedStringArray = _front_violations(func(a): _swap_suspension(a, ReversedSpring))
	check(" ".join(v).contains("creates energy") or " ".join(v).contains("compression limit"), "reversed spring sign: %s" % v)


func test_f8_energy_injecting_damping_is_detected() -> void:
	var v: PackedStringArray = _front_violations(func(a): _swap_suspension(a, InjectingDamping))
	check(not v.is_empty() and (" ".join(v).contains("creates energy") or " ".join(v).contains("does not dissipate")), "damping with the wrong sign: %s" % v)


func test_f9_a_force_applied_to_only_one_side_is_detected() -> void:
	var v: PackedStringArray = _front_violations(func(a): _swap_suspension(a, OneSidedForce))
	check(" ".join(v).contains("changes the total momentum"), "one-sided suspension force: %s" % v)


func test_f10_a_front_axle_left_on_the_upper_steering_assembly_is_detected() -> void:
	var v: PackedStringArray = _front_violations(func(a):
		var old = a.front_axle_joint
		a.world.remove_constraint(old)
		var spec = a.config.front_axle
		var j = HingeJoint.from_world(a.steering, a.front_wheel, spec.point, spec.axis, MotorcycleBuilder.hinge_reference(spec.axis))
		a.world.add_constraint(j)
		a.front_axle_joint = j)
	check(" ".join(v).contains("not held by the slider's axle"), "front axle on the steering assembly: %s" % v)
