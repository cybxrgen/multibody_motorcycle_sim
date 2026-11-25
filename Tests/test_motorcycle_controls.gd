extends "res://Tests/sim_test_case.gd"
## M2.5.2 basic direct controls (throttle, brake, steering) and the integrated motorcycle behaviour. Expectations are written here:
## torque directions from the axis convention (forward rolling = NEGATIVE rate about the wheel's +X axle), brake magnitudes from the
## documented cap min(command, I_red |w_rel| / dt). Tolerances are stated where used.

const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")

const DT: float = 1.0 / 120.0
## Rate noise (rad/s) of a relative wheel spin: the swingarm / fork carrier is coupled to the rest of the machine through the hinge and the
## shock, which moves the relative rate by a few 1e-4 rad/s (measured 3.5e-4 at 20 rad/s initial spin); 5e-3 is 14x that and 0.025 % of the spin.
const NOISE: float = 5.0e-3


## Assembly without ground and tyres, zero gravity: the controls alone act (a free-space fixture for torque / momentum checks).
func free_space():
	var c = MotorcyclePrototype.config_with_collision()
	c.ground = null
	c.front_tire = null
	c.rear_tire = null
	var world = SimWorld.new()
	world.config.gravity = Vector3.ZERO
	return MotorcycleBuilder.build(c, world)


func apply_once(a) -> void:
	for body in a.world.bodies:
		body.clear_accumulators()
	a.controls.apply_forces(DT, a.world.config)


func test_zero_inputs_apply_nothing() -> void:
	var a = free_space()
	apply_once(a)
	for body in a.bodies():
		check(body.force_accumulator == Vector3.ZERO and body.torque_accumulator == Vector3.ZERO, "%s: no force and no torque at zero input" % body.body_name)
	check(a.controls.last_drive_torque == 0.0 and a.controls.last_front_brake_torque == 0.0 and a.controls.last_rear_brake_torque == 0.0, "reported torques are zero")


func test_throttle_is_a_torque_about_the_rear_axle_scaled_by_the_input() -> void:
	var a = free_space()
	var axle: Vector3 = a.rear_wheel.orientation * Vector3.RIGHT
	var maximum: float = MotorcyclePrototype.MAX_DRIVE_TORQUE
	var previous: float = 0.0
	for t in [0.25, 0.5, 1.0]:
		check(a.controls.set_inputs(t, 0.0, 0.0), "valid input %s" % t)
		apply_once(a)
		check_vec3_near(a.rear_wheel.torque_accumulator, -axle * t * maximum, 1.0e-4, "throttle %s: torque %s N m about the axle, forward-rolling sign (-axle)" % [t, t * maximum])
		check_vec3_near(a.swingarm.torque_accumulator, axle * t * maximum, 1.0e-4, "the equal and opposite reaction acts on the swingarm")
		check(a.rear_wheel.torque_accumulator.length() > previous, "torque grows with the input")
		previous = a.rear_wheel.torque_accumulator.length()
		for body in a.bodies():
			check(body.force_accumulator == Vector3.ZERO, "%s: throttle applies no force" % body.body_name)
		check(a.front_wheel.torque_accumulator == Vector3.ZERO and a.chassis.torque_accumulator == Vector3.ZERO, "nothing on the front wheel or chassis")
	check(previous <= maximum + 1.0e-4, "never above the configured maximum")


func test_throttle_does_not_accelerate_the_chassis_directly() -> void:
	var a = free_space()
	a.controls.set_inputs(1.0, 0.0, 0.0)
	for k in 120:
		a.world.step()
	check(a.world.total_linear_momentum().length() < 1.0e-3, "internal torque pair: total linear momentum stays zero (%s)" % a.world.total_linear_momentum())
	var spin: float = a.rear_wheel.angular_velocity.dot(a.rear_wheel.orientation * Vector3.RIGHT)
	check(spin < -1.0, "the rear wheel spins forward (%s rad/s)" % spin)


func test_rear_torque_moves_the_motorcycle_only_through_the_tyre() -> void:
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	a.controls.set_inputs(1.0, 0.0, 0.0)
	var peak_z: float = 0.0
	for k in 240:
		a.world.step()
	check(a.chassis.linear_velocity.z < -1.0, "forward (-Z) motion after 2 s (%s m/s)" % a.chassis.linear_velocity.z)
	check(a.rear_tire.longitudinal_force() > 0.0 or a.rear_tire.longitudinal_force() < 0.0, "the tyre carries a longitudinal force")
	check(a.world.numerical_fault == "", "no numerical fault")
	# Same run with the tyre force switched off: the wheel spins up but the motorcycle does not accelerate.
	var b = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	b.set_tire_forces_enabled(false)
	b.controls.set_inputs(1.0, 0.0, 0.0)
	for k in 240:
		b.world.step()
	check(absf(b.chassis.linear_velocity.z) < 0.05, "without tyre traction there is no forward acceleration (%s m/s)" % b.chassis.linear_velocity.z)


func test_brake_opposes_both_wheel_spins_with_the_documented_magnitude() -> void:
	var a = free_space()
	for sign in [-1.0, 1.0]:
		for pair in [[a.front_wheel, a.fork_slider, MotorcyclePrototype.MAX_FRONT_BRAKE_TORQUE], [a.rear_wheel, a.swingarm, MotorcyclePrototype.MAX_REAR_BRAKE_TORQUE]]:
			for body in a.world.bodies:
				body.angular_velocity = Vector3.ZERO
			var u: Vector3 = pair[0].orientation * Vector3.RIGHT
			pair[0].angular_velocity = u * (sign * 30.0)
			a.controls.set_inputs(0.0, 1.0, 0.0)
			apply_once(a)
			var torque: float = pair[0].torque_accumulator.dot(u)
			check(torque * sign < 0.0, "%s spinning %s: brake torque opposes it (%s)" % [pair[0].body_name, sign, torque])
			var i_w: float = u.dot(pair[0].inertia_world * u)
			var i_c: float = u.dot(pair[1].inertia_world * u)
			var i_red: float = 1.0 / (1.0 / i_w + 1.0 / i_c)
			check_near(absf(torque), minf(pair[2], 0.5 * i_red * 30.0 / DT), 1.0e-3, "%s: magnitude = min(command, 1/2 I_red |w| / dt)" % pair[0].body_name)
			check_vec3_near(pair[1].torque_accumulator, -pair[0].torque_accumulator, 1.0e-4, "reaction on the carrier")
	# Half brake gives half the commanded torque (while below the stopping cap).
	a.rear_wheel.angular_velocity = (a.rear_wheel.orientation * Vector3.RIGHT) * -30.0
	a.front_wheel.angular_velocity = Vector3.ZERO
	a.controls.set_inputs(0.0, 0.5, 0.0)
	apply_once(a)
	check_near(absf(a.rear_wheel.torque_accumulator.dot(a.rear_wheel.orientation * Vector3.RIGHT)), 0.5 * MotorcyclePrototype.MAX_REAR_BRAKE_TORQUE, 1.0e-3, "brake 0.5 = half the maximum")
	check(a.front_wheel.torque_accumulator == Vector3.ZERO, "a stationary front wheel gets no torque")


func test_brake_cannot_reverse_a_wheel_or_inject_energy() -> void:
	var a = free_space()
	var u: Vector3 = a.rear_wheel.orientation * Vector3.RIGHT
	a.rear_wheel.angular_velocity = -u * 20.0
	a.front_wheel.angular_velocity = -u * 20.0
	a.controls.set_inputs(0.0, 1.0, 0.0)
	var worst_growth: float = -INF
	var reversed: bool = false
	var last: Array = [20.0, 20.0]
	for k in 600:
		a.world.step()
		var now: Array = []
		for pair in [[a.rear_wheel, a.swingarm], [a.front_wheel, a.fork_slider]]:
			var rate: float = (pair[0].angular_velocity - pair[1].angular_velocity).dot(u)
			reversed = reversed or rate > NOISE
			now.append(absf(rate))
		for i in 2:
			worst_growth = maxf(worst_growth, now[i] - last[i])
		last = now
	check(not reversed, "the wheel rotation never reverses sign")
	check(worst_growth < NOISE, "|relative spin| never grows, so the brake does no positive work (worst step %s rad/s)" % worst_growth)
	check(absf(a.rear_wheel.angular_velocity.dot(u)) < 1.0, "the rear wheel is braked to (almost) rest")
	# A stationary motorcycle with the brake on stays put.
	var b = free_space()
	b.controls.set_inputs(0.0, 1.0, 0.0)
	apply_once(b)
	check(b.rear_wheel.torque_accumulator == Vector3.ZERO and b.front_wheel.torque_accumulator == Vector3.ZERO, "no torque on stationary wheels")


func test_throttle_and_brake_together_apply_both_torques() -> void:
	var a = free_space()
	var u: Vector3 = a.rear_wheel.orientation * Vector3.RIGHT
	a.rear_wheel.angular_velocity = -u * 30.0
	a.controls.set_inputs(0.5, 0.5, 0.0)
	apply_once(a)
	var expected: float = 0.5 * MotorcyclePrototype.MAX_DRIVE_TORQUE
	var brake: float = 0.5 * MotorcyclePrototype.MAX_REAR_BRAKE_TORQUE
	check_near(a.rear_wheel.torque_accumulator.dot(u), -expected + brake, 1.0e-3, "net rear axle torque = drive (forward) + brake (opposing): superposition, no priority")


func test_steering_commands_the_hinge_motor_with_the_right_sign_and_cap() -> void:
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	var j = a.steering_joint
	a.controls.set_inputs(0.0, 0.0, 1.0)
	a.world.step()
	check(j.motor_enabled and absf(j.motor_target_speed - MotorcyclePrototype.MAX_STEERING_RATE) < 1.0e-9 and absf(j.motor_max_torque - MotorcyclePrototype.MAX_STEERING_TORQUE) < 1.0e-9, "left: target +max rate with the torque cap")
	for k in 30:
		a.world.step()
	check(j.angle() > 0.1, "steering left = positive hinge angle (%s)" % j.angle())
	check(a.fork_slider.position.x < -0.005, "the front of the fork moves toward -X (left) (%s)" % a.fork_slider.position.x)
	check(absf(j.motor_impulse()) <= MotorcyclePrototype.MAX_STEERING_TORQUE * DT * (1.0 + 1.0e-6), "motor impulse stays within torque * dt")
	var b = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	b.controls.set_inputs(0.0, 0.0, -0.5)
	for k in 30:
		b.world.step()
	check(b.steering_joint.angle() < -0.03 and absf(b.steering_joint.angle()) < absf(a.steering_joint.angle()), "right / half input: negative and slower (%s)" % b.steering_joint.angle())
	check(a.world.numerical_fault == "" and b.world.numerical_fault == "", "no fault")


func test_steering_respects_the_hinge_limits_and_release_holds_it_still() -> void:
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	var j = a.steering_joint
	check(j.limit_enabled and absf(j.upper_limit - MotorcyclePrototype.STEERING_LIMIT) < 1.0e-9 and absf(j.lower_limit + MotorcyclePrototype.STEERING_LIMIT) < 1.0e-9, "the configured limits are on the steering hinge")
	a.controls.set_inputs(0.0, 0.0, 1.0)
	var peak: float = 0.0
	for k in 360:
		a.world.step()
		peak = maxf(peak, j.angle())
	check(peak <= MotorcyclePrototype.STEERING_LIMIT + 5.0e-3, "the motor never drives through the limit (peak %s rad)" % peak)
	check(peak > MotorcyclePrototype.STEERING_LIMIT - 0.02, "it does reach the stop (peak %s)" % peak)
	# Release in a mid-range angle: the rate goes to zero and the angle stays.
	var b = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	b.controls.set_inputs(0.0, 0.0, 1.0)
	for k in 30:
		b.world.step()
	b.controls.set_inputs(0.0, 0.0, 0.0)
	for k in 30:
		b.world.step()
	var held: float = b.steering_joint.angle()
	for k in 120:
		b.world.step()
	check(absf(b.steering_joint.angle_rate()) < 1.0e-2 and absf(b.steering_joint.angle() - held) < 2.0e-3, "released: the steering rate is held at zero and the angle stays (%s -> %s, rate %s)" % [held, b.steering_joint.angle(), b.steering_joint.angle_rate()])
	check(b.steering_joint.motor_enabled and b.steering_joint.motor_target_speed == 0.0, "documented neutral behaviour: motor on with target rate 0")


func test_controls_are_evaluated_once_per_fixed_step() -> void:
	var a = free_space()
	a.controls.set_inputs(1.0, 0.0, 0.0)
	var before: float = a.rear_wheel.angular_velocity.dot(a.rear_wheel.orientation * Vector3.RIGHT)
	var steps: int = a.world.advance(4.0 * DT)
	check(steps == 4, "four fixed steps ran")
	var after: float = a.rear_wheel.angular_velocity.dot(a.rear_wheel.orientation * Vector3.RIGHT)
	var b = free_space()
	b.controls.set_inputs(1.0, 0.0, 0.0)
	for k in 4:
		b.world.step()
	check(after == b.rear_wheel.angular_velocity.dot(b.rear_wheel.orientation * Vector3.RIGHT), "advance() over several steps = the same number of single steps: the torque acts at every fixed step")
	check(after < before - 0.1, "and it did spin the wheel up")


func test_input_validation_and_safe_removal() -> void:
	var a = free_space()
	a.controls.set_inputs(0.3, 0.2, -0.4)
	for bad in [[NAN, 0.0, 0.0], [1.5, 0.0, 0.0], [-0.1, 0.0, 0.0], [0.0, 1.1, 0.0], [0.0, NAN, 0.0], [0.0, 0.0, 2.0], [0.0, 0.0, INF]]:
		check(not a.controls.set_inputs(bad[0], bad[1], bad[2]), "input %s is refused" % [bad])
	check(a.controls.throttle == 0.3 and a.controls.brake == 0.2 and a.controls.steering == -0.4, "refused inputs leave the old values")
	a.controls.throttle = NAN
	check(not a.controls.validate().is_empty() and not a.world.step(), "an invalid value written directly stops the world instead of being simulated")
	a.controls.throttle = 0.0
	check(a.world.step(), "and stepping resumes when valid")
	a.world.remove_body(a.rear_wheel)
	check(not a.world.force_providers.has(a.controls), "removing a body drops the provider")
	check(a.world.step(), "the world keeps stepping")
	var b = free_space()
	b.controls.set_inputs(0.0, 0.0, 1.0)
	b.world.step()
	b.remove_controls()
	check(not b.steering_joint.motor_enabled and b.controls == null and b.world.force_providers.size() == 2, "assembly teardown releases the motor and the provider")


func test_integrated_fall_and_combined_inputs_are_stable() -> void:
	# Fall: a small lateral disturbance (no stabilizer exists); the machine topples onto its structural boxes and settles.
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	for body in a.bodies():
		body.linear_velocity = Vector3(0.3, 0.0, 0.0)
	var box_contact_seen: Dictionary = {}
	var worst_gap: float = INF
	for k in 1200:
		a.world.step()
		for name in ["chassis", "steering", "fork_slider", "swingarm"]:
			if not a.active_box_contacts(name).is_empty():
				box_contact_seen[name] = true
		for c in a.all_box_contacts():
			if c.contact_active:
				worst_gap = minf(worst_gap, c.signed_gap())
	check(a.world.numerical_fault == "", "no numerical fault during the fall")
	check(a.chassis.position.y < 0.45 or not box_contact_seen.is_empty(), "the motorcycle fell (chassis height %s)" % a.chassis.position.y)
	check(box_contact_seen.has("chassis"), "structural parts touched the ground: %s" % [box_contact_seen.keys()])
	check(worst_gap > -0.02, "box penetration stays bounded (%s m)" % worst_gap)
	var speed: float = 0.0
	for body in a.bodies():
		speed = maxf(speed, body.linear_velocity.length())
	check(speed < 1.0, "after settling nothing moves fast (%s m/s)" % speed)
	# Combined inputs on a running machine: no instability.
	var b = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	b.controls.set_inputs(1.0, 0.0, 0.0)
	for k in 240:
		b.world.step()
	var v_before: float = -b.chassis.linear_velocity.z
	check(v_before > 1.0, "throttle accelerates the motorcycle (%s m/s)" % v_before)
	b.controls.set_inputs(0.0, 1.0, 0.0)
	for k in 120:
		b.world.step()
	var v_after: float = -b.chassis.linear_velocity.z
	check(v_after < v_before - 0.3, "braking reduces the forward speed (%s -> %s m/s)" % [v_before, v_after])
	b.controls.set_inputs(0.5, 0.3, 1.0)
	for k in 120:
		b.world.step()
	check(b.world.numerical_fault == "" and b.chassis.position.is_finite(), "combined throttle, brake and steering run without numerical trouble")
	check(b.steering_joint.angle() > 0.05, "steering changed the steering assembly orientation (%s rad)" % b.steering_joint.angle())
