extends "res://Tests/sim_test_case.gd"
## M2.4.2 G: tyres in the motorcycle assembly: registration, references, topology, transactional builder, validation, and the four
## controlled scenarios (straight-line acceleration, braking, cornering response, combined braking + lateral slip) on the assembled
## motorcycle.
##
## ALL DRIVE, BRAKE AND STEERING INPUTS HERE ARE EXTERNAL TEST INPUTS (an axle torque applied to a wheel body, an initial steering
## angle): there is no engine, drivetrain, rider or controller. The assembled motorcycle runs on the TEST-ONLY planar fixture of
## Tests/tire_rig.gd (forward and vertical translation free, lateral translation and rotation of the chassis removed), because a
## riderless two-point-contact motorcycle cannot stand or run straight by itself. Load transfer is therefore not represented.

const Rig = preload("res://Tests/tire_rig.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")
const TireGroundContact = preload("res://Simulation/Motorcycle/tire_ground_contact.gd")
const TireModel = preload("res://Simulation/Motorcycle/tire_model.gd")

const DT: float = 1.0 / 120.0
const G: float = 9.81


class RefusingWorld extends "res://Simulation/Core/sim_world.gd":
	var refuse_contact_number: int = 0
	var contacts_seen: int = 0

	func add_constraint(constraint):
		if "wheel_radius" in constraint:
			contacts_seen += 1
			if contacts_seen == refuse_contact_number:
				return null
		return super.add_constraint(constraint)


func empty(world) -> bool:
	return world.bodies.is_empty() and world.constraints.is_empty() and world.force_providers.is_empty()


func test_g1_both_tyres_are_constructed_registered_and_reference_their_wheel_and_contact() -> void:
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_tires())
	check(a != null and a.has_ground(), "builds with ground and tyres")
	var w = a.world
	check(a.front_tire != null and a.rear_tire != null, "named front and rear tyre references exist")
	check(a.front_tire == a.front_wheel_contact and a.rear_tire == a.rear_wheel_contact, "each tyre IS the contact of its wheel")
	check(a.front_tire.body_b == a.front_wheel and a.rear_tire.body_b == a.rear_wheel, "tyres reference the correct wheels (front/rear not swapped)")
	check(a.front_tire.body_a == a.ground and a.rear_tire.body_a == a.ground, "and the shared static ground")
	check(w.constraints.has(a.front_tire) and w.constraints.has(a.rear_tire), "both tyre contacts are registered in the world")
	check(a.front_tire is TireGroundContact and a.rear_tire is TireGroundContact, "of the tyre-contact class")
	check(a.tires() == [a.front_tire, a.rear_tire], "tires() lists front then rear")
	check_near(a.front_tire.wheel_radius, 0.31, 0.0, "front contact radius")
	check_near(a.rear_tire.wheel_radius, 0.32, 0.0, "rear contact radius")
	check(a.front_tire.tire != a.rear_tire.tire, "the two tyre specifications are independent objects")
	check(a.front_tire.validate().is_empty() and a.rear_tire.validate().is_empty(), "both contacts validate")


func test_g2_existing_topology_ground_and_suspension_are_preserved() -> void:
	var plain = MotorcycleBuilder.build(MotorcyclePrototype.config_with_ground())
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_tires())
	check(a.world.bodies.size() == plain.world.bodies.size() and a.world.bodies.size() == 7, "six dynamic bodies plus the ground, as in M2.4.1")
	check(a.world.constraints.size() == plain.world.constraints.size() and a.world.constraints.size() == 7, "four hinges, one prismatic joint and two contacts")
	check(a.world.force_providers.size() == 2 and a.force_providers() == [a.front_suspension, a.rear_suspension], "both suspension providers are still registered")
	for body in a.bodies():
		check(not body.is_static(), "'%s' is dynamic" % body.body_name)
	check(a.ground.is_static(), "the ground is static")
	check(a.joints().size() == 4 and a.fork_joint != null, "four hinges and the prismatic fork joint")
	check(a.has_ground() and a.ground_contacts() == [a.front_tire, a.rear_tire], "ground_contacts() are the tyre contacts")
	check_near(a.front_tire.friction, MotorcyclePrototype.GROUND_FRICTION, 0.0, "the surface friction reaches the tyre contact")
	check_near(a.front_tire.friction_coefficient_longitudinal(), 0.8, 1.0e-9, "mu_x = grip_x * surface friction")


func test_g3_a_build_without_tyres_is_the_unchanged_generic_m241_assembly() -> void:
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_ground())
	check(a.front_tire == null and a.rear_tire == null and a.tires().is_empty(), "no tyre references")
	check(not (a.front_wheel_contact is TireGroundContact) and a.front_wheel_contact.get_script() == WheelGroundContact, "generic WheelGroundContact (Coulomb friction owner)")
	var mixed = MotorcyclePrototype.config_with_ground()
	mixed.rear_tire = MotorcyclePrototype.tire_spec(mixed.rear_wheel_radius)
	var m = MotorcycleBuilder.build(mixed)
	check(m.front_tire == null and m.rear_tire != null, "tyres are independently optional per wheel")
	check(not (m.front_wheel_contact is TireGroundContact) and m.rear_wheel_contact is TireGroundContact, "so exactly one friction owner exists per wheel")
	check(m.tires() == [m.rear_tire], "tires() lists only the equipped wheel")


func test_g4_configuration_validation_names_every_violation_before_anything_is_built() -> void:
	var bad_cases: Array = []
	var c1 = MotorcyclePrototype.config()
	c1.front_tire = MotorcyclePrototype.tire_spec(0.31)
	bad_cases.append([c1, "requires a ground"])
	var c2 = MotorcyclePrototype.config_with_tires()
	c2.rear_tire = c2.front_tire
	bad_cases.append([c2, "independent"])
	for edit in [["rolling_radius", 0.0], ["rolling_radius", NAN], ["rolling_radius", 0.40], ["rolling_radius", 0.10], ["longitudinal_stiffness", -1.0],
			["lateral_stiffness", 0.0], ["grip_longitudinal", 0.0], ["grip_lateral", INF], ["rolling_resistance", -0.01], ["rolling_resistance", 0.9],
			["rolling_resistance_speed", 0.0], ["low_speed_reference", NAN]]:
		var c = MotorcyclePrototype.config_with_tires()
		c.rear_tire.set(edit[0], edit[1])
		bad_cases.append([c, "rear tire"])
	for case in bad_cases:
		var errors: PackedStringArray = case[0].validate()
		check(not errors.is_empty() and "; ".join(errors).contains(case[1]), "refused: %s" % [case[1]])
		var world = RefusingWorld.new()
		check(MotorcycleBuilder.build(case[0], world) == null and empty(world), "build returns null and leaves the world empty (%s)" % [case[1]])
	var front = MotorcyclePrototype.config_with_tires()
	front.front_tire.rolling_radius = 0.05
	check("; ".join(front.validate()).contains("front tire"), "the message names the offending wheel")
	check(MotorcyclePrototype.config_with_tires().validate().is_empty(), "the prototype tyre configuration is valid")


func test_g5_builder_failure_rolls_back_every_tyre_object_and_a_valid_build_follows() -> void:
	for refuse in [1, 2]:
		var world = RefusingWorld.new()
		world.refuse_contact_number = refuse
		check(MotorcycleBuilder.build(MotorcyclePrototype.config_with_tires(), world) == null, "build fails when contact #%d is refused" % refuse)
		check(empty(world), "no orphaned body, hinge, joint, tyre contact or provider remains (#%d)" % refuse)
		world.refuse_contact_number = 0
		var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_tires(), world)
		check(a != null and a.front_tire != null and a.rear_tire != null and world.bodies.size() == 7 and world.constraints.size() == 7, "a valid build into the same world then succeeds (#%d)" % refuse)
		check(world.step(), "and the world steps")


func test_g6_removing_the_ground_clears_the_tyre_references_and_disabling_tyres_keeps_the_normal_contact() -> void:
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_tires())
	a.set_tire_forces_enabled(false)
	check(not a.front_tire.tire_forces_enabled and not a.rear_tire.tire_forces_enabled, "tyre forces disabled")
	check(a.front_tire.enabled and a.rear_tire.enabled, "while the contacts stay enabled")
	a.remove_ground()
	check(a.front_tire == null and a.rear_tire == null and a.tires().is_empty(), "tyre references cleared with the ground")
	check(a.world.constraints.size() == 5 and a.world.bodies.size() == 6, "no orphaned tyre contact or ground body")
	check(a.world.force_providers.size() == 2, "suspension providers untouched")


func test_g7_loads_and_suspension_still_work_with_tyres() -> void:
	var rig: Dictionary = Rig.carriage_motorcycle()
	var a = rig.assembly
	var w = rig.world
	for i in 480:
		w.step()
	check(w.numerical_fault == "", "no numerical fault")
	var total: float = a.front_tire.normal_load() + a.rear_tire.normal_load()
	check_near(total, a.total_mass() * G, 0.005 * a.total_mass() * G, "the two tyre loads carry the motorcycle's weight (%s of %s N)" % [total, a.total_mass() * G])
	check(a.front_tire.normal_load() > 100.0 and a.rear_tire.normal_load() > 100.0, "both wheels loaded")
	check(a.fork_joint.translation() < -1.0e-4, "the fork is compressed by the load (the spring-damper provider is acting: %s m)" % a.fork_joint.translation())
	check(a.front_tire.friction_utilization() < 0.05 and a.rear_tire.friction_utilization() < 0.05, "no spurious traction at rest")
	check(a.max_anchor_error() < 5.0e-3, "hinges hold (%s m)" % a.max_anchor_error())


func test_g8_straight_line_acceleration_with_an_external_rear_wheel_torque() -> void:
	var rig: Dictionary = Rig.carriage_motorcycle()
	var a = rig.assembly
	var w = rig.world
	for i in 240:
		w.step()
	var dynamic: Array = a.bodies() + [rig.carriage]
	var u: Vector3 = a.rear_wheel.orientation * Vector3.RIGHT
	var momentum_error: float = 0.0
	var max_util: float = 0.0
	for i in 240:
		Rig.drive_torque(a.rear_wheel, u, 60.0)  # EXTERNAL TEST INPUT
		var p0: Vector3 = Rig.total_linear_momentum(dynamic)
		w.step()
		var dp: Vector3 = Rig.total_linear_momentum(dynamic) - p0
		var impulse: Vector3 = (a.front_tire.tangential_force_world() + a.rear_tire.tangential_force_world()) * DT
		momentum_error = maxf(momentum_error, absf(dp.z - impulse.z))
		max_util = maxf(max_util, maxf(a.front_tire.friction_utilization(), a.rear_tire.friction_utilization()))
	var forward: float = -rig.carriage.linear_velocity.z
	check(forward > 0.5, "the motorcycle accelerates forward (%s m/s after 2 s)" % forward)
	check(a.rear_tire.longitudinal_force() > 50.0, "the driven rear tyre pushes forward (%s N)" % a.rear_tire.longitudinal_force())
	check(a.rear_tire.current_slip().slip_x > 0.0, "with positive driving slip")
	check(absf(a.front_tire.current_slip().slip_x) < 0.01, "the undriven front wheel stays near rolling (%s)" % a.front_tire.current_slip().slip_x)
	check(a.rear_tire.longitudinal_force() > a.front_tire.longitudinal_force(), "the rear tyre carries the drive force")
	check(momentum_error < 2.0e-3, "momentum balance: the machine's momentum change equals the tyre impulses every step (worst %s N s)" % momentum_error)
	check(max_util <= 1.0 + 1.0e-5, "inside the friction envelope (max %s)" % max_util)
	check(a.rear_tire.normal_load() > 100.0, "still loaded")


func set_rolling(rig: Dictionary, speed: float) -> void:
	var a = rig.assembly
	for body in a.bodies() + [rig.carriage]:
		body.linear_velocity = Vector3(0.0, 0.0, -speed)
	for pair in [[a.front_wheel, 0.31], [a.rear_wheel, 0.32]]:
		var u: Vector3 = pair[0].orientation * Vector3.RIGHT
		pair[0].angular_velocity = -u * (speed / pair[1])


func test_g9_braking_with_external_wheel_torques() -> void:
	var rig: Dictionary = Rig.carriage_motorcycle()
	var a = rig.assembly
	var w = rig.world
	set_rolling(rig, 8.0)
	var uf: Vector3 = a.front_wheel.orientation * Vector3.RIGHT
	var ur: Vector3 = a.rear_wheel.orientation * Vector3.RIGHT
	var max_util: float = 0.0
	for i in 120:
		Rig.drive_torque(a.front_wheel, uf, -40.0)  # EXTERNAL TEST INPUTS
		Rig.drive_torque(a.rear_wheel, ur, -40.0)
		w.step()
		max_util = maxf(max_util, maxf(a.front_tire.friction_utilization(), a.rear_tire.friction_utilization()))
	check(-rig.carriage.linear_velocity.z < 8.0 - 0.8, "the motorcycle slows down (%s m/s)" % -rig.carriage.linear_velocity.z)
	check(a.front_tire.longitudinal_force() < -50.0 and a.rear_tire.longitudinal_force() < -50.0, "both tyres push backward (%s, %s N)" % [a.front_tire.longitudinal_force(), a.rear_tire.longitudinal_force()])
	check(a.front_tire.current_slip().slip_x < 0.0 and a.rear_tire.current_slip().slip_x < 0.0, "with negative (braking) slip")
	check(max_util <= 1.0 + 1.0e-5, "inside the envelope (max %s)" % max_util)
	check(w.numerical_fault == "", "no numerical fault")


## Rotates the steering assembly, fork slider and front wheel about the steering axis through the steering-head pivot by `angle` (rad)
## BEFORE the first step: a controlled steering input (test setup). The hinge and the prismatic joint stay satisfied (the rotation is
## about the hinge axis); the wheel keeps rolling about its new axle.
func steer(rig: Dictionary, angle: float) -> void:
	var a = rig.assembly
	var spec = a.config.steering_head
	var q: Quaternion = Quaternion(spec.axis.normalized(), angle)
	for body in [a.steering, a.fork_slider, a.front_wheel]:
		body.position = spec.point + q * (body.position - spec.point)
		body.orientation = (q * body.orientation).normalized()
		body.update_world_inertia()


func test_g10_cornering_response_to_a_steering_input_is_a_restoring_lateral_force() -> void:
	var speed: float = 8.0
	var results: Array = []
	for angle in [0.08, -0.08]:
		var rig: Dictionary = Rig.carriage_motorcycle()
		var a = rig.assembly
		for i in 240:
			rig.world.step()  # settle under gravity so the tyres are loaded
		set_rolling(rig, speed)
		steer(rig, angle)
		var u: Vector3 = a.front_wheel.orientation * Vector3.RIGHT
		a.front_wheel.angular_velocity = -u * (speed / 0.31)
		# M2.4.2.1: rotating the front end by hand about the raked steering axis lifts the contact point by 0.15 mm, more than the 0.1 mm touch
		# tolerance, and a tyre does not act across an air gap (the measurement step used to see a force across it). Step until the wheel touches again
		# (one step, the load of the gap closing) and measure from there; the expectation and every tolerance below are unchanged.
		var settle_steps: int = 0
		while a.front_tire.signed_gap() > a.front_tire.TOUCH_TOLERANCE and settle_steps < 5:
			rig.world.step()
			settle_steps += 1
		check(settle_steps <= 2 and a.front_tire.signed_gap() <= a.front_tire.TOUCH_TOLERANCE, "the front wheel touches again within two steps of the steering input (%d)" % settle_steps)
		var e_x: Vector3 = TireModel.axes(u, Vector3.UP).longitudinal
		var heading_error: float = acos(clampf(e_x.dot(Vector3(0.0, 0.0, -1.0)), -1.0, 1.0))
		var front = a.front_tire
		var s: Dictionary = front.current_slip()  # start-of-step slip, before the solve reduces it
		rig.world.step()
		# Hand derivation: velocity (0,0,-v) split into the wheel's axes: hub speed v cos(eps), sideways v sin(eps); the wheel spins at v / R.
		var hub: float = speed * cos(heading_error)
		var expected: float = speed * sin(heading_error) / sqrt(hub * hub + 0.25)
		check(heading_error > 0.03, "the steering input turns the front wheel's heading by %s rad" % heading_error)
		check_near(absf(s.slip_y), expected, 0.15 * expected, "front lateral slip = v sin(eps) / sqrt(v^2 cos^2(eps) + v_ref^2) (%s vs %s)" % [absf(s.slip_y), expected])
		check(front.lateral_force() * s.slip_y > 0.0, "force has the sign of the lateral slip (restoring)")
		check(front.lateral_force() * s.tangential_velocity_y < 0.0, "and opposes the sideways velocity of the contact point")
		check(absf(front.lateral_force()) > 20.0, "a clear cornering force (%s N)" % front.lateral_force())
		check(absf(a.rear_tire.lateral_force()) < 0.2 * absf(front.lateral_force()), "while the unsteered rear tyre carries little")
		results.append(front.lateral_force())
	check(results[0] * results[1] < 0.0, "steering the other way reverses the cornering force")
	check_near(absf(results[0]), absf(results[1]), 0.15 * absf(results[0]), "with similar magnitude")


func test_g11_braking_reduces_the_cornering_force_of_the_same_front_tyre() -> void:
	var forces: Array = []
	for brake in [0.0, 120.0]:
		var rig: Dictionary = Rig.carriage_motorcycle()
		var a = rig.assembly
		for i in 240:
			rig.world.step()
		set_rolling(rig, 8.0)
		steer(rig, 0.08)
		var u: Vector3 = a.front_wheel.orientation * Vector3.RIGHT
		a.front_wheel.angular_velocity = -u * (8.0 / 0.31)
		for i in 3:
			Rig.drive_torque(a.front_wheel, u, -brake)  # EXTERNAL TEST INPUT
			rig.world.step()
		forces.append([absf(a.front_tire.lateral_force()), absf(a.front_tire.longitudinal_force()), a.front_tire.friction_utilization()])
	check(forces[1][1] > forces[0][1] + 50.0, "braking adds longitudinal force (%s -> %s N)" % [forces[0][1], forces[1][1]])
	check(forces[1][0] < forces[0][0], "and reduces the lateral force of the same steered tyre (%s -> %s N)" % [forces[0][0], forces[1][0]])
	check(forces[1][2] <= 1.0 + 1.0e-5 and forces[0][2] <= 1.0 + 1.0e-5, "both inside the friction envelope")
	check(forces[1][2] > forces[0][2], "with higher utilisation under braking and cornering together")


func test_g12_disabling_the_tyres_leaves_a_frictionless_rolling_machine_that_still_carries_its_weight() -> void:
	var rig: Dictionary = Rig.carriage_motorcycle()
	var a = rig.assembly
	set_rolling(rig, 5.0)
	a.set_tire_forces_enabled(false)
	for i in 240:
		rig.world.step()
	check_near(-rig.carriage.linear_velocity.z, 5.0, 1.0e-3, "no tyre and no generic friction slows it")
	# The fixture's 1 kg carriage hangs on the chassis slide, so the wheels carry (m + 1 kg) g.
	check_near(a.front_tire.normal_load() + a.rear_tire.normal_load(), (a.total_mass() + 1.0) * G, 0.003 * a.total_mass() * G, "the unilateral normal contact still carries the weight")
	check_near(a.front_tire.longitudinal_force() + a.rear_tire.longitudinal_force() + a.front_tire.lateral_force() + a.rear_tire.lateral_force(), 0.0, 0.0, "all tangential forces are zero")
