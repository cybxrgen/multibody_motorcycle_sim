extends "res://Tests/sim_test_case.gd"
## M2.5.3 Stage D: the distributed contact patch in the ASSEMBLED motorcycle. Every test builds the motorcycle through MotorcycleBuilder
## (MotorcyclePrototype.config_with_patches(), a fresh configuration each time) and reads the assembly, never a patch built by hand.
## Groups: builder integration (d1-d5), front/rear independence (d6-d7), wheel motion and geometry (d8-d11), contact lifecycle (d12-d13),
## transactional construction and cleanup (d14-d16), roll-related observation (d17).
## The roll observation (d17) is a deterministic MEASUREMENT of how the distributed contact connects to the assembly; the riderless machine
## is not self-balancing and nothing here asserts upright operation.

const Rig = preload("res://Tests/patch_rig.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const TirePatchNormalSupport = preload("res://Simulation/Motorcycle/tire_patch_normal_support.gd")
const TireGroundContact = preload("res://Simulation/Motorcycle/tire_ground_contact.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")
const TirePatchSpec = preload("res://Simulation/Motorcycle/motorcycle_tire_patch_spec.gd")
const TireGeometry = preload("res://Simulation/Motorcycle/tire_geometry.gd")
const TireModel = preload("res://Simulation/Motorcycle/tire_model.gd")

const WEIGHT: float = 186.0 * Rig.G


## A world that refuses the n-th registration of a distributed patch (recognised by its stiffness density).
class PatchRefusingWorld extends "res://Simulation/Core/sim_world.gd":
	var refuse_patch_number: int = 0
	var patches_seen: int = 0

	func add_constraint(constraint):
		if "stiffness_density" in constraint:
			patches_seen += 1
			if patches_seen == refuse_patch_number:
				return null
		return super.add_constraint(constraint)


func world_is_empty(world) -> bool:
	return world.bodies.is_empty() and world.constraints.is_empty() and world.force_providers.is_empty()


func patches_in(world) -> Array:
	var found: Array = []
	for constraint in world.constraints:
		if "stiffness_density" in constraint:
			found.append(constraint)
	return found


# --- Builder integration -------------------------------------------------------------------------------------------------------

func test_d1_the_builder_creates_and_registers_one_independent_patch_per_wheel() -> void:
	var config = MotorcyclePrototype.config_with_patches()
	var a = MotorcycleBuilder.build(config)
	check(a != null and a.has_ground(), "builds with ground and patches")
	var f = a.front_tire_patch
	var r = a.rear_tire_patch
	check(f is TirePatchNormalSupport and r is TirePatchNormalSupport and f != r, "one TirePatchNormalSupport per wheel, two distinct objects")
	check(f.body_b == a.front_wheel and r.body_b == a.rear_wheel, "each patch references its own wheel (front/rear not swapped)")
	check(f.body_a == a.ground and r.body_a == a.ground, "and the shared static ground")
	check(a.world.constraints.has(f) and a.world.constraints.has(r) and patches_in(a.world).size() == 2, "exactly the two patches are registered in the world")
	check(a.front_wheel_contact == f and a.rear_wheel_contact == r and a.front_tire == f and a.rear_tire == r, "a patch IS its wheel's ground contact and tyre owner")
	check(a.ground_contacts() == [f, r] and a.tires() == [f, r], "ground_contacts() and tires() list front then rear")
	check(f.validate().is_empty() and r.validate().is_empty(), "both patches validate")
	check_near(f.wheel_radius, config.front_wheel_radius, 0.0, "front patch wheel radius from the configuration")
	check_near(r.wheel_radius, config.rear_wheel_radius, 0.0, "rear patch wheel radius from the configuration")
	check(f.geometry == a.front_tire_geometry and r.geometry == a.rear_tire_geometry, "each patch reads its own wheel's geometry object of the assembly (one owner)")
	check(f.geometry != config.front_tire_geometry and r.geometry != config.rear_tire_geometry, "which is a copy of the configuration's")
	check(f.lateral_count == config.front_tire_patch.lateral_count and r.circumferential_count == config.rear_tire_patch.circumferential_count, "grid from the patch specification")
	check_near(f.stiffness_density, config.front_tire_patch.stiffness_density, 0.0, "stiffness density from the patch specification")
	check_near(r.damping_density, config.rear_tire_patch.damping_density, 0.0, "damping density from the patch specification")
	check_near(f.max_compression, config.front_tire_patch.max_compression, 0.0, "bottoming compression from the patch specification")
	check(f.plane_normal == config.ground.normal and f.plane_point == config.ground.point, "plane from the ground specification")
	check_near(f.surface_friction, config.ground.friction, 0.0, "surface friction from the ground specification")
	check(f.tire != null and r.tire != null and f.tire != r.tire and f.tire != config.front_tire, "each patch owns its own COPY of its wheel's tyre")
	check_near(f.tire.longitudinal_stiffness, config.front_tire.longitudinal_stiffness, 0.0, "with the configured tyre parameters, unchanged")
	check(f.tire_forces_enabled and r.tire_forces_enabled, "tyre forces are enabled")
	check(f._rows_by_index != r._rows_by_index or (f._rows_by_index.is_empty() and r._rows_by_index.is_empty()), "no shared row state")


func test_d2_no_legacy_one_point_contact_and_exactly_one_tangential_owner_per_wheel() -> void:
	var a = Rig.assemble()
	var legacy: int = 0
	var ground_contacts_per_wheel: Dictionary = {a.front_wheel: 0, a.rear_wheel: 0}
	var owners_per_wheel: Dictionary = {a.front_wheel: 0, a.rear_wheel: 0}
	for constraint in a.world.constraints:
		if constraint is TireGroundContact or constraint is WheelGroundContact:
			legacy += 1
		if ground_contacts_per_wheel.has(constraint.body_b) and constraint.body_a == a.ground:
			ground_contacts_per_wheel[constraint.body_b] += 1
		if owners_per_wheel.has(constraint.body_b) and "tire" in constraint and constraint.tire != null:
			owners_per_wheel[constraint.body_b] += 1
	check(legacy == 0, "no one-point TireGroundContact / WheelGroundContact is registered in the assembled motorcycle")
	check(ground_contacts_per_wheel[a.front_wheel] == 1 and ground_contacts_per_wheel[a.rear_wheel] == 1, "exactly one ground contact per wheel (the patch)")
	check(owners_per_wheel[a.front_wheel] == 1 and owners_per_wheel[a.rear_wheel] == 1, "exactly one tangential tyre-force owner per wheel")
	check(a.world.constraints.size() == 4 + 1 + 2 + 32, "four hinges, one prismatic joint, two patches and 32 box corners: nothing else")
	check(a.front_tire_patch.tire_forces_enabled, "the patch owns the tangential response")
	# No other constraint reaches a wheel except its axle hinge (fork slider / swingarm).
	for constraint in a.world.constraints:
		for wheel in [a.front_wheel, a.rear_wheel]:
			if constraint.body_b == wheel or constraint.body_a == wheel:
				check(constraint == a.front_tire_patch or constraint == a.rear_tire_patch or constraint == a.front_axle_joint or constraint == a.rear_axle_joint,
					"a wheel is touched only by its axle hinge and its patch")


func test_d3_rolling_resistance_is_applied_once_per_wheel() -> void:
	var a = Rig.assemble()
	Rig.roll_forward(a, 2.0)
	Rig.step(a, 40)
	var owners: int = 0
	for constraint in a.world.constraints:
		if constraint.has_method("rolling_resistance_torque"):
			owners += 1
	check(owners == 2, "exactly two rolling-resistance owners in the world: one per wheel")
	for side in ["front", "rear"]:
		var patch = Rig.patch_of(a, side)
		var wheel = Rig.wheel_of(a, side)
		var u: Vector3 = wheel.orientation * Vector3.RIGHT
		var spin: float = -wheel.angular_velocity.dot(u)
		var expected: float = patch.tire.rolling_resistance * patch.normal_load() * patch.tire.rolling_radius * (spin * patch.tire.rolling_radius / patch.tire.rolling_resistance_speed) \
				/ sqrt(1.0 + pow(spin * patch.tire.rolling_radius / patch.tire.rolling_resistance_speed, 2.0))
		check(patch.normal_load() > 100.0 and spin > 3.0, "%s: loaded and rolling" % side)
		check_near(patch.rolling_resistance_torque(), expected, 0.03 * expected, "%s: rolling-resistance moment is c_rr N R_eff psi once, not twice" % side)


func test_d4_wheels_without_a_patch_specification_keep_the_one_point_contact() -> void:
	var plain = MotorcycleBuilder.build(MotorcyclePrototype.config_with_tires())
	check(plain.front_tire is TireGroundContact and plain.rear_tire is TireGroundContact, "no patch specification: the one-point tyre contacts, as before")
	check(plain.front_tire_patch == null and plain.rear_tire_patch == null and patches_in(plain.world).is_empty(), "and no patch")
	var mixed_config = MotorcyclePrototype.config_with_patches()
	mixed_config.rear_tire_patch = null
	var mixed = MotorcycleBuilder.build(mixed_config)
	check(mixed.front_tire is TirePatchNormalSupport and mixed.rear_tire is TireGroundContact, "a patch on the front wheel only leaves the rear one-point contact")
	check(mixed.front_tire_patch != null and mixed.rear_tire_patch == null, "patch references follow the configuration per wheel")
	var count: int = 0
	for constraint in mixed.world.constraints:
		if constraint.body_b == mixed.front_wheel and constraint.body_a == mixed.ground:
			count += 1
	check(count == 1, "the patch wheel has no second (one-point) contact next to its patch")


func test_d5_configuration_rules_for_patches() -> void:
	var errors_for = func(mutate: Callable) -> String:
		var config = MotorcyclePrototype.config_with_patches()
		mutate.call(config)
		return "; ".join(config.validate())
	check(MotorcyclePrototype.config_with_patches().validate().is_empty(), "the prototype patch configuration is valid")
	check("requires a ground" in errors_for.call(func(c): c.ground = null; c.front_tire = null; c.rear_tire = null), "a patch needs a ground")
	check("friction must be > 0" in errors_for.call(func(c): c.ground.friction = 0.0), "a patch needs a ground friction > 0")
	check("requires the wheel's tyre" in errors_for.call(func(c): c.front_tire = null), "a patch needs the wheel's tyre")
	check("requires the wheel's tyre geometry" in errors_for.call(func(c): c.rear_tire_geometry = null), "a patch needs the wheel's tyre geometry")
	check("independent specification objects" in errors_for.call(func(c): c.rear_tire_patch = c.front_tire_patch), "front and rear patch specifications are distinct objects")
	check("lateral sample count" in errors_for.call(func(c): c.front_tire_patch.lateral_count = 1), "the grid is validated (the Stage A bounds)")
	check("stiffness density" in errors_for.call(func(c): c.rear_tire_patch.stiffness_density = 0.0), "the stiffness density is validated")
	check("damping density" in errors_for.call(func(c): c.rear_tire_patch.damping_density = -1.0), "the damping density is validated")
	check("maximum compression" in errors_for.call(func(c): c.front_tire_patch.max_compression = 0.2), "the bottoming compression is bounded by half the wheel radius")
	check(MotorcycleBuilder.build(_config_with(func(c): c.front_tire_patch.lateral_count = 1)) == null, "the builder refuses an invalid patch configuration")
	var spec: TirePatchSpec = MotorcyclePrototype.patch_spec()
	var copy: TirePatchSpec = spec.duplicate_spec()
	copy.lateral_count = 5
	check(spec.lateral_count == MotorcyclePrototype.PATCH_LATERAL_COUNT and copy != spec, "duplicate_spec() returns an independent copy")


func _config_with(mutate: Callable):
	var config = MotorcyclePrototype.config_with_patches()
	mutate.call(config)
	return config


# --- Front / rear independence ---------------------------------------------------------------------------------------------------

func test_d6_front_and_rear_configuration_are_independent() -> void:
	var config = MotorcyclePrototype.config_with_patches()
	config.front_tire_patch = TirePatchSpec.new(5, 24, 2.0e6, 5.0e4, 0.04)
	config.rear_tire_patch = TirePatchSpec.new(11, 64, 6.0e6, 1.5e5, 0.06)
	config.front_tire.longitudinal_stiffness = 11.0
	var a = MotorcycleBuilder.build(config)
	var f = a.front_tire_patch
	var r = a.rear_tire_patch
	check(f.lateral_count == 5 and f.circumferential_count == 24 and r.lateral_count == 11 and r.circumferential_count == 64, "front and rear sampling configurations are independent")
	check_near(f.stiffness_density, 2.0e6, 0.0, "front stiffness")
	check_near(r.stiffness_density, 6.0e6, 0.0, "rear stiffness")
	check_near(f.damping_density, 5.0e4, 0.0, "front damping")
	check_near(r.damping_density, 1.5e5, 0.0, "rear damping")
	check_near(f.max_compression, 0.04, 0.0, "front bottoming")
	check_near(r.max_compression, 0.06, 0.0, "rear bottoming")
	check_near(f.tire.longitudinal_stiffness, 11.0, 0.0, "the front tyre parameters reach only the front patch")
	check_near(r.tire.longitudinal_stiffness, MotorcyclePrototype.TIRE_LONGITUDINAL_STIFFNESS, 0.0, "the rear tyre keeps its own")
	# Editing the configuration after the build reaches no running patch.
	config.front_tire_patch.stiffness_density = 1.0e5
	config.rear_tire.lateral_stiffness = 99.0
	check_near(f.stiffness_density, 2.0e6, 0.0, "a later edit of the specification does not reach the patch")
	check_near(r.tire.lateral_stiffness, MotorcyclePrototype.TIRE_LATERAL_STIFFNESS, 0.0, "nor the patch's tyre copy")
	# Geometry: front edits stay at the front, rear edits at the rear.
	var rear_width: float = r.geometry.width
	var rear_crown: float = r.geometry.crown_radius
	a.front_tire_geometry.crown_radius = 0.09
	a.front_tire_geometry.width = 0.10
	check_near(f.geometry.crown_radius, 0.09, 0.0, "front geometry edit reaches the front patch")
	check_near(r.geometry.width, rear_width, 0.0, "and not the rear patch (width)")
	check_near(r.geometry.crown_radius, rear_crown, 0.0, "and not the rear patch (crown)")
	var front_width: float = f.geometry.width
	a.rear_tire_geometry.crown_radius = 0.12
	check_near(f.geometry.width, front_width, 0.0, "rear edit leaves the front patch's geometry alone")
	check_near(f.geometry.crown_radius, 0.09, 0.0, "(crown)")
	check(f.geometry != r.geometry, "the two geometry objects are distinct")
	Rig.step(a, 120)
	check(f.is_touching() and r.is_touching(), "both patches still work with the edited geometries and different grids")
	check(f.carried_row_count() <= 5 * 24 and r.carried_row_count() <= 11 * 64, "each patch keeps its own row state within its own grid")


func test_d7_the_wheels_occupy_different_contact_states_and_keep_independent_loads() -> void:
	var a = Rig.assemble()
	var f = a.front_tire_patch
	var r = a.rear_tire_patch
	var full: int = MotorcyclePrototype.PATCH_LATERAL_COUNT * MotorcyclePrototype.PATCH_CIRCUMFERENTIAL_COUNT
	Rig.step(a, 1)
	check(not f.is_touching() and not r.is_touching() and f.normal_load() == 0.0 and r.normal_load() == 0.0, "first step: the reference pose carries no physical load")
	Rig.step(a, 1)
	check(r.is_touching() and not f.is_touching(), "the rear wheel touches while the front wheel (3 mm above) is still separated")
	check(r.normal_load() > 0.0 and f.normal_load() == 0.0 and f.active_sample_count() == 0 and f.carried_row_count() == 0, "so only the rear patch carries load, rows and state")
	check(f.longitudinal_force() == 0.0 and f.lateral_force() == 0.0 and f.rolling_resistance_torque() == 0.0, "the separated front patch applies no traction or rolling resistance")
	Rig.step(a, 238)
	check(f.is_touching() and r.is_touching(), "later both touch")
	check(f.normal_load() > 0.0 and r.normal_load() > 0.0 and absf(f.normal_load() - r.normal_load()) > 20.0, "with different physical loads")
	check_near(f.normal_load() + r.normal_load(), WEIGHT, 0.05 * WEIGHT, "which together carry the machine's weight")
	check(f.active_sample_count() != r.active_sample_count() and f.active_sample_count() < full and r.active_sample_count() < full, "and different partial footprints")
	check(f.centre_of_pressure().z < r.centre_of_pressure().z - 1.0, "at their own wheels")
	check(absf(f.contact_area() - r.contact_area()) > 1.0e-4, "with different contact areas (different tread widths)")


# --- Wheel motion and geometry ---------------------------------------------------------------------------------------------------

func test_d8_patches_follow_the_wheels_through_suspension_travel_and_wheel_rotation() -> void:
	var a = Rig.assemble()
	Rig.step(a, 240)
	check(a.fork_joint.translation() < -0.03, "the front suspension is compressed (travel %s)" % a.fork_joint.translation())
	check(absf(a.swingarm_joint.angle()) > 0.1, "the rear suspension has moved the swingarm (%s rad)" % a.swingarm_joint.angle())
	for side in ["front", "rear"]:
		check(Rig.patch_of(a, side).active_sample_count() > 5, "%s: contact with the suspension compressed" % side)
		var error: float = Rig.start_of_step_pose_error(a, side)
		check(error < 2.0e-6, "%s: the patch samples are those of the wheel pose at the start of the step (error %s m)" % [side, error])
	# Wheel rotation: spin both wheels; the material samples turn with the wheel.
	var rear_before: Quaternion = a.rear_wheel.orientation
	Rig.roll_forward(a, 4.0)
	Rig.step(a, 6)
	var turned: float = (rear_before.inverse() * a.rear_wheel.orientation).get_angle()
	check(turned > 0.15, "the rear wheel has turned about its axle (%s rad)" % turned)
	for side in ["front", "rear"]:
		var error: float = Rig.start_of_step_pose_error(a, side)
		check(error < 2.0e-6, "%s: samples follow the rotating wheel (error %s m)" % [side, error])
	# Suspension compression changes the front patch location with the wheel.
	var before: Vector3 = a.front_tire_patch.centre_of_pressure()
	var wheel_before: Vector3 = a.front_wheel.position
	Rig.step(a, 30)
	var moved: Vector3 = a.front_wheel.position - wheel_before
	var cop_moved: Vector3 = a.front_tire_patch.centre_of_pressure() - before
	check((moved - cop_moved).length() < 0.02 + 0.5 * moved.length(), "the front contact moves with the front wheel")


func test_d9_the_front_patch_follows_steering() -> void:
	var a = Rig.assemble()
	Rig.step(a, 120)
	a.controls.steering = 1.0
	Rig.step(a, 60)
	var angle: float = a.steering_joint.angle()
	check(absf(angle) > 0.2, "the steering assembly has turned (%s rad)" % angle)
	var u_front: Vector3 = a.front_wheel.orientation * Vector3.RIGHT
	var u_rear: Vector3 = a.rear_wheel.orientation * Vector3.RIGHT
	check(u_front.angle_to(u_rear) > 0.1, "the front axle is no longer parallel to the rear axle (%s rad)" % u_front.angle_to(u_rear))
	# The patch frame is the one of the START of the last step, so the axle is read before that step.
	var u_start: Vector3 = a.front_wheel.orientation * Vector3.RIGHT
	var error: float = Rig.start_of_step_pose_error(a, "front")
	check(error < 2.0e-6, "the front patch samples follow the steered wheel (error %s m)" % error)
	var front_axes: Dictionary = TireModel.axes(u_start, Vector3.UP)
	check_vec3_near(a.front_tire_patch.longitudinal_axis(), front_axes.longitudinal, 1.0e-5, "the front tangent frame follows the wheel (yaw and camber)")
	var rear_error: float = Rig.start_of_step_pose_error(a, "rear")
	check(rear_error < 2.0e-6, "the rear patch is unaffected in its own frame (error %s m)" % rear_error)
	check(a.front_tire_patch.longitudinal_axis().angle_to(a.rear_tire_patch.longitudinal_axis()) > 0.05, "the two wheels have different rolling directions")
	check(a.front_tire_patch.is_touching() and a.front_tire_patch.normal_load() > 100.0, "the steered front patch carries load")


func test_d10_the_patches_follow_the_orientation_of_the_motorcycle() -> void:
	var a = Rig.assemble()
	Rig.move_rigidly(a, Quaternion(Vector3.UP, 0.35) * Quaternion(Vector3.BACK, 0.08), Vector3(0.4, -0.004, -0.7))
	Rig.step(a, 4)
	for side in ["front", "rear"]:
		var patch = Rig.patch_of(a, side)
		check(patch.is_touching() and patch.normal_load() > 0.0, "%s: the yawed and rolled machine touches" % side)
		var error: float = Rig.start_of_step_pose_error(a, side)
		check(error < 2.0e-6, "%s: samples follow the new orientation (error %s m)" % [side, error])
	var heading: Vector3 = a.rear_tire_patch.longitudinal_axis()
	var expected: Vector3 = Quaternion(Vector3.UP, 0.35) * Vector3(0.0, 0.0, -1.0)
	check(heading.angle_to(expected) < 0.03, "the tangent frame turned with the machine (yaw 0.35 rad)")
	check(a.rear_tire_patch.centre_of_pressure().x > 0.2, "and the contact moved with the machine to its new place")


func test_d11_camber_moves_the_contact_laterally_in_the_assembled_motorcycle() -> void:
	var upright = Rig.assemble()
	Rig.step(upright, 240)
	for side in ["front", "rear"]:
		var patch = Rig.patch_of(upright, side)
		check(absf(patch.centre_of_pressure().x - Rig.wheel_of(upright, side).position.x) < 2.0e-3, "%s upright: the contact lies in the wheel's centre plane" % side)
	for sign in [1.0, -1.0]:
		var phi: float = 0.15 * sign
		var a = Rig.assemble()
		Rig.roll(a, phi, 0.01)
		var camber: float = asin((a.rear_wheel.orientation * Vector3.RIGHT).y)
		check_near(camber, phi, 1.0e-3, "the rolled machine has the intended camber")
		Rig.step(a, 1)
		for side in ["front", "rear"]:
			var patch = Rig.patch_of(a, side)
			var wheel = Rig.wheel_of(a, side)
			var radius: float = patch.wheel_radius
			var crown: float = patch.geometry.crown_radius
			var migration: float = patch.centre_of_pressure().x - wheel.position.x
			var expected: float = (radius - crown) * sin(phi)
			check(patch.is_touching(), "%s %+.2f: touching" % [side, phi])
			check_near(migration, expected, 0.06 * absf(expected), "%s camber %+.2f: the contact lies (R - rho) sin(camber) = %.4f m to the side of the wheel centre plane" % [side, phi, expected])
			var lowest: Dictionary = TireGeometry.lowest_point(wheel.position, wheel.orientation, patch.geometry, radius, patch.plane_point, patch.plane_normal)
			check(lowest.valid and absf(patch.centre_of_pressure().x - lowest.wheel_point.x) < 4.0e-3, "%s: and agrees with the lowest tyre point of the Stage A geometry" % side)
			check(absf(migration) < 0.9 * radius * absf(sin(phi)), "%s: less than the zero-width centreline wheel would put it (R sin camber)" % side)


# --- Contact lifecycle ---------------------------------------------------------------------------------------------------------

func test_d12_a_positive_gap_produces_no_physical_load_traction_or_rolling_resistance() -> void:
	var a = Rig.assemble()
	Rig.shift(a, Vector3(0.0, 0.002, 0.0))
	Rig.roll_forward(a, 3.0)
	for body in a.bodies():
		body.linear_velocity += Vector3(0.0, -1.0, 0.0)
	var checked_steps: int = 0
	for k in 12:
		var lowest_gap: float = INF
		for side in ["front", "rear"]:
			var wheel = Rig.wheel_of(a, side)
			var patch = Rig.patch_of(a, side)
			var lowest: Dictionary = TireGeometry.lowest_point(wheel.position, wheel.orientation, patch.geometry, patch.wheel_radius, patch.plane_point, patch.plane_normal)
			lowest_gap = minf(lowest_gap, lowest.gap)
		a.world.step()
		if lowest_gap > 1.0e-6:
			checked_steps += 1
			for side in ["front", "rear"]:
				var patch = Rig.patch_of(a, side)
				var gap_of_side: float = TireGeometry.lowest_point(Rig.wheel_of(a, side).position, Rig.wheel_of(a, side).orientation, patch.geometry, patch.wheel_radius, patch.plane_point, patch.plane_normal).gap
				if gap_of_side > 1.0e-6 and patch.normal_load() == 0.0:
					check(patch.longitudinal_force() == 0.0 and patch.lateral_force() == 0.0 and patch.rolling_resistance_torque() == 0.0 and patch.tangential_force_world() == Vector3.ZERO,
						"%s: no traction or rolling resistance without physical load (step %d)" % [side, k])
	check(checked_steps >= 1, "at least one step started with both wheels above the plane")
	# The wheels approach at about 1 m/s: the first step starts 2 mm above the plane and ends in penetration, yet carries no load.
	var b = Rig.assemble()
	Rig.shift(b, Vector3(0.0, 0.002, 0.0))
	Rig.roll_forward(b, 3.0)
	for body in b.bodies():
		body.linear_velocity += Vector3(0.0, -1.0, 0.0)
	Rig.step(b, 1)
	for side in ["front", "rear"]:
		var patch = Rig.patch_of(b, side)
		check(not patch.is_touching() and patch.normal_load() == 0.0 and patch.active_sample_count() == 0 and patch.carried_row_count() == 0 and patch.rolling_resistance_torque() == 0.0 \
				and patch.longitudinal_force() == 0.0 and patch.lateral_force() == 0.0, "%s: positive geometric gap at the start of the step: no physical load, traction, rolling resistance or row (speculative approach stays distinct)" % side)
	Rig.step(b, 1)
	check(b.rear_tire_patch.normal_load() > 0.0 or b.front_tire_patch.normal_load() > 0.0, "the next step (gap now negative) does carry physical load")


func test_d13_compression_partial_contact_separation_and_recontact() -> void:
	var a = Rig.assemble()
	var f = a.front_tire_patch
	var r = a.rear_tire_patch
	var full: int = MotorcyclePrototype.PATCH_LATERAL_COUNT * MotorcyclePrototype.PATCH_CIRCUMFERENTIAL_COUNT
	Rig.shift(a, Vector3(0.0, 0.004, 0.0))
	Rig.roll_forward(a, 2.0)
	var partial_seen: bool = false
	for k in 40:
		a.world.step()
		if r.is_touching():
			partial_seen = r.active_sample_count() > 0 and r.active_sample_count() < full / 4 and r.contact_area() < 0.25 * 0.03
			break
	check(partial_seen, "first contact is a PARTIAL patch (a few samples of the %d candidates)" % full)
	Rig.step(a, 200)
	check(f.is_touching() and r.is_touching() and f.normal_load() > 100.0 and r.normal_load() > 100.0, "settled: both patches carry physical normal load")
	check(f.max_penetration() > 0.002 and r.max_penetration() > 0.002 and f.max_penetration() <= f.max_compression + 0.01, "with real compression")
	check(f.carried_row_count() == f.active_sample_count() and r.carried_row_count() == r.active_sample_count(), "carried rows are exactly the active samples")
	check(r.rolling_resistance_torque() != 0.0 and r.tangential_response_count() > 0, "rolling: rolling resistance and the tangential response act with load")
	# Complete separation.
	Rig.shift(a, Vector3(0.0, 0.08, 0.0))
	Rig.step(a, 1)
	for side in ["front", "rear"]:
		var patch = Rig.patch_of(a, side)
		check(not patch.is_touching() and patch.normal_load() == 0.0 and patch.active_sample_count() == 0 and patch.carried_row_count() == 0 and patch.carried_impulse_total() == 0.0,
			"%s: complete separation clears the physical load and every carried row" % side)
		check(patch.carried_tire_impulse() == Vector3.ZERO and patch.tangential_force_world() == Vector3.ZERO and patch.tangential_moment_world() == Vector3.ZERO \
				and patch.rolling_resistance_torque() == 0.0 and patch.longitudinal_force() == 0.0 and patch.lateral_force() == 0.0, "%s: and the traction, the carried tyre impulse and the rolling resistance" % side)
		check(not patch.has_aggregate_contact() and patch.last_reason == "no compressed sample", "%s: and the aggregate state (reason: %s)" % [side, patch.last_reason])
	# Re-contact: fall back; every separated step stays clear, the first contact step starts cold.
	var recontact_step: int = -1
	for k in 60:
		a.world.step()
		if r.is_touching() or f.is_touching():
			recontact_step = k
			break
		check(r.carried_row_count() == 0 and f.carried_row_count() == 0 and r.rolling_resistance_torque() == 0.0 and f.rolling_resistance_torque() == 0.0, "separated step %d stays clear" % k)
	check(recontact_step >= 0, "the machine falls back onto the ground (re-contact after %d steps)" % recontact_step)
	for side in ["front", "rear"]:
		var patch = Rig.patch_of(a, side)
		if patch.is_touching():
			check(patch.carried_row_count() == patch.active_sample_count() and patch.last_reason == "", "%s: re-contact builds its rows from the new compression only" % side)
			check(patch.normal_load() > 0.0 and is_finite(patch.normal_load()), "%s: with a finite positive load" % side)
	# The riderless machine bounces after the 8 cm drop and eventually falls over (no rider, not self-balancing): only the load shortly after the impact is judged.
	Rig.step(a, 45)
	check(f.is_touching() and r.is_touching() and absf(f.normal_load() + r.normal_load() - WEIGHT) < 0.25 * WEIGHT, "and shortly after the impact both patches again carry about the weight together")


# --- Transactional construction and cleanup --------------------------------------------------------------------------------------

func test_d14_a_refused_patch_registration_rolls_everything_back() -> void:
	for refuse in [1, 2]:
		var world = PatchRefusingWorld.new()
		world.refuse_patch_number = refuse
		var existing = make_body(2.0, Basis.IDENTITY)
		world.add_body(existing)
		var a = Rig.assemble(Callable(), world)
		check(a == null, "build() returns null when patch #%d is refused" % refuse)
		check(world.patches_seen == refuse, "patch #%d was reached (the front one was already registered when the rear one failed)" % refuse)
		check(world.bodies == [existing] and world.constraints.is_empty() and world.force_providers.is_empty(), "patch #%d refused: nothing of the machine remains, earlier content is kept" % refuse)
		check(patches_in(world).is_empty(), "no patch stays registered")
	# The failure leaves the world usable: a later build in the same world succeeds.
	var world_ok = PatchRefusingWorld.new()
	var ok = Rig.assemble(Callable(), world_ok)
	check(ok != null and patches_in(world_ok).size() == 2, "a world without refusals builds normally")


func test_d15_a_failed_patch_construction_rolls_everything_back() -> void:
	for failing_side in ["front", "rear"]:
		var created: Array = []
		var factory: Callable = func(side, ground, wheel, geometry, radius, point, normal, spec, tire, friction):
			if side == failing_side:
				return null
			var patch = TirePatchNormalSupport.create_with_tire(ground, wheel, geometry, radius, point, normal, spec.stiffness_density, spec.damping_density,
					spec.max_compression, spec.lateral_count, spec.circumferential_count, tire, friction)
			created.append(patch)
			return patch
		MotorcycleBuilder.wheel_patch_factory = factory
		var world = make_world(Vector3(0.0, -9.81, 0.0))
		var existing = make_body(2.0, Basis.IDENTITY)
		world.add_body(existing)
		var a = Rig.assemble(Callable(), world)
		MotorcycleBuilder.wheel_patch_factory = Callable()
		check(a == null, "build() returns null when the %s patch cannot be constructed" % failing_side)
		check(created.size() == (0 if failing_side == "front" else 1), "%s fails: %d patch(es) had been constructed before" % [failing_side, created.size()])
		for patch in created:
			check(not world.constraints.has(patch), "the already constructed patch is not left registered")
		check(world.bodies == [existing] and world.constraints.is_empty() and world.force_providers.is_empty(), "%s patch failure: nothing of the machine remains, earlier content is kept" % failing_side)
		check(world.step() , "the restored world still steps")
	check(not MotorcycleBuilder.wheel_patch_factory.is_valid(), "the test seam is restored")
	var normal = Rig.assemble()
	check(normal != null and patches_in(normal.world).size() == 2, "without the seam both patches are built by TirePatchNormalSupport.create_with_tire")


func test_d16_cleanup_after_successful_registration_removes_both_patches() -> void:
	var a = Rig.assemble()
	Rig.step(a, 60)
	var world = a.world
	var f = a.front_tire_patch
	var r = a.rear_tire_patch
	a.set_ground_contacts_enabled(false)
	check(not f.enabled and not r.enabled, "disabling the ground contacts disables both patches")
	Rig.step(a, 1)
	check(f.normal_load() == 0.0 and r.normal_load() == 0.0 and f.carried_row_count() == 0 and r.carried_row_count() == 0, "a disabled patch carries no load or rows (re-enabling starts cold)")
	a.set_ground_contacts_enabled(true)
	Rig.step(a, 60)
	check(f.enabled and f.is_touching() or r.is_touching(), "re-enabled patches act again")
	var ground = a.ground
	a.remove_ground()
	check(not world.constraints.has(f) and not world.constraints.has(r) and patches_in(world).is_empty(), "remove_ground() unregisters both patches")
	check(a.front_wheel_contact == null and a.rear_wheel_contact == null and a.front_tire == null and a.rear_tire == null and a.front_tire_patch == null and a.rear_tire_patch == null,
		"and drops every reference to them")
	check(not world.has_body(ground) and world.constraints.size() == 5, "the ground, the box contacts and the patches are gone; the five joints stay")
	check(world.step(), "the world still steps without ground")


# --- Roll-related observation ------------------------------------------------------------------------------------------------------

func test_d17_the_distributed_contact_geometry_sets_the_roll_lever_arm_of_the_assembly() -> void:
	var results: Dictionary = {}
	var starts: Dictionary = {}
	for sign in [1.0, -1.0]:
		var a = Rig.assemble()
		Rig.roll(a, 0.15 * sign, 0.01)
		# The patch samples are those of the START of the step, so the reference points of the moments are read before it.
		starts[sign] = {"chassis": a.chassis.position, "front": a.front_wheel.position, "rear": a.rear_wheel.position}
		Rig.step(a, 1)
		results[sign] = a
	var up = results[1.0]
	var down = results[-1.0]
	for side in ["front", "rear"]:
		var p_up = Rig.patch_of(up, side)
		var p_down = Rig.patch_of(down, side)
		check(p_up.normal_load() > 50.0 and p_down.normal_load() > 50.0, "%s: both rolled machines carry load" % side)
		# The wrench of the patch about the chassis centre of mass is the moment of ONE resultant at the centre of pressure.
		var about: Vector3 = starts[1.0].chassis
		var resultant: Vector3 = (p_up.centre_of_pressure() - about).cross(p_up.total_force_world())
		check_vec3_near(p_up.total_moment_world(about), resultant, 2.0e-3 * (1.0 + resultant.length()), "%s: sum of the sample moments = (cop - com) x N n" % side)
		check_vec3_near(p_up.applied_wheel_moment(), p_up.total_moment_world(starts[1.0][side]),
			2.0e-3 * (1.0 + p_up.applied_wheel_moment().length()), "%s: the moment the solver applied to the wheel is the geometric moment of the samples" % side)
		# Mirror symmetry of the (symmetric) machine: the roll moment about the chassis changes sign, the load stays.
		var mz_up: float = p_up.total_moment_world(about).z
		var mz_down: float = p_down.total_moment_world(starts[-1.0].chassis).z
		check(absf(mz_up) > 1.0 and absf(mz_up + mz_down) < 0.01 * absf(mz_up), "%s: roll moment about the chassis is antisymmetric (%s / %s N m)" % [side, mz_up, mz_down])
		check_near(p_up.normal_load(), p_down.normal_load(), 0.01 * p_up.normal_load(), "%s: and the load is the same" % side)
	# The lateral lever arm depends on the crown radius of the finite-width tyre: (R - rho) sin(camber).
	var lever: Dictionary = {}
	for crown in [0.10, 0.15]:
		var b = Rig.assemble(func(c): c.rear_tire_geometry = TireGeometry.new(0.18, crown))
		Rig.roll(b, 0.15, 0.01)
		Rig.step(b, 1)
		lever[crown] = b.rear_tire_patch.centre_of_pressure().x - b.rear_wheel.position.x
		check_near(lever[crown], (0.32 - crown) * sin(0.15), 0.06 * (0.32 - crown) * sin(0.15), "rear crown radius %.2f: lever arm (R - rho) sin(camber)" % crown)
	check(lever[0.10] > lever[0.15] + 0.004, "a larger crown radius shortens the lever arm of the contact about the roll axis in the assembled machine")
