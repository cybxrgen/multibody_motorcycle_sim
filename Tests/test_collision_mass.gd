extends "res://Tests/sim_test_case.gd"
## M2.5.2 physical collision-box geometry, its configuration / builder integration (transactional), and the estimated per-part mass
## distribution with inertia consistency. Expectations are written in this file (explicit corner arithmetic, explicit masses).

const CollisionBox = preload("res://Simulation/Motorcycle/motorcycle_collision_box.gd")
const ControlSpec = preload("res://Simulation/Motorcycle/motorcycle_control_spec.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const BoxGroundContact = preload("res://Simulation/Motorcycle/box_ground_contact.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")

const TOL: float = 2.0e-6
const BOX_BODIES: Array = ["chassis", "steering", "fork_slider", "swingarm"]


## A world that refuses the n-th box-ground contact / the controls (failure injection for the transactional builder).
class RefusingWorld extends "res://Simulation/Core/sim_world.gd":
	var refuse_box_contact_number: int = 0
	var box_contacts_seen: int = 0
	var refuse_controls: bool = false

	func add_constraint(constraint):
		if "corner_index" in constraint:
			box_contacts_seen += 1
			if box_contacts_seen == refuse_box_contact_number:
				return null
		return super.add_constraint(constraint)

	func add_force_provider(provider):
		if refuse_controls and "throttle" in provider:
			return null
		return super.add_force_provider(provider)


func is_clean(world) -> bool:
	return world.bodies.is_empty() and world.constraints.is_empty() and world.force_providers.is_empty()


# --- Collision box definition ---

func test_valid_box_is_accepted_and_independent_of_visual_size() -> void:
	var box = CollisionBox.new("chassis", Vector3(0.3, 0.5, 1.0))
	check(box.validate().is_empty() and box.is_valid(), "a positive finite box with a structural owner is valid")
	var c = MotorcyclePrototype.config_with_collision()
	c.chassis.visual_size = Vector3(9.0, 9.0, 9.0)
	check(c.chassis_box.size == Vector3(0.30, 0.55, 1.00), "the collision size does not follow visual_size (they are separate fields)")
	check(c.validate().is_empty(), "changing the presentation size leaves the physical box valid")


func test_invalid_box_parameters_are_rejected() -> void:
	var cases: Array = [
		["zero dimension", func(b): b.size = Vector3(0.3, 0.0, 1.0), "size"],
		["negative dimension", func(b): b.size = Vector3(0.3, 0.5, -1.0), "size"],
		["NaN dimension", func(b): b.size = Vector3(NAN, 0.5, 1.0), "size"],
		["infinite dimension", func(b): b.size = Vector3(0.3, INF, 1.0), "size"],
		["NaN centre", func(b): b.center = Vector3(0.0, NAN, 0.0), "centre"],
		["infinite centre", func(b): b.center = Vector3(INF, 0.0, 0.0), "centre"],
		["NaN orientation", func(b): b.orientation = Quaternion(NAN, 0.0, 0.0, 1.0), "orientation"],
		["degenerate (zero) orientation", func(b): b.orientation = Quaternion(0.0, 0.0, 0.0, 0.0), "orientation"],
		["non-unit orientation", func(b): b.orientation = Quaternion(0.0, 0.0, 0.0, 2.0), "orientation"],
		["wheel owner", func(b): b.body_name = "front_wheel", "owner"],
		["empty owner", func(b): b.body_name = "", "owner"],
		["unknown owner", func(b): b.body_name = "rider", "owner"],
		["negative friction", func(b): b.friction = -0.1, "friction"],
		["NaN friction", func(b): b.friction = NAN, "friction"],
	]
	for case in cases:
		var box = CollisionBox.new("chassis", Vector3(0.3, 0.5, 1.0))
		case[1].call(box)
		var errors: PackedStringArray = box.validate()
		check(not errors.is_empty() and case[2] in "; ".join(errors), "%s is rejected naming '%s' (%s)" % [case[0], case[2], errors])
		check(box.edge_vertices().is_empty(), "%s: no edges are produced for an invalid box" % case[0])


func test_corners_respect_centre_orientation_and_dimensions() -> void:
	# Box 0.2 x 0.4 x 0.6, centre (0.1, 0, 0), rotated 90 degrees about +Y: local X -> -Z, local Z -> +X (right-handed rotation).
	var box = CollisionBox.new("swingarm", Vector3(0.2, 0.4, 0.6), Vector3(0.1, 0.0, 0.0), Quaternion(Vector3.UP, PI / 2.0))
	var corners: Array = box.corners_local()
	check(corners.size() == 8, "eight corners")
	for k in 8:
		var sx: float = 1.0 if (k & 1) != 0 else -1.0
		var sy: float = 1.0 if (k & 2) != 0 else -1.0
		var sz: float = 1.0 if (k & 4) != 0 else -1.0
		# offset (sx 0.1, sy 0.2, sz 0.3) rotated by 90 deg about Y: (x, y, z) -> (z, y, -x)
		var expected: Vector3 = Vector3(0.1, 0.0, 0.0) + Vector3(sz * 0.3, sy * 0.2, -sx * 0.1)
		check_vec3_near(corners[k], expected, TOL, "corner %d in the body frame" % k)
	# The box covers the extents: x in [-0.2, 0.4], y in [-0.2, 0.2], z in [-0.1, 0.1].
	var low: Vector3 = Vector3(INF, INF, INF)
	var high: Vector3 = Vector3(-INF, -INF, -INF)
	for c in corners:
		low = Vector3(minf(low.x, c.x), minf(low.y, c.y), minf(low.z, c.z))
		high = Vector3(maxf(high.x, c.x), maxf(high.y, c.y), maxf(high.z, c.z))
	check_vec3_near(low, Vector3(-0.2, -0.2, -0.1), TOL, "lower extent")
	check_vec3_near(high, Vector3(0.4, 0.2, 0.1), TOL, "upper extent")
	check(box.edge_vertices().size() == 24, "12 edges as 24 end points")


func test_corners_follow_body_translation_and_rotation() -> void:
	var box = CollisionBox.new("chassis", Vector3(0.2, 0.4, 0.6), Vector3(0.0, 0.1, 0.0))
	var position: Vector3 = Vector3(1.0, 2.0, 3.0)
	var rotation: Quaternion = Quaternion(Vector3.BACK, PI / 2.0)  # about +Z: (x, y) -> (-y, x)
	var corners: Array = box.corners_world(position, rotation)
	for k in 8:
		var local: Vector3 = Vector3((0.1 if (k & 1) != 0 else -0.1), 0.1 + (0.2 if (k & 2) != 0 else -0.2), (0.3 if (k & 4) != 0 else -0.3))
		var expected: Vector3 = position + Vector3(-local.y, local.x, local.z)
		check_vec3_near(corners[k], expected, TOL, "world corner %d follows translation and rotation" % k)
	check(box.contains_body_origin(), "the origin (COM) of this body is inside the box (offset 0.1 < half height 0.2)")
	var far = CollisionBox.new("chassis", Vector3(0.2, 0.4, 0.6), Vector3(0.0, 0.5, 0.0))
	check(not far.contains_body_origin(), "a box 0.5 m above the COM does not contain it")


# --- Configuration and builder integration ---

func test_config_validation_of_boxes() -> void:
	var good = MotorcyclePrototype.config_with_collision()
	check(good.validate().is_empty(), "the prototype collision configuration is valid: %s" % good.validate())
	var cases: Array = [
		["slot names another body", func(c): c.chassis_box.body_name = "swingarm", "names owner"],
		["wheel owner in a slot", func(c): c.swingarm_box.body_name = "rear_wheel", "owner"],
		["box without the COM", func(c): c.steering_box.center = Vector3(0.0, 2.0, 0.0), "centre of mass"],
		["one object in two slots", func(c): c.fork_slider_box = c.steering_box, "independent"],
		["invalid dimension", func(c): c.swingarm_box.size = Vector3(0.2, -0.1, 0.5), "size"],
		["negative control limit", func(c): c.controls.max_drive_torque = -1.0, "max_drive_torque"],
		["NaN steering rate", func(c): c.controls.max_steering_rate = NAN, "max_steering_rate"],
	]
	for case in cases:
		var c = MotorcyclePrototype.config_with_collision()
		case[1].call(c)
		var errors: PackedStringArray = c.validate()
		check(not errors.is_empty() and case[2] in "; ".join(errors), "%s is refused naming '%s' (%s)" % [case[0], case[2], errors])


func test_invalid_geometry_fails_the_build_without_leaving_anything() -> void:
	var world = SimWorld.new()
	var c = MotorcyclePrototype.config_with_collision()
	c.swingarm_box.size = Vector3(0.2, 0.0, 0.5)
	check(MotorcycleBuilder.build(c, world) == null, "an invalid box refuses the build")
	check(is_clean(world), "no body, constraint or provider was registered")
	var other = SimWorld.new()
	var keep = make_body(1.0, Basis.IDENTITY)
	other.add_body(keep)
	check(MotorcycleBuilder.build(c, other) == null and other.bodies == [keep] and other.constraints.is_empty(), "a world that already had content keeps exactly that content")


func test_a_failure_while_registering_box_contacts_or_controls_rolls_back_everything() -> void:
	for number in [1, 9, 17, 32]:
		var world = RefusingWorld.new()
		world.refuse_box_contact_number = number
		check(MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision(), world) == null, "refusing box contact #%d fails the build" % number)
		check(is_clean(world), "refusing box contact #%d leaves no body, constraint (joints, tyre or box contacts) or provider" % number)
	var world_c = RefusingWorld.new()
	world_c.refuse_controls = true
	check(MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision(), world_c) == null, "refusing the controls fails the build")
	check(is_clean(world_c), "refusing the controls leaves nothing (suspensions, contacts and boxes rolled back)")


func test_assembly_ownership_four_boxes_no_wheel_box() -> void:
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	check(a != null, "builds")
	check(a.collision_boxes.keys() == BOX_BODIES, "boxes exist for exactly the four structural bodies in order: %s" % [a.collision_boxes.keys()])
	check(a.bodies().size() == 6 and a.constraints().size() == 5, "still six dynamic bodies and five structural joints")
	check(a.world.bodies.size() == 7, "six bodies plus the ground (no body was added for the collision geometry)")
	var wheels: Array = [a.front_wheel, a.rear_wheel]
	var box_count: int = 0
	var tire_count: int = 0
	for c in a.world.constraints:
		if "corner_index" in c:
			box_count += 1
			check(not (c.body_b in wheels), "no box contact on a wheel")
			check(c.body_a == a.ground and a.box_body(c.box.body_name) == c.body_b, "box contact: ground (A) and the box owner (B)")
		elif "tire" in c:
			tire_count += 1
	check(box_count == 32 and tire_count == 2, "32 box corner contacts and exactly 2 tyre contacts (%d, %d)" % [box_count, tire_count])
	check(a.box_contacts.keys() == BOX_BODIES, "box contacts are keyed by the four structural bodies only")
	for body_name in BOX_BODIES:
		var slots: Array = a.box_contacts[body_name]
		check(slots.size() == 8, "%s: eight corner slots" % body_name)
		for k in 8:
			check(slots[k].corner_index == k and slots[k].box == a.collision_boxes[body_name], "%s slot %d is corner %d of the shared box object" % [body_name, k, k])
	for wheel in wheels:
		var count: int = 0
		for c in a.world.constraints:
			if c.body_a == a.ground and c.body_b == wheel:
				count += 1
		check(count == 1, "%s has exactly one ground contact (its tyre)" % wheel.body_name)
	check(a.front_wheel_contact.has_method("normal_load") and a.rear_wheel_contact.has_method("normal_load"), "the wheel contacts are the tyre contacts")
	check(a.world.force_providers.has(a.front_suspension) and a.world.force_providers.has(a.rear_suspension) and a.world.force_providers.has(a.controls), "suspensions intact and the controls registered")
	check(a.world.force_providers.size() == 3, "exactly three providers")


func test_boxes_are_stored_as_copies_and_work_without_ground() -> void:
	var c = MotorcyclePrototype.config_with_collision()
	c.ground = null
	c.front_tire = null
	c.rear_tire = null
	var a = MotorcycleBuilder.build(c)
	check(a != null and a.collision_boxes.size() == 4 and a.box_contacts.is_empty(), "without ground the boxes exist and no contact is created")
	check(a.collision_boxes["chassis"] != c.chassis_box and a.collision_boxes["chassis"].size == c.chassis_box.size, "the assembly keeps an independent copy")
	c.chassis_box.size = Vector3(1.0, 1.0, 1.0)
	check(a.collision_boxes["chassis"].size == Vector3(0.30, 0.55, 1.00), "later edits of the configuration do not reach the assembly")
	var plain = MotorcycleBuilder.build(MotorcyclePrototype.config_with_tires())
	check(plain.collision_boxes.is_empty() and plain.box_contacts.is_empty() and plain.controls == null, "a configuration without boxes or controls builds exactly the M2.5.1 assembly")


func test_teardown_removes_box_resources_cleanly() -> void:
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	a.remove_collision_box("swingarm")
	check(not a.collision_boxes.has("swingarm") and not a.box_contacts.has("swingarm"), "the swingarm box is gone")
	var left: int = 0
	for c in a.world.constraints:
		if "corner_index" in c:
			left += 1
			check(c.body_b != a.swingarm, "no contact of the removed box remains")
	check(left == 24, "the other three boxes keep their 24 contacts")
	a.remove_controls()
	check(a.controls == null and a.world.force_providers.size() == 2 and not a.steering_joint.motor_enabled, "controls removed and the steering motor released")
	a.remove_ground()
	check(a.box_contacts.is_empty(), "removing the ground drops the remaining box contacts")
	for c in a.world.constraints:
		check(not ("corner_index" in c), "no box contact is left in the world")
	var b = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	b.world.remove_body(b.chassis)
	check(b.world.constraints.filter(func(c): return "corner_index" in c and c.body_b == b.chassis).is_empty(), "removing a body removes its box contacts")
	check(b.world.step() or true, "the world remains steppable-or-refusing without crashing")


# --- Mass distribution and inertia ---

func test_six_independent_positive_estimated_masses() -> void:
	var c = MotorcyclePrototype.config_with_collision()
	var expected: Array = [128.0, 14.0, 8.0, 9.0, 12.0, 15.0]
	var specs: Array = c.body_specs()
	check(specs.size() == 6, "six body specs")
	var total: float = 0.0
	for k in 6:
		check(specs[k].mass > 0.0 and is_finite(specs[k].mass), "%s has a positive finite mass" % specs[k].body_name)
		check_near(specs[k].mass, expected[k], 0.0, "%s estimated mass" % specs[k].body_name)
		total += specs[k].mass
	check_near(total, 186.0, 1.0e-9, "estimated total motorcycle mass (no rider) is 186 kg")
	var a = MotorcycleBuilder.build(c)
	check_near(a.total_mass(), total, 1.0e-9, "the assembly's total mass equals the sum of the six configured masses")
	for k in 6:
		check_near(a.bodies()[k].mass, specs[k].mass, 0.0, "body %d keeps its own mass" % k)
	check(a.chassis.mass < a.total_mass() * 0.75, "the chassis does not carry the whole mass")


func test_changing_one_mass_does_not_change_another() -> void:
	for k in 6:
		var base = MotorcyclePrototype.config_with_collision()
		var changed = MotorcyclePrototype.config_with_collision()
		changed.body_specs()[k].mass += 5.0
		for j in 6:
			if j != k:
				check_near(changed.body_specs()[j].mass, base.body_specs()[j].mass, 0.0, "changing body %d leaves body %d's mass alone" % [k, j])
		check_near(changed.body_specs()[k].mass, base.body_specs()[k].mass + 5.0, 1.0e-12, "body %d changed" % k)
		var a = MotorcycleBuilder.build(changed)
		check(a != null, "a configuration with one changed mass builds")
		if a != null:
			check_near(a.total_mass(), 186.0 + 5.0, 1.0e-9, "total follows the one changed mass")
	var c = MotorcyclePrototype.config_with_collision()
	c.front_wheel.mass = 20.0
	check_near(c.rear_wheel.mass, 15.0, 0.0, "the two wheel specs are independent objects")


func test_inertia_is_valid_symmetric_and_positive_for_all_six_bodies() -> void:
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	for body in a.bodies():
		var i: Basis = body.local_inertia
		check(SimMassProperties.validate_inertia(i) == "", "%s: inertia passes the project's validation" % body.body_name)
		check_basis_near(i, i.transposed(), 1.0e-9, "%s: inertia is symmetric" % body.body_name)
		check(i.x.x > 0.0 and i.y.y > 0.0 and i.z.z > 0.0, "%s: principal (diagonal) moments are positive" % body.body_name)
		check(i.x.x <= i.y.y + i.z.z and i.y.y <= i.x.x + i.z.z and i.z.z <= i.x.x + i.y.y, "%s: moments satisfy the triangle inequality (a physical body)" % body.body_name)
		check(body.mass_properties.valid, "%s: valid mass properties" % body.body_name)


func test_box_inertia_agrees_with_the_collision_box_and_wheels_keep_their_own_model() -> void:
	var c = MotorcyclePrototype.config_with_collision()
	var specs: Dictionary = {"chassis": c.chassis, "steering": c.steering, "fork_slider": c.fork_slider, "swingarm": c.swingarm}
	for body_name in BOX_BODIES:
		var spec = specs[body_name]
		var box = c.collision_boxes()[BOX_BODIES.find(body_name)]
		var expected: Basis = SimMassProperties.box_inertia(spec.mass, box.size)
		check_basis_near(spec.local_inertia, expected, 1.0e-6, "%s: inertia is that of a uniform solid box of the configured mass and the physical dimensions" % body_name)
		# Explicit formula I = m/12 (b^2 + c^2) for the x axis.
		check_near(spec.local_inertia.x.x, spec.mass / 12.0 * (box.size.y * box.size.y + box.size.z * box.size.z), 1.0e-6, "%s: I_xx = m/12 (h^2 + l^2)" % body_name)
		check(box.contains_body_origin(), "%s: the body COM lies inside its collision box (one frame for mass and geometry)" % body_name)
	for pair in [[c.front_wheel, c.front_wheel_radius, c.front_wheel_width], [c.rear_wheel, c.rear_wheel_radius, c.rear_wheel_width]]:
		var wheel_i: Basis = MotorcycleConfig.wheel_inertia(pair[0].mass, pair[1], MotorcyclePrototype.WHEEL_INNER_FRACTION * pair[1], pair[2])
		check_basis_near(pair[0].local_inertia, wheel_i, 1.0e-6, "%s keeps the hollow-cylinder wheel inertia" % pair[0].body_name)
		var as_box: Basis = SimMassProperties.box_inertia(pair[0].mass, Vector3(pair[2], 2.0 * pair[1], 2.0 * pair[1]))
		check(absf(pair[0].local_inertia.x.x - as_box.x.x) > 0.05 * as_box.x.x, "%s is not modelled as a rectangular box" % pair[0].body_name)


func test_builder_rejects_invalid_mass_or_inertia() -> void:
	var cases: Array = [
		["zero mass", func(c): c.swingarm.mass = 0.0],
		["negative mass", func(c): c.chassis.mass = -1.0],
		["NaN mass", func(c): c.rear_wheel.mass = NAN],
		["infinite mass", func(c): c.steering.mass = INF],
		["zero inertia", func(c): c.fork_slider.local_inertia = Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)],
		["non-symmetric inertia", func(c): c.chassis.local_inertia = Basis(Vector3(5, 1, 0), Vector3(0, 5, 0), Vector3(0, 0, 5))],
		["negative principal moment", func(c): c.front_wheel.local_inertia = Basis.from_scale(Vector3(-1.0, 1.0, 1.0))],
		["NaN inertia", func(c): c.swingarm.local_inertia = Basis.from_scale(Vector3(NAN, 1.0, 1.0))],
	]
	for case in cases:
		var world = SimWorld.new()
		var c = MotorcyclePrototype.config_with_collision()
		case[1].call(c)
		check(MotorcycleBuilder.build(c, world) == null, "%s refuses the build" % case[0])
		check(is_clean(world), "%s leaves the world empty" % case[0])
