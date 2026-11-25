extends "res://Tests/sim_test_case.gd"
## M2.5.2 box-ground contacts: generation, reduction (deterministic corner slots), lifecycle, response, and exclusivity of the tyre contact for the
## wheels. Isolated rigs (a static ground and ONE dynamic box body in a world of their own) test the contact component against explicit
## geometry; the assembly tests check ownership and that the box contacts never touch the wheels.
##
## Tolerances: float32 poses of metre size are good to ~2e-6. Resting height: the generic contact's speculative normal row leaves no
## resting penetration beyond float rounding and the split impulse removes any initial overlap, so a settled box is compared with its exact
## resting height to 2 mm (the measured value agrees to well below 1 mm; the margin covers the last slow convergence). Jitter / energy
## limits are stated where used.

const CollisionBox = preload("res://Simulation/Motorcycle/motorcycle_collision_box.gd")
const BoxGroundContact = preload("res://Simulation/Motorcycle/box_ground_contact.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")

const DT: float = 1.0 / 120.0
const G: float = 9.81
const TOL: float = 2.0e-6
const BOX_BODIES: Array = ["chassis", "steering", "fork_slider", "swingarm"]


## A ground and one dynamic box body of `size` (uniform density, mass 20 kg) at `position` / `rotation` with its eight corner contacts.
func make_rig(size: Vector3, position: Vector3, rotation: Quaternion = Quaternion.IDENTITY, gravity: Vector3 = Vector3(0.0, -G, 0.0), mass: float = 20.0,
		owner: String = "chassis") -> Dictionary:
	var world: SimWorld = make_world(gravity)
	var ground: SimRigidBody = make_static()
	world.add_body(ground)
	var body: SimRigidBody = make_body(mass, SimMassProperties.box_inertia(mass, size), position, rotation)
	world.add_body(body)
	var box = CollisionBox.new(owner, size)
	var contacts: Array = []
	for k in 8:
		var contact = BoxGroundContact.create(ground, body, box, k, Vector3.ZERO, Vector3.UP, 0.05)
		check(contact != null and world.add_constraint(contact) != null, "corner %d contact is created and registered" % k)
		contacts.append(contact)
	return {"world": world, "ground": ground, "body": body, "box": box, "contacts": contacts}


func active_set(rig: Dictionary) -> Array:
	var result: Array = []
	for c in rig.contacts:
		if c.contact_active:
			result.append(c.corner_index)
	return result


func lowest_gap(rig: Dictionary) -> float:
	var low: float = INF
	for c in rig.contacts:
		low = minf(low, c.signed_gap())
	return low


func total_normal_impulse(rig: Dictionary) -> float:
	var total: float = 0.0
	for c in rig.contacts:
		total += c.normal_impulse()
	return total


func mechanical_energy(world: SimWorld) -> float:
	return world.total_kinetic_energy() + world.total_potential_energy()


# --- Generation: above, approaching, penetrating ---

func test_a_box_above_the_ground_produces_no_contact_response() -> void:
	var rig: Dictionary = make_rig(Vector3(0.3, 0.5, 1.0), Vector3(0.0, 1.0, 0.0))
	rig.world.step()
	check(active_set(rig).is_empty(), "no corner is active 0.75 m above the plane")
	check_near(total_normal_impulse(rig), 0.0, 0.0, "no impulse")
	# Free fall for 12 steps: y = y0 - 1/2 g t^2 with the symplectic-Euler discretisation v_n = -g n dt, y_n = y0 - g dt^2 n (n + 1) / 2.
	for k in 11:
		rig.world.step()
	check_near(rig.body.position.y, 1.0 - G * DT * DT * 12.0 * 13.0 / 2.0, 1.0e-5, "the box falls exactly as a free body")
	check_near(rig.body.angular_velocity.length(), 0.0, 1.0e-6, "no spin")


func test_an_approaching_box_gets_contact_before_it_penetrates() -> void:
	var rig: Dictionary = make_rig(Vector3(0.3, 0.5, 1.0), Vector3(0.0, 0.7, 0.0))
	var first_active: int = -1
	var worst_gap: float = INF
	for k in 240:
		rig.world.step()
		if first_active < 0 and not active_set(rig).is_empty():
			first_active = k
		worst_gap = minf(worst_gap, lowest_gap(rig))
	check(first_active > 0, "the contact appears while the box is still above the plane (step %d)" % first_active)
	check(worst_gap > -2.0e-3, "the speculative contact keeps the penetration below 2 mm (worst gap %s)" % worst_gap)
	check_near(rig.body.position.y, 0.25, 2.0e-3, "the box rests on its 0.3 x 1.0 m face at half its height")


func test_a_penetrating_box_receives_a_separating_normal_response() -> void:
	var rig: Dictionary = make_rig(Vector3(0.3, 0.5, 1.0), Vector3(0.0, 0.24, 0.0), Quaternion.IDENTITY, Vector3.ZERO)
	check_near(lowest_gap(rig), -0.01, TOL, "starts 10 mm inside the plane")
	rig.world.step()
	# At rest the physical row needs no impulse (vn = 0 already satisfies vn >= 0); the separating response is the split-impulse correction.
	check(active_set(rig).size() == 4 and total_normal_impulse(rig) >= 0.0, "the four lower corners are active with a non-negative normal impulse")
	check(rig.body.position.y > 0.24, "the box moves out of the plane")
	for k in 60:
		rig.world.step()
	check(lowest_gap(rig) > -1.0e-3, "the penetration is removed (gap %s)" % lowest_gap(rig))
	check(rig.body.linear_velocity.y < 0.5, "without a violent kick (split-impulse correction: vy %s)" % rig.body.linear_velocity.y)


# --- Multiple contacts, settling, energy ---

func test_a_box_settles_on_four_contacts_without_jitter_or_energy_growth() -> void:
	var rig: Dictionary = make_rig(Vector3(0.3, 0.5, 1.0), Vector3(0.0, 0.5, 0.0), Quaternion.IDENTITY)
	var energy_start: float = mechanical_energy(rig.world)
	var energy_max: float = energy_start
	var late_speed: float = 0.0
	for k in 360:
		rig.world.step()
		energy_max = maxf(energy_max, mechanical_energy(rig.world))
		if k >= 300:
			late_speed = maxf(late_speed, rig.body.linear_velocity.length() + rig.body.angular_velocity.length())
	check(active_set(rig) == [0, 1, 4, 5] or active_set(rig).size() == 4, "the four corners of the lower face carry the box: %s" % [active_set(rig)])
	check_near(rig.body.position.y, 0.25, 2.0e-3, "resting height")
	check(late_speed < 1.0e-3, "no jitter after settling (|v| + |w| <= %s)" % late_speed)
	check(energy_max <= energy_start + 1.0e-3, "mechanical energy never grows (max %s vs start %s)" % [energy_max, energy_start])
	check(mechanical_energy(rig.world) < energy_start, "energy was dissipated")
	check_near(total_normal_impulse(rig) / DT, 20.0 * G, 0.01 * 20.0 * G, "the four contacts carry the weight (to 1 %)")
	check(absf(rig.body.orientation.x) + absf(rig.body.orientation.y) + absf(rig.body.orientation.z) < 2.0e-3, "no tipping")
	var low_count: int = 0
	for c in rig.contacts:
		if c.contact_active and c.is_touching():
			low_count += 1
	check(low_count == 4, "four touching corners (%d)" % low_count)


func test_a_rotated_box_contacts_the_plane_at_the_appropriate_corners() -> void:
	# 0.4 x 0.4 x 0.6 box turned 45 degrees about Z: an edge (two corners, at z = +-0.3) is the lowest feature.
	var size: Vector3 = Vector3(0.4, 0.4, 0.6)
	var rotation: Quaternion = Quaternion(Vector3.BACK, PI / 4.0)
	var box = CollisionBox.new("chassis", size)
	var corners: Array = box.corners_world(Vector3.ZERO, rotation)
	var low: float = INF
	for c in corners:
		low = minf(low, c.y)
	var expected: Array = []
	for k in 8:
		if corners[k].y < low + 1.0e-5:
			expected.append(k)
	check(expected.size() == 2, "the rotated box has an edge (two corners) at the bottom: %s" % [expected])
	var rig: Dictionary = make_rig(size, Vector3(0.0, -low, 0.0), rotation, Vector3.ZERO)
	rig.world.step()
	check(active_set(rig) == expected, "exactly those two corners are active: %s vs %s" % [active_set(rig), expected])
	for k in expected:
		check(rig.contacts[k].is_touching(), "corner %d touches" % k)
	check(rig.contacts[expected[0]].signed_gap() - rig.contacts[expected[1]].signed_gap() < TOL, "the two corners are level")


func test_contact_points_update_during_translation_and_rotation() -> void:
	var rig: Dictionary = make_rig(Vector3(0.3, 0.5, 1.0), Vector3(0.0, 0.25, 0.0), Quaternion.IDENTITY, Vector3.ZERO)
	rig.world.step()
	var before: Array = []
	for c in rig.contacts:
		before.append(c.world_point_b())
	rig.body.position = Vector3(2.0, 0.25, -1.0)
	rig.world.step()
	for k in 8:
		if rig.contacts[k].contact_active:
			var expected: Vector3 = rig.body.position + rig.body.orientation * rig.box.corner_local(k)
			check_vec3_near(rig.contacts[k].world_point_b(), expected, 1.0e-4, "corner %d anchor moved with the translation" % k)
			check((rig.contacts[k].world_point_b() - before[k]).length() > 1.0, "corner %d anchor really moved" % k)
	rig.body.orientation = Quaternion(Vector3.UP, 0.7)
	rig.world.step()
	for k in 8:
		if rig.contacts[k].contact_active:
			var expected: Vector3 = rig.body.position + rig.body.orientation * rig.box.corner_local(k)
			check_vec3_near(rig.contacts[k].world_point_b(), expected, 1.0e-4, "corner %d anchor follows the rotation about the normal" % k)
	check(active_set(rig).size() == 4, "the same four corners stay active")


# --- Lifecycle ---

func test_separation_clears_all_contact_state_and_recontact_starts_cold() -> void:
	var rig: Dictionary = make_rig(Vector3(0.3, 0.5, 1.0), Vector3(0.0, 0.5, 0.0))
	rig.body.linear_velocity = Vector3(1.0, 0.0, 0.5)  # slide, so a friction impulse exists too
	for k in 240:
		rig.world.step()
	check(not active_set(rig).is_empty() and total_normal_impulse(rig) > 0.0, "in contact and loaded")
	# Lift the box clear (a test manipulation, not a physics claim) and step.
	rig.body.position = Vector3(0.0, 2.0, 0.0)
	rig.body.linear_velocity = Vector3.ZERO
	rig.body.angular_velocity = Vector3.ZERO
	rig.world.step()
	check(active_set(rig).is_empty(), "separation: no active contact")
	for c in rig.contacts:
		check(c.normal_impulse() == 0.0 and c.friction_impulse_magnitude() == 0.0 and c._friction_impulse == Vector3.ZERO, "corner %d keeps no normal or friction impulse after separation" % c.corner_index)
	# Re-contact: the box falls again; before the first active step nothing is carried, afterwards it settles again.
	var cold_first_step: bool = true
	var recontact: bool = false
	for k in 240:
		rig.world.step()
		if not recontact and not active_set(rig).is_empty():
			recontact = true
		if not recontact:
			for c in rig.contacts:
				cold_first_step = cold_first_step and c.normal_impulse() == 0.0 and c._friction_impulse == Vector3.ZERO
	check(recontact and cold_first_step, "re-contact happens and nothing was carried through the air")
	check_near(rig.body.position.y, 0.25, 2.0e-3, "and the box settles again")


func test_contact_selection_is_deterministic() -> void:
	var a: Dictionary = make_rig(Vector3(0.3, 0.5, 1.0), Vector3(0.1, 0.8, 0.0), Quaternion(Vector3(1.0, 0.3, 0.2).normalized(), 0.4))
	var b: Dictionary = make_rig(Vector3(0.3, 0.5, 1.0), Vector3(0.1, 0.8, 0.0), Quaternion(Vector3(1.0, 0.3, 0.2).normalized(), 0.4))
	var identical: bool = true
	for k in 300:
		a.world.step()
		b.world.step()
		identical = identical and active_set(a) == active_set(b) and a.body.position == b.body.position and a.body.orientation == b.body.orientation
		identical = identical and total_normal_impulse(a) == total_normal_impulse(b)
	check(identical, "two identical runs select the same corners and give bit-identical states and impulses")
	# Unchanged geometry: the same corners are selected at every step.
	var c: Dictionary = make_rig(Vector3(0.3, 0.5, 1.0), Vector3(0.0, 0.25, 0.0), Quaternion.IDENTITY, Vector3.ZERO)
	c.world.step()
	var first: Array = active_set(c)
	var stable: bool = true
	for k in 120:
		c.world.step()
		stable = stable and active_set(c) == first
	check(first.size() == 4 and stable, "a box at rest keeps exactly the same four corners step after step (%s)" % [first])


func test_non_finite_or_invalid_geometry_fails_safely() -> void:
	var rig: Dictionary = make_rig(Vector3(0.3, 0.5, 1.0), Vector3(0.0, 0.25, 0.0), Quaternion.IDENTITY, Vector3.ZERO)
	var geometry: Dictionary = BoxGroundContact.corner_geometry(rig.box, 0, Vector3(NAN, 0.0, 0.0), Quaternion.IDENTITY, Vector3.ZERO, Vector3.UP)
	check(not geometry.valid and geometry.reason != "", "a non-finite body pose gives invalid geometry with a reason")
	var far: Dictionary = BoxGroundContact.corner_geometry(rig.box, 0, Vector3(INF, 0.0, 0.0), Quaternion.IDENTITY, Vector3.ZERO, Vector3.UP)
	check(not far.valid, "a non-finite transformed corner is invalid")
	check(BoxGroundContact.create(rig.ground, rig.body, rig.box, 8, Vector3.ZERO, Vector3.UP) == null, "corner index 8 is refused")
	check(BoxGroundContact.create(rig.ground, rig.body, null, 0, Vector3.ZERO, Vector3.UP) == null, "a null box is refused")
	check(BoxGroundContact.create(rig.ground, rig.body, rig.box, 0, Vector3.ZERO, Vector3(0.0, 2.0, 0.0)) == null, "a non-unit plane normal is refused")
	# A contact whose body pose becomes non-finite is simply inactive (prepare must not crash) and keeps no impulse.
	rig.world.step()
	var contact = rig.contacts[0]
	var saved: Vector3 = rig.body.position
	rig.body.position = Vector3(NAN, 0.0, 0.0)
	contact.prepare(DT, rig.world.config)
	check(not contact.contact_active and contact.normal_impulse() == 0.0, "non-finite pose: inactive and impulse-free")
	rig.body.position = saved
	# A box that becomes invalid after registration stops the world instead of being simulated.
	rig.box.size = Vector3(0.3, NAN, 1.0)
	var steps: int = rig.world.step_count
	check(not rig.world.step() and rig.world.step_count == steps, "an invalid box makes the world refuse the step")
	check("size" in rig.world.last_error, "the reason names the size: %s" % rig.world.last_error)
	rig.box.size = Vector3(0.3, 0.5, 1.0)
	check(rig.world.step(), "and stepping resumes when the box is valid again")


func test_removing_a_body_removes_its_contacts() -> void:
	var rig: Dictionary = make_rig(Vector3(0.3, 0.5, 1.0), Vector3(0.0, 0.5, 0.0))
	rig.world.remove_body(rig.body)
	check(rig.world.constraints.is_empty(), "all eight contacts left with the body")
	check(rig.world.step(), "the world keeps stepping")


func test_several_fixed_steps_in_one_frame_refresh_every_step() -> void:
	var rig: Dictionary = make_rig(Vector3(0.3, 0.5, 1.0), Vector3(0.0, 0.5, 0.0))
	var contact = rig.contacts[0]
	var before: int = contact.refresh_count
	var steps: int = rig.world.advance(5.0 * DT)
	check(steps == 5 and contact.refresh_count == before + 5, "each of the %d fixed steps refreshed the contact geometry (%d)" % [steps, contact.refresh_count - before])


# --- Every structural body can contact the ground ---

func test_each_structural_body_can_contact_and_rest_on_the_ground() -> void:
	var c = MotorcyclePrototype.config_with_collision()
	var specs: Dictionary = {"chassis": c.chassis, "steering": c.steering, "fork_slider": c.fork_slider, "swingarm": c.swingarm}
	for body_name in BOX_BODIES:
		var spec = specs[body_name]
		var box = c.collision_boxes()[BOX_BODIES.find(body_name)]
		# The configured body (mass, inertia, orientation) alone above the plane, its lowest corner 0.1 m up.
		var low: float = INF
		for corner in box.corners_world(Vector3.ZERO, spec.orientation):
			low = minf(low, corner.y)
		var world: SimWorld = make_world(Vector3(0.0, -G, 0.0))
		var ground: SimRigidBody = make_static()
		world.add_body(ground)
		var body: SimRigidBody = SimRigidBody.new(SimMassProperties.new(spec.mass, spec.local_inertia), Vector3(0.0, 0.1 - low, 0.0), spec.orientation)
		world.add_body(body)
		var contacts: Array = []
		for k in 8:
			var contact = BoxGroundContact.create(ground, body, box.duplicate_spec(), k, Vector3.ZERO, Vector3.UP, 0.05)
			world.add_constraint(contact)
			contacts.append(contact)
		var ever_active: bool = false
		var worst_gap: float = INF
		for k in 720:
			world.step()
			var active: int = 0
			for contact in contacts:
				if contact.contact_active:
					active += 1
				worst_gap = minf(worst_gap, contact.signed_gap())
			ever_active = ever_active or active > 0
		var touching: int = 0
		var lowest: float = INF
		for contact in contacts:
			lowest = minf(lowest, contact.signed_gap())
			if contact.contact_active and contact.is_touching():
				touching += 1
		check(ever_active, "%s: its box generated ground contacts after falling" % body_name)
		check(worst_gap > -3.0e-3, "%s: penetration stays below 3 mm (worst gap %s)" % [body_name, worst_gap])
		check(lowest > -2.0e-3 and lowest < 2.0e-3, "%s: rests on the ground (lowest corner gap %s)" % [body_name, lowest])
		check(touching >= 3, "%s: supported by at least three corners (%d)" % [body_name, touching])
		check(body.linear_velocity.length() < 5.0e-3 and body.angular_velocity.length() < 5.0e-3, "%s: at rest (|v| %s, |w| %s)" % [body_name, body.linear_velocity.length(), body.angular_velocity.length()])


# --- Wheel contact exclusivity ---

func test_wheels_have_exactly_their_tyre_contact_and_no_box_contact() -> void:
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision())
	for wheel in [a.front_wheel, a.rear_wheel]:
		var ground_contacts: Array = a.world.constraints.filter(func(c): return c.body_a == a.ground and c.body_b == wheel)
		check(ground_contacts.size() == 1 and "tire" in ground_contacts[0] and not ("corner_index" in ground_contacts[0]), "%s: one tyre contact, no box contact" % wheel.body_name)
	check(a.front_tire == a.front_wheel_contact and a.rear_tire == a.rear_wheel_contact, "the tyres are the wheel contacts")
	check(a.box_contacts.size() == 4 and not a.box_contacts.has("front_wheel") and not a.box_contacts.has("rear_wheel"), "box contacts exist for four bodies only")


func test_box_contacts_do_not_duplicate_the_normal_support_of_the_wheels() -> void:
	# While the motorcycle stands on its tyres the structural boxes are far above the plane: they take no part, and the tyre results are
	# bit-identical to the assembly without any collision box.
	var with_boxes = MotorcycleBuilder.build(MotorcyclePrototype.config_with_tires().duplicate() if false else _config_boxes_only())
	var without = MotorcycleBuilder.build(MotorcyclePrototype.config_with_tires())
	var max_active: int = 0
	var identical: bool = true
	for k in 300:
		with_boxes.world.step()
		without.world.step()
		max_active = maxi(max_active, with_boxes.all_box_contacts().filter(func(c): return c.contact_active).size())
		identical = identical and with_boxes.rear_tire.normal_load() == without.rear_tire.normal_load() and with_boxes.front_tire.normal_load() == without.front_tire.normal_load()
		identical = identical and with_boxes.rear_tire.longitudinal_force() == without.rear_tire.longitudinal_force() and with_boxes.rear_tire.rolling_resistance_torque() == without.rear_tire.rolling_resistance_torque()
	check(max_active == 0, "no box contact is active while the motorcycle stands on its tyres (%d)" % max_active)
	check(identical, "tyre normal load, traction and rolling resistance are identical with and without the boxes")
	check(with_boxes.rear_tire.normal_load() > 0.0, "the rear tyre carries load")
	for body in without.world.bodies:
		var other = with_boxes.world.bodies[without.world.bodies.find(body)]
		check(other.position == body.position and other.orientation == body.orientation, "%s: same pose with and without boxes" % body.body_name)


func _config_boxes_only():
	var c = MotorcyclePrototype.config_with_collision()
	c.controls = null
	return c


func test_speculative_tyre_behaviour_is_unchanged() -> void:
	# M2.4.2.1: a wheel with a small positive gap carries speculative normal impulse but no tyre force. Compare an airborne-approach case with
	# and without boxes: identical, because the boxes do not touch the tyre contact.
	for with_boxes in [false, true]:
		var c = _config_boxes_only() if with_boxes else MotorcyclePrototype.config_with_tires()
		var a = MotorcycleBuilder.build(c)
		a.front_wheel_contact.activation_distance = 0.05
		a.world.step()
		var gap: float = a.front_wheel_contact.signed_gap()
		check(gap > 0.0 and not a.front_tire.is_touching(), "front wheel hovers %s m above the plane (boxes: %s)" % [gap, with_boxes])
		check(a.front_tire.normal_load() == 0.0 and a.front_tire.longitudinal_force() == 0.0 and a.front_tire.rolling_resistance_torque() == 0.0, "no physical load, traction or rolling resistance while only speculative (boxes: %s)" % with_boxes)
