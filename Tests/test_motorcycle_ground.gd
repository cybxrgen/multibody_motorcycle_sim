extends "res://Tests/sim_test_case.gd"
## M2.4.1 motorcycle integration: ground and both wheel contacts in the built assembly, topology preserved, gravitational load
## transmitted through the two contacts, contact loss / re-contact, transactional builder, disabling and removing the ground.
##
## ROLL STABILITY IS NOT TESTED AND NOT FAKED. A riderless motorcycle on two narrow point contacts is not statically stable in
## roll; nothing in the production assembly supports it laterally. The vertical-support tests below use the TEST-ONLY fixture of
## Tests/ground_rig.gd (a vertical-slide joint between the chassis and a static body, outside the production assembly) so that the
## vertical load path through the two wheel contacts can be checked on its own. Two point contacts do not provide a physically
## complete tyre-width or rider-supported roll-stability model.
## Tolerances: the settled support force is compared with the total weight to 0.1 % (the measured steady state agrees to ~1e-5;
## the margin covers the suspension still creeping towards its equilibrium after the simulated time).

const Rig = preload("res://Tests/ground_rig.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")

const DT: float = 1.0 / 120.0
const G: float = 9.81


## A world that refuses to register the n-th WheelGroundContact (failure injection for the transactional builder).
class RefusingWorld extends "res://Simulation/Core/sim_world.gd":
	var refuse_contact_number: int = 0
	var contacts_seen: int = 0

	func add_constraint(constraint):
		if "wheel_radius" in constraint:
			contacts_seen += 1
			if contacts_seen == refuse_contact_number:
				return null
		return super.add_constraint(constraint)


func build_with_ground(world = null):
	return MotorcycleBuilder.build(MotorcyclePrototype.config_with_ground(), world)


func test_ground_and_both_wheel_contacts_are_registered() -> void:
	var a = build_with_ground()
	check(a != null and a.has_ground(), "the assembly has a ground")
	var w = a.world
	check(w.has_body(a.ground), "the ground body belongs to the same world as the motorcycle")
	check(w.constraints.has(a.front_wheel_contact) and w.constraints.has(a.rear_wheel_contact), "both wheel contacts are registered in the world")
	check(a.front_wheel_contact.body_a == a.ground and a.front_wheel_contact.body_b == a.front_wheel, "front contact: ground (A) and the FRONT wheel (B)")
	check(a.rear_wheel_contact.body_a == a.ground and a.rear_wheel_contact.body_b == a.rear_wheel, "rear contact: ground (A) and the REAR wheel (B)")
	check(a.front_wheel_contact != a.rear_wheel_contact, "two distinct contacts (a single shared contact would leave one wheel unsupported)")
	check(a.ground.is_static() and a.ground.body_name == "ground", "the ground is a static body named 'ground'")
	check(a.ground_contacts() == [a.front_wheel_contact, a.rear_wheel_contact], "ground_contacts() lists front then rear")
	check_near(a.front_wheel_contact.wheel_radius, 0.31, 0.0, "front contact uses the front wheel radius")
	check_near(a.rear_wheel_contact.wheel_radius, 0.32, 0.0, "rear contact uses the rear wheel radius")
	check_near(a.front_wheel_contact.friction, MotorcyclePrototype.GROUND_FRICTION, 0.0, "configured friction reaches the contact")
	check_vec3_near(a.front_wheel_contact.plane_normal, Vector3.UP, 0.0, "the ground normal reaches the contact")
	# Reference pose: the rear wheel touches the plane (axle 0.32 m high, R 0.32), the front wheel is ~3 mm above it.
	check_near(a.rear_wheel_contact.signed_gap(), 0.0, 1.0e-5, "rear wheel exactly touches at the reference pose")
	check(a.front_wheel_contact.signed_gap() > 0.0 and a.front_wheel_contact.signed_gap() < 0.005, "front wheel is a few mm above the plane (%s)" % a.front_wheel_contact.signed_gap())


func test_topology_without_ground_is_unchanged_and_with_ground_only_adds_the_ground_and_two_contacts() -> void:
	var plain = MotorcycleBuilder.build(MotorcyclePrototype.config())
	check(plain != null and not plain.has_ground(), "no ground in the plain configuration")
	check(plain.ground == null and plain.front_wheel_contact == null and plain.rear_wheel_contact == null and plain.ground_contacts().is_empty(), "no ground references")
	check(plain.world.bodies.size() == 6 and plain.world.constraints.size() == 5 and plain.world.force_providers.size() == 2, "plain assembly: 6 bodies, 5 constraints, 2 providers")
	var a = build_with_ground()
	check(a.bodies().size() == 6 and a.joints().size() == 4 and a.constraints().size() == 5, "named topology: six dynamic bodies, four hinges, one prismatic joint")
	check(a.world.bodies.size() == 7 and a.world.constraints.size() == 7 and a.world.force_providers.size() == 2, "world: 6 + ground bodies, 5 + 2 contacts, 2 providers")
	for i in 6:
		check(a.world.bodies[i] == a.bodies()[i] and not a.bodies()[i].is_static(), "body %d keeps its place and stays dynamic" % i)
	check(a.world.bodies[6] == a.ground, "the ground is registered after the six motorcycle bodies")
	check(a.world.constraints.slice(0, 5) == a.constraints(), "the five joints keep their registration order")
	check(a.world.force_providers == a.force_providers() and a.front_suspension != null and a.rear_suspension != null, "both suspension providers remain registered")
	# Same masses and suspension parameters as without ground (nothing was tuned for the contact).
	var p = MotorcyclePrototype.config()
	var q = MotorcyclePrototype.config_with_ground()
	check_near(a.total_mass(), plain.total_mass(), 0.0, "total mass unchanged")
	check(q.front_suspension.stiffness == p.front_suspension.stiffness and q.rear_suspension.stiffness == p.rear_suspension.stiffness \
		and q.front_suspension.damping == p.front_suspension.damping and q.rear_suspension.damping == p.rear_suspension.damping, "suspension parameters unchanged")
	check(a.max_anchor_error() < 1.0e-5 and a.max_axis_error() < 1.0e-5, "hinges are assembled with zero error")


func test_gravitational_load_is_transmitted_through_both_wheel_contacts() -> void:
	# Test-only roll fixture (see the file header). After the suspension has settled, the two normal forces sum to the weight.
	var f: Dictionary = Rig.fixtured_motorcycle()
	check(not f.is_empty(), "fixtured motorcycle built")
	var a = f.assembly
	var weight: float = a.total_mass() * G
	var min_front: float = INF
	var min_rear: float = INF
	var ok: bool = true
	for i in 1200:
		ok = f.world.step() and ok
		min_front = minf(min_front, a.front_wheel_contact.normal_impulse())
		min_rear = minf(min_rear, a.rear_wheel_contact.normal_impulse())
	check(ok and f.world.numerical_fault == "", "10 s of loaded contact ran without a numerical fault")
	var support: float = (a.front_wheel_contact.normal_impulse() + a.rear_wheel_contact.normal_impulse()) / DT
	check_near(support, weight, weight * 1.0e-3, "front + rear normal force = total weight (%s N vs %s N)" % [support, weight])
	check(a.front_wheel_contact.normal_impulse() / DT > 0.1 * weight, "the FRONT wheel contact carries a real share (%s N)" % (a.front_wheel_contact.normal_impulse() / DT))
	check(a.rear_wheel_contact.normal_impulse() / DT > 0.1 * weight, "the REAR wheel contact carries a real share (%s N)" % (a.rear_wheel_contact.normal_impulse() / DT))
	check(min_front >= 0.0 and min_rear >= 0.0, "the normal impulses were never negative (the ground never pulls)")
	check(absf(a.front_wheel_contact.signed_gap()) < 1.0e-3 and absf(a.rear_wheel_contact.signed_gap()) < 1.0e-3, "both wheels sit on the plane (gaps %s, %s)" % [a.front_wheel_contact.signed_gap(), a.rear_wheel_contact.signed_gap()])
	check(a.max_anchor_error() < 2.0e-3 and a.max_axis_error() < 2.0e-3, "the assembly stayed together under load (anchor %s m, axis %s rad)" % [a.max_anchor_error(), a.max_axis_error()])
	check(a.chassis.position.y < MotorcyclePrototype.config().chassis.position.y, "the suspension sagged under the load (the chassis is lower)")
	check(a.ground.position == Vector3.ZERO and a.ground.linear_velocity == Vector3.ZERO, "the static ground is untouched")
	check(a.chassis.position.is_finite() and a.chassis.linear_velocity.length() < 1.0e-2, "the loaded assembly is at rest")


func test_contact_loss_and_recontact_do_not_corrupt_the_assembly() -> void:
	var f: Dictionary = Rig.fixtured_motorcycle()
	var a = f.assembly
	for i in 600:
		f.world.step()
	# Throw the whole motorcycle upward (every body gets the same velocity, so no joint is stressed): both wheels leave the ground.
	for body in a.bodies():
		body.linear_velocity += Vector3(0, 2.0, 0)
	var lost: bool = false
	var ghost: float = 0.0
	for i in 30:
		f.world.step()
		if not a.front_wheel_contact.contact_active and not a.rear_wheel_contact.contact_active:
			lost = true
			ghost = maxf(ghost, a.front_wheel_contact.normal_impulse() + a.rear_wheel_contact.normal_impulse() + a.front_wheel_contact.friction_impulse_magnitude() + a.rear_wheel_contact.friction_impulse_magnitude())
	check(lost, "both contacts became inactive while the motorcycle was airborne")
	check(ghost == 0.0, "no contact impulse acted at a distance during flight (%s)" % ghost)
	check(a.front_wheel_contact.signed_gap() > 0.05 and a.rear_wheel_contact.signed_gap() > 0.05, "both wheels are clearly above the plane")
	# Land again and settle.
	var recontacted: bool = false
	for i in 1500:
		f.world.step()
		recontacted = recontacted or (a.front_wheel_contact.contact_active and a.rear_wheel_contact.contact_active and a.front_wheel_contact.normal_impulse() > 0.0 and a.rear_wheel_contact.normal_impulse() > 0.0)
	check(recontacted, "both wheels made contact again")
	check(f.world.numerical_fault == "" and a.chassis.position.is_finite(), "no numerical fault through loss and re-contact")
	var weight: float = a.total_mass() * G
	var support: float = (a.front_wheel_contact.normal_impulse() + a.rear_wheel_contact.normal_impulse()) / DT
	check_near(support, weight, weight * 1.0e-2, "after landing the contacts again carry the weight (%s N vs %s N)" % [support, weight])
	check(a.max_anchor_error() < 5.0e-3 and a.max_axis_error() < 5.0e-3, "joints are intact after the landing (anchor %s m, axis %s rad)" % [a.max_anchor_error(), a.max_axis_error()])
	check(a.world.bodies.size() == 8 and a.world.constraints.size() == 8, "world membership unchanged (6 + ground + fixture anchor, 5 + 2 + fixture)")


func test_one_missing_wheel_contact_is_detected_by_the_load_split() -> void:
	# Regression sensitivity: remove the front contact and the motorcycle is no longer carried by the front wheel. The load test's
	# requirement that BOTH contacts carry a real share would fail.
	var f: Dictionary = Rig.fixtured_motorcycle()
	var a = f.assembly
	f.world.remove_constraint(a.front_wheel_contact)
	for i in 120:
		f.world.step()
	check(a.front_wheel.position.y < 0.2, "without a front contact the front wheel falls through the plane (y %s)" % a.front_wheel.position.y)
	check(a.front_wheel_contact.normal_impulse() / DT < 0.1 * a.total_mass() * G, "and carries no load, which the load test would reject")


func test_builder_failure_rolls_back_ground_contacts_and_everything_else() -> void:
	var world := RefusingWorld.new()
	var existing = make_sphere(1.0, 0.5)
	world.add_body(existing)
	var before_bodies: Array = world.bodies.duplicate()
	for refused in [1, 2]:
		world.refuse_contact_number = refused
		world.contacts_seen = 0
		var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_ground(), world)
		check(a == null, "build fails when the contact #%d is refused" % refused)
		check(world.bodies == before_bodies, "contact #%d refused: bodies restored (ground and six bodies removed)" % refused)
		check(world.constraints.is_empty() and world.force_providers.is_empty(), "contact #%d refused: no joint, contact or provider remains" % refused)
	# A later valid build into the SAME world succeeds.
	world.refuse_contact_number = 0
	var b = MotorcycleBuilder.build(MotorcyclePrototype.config_with_ground(), world)
	check(b != null and b.has_ground(), "a subsequent valid build into the same world succeeds")
	check(world.bodies.size() == 8 and world.constraints.size() == 7 and world.force_providers.size() == 2, "exactly one assembly is registered (existing body + 6 + ground)")
	check(world.bodies[0] == existing, "pre-existing content keeps its place")
	check(world.step(), "the rebuilt world steps")
	# An invalid configuration is refused before anything is registered.
	var bad = MotorcyclePrototype.config_with_ground()
	bad.ground.friction = -1.0
	var counts: Array = [world.bodies.size(), world.constraints.size()]
	check(MotorcycleBuilder.build(bad, world) == null and [world.bodies.size(), world.constraints.size()] == counts, "invalid ground: refused without touching the world")


func test_disabling_and_removing_the_ground_leaves_no_orphans() -> void:
	var a = build_with_ground()
	var w = a.world
	a.set_ground_contacts_enabled(false)
	check(not a.front_wheel_contact.enabled and not a.rear_wheel_contact.enabled, "both contacts disabled")
	check(a.front_wheel_contact.normal_impulse() == 0.0 and a.front_wheel_contact.friction_impulse_world() == Vector3.ZERO, "disabling discards carried impulses")
	for i in 60:
		w.step()
	check(a.rear_wheel.position.y < 0.3, "with the contacts disabled nothing supports the wheel (rear y %s)" % a.rear_wheel.position.y)
	check(a.ground.position == Vector3.ZERO, "disabled ground does not move")
	a.set_ground_contacts_enabled(true)
	check(a.front_wheel_contact.enabled and a.rear_wheel_contact.enabled, "re-enabled")
	check(w.step(), "the world steps after re-enabling")
	a.remove_ground()
	check(not a.has_ground() and a.ground == null and a.front_wheel_contact == null and a.rear_wheel_contact == null, "the assembly no longer references a ground")
	check(w.bodies.size() == 6 and w.constraints.size() == 5 and w.force_providers.size() == 2, "world back to 6 bodies, 5 joints, 2 providers")
	for c in w.constraints:
		check(w.has_body(c.body_a) and w.has_body(c.body_b), "no orphaned constraint (a constraint references a body that is gone)")
	check(w.step() and w.numerical_fault == "", "the world still steps")
	a.remove_ground()
	check(w.bodies.size() == 6, "removing twice is harmless")
	var plain = MotorcycleBuilder.build(MotorcyclePrototype.config())
	plain.remove_ground()
	plain.set_ground_contacts_enabled(false)
	check(plain.world.bodies.size() == 6, "both operations are no-ops without a ground")


func test_removing_the_ground_body_from_the_world_removes_its_contacts() -> void:
	# SimWorld.remove_body drops every constraint referencing the body: removing the ground directly leaves no orphan either.
	var a = build_with_ground()
	a.world.remove_body(a.ground)
	check(a.world.constraints.size() == 5, "both contacts were dropped with the ground body")
	check(not a.world.constraints.has(a.front_wheel_contact) and not a.world.constraints.has(a.rear_wheel_contact), "neither contact remains")
	check(a.world.step(), "the world steps")


func test_ground_stays_static_through_a_full_sustained_run_of_the_assembly() -> void:
	var f: Dictionary = Rig.fixtured_motorcycle()
	var a = f.assembly
	for i in 300:
		f.world.step()
	check(a.ground.is_static() and a.ground.position == Vector3.ZERO and a.ground.linear_velocity == Vector3.ZERO and a.ground.angular_velocity == Vector3.ZERO, "ground static, unmoved and velocity-free after 2.5 s under load")
	check(a.ground.published_state.position == Vector3.ZERO, "and its published state is unchanged")


func test_roll_instability_of_two_point_contacts_is_physical_and_recorded() -> void:
	# RECORDED LIMITATION (not a pass/fail requirement on upright balance): the assembly has no lateral support, so any roll
	# disturbance grows and the motorcycle topples. The test shows that this is the physics of the model, not a solver fault: the
	# run stays finite, the ground never pulls, and the contact lifecycle stays consistent. The undisturbed assembly only stays upright
	# because every operation is exactly symmetric about x = 0 (no lateral disturbance ever arises), which is NOT stability.
	var a = build_with_ground()
	for i in 240:
		a.world.step()
	a.chassis.angular_velocity += Vector3(0, 0, 0.01)  # small roll disturbance about the longitudinal axis
	var tilted: bool = false
	var min_normal: float = INF
	var ok: bool = true
	for i in 600:
		ok = a.world.step() and ok
		min_normal = minf(min_normal, minf(a.front_wheel_contact.normal_impulse(), a.rear_wheel_contact.normal_impulse()))
		tilted = tilted or (a.chassis.orientation * Vector3.UP).y < cos(deg_to_rad(30.0))
	check(tilted, "a 0.01 rad/s roll disturbance topples the motorcycle beyond 30 degrees within 5 s: two point contacts give no roll stability")
	check(ok and a.world.numerical_fault == "", "toppling is a physical result, not a numerical fault")
	check(min_normal >= 0.0, "the ground never pulled during the topple (min normal impulse %s)" % min_normal)
	check(a.chassis.position.is_finite() and a.front_wheel.position.is_finite() and a.rear_wheel.position.is_finite(), "the state stays finite")
