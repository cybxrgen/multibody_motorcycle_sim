extends "res://Tests/sim_test_case.gd"
## M2.5.3 Stage D correction: the motorcycle test scene's contact diagnostics (Scenes/motorcycle_test.gd, _wheel_report / markers / report text).
## A patched wheel reports the finite-width GEOMETRIC gap and lowest point (TireGeometry.lowest_point), its PHYSICAL contact from the patch's aggregate
## state, and slip at centre_of_pressure() only while that state is valid; a wheel without a patch keeps the one-point WheelGroundContact path.

const Scene = preload("res://Scenes/motorcycle_test.gd")
const TireGeometry = preload("res://Simulation/Motorcycle/tire_geometry.gd")
const TireModel = preload("res://Simulation/Motorcycle/tire_model.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")

const SETTLE_STEPS: int = 60


func make_scene(settled: bool):
	var scene = Scene.new()
	scene.wireframe.log_diagnostics = false
	scene.patch_view.log_diagnostics = false
	scene._ready()
	if settled:
		run_steps(scene.world, SETTLE_STEPS)
		for body in scene.world.bodies:
			body.reset_published_state()
	return scene


func free_scene(scene) -> void:
	var overlay = scene.debug_overlay
	if overlay.draw.get_parent() == null:
		overlay.draw.free()
	if overlay.get_parent() == null:
		overlay.free()
	scene.free()


func lowest_of(scene, k: int) -> Dictionary:
	var patch = scene._patch_of(k)
	var t: Transform3D = scene._pose(patch.body_b, 1.0)
	return TireGeometry.lowest_point(t.origin, t.basis.get_rotation_quaternion(), patch.geometry, patch.wheel_radius, patch.plane_point, patch.plane_normal)


func test_a_patched_gap_and_marker_use_the_finite_width_lowest_point() -> void:
	var scene = make_scene(false)
	var patch = scene._patch_of(0)
	# Camber the front wheel 0.4 rad about its forward axis: the finite-width lowest point leaves the centreline.
	var wheel = patch.body_b
	var before: Dictionary = scene._wheel_report(0, 1.0)
	wheel.orientation = Quaternion(Vector3(0.0, 0.0, -1.0), 0.4) * wheel.orientation
	wheel.reset_published_state()
	var report: Dictionary = scene._wheel_report(0, 1.0)
	var expected: Dictionary = lowest_of(scene, 0)
	var t: Transform3D = scene._pose(wheel, 1.0)
	var circular: Dictionary = WheelGroundContact.compute_geometry(t.origin, t.basis.get_rotation_quaternion(), patch.wheel_radius, patch.plane_point, patch.plane_normal)
	check(report.patched and report.valid and expected.valid, "patched wheel: finite-width path, valid")
	check_near(report.gap, expected.gap, 1e-9, "the gap is TireGeometry.lowest_point's gap with the configured geometry")
	check_vec3_near(report.wheel_point, expected.wheel_point, 1e-9, "the lowest point is TireGeometry.lowest_point's")
	check(absf(report.lateral_offset) > 1e-3, "the cambered finite-width point lies off the centreline")
	check(report.wheel_point.distance_to(circular.wheel_point) > 1e-3, "and differs from the circular wheel-radius projection")
	check(absf(report.gap - before.gap) > 1e-4, "the report follows the current wheel pose")
	scene._update_contact_markers(1.0)
	check_vec3_near(scene._markers[0].position, expected.wheel_point, 1e-9, "the marker sits at the finite-width lowest point")
	free_scene(scene)


func test_b_physical_contact_follows_the_aggregate_patch_state() -> void:
	var scene = make_scene(false)
	var patch = scene._patch_of(0)
	var wheel = patch.body_b
	# Push the wheel 5 cm into the ground: the circular gap says touching, but the patch has not been prepared (no aggregate contact).
	wheel.position += Vector3(0.0, -0.05, 0.0)
	wheel.reset_published_state()
	var t: Transform3D = scene._pose(wheel, 1.0)
	var circular: Dictionary = WheelGroundContact.compute_geometry(t.origin, t.basis.get_rotation_quaternion(), patch.wheel_radius, patch.plane_point, patch.plane_normal)
	check(circular.gap <= WheelGroundContact.TOUCH_TOLERANCE, "setup: the circular gap alone would say touching")
	check(not patch.has_aggregate_contact(), "setup: no aggregate physical contact")
	var report: Dictionary = scene._wheel_report(0, 1.0)
	check(not report.physical_contact, "no aggregate contact: not physically in contact despite the circular gap")
	scene._update_contact_markers(1.0)
	check(scene._markers[0].material_override.albedo_color.is_equal_approx(Color(1.0, 0.55, 0.0)), "the marker is orange (no physical contact)")
	check("front: " in scene._contact_report() and "NO PHYSICAL CONTACT" in scene._contact_report(), "the text reports no physical contact")
	free_scene(scene)

	scene = make_scene(true)
	check(scene._patch_of(0).has_aggregate_contact(), "settled: aggregate contact present")
	check(scene._wheel_report(0, 1.0).physical_contact, "aggregate contact: physically in contact")
	scene._update_contact_markers(1.0)
	check(scene._markers[0].material_override.albedo_color.is_equal_approx(Color(0.1, 0.9, 0.2)), "the marker is green")
	free_scene(scene)


func test_c_patched_slip_uses_the_centre_of_pressure() -> void:
	var scene = make_scene(true)
	for k in 2:
		var patch = scene._patch_of(k)
		var cop: Vector3 = patch.centre_of_pressure()
		var report: Dictionary = scene._wheel_report(k, 1.0)
		check(patch.has_aggregate_contact() and cop.is_finite(), "wheel %d: valid aggregate contact with a finite centre of pressure" % k)
		check(report.slip.valid, "wheel %d: slip is shown" % k)
		check_vec3_near(report.slip.contact_point, cop, 1e-9, "wheel %d: the slip contact point is centre_of_pressure()" % k)
		var pub = patch.body_b.published_state
		var direct: Dictionary = TireModel.slip(pub.position, pub.orientation, pub.linear_velocity, pub.angular_velocity, patch.wheel_radius,
			patch.tire.rolling_radius, patch.plane_point, patch.plane_normal, patch.tire.low_speed_reference, cop)
		check_near(report.slip.slip_x, direct.slip_x, 1e-12, "wheel %d: slip x equals TireModel.slip at the centre of pressure" % k)
		check_near(report.slip.slip_y, direct.slip_y, 1e-12, "wheel %d: slip y equals TireModel.slip at the centre of pressure" % k)
	free_scene(scene)


func test_d_no_stale_slip_without_aggregate_contact() -> void:
	var scene = make_scene(true)
	var patch = scene._patch_of(0)
	check(scene._wheel_report(0, 1.0).slip.valid, "setup: slip shown while in contact")
	# The last solved loads stay in the patch (centre_of_pressure() is still finite) but the support is no longer active.
	patch.contact_active = false
	check(not patch.has_aggregate_contact() and patch.centre_of_pressure().is_finite(), "setup: stale load remains, aggregate contact is gone")
	var report: Dictionary = scene._wheel_report(0, 1.0)
	check(not report.physical_contact and not report.slip.valid, "no aggregate contact: no physical contact and no slip")
	var text: String = scene._contact_report()
	check("slip n/a" in text and "NO PHYSICAL CONTACT" in text, "the text shows slip n/a and no physical contact")
	free_scene(scene)
	scene = make_scene(false)
	check(not scene._wheel_report(0, 1.0).slip.valid and not scene._wheel_report(1, 1.0).slip.valid, "never prepared: no slip either")
	free_scene(scene)


func test_e_a_wheel_without_a_patch_keeps_the_one_point_path() -> void:
	var scene = make_scene(false)
	var world = SimWorld.new()
	var assembly = MotorcycleBuilder.build(MotorcyclePrototype.config_with_tires(), world)
	check(assembly != null and assembly.front_tire_patch == null and assembly.rear_tire_patch == null, "setup: a legacy assembly without patches")
	scene.world = world
	scene.assembly = assembly
	run_steps(world, SETTLE_STEPS)
	for body in world.bodies:
		body.reset_published_state()
	for k in 2:
		var contact = assembly.ground_contacts()[k]
		var report: Dictionary = scene._wheel_report(k, 1.0)
		var t: Transform3D = scene._pose(contact.body_b, 1.0)
		var geometry: Dictionary = WheelGroundContact.compute_geometry(t.origin, t.basis.get_rotation_quaternion(), contact.wheel_radius, contact.plane_point, contact.plane_normal)
		var pub = contact.body_b.published_state
		var slip: Dictionary = TireModel.slip(pub.position, pub.orientation, pub.linear_velocity, pub.angular_velocity, contact.wheel_radius,
			contact.tire.rolling_radius, contact.plane_point, contact.plane_normal, contact.tire.low_speed_reference)
		check(not report.patched, "wheel %d: legacy path" % k)
		check_near(report.gap, geometry.gap, 1e-12, "wheel %d: circular gap" % k)
		check_vec3_near(report.wheel_point, geometry.wheel_point, 1e-12, "wheel %d: circular lowest point" % k)
		check(report.physical_contact == (geometry.gap <= WheelGroundContact.TOUCH_TOLERANCE), "wheel %d: gap-based touching" % k)
		check_vec3_near(report.slip.contact_point, slip.contact_point, 1e-12, "wheel %d: slip at the default (circular lowest) contact point" % k)
		check_near(report.slip.slip_x, slip.slip_x, 1e-12, "wheel %d: slip x as before" % k)
	scene._update_contact_markers(1.0)
	check_vec3_near(scene._markers[0].position, scene._wheel_report(0, 1.0).wheel_point, 1e-12, "the legacy marker is at the circular lowest point")
	check("TOUCHING" in scene._contact_report() or "SEPARATED" in scene._contact_report(), "legacy state words are kept")
	free_scene(scene)


func test_f_front_and_rear_patched_wheels_are_reported_independently() -> void:
	var scene = make_scene(true)
	var front: Dictionary = scene._wheel_report(0, 1.0)
	var rear: Dictionary = scene._wheel_report(1, 1.0)
	check_vec3_near(front.wheel_point, lowest_of(scene, 0).wheel_point, 1e-9, "front lowest point from the front geometry")
	check_vec3_near(rear.wheel_point, lowest_of(scene, 1).wheel_point, 1e-9, "rear lowest point from the rear geometry")
	check(front.wheel_point.distance_to(rear.wheel_point) > 0.5, "the two wheels are different points")
	check(front.physical_contact and rear.physical_contact, "both in physical contact")
	# Remove only the front aggregate contact: the rear report must not change.
	scene._patch_of(0).contact_active = false
	var front2: Dictionary = scene._wheel_report(0, 1.0)
	var rear2: Dictionary = scene._wheel_report(1, 1.0)
	check(not front2.physical_contact and not front2.slip.valid, "front: no physical contact, no slip")
	check(rear2.physical_contact and rear2.slip.valid, "rear: unaffected")
	check_vec3_near(rear2.slip.contact_point, scene._patch_of(1).centre_of_pressure(), 1e-9, "rear slip still at the rear centre of pressure")
	free_scene(scene)
