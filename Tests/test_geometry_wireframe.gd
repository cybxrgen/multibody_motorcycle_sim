extends "res://Tests/sim_test_case.gd"
## M2.5.2 geometry wireframe (Simulation/Debug/geometry_wireframe.gd): it shows ONLY the physical collision boxes, from the very objects that
## generate the box-ground contacts. Presentation only, so besides the geometry the key claim is that the simulation is bit-identical with
## the visualization on and off. Expectations are computed in this file (the box corners from explicit centre / rotation / half extents).
## Tolerance: Godot's Vector3 / Quaternion are float32; metre-sized points are good to ~2e-6, compared with 5e-6.

const GeometryWireframe = preload("res://Simulation/Debug/geometry_wireframe.gd")
const SimulationDebug = preload("res://Simulation/Debug/simulation_debug.gd")
const TireGeometry = preload("res://Simulation/Motorcycle/tire_geometry.gd")
const CollisionBox = preload("res://Simulation/Motorcycle/motorcycle_collision_box.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")

const TOL: float = 5.0e-6
const BOX_BODIES: Array = ["chassis", "steering", "fork_slider", "swingarm"]


func make_assembly(config = null):
	var world = SimWorld.new()
	var assembly = MotorcycleBuilder.build(config if config != null else MotorcyclePrototype.config_with_collision(), world)
	check(assembly != null, "the assembly builds")
	return assembly


func make_wireframe(assembly) -> GeometryWireframe:
	var w: GeometryWireframe = GeometryWireframe.new()
	w.log_diagnostics = false
	w.setup(assembly)
	return w


## World-space end points of a mesh instance's lines (the instance transform applied to the mesh vertices).
func world_vertices(instance: MeshInstance3D) -> PackedVector3Array:
	var local: PackedVector3Array = mesh_vertices(instance)
	var result: PackedVector3Array = PackedVector3Array()
	for v in local:
		result.append(instance.transform * v)
	return result


func mesh_vertices(instance: MeshInstance3D) -> PackedVector3Array:
	return instance.mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX]


## SimulationDebug adds its line batcher to itself only in _ready (inside the tree); outside the tree the batcher must be freed by hand.
func free_overlay(overlay: SimulationDebug) -> void:
	if overlay.draw.get_parent() == null:
		overlay.draw.free()
	if overlay.get_parent() == null:
		overlay.free()


func set_pose(body, position: Vector3, orientation: Quaternion) -> void:
	body.previous_state.position = position
	body.previous_state.orientation = orientation
	body.published_state.position = position
	body.published_state.orientation = orientation


func key_event(keycode: int, pressed: bool = true) -> InputEventKey:
	var key: InputEventKey = InputEventKey.new()
	key.pressed = pressed
	key.keycode = keycode
	return key


# --- Toggle and independence ---

func test_toggle_starts_off_and_switches_the_layer() -> void:
	var a = make_assembly()
	var w = make_wireframe(a)
	check(not w.enabled and not w.visible, "off by default")
	check(w.toggle() and w.enabled and w.visible, "toggle switches it on")
	check(not w.toggle() and not w.enabled and not w.visible, "and off again")
	w.free()


func test_setup_again_replaces_the_items() -> void:
	var a = make_assembly()
	var w = make_wireframe(a)
	var first = w.box_instance("chassis")
	w.set_enabled(true)
	w.setup(a)
	check(w.box_count() == 4 and w.box_instance("chassis") != first, "setup again rebuilds the four boxes")
	check(w.enabled and w.visible, "the enabled state survives a new setup")
	w.setup(null)
	check(w.box_count() == 0 and w.diagnostics.size() == 1, "a null assembly leaves it empty with a diagnostic")
	w.free()


func test_disabled_wireframe_does_not_touch_anything() -> void:
	var a = make_assembly()
	var w = make_wireframe(a)
	var instance: MeshInstance3D = w.box_instance("chassis")
	var before: Transform3D = instance.transform
	set_pose(a.chassis, Vector3(5.0, 5.0, 5.0), Quaternion.IDENTITY)
	w.update_wireframe(1.0)
	check(instance.transform == before, "no transform is written while disabled")
	w.free()


func test_other_debug_flags_are_independent() -> void:
	var a = make_assembly()
	var w = make_wireframe(a)
	var overlay: SimulationDebug = SimulationDebug.new()
	overlay.world = a.world
	overlay.show_local_axes = true
	overlay.show_com = true
	overlay.show_anchors = true
	w.toggle()
	check(overlay.show_local_axes and overlay.show_com and overlay.show_anchors, "toggling the wireframe leaves the axes, COM and anchor flags alone")
	overlay.show_com = false
	check(w.enabled, "toggling an overlay flag leaves the wireframe alone")
	w.free()
	free_overlay(overlay)


func test_scene_keys_status_text_and_controls_keys() -> void:
	var scene = load("res://Scenes/motorcycle_test.gd").new()
	scene.wireframe.log_diagnostics = false
	scene._ready()  # the runner is inside SceneTree._initialize, where adding to the tree does not run _ready yet; calling it builds the scenario
	scene._process(0.016)
	check(not scene.wireframe.enabled and "[G] collision-box + tyre wireframe: off" in scene._label.text, "wireframe starts off and the label says so")
	check("[W/Up] throttle" in scene._label.text and "[S/Down] brake" in scene._label.text and "[A/Left] steer left" in scene._label.text and "[D/Right] steer right" in scene._label.text, "the help text lists the continuous controls")
	check(not ("[T]" in scene._label.text) and not ("[B]" in scene._label.text), "the old external T / B torque toggles are gone from the help text")
	scene._unhandled_input(key_event(KEY_G))
	scene._process(0.016)
	check(scene.wireframe.enabled and scene.wireframe.visible and "wireframe: ON" in scene._label.text, "G enables it")
	check(scene.wireframe.box_count() == 4 and scene.wireframe.tire_count() == 2, "the scene's wireframe shows the four physical boxes and both tyre surfaces")
	var axes_before: bool = scene.debug_overlay.show_local_axes
	scene._unhandled_input(key_event(KEY_L))
	check(scene.debug_overlay.show_local_axes == (not axes_before) and scene.wireframe.enabled, "L still toggles the local axes only")
	var drive_before = scene.assembly.controls.throttle
	scene._unhandled_input(key_event(KEY_T))
	scene._unhandled_input(key_event(KEY_B))
	check(scene.assembly.controls.throttle == drive_before and scene.assembly.controls.brake == 0.0, "T and B no longer do anything")
	scene._unhandled_input(key_event(KEY_SPACE))
	check(scene.paused, "Space pauses")
	var steps_before: int = scene.world.step_count
	scene._unhandled_input(key_event(KEY_N))
	check(scene.world.step_count == steps_before + 1, "N performs a single step while paused")
	scene._unhandled_input(key_event(KEY_SPACE))
	check(not scene.paused, "Space resumes")
	scene._unhandled_input(key_event(KEY_R))
	check(scene.world.step_count == 0 and scene.assembly != null, "R resets the scenario")
	scene._unhandled_input(key_event(KEY_G))
	check(not scene.wireframe.enabled and not scene.wireframe.visible, "G disables it again")
	free_overlay(scene.debug_overlay)
	scene.free()


func test_keyboard_state_maps_to_normalized_inputs() -> void:
	var Scene = load("res://Scenes/motorcycle_test.gd")
	var pressed: Dictionary = {}  # one dictionary object, mutated below (a lambda captures the variable's value when it is created)
	var is_down: Callable = func(k: int) -> bool: return pressed.has(k)
	check(Scene.inputs_from_keys(is_down) == Vector3.ZERO, "no key = zero inputs")
	pressed.clear()
	pressed.merge({KEY_W: true})
	check(Scene.inputs_from_keys(is_down) == Vector3(1, 0, 0), "W = throttle")
	pressed.clear()
	pressed.merge({KEY_UP: true, KEY_W: true})
	check(Scene.inputs_from_keys(is_down) == Vector3(1, 0, 0), "W and Up together stay normalized (1)")
	pressed.clear()
	pressed.merge({KEY_S: true})
	check(Scene.inputs_from_keys(is_down) == Vector3(0, 1, 0), "S = brake")
	pressed.clear()
	pressed.merge({KEY_DOWN: true})
	check(Scene.inputs_from_keys(is_down) == Vector3(0, 1, 0), "Down = brake")
	pressed.clear()
	pressed.merge({KEY_A: true})
	check(Scene.inputs_from_keys(is_down) == Vector3(0, 0, 1), "A = steer left (+1)")
	pressed.clear()
	pressed.merge({KEY_RIGHT: true})
	check(Scene.inputs_from_keys(is_down) == Vector3(0, 0, -1), "Right = steer right (-1)")
	pressed.clear()
	pressed.merge({KEY_A: true, KEY_D: true})
	check(Scene.inputs_from_keys(is_down) == Vector3(0, 0, 0), "both steering keys cancel")
	pressed.clear()
	pressed.merge({KEY_W: true, KEY_S: true, KEY_LEFT: true})
	check(Scene.inputs_from_keys(is_down) == Vector3(1, 1, 1), "combined inputs")


# --- The boxes ARE the physical collision geometry ---

func test_only_the_four_physical_boxes_and_no_wheel_box() -> void:
	var a = make_assembly()
	var w = make_wireframe(a)
	check(w.box_count() == 4, "four boxes")
	for body_name in BOX_BODIES:
		check(w.box_instance(body_name) != null, "%s has a box" % body_name)
	check(w.box_instance("front_wheel") == null and w.box_instance("rear_wheel") == null, "no wheel box")
	check(w.tire_count() == 0 and w.get_child_count() == 4, "without tyre geometry only the four boxes are drawn")
	w.free()


func test_wireframe_geometry_equals_the_collision_geometry_used_by_the_contacts() -> void:
	var a = make_assembly()
	var w = make_wireframe(a)
	w.set_enabled(true)
	for body_name in BOX_BODIES:
		var box = a.collision_boxes[body_name]
		check(a.box_contacts[body_name][0].box == box, "%s: the contacts read the same box object the wireframe draws" % body_name)
		var body = a.box_body(body_name)
		w.update_wireframe(1.0)  # published state of a fresh world = the reference pose
		var drawn: PackedVector3Array = world_vertices(w.box_instance(body_name))
		var expected: PackedVector3Array = box.edge_vertices()
		check(drawn.size() == 24 and expected.size() == 24, "%s: 12 edges" % body_name)
		for k in mini(drawn.size(), expected.size()):
			check_vec3_near(drawn[k], body.position + body.orientation * expected[k], TOL, "%s: vertex %d is the box corner placed by the body pose" % [body_name, k])
		# Every contact corner is one of the drawn corners (same ordering and placement).
		for contact in a.box_contacts[body_name]:
			var corner: Vector3 = body.position + body.orientation * box.corner_local(contact.corner_index)
			var found: bool = false
			for v in drawn:
				found = found or (v - corner).length() < TOL
			check(found, "%s: contact corner %d is a vertex of the drawn box" % [body_name, contact.corner_index])
	w.free()


func test_box_dimensions_centre_and_orientation_are_the_configured_ones() -> void:
	var c = MotorcyclePrototype.config_with_collision()
	c.swingarm_box = CollisionBox.new("swingarm", Vector3(0.2, 0.1, 0.6), Vector3(0.0, 0.02, 0.05), Quaternion(Vector3.RIGHT, 0.1))
	var a = make_assembly(c)
	var w = make_wireframe(a)
	var vertices: PackedVector3Array = mesh_vertices(w.box_instance("swingarm"))
	var q: Quaternion = Quaternion(Vector3.RIGHT, 0.1)
	var centre: Vector3 = Vector3.ZERO
	for v in vertices:
		centre += v / float(vertices.size())
	check_vec3_near(centre, Vector3(0.0, 0.02, 0.05), TOL, "the drawn box is centred on the configured body-local centre")
	var axis_x: Vector3 = Vector3.ZERO
	var axis_y: Vector3 = Vector3.ZERO
	var axis_z: Vector3 = Vector3.ZERO
	var corner_a: Vector3 = Vector3(0.1, 0.05, 0.3)
	# Edge length along each configured box axis equals the configured dimension.
	var edges: Array = []
	for k in range(0, vertices.size(), 2):
		edges.append(vertices[k + 1] - vertices[k])
	var lengths: Array = edges.map(func(e): return snappedf(e.length(), 1.0e-5))
	check(lengths.count(snappedf(0.2, 1.0e-5)) == 4 and lengths.count(snappedf(0.1, 1.0e-5)) == 4 and lengths.count(snappedf(0.6, 1.0e-5)) == 4, "four edges per configured dimension (0.2, 0.1, 0.6)")
	var along_z: int = 0
	for e in edges:
		if absf(absf(e.normalized().dot(q * Vector3.BACK)) - 1.0) < 1.0e-5:
			along_z += 1
	check(along_z == 4, "the length-0.6 edges lie along the rotated box axis (the box orientation is applied)")
	w.free()


func test_no_box_without_configured_collision_geometry() -> void:
	var a = make_assembly(MotorcyclePrototype.config_with_tires())  # visual_size exists, collision boxes do not
	var w = make_wireframe(a)
	check(w.box_count() == 0 and w.get_child_count() == 0, "no box is synthesized from visual_size or the wheel dimensions")
	var c = MotorcyclePrototype.config_with_collision()
	c.fork_slider_box = null
	var partial = make_wireframe(make_assembly(c))
	check(partial.box_count() == 3 and partial.box_instance("fork_slider") == null, "a body without collision geometry gets no box")
	w.free()
	partial.free()


func test_boxes_follow_translation_and_rotation() -> void:
	var a = make_assembly()
	var w = make_wireframe(a)
	w.set_enabled(true)
	var position: Vector3 = Vector3(1.0, 2.0, -3.0)
	var rotation: Quaternion = Quaternion(Vector3(1.0, 2.0, 0.5).normalized(), 0.8)
	set_pose(a.chassis, position, rotation)
	w.update_wireframe(1.0)
	var drawn: PackedVector3Array = world_vertices(w.box_instance("chassis"))
	var expected: PackedVector3Array = a.collision_boxes["chassis"].edge_vertices()
	for k in drawn.size():
		check_vec3_near(drawn[k], position + rotation * expected[k], TOL, "chassis vertex %d follows the pose" % k)
	set_pose(a.swingarm, Vector3.ZERO, Quaternion.IDENTITY)
	w.update_wireframe(1.0)
	check_vec3_near(w.box_instance("swingarm").transform.origin, Vector3.ZERO, TOL, "poses are independent per box")
	w.free()


func test_runtime_geometry_change_rebuilds_only_that_box() -> void:
	var a = make_assembly()
	var w = make_wireframe(a)
	w.set_enabled(true)
	w.update_wireframe(1.0)
	var meshes_before: Dictionary = {}
	for body_name in BOX_BODIES:
		meshes_before[body_name] = w.box_instance(body_name).mesh
	w.update_wireframe(1.0)
	for body_name in BOX_BODIES:
		check(w.box_instance(body_name).mesh == meshes_before[body_name], "%s: no rebuild per frame" % body_name)
	a.collision_boxes["chassis"].size = Vector3(0.30, 0.55, 1.10)
	w.update_wireframe(1.0)
	check(w.box_instance("chassis").mesh != meshes_before["chassis"], "the changed box is rebuilt")
	for body_name in ["steering", "fork_slider", "swingarm"]:
		check(w.box_instance(body_name).mesh == meshes_before[body_name], "%s is untouched" % body_name)
	var z_max: float = -INF
	for v in mesh_vertices(w.box_instance("chassis")):
		z_max = maxf(z_max, v.z)
	check_near(z_max, 0.55, TOL, "the rebuilt box has the new half length")
	a.collision_boxes["chassis"].size = Vector3(0.30, 0.55, 1.10 + 1.0e-9)
	var mesh_now = w.box_instance("chassis").mesh
	w.update_wireframe(1.0)
	check(w.box_instance("chassis").mesh == mesh_now, "a change below the tolerance rebuilds nothing")
	a.collision_boxes["swingarm"].size = Vector3(0.2, -0.1, 0.5)
	w.update_wireframe(1.0)
	check(w.box_instance("swingarm") == null and not w.diagnostics.is_empty(), "an invalid box is skipped with a diagnostic")
	a.collision_boxes["swingarm"].size = Vector3(0.2, 0.1, 0.5)
	w.update_wireframe(1.0)
	check(w.box_instance("swingarm") != null, "it comes back when valid again")
	a.remove_collision_box("swingarm")
	w.update_wireframe(1.0)
	check(w.box_instance("swingarm") == null and w.box_count() == 3, "a removed box disappears from the wireframe")
	w.free()


func snapshot(world) -> Array:
	var data: Array = []
	for body in world.bodies:
		data.append([body.position, body.orientation, body.linear_velocity, body.angular_velocity])
	return data


func test_simulation_is_identical_with_the_visualization_on_and_off() -> void:
	var world_a = SimWorld.new()
	var assembly_a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision(), world_a)
	var world_b = SimWorld.new()
	var assembly_b = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision(), world_b)
	var w: GeometryWireframe = make_wireframe(assembly_b)
	w.set_enabled(true)
	var world_c = SimWorld.new()
	var assembly_c = MotorcycleBuilder.build(MotorcyclePrototype.config_with_collision(), world_c)
	var w_c: GeometryWireframe = make_wireframe(assembly_c)
	for k in 240:
		world_a.step()
		world_b.step()
		world_c.step()
		w.update_wireframe(float(k % 4) / 4.0)
		if k % 7 == 0:
			w_c.toggle()
		w_c.update_wireframe(1.0)
	check(snapshot(world_a) == snapshot(world_b) and snapshot(world_a) == snapshot(world_c), "every body state is bit-identical with the wireframe off, on, and toggled during the run")
	check(world_a.step_count == world_b.step_count and world_a.simulation_time == world_b.simulation_time, "step count and time are identical")
	w.free()
	w_c.free()


func test_tyre_surfaces_are_drawn_from_the_tyre_geometry_and_follow_the_wheels() -> void:
	var c = MotorcyclePrototype.config_with_collision()
	c.front_tire_geometry = TireGeometry.new(0.12, 0.07)
	c.rear_tire_geometry = TireGeometry.new(0.18, 0.10)
	var a = make_assembly(c)
	var w = make_wireframe(a)
	w.set_enabled(true)
	check(w.tire_count() == 2 and w.box_count() == 4 and w.get_child_count() == 6, "two tyre surfaces besides the four boxes")
	for pair in [["front", a.front_wheel, 0.12, 0.31], ["rear", a.rear_wheel, 0.18, 0.32]]:
		var vertices: PackedVector3Array = mesh_vertices(w.tire_instance(pair[0]))
		var s_max: float = 0.0
		var r_max: float = 0.0
		for v in vertices:
			s_max = maxf(s_max, absf(v.x))
			r_max = maxf(r_max, Vector2(v.y, v.z).length())
		check_near(s_max, 0.5 * pair[2], TOL, "%s surface spans the tyre width" % pair[0])
		check_near(r_max, pair[3], TOL, "%s surface reaches the wheel radius" % pair[0])
		check(w.tire_instance(pair[0]).get_parent() == w, "drawn by the wireframe")
	set_pose(a.rear_wheel, Vector3(1.0, 0.5, 2.0), Quaternion.IDENTITY)
	w.update_wireframe(1.0)
	check_vec3_near(w.tire_instance("rear").transform.origin, Vector3(1.0, 0.5, 2.0), TOL, "the tyre follows its wheel")
	w.free()
