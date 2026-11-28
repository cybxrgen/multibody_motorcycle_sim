extends "res://Tests/sim_test_case.gd"
## M2.5.3 Stage D: the contact-patch debug view (Simulation/Debug/tire_patch_debug.gd) on the ASSEMBLED motorcycle: it is optional, draws the
## candidate and the active samples with their normals and relative normal support, distinguishes the wheels, follows the wheel poses, reflects the
## CURRENT patch state, leaves the tyre wireframe alone and never influences the simulation.

const Rig = preload("res://Tests/patch_rig.gd")
const TirePatchDebug = preload("res://Simulation/Debug/tire_patch_debug.gd")
const GeometryWireframe = preload("res://Simulation/Debug/geometry_wireframe.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")

const FULL: int = MotorcyclePrototype.PATCH_LATERAL_COUNT * MotorcyclePrototype.PATCH_CIRCUMFERENTIAL_COUNT


func make_view(assembly, enabled: bool = true) -> TirePatchDebug:
	var view: TirePatchDebug = TirePatchDebug.new()
	view.log_diagnostics = false
	view.setup(assembly)
	view.set_enabled(enabled)
	return view


func surface_arrays(view: TirePatchDebug, side: String) -> Array:
	var instance = view.instance(side)
	return instance.mesh.surface_get_arrays(0) if instance != null else []


func snapshot(world) -> Array:
	var data: Array = []
	for body in world.bodies:
		data.append([body.position, body.orientation, body.linear_velocity, body.angular_velocity])
	return data


func test_e1_the_view_is_off_by_default_toggles_and_draws_nothing_while_disabled() -> void:
	var a = Rig.assemble()
	var view: TirePatchDebug = make_view(a, false)
	check(not view.enabled and not view.visible, "off by default and hidden")
	Rig.step(a, 30)
	view.update_patch_view(1.0)
	check(view.instance("front") == null and view.instance("rear") == null and view.get_child_count() == 0 and view.segment_count("front") == 0, "a disabled view draws nothing")
	check(view.toggle() and view.enabled and view.visible, "toggle() enables it")
	view.update_patch_view(1.0)
	check(view.instance("front") != null and view.instance("rear") != null, "and it draws both wheels")
	check(not view.toggle() and not view.visible, "toggle() disables it again")
	view.setup(a)
	check(view.get_child_count() == 0, "setup() again replaces the content")
	view.free()


func test_e2_front_and_rear_are_distinct_and_candidates_differ_from_active_samples() -> void:
	var a = Rig.assemble()
	Rig.step(a, 240)
	var view: TirePatchDebug = make_view(a)
	view.update_patch_view(1.0)
	check(view.instance("front") != view.instance("rear") and view.get_child_count() == 2, "one separate line mesh per wheel")
	var colors_by_side: Dictionary = {}
	for side in ["front", "rear"]:
		var patch = Rig.patch_of(a, side)
		check(patch.active_sample_count() > 0, "%s: in contact" % side)
		check(view.active_positions(side).size() == patch.active_sample_count(), "%s: the active samples drawn are the patch's physically active samples" % side)
		check(view.active_positions(side).size() + view.candidate_positions(side).size() == FULL, "%s: every candidate sample is drawn, either inactive or active" % side)
		check(view.candidate_positions(side).size() > 0 and view.segment_count(side) == FULL, "%s: inactive candidates remain visible beside the active samples" % side)
		var arrays: Array = surface_arrays(view, side)
		check(arrays.size() > 0 and arrays[Mesh.ARRAY_VERTEX].size() == 2 * FULL, "%s: one normal tick (two vertices) per sample" % side)
		var seen: Dictionary = {}
		for color in arrays[Mesh.ARRAY_COLOR]:
			seen[color] = true
		colors_by_side[side] = seen
		check(seen.size() >= 2, "%s: candidates and active samples use different colours (%d colours)" % [side, seen.size()])
	var shared: int = 0
	for color in colors_by_side["front"]:
		if colors_by_side["rear"].has(color) and color != TirePatchDebug.BOTTOMED_COLOR:
			shared += 1
	check(shared == 0, "the front and the rear wheel are drawn in different colours")
	view.free()


func test_e3_samples_and_normals_follow_the_actual_wheel_poses() -> void:
	var a = Rig.assemble()
	Rig.step(a, 60)
	var view: TirePatchDebug = make_view(a)
	Rig.move_rigidly(a, Quaternion(Vector3.UP, 0.5) * Quaternion(Vector3.BACK, 0.1), Vector3(1.2, -0.004, -0.6))
	Rig.step(a, 3)
	view.update_patch_view(1.0)
	for side in ["front", "rear"]:
		var wheel = Rig.wheel_of(a, side)
		var arrays: Array = surface_arrays(view, side)
		var vertices: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX]
		var centroid: Vector3 = Vector3.ZERO
		var outward: bool = true
		var unit: bool = true
		for k in range(0, vertices.size(), 2):
			centroid += vertices[k]
			var tick: Vector3 = vertices[k + 1] - vertices[k]
			unit = unit and tick.length() > 0.005 and tick.length() < 0.13
			outward = outward and tick.normalized().dot(vertices[k] - wheel.published_state.position) > 0.0
		centroid /= float(vertices.size() / 2)
		check((centroid - wheel.published_state.position).length() < 1.0e-4, "%s: the sample ring is centred on the wheel's published pose (%s m off)" % [side, (centroid - wheel.published_state.position).length()])
		check(unit, "%s: every tick has a sensible length" % side)
		check(outward, "%s: every tick points along the outward tyre-surface normal" % side)
	# The interpolated pose: alpha 0 is the previous published state, alpha 1 the latest one.
	view.update_patch_view(0.0)
	var arrays_0: Array = surface_arrays(view, "rear")
	var c0: Vector3 = Vector3.ZERO
	for k in range(0, arrays_0[Mesh.ARRAY_VERTEX].size(), 2):
		c0 += arrays_0[Mesh.ARRAY_VERTEX][k]
	c0 /= float(arrays_0[Mesh.ARRAY_VERTEX].size() / 2)
	check((c0 - a.rear_wheel.previous_state.position).length() < 1.0e-4, "alpha 0 follows the previous published pose")
	view.free()


func test_e4_the_view_reflects_the_current_patch_state_not_a_cache() -> void:
	var a = Rig.assemble()
	var view: TirePatchDebug = make_view(a)
	Rig.step(a, 1)
	view.update_patch_view(1.0)
	check(view.active_positions("rear").is_empty() and view.candidate_positions("rear").size() == FULL, "before any physical contact all candidates are inactive")
	Rig.step(a, 240)
	view.update_patch_view(1.0)
	var settled_active: int = view.active_positions("rear").size()
	check(settled_active == a.rear_tire_patch.active_sample_count() and settled_active > 5, "in contact: the active samples are the patch's")
	Rig.shift(a, Vector3(0.0, 0.08, 0.0))
	Rig.step(a, 1)
	view.update_patch_view(1.0)
	check(view.active_positions("rear").is_empty() and view.active_positions("front").is_empty() and view.candidate_positions("front").size() == FULL,
		"after complete separation no active sample is left over")
	check(view.segment_count("front") == FULL and view.segment_count("rear") == FULL, "the candidates are still drawn")
	Rig.step(a, 12)
	view.update_patch_view(1.0)
	check(view.active_positions("rear").size() == a.rear_tire_patch.active_sample_count() and view.active_positions("front").size() == a.front_tire_patch.active_sample_count(),
		"and after re-contact the view matches the patch again")
	view.free()


func test_e5_the_relative_normal_support_is_drawn_as_the_tick_length() -> void:
	var a = Rig.assemble()
	Rig.step(a, 240)
	var view: TirePatchDebug = make_view(a)
	view.update_patch_view(1.0)
	for side in ["front", "rear"]:
		var support: PackedFloat32Array = view.active_relative_support(side)
		var ticks: PackedFloat32Array = view.active_tick_lengths(side)
		var report: Array = Rig.patch_of(a, side).sample_report()
		check(support.size() == ticks.size() and support.size() == report.size(), "%s: one support value per active sample" % side)
		var highest: float = 0.0
		var in_range: bool = true
		for value in support:
			highest = maxf(highest, value)
			in_range = in_range and value >= 0.0 and value <= 1.0
		check(in_range and absf(highest - 1.0) < 1.0e-6, "%s: relative support lies in [0, 1] and the most loaded sample is 1" % side)
		var ordered: bool = true
		for i in report.size():
			for j in report.size():
				if report[i].load > report[j].load + 1.0e-3:
					ordered = ordered and ticks[i] >= ticks[j] - 1.0e-7
		check(ordered, "%s: the more loaded a sample, the longer its tick" % side)
		check(support.size() > 3 and ticks.size() > 3, "%s: several samples" % side)
		var longest: float = 0.0
		var shortest: float = INF
		for t in ticks:
			longest = maxf(longest, t)
			shortest = minf(shortest, t)
		check_near(longest, TirePatchDebug.ACTIVE_TICK_MAX, 1.0e-6, "%s: the most loaded sample has the longest tick" % side)
		check(shortest < longest - 0.005 and shortest >= TirePatchDebug.ACTIVE_TICK_MIN - 1.0e-7, "%s: the load distribution is visible (tick lengths differ)" % side)
	view.free()


func test_e6_the_simulation_is_identical_with_the_view_off_on_and_toggled() -> void:
	var worlds: Array = []
	var assemblies: Array = []
	for k in 3:
		var a = Rig.assemble()
		assemblies.append(a)
		worlds.append(a.world)
	var on: TirePatchDebug = make_view(assemblies[1])
	var toggled: TirePatchDebug = make_view(assemblies[2], false)
	for k in 240:
		for a in assemblies:
			a.world.step()
		on.update_patch_view(float(k % 4) / 4.0)
		if k % 9 == 0:
			toggled.toggle()
		toggled.update_patch_view(1.0)
	check(snapshot(worlds[0]) == snapshot(worlds[1]) and snapshot(worlds[0]) == snapshot(worlds[2]), "every body state is bit-identical with the view off, on and toggled")
	check(worlds[0].step_count == worlds[1].step_count and worlds[0].simulation_time == worlds[1].simulation_time, "step count and time are identical")
	check(worlds[0].constraints.size() == worlds[1].constraints.size() and worlds[0].bodies.size() == worlds[1].bodies.size(), "and the view registered nothing in the world")
	on.free()
	toggled.free()


func test_e7_the_tyre_wireframe_is_neither_replaced_nor_duplicated() -> void:
	var a = Rig.assemble()
	Rig.step(a, 120)
	var wire: GeometryWireframe = GeometryWireframe.new()
	wire.log_diagnostics = false
	wire.setup(a)
	wire.set_enabled(true)
	wire.update_wireframe(1.0)
	var before: Array = [wire.tire_count(), wire.box_count(), wire.get_child_count(), wire.tire_instance("front").mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX].size()]
	var view: TirePatchDebug = make_view(a)
	view.update_patch_view(1.0)
	wire.update_wireframe(1.0)
	var after: Array = [wire.tire_count(), wire.box_count(), wire.get_child_count(), wire.tire_instance("front").mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX].size()]
	check(before == after and before[0] == 2, "the tyre wireframe is unchanged by the patch view (two tyre surfaces, same vertices)")
	check(view.get_parent() == null and wire.get_children().has(view.instance("front")) == false, "the patch view draws in its own node, never inside the wireframe")
	check(view.get_child_count() == 2, "the patch view adds exactly its own two line meshes")
	view.free()
	wire.free()


func test_e8_a_wheel_without_a_patch_shows_nothing() -> void:
	var a = MotorcycleBuilder.build(MotorcyclePrototype.config_with_tires())
	var view: TirePatchDebug = make_view(a)
	Rig.step(a, 30)
	view.update_patch_view(1.0)
	check(view.instance("front") == null and view.instance("rear") == null and view.get_child_count() == 0, "one-point wheels have no patch view")
	var mixed_config = MotorcyclePrototype.config_with_patches()
	mixed_config.rear_tire_patch = null
	var mixed = MotorcycleBuilder.build(mixed_config)
	var mixed_view: TirePatchDebug = make_view(mixed)
	Rig.step(mixed, 120)
	mixed_view.update_patch_view(1.0)
	check(mixed_view.instance("front") != null and mixed_view.instance("rear") == null, "only the wheel with a patch is drawn")
	var empty: TirePatchDebug = TirePatchDebug.new()
	empty.log_diagnostics = false
	empty.setup(null)
	empty.set_enabled(true)
	empty.update_patch_view(1.0)
	check(empty.get_child_count() == 0 and not empty.diagnostics.is_empty(), "no assembly: an empty view with a diagnostic, no error")
	view.free()
	mixed_view.free()
	empty.free()


func test_e9_the_motorcycle_scene_has_the_patch_view_on_the_p_key() -> void:
	var scene = load("res://Scenes/motorcycle_test.gd").new()
	scene.wireframe.log_diagnostics = false
	scene.patch_view.log_diagnostics = false
	scene._ready()
	scene._process(0.016)
	check(scene.assembly.front_tire_patch != null and scene.assembly.rear_tire_patch != null, "the scene's motorcycle has a distributed patch on each wheel")
	check(not scene.patch_view.enabled and "[P] contact-patch samples" in scene._label.text, "the patch view starts off and the help text names the P key")
	var key: InputEventKey = InputEventKey.new()
	key.pressed = true
	key.keycode = KEY_P
	scene._unhandled_input(key)
	scene._process(0.016)
	check(scene.patch_view.enabled and scene.patch_view.visible, "P enables it")
	check(scene.patch_view.instance("front") != null and scene.patch_view.instance("rear") != null, "and both wheels are drawn")
	check(not scene.wireframe.enabled, "G (the tyre wireframe) is independent")
	scene._unhandled_input(key)
	check(not scene.patch_view.enabled and not scene.patch_view.visible, "P disables it again")
	check("tyre contacts" in scene._contact_report() and "front:" in scene._contact_report(), "the contact report still works with patch wheels")
	var overlay = scene.debug_overlay
	if overlay.draw.get_parent() == null:
		overlay.draw.free()
	if overlay.get_parent() == null:
		overlay.free()
	scene.free()
