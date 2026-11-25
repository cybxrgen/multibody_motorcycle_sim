extends "res://Tests/sim_test_case.gd"
## M2.5.1.2: toggleable geometry wireframe (Simulation/Debug/geometry_wireframe.gd): oriented bounding boxes of the six bodies and the
## finite-width tyre surfaces. Presentation only, so besides the geometry the key claim tested here is that the simulation is
## bit-identical with the visualization on and off.
##
## Expectations are computed IN THIS FILE: box corners from explicit rotations of explicit half extents, the tyre radius from the closed
## form r(s) = R - rho + sqrt(rho^2 - s^2), and the tyre/TireGeometry agreement against TireGeometry.point_at_offset (the contact
## geometry's own evaluation of a circle of the tread). Tolerances: Godot's Vector3/Quaternion are float32, so poses/points of metre size
## are good to ~2e-6 and are compared with 5e-6; the lowest-vertex test compares a DISCRETE mesh with the exact lowest point and its
## tolerance (1.5e-3 m) is the sampling error of the documented resolution (48 angles, 12 profile segments), derived where it is used.

const GeometryWireframe = preload("res://Simulation/Debug/geometry_wireframe.gd")
const SimulationDebug = preload("res://Simulation/Debug/simulation_debug.gd")
const TireGeometry = preload("res://Simulation/Motorcycle/tire_geometry.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")

const TOL: float = 5.0e-6
const BODY_NAMES: Array = ["chassis", "steering", "fork_slider", "swingarm", "front_wheel", "rear_wheel"]


## The scene's illustrative configuration: ground, tyres and a different tyre geometry per wheel.
func make_config(with_geometry: bool = true):
	var c = MotorcyclePrototype.config_with_tires()
	if with_geometry:
		c.front_tire_geometry = TireGeometry.new(0.12, 0.07)
		c.rear_tire_geometry = TireGeometry.new(0.18, 0.10)
	return c


func make_assembly(with_geometry: bool = true):
	var world = SimWorld.new()
	var assembly = MotorcycleBuilder.build(make_config(with_geometry), world)
	check(assembly != null, "the assembly builds")
	return assembly


func make_wireframe(assembly) -> GeometryWireframe:
	var w: GeometryWireframe = GeometryWireframe.new()
	w.log_diagnostics = false
	w.setup(assembly)
	return w


func free_wireframe(w: GeometryWireframe) -> void:
	w.free()


## World-space end points of a mesh instance's lines (the instance transform applied to the mesh vertices).
func world_vertices(instance: MeshInstance3D) -> PackedVector3Array:
	var local: PackedVector3Array = instance.mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX]
	var result: PackedVector3Array = PackedVector3Array()
	for v in local:
		result.append(instance.transform * v)
	return result


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


# --- Toggle ---

func test_toggle_starts_off_and_switches_both_layers() -> void:
	var assembly = make_assembly()
	var w: GeometryWireframe = make_wireframe(assembly)
	check(not w.enabled and not w.visible, "starts disabled and hidden")
	check(w.box_count() == 6 and w.tire_count() == 2, "both layers are built (6 boxes, 2 tyres) but hidden")
	for round in 3:
		check(w.toggle() and w.enabled and w.visible, "toggle %d enables the layers" % round)
		w.update_wireframe(1.0)
		check(w.box_instance("chassis").visible and w.tire_instance("front").visible and w.tire_instance("rear").visible, "boxes and tyres are shown together")
		check(not w.toggle() and not w.enabled and not w.visible, "toggle %d disables them again" % round)
	free_wireframe(w)


## A reset rebuilds the scene's assembly: setup() replaces the old items and keeps the enabled state.
func test_setup_again_replaces_items_and_keeps_state() -> void:
	var w: GeometryWireframe = make_wireframe(make_assembly())
	w.set_enabled(true)
	var second = make_assembly()
	w.setup(second)
	check(w.enabled and w.visible and w.box_count() == 6 and w.tire_count() == 2, "rebuilt for the new assembly, still enabled")
	check(w.get_child_count() == 8, "no stale items remain (8 children)")
	w.update_wireframe(1.0)
	check_vec3_near(w.box_instance("chassis").transform.origin, second.chassis.published_state.position, TOL, "follows the NEW assembly")
	free_wireframe(w)


func test_disabled_wireframe_does_not_touch_anything() -> void:
	var assembly = make_assembly()
	var w: GeometryWireframe = make_wireframe(assembly)
	var before: Transform3D = w.box_instance("chassis").transform
	set_pose(assembly.chassis, Vector3(3, 2, 1), Quaternion(Vector3.UP, 0.7))
	w.update_wireframe(1.0)
	check(w.box_instance("chassis").transform == before, "no update work while disabled")
	free_wireframe(w)


func test_other_debug_flags_are_independent() -> void:
	var overlay: SimulationDebug = SimulationDebug.new()
	var defaults: Array = [overlay.show_world_axes, overlay.show_com, overlay.show_local_axes, overlay.show_anchors]
	var w: GeometryWireframe = make_wireframe(make_assembly())
	w.toggle()
	w.toggle()
	w.toggle()
	check([overlay.show_world_axes, overlay.show_com, overlay.show_local_axes, overlay.show_anchors] == defaults, "the toggle has no link to the overlay flags")
	overlay.show_local_axes = true
	check(not w.enabled or w.enabled, "")  # the wireframe state is its own variable
	check(w.enabled, "toggling an overlay flag does not change the wireframe state (it stays as toggled)")
	free_wireframe(w)
	free_overlay(overlay)


## The scene script: G toggles the wireframe, the existing keys keep working, the label shows the state (script-level; no rendering).
func test_scene_keys_and_status_text() -> void:
	var scene = load("res://Scenes/motorcycle_test.gd").new()
	scene.wireframe.log_diagnostics = false
	scene._ready()  # the runner is itself inside SceneTree._initialize, where adding to the tree does not run _ready yet; calling it builds the scenario
	scene._process(0.016)
	check(not scene.wireframe.enabled, "scene starts with the wireframe off")
	check("[G] geometry wireframe: off" in scene._label.text, "status text shows the key and the state (off)")
	var key: InputEventKey = InputEventKey.new()
	key.pressed = true
	key.keycode = KEY_G
	scene._unhandled_input(key)
	scene._process(0.016)
	check(scene.wireframe.enabled and scene.wireframe.visible, "G enables it")
	check("[G] geometry wireframe: ON" in scene._label.text, "status text shows ON")
	check(scene.wireframe.box_count() == 6 and scene.wireframe.tire_count() == 2, "the scene's assembly has six boxes and two tyres")
	var axes_before: bool = scene.debug_overlay.show_local_axes
	var l: InputEventKey = InputEventKey.new()
	l.pressed = true
	l.keycode = KEY_L
	scene._unhandled_input(l)
	check(scene.debug_overlay.show_local_axes == (not axes_before) and scene.wireframe.enabled, "L still toggles the local axes and leaves the wireframe alone")
	var space: InputEventKey = InputEventKey.new()
	space.pressed = true
	space.keycode = KEY_SPACE
	scene._unhandled_input(space)
	check(scene.paused, "Space still pauses")
	scene._unhandled_input(key)
	check(not scene.wireframe.enabled and not scene.wireframe.visible, "G disables it again")
	free_overlay(scene.debug_overlay)
	scene.free()


# --- Bounding boxes ---

func test_six_bodies_with_configured_dimensions() -> void:
	var assembly = make_assembly()
	var c = assembly.config
	var entries: Array = GeometryWireframe.bounds_entries(assembly)
	check(entries.size() == 6, "six boxes")
	var names: Array = []
	for e in entries:
		names.append(e.name)
	for expected in BODY_NAMES:
		check(expected in names, "box for %s" % expected)
	var by_name: Dictionary = {}
	for e in entries:
		by_name[e.name] = e
	check_vec3_near(by_name["chassis"].half_extents, Vector3(0.15, 0.275, 0.5), 1.0e-7, "chassis 0.30 x 0.55 x 1.00 m")
	check_vec3_near(by_name["steering"].half_extents, Vector3(0.30, 0.15, 0.08), 1.0e-7, "steering 0.60 x 0.30 x 0.16 m")
	check_vec3_near(by_name["fork_slider"].half_extents, Vector3(0.12, 0.20, 0.07), 1.0e-7, "fork slider 0.24 x 0.40 x 0.14 m")
	check_near(by_name["swingarm"].half_extents.z, 0.5 * c.swingarm.visual_size.z, 1.0e-7, "swingarm length")
	# Wheels: half width of the TYRE geometry (0.12 / 0.18), radius of the contact geometry (0.31 / 0.32).
	check_vec3_near(by_name["front_wheel"].half_extents, Vector3(0.06, 0.31, 0.31), 1.0e-7, "front wheel: tyre width 0.12, radius 0.31")
	check_vec3_near(by_name["rear_wheel"].half_extents, Vector3(0.09, 0.32, 0.32), 1.0e-7, "rear wheel: tyre width 0.18, radius 0.32")
	for e in entries:
		check_vec3_near(e.center, Vector3.ZERO, 0.0, "%s box is centred on the COM frame" % e.name)


## The prototype's four box dimensions are the ones that produced the inertia tensors: for a uniform box
## I_xx = m (y^2 + z^2) / 12 etc., so x^2 = 6 (I_yy + I_zz - I_xx) / m. This ties the boxes to the physical mass distribution.
func test_box_sizes_agree_with_the_inertia_boxes_of_the_prototype() -> void:
	var c = MotorcyclePrototype.config()
	for spec in [c.chassis, c.steering, c.fork_slider, c.swingarm]:
		var i: Basis = spec.local_inertia
		var m: float = spec.mass
		var size: Vector3 = Vector3(sqrt(6.0 * (i.y.y + i.z.z - i.x.x) / m), sqrt(6.0 * (i.x.x + i.z.z - i.y.y) / m), sqrt(6.0 * (i.x.x + i.y.y - i.z.z) / m))
		check_vec3_near(size, spec.visual_size, 1.0e-5, "%s: size recovered from the inertia tensor equals the configured size" % spec.body_name)


func test_wheel_without_tire_geometry_uses_the_config_width() -> void:
	var assembly = make_assembly(false)
	var by_name: Dictionary = {}
	for e in GeometryWireframe.bounds_entries(assembly):
		by_name[e.name] = e
	check(by_name.size() == 6, "six boxes without tyre geometry")
	check_vec3_near(by_name["front_wheel"].half_extents, Vector3(0.5 * assembly.config.front_wheel_width, 0.31, 0.31), 1.0e-7, "front wheel falls back to the wheel width")
	var w: GeometryWireframe = make_wireframe(assembly)
	check(w.box_count() == 6 and w.tire_count() == 0, "boxes yes, tyre surfaces none")
	check(w.diagnostics.size() == 2, "one diagnostic per missing tyre geometry (got %s)" % [w.diagnostics])
	free_wireframe(w)


func test_box_edges() -> void:
	var v: PackedVector3Array = GeometryWireframe.box_line_vertices(Vector3(1, 2, 3), Vector3(10, 20, 30))
	check(v.size() == 24, "12 edges")
	var lengths: Array = [0, 0, 0]
	for k in range(0, v.size(), 2):
		var d: Vector3 = v[k + 1] - v[k]
		check(is_equal_approx(d.length(), 2.0) or is_equal_approx(d.length(), 4.0) or is_equal_approx(d.length(), 6.0), "edge length is a full side length")
		lengths[0] += 1 if is_equal_approx(d.length(), 2.0) else 0
		lengths[1] += 1 if is_equal_approx(d.length(), 4.0) else 0
		lengths[2] += 1 if is_equal_approx(d.length(), 6.0) else 0
	check(lengths == [4, 4, 4], "four edges per axis")
	for p in v:
		check(absf(p.x - 10.0) == 1.0 and absf(p.y - 20.0) == 2.0 and absf(p.z - 30.0) == 3.0, "every end point is a corner")
	check(GeometryWireframe.box_line_vertices(Vector3(1, 0, 1)).is_empty(), "a zero extent gives no box")
	check(GeometryWireframe.box_line_vertices(Vector3(1, NAN, 1)).is_empty(), "a NaN extent gives no box")
	check(GeometryWireframe.box_line_vertices(Vector3(1, -1, 1)).is_empty(), "a negative extent gives no box")


func test_boxes_follow_translation_and_rotation() -> void:
	var assembly = make_assembly()
	var w: GeometryWireframe = make_wireframe(assembly)
	w.set_enabled(true)
	var q: Quaternion = Quaternion(Vector3(1, 2, 3).normalized(), 0.9)
	var p: Vector3 = Vector3(1.5, -0.4, 2.0)
	set_pose(assembly.swingarm, p, q)
	w.update_wireframe(1.0)
	var half: Vector3 = assembly.config.swingarm.visual_size * 0.5
	var instance: MeshInstance3D = w.box_instance("swingarm")
	var world: PackedVector3Array = world_vertices(instance)
	var basis: Basis = Basis(q)
	var worst: float = 0.0
	for v in world:
		# Every end point must be a corner: p + R * (+-half), computed here from the quaternion.
		var local: Vector3 = basis.inverse() * (v - p)
		worst = maxf(worst, (Vector3(absf(local.x), absf(local.y), absf(local.z)) - half).length())
	check(worst < TOL, "all end points are corners of the box rotated and translated with the body (worst %s)" % worst)
	# Orientation: the first edge runs along the body's local X axis, rotated into the world; not along a world axis.
	var edge: Vector3 = (world[1] - world[0]).normalized()
	check_vec3_near(edge.abs(), (basis * Vector3.RIGHT).abs(), TOL, "the first edge is the body's local X axis in the world")
	check(edge.abs().x < 0.99, "the box is not world-axis-aligned")
	# Interpolation of the pose is the one of the existing overlay.
	w.update_wireframe(0.0)
	check_vec3_near(instance.transform.origin, assembly.swingarm.previous_state.position, TOL, "alpha 0 uses the previous state")
	free_wireframe(w)


func test_missing_or_invalid_geometry_is_skipped_safely() -> void:
	var w0: GeometryWireframe = GeometryWireframe.new()
	w0.log_diagnostics = false
	w0.setup(null)
	check(w0.box_count() == 0 and w0.tire_count() == 0 and not w0.diagnostics.is_empty(), "a null assembly gives an empty wireframe with a diagnostic")
	w0.set_enabled(true)
	w0.update_wireframe(1.0)  # must not raise
	free_wireframe(w0)

	var assembly = make_assembly()
	assembly.swingarm = null
	assembly.config.chassis.visual_size = Vector3(0.3, 0.0, 1.0)
	assembly.rear_tire_geometry.width = NAN
	var w: GeometryWireframe = make_wireframe(assembly)
	check(w.box_count() == 3, "swingarm (no body), chassis (zero size) and rear wheel (NaN width) are skipped: %d boxes" % w.box_count())
	check(w.tire_count() == 1 and w.tire_instance("rear") == null and w.tire_instance("front") != null, "the invalid rear tyre is skipped, the front one is drawn")
	check(w.diagnostics.size() >= 3, "each skipped item is reported (got %s)" % [w.diagnostics])
	w.set_enabled(true)
	w.update_wireframe(1.0)
	assembly.front_tire_geometry = null  # the reference disappears while running
	w.update_wireframe(1.0)
	check(w.tire_count() == 0, "a vanished geometry reference removes the surface instead of crashing")
	free_wireframe(w)


# --- Tyre surface ---

func test_tire_vertices_follow_the_profile_and_cover_the_width() -> void:
	var geometry = TireGeometry.new(0.18, 0.10)
	var radius: float = 0.32
	var v: PackedVector3Array = GeometryWireframe.tire_line_vertices(geometry, radius)
	check(v.size() == 2 * GeometryWireframe.LINE_SEGMENTS_PER_TIRE, "vertex count is the documented, bounded one (%d)" % v.size())
	check(v.size() <= 4000, "bounded resolution")
	var min_x: float = INF
	var max_x: float = -INF
	var max_radial: float = 0.0
	var worst: float = 0.0
	var shoulder_radius: float = INF
	for p in v:
		min_x = minf(min_x, p.x)
		max_x = maxf(max_x, p.x)
		var radial: float = sqrt(p.y * p.y + p.z * p.z)
		max_radial = maxf(max_radial, radial)
		var expected: float = radius - 0.10 + sqrt(0.10 * 0.10 - p.x * p.x)  # closed form, written here
		worst = maxf(worst, absf(radial - expected))
		if absf(absf(p.x) - 0.09) < 1.0e-6:
			shoulder_radius = minf(shoulder_radius, radial)
	check_near(min_x, -0.09, 1.0e-6, "the surface reaches the left shoulder (finite width)")
	check_near(max_x, 0.09, 1.0e-6, "and the right shoulder")
	check(worst < TOL, "every vertex lies on the circular-crown profile (worst %s)" % worst)
	check_near(max_radial, radius, TOL, "the largest radius is the wheel radius (at the centreline)")
	check_near(shoulder_radius, radius - 0.10 + sqrt(0.10 * 0.10 - 0.09 * 0.09), TOL, "the shoulder radius is smaller by the shoulder drop")
	check(shoulder_radius < radius - 0.02, "the cross-section is not a cylinder")
	check(GeometryWireframe.tire_line_vertices(null, radius).is_empty(), "no geometry, no surface")
	check(GeometryWireframe.tire_line_vertices(geometry, NAN).is_empty(), "invalid wheel radius, no surface")
	check(GeometryWireframe.tire_line_vertices(TireGeometry.new(0.5, 0.25), 0.1).is_empty(), "a tread whose shoulder drop exceeds the radius rule gives no surface")


## The mesh's surface points are the ones TireGeometry itself evaluates for a circle of the tread (point_at_offset), in the wheel
## frame: for several wheel poses and lateral offsets the contact function's local point is a point of the wireframe's surface.
func test_vertices_agree_with_tire_geometry_samples() -> void:
	var geometry = TireGeometry.new(0.12, 0.07)
	var radius: float = 0.31
	var poses: Array = [Quaternion(Vector3(0, 0, 1), 0.0), Quaternion(Vector3(0, 0, 1), 0.4), Quaternion(Vector3(0, 0, 1), -0.7),
		Quaternion(Vector3(0.3, 0.2, 1.0).normalized(), 0.5) * Quaternion(Vector3.RIGHT, 1.1)]
	var worst: float = 0.0
	var count: int = 0
	for q in poses:
		for s in [-0.06, -0.03, 0.0, 0.02, 0.06]:
			var sample: Dictionary = TireGeometry.point_at_offset(Vector3(0.2, 0.4, -0.1), q, geometry, radius, Vector3.ZERO, Vector3.UP, s)
			if not sample.valid:
				continue
			var local: Vector3 = sample.local_point
			var theta: float = atan2(local.z, local.y)
			var mine: Vector3 = GeometryWireframe.tire_surface_point(geometry, radius, s, theta)
			worst = maxf(worst, (mine - local).length())
			count += 1
	check(count == 20, "all poses give valid samples (%d)" % count)
	check(worst < TOL, "the wireframe's surface point equals TireGeometry's local point (worst %s)" % worst)
	# The model's own lowest point (global optimum over the tread) is on that surface, too.
	var lowest: Dictionary = TireGeometry.lowest_point(Vector3.ZERO, Quaternion(Vector3(0, 0, 1), 0.4), geometry, radius, Vector3.ZERO, Vector3.UP)
	var s_star: float = lowest.lateral_offset
	var theta_star: float = atan2(lowest.local_point.z, lowest.local_point.y)
	check_vec3_near(GeometryWireframe.tire_surface_point(geometry, radius, s_star, theta_star), lowest.local_point, TOL, "the lowest point (camber 0.4) is a surface point")


func test_front_and_rear_use_their_own_geometry() -> void:
	var assembly = make_assembly()
	var w: GeometryWireframe = make_wireframe(assembly)
	var extents: Array = []
	for side in ["front", "rear"]:
		var v: PackedVector3Array = w.tire_instance(side).mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX]
		var max_x: float = 0.0
		var max_r: float = 0.0
		for p in v:
			max_x = maxf(max_x, absf(p.x))
			max_r = maxf(max_r, sqrt(p.y * p.y + p.z * p.z))
		extents.append([max_x, max_r])
	check_near(extents[0][0], 0.06, 1.0e-6, "front half width")
	check_near(extents[0][1], 0.31, TOL, "front wheel radius")
	check_near(extents[1][0], 0.09, 1.0e-6, "rear half width")
	check_near(extents[1][1], 0.32, TOL, "rear wheel radius")
	free_wireframe(w)


func test_tire_surface_follows_wheel_pose_camber_and_spin() -> void:
	var assembly = make_assembly()
	var w: GeometryWireframe = make_wireframe(assembly)
	w.set_enabled(true)
	var camber: Quaternion = Quaternion(Vector3(0, 0, 1), 0.35)
	var steer: Quaternion = Quaternion(Vector3.UP, 0.2)
	var spin: Quaternion = Quaternion(Vector3.RIGHT, 2.3)
	var q: Quaternion = steer * camber * spin  # axle (local +X) tilted by the camber about the world Z axis, then steered; spun about the axle
	var p: Vector3 = Vector3(0.1, 0.35, -0.6)
	set_pose(assembly.front_wheel, p, q)
	w.update_wireframe(1.0)
	var instance: MeshInstance3D = w.tire_instance("front")
	check_vec3_near(instance.transform.origin, p, TOL, "the surface follows the wheel position")
	check_basis_near(instance.transform.basis, Basis(q), TOL, "and its orientation (steering, camber, spin)")
	# Every world vertex stays on the tread: its distance from the axle line equals the profile radius at its axle coordinate.
	var axle: Vector3 = q * Vector3.RIGHT
	var worst: float = 0.0
	for v in world_vertices(instance):
		var rel: Vector3 = v - p
		var s: float = rel.dot(axle)
		var radial: float = (rel - axle * s).length()
		worst = maxf(worst, absf(radial - (0.31 - 0.07 + sqrt(0.07 * 0.07 - s * s))))
	check(worst < 2.0e-5, "world vertices stay on the profile about the tilted axle (worst %s)" % worst)
	# Camber moves the lowest point toward the low end of the axle: compare the lowest vertex with the exact lowest point of the model.
	set_pose(assembly.front_wheel, Vector3.ZERO, camber)
	w.update_wireframe(1.0)
	var lowest: Dictionary = TireGeometry.lowest_point(Vector3.ZERO, camber, assembly.front_tire_geometry, 0.31, Vector3.ZERO, Vector3.UP)
	var min_y: float = INF
	for v in world_vertices(w.tire_instance("front")):
		min_y = minf(min_y, v.y)
	check(lowest.valid and min_y >= lowest.gap - 1.0e-6, "no vertex is below the exact lowest point (%s vs %s)" % [min_y, lowest.gap])
	# Sampling error of the mesh: angle step 7.5 deg gives at most R (1 - cos 3.75 deg) = 6.7e-4 m in radius; the offset step 0.01 m
	# changes the height by < rho * (0.005/rho)^2 / 2 ~ 1.8e-4 m. Sum < 1.5e-3 m.
	check(min_y - lowest.gap < 1.5e-3, "the lowest vertex is within the sampling error of the exact lowest point (%s)" % (min_y - lowest.gap))
	free_wireframe(w)


func test_no_per_frame_rebuild() -> void:
	var assembly = make_assembly()
	var w: GeometryWireframe = make_wireframe(assembly)
	w.set_enabled(true)
	var meshes: Array = [w.tire_instance("front").mesh, w.tire_instance("rear").mesh, w.box_instance("chassis").mesh]
	var children: int = w.get_child_count()
	for k in 200:
		w.update_wireframe(0.5)
	check(w.get_child_count() == children, "no node is created per frame")
	check(w.tire_instance("front").mesh == meshes[0] and w.tire_instance("rear").mesh == meshes[1] and w.box_instance("chassis").mesh == meshes[2], "meshes are reused")
	# A changed dimension rebuilds that tyre once (and only that one).
	assembly.front_tire_geometry.width = 0.10
	w.update_wireframe(0.5)
	check(w.tire_instance("front").mesh != meshes[0] and w.tire_instance("rear").mesh == meshes[1], "a configuration change rebuilds only the affected tyre")
	var v: PackedVector3Array = w.tire_instance("front").mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX]
	var max_x: float = 0.0
	for p in v:
		max_x = maxf(max_x, absf(p.x))
	check_near(max_x, 0.05, 1.0e-6, "the rebuilt surface has the new width")
	free_wireframe(w)


# --- No effect on the simulation ---

func snapshot(world) -> Array:
	var data: Array = []
	for body in world.bodies:
		data.append([body.position, body.orientation, body.linear_velocity, body.angular_velocity])
	return data


func test_simulation_is_identical_with_the_visualization_on_and_off() -> void:
	var world_a = SimWorld.new()
	var assembly_a = MotorcycleBuilder.build(make_config(false), world_a)  # no tyre geometry, no visualization
	var world_b = SimWorld.new()
	var assembly_b = MotorcycleBuilder.build(make_config(true), world_b)  # tyre geometry + wireframe enabled and updated every step
	var w: GeometryWireframe = make_wireframe(assembly_b)
	w.set_enabled(true)
	var world_c = SimWorld.new()
	var assembly_c = MotorcycleBuilder.build(make_config(true), world_c)  # tyre geometry, wireframe toggled on/off during the run
	var w_c: GeometryWireframe = make_wireframe(assembly_c)
	for k in 360:
		world_a.step()
		world_b.step()
		world_c.step()
		w.update_wireframe(float(k % 4) / 4.0)
		if k % 7 == 0:
			w_c.toggle()
		w_c.update_wireframe(1.0)
	var a: Array = snapshot(world_a)
	var b: Array = snapshot(world_b)
	var c: Array = snapshot(world_c)
	check(a.size() == b.size() and a.size() == c.size(), "same bodies")
	var identical: bool = true
	for k in a.size():
		identical = identical and a[k] == b[k] and a[k] == c[k]
	check(identical, "every body state is bit-identical with the wireframe off, on, and toggled during the run")
	check(world_a.step_count == world_b.step_count and world_a.simulation_time == world_b.simulation_time, "step count and time are identical")
	check(assembly_a.front_wheel_contact.normal_load() == assembly_b.front_wheel_contact.normal_load(), "contact results are identical")
	free_wireframe(w)
	free_wireframe(w_c)


# --- Runtime dimension refresh (M2.5.1.2.1) ---

## Local vertices of an instance's mesh (before the instance transform).
func mesh_vertices(instance: MeshInstance3D) -> PackedVector3Array:
	return instance.mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX]


## Per-axis [min, max] of a vertex list, as [Vector3 min, Vector3 max].
func extents(vertices: PackedVector3Array) -> Array:
	var low: Vector3 = Vector3(INF, INF, INF)
	var high: Vector3 = Vector3(-INF, -INF, -INF)
	for v in vertices:
		low = Vector3(minf(low.x, v.x), minf(low.y, v.y), minf(low.z, v.z))
		high = Vector3(maxf(high.x, v.x), maxf(high.y, v.y), maxf(high.z, v.z))
	return [low, high]


func check_signature(actual: Array, expected: Array, message: String) -> void:
	check(actual.size() == expected.size(), "%s: signature length %d (expected %d)" % [message, actual.size(), expected.size()])
	for k in mini(actual.size(), expected.size()):
		check_near(actual[k], expected[k], 1.0e-7, "%s: component %d" % [message, k])


## The actual runtime path: build, enable, update every frame; then change ONE tyre's width in the assembly's geometry object (the
## object the visualization reads each frame) and call update_wireframe() again. Expected numbers are written here: front tyre width
## 0.12 -> 0.10 m, crown radius 0.07 m, wheel radius 0.31 m; rear tyre 0.18 m / 0.10 m / 0.32 m.
func test_changing_one_tire_width_at_runtime_refreshes_only_that_wheel() -> void:
	var assembly = make_assembly()
	var w: GeometryWireframe = make_wireframe(assembly)
	w.set_enabled(true)
	var front_pose: Transform3D = Transform3D(Basis(Quaternion(Vector3(0, 0, 1), 0.2)), Vector3(0.1, 0.35, -0.6))
	var rear_pose: Transform3D = Transform3D(Basis(Quaternion(Vector3(0, 1, 0), 0.4)), Vector3(-0.2, 0.33, 0.8))
	set_pose(assembly.front_wheel, front_pose.origin, front_pose.basis.get_rotation_quaternion())
	set_pose(assembly.rear_wheel, rear_pose.origin, rear_pose.basis.get_rotation_quaternion())
	w.update_wireframe(1.0)

	# Initial state: sources, signatures, meshes.
	check_signature(w.tire_signature("front"), [0.12, 0.07, 0.31], "initial front tyre")
	check_signature(w.tire_signature("rear"), [0.18, 0.10, 0.32], "initial rear tyre")
	check_signature(w.box_signature("front_wheel"), [0.06, 0.31, 0.31, 0.0, 0.0, 0.0], "initial front box")
	check_signature(w.box_signature("rear_wheel"), [0.09, 0.32, 0.32, 0.0, 0.0, 0.0], "initial rear box")
	var front_tire_mesh = w.tire_instance("front").mesh
	var rear_tire_mesh = w.tire_instance("rear").mesh
	var front_box_mesh = w.box_instance("front_wheel").mesh
	var rear_box_mesh = w.box_instance("rear_wheel").mesh
	var other_meshes: Dictionary = {}
	for name in ["chassis", "steering", "fork_slider", "swingarm"]:
		other_meshes[name] = w.box_instance(name).mesh
	var rear_tire_vertices: PackedVector3Array = mesh_vertices(w.tire_instance("rear")).duplicate()
	var rear_box_vertices: PackedVector3Array = mesh_vertices(w.box_instance("rear_wheel")).duplicate()
	var rear_tire_signature: Array = w.tire_signature("rear")
	var rear_box_signature: Array = w.box_signature("rear_wheel")
	var front_box_x: Array = extents(front_box_mesh.surface_get_arrays(0)[Mesh.ARRAY_VERTEX])
	check_near(front_box_x[1].x, 0.06, 1.0e-6, "the front box starts 0.12 m wide")

	# Repeated updates with nothing changed rebuild nothing (and keep the transforms current).
	for k in 50:
		w.update_wireframe(1.0)
	check(w.tire_instance("front").mesh == front_tire_mesh and w.box_instance("front_wheel").mesh == front_box_mesh, "no rebuild without a change (front)")
	check(w.tire_instance("rear").mesh == rear_tire_mesh and w.box_instance("rear_wheel").mesh == rear_box_mesh, "no rebuild without a change (rear)")
	# A change below the tolerance (float32-sized noise) is not a change.
	assembly.front_tire_geometry.width = 0.12 + 1.0e-9
	w.update_wireframe(1.0)
	check(w.tire_instance("front").mesh == front_tire_mesh and w.box_instance("front_wheel").mesh == front_box_mesh, "a 1e-9 m change does not rebuild")

	var front_tire_transform: Transform3D = w.tire_instance("front").transform
	var front_box_transform: Transform3D = w.box_instance("front_wheel").transform
	var rear_tire_transform: Transform3D = w.tire_instance("rear").transform
	var rear_box_transform: Transform3D = w.box_instance("rear_wheel").transform

	# The change: ONLY the front tyre width, to another valid value; crown radius and the rear tyre untouched.
	assembly.front_tire_geometry.width = 0.10
	w.update_wireframe(1.0)

	# Front tyre surface: rebuilt, spans the new width, agrees with the profile (closed form written here), signature updated.
	var front_tire_instance: MeshInstance3D = w.tire_instance("front")
	check(front_tire_instance.mesh != front_tire_mesh, "the front tyre surface was rebuilt")
	var front_vertices: PackedVector3Array = mesh_vertices(front_tire_instance)
	var front_extent: Array = extents(front_vertices)
	check_near(front_extent[0].x, -0.05, 1.0e-6, "front surface reaches the new left shoulder")
	check_near(front_extent[1].x, 0.05, 1.0e-6, "and the new right shoulder")
	var worst: float = 0.0
	for p in front_vertices:
		worst = maxf(worst, absf(sqrt(p.y * p.y + p.z * p.z) - (0.31 - 0.07 + sqrt(0.07 * 0.07 - p.x * p.x))))
	check(worst < TOL, "front vertices lie on the configured profile (worst %s)" % worst)
	check_signature(w.tire_signature("front"), [0.10, 0.07, 0.31], "front tyre after the change")

	# Front bounding box: rebuilt with the new local dimensions (half extents 0.05 x 0.31 x 0.31) and a new signature.
	var front_box_instance: MeshInstance3D = w.box_instance("front_wheel")
	check(front_box_instance.mesh != front_box_mesh, "the front bounding box was rebuilt")
	var box_extent: Array = extents(mesh_vertices(front_box_instance))
	check_vec3_near(box_extent[0], Vector3(-0.05, -0.31, -0.31), 1.0e-6, "front box minimum corner (local)")
	check_vec3_near(box_extent[1], Vector3(0.05, 0.31, 0.31), 1.0e-6, "front box maximum corner (local)")
	check(mesh_vertices(front_box_instance).size() == 24, "still 12 edges")
	check_signature(w.box_signature("front_wheel"), [0.05, 0.31, 0.31, 0.0, 0.0, 0.0], "front box after the change")

	# Rear tyre and rear box: same mesh objects, same vertices, same signatures.
	check(w.tire_instance("rear").mesh == rear_tire_mesh, "the rear tyre surface was not rebuilt")
	check(w.box_instance("rear_wheel").mesh == rear_box_mesh, "the rear bounding box was not rebuilt")
	check(mesh_vertices(w.tire_instance("rear")) == rear_tire_vertices, "rear tyre vertices unchanged")
	check(mesh_vertices(w.box_instance("rear_wheel")) == rear_box_vertices, "rear box vertices unchanged")
	check(w.tire_signature("rear") == rear_tire_signature and w.box_signature("rear_wheel") == rear_box_signature, "rear signatures unchanged")
	check_vec3_near(extents(rear_box_vertices)[1], Vector3(0.09, 0.32, 0.32), 1.0e-6, "rear box is still 0.18 m wide, radius 0.32")
	for name in other_meshes:
		check(w.box_instance(name).mesh == other_meshes[name], "the %s box was not rebuilt" % name)

	# Transforms did not depend on the rebuild: the poses were not touched, and every transform is the published pose.
	check(w.tire_instance("front").transform == front_tire_transform and w.box_instance("front_wheel").transform == front_box_transform, "front transforms unchanged by the dimension change")
	check(w.tire_instance("rear").transform == rear_tire_transform and w.box_instance("rear_wheel").transform == rear_box_transform, "rear transforms unchanged")
	check_vec3_near(w.box_instance("front_wheel").transform.origin, front_pose.origin, TOL, "the rebuilt box sits at the wheel position")
	check_basis_near(w.box_instance("front_wheel").transform.basis, front_pose.basis, TOL, "and has the wheel orientation")

	# And the other way round: a pose change moves the items without any rebuild.
	var moved: Transform3D = Transform3D(Basis(Quaternion(Vector3(1, 0, 0), 1.0)), Vector3(0.5, 0.4, -0.9))
	set_pose(assembly.front_wheel, moved.origin, moved.basis.get_rotation_quaternion())
	var rebuilt_tire_mesh = w.tire_instance("front").mesh
	var rebuilt_box_mesh = w.box_instance("front_wheel").mesh
	w.update_wireframe(1.0)
	check(w.tire_instance("front").mesh == rebuilt_tire_mesh and w.box_instance("front_wheel").mesh == rebuilt_box_mesh, "a pose change rebuilds no geometry")
	check_vec3_near(w.box_instance("front_wheel").transform.origin, moved.origin, TOL, "the box follows the new position")
	check_basis_near(w.tire_instance("front").transform.basis, moved.basis, TOL, "and the tyre the new orientation")
	check(w.tire_instance("rear").transform == rear_tire_transform, "the rear wheel is not moved")
	free_wireframe(w)


## A body-spec size is the other kind of source dimension: changing one box rebuilds that box only; an invalid size skips it with a
## diagnostic (never a crash) and a later valid size brings it back.
func test_changing_a_body_size_at_runtime_rebuilds_only_that_box() -> void:
	var assembly = make_assembly()
	var w: GeometryWireframe = make_wireframe(assembly)
	w.set_enabled(true)
	w.update_wireframe(1.0)
	var meshes: Dictionary = {}
	for name in BODY_NAMES:
		meshes[name] = w.box_instance(name).mesh
	assembly.config.chassis.visual_size = Vector3(0.40, 0.60, 1.20)
	w.update_wireframe(1.0)
	check(w.box_instance("chassis").mesh != meshes["chassis"], "the chassis box was rebuilt")
	var chassis_extent: Array = extents(mesh_vertices(w.box_instance("chassis")))
	check_vec3_near(chassis_extent[1], Vector3(0.20, 0.30, 0.60), 1.0e-6, "chassis half extents 0.20 x 0.30 x 0.60")
	check_signature(w.box_signature("chassis"), [0.20, 0.30, 0.60, 0.0, 0.0, 0.0], "chassis signature")
	for name in BODY_NAMES:
		if name != "chassis":
			check(w.box_instance(name).mesh == meshes[name], "the %s box was not rebuilt" % name)
	for k in 10:
		w.update_wireframe(1.0)
	var rebuilt = w.box_instance("chassis").mesh
	w.update_wireframe(1.0)
	check(w.box_instance("chassis").mesh == rebuilt, "no further rebuilds")

	assembly.config.chassis.visual_size = Vector3(0.40, 0.0, 1.20)
	w.update_wireframe(1.0)
	check(w.box_instance("chassis") == null and w.box_count() == 5, "an invalid size skips only that box")
	check(not w.diagnostics.is_empty(), "with a diagnostic")
	check(w.box_signature("chassis") == [], "and no signature")
	w.update_wireframe(1.0)  # must not raise or rebuild anything else
	check(w.box_instance("steering").mesh == meshes["steering"], "other boxes unaffected")
	assembly.config.chassis.visual_size = Vector3(0.30, 0.55, 1.00)
	w.update_wireframe(1.0)
	check(w.box_instance("chassis") != null and w.box_count() == 6, "a valid size brings the box back")
	check_vec3_near(extents(mesh_vertices(w.box_instance("chassis")))[1], Vector3(0.15, 0.275, 0.5), 1.0e-6, "with the restored dimensions")
	free_wireframe(w)
