extends Node3D
## Toggleable debug view of the distributed contact patches of the assembled motorcycle (M2.5.3 Stage D). PRESENTATION ONLY: it reads the wheels'
## published states and the patches' reporting functions (sample_report()) and never writes physical state, adds no force, constraint or provider,
## and nothing in the simulation depends on it being enabled (off by default, like GeometryWireframe; the scene toggles it with a key).
## It is separate from GeometryWireframe on purpose: that class draws the tyre SURFACE wireframe and the collision boxes and is left untouched, so the
## patch view neither replaces nor duplicates it.
##
## Per wheel with a patch (`assembly.front_tire_patch` / `rear_tire_patch`; a wheel with a one-point contact shows nothing) ONE line mesh in world
## coordinates is rebuilt from the CURRENT state every update (nothing is cached between frames):
##   * every CANDIDATE tyre-surface sample (TirePatchSampler on the wheel pose, the patch's own geometry, grid, plane): a short tick along its
##     outward surface normal in the dim colour of its wheel, i.e. an INACTIVE candidate;
##   * every ACTIVE physical contact sample (the indices of patch.sample_report(): physically compressed, g < 0 at the last prepared step): a longer tick
##     in the bright colour of its wheel, whose length grows with the sample's relative normal support (its load over the largest load of that wheel,
##     the "load-distribution indicator"); a bottomed sample is drawn in white.
## Front (cyan) and rear (orange) are distinguished by colour and are separate instances. The wheel pose is the INTERPOLATED published pose (the same
## pose the wheel proxy and the tyre wireframe use); the classification and the loads are those of the patch's last solved step.
## A wheel without a patch, an invalid sampling or a missing body gives an empty view with an entry in `diagnostics`. Nothing here raises.

const SimBodyState = preload("res://Simulation/Core/sim_body_state.gd")
const TirePatchSampler = preload("res://Simulation/Motorcycle/tire_patch_sampler.gd")

const FRONT_COLOR_CANDIDATE: Color = Color(0.15, 0.45, 0.55, 0.55)
const FRONT_COLOR_ACTIVE: Color = Color(0.2, 1.0, 1.0, 1.0)
const REAR_COLOR_CANDIDATE: Color = Color(0.55, 0.35, 0.12, 0.55)
const REAR_COLOR_ACTIVE: Color = Color(1.0, 0.6, 0.15, 1.0)
const BOTTOMED_COLOR: Color = Color(1.0, 1.0, 1.0, 1.0)
## Tick lengths (m): candidates, and active samples from the smallest (no load yet) to the largest (the most loaded sample of the wheel).
const CANDIDATE_TICK: float = 0.012
const ACTIVE_TICK_MIN: float = 0.02
const ACTIVE_TICK_MAX: float = 0.12

## Master switch (off by default).
var enabled: bool = false
## Why a view is empty (one entry per distinct message).
var diagnostics: PackedStringArray = PackedStringArray()
var log_diagnostics: bool = true

var _assembly = null
var _items: Array = []


func _init() -> void:
	visible = false


func toggle() -> bool:
	set_enabled(not enabled)
	return enabled


func set_enabled(value: bool) -> void:
	enabled = value
	visible = value


## Builds the (empty) per-wheel items of `assembly`; replaces any earlier content. A null assembly leaves the view empty with a diagnostic.
func setup(assembly) -> void:
	_clear()
	_assembly = assembly
	if assembly == null:
		_note("tire patch debug: no assembly")
		return
	for side in ["front", "rear"]:
		_items.append({"side": side, "instance": null, "candidates": PackedVector3Array(), "active": PackedVector3Array(), "active_loads": PackedFloat32Array(),
			"active_bottomed": PackedByteArray(), "max_load": 0.0, "tick_lengths": PackedFloat32Array()})
	visible = enabled


## Rebuilds the view of every patch wheel from the current state (alpha = SimWorld.interpolation_alpha()). Does nothing while disabled.
func update_patch_view(alpha: float) -> void:
	if not enabled or _assembly == null:
		return
	for item in _items:
		_refresh(item, alpha)


## The MeshInstance3D of "front" / "rear" (null when that wheel has no patch or nothing was drawn yet).
func instance(side: String):
	for item in _items:
		if item.side == side:
			return item.instance
	return null


## World positions of the INACTIVE candidate samples drawn for the side at the last update (a copy).
func candidate_positions(side: String) -> PackedVector3Array:
	for item in _items:
		if item.side == side:
			return item.candidates.duplicate()
	return PackedVector3Array()


## World positions of the ACTIVE physical contact samples drawn for the side (a copy).
func active_positions(side: String) -> PackedVector3Array:
	for item in _items:
		if item.side == side:
			return item.active.duplicate()
	return PackedVector3Array()


## Relative normal support (0..1, load over the wheel's largest sample load; 0 for all while no load is reported) of the active samples, same order.
func active_relative_support(side: String) -> PackedFloat32Array:
	var result: PackedFloat32Array = PackedFloat32Array()
	for item in _items:
		if item.side == side:
			for load in item.active_loads:
				result.append(load / item.max_load if item.max_load > 0.0 else 0.0)
	return result


## Tick length (m) of every active sample of the side, same order (the drawn load-distribution indicator).
func active_tick_lengths(side: String) -> PackedFloat32Array:
	for item in _items:
		if item.side == side:
			return item.tick_lengths.duplicate()
	return PackedFloat32Array()


## Number of line segments in the side's mesh (one per candidate and one per active sample).
func segment_count(side: String) -> int:
	for item in _items:
		if item.side == side:
			return item.candidates.size() + item.active.size()
	return 0


func _patch_of(side: String):
	return _assembly.front_tire_patch if side == "front" else _assembly.rear_tire_patch


func _refresh(item: Dictionary, alpha: float) -> void:
	var side: String = item.side
	var patch = _patch_of(side)
	var body = _assembly.front_wheel if side == "front" else _assembly.rear_wheel
	_drop(item)
	if patch == null:
		return
	if body == null:
		_note("tire patch debug: %s has no wheel body; skipped" % side)
		return
	var pose: Transform3D = SimBodyState.interpolate_transform(body.previous_state, body.published_state, alpha)
	var data: Dictionary = TirePatchSampler.sample(pose.origin, pose.basis.get_rotation_quaternion(), patch.geometry, patch.wheel_radius,
			patch.plane_point, patch.plane_normal, patch.lateral_count, patch.circumferential_count)
	if not data.valid:
		_note("tire patch debug: %s sampling is invalid (%s); skipped" % [side, data.reason])
		return
	# The state of the patch: which samples are physically active, with which load (a copy; read-only).
	var state: Dictionary = {}
	var max_load: float = 0.0
	for entry in patch.sample_report():
		state[entry.index] = entry
		if is_finite(entry.load):
			max_load = maxf(max_load, entry.load)
	var candidate_color: Color = FRONT_COLOR_CANDIDATE if side == "front" else REAR_COLOR_CANDIDATE
	var active_color: Color = FRONT_COLOR_ACTIVE if side == "front" else REAR_COLOR_ACTIVE
	var vertices: PackedVector3Array = PackedVector3Array()
	var colors: PackedColorArray = PackedColorArray()
	for sample in data.samples:
		var index: int = sample.lateral_index * patch.circumferential_count + sample.angular_index
		var length: float = CANDIDATE_TICK
		var color: Color = candidate_color
		if state.has(index):
			var entry: Dictionary = state[index]
			var relative: float = clampf(entry.load / max_load, 0.0, 1.0) if max_load > 0.0 and is_finite(entry.load) else 0.0
			length = ACTIVE_TICK_MIN + (ACTIVE_TICK_MAX - ACTIVE_TICK_MIN) * relative
			color = BOTTOMED_COLOR if entry.bottomed else active_color
			item.active.append(sample.position)
			item.active_loads.append(entry.load if is_finite(entry.load) else 0.0)
			item.active_bottomed.append(1 if entry.bottomed else 0)
			item.tick_lengths.append(length)
		else:
			item.candidates.append(sample.position)
		vertices.append(sample.position)
		vertices.append(sample.position + sample.normal * length)
		colors.append(color)
		colors.append(color)
	item.max_load = max_load
	if vertices.is_empty():
		return
	var arrays: Array = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_COLOR] = colors
	var mesh: ArrayMesh = ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, arrays)
	var mesh_instance: MeshInstance3D = MeshInstance3D.new()
	mesh_instance.mesh = mesh
	var material: StandardMaterial3D = StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.vertex_color_use_as_albedo = true
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.no_depth_test = true
	mesh_instance.material_override = material
	mesh_instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	mesh_instance.name = "%s_patch" % side
	add_child(mesh_instance)
	item.instance = mesh_instance


## Frees the previous mesh and empties the item (the view is rebuilt from the state every update).
func _drop(item: Dictionary) -> void:
	if item.instance != null and is_instance_valid(item.instance):
		remove_child(item.instance)
		item.instance.free()
	item.instance = null
	item.candidates = PackedVector3Array()
	item.active = PackedVector3Array()
	item.active_loads = PackedFloat32Array()
	item.active_bottomed = PackedByteArray()
	item.tick_lengths = PackedFloat32Array()
	item.max_load = 0.0


func _clear() -> void:
	for item in _items:
		_drop(item)
	_items.clear()
	diagnostics = PackedStringArray()
	_assembly = null


func _note(message: String) -> void:
	if message in diagnostics:
		return
	diagnostics.append(message)
	if log_diagnostics:
		push_warning(message)
