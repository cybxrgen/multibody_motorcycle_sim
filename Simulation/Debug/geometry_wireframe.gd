extends Node3D
## Toggleable wireframe of the PHYSICAL collision boxes (M2.5.2; replaces the M2.5.1.2 bounding-box / tyre-surface view). It draws exactly
## the oriented boxes that generate the box-ground contacts: the MotorcycleCollisionBox objects stored in `assembly.collision_boxes` (the very
## same objects BoxGroundContact reads), with their body-local centre, orientation and dimensions, following the bodies' PUBLISHED
## (interpolated) transforms. PRESENTATION ONLY: it never writes physical state, adds no force, constraint or provider, and nothing in the
## simulation depends on it being enabled.
##
## There is no other source of boxes: no box is synthesized from `visual_size`, no wheel box and no tyre surface is drawn (the wheels have
## no physical collision box; their tyre contact is a different mechanism), and a body without configured collision geometry shows nothing.
##
## ---- Refresh ----
## Every item keeps the signature of the box geometry it was built from (MotorcycleCollisionBox.signature(): size, centre, orientation).
## update_wireframe() compares the box's current signature (same_signature(): every component within DIMENSION_TOLERANCE, NaN never equal)
## and rebuilds ONLY an item whose geometry changed; the world transform of every item is set each frame from the published state.
## A removed box frees its item; an invalid box (or a missing body) skips the item with an entry in `diagnostics` (one warning when
## `log_diagnostics`), and it comes back when valid again. Nothing here raises.

const SimBodyState = preload("res://Simulation/Core/sim_body_state.gd")

## Two signatures are equal when every component differs by at most this (0.1 micrometre, above float32 rounding of a sub-metre value).
const DIMENSION_TOLERANCE: float = 1.0e-7
## The four possible owners in the order their boxes are built.
const BODY_NAMES: Array = ["chassis", "steering", "fork_slider", "swingarm"]
const BOX_COLOR: Color = Color(0.95, 0.95, 0.95, 0.9)

## Master switch (off by default).
var enabled: bool = false
## Messages about skipped items (deduplicated, in the order they occurred).
var diagnostics: PackedStringArray = PackedStringArray()
## Also print each new diagnostic with push_warning (tests switch this off).
var log_diagnostics: bool = true

var _assembly = null
## One entry per configured box: {name, box, body, instance, signature}. An instance is null while the item is skipped.
var _boxes: Array = []


func _init() -> void:
	visible = false


func toggle() -> bool:
	set_enabled(not enabled)
	return enabled


func set_enabled(value: bool) -> void:
	enabled = value
	visible = value


## Builds the box meshes of `assembly` (one per configured collision box); replaces any earlier content. A null assembly leaves the
## wireframe empty with a diagnostic.
func setup(assembly) -> void:
	_clear()
	_assembly = assembly
	if assembly == null:
		_note("geometry wireframe: no assembly")
		return
	for body_name in BODY_NAMES:
		if assembly.collision_boxes.has(body_name):
			_boxes.append({"name": body_name, "box": null, "body": null, "instance": null, "signature": null})
			_refresh_box(_boxes[_boxes.size() - 1])
	visible = enabled


## Sets the pose of every item from the published states (interpolated by alpha). Does nothing while disabled.
func update_wireframe(alpha: float) -> void:
	if not enabled or _assembly == null:
		return
	for item in _boxes:
		_refresh_box(item)
		_place(item, alpha)


## Boxes that currently have a mesh.
func box_count() -> int:
	var count: int = 0
	for item in _boxes:
		if item.instance != null:
			count += 1
	return count


## The instance (MeshInstance3D) of a box, by body name, or null.
func box_instance(body_name: String):
	for item in _boxes:
		if item.name == body_name:
			return item.instance
	return null


## The stored geometry signature of a box (by body name): [] when unknown or skipped. A copy.
func box_signature(body_name: String) -> Array:
	for item in _boxes:
		if item.name == body_name:
			return (item.signature as Array).duplicate() if item.signature != null else []
	return []


func _place(item: Dictionary, alpha: float) -> void:
	var instance = item.instance
	var body = item.body
	if instance == null:
		return
	if body == null or not is_instance_valid(instance):
		instance.visible = false
		return
	instance.visible = true
	instance.transform = SimBodyState.interpolate_transform(body.previous_state, body.published_state, alpha)


## (Re)builds one box when its physical geometry changed (or it was never built); the box and body references are re-read every call.
func _refresh_box(item: Dictionary) -> void:
	var box = _assembly.collision_boxes.get(item.name, null)
	item.box = box
	item.body = _assembly.box_body(item.name) if box != null else null
	var valid: bool = box != null and item.body != null and box.is_valid()
	var signature: Array = box.signature() if valid else []
	if item.signature != null and same_signature(signature, item.signature) and (item.instance != null or not valid):
		return
	item.signature = signature
	if item.instance != null:
		if is_instance_valid(item.instance):
			remove_child(item.instance)
			item.instance.free()
		item.instance = null
	if not valid:
		if box != null:
			_note("geometry wireframe: collision box of '%s' is invalid or its body is unavailable; skipped" % item.name)
		return
	item.instance = _make_instance(box.edge_vertices(), BOX_COLOR)


func _make_instance(vertices: PackedVector3Array, color: Color) -> MeshInstance3D:
	var instance: MeshInstance3D = MeshInstance3D.new()
	if not vertices.is_empty():
		var arrays: Array = []
		arrays.resize(Mesh.ARRAY_MAX)
		arrays[Mesh.ARRAY_VERTEX] = vertices
		var mesh: ArrayMesh = ArrayMesh.new()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_LINES, arrays)
		instance.mesh = mesh
		var material: StandardMaterial3D = StandardMaterial3D.new()
		material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		material.albedo_color = color
		material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		material.no_depth_test = true
		instance.material_override = material
	instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(instance)
	return instance


func _clear() -> void:
	for item in _boxes:
		if item.instance != null and is_instance_valid(item.instance):
			remove_child(item.instance)
			item.instance.free()
	_boxes.clear()
	diagnostics = PackedStringArray()
	_assembly = null


func _note(message: String) -> void:
	if message in diagnostics:
		return
	diagnostics.append(message)
	if log_diagnostics:
		push_warning(message)


## True when two signatures have the same length and every component differs by at most DIMENSION_TOLERANCE (NaN never matches).
static func same_signature(a: Array, b: Array) -> bool:
	if a.size() != b.size():
		return false
	for k in a.size():
		if not (absf(a[k] - b[k]) <= DIMENSION_TOLERANCE):
			return false
	return true
