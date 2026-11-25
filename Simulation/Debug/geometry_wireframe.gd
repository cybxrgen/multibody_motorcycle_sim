extends Node3D
## Toggleable wireframe of the PHYSICAL collision boxes (M2.5.2; replaces the M2.5.1.2 bounding-box / tyre-surface view). It draws exactly
## the oriented boxes that generate the box-ground contacts: the MotorcycleCollisionBox objects stored in `assembly.collision_boxes` (the very
## same objects BoxGroundContact reads), with their body-local centre, orientation and dimensions, following the bodies' PUBLISHED
## (interpolated) transforms. PRESENTATION ONLY: it never writes physical state, adds no force, constraint or provider, and nothing in the
## simulation depends on it being enabled.
##
## There is no other source of boxes: no box is synthesized from `visual_size`, no wheel box is drawn (the wheels have no physical collision
## box), and a body without configured collision geometry shows no box. Separately, the finite-width TYRE SURFACE of each wheel that has a
## TireGeometry is drawn (green front, violet rear) as in M2.5.1.2: tyre geometry for inspection, NOT a collision shape.
##
## ---- Refresh ----
## Every item keeps the signature of the box geometry it was built from (MotorcycleCollisionBox.signature(): size, centre, orientation).
## update_wireframe() compares the box's current signature (same_signature(): every component within DIMENSION_TOLERANCE, NaN never equal)
## and rebuilds ONLY an item whose geometry changed; the world transform of every item is set each frame from the published state.
## A removed box frees its item; an invalid box (or a missing body) skips the item with an entry in `diagnostics` (one warning when
## `log_diagnostics`), and it comes back when valid again. Nothing here raises.

const SimBodyState = preload("res://Simulation/Core/sim_body_state.gd")

## Two signatures are equal when every component differs by at most this (0.1 micrometre, above float32 rounding of a sub-metre value).
## Resolution of the tyre surface (bounded: LINE_SEGMENTS_PER_TIRE segments per tyre).
const CIRCUMFERENTIAL_SEGMENTS: int = 48
const PROFILE_SEGMENTS: int = 12
const MERIDIANS: int = 16
const LINE_SEGMENTS_PER_TIRE: int = (PROFILE_SEGMENTS + 1) * CIRCUMFERENTIAL_SEGMENTS + MERIDIANS * PROFILE_SEGMENTS
const FRONT_TIRE_COLOR: Color = Color(0.35, 1.0, 0.55, 0.9)
const REAR_TIRE_COLOR: Color = Color(0.7, 0.55, 1.0, 0.9)

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
## One entry per tyre that is drawn: {name, body, side, instance, signature}. Presentation of the tyre GEOMETRY only (not a collision shape).
var _tires: Array = []


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
	for side in ["front", "rear"]:
		_tires.append({"name": side + "_tire", "body": assembly.front_wheel if side == "front" else assembly.rear_wheel, "side": side,
			"instance": null, "signature": null})
		_refresh_tire(_tires[_tires.size() - 1])
	visible = enabled


## Sets the pose of every item from the published states (interpolated by alpha). Does nothing while disabled.
func update_wireframe(alpha: float) -> void:
	if not enabled or _assembly == null:
		return
	for item in _boxes:
		_refresh_box(item)
		_place(item, alpha)
	for item in _tires:
		_refresh_tire(item)
		_place(item, alpha)


## Boxes that currently have a mesh.
func box_count() -> int:
	var count: int = 0
	for item in _boxes:
		if item.instance != null:
			count += 1
	return count


## Tyres that currently have a surface mesh.
func tire_count() -> int:
	var count: int = 0
	for item in _tires:
		if item.instance != null:
			count += 1
	return count


## The instance of the front or rear tyre surface ("front" / "rear"), or null.
func tire_instance(side: String):
	for item in _tires:
		if item.side == side:
			return item.instance
	return null


func tire_signature(side: String) -> Array:
	for item in _tires:
		if item.side == side:
			return (item.signature as Array).duplicate() if item.signature != null else []
	return []


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


## (Re)builds the tyre's surface when its geometry reference, dimensions or wheel radius changed (or it was never built).
func _refresh_tire(item: Dictionary) -> void:
	var geometry = _assembly.front_tire_geometry if item.side == "front" else _assembly.rear_tire_geometry
	var radius: float = _assembly.config.front_wheel_radius if item.side == "front" else _assembly.config.rear_wheel_radius
	var signature: Array = [] if geometry == null else [geometry.width, geometry.crown_radius, radius]
	if item.signature != null and same_signature(signature, item.signature) and (item.instance != null or geometry == null):
		return
	item.signature = signature
	if item.instance != null:
		if is_instance_valid(item.instance):
			remove_child(item.instance)
			item.instance.free()
		item.instance = null
	if item.body == null:
		_note("geometry wireframe: %s has no wheel body; skipped" % item.name)
		return
	if geometry == null:
		_note("geometry wireframe: %s has no tire geometry configured; skipped" % item.name)
		return
	var vertices: PackedVector3Array = tire_line_vertices(geometry, radius)
	if vertices.is_empty():
		_note("geometry wireframe: %s geometry is invalid for wheel radius %s; skipped" % [item.name, radius])
		return
	item.instance = _make_instance(vertices, FRONT_TIRE_COLOR if item.side == "front" else REAR_TIRE_COLOR)


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
	for item in _boxes + _tires:
		if item.instance != null and is_instance_valid(item.instance):
			remove_child(item.instance)
			item.instance.free()
	_boxes.clear()
	_tires.clear()
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


## A surface point of the tyre in the WHEEL frame at lateral offset s and angle theta (radians, about the axle, from local +Y toward
## +Z): (s, r cos theta, r sin theta) with r = geometry.profile_radius(wheel_radius, s). NaN-vector when the profile is invalid.
static func tire_surface_point(geometry, wheel_radius: float, lateral_offset: float, theta: float) -> Vector3:
	var radius: float = geometry.profile_radius(wheel_radius, lateral_offset)
	if is_nan(radius):
		return Vector3(NAN, NAN, NAN)
	return Vector3(lateral_offset, radius * cos(theta), radius * sin(theta))


## The line-list vertices (pairs) of the tyre surface in the wheel frame: PROFILE_SEGMENTS + 1 rings and MERIDIANS meridians. Empty
## when the geometry is null or invalid for `wheel_radius`.
static func tire_line_vertices(geometry, wheel_radius: float) -> PackedVector3Array:
	var vertices: PackedVector3Array = PackedVector3Array()
	if geometry == null or not geometry.is_valid(wheel_radius):
		return vertices
	var half: float = 0.5 * geometry.width
	var stations: Array = []  # [rings][N points]
	for j in PROFILE_SEGMENTS + 1:
		var s: float = clampf(-half + geometry.width * float(j) / float(PROFILE_SEGMENTS), -half, half)
		var ring: Array = []
		for k in CIRCUMFERENTIAL_SEGMENTS:
			var point: Vector3 = tire_surface_point(geometry, wheel_radius, s, TAU * float(k) / float(CIRCUMFERENTIAL_SEGMENTS))
			if not point.is_finite():
				return PackedVector3Array()
			ring.append(point)
		stations.append(ring)
	for ring in stations:
		for k in CIRCUMFERENTIAL_SEGMENTS:
			vertices.append(ring[k])
			vertices.append(ring[(k + 1) % CIRCUMFERENTIAL_SEGMENTS])
	var stride: int = CIRCUMFERENTIAL_SEGMENTS / MERIDIANS
	for m in MERIDIANS:
		for j in PROFILE_SEGMENTS:
			vertices.append(stations[j][m * stride])
			vertices.append(stations[j + 1][m * stride])
	return vertices
