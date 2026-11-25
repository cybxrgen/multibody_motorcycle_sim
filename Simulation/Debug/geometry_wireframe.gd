extends Node3D
## Toggleable geometry wireframe (M2.5.1.2): oriented bounding boxes of the six dynamic motorcycle bodies and the finite-width 3D tyre
## surfaces of the two wheels. PRESENTATION ONLY: reads the assembly's configuration and the PUBLISHED body states (interpolated), never
## writes physical state, adds no force, constraint or provider, and nothing in the simulation depends on it being enabled.
##
## ---- Bounding boxes (oriented, body-local) ----
## Each box is centred on the body's COM frame and axis-aligned in the BODY frame, so it rotates with the body (never a world-aligned
## box). Sources, in order of authority:
##   * wheels: half extents (w/2, R, R) with R the wheel radius of the contact geometry (MotorcycleConfig.*_wheel_radius) and w the
##     width of the tyre's TireGeometry when the wheel has one (it is the tread's lateral extent), else the wheel width of the config.
##     R is the exact outer radius of the tyre surface (the profile radius is R at the centreline and smaller toward the shoulders).
##   * the four box-shaped bodies: MotorcycleBodySpec.visual_size, the only per-body dimension the configuration carries. There is no
##     physical bounding volume in the repository; for the prototype it is the same Vector3 that produced the body's inertia tensor
##     (box_inertia), which Tests/test_geometry_wireframe.gd checks. It is NOT the size of whatever mesh a scene happens to draw.
##
## ---- Tyre surface ----
## Built ONCE per tyre (and rebuilt only when that tyre's width, crown radius or wheel radius changes) as an ArrayMesh of line segments
## in the wheel frame (origin = wheel COM, axle = local +X): a surface point at lateral offset s and angle theta is
##   (s, r(s) cos theta, r(s) sin theta),   r(s) = TireGeometry.profile_radius(R, s),
## i.e. the radius comes from the geometry model itself (no second profile equation here). Rings at PROFILE_SEGMENTS + 1 offsets from
## shoulder to shoulder (the crown is among them when PROFILE_SEGMENTS is even) with CIRCUMFERENTIAL_SEGMENTS segments each, plus
## MERIDIANS lines across the profile. Per frame only the node's transform is set (the wheel pose), nothing is rebuilt or allocated.
##
## ---- Refresh when dimensions change ----
## Every box and every tyre surface keeps the SIGNATURE of the source dimensions its current geometry was built from: a box
## [half extents x y z, centre x y z] (body frame), a tyre [width, crown radius, wheel radius]. update_wireframe() compares the current
## source dimensions with the stored signature (same_signature(): every component within DIMENSION_TOLERANCE, so float32 rounding of a
## recomputed size is not a change but any real change is) and rebuilds ONLY the item whose signature changed. Pose is not part of any
## signature: the world transform of every item is set each frame from the published state, independently of geometry rebuilds.
##
## Safe by construction: a missing body, a missing or invalid geometry or a non-finite dimension skips that item with an entry in
## `diagnostics` (and one warning in the log when `log_diagnostics` is true); it never raises.

const SimBodyState = preload("res://Simulation/Core/sim_body_state.gd")

## Resolution of the tyre surface (bounded: LINE_SEGMENTS_PER_TIRE segments per tyre).
const CIRCUMFERENTIAL_SEGMENTS: int = 48
const PROFILE_SEGMENTS: int = 12
const MERIDIANS: int = 16
const LINE_SEGMENTS_PER_TIRE: int = (PROFILE_SEGMENTS + 1) * CIRCUMFERENTIAL_SEGMENTS + MERIDIANS * PROFILE_SEGMENTS

## Two signatures are equal when every component differs by at most this (m): 0.1 micrometre, above the float32 rounding of a
## sub-metre dimension (about 6e-8) and far below any intended dimension change.
const DIMENSION_TOLERANCE: float = 1.0e-7
## The six bodies in the order their boxes are built.
const BODY_NAMES: Array = ["chassis", "steering", "fork_slider", "swingarm", "front_wheel", "rear_wheel"]

const BOX_COLOR: Color = Color(0.95, 0.95, 0.95, 0.9)
const FRONT_TIRE_COLOR: Color = Color(0.35, 1.0, 0.55, 0.9)
const REAR_TIRE_COLOR: Color = Color(0.7, 0.55, 1.0, 0.9)

## Master switch of both layers (off by default).
var enabled: bool = false
## Messages about skipped items (deduplicated, in the order they occurred).
var diagnostics: PackedStringArray = PackedStringArray()
## Also print each new diagnostic with push_warning (tests switch this off).
var log_diagnostics: bool = true

var _assembly = null
## One entry per body: {name, body, instance, signature}; one per tyre: {name, body, side, instance, signature}. An instance is null while
## the item is skipped (unavailable body or invalid dimensions); its slot stays so a later valid state is picked up.
var _boxes: Array = []
var _tires: Array = []


func toggle() -> bool:
	set_enabled(not enabled)
	return enabled


func set_enabled(value: bool) -> void:
	enabled = value
	visible = value


func _init() -> void:
	visible = false


## Builds the box meshes and (when the geometry exists) the tyre meshes for `assembly`; replaces any earlier content. A null assembly
## leaves the wireframe empty with a diagnostic.
func setup(assembly) -> void:
	_clear()
	_assembly = assembly
	if assembly == null:
		_note("geometry wireframe: no assembly")
		return
	for body_name in BODY_NAMES:
		_boxes.append({"name": body_name, "body": null, "instance": null, "signature": null})
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
		_refresh_box(item)  # cheap signature compare; rebuilds only after a dimension change
		_place(item, alpha)
	for item in _tires:
		_refresh_tire(item)  # same for the tyre surfaces
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


## The instance (MeshInstance3D) of a box, by body name, or null.
func box_instance(body_name: String):
	for item in _boxes:
		if item.name == body_name:
			return item.instance
	return null


## The instance of the front or rear tyre surface ("front" / "rear"), or null.
func tire_instance(side: String):
	for item in _tires:
		if item.side == side:
			return item.instance
	return null


## The stored dimension signature of a box (by body name) or a tyre ("front" / "rear"): [] when unknown or skipped. A copy.
func box_signature(body_name: String) -> Array:
	for item in _boxes:
		if item.name == body_name:
			return (item.signature as Array).duplicate() if item.signature != null else []
	return []


func tire_signature(side: String) -> Array:
	for item in _tires:
		if item.side == side:
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


## (Re)builds one box when its source dimensions changed (or it was never built); the body reference is re-read every call.
func _refresh_box(item: Dictionary) -> void:
	var entry: Dictionary = bound_entry(_assembly, item.name, self)
	item.body = entry.body if not entry.is_empty() else null
	var signature: Array = [] if entry.is_empty() else [entry.half_extents.x, entry.half_extents.y, entry.half_extents.z,
		entry.center.x, entry.center.y, entry.center.z]
	if item.signature != null and same_signature(signature, item.signature) and (item.instance != null or entry.is_empty()):
		return
	item.signature = signature
	if item.instance != null:
		if is_instance_valid(item.instance):
			remove_child(item.instance)
			item.instance.free()
		item.instance = null
	if entry.is_empty():
		return  # bound_entry already reported why
	item.instance = _make_instance(box_line_vertices(entry.half_extents, entry.center), BOX_COLOR)


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


# ---------------------------------------------------------------------------------------------------------------------
# Pure geometry (static; scene-tree independent, unit-tested)
# ---------------------------------------------------------------------------------------------------------------------

## The oriented bounding box of every one of the six bodies that can be bounded:
## [{name, body, center (body frame), half_extents (body frame), source}]. Bodies or dimensions that are unavailable or invalid are
## skipped; `reporter` (any object with `_note(String)`, may be null) receives one message for each.
static func bounds_entries(assembly, reporter = null) -> Array:
	var entries: Array = []
	if assembly == null or assembly.config == null:
		_report(reporter, "geometry wireframe: assembly or configuration unavailable")
		return entries
	for body_name in BODY_NAMES:
		var entry: Dictionary = bound_entry(assembly, body_name, reporter)
		if not entry.is_empty():
			entries.append(entry)
	return entries


## The box of ONE body by name (see BODY_NAMES), computed from the assembly's current configuration; {} (with a message to
## `reporter`) when the body or its dimensions are unavailable or invalid.
static func bound_entry(assembly, body_name: String, reporter = null) -> Dictionary:
	if assembly == null or assembly.config == null:
		_report(reporter, "geometry wireframe: assembly or configuration unavailable")
		return {}
	var config = assembly.config
	var box_bodies: Dictionary = {"chassis": [assembly.chassis, config.chassis], "steering": [assembly.steering, config.steering],
		"fork_slider": [assembly.fork_slider, config.fork_slider], "swingarm": [assembly.swingarm, config.swingarm]}
	if box_bodies.has(body_name):
		var body = box_bodies[body_name][0]
		var spec = box_bodies[body_name][1]
		if body == null or spec == null:
			_report(reporter, "geometry wireframe: body '%s' unavailable; no bounding box" % body_name)
			return {}
		var size: Vector3 = spec.visual_size
		if not (size.is_finite() and size.x > 0.0 and size.y > 0.0 and size.z > 0.0):
			_report(reporter, "geometry wireframe: body '%s' has no valid size (%s); no bounding box" % [body_name, size])
			return {}
		return {"name": body_name, "body": body, "center": Vector3.ZERO, "half_extents": size * 0.5, "source": "body spec size"}
	if body_name != "front_wheel" and body_name != "rear_wheel":
		_report(reporter, "geometry wireframe: unknown body '%s'" % body_name)
		return {}
	var side: String = "front" if body_name == "front_wheel" else "rear"
	var wheel = assembly.front_wheel if side == "front" else assembly.rear_wheel
	var geometry = assembly.front_tire_geometry if side == "front" else assembly.rear_tire_geometry
	var radius: float = config.front_wheel_radius if side == "front" else config.rear_wheel_radius
	var width: float = geometry.width if geometry != null else (config.front_wheel_width if side == "front" else config.rear_wheel_width)
	var source: String = "tire geometry width, wheel radius" if geometry != null else "wheel width, wheel radius"
	if wheel == null:
		_report(reporter, "geometry wireframe: %s wheel unavailable; no bounding box" % side)
		return {}
	if not (is_finite(radius) and radius > 0.0 and is_finite(width) and width > 0.0):
		_report(reporter, "geometry wireframe: %s wheel has no valid dimensions (radius %s, width %s); no bounding box" % [side, radius, width])
		return {}
	return {"name": body_name, "body": wheel, "center": Vector3.ZERO, "half_extents": Vector3(0.5 * width, radius, radius), "source": source}


## True when two signatures have the same length and every component differs by at most DIMENSION_TOLERANCE (NaN never matches).
static func same_signature(a: Array, b: Array) -> bool:
	if a.size() != b.size():
		return false
	for k in a.size():
		if not (absf(a[k] - b[k]) <= DIMENSION_TOLERANCE):
			return false
	return true


static func _report(reporter, message: String) -> void:
	if reporter != null:
		reporter._note(message)


## The 12 edges of a box as 24 end points (line-list order), in the frame the centre and half extents are given in. Empty when the
## input is not finite or a half extent is not > 0.
static func box_line_vertices(half_extents: Vector3, center: Vector3 = Vector3.ZERO) -> PackedVector3Array:
	var vertices: PackedVector3Array = PackedVector3Array()
	if not (half_extents.is_finite() and center.is_finite() and half_extents.x > 0.0 and half_extents.y > 0.0 and half_extents.z > 0.0):
		return vertices
	var corners: Array = []
	for k in 8:
		corners.append(center + Vector3(half_extents.x * (1.0 if (k & 1) != 0 else -1.0), half_extents.y * (1.0 if (k & 2) != 0 else -1.0),
			half_extents.z * (1.0 if (k & 4) != 0 else -1.0)))
	for k in 8:
		for bit in [1, 2, 4]:
			if (k & bit) == 0:
				vertices.append(corners[k])
				vertices.append(corners[k | bit])
	return vertices


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
