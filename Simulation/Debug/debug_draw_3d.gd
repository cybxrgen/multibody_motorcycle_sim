extends MeshInstance3D
## Immediate-mode line batcher. Call begin_frame(), then line()/axes()/cross(), then flush().
## Purely visual; holds no physical state.

var _mesh: ImmediateMesh = ImmediateMesh.new()
var _material: StandardMaterial3D = StandardMaterial3D.new()
var _points: PackedVector3Array = PackedVector3Array()
var _colors: PackedColorArray = PackedColorArray()


func _init() -> void:
	_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	_material.vertex_color_use_as_albedo = true
	_material.no_depth_test = true
	mesh = _mesh
	top_level = true
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF


func begin_frame() -> void:
	_points.clear()
	_colors.clear()


func line(a: Vector3, b: Vector3, color: Color) -> void:
	_points.append(a)
	_points.append(b)
	_colors.append(color)
	_colors.append(color)


## RGB axes of a transform (x red, y green, z blue).
func axes(t: Transform3D, length: float) -> void:
	line(t.origin, t.origin + t.basis.x.normalized() * length, Color(1, 0.2, 0.2))
	line(t.origin, t.origin + t.basis.y.normalized() * length, Color(0.2, 1, 0.2))
	line(t.origin, t.origin + t.basis.z.normalized() * length, Color(0.3, 0.5, 1))


func cross(p: Vector3, size: float, color: Color) -> void:
	line(p - Vector3(size, 0, 0), p + Vector3(size, 0, 0), color)
	line(p - Vector3(0, size, 0), p + Vector3(0, size, 0), color)
	line(p - Vector3(0, 0, size), p + Vector3(0, 0, size), color)


func flush() -> void:
	_mesh.clear_surfaces()
	if _points.is_empty():
		return
	_mesh.surface_begin(Mesh.PRIMITIVE_LINES, _material)
	for i in _points.size():
		_mesh.surface_set_color(_colors[i])
		_mesh.surface_add_vertex(_points[i])
	_mesh.surface_end()
