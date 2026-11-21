extends Node3D
## Owns the visual proxies and updates them from the world each frame.

const BodyVisualProxy = preload("res://Simulation/Presentation/body_visual_proxy.gd")

var world = null
var proxies: Array = []


func add_box(body, size: Vector3, color: Color) -> void:
	var mesh: BoxMesh = BoxMesh.new()
	mesh.size = size
	_add(body, mesh, color)


func add_sphere(body, radius: float, color: Color) -> void:
	var mesh: SphereMesh = SphereMesh.new()
	mesh.radius = radius
	mesh.height = radius * 2.0
	_add(body, mesh, color)


## Any mesh, placed relative to the body's COM frame by `mesh_offset`. A body may have several proxies.
func add_mesh(body, mesh: Mesh, color: Color, mesh_offset: Transform3D = Transform3D.IDENTITY) -> void:
	_add(body, mesh, color, mesh_offset)


func clear() -> void:
	for p in proxies:
		p.queue_free()
	proxies.clear()


func render(alpha: float) -> void:
	for p in proxies:
		p.apply_state(alpha)


func _add(body, mesh: Mesh, color: Color, mesh_offset: Transform3D = Transform3D.IDENTITY) -> void:
	var proxy: BodyVisualProxy = BodyVisualProxy.new()
	add_child(proxy)
	proxy.setup(body, mesh, color, mesh_offset)
	proxies.append(proxy)
