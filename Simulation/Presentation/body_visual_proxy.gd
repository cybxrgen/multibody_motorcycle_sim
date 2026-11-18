extends Node3D
## Visual stand-in for one simulated body. Reads interpolated published state; never writes physics.
## The node origin is the body's center of mass.

const SimBodyState = preload("res://Simulation/Core/sim_body_state.gd")

var body = null
var _mesh_instance: MeshInstance3D = MeshInstance3D.new()


func _init() -> void:
	add_child(_mesh_instance)


func setup(p_body, mesh: Mesh, color: Color) -> void:
	body = p_body
	var material: StandardMaterial3D = StandardMaterial3D.new()
	material.albedo_color = color
	_mesh_instance.mesh = mesh
	_mesh_instance.material_override = material
	apply_state(1.0)


func apply_state(alpha: float) -> void:
	if body == null:
		return
	transform = SimBodyState.interpolate_transform(body.previous_state, body.published_state, alpha)
