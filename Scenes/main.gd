extends Node3D
## Entry scene: hosts the simulation inspection scene.


func _ready() -> void:
	var packed: PackedScene = load("res://Scenes/simulation_test.tscn")
	add_child(packed.instantiate())
