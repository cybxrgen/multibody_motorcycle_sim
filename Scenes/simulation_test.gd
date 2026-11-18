extends Node3D
## Inspection scene: a double pendulum (chain of two ball joints from a static anchor) and a free-spinning
## box on a ball joint at its center of mass (intermediate-axis tumbling shows the gyroscopic term).
## This script is the only place that advances the world; it does so with the fixed-step accumulator.
##
## Keys: Space pause/resume, N single step (paused), R reset, L local axes, C COM markers, J anchors,
##       mouse drag orbit, wheel zoom.

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")
const SimMassProperties = preload("res://Simulation/Core/sim_mass_properties.gd")
const BallJoint = preload("res://Simulation/Solver/ball_joint_constraint.gd")
const SimulationRenderer = preload("res://Simulation/Presentation/simulation_renderer.gd")
const SimulationDebug = preload("res://Simulation/Debug/simulation_debug.gd")

var world: SimWorld
var renderer: SimulationRenderer = SimulationRenderer.new()
var debug_overlay: SimulationDebug = SimulationDebug.new()
var paused: bool = false

var _camera: Camera3D = Camera3D.new()
var _yaw: float = 0.4
var _pitch: float = -0.25
var _distance: float = 9.0
var _target: Vector3 = Vector3(1.5, 3.5, 0.0)
var _label: Label = Label.new()


func _ready() -> void:
	add_child(renderer)
	add_child(debug_overlay)
	add_child(_camera)
	var light: DirectionalLight3D = DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-50, -30, 0)
	add_child(light)
	var env: WorldEnvironment = WorldEnvironment.new()
	env.environment = Environment.new()
	env.environment.background_mode = Environment.BG_COLOR
	env.environment.background_color = Color(0.12, 0.13, 0.16)
	env.environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.environment.ambient_light_color = Color(0.6, 0.6, 0.65)
	add_child(env)
	var layer: CanvasLayer = CanvasLayer.new()
	_label.position = Vector2(12, 8)
	layer.add_child(_label)
	add_child(layer)
	_build_scenario()
	_update_camera()


func _build_scenario() -> void:
	renderer.clear()
	world = SimWorld.new()
	renderer.world = world
	debug_overlay.world = world

	# Double pendulum, released horizontally from a static anchor at (0, 5, 0).
	var pivot: Vector3 = Vector3(0, 5, 0)
	var anchor: SimRigidBody = world.add_body(SimRigidBody.new(SimMassProperties.new(0.0), pivot))
	var link_size: Vector3 = Vector3(1, 0.1, 0.1)
	var link_inertia: Basis = SimMassProperties.box_inertia(1.0, link_size)
	var link1: SimRigidBody = world.add_body(SimRigidBody.new(SimMassProperties.new(1.0, link_inertia), pivot + Vector3(0.5, 0, 0)))
	var link2: SimRigidBody = world.add_body(SimRigidBody.new(SimMassProperties.new(1.0, link_inertia), pivot + Vector3(1.5, 0, 0)))
	world.add_constraint(BallJoint.new(anchor, link1, Vector3.ZERO, Vector3(-0.5, 0, 0)))
	world.add_constraint(BallJoint.new(link1, link2, Vector3(0.5, 0, 0), Vector3(-0.5, 0, 0)))
	renderer.add_sphere(anchor, 0.08, Color(0.6, 0.6, 0.6))
	renderer.add_box(link1, link_size, Color(0.9, 0.5, 0.2))
	renderer.add_box(link2, link_size, Color(0.3, 0.7, 0.9))

	# Torque-free spinner: ball joint at its COM supports its weight, spin about the intermediate axis.
	var spin_pos: Vector3 = Vector3(4, 3, 0)
	var spin_size: Vector3 = Vector3(0.6, 0.3, 0.15)
	var spin_anchor: SimRigidBody = world.add_body(SimRigidBody.new(SimMassProperties.new(0.0), spin_pos))
	var spinner: SimRigidBody = world.add_body(SimRigidBody.new(SimMassProperties.new(1.0, SimMassProperties.box_inertia(1.0, spin_size)), spin_pos))
	spinner.angular_velocity = Vector3(0.2, 6.0, 0.2)
	world.add_constraint(BallJoint.new(spin_anchor, spinner, Vector3.ZERO, Vector3.ZERO))
	renderer.add_sphere(spin_anchor, 0.06, Color(0.6, 0.6, 0.6))
	renderer.add_box(spinner, spin_size, Color(0.8, 0.3, 0.5))


func _process(delta: float) -> void:
	if not paused:
		world.advance(delta)
	var alpha: float = world.interpolation_alpha() if not paused else 1.0
	renderer.render(alpha)
	debug_overlay.update_overlay(alpha)
	_label.text = "%s\n%s\n[Space] pause  [N] step  [R] reset  [L] local axes  [C] COM  [J] anchors" % [
		"PAUSED" if paused else "running", debug_overlay.build_report()]


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_SPACE:
				paused = not paused
			KEY_N:
				if paused:
					world.step()
			KEY_R:
				_build_scenario()
			KEY_L:
				debug_overlay.show_local_axes = not debug_overlay.show_local_axes
			KEY_C:
				debug_overlay.show_com = not debug_overlay.show_com
			KEY_J:
				debug_overlay.show_anchors = not debug_overlay.show_anchors
	elif event is InputEventMouseMotion and (event.button_mask & MOUSE_BUTTON_MASK_LEFT) != 0:
		_yaw -= event.relative.x * 0.005
		_pitch = clampf(_pitch - event.relative.y * 0.005, -1.5, 1.5)
		_update_camera()
	elif event is InputEventMouseButton and event.pressed:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			_distance = maxf(2.0, _distance * 0.92)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_distance = minf(40.0, _distance * 1.08)
		_update_camera()


func _update_camera() -> void:
	var rot: Basis = Basis.from_euler(Vector3(_pitch, _yaw, 0.0))
	_camera.transform = Transform3D(rot, _target + rot * Vector3(0, 0, _distance))
