extends Node3D
## M2.1/M2.2 demonstration: the motorcycle assembly (six rigid bodies, four free hinges, the fork's prismatic joint with a
## spring-damper and finite travel) in ZERO gravity, built by MotorcycleBuilder from the ILLUSTRATIVE prototype parameters (not a
## real motorcycle). It is not riding: there is no ground, no rear suspension, no tyre and no drive, and the hinges have no limits,
## so the free parts keep turning and pass through each other. The front fork is a real telescopic suspension: the upper steering
## assembly (orange) and the lower fork slider (cyan) are separate bodies; the front wheel (red) hangs on the slider and the slider
## slides along the raked fork axis against the spring-damper and its travel stops.
##
## Initial conditions (set once at scenario build, exactly as simulation_test.gd sets its spinner's velocity; nothing is applied
## afterwards): for every hinge an equal and opposite pair of angular impulses about its axis, so total momentum is zero and the
## assembly does not drift: steering, swingarm and both wheels each get a different relative rotation. One more equal and opposite
## pair of impulses along the fork axis (steering assembly vs fork slider) makes the fork travel visibly. This script only advances
## the world with the fixed-step accumulator (like simulation_test.gd) and renders PUBLISHED state; it never writes physical state.
##
## Colors: chassis blue-grey, steering assembly orange, fork slider cyan, swingarm green, front wheel red, rear wheel yellow; wheels carry a contrasting
## spoke bar and valve marker so their rotation is visible. Overlay: magenta/cyan crosses are the two anchors of each hinge.
## Keys: Space pause/resume, N single step (paused), R reset, L local axes, C COM markers, J anchors, mouse drag orbit, wheel zoom.

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const SimBodyState = preload("res://Simulation/Core/sim_body_state.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const SimulationRenderer = preload("res://Simulation/Presentation/simulation_renderer.gd")
const SimulationDebug = preload("res://Simulation/Debug/simulation_debug.gd")

var world: SimWorld
var assembly = null
var renderer: SimulationRenderer = SimulationRenderer.new()
var debug_overlay: SimulationDebug = SimulationDebug.new()
var paused: bool = false

var _camera: Camera3D = Camera3D.new()
var _yaw: float = 0.9
var _pitch: float = -0.2
var _distance: float = 3.6
var _target: Vector3 = Vector3(0.0, 0.6, 0.0)
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
	world.config.gravity = Vector3.ZERO
	var config = MotorcyclePrototype.config()
	assembly = MotorcycleBuilder.build(config, world)
	if assembly == null:
		push_error("motorcycle_test: the assembly could not be built.")
		return
	renderer.world = world
	debug_overlay.world = world

	# Initial relative rotations: equal and opposite angular impulses about each hinge axis (total momentum stays zero).
	var specs: Array = config.joint_specs()
	var joints: Array = assembly.joints()
	var kicks: Array = [1.5, 1.0, config.front_wheel.local_inertia.x.x * -8.0, config.rear_wheel.local_inertia.x.x * -5.0]
	for k in 4:
		joints[k].body_b.apply_angular_impulse(specs[k].axis * kicks[k])
		joints[k].body_a.apply_angular_impulse(specs[k].axis * -kicks[k])
	# Fork travel: equal and opposite linear impulses along the fork axis on the slider and the steering assembly (zero net momentum).
	assembly.fork_slider.apply_impulse(config.fork_travel.axis * -25.0)
	assembly.steering.apply_impulse(config.fork_travel.axis * 25.0)

	renderer.add_box(assembly.chassis, config.chassis.visual_size, Color(0.45, 0.55, 0.75))
	renderer.add_box(assembly.steering, config.steering.visual_size, Color(0.95, 0.55, 0.15))
	renderer.add_box(assembly.swingarm, config.swingarm.visual_size, Color(0.3, 0.75, 0.35))
	renderer.add_box(assembly.fork_slider, config.fork_slider.visual_size, Color(0.25, 0.8, 0.85))
	_add_wheel(assembly.front_wheel, config.front_wheel_radius, config.front_wheel_width, Color(0.85, 0.2, 0.2))
	_add_wheel(assembly.rear_wheel, config.rear_wheel_radius, config.rear_wheel_width, Color(0.9, 0.8, 0.2))


## Wheel proxy: a cylinder with its axis along the body-local X axle, a diametral spoke bar and an off-centre valve marker.
func _add_wheel(body, radius: float, width: float, color: Color) -> void:
	var cylinder: CylinderMesh = CylinderMesh.new()
	cylinder.top_radius = radius
	cylinder.bottom_radius = radius
	cylinder.height = width
	renderer.add_mesh(body, cylinder, color, Transform3D(Basis(Vector3.BACK, -PI / 2.0), Vector3.ZERO))
	var spoke: BoxMesh = BoxMesh.new()
	spoke.size = Vector3(width * 1.15, radius * 2.0 * 0.97, 0.035)
	renderer.add_mesh(body, spoke, Color(0.1, 0.1, 0.12))
	var valve: BoxMesh = BoxMesh.new()
	valve.size = Vector3(width * 1.3, 0.05, 0.05)
	renderer.add_mesh(body, valve, Color(1.0, 1.0, 1.0), Transform3D(Basis.IDENTITY, Vector3(0.0, radius * 0.88, 0.0)))


func _process(delta: float) -> void:
	if world == null or assembly == null:
		return
	if not paused:
		world.advance(delta)
	var alpha: float = world.interpolation_alpha() if not paused else 1.0
	renderer.render(alpha)
	debug_overlay.update_overlay(alpha)
	_label.text = "%s\n%s\n%s\n%s\n[Space] pause  [N] step  [R] reset  [L] local axes  [C] COM  [J] anchors\nZero gravity, free hinges, illustrative prototype values: not a ride." % [
		"PAUSED" if paused else "running", debug_overlay.build_report(), _joint_report(), _fork_report()]


## Relative angular velocity about each hinge axis, from PUBLISHED state only (read-only; no constraint or body is touched).
func _joint_report() -> String:
	var names: Array = ["steering", "swingarm", "front wheel", "rear wheel"]
	var text: String = "relative rate [rad/s]:"
	var joints: Array = assembly.joints()
	for k in 4:
		var j = joints[k]
		var axis: Vector3 = j.body_a.published_state.orientation * j.local_axis_a
		var rate: float = (j.body_b.published_state.angular_velocity - j.body_a.published_state.angular_velocity).dot(axis)
		text += "  %s %.2f" % [names[k], rate]
	return text


## Fork displacement (m, + = extension, - = compression) and axial velocity (m/s) of the spring seats, computed ONLY from the
## published (render-side) body states with the same formula as FrontSuspension (read-only; no body, joint or force is touched).
func _fork_report() -> String:
	var susp = assembly.front_suspension
	var a = assembly.steering.published_state
	var b = assembly.fork_slider.published_state
	var axis: Vector3 = a.orientation * assembly.fork_joint.local_axis_a
	var offset_a: Vector3 = a.orientation * susp.local_attachment_a
	var offset_b: Vector3 = b.orientation * susp.local_attachment_b
	var separation: Vector3 = (b.position + offset_b) - (a.position + offset_a)
	var v_a: Vector3 = a.linear_velocity + a.angular_velocity.cross(offset_a)
	var v_b: Vector3 = b.linear_velocity + b.angular_velocity.cross(offset_b)
	var displacement: float = separation.dot(axis) - susp.reference_separation
	var travel: String = "travel %.3f..%.3f m" % [assembly.fork_joint.lower_limit, assembly.fork_joint.upper_limit]
	return "fork: displacement %+.4f m  axial velocity %+.3f m/s  (%s)" % [displacement, (v_b - v_a).dot(axis), travel]


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_SPACE:
				paused = not paused
			KEY_N:
				if paused and world != null:
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
			_distance = maxf(1.0, _distance * 0.92)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_distance = minf(20.0, _distance * 1.08)
		_update_camera()


func _update_camera() -> void:
	var rot: Basis = Basis.from_euler(Vector3(_pitch, _yaw, 0.0))
	_camera.transform = Transform3D(rot, _target + rot * Vector3(0, 0, _distance))
