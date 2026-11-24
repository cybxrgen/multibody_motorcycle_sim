extends Node3D
## M2.4.2 demonstration: the motorcycle assembly (six rigid bodies, four free hinges, the fork's prismatic joint with a spring-damper
## and finite travel, the rear spring-damper) under GRAVITY on a flat static ground plane (y = 0), built by MotorcycleBuilder from the
## ILLUSTRATIVE prototype parameters (not a real motorcycle) with MotorcyclePrototype.config_with_tires(). Each wheel touches the plane
## through one TireGroundContact: the generic unilateral normal response plus a deliberately simple tyre model (longitudinal and lateral slip,
## combined-slip friction ellipse, rolling resistance) that is the ONLY owner of the tangential force (no generic Coulomb friction on top).
## Tyre values are illustrative prototype parameters, NOT measured tyre data. Nothing is positioned by hand: the scene only advances the
## world with the fixed-step accumulator and renders PUBLISHED state; it never writes physical state.
##
## PHYSICAL MODEL vs EXTERNAL TEST INPUT. The physical model is everything the world integrates by itself: bodies, joints, suspension,
## normal contact and the tyre model. The only external inputs are the two toggles below, applied by a small TEST-ONLY force provider
## (TestInput, defined in this script, registered in the world so it acts at every fixed step): an axle torque on the rear wheel
## ([T], "drive") and an axle torque opposing the spin of both wheels ([B], "brake"). They are NOT an engine, clutch, gearbox, drivetrain,
## brake system, rider or controller. Wheel torques only change the wheel spin; the tyres turn the resulting slip into force.
## Cornering and combined braking + lateral slip need a lateral disturbance that a riderless motorcycle cannot hold, so they are exercised
## in Tests/test_tire_forces.gd (single wheel) and Tests/test_motorcycle_tires.gd (assembly on a TEST-ONLY planar fixture), not here.
##
## NOT a ride, NOT self-balancing, NOT a complete tyre model: a riderless motorcycle on two narrow point contacts is not stable in roll, and
## there is no rider, drive, steering control, stand or stabilizer. The scene is exactly symmetric about its x = 0 plane, so under [T] and
## [B] (both symmetric) it keeps standing/running straight in a knife-edge balance; that is NOT stability. A small disturbance topples it.
##
## Colors: chassis blue-grey, steering assembly orange, fork slider cyan, swingarm green, front wheel red, rear wheel yellow; wheels carry
## a contrasting spoke bar and valve marker so their rotation is visible; the dark slab is the ground plane. Contact markers (spheres at
## each wheel's lowest point, derived from the published pose): GREEN = touching (|gap| < 0.1 mm or penetrating), ORANGE = separated.
## Overlay: magenta/cyan crosses are the two anchors of each constraint (hinges and contacts). The label lists, per tyre, the signed gap,
## the normal load (last step's normal impulse / dt, read-only), the tyre slips (from the published pose and velocities), the longitudinal
## and lateral tyre force, the friction-ellipse utilisation and the rolling-resistance moment.
## Keys: Space pause/resume, N single step (paused), R reset, T external drive torque on/off, B external brake torque on/off, L local axes,
## C COM markers, J anchors, mouse drag orbit, wheel zoom.

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const SimBodyState = preload("res://Simulation/Core/sim_body_state.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")
const SimulationRenderer = preload("res://Simulation/Presentation/simulation_renderer.gd")
const SimulationDebug = preload("res://Simulation/Debug/simulation_debug.gd")
const SimForceProvider = preload("res://Simulation/Core/sim_force_provider.gd")
const TireModel = preload("res://Simulation/Motorcycle/tire_model.gd")

## Illustrative magnitudes of the two external test inputs (N m).
const DRIVE_TORQUE: float = 60.0
const BRAKE_TORQUE: float = 40.0


## TEST-ONLY external input (not part of the motorcycle model): axle torques applied at every fixed step.
class TestInput extends SimForceProvider:
	var front_wheel
	var rear_wheel
	var drive: bool = false
	var brake: bool = false

	func involved_bodies() -> Array:
		return [front_wheel, rear_wheel]

	func apply_forces(_dt: float, _config) -> void:
		var rear_axle: Vector3 = rear_wheel.orientation * Vector3.RIGHT
		if drive:
			rear_wheel.apply_torque(-rear_axle * DRIVE_TORQUE)  # positive drive: the wheel turns about -axle (forward rolling)
		if brake:
			for wheel in [front_wheel, rear_wheel]:
				var axle: Vector3 = wheel.orientation * Vector3.RIGHT
				var spin: float = -wheel.angular_velocity.dot(axle)
				wheel.apply_torque(axle * BRAKE_TORQUE * clampf(spin / 0.5, -1.0, 1.0))  # opposes the spin

var world: SimWorld
var assembly = null
var renderer: SimulationRenderer = SimulationRenderer.new()
var debug_overlay: SimulationDebug = SimulationDebug.new()
var paused: bool = false
var test_input: TestInput = TestInput.new()

var _camera: Camera3D = Camera3D.new()
var _yaw: float = 0.9
var _pitch: float = -0.2
var _distance: float = 3.6
var _target: Vector3 = Vector3(0.0, 0.6, 0.0)
var _label: Label = Label.new()
var _markers: Array = []


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
	for marker in _markers:
		marker.queue_free()
	_markers.clear()
	world = SimWorld.new()  # default SimConfig: gravity (0, -9.81, 0)
	var config = MotorcyclePrototype.config_with_tires()
	assembly = MotorcycleBuilder.build(config, world)
	if assembly == null:
		push_error("motorcycle_test: the assembly could not be built.")
		return
	test_input = TestInput.new()
	test_input.front_wheel = assembly.front_wheel
	test_input.rear_wheel = assembly.rear_wheel
	world.add_force_provider(test_input)
	renderer.world = world
	debug_overlay.world = world

	renderer.add_box(assembly.chassis, config.chassis.visual_size, Color(0.45, 0.55, 0.75))
	renderer.add_box(assembly.steering, config.steering.visual_size, Color(0.95, 0.55, 0.15))
	renderer.add_box(assembly.swingarm, config.swingarm.visual_size, Color(0.3, 0.75, 0.35))
	renderer.add_box(assembly.fork_slider, config.fork_slider.visual_size, Color(0.25, 0.8, 0.85))
	_add_wheel(assembly.front_wheel, config.front_wheel_radius, config.front_wheel_width, Color(0.85, 0.2, 0.2))
	_add_wheel(assembly.rear_wheel, config.rear_wheel_radius, config.rear_wheel_width, Color(0.9, 0.8, 0.2))

	# Ground plane: a thin slab whose TOP face is the plane y = 0 (the ground body's COM frame is on the plane).
	var slab: BoxMesh = BoxMesh.new()
	slab.size = Vector3(12.0, 0.02, 12.0)
	renderer.add_mesh(assembly.ground, slab, Color(0.2, 0.22, 0.25), Transform3D(Basis.IDENTITY, Vector3(0.0, -0.01, 0.0)))
	for k in 2:
		var marker: MeshInstance3D = MeshInstance3D.new()
		var sphere: SphereMesh = SphereMesh.new()
		sphere.radius = 0.03
		sphere.height = 0.06
		marker.mesh = sphere
		marker.material_override = StandardMaterial3D.new()
		add_child(marker)
		_markers.append(marker)


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
	_update_contact_markers(alpha)
	_label.text = "%s\n%s\n%s\n%s\n%s\n[Space] pause  [N] step  [R] reset  [T] external drive torque  [B] external brake torque  [L] local axes  [C] COM  [J] anchors\nEXTERNAL TEST INPUT (not a drivetrain or rider): drive %s (%.0f N m on the rear axle), brake %s (%.0f N m against both spins).\nPhysical model: gravity, free hinges, suspension, normal contact, simple illustrative tyres (not measured data). Not a ride, not self-balancing, not a complete tyre model (no roll stability: running only because the scene is exactly symmetric).\nGreen marker = touching, orange = separated." % [
		"PAUSED" if paused else "running", debug_overlay.build_report(), _joint_report(), _fork_report(), _contact_report(),
		"ON" if test_input.drive else "off", DRIVE_TORQUE, "ON" if test_input.brake else "off", BRAKE_TORQUE]


## Pose of a body for rendering (interpolated PUBLISHED state; read-only).
func _pose(body, alpha: float) -> Transform3D:
	return SimBodyState.interpolate_transform(body.previous_state, body.published_state, alpha)


## Signed gap (m) of a wheel's lowest point above the plane, from the interpolated published pose; NAN for degenerate geometry.
func _published_geometry(contact, alpha: float) -> Dictionary:
	var t: Transform3D = _pose(contact.body_b, alpha)
	return WheelGroundContact.compute_geometry(t.origin, t.basis.get_rotation_quaternion(), contact.wheel_radius, contact.plane_point, contact.plane_normal)


## One sphere per wheel at its lowest point; green while touching (gap < 0.1 mm, including penetration), orange while separated.
func _update_contact_markers(alpha: float) -> void:
	var contacts: Array = assembly.ground_contacts()
	for k in contacts.size():
		var geometry: Dictionary = _published_geometry(contacts[k], alpha)
		var marker: MeshInstance3D = _markers[k]
		marker.visible = geometry.valid
		if geometry.valid:
			marker.position = geometry.wheel_point
			marker.material_override.albedo_color = Color(0.1, 0.9, 0.2) if geometry.gap <= WheelGroundContact.TOUCH_TOLERANCE else Color(1.0, 0.55, 0.0)


## Per tyre: signed gap and the contact state (from the published pose), the normal load, the tyre force, the friction utilisation and
## the rolling-resistance moment (last solved step, read-only queries of the solver's results), and the slips evaluated from the PUBLISHED
## state with TireModel.slip (read-only). Nothing is written.
func _contact_report() -> String:
	var text: String = "tyre contacts (surface mu = %.2f, illustrative tyre parameters):" % assembly.front_wheel_contact.friction
	var names: Array = ["front", "rear"]
	var contacts: Array = assembly.ground_contacts()
	for k in contacts.size():
		var contact = contacts[k]
		var geometry: Dictionary = _published_geometry(contact, 1.0)
		var gap_text: String = "%+.2f mm" % (geometry.gap * 1000.0) if geometry.valid else "n/a (degenerate)"
		var state: String = "TOUCHING" if geometry.valid and geometry.gap <= WheelGroundContact.TOUCH_TOLERANCE else "SEPARATED"
		var pub = contact.body_b.published_state
		var slip: Dictionary = TireModel.slip(pub.position, pub.orientation, pub.linear_velocity, pub.angular_velocity, contact.wheel_radius,
			contact.tire.rolling_radius, contact.plane_point, contact.plane_normal, contact.tire.low_speed_reference)
		var slip_text: String = "slip x %+.3f y %+.3f" % [slip.slip_x, slip.slip_y] if slip.valid else "slip n/a"
		text += "\n  %s: gap %s  %s   normal %.0f N   Fx %+.0f N  Fy %+.0f N  (limits %.0f / %.0f N, utilisation %.2f)   rolling resistance %+.2f N m   %s" % [names[k], gap_text, slip_text,
			contact.normal_load(), contact.longitudinal_force(), contact.lateral_force(), contact.longitudinal_force_limit(), contact.lateral_force_limit(),
			contact.friction_utilization(), contact.rolling_resistance_torque(), state]
	return text


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
			KEY_T:
				test_input.drive = not test_input.drive
			KEY_B:
				test_input.brake = not test_input.brake
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
