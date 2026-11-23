extends RefCounted
## ILLUSTRATIVE TEST-FIXTURE PARAMETERS for the motorcycle assembly (M2.1 rigid assembly, M2.2 front telescopic fork).
##
## These numbers are NOT measured and NOT calibrated: they are plausible, motorcycle-proportioned values chosen so that the
## first assembly is a well-conditioned, runnable example (distinct masses, non-trivial inertias, a raked steering axis and a
## tilted swingarm so that the bodies do not share local frames). Do not read them as the specification of any real
## motorcycle, and do not tune the simulation against them. Every value can be changed in the returned MotorcycleConfig.
##
## Reference pose (all hinge angles zero), world frame, +Y up, local -Z forward, +X right, flat ground plane at y = 0 (simulated only by config_with_ground(), M2.4.1):
##   * Everything lies in the symmetry plane x = 0. The motorcycle points toward -Z (front wheel at negative z).
##   * Steering axis: raked by RAKE from the vertical, leaning rearward toward the top: a = (0, cos RAKE, sin RAKE).
##     Positive steering-hinge angle = right-handed about +a = steering to the LEFT (front of the fork toward -X).
##   * Fork axis (prismatic joint): the steering axis line, directed DOWN the fork (-a), from the steering assembly toward the fork
##     slider. Positive coordinate = EXTENSION of the fork, negative = compression. The slider moves along the raked axis, not
##     along world vertical.
##   * Swingarm axis +X. Positive angle = right-handed about +X = the rear end of the swingarm moves DOWN.
##   * Wheel axles +X. Positive angle = right-handed about +X = the wheel top moves rearward, so rolling FORWARD (toward -Z)
##     is a NEGATIVE angle rate about the +X axle axis.
## Primitive approximations (full side lengths, uniform density; inertia about the COM in the body frame):
##   chassis    solid box 0.30 x 0.55 x 1.00 m (x, y, z)           -> SimMassProperties.box_inertia
##   steering   solid box 0.60 x 0.30 x 0.16 m in the fork frame   -> box_inertia (local +Y along the steering axis); the steering
##              stem, triple clamps, handlebar and upper fork tubes (14 kg)
##   fork slider solid box 0.24 x 0.40 x 0.14 m in the fork frame  -> box_inertia; lower fork legs and axle carrier (8 kg). Together
##              steering + slider are the 22 kg fork assembly of M2.1
##   swingarm   solid box 0.20 x 0.10 x L m (L = pivot-to-axle)    -> box_inertia (local +Z along the swingarm)
##   wheels     hollow cylinder, outer radius R, inner radius WHEEL_INNER_FRACTION * R (a rim-and-tyre-like ring), width w
##                                                                 -> MotorcycleConfig.wheel_inertia (axle = local X)
## Front suspension (illustrative, NOT calibrated): spring seats on the fork axis, k = 12 kN/m, c = 400 N s/m, rest displacement
## 0 (unloaded at the reference pose), travel limits 0.12 m compression / 0.03 m extension from the reference pose.
## Rear suspension (M2.3, illustrative, NOT calibrated): a direct spring-damper from a chassis point (0, 0.80, 0.22) to a swingarm
## point (0, 0.40, 0.58) behind and below it, so the shock is inclined (neither vertical nor along a body axis) and its direction
## changes as the swingarm rotates; k = 30 kN/m, c = 1200 N s/m, rest length = the attachment distance at the reference pose
## (unloaded there; no preload or sag, like the front). With the shock's lever arm about the swingarm pivot (~0.21 m) this is a
## torsional stiffness of ~1.3 kN m/rad (about 0.4 damping ratio against the swingarm-plus-wheel inertia); the numbers are a
## plausible order of magnitude for a demonstration, not a claim about any motorcycle.

const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const BodySpec = preload("res://Simulation/Motorcycle/motorcycle_body_spec.gd")
const JointSpec = preload("res://Simulation/Motorcycle/motorcycle_joint_spec.gd")
const GroundSpec = preload("res://Simulation/Motorcycle/motorcycle_ground_spec.gd")
const SimMassProperties = preload("res://Simulation/Core/sim_mass_properties.gd")

## Steering-axis rake from the vertical (degrees).
const RAKE_DEGREES: float = 26.0
## Inner radius of the wheel ring as a fraction of the outer radius (mass concentrated toward the rim).
const WHEEL_INNER_FRACTION: float = 0.75


## Illustrative Coulomb friction coefficient of the prototype ground (M2.4.1; not a tyre model, not calibrated).
const GROUND_FRICTION: float = 0.8


## The prototype configuration with the flat ground plane y = 0 (normal +Y) and its two wheel contacts (M2.4.1). The reference pose
## has the rear wheel exactly touching the plane and the front wheel about 3 mm above it. A fresh configuration each call.
static func config_with_ground() -> MotorcycleConfig:
	var c: MotorcycleConfig = config()
	c.ground = GroundSpec.new(Vector3.ZERO, Vector3.UP, GROUND_FRICTION)
	return c


## A fresh, independent configuration each call (callers may mutate it). It has no ground (see config_with_ground()).
static func config() -> MotorcycleConfig:
	var c: MotorcycleConfig = MotorcycleConfig.new()
	var rake: float = deg_to_rad(RAKE_DEGREES)

	# Steering geometry. `steer_axis` points up and rearward; `fork_forward` is perpendicular to it and points forward (-Z).
	var steer_axis: Vector3 = Vector3(0.0, cos(rake), sin(rake))
	var fork_forward: Vector3 = Vector3(0.0, sin(rake), -cos(rake))
	var head_point: Vector3 = Vector3(0.0, 0.88, -0.33)  # on the steering axis, at the chassis
	var fork_length: float = 0.65    # head point to the front axle measured along the steering axis
	var fork_offset: float = 0.04    # front axle displaced forward from the steering axis, perpendicular to it
	var steering_com_below_head: float = 0.12  # steering-assembly (upper fork) COM, measured down the axis from the head point
	var steering_com_offset: float = 0.02      # steering-assembly COM, forward of the axis
	var slider_com_below_head: float = 0.50    # fork-slider (lower fork) COM, measured down the axis from the head point
	var slider_com_offset: float = 0.03        # fork-slider COM, forward of the axis
	var fork_joint_below_head: float = 0.35    # prismatic joint point on the axis, measured down from the head point
	var seat_upper_below_head: float = 0.20    # spring seat on the steering assembly (on the axis)
	var seat_lower_below_head: float = 0.42    # spring seat on the fork slider (on the axis)
	var front_axle_point: Vector3 = head_point - steer_axis * fork_length + fork_forward * fork_offset

	# Swingarm geometry.
	var pivot_point: Vector3 = Vector3(0.0, 0.42, 0.28)
	var rear_axle_point: Vector3 = Vector3(0.0, 0.32, 0.83)
	var arm: Vector3 = rear_axle_point - pivot_point
	var arm_length: float = arm.length()
	# Swingarm frame: local +Z along the arm. A rotation about +X by `tilt` maps +Z to (0, -sin tilt, cos tilt).
	var tilt: float = atan2(-arm.y, arm.z)

	# Wheels.
	c.front_wheel_radius = 0.31
	c.rear_wheel_radius = 0.32
	c.front_wheel_width = 0.12
	c.rear_wheel_width = 0.18

	# Bodies (name, mass kg, inertia about COM, COM position, orientation, box proxy size).
	c.chassis = BodySpec.new("chassis", 120.0, SimMassProperties.box_inertia(120.0, Vector3(0.30, 0.55, 1.00)),
		Vector3(0.0, 0.62, 0.06), Quaternion.IDENTITY, Vector3(0.30, 0.55, 1.00))
	# The fork frame is the world frame rotated about +X by the rake: local +Y = steering axis, local -Z = fork_forward.
	c.steering = BodySpec.new("steering", 14.0, SimMassProperties.box_inertia(14.0, Vector3(0.60, 0.30, 0.16)),
		head_point - steer_axis * steering_com_below_head + fork_forward * steering_com_offset,
		Quaternion(Vector3.RIGHT, rake), Vector3(0.60, 0.30, 0.16))
	c.fork_slider = BodySpec.new("fork_slider", 8.0, SimMassProperties.box_inertia(8.0, Vector3(0.24, 0.40, 0.14)),
		head_point - steer_axis * slider_com_below_head + fork_forward * slider_com_offset,
		Quaternion(Vector3.RIGHT, rake), Vector3(0.24, 0.40, 0.14))
	c.swingarm = BodySpec.new("swingarm", 9.0, SimMassProperties.box_inertia(9.0, Vector3(0.20, 0.10, arm_length)),
		(pivot_point + rear_axle_point) * 0.5, Quaternion(Vector3.RIGHT, tilt), Vector3(0.20, 0.10, arm_length))
	c.front_wheel = BodySpec.new("front_wheel", 12.0,
		MotorcycleConfig.wheel_inertia(12.0, c.front_wheel_radius, WHEEL_INNER_FRACTION * c.front_wheel_radius, c.front_wheel_width),
		front_axle_point, Quaternion.IDENTITY)
	c.rear_wheel = BodySpec.new("rear_wheel", 15.0,
		MotorcycleConfig.wheel_inertia(15.0, c.rear_wheel_radius, WHEEL_INNER_FRACTION * c.rear_wheel_radius, c.rear_wheel_width),
		rear_axle_point, Quaternion.IDENTITY)

	# Hinges (world pivot, world axis at the reference pose). Each wheel COM is on its axle.
	c.steering_head = JointSpec.new("steering_head", head_point, steer_axis)
	c.swingarm_pivot = JointSpec.new("swingarm_pivot", pivot_point, Vector3.RIGHT)
	c.front_axle = JointSpec.new("front_axle", front_axle_point, Vector3.RIGHT)
	c.rear_axle = JointSpec.new("rear_axle", rear_axle_point, Vector3.RIGHT)

	# Fork: prismatic joint on the steering axis line, directed down the fork (positive = extension), and the spring-damper.
	c.fork_travel = JointSpec.new("fork_travel", head_point - steer_axis * fork_joint_below_head, -steer_axis)
	c.front_suspension.upper_attachment = head_point - steer_axis * seat_upper_below_head
	c.front_suspension.lower_attachment = head_point - steer_axis * seat_lower_below_head
	c.front_suspension.stiffness = 12000.0
	c.front_suspension.damping = 400.0
	c.front_suspension.rest_displacement = 0.0
	c.front_suspension.travel_limits_enabled = true
	c.front_suspension.lower_travel_limit = -0.12
	c.front_suspension.upper_travel_limit = 0.03

	# Rear suspension: direct spring-damper between a chassis point and a swingarm point; rest length = reference length.
	c.rear_suspension.chassis_attachment = Vector3(0.0, 0.80, 0.22)
	c.rear_suspension.swingarm_attachment = Vector3(0.0, 0.40, 0.58)
	c.rear_suspension.stiffness = 30000.0
	c.rear_suspension.damping = 1200.0
	c.rear_suspension.rest_length = (c.rear_suspension.swingarm_attachment - c.rear_suspension.chassis_attachment).length()
	return c
