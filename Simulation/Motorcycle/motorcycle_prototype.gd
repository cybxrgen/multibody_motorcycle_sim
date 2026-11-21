extends RefCounted
## ILLUSTRATIVE TEST-FIXTURE PARAMETERS for the M2.1 rigid motorcycle assembly.
##
## These numbers are NOT measured and NOT calibrated: they are plausible, motorcycle-proportioned values chosen so that the
## first assembly is a well-conditioned, runnable example (distinct masses, non-trivial inertias, a raked steering axis and a
## tilted swingarm so that the bodies do not share local frames). Do not read them as the specification of any real
## motorcycle, and do not tune the simulation against them. Every value can be changed in the returned MotorcycleConfig.
##
## Reference pose (all hinge angles zero), world frame, +Y up, local -Z forward, +X right, ground plane (not simulated) at y = 0:
##   * Everything lies in the symmetry plane x = 0. The motorcycle points toward -Z (front wheel at negative z).
##   * Steering axis: raked by RAKE from the vertical, leaning rearward toward the top: a = (0, cos RAKE, sin RAKE).
##     Positive steering-hinge angle = right-handed about +a = steering to the LEFT (front of the fork toward -X).
##   * Swingarm axis +X. Positive angle = right-handed about +X = the rear end of the swingarm moves DOWN.
##   * Wheel axles +X. Positive angle = right-handed about +X = the wheel top moves rearward, so rolling FORWARD (toward -Z)
##     is a NEGATIVE angle rate about the +X axle axis.
## Primitive approximations (full side lengths, uniform density; inertia about the COM in the body frame):
##   chassis    solid box 0.30 x 0.55 x 1.00 m (x, y, z)           -> SimMassProperties.box_inertia
##   steering   solid box 0.60 x 0.70 x 0.16 m in the fork frame   -> box_inertia (local +Y along the steering axis)
##   swingarm   solid box 0.20 x 0.10 x L m (L = pivot-to-axle)    -> box_inertia (local +Z along the swingarm)
##   wheels     hollow cylinder, outer radius R, inner radius WHEEL_INNER_FRACTION * R (a rim-and-tyre-like ring), width w
##                                                                 -> MotorcycleConfig.wheel_inertia (axle = local X)

const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const BodySpec = preload("res://Simulation/Motorcycle/motorcycle_body_spec.gd")
const JointSpec = preload("res://Simulation/Motorcycle/motorcycle_joint_spec.gd")
const SimMassProperties = preload("res://Simulation/Core/sim_mass_properties.gd")

## Steering-axis rake from the vertical (degrees).
const RAKE_DEGREES: float = 26.0
## Inner radius of the wheel ring as a fraction of the outer radius (mass concentrated toward the rim).
const WHEEL_INNER_FRACTION: float = 0.75


## A fresh, independent configuration each call (callers may mutate it).
static func config() -> MotorcycleConfig:
	var c: MotorcycleConfig = MotorcycleConfig.new()
	var rake: float = deg_to_rad(RAKE_DEGREES)

	# Steering geometry. `steer_axis` points up and rearward; `fork_forward` is perpendicular to it and points forward (-Z).
	var steer_axis: Vector3 = Vector3(0.0, cos(rake), sin(rake))
	var fork_forward: Vector3 = Vector3(0.0, sin(rake), -cos(rake))
	var head_point: Vector3 = Vector3(0.0, 0.88, -0.33)  # on the steering axis, at the chassis
	var fork_length: float = 0.65    # head point to the front axle measured along the steering axis
	var fork_offset: float = 0.04    # front axle displaced forward from the steering axis, perpendicular to it
	var fork_com_below_head: float = 0.30   # fork-assembly COM, measured down the axis from the head point
	var fork_com_offset: float = 0.02       # fork-assembly COM, forward of the axis
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
	c.steering = BodySpec.new("steering", 22.0, SimMassProperties.box_inertia(22.0, Vector3(0.60, 0.70, 0.16)),
		head_point - steer_axis * fork_com_below_head + fork_forward * fork_com_offset,
		Quaternion(Vector3.RIGHT, rake), Vector3(0.60, 0.70, 0.16))
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
	return c
