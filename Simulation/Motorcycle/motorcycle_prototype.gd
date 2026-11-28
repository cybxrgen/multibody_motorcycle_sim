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
const TireSpec = preload("res://Simulation/Motorcycle/motorcycle_tire_spec.gd")
const SimMassProperties = preload("res://Simulation/Core/sim_mass_properties.gd")
const CollisionBox = preload("res://Simulation/Motorcycle/motorcycle_collision_box.gd")
const ControlSpec = preload("res://Simulation/Motorcycle/motorcycle_control_spec.gd")
const TireGeometry = preload("res://Simulation/Motorcycle/tire_geometry.gd")
const TirePatchSpec = preload("res://Simulation/Motorcycle/motorcycle_tire_patch_spec.gd")

## ---- Estimated mass distribution (M2.5.2; ESTIMATES, not measured data, not a calibrated or real motorcycle) ----
## Category assumed: a middleweight (about 600 cc class) standard / naked road motorcycle, WITHOUT rider, luggage and fuel but with oil and
## coolant (about 185 kg). Each body carries only its own part and the rider is not included (rider mass belongs to a later increment):
##   chassis      128 kg  frame, engine and gearbox, fluids, exhaust, radiator, tank shell, seat, bodywork, battery and electrics. The M2.1
##                        value of 120 kg was raised by 8 kg because the engine, exhaust and fluids sit on this body and a 600 cc-class
##                        engine alone is of the order of 55-65 kg.
##   steering     14 kg   steering stem, triple clamps, handlebar with controls, headlight and instruments, upper fork tubes (unchanged)
##   fork_slider   8 kg   lower fork legs, axle carrier, front brake calipers, front mudguard (unchanged; steering + slider stay the 22 kg fork)
##   swingarm      9 kg   swingarm, rear brake caliper and carrier, chain guard, share of the linkage (unchanged)
##   front_wheel  12 kg   rim, tyre, brake discs, hub (unchanged)
##   rear_wheel   15 kg   rim, tyre, brake disc, rear sprocket and hub (unchanged)
## Total 186 kg (the M2.1 illustrative total was 178 kg; the +8 kg is the chassis revision above and nothing else). All six masses are
## independent specification fields: changing one never changes another (Tests/test_collision_mass.gd).
const MASS_CHASSIS: float = 128.0
const MASS_STEERING: float = 14.0
const MASS_FORK_SLIDER: float = 8.0
const MASS_SWINGARM: float = 9.0
const MASS_FRONT_WHEEL: float = 12.0
const MASS_REAR_WHEEL: float = 15.0

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


## ILLUSTRATIVE initial tyre parameters of the prototype (M2.4.2): simulation fixture values of a deliberately simple model, NOT measured
## or fitted tyre data, and not tuned against any result. The rolling radius equals the wheel radius (no loaded-radius reduction).
const TIRE_LONGITUDINAL_STIFFNESS: float = 15.0
const TIRE_LATERAL_STIFFNESS: float = 10.0
const TIRE_GRIP_LONGITUDINAL: float = 1.0
const TIRE_GRIP_LATERAL: float = 1.0
const TIRE_ROLLING_RESISTANCE: float = 0.015
const TIRE_ROLLING_RESISTANCE_SPEED: float = 0.2
const TIRE_LOW_SPEED_REFERENCE: float = 0.5


## One prototype tyre for a wheel of the given radius (a fresh object each call).
static func tire_spec(wheel_radius: float) -> TireSpec:
	var t: TireSpec = TireSpec.new()
	t.rolling_radius = wheel_radius
	t.longitudinal_stiffness = TIRE_LONGITUDINAL_STIFFNESS
	t.lateral_stiffness = TIRE_LATERAL_STIFFNESS
	t.grip_longitudinal = TIRE_GRIP_LONGITUDINAL
	t.grip_lateral = TIRE_GRIP_LATERAL
	t.rolling_resistance = TIRE_ROLLING_RESISTANCE
	t.rolling_resistance_speed = TIRE_ROLLING_RESISTANCE_SPEED
	t.low_speed_reference = TIRE_LOW_SPEED_REFERENCE
	return t


## The ground configuration plus independent front and rear tyres (M2.4.2). A fresh configuration each call.
static func config_with_tires() -> MotorcycleConfig:
	var c: MotorcycleConfig = config_with_ground()
	c.front_tire = tire_spec(c.front_wheel_radius)
	c.rear_tire = tire_spec(c.rear_wheel_radius)
	return c


## ---- Physical collision boxes and basic controls (M2.5.2, illustrative, NOT measured) ----
## Each structural box is centred on its body's COM frame with the SAME full dimensions that produce the body's inertia tensor
## (SimMassProperties.box_inertia), so geometry, mass and inertia describe one solid box (uniform density approximation of the part).
## Surface friction of every box against the ground: 0.6 (a hard plastic / metal part on asphalt, illustrative).
const BOX_FRICTION: float = 0.6
## Illustrative control limits: 150 N m drive torque on the rear axle, 250 / 150 N m front / rear brake torque, steering motor 1.0 rad/s
## with a 40 N m cap, steering limit +-0.5 rad (about 29 degrees).
const MAX_DRIVE_TORQUE: float = 150.0
const MAX_FRONT_BRAKE_TORQUE: float = 250.0
const MAX_REAR_BRAKE_TORQUE: float = 150.0
const MAX_STEERING_RATE: float = 1.0
const MAX_STEERING_TORQUE: float = 40.0
const STEERING_LIMIT: float = 0.5


## The tyre configuration (ground, tyres) plus the four physical collision boxes and the basic controls (M2.5.2). The collision boxes
## take their dimensions from the body proxies that define the inertia boxes. A fresh configuration each call.
static func config_with_collision() -> MotorcycleConfig:
	var c: MotorcycleConfig = config_with_tires()
	c.chassis_box = CollisionBox.new("chassis", c.chassis.visual_size, Vector3.ZERO, Quaternion.IDENTITY, BOX_FRICTION)
	c.steering_box = CollisionBox.new("steering", c.steering.visual_size, Vector3.ZERO, Quaternion.IDENTITY, BOX_FRICTION)
	c.fork_slider_box = CollisionBox.new("fork_slider", c.fork_slider.visual_size, Vector3.ZERO, Quaternion.IDENTITY, BOX_FRICTION)
	c.swingarm_box = CollisionBox.new("swingarm", c.swingarm.visual_size, Vector3.ZERO, Quaternion.IDENTITY, BOX_FRICTION)
	c.controls = ControlSpec.new(MAX_DRIVE_TORQUE, MAX_FRONT_BRAKE_TORQUE, MAX_REAR_BRAKE_TORQUE, MAX_STEERING_RATE, MAX_STEERING_TORQUE, STEERING_LIMIT)
	return c


## ---- Distributed contact patches (M2.5.3 Stage D, illustrative, NOT measured) ----
## Tread sections (full width, crown radius) of the two prototype tyres: 120 mm / 70 mm front, 180 mm / 100 mm rear. Grid and compliance of the patch:
## 9 x 48 samples, k_A = 4e6 N/m^3, c_A = 1e5 N s/m^3, bottoming at 50 mm; the values the Stage B/C tests use, not tuned against any result.
const FRONT_TIRE_WIDTH: float = 0.12
const FRONT_TIRE_CROWN_RADIUS: float = 0.07
const REAR_TIRE_WIDTH: float = 0.18
const REAR_TIRE_CROWN_RADIUS: float = 0.10
const PATCH_LATERAL_COUNT: int = 9
const PATCH_CIRCUMFERENTIAL_COUNT: int = 48
const PATCH_STIFFNESS_DENSITY: float = 4.0e6
const PATCH_DAMPING_DENSITY: float = 1.0e5
const PATCH_MAX_COMPRESSION: float = 0.05


## One prototype patch specification (a fresh object each call).
static func patch_spec() -> TirePatchSpec:
	return TirePatchSpec.new(PATCH_LATERAL_COUNT, PATCH_CIRCUMFERENTIAL_COUNT, PATCH_STIFFNESS_DENSITY, PATCH_DAMPING_DENSITY, PATCH_MAX_COMPRESSION)


## The collision configuration with the finite-width tyre geometry and ONE distributed contact patch per wheel (M2.5.3 Stage D): the wheels carry
## TirePatchNormalSupport instead of the one-point contact. A fresh configuration each call.
static func config_with_patches() -> MotorcycleConfig:
	var c: MotorcycleConfig = config_with_collision()
	c.front_tire_geometry = TireGeometry.new(FRONT_TIRE_WIDTH, FRONT_TIRE_CROWN_RADIUS)
	c.rear_tire_geometry = TireGeometry.new(REAR_TIRE_WIDTH, REAR_TIRE_CROWN_RADIUS)
	c.front_tire_patch = patch_spec()
	c.rear_tire_patch = patch_spec()
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
	c.chassis = BodySpec.new("chassis", MASS_CHASSIS, SimMassProperties.box_inertia(MASS_CHASSIS, Vector3(0.30, 0.55, 1.00)),
		Vector3(0.0, 0.62, 0.06), Quaternion.IDENTITY, Vector3(0.30, 0.55, 1.00))
	# The fork frame is the world frame rotated about +X by the rake: local +Y = steering axis, local -Z = fork_forward.
	c.steering = BodySpec.new("steering", MASS_STEERING, SimMassProperties.box_inertia(MASS_STEERING, Vector3(0.60, 0.30, 0.16)),
		head_point - steer_axis * steering_com_below_head + fork_forward * steering_com_offset,
		Quaternion(Vector3.RIGHT, rake), Vector3(0.60, 0.30, 0.16))
	c.fork_slider = BodySpec.new("fork_slider", MASS_FORK_SLIDER, SimMassProperties.box_inertia(MASS_FORK_SLIDER, Vector3(0.24, 0.40, 0.14)),
		head_point - steer_axis * slider_com_below_head + fork_forward * slider_com_offset,
		Quaternion(Vector3.RIGHT, rake), Vector3(0.24, 0.40, 0.14))
	c.swingarm = BodySpec.new("swingarm", MASS_SWINGARM, SimMassProperties.box_inertia(MASS_SWINGARM, Vector3(0.20, 0.10, arm_length)),
		(pivot_point + rear_axle_point) * 0.5, Quaternion(Vector3.RIGHT, tilt), Vector3(0.20, 0.10, arm_length))
	c.front_wheel = BodySpec.new("front_wheel", MASS_FRONT_WHEEL,
		MotorcycleConfig.wheel_inertia(MASS_FRONT_WHEEL, c.front_wheel_radius, WHEEL_INNER_FRACTION * c.front_wheel_radius, c.front_wheel_width),
		front_axle_point, Quaternion.IDENTITY)
	c.rear_wheel = BodySpec.new("rear_wheel", MASS_REAR_WHEEL,
		MotorcycleConfig.wheel_inertia(MASS_REAR_WHEEL, c.rear_wheel_radius, WHEEL_INNER_FRACTION * c.rear_wheel_radius, c.rear_wheel_width),
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
