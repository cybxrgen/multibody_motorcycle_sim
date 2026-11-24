extends RefCounted
## Shared fixtures for the M2.4.2 tyre tests (not a test file: the runner only discovers test_*.gd).
##
## wheel_rig(): ONE prototype wheel (hollow cylinder, same mass and inertia as the motorcycle's wheel) on a static ground plane with a
## TireGroundContact, set up from tyre-axes quantities: forward and lateral speed along the wheel's own longitudinal/lateral axes, the
## spin as a multiple of the free-rolling spin, yaw (rotation about the plane normal) and camber (rotation about the forward axis).
## Nothing is tuned: tyre values are MotorcyclePrototype.tire_spec() unless overridden.
##
## carriage_motorcycle(): the assembled motorcycle with tyres plus a TEST-ONLY planar fixture (never part of MotorcycleBuilder output):
## a light carriage that may slide along world Z (forward/back) on a static anchor, and the chassis slides along world Y on the carriage.
## It leaves vertical and forward translation free (the weight is carried by the two tyre contacts and the tyre forces can accelerate or
## brake the whole machine) and removes lateral translation, yaw, pitch and roll of the chassis, which a riderless two-point-contact
## motorcycle cannot hold by itself. It also removes pitching, so load transfer is NOT represented in these fixtures.

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")
const SimMassProperties = preload("res://Simulation/Core/sim_mass_properties.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")
const TireGroundContact = preload("res://Simulation/Motorcycle/tire_ground_contact.gd")
const TireModel = preload("res://Simulation/Motorcycle/tire_model.gd")
const PrismaticJoint = preload("res://Simulation/Solver/prismatic_joint_constraint.gd")
const GroundRig = preload("res://Tests/ground_rig.gd")

const DT: float = 1.0 / 120.0
const G: float = 9.81


## Prototype tyre of the front (true) or rear wheel with some fields overridden: {"field_name": value}.
static func spec(front: bool, overrides: Dictionary = {}):
	var radius: float = GroundRig.wheel_params(front)[0]
	var s = MotorcyclePrototype.tire_spec(radius)
	for key in overrides:
		s.set(key, overrides[key])
	return s


## Wheel rig. Options (all optional):
##   front (false), friction (0.8: the surface coefficient), gravity_scale (1.0), spec_overrides ({}), spec (a ready spec, wins),
##   yaw (0 rad), camber (0 rad), height (0 m extra gap above touching), forward_speed (0 m/s along e_x), lateral_speed (0 m/s along
##   e_y = right), spin_ratio (1.0 = the free-rolling spin forward_speed / R_eff), spin (rad/s, overrides spin_ratio),
##   omega_extra (Vector3 added to the angular velocity), contact_script (a TireGroundContact subclass script used instead).
## Returns {world, ground, wheel, contact, radius, mass, spec, u, e_x, e_y}.
static func wheel_rig(opts: Dictionary = {}) -> Dictionary:
	var front: bool = opts.get("front", false)
	var p: Array = GroundRig.wheel_params(front)
	var radius: float = p[0]
	var mass: float = p[1]
	var w: SimWorld = SimWorld.new()
	w.config.gravity = Vector3(0.0, -G * opts.get("gravity_scale", 1.0), 0.0)
	var ground: SimRigidBody = SimRigidBody.new(SimMassProperties.new(0.0), Vector3.ZERO)
	var q: Quaternion = Quaternion(Vector3.UP, opts.get("yaw", 0.0)) * Quaternion(Vector3(0.0, 0.0, -1.0), opts.get("camber", 0.0))
	var inertia: Basis = MotorcycleConfig.wheel_inertia(mass, radius, GroundRig.WHEEL_INNER_FRACTION * radius, p[2])
	var wheel: SimRigidBody = SimRigidBody.new(SimMassProperties.new(mass, inertia), Vector3.ZERO, q)
	var origin_geometry: Dictionary = WheelGroundContact.compute_geometry(Vector3.ZERO, q, radius, Vector3.ZERO, Vector3.UP)
	wheel.position = Vector3(0.0, -origin_geometry.gap + opts.get("height", 0.0), 0.0)
	var tire_spec = opts.get("spec", spec(front, opts.get("spec_overrides", {})))
	var u: Vector3 = q * Vector3.RIGHT
	var tyre_axes: Dictionary = TireModel.axes(u, Vector3.UP)
	var e_x: Vector3 = tyre_axes.longitudinal
	var e_y: Vector3 = tyre_axes.lateral
	var vf: float = opts.get("forward_speed", 0.0)
	wheel.linear_velocity = e_x * vf + e_y * opts.get("lateral_speed", 0.0)
	var spin: float = opts.get("spin", opts.get("spin_ratio", 1.0) * vf / tire_spec.rolling_radius)
	wheel.angular_velocity = -u * spin + opts.get("omega_extra", Vector3.ZERO)
	w.add_body(ground)
	w.add_body(wheel)
	var friction: float = opts.get("friction", 0.8)
	var contact = null
	if opts.has("contact_script"):
		contact = make_contact(opts.contact_script, ground, wheel, radius, tire_spec, friction)
	else:
		contact = TireGroundContact.create_with_tire(ground, wheel, radius, Vector3.ZERO, Vector3.UP, friction, tire_spec)
	w.add_constraint(contact)
	return {"world": w, "ground": ground, "wheel": wheel, "contact": contact, "radius": radius, "mass": mass, "spec": tire_spec,
		"u": u, "e_x": e_x, "e_y": e_y}


## A contact of an arbitrary TireGroundContact subclass script (mutants), seated on the wheel's current pose like create_with_tire().
static func make_contact(script, ground: SimRigidBody, wheel: SimRigidBody, radius: float, tire_spec, friction: float):
	var geometry: Dictionary = WheelGroundContact.compute_geometry(wheel.position, wheel.orientation, radius, Vector3.ZERO, Vector3.UP)
	var inv_a: Quaternion = ground.orientation.inverse()
	var inv_b: Quaternion = wheel.orientation.inverse()
	return script.new(ground, wheel, inv_a * (geometry.ground_point - ground.position), inv_b * (geometry.wheel_point - wheel.position),
		inv_a * Vector3.UP, friction, radius, Vector3.ZERO, Vector3.UP, 0.05, tire_spec)


## External TEST INPUT: a torque about the axle of magnitude T that drives a forward-rolling wheel (the wheel turns about -u).
## Call it before every step() (the accumulators are cleared by each step). Negative T brakes a forward-rolling wheel.
static func drive_torque(wheel: SimRigidBody, u: Vector3, torque: float) -> void:
	wheel.apply_torque(-u * torque)


static func total_linear_momentum(bodies: Array) -> Vector3:
	var total: Vector3 = Vector3.ZERO
	for body in bodies:
		total += body.linear_momentum()
	return total


## Motorcycle with tyres on the prototype ground plus the planar TEST-ONLY fixture (see the file header). Returns
## {world, assembly, anchor, carriage, track, slide} or an empty dictionary when the build failed.
static func carriage_motorcycle(config: MotorcycleConfig = null, world: SimWorld = null) -> Dictionary:
	var w: SimWorld = world if world != null else SimWorld.new()
	var cfg: MotorcycleConfig = config if config != null else MotorcyclePrototype.config_with_tires()
	var assembly = MotorcycleBuilder.build(cfg, w)
	if assembly == null:
		return {}
	var origin: Vector3 = assembly.chassis.position
	var anchor: SimRigidBody = SimRigidBody.new(SimMassProperties.new(0.0), origin)
	var carriage: SimRigidBody = SimRigidBody.new(SimMassProperties.new(1.0, SimMassProperties.sphere_inertia(1.0, 0.1)), origin)
	w.add_body(anchor)
	w.add_body(carriage)
	var track = PrismaticJoint.from_world(anchor, carriage, origin, Vector3.BACK, Vector3.RIGHT)
	var slide = PrismaticJoint.from_world(carriage, assembly.chassis, origin, Vector3.UP, Vector3.RIGHT)
	w.add_constraint(track)
	w.add_constraint(slide)
	return {"world": w, "assembly": assembly, "anchor": anchor, "carriage": carriage, "track": track, "slide": slide}
