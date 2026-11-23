extends RefCounted
## Shared test fixtures for the M2.4.1 wheel-ground contact tests (not a test file: the runner only discovers test_*.gd).
##
## TEST-ONLY roll fixture. A riderless motorcycle on two narrow point contacts is not stable in roll, and nothing in the production
## assembly may fake it (no stand, no stabilizer). Vertical-support tests therefore attach the chassis to a static body with a
## generic PrismaticJointConstraint along world Y: horizontal translation, yaw, pitch and roll of the chassis are removed, vertical
## translation stays free, so the whole weight must be carried by the two wheel contacts. The fixture lives only in the world
## of the test that builds it; it is never part of MotorcycleBuilder output.

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")
const SimMassProperties = preload("res://Simulation/Core/sim_mass_properties.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const PrismaticJoint = preload("res://Simulation/Solver/prismatic_joint_constraint.gd")

const WHEEL_INNER_FRACTION: float = 0.75
const DT: float = 1.0 / 120.0


## Thin-rim wheel parameters used by the single-wheel tests: [radius, mass, width] for the front and the rear prototype wheel.
static func wheel_params(front: bool) -> Array:
	return [0.31, 12.0, 0.12] if front else [0.32, 15.0, 0.18]


## k = I_axle / (m R^2) of the hollow-cylinder wheel (inner radius 0.75 R): 0.5 (1 + 0.75^2).
static func axle_inertia_factor() -> float:
	return 0.5 * (1.0 + WHEEL_INNER_FRACTION * WHEEL_INNER_FRACTION)


## World with a static ground at the origin (plane y = 0, normal +Y) and one wheel (axle = local X) whose centre is at height
## `center_y` (default R: touching). Returns {world, ground, wheel, contact, radius, mass}.
static func wheel_rig(front: bool, friction: float, center_y: float = -1.0, gravity: Vector3 = Vector3(0.0, -9.81, 0.0),
		plane_offset: float = 0.0) -> Dictionary:
	var p: Array = wheel_params(front)
	var radius: float = p[0]
	var mass: float = p[1]
	var w: SimWorld = SimWorld.new()
	w.config.gravity = gravity
	var ground: SimRigidBody = SimRigidBody.new(SimMassProperties.new(0.0), Vector3(0.0, plane_offset, 0.0))
	var inertia: Basis = MotorcycleConfig.wheel_inertia(mass, radius, WHEEL_INNER_FRACTION * radius, p[2])
	var wheel: SimRigidBody = SimRigidBody.new(SimMassProperties.new(mass, inertia), Vector3(0.0, plane_offset + (radius if center_y < 0.0 else center_y), 0.0))
	w.add_body(ground)
	w.add_body(wheel)
	var contact = WheelGroundContact.create(ground, wheel, radius, Vector3(0.0, plane_offset, 0.0), Vector3.UP, friction)
	w.add_constraint(contact)
	return {"world": w, "ground": ground, "wheel": wheel, "contact": contact, "radius": radius, "mass": mass}


## Motorcycle on the prototype ground plus the test-only vertical-slide roll fixture (see the file header). Returns
## {world, assembly, fixture, anchor} or an empty dictionary when the build failed.
static func fixtured_motorcycle(world: SimWorld = null) -> Dictionary:
	var w: SimWorld = world if world != null else SimWorld.new()
	var assembly = MotorcycleBuilder.build(MotorcyclePrototype.config_with_ground(), w)
	if assembly == null:
		return {}
	var anchor: SimRigidBody = SimRigidBody.new(SimMassProperties.new(0.0), assembly.chassis.position)
	w.add_body(anchor)
	var fixture = PrismaticJoint.from_world(anchor, assembly.chassis, assembly.chassis.position, Vector3.UP, Vector3.RIGHT)
	w.add_constraint(fixture)
	return {"world": w, "assembly": assembly, "fixture": fixture, "anchor": anchor}
