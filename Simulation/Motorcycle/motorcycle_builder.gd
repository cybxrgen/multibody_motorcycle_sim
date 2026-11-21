extends RefCounted
## Builds the rigid motorcycle assembly (M2.1) from a MotorcycleConfig: five independent dynamic rigid bodies registered in a
## SimWorld and four instances of the existing generic HingeJointConstraint. Static functions only; scene-tree independent;
## no motorcycle-specific behaviour exists in Simulation/Core or Simulation/Solver.
##
## Joint frames are NOT typed in per body. Each hinge is created with HingeJointConstraint.from_world(a, b, point, axis,
## reference): the world pivot, world axis and ONE world reference direction (perpendicular to the axis) are converted into
## each body's own local frame using that body's orientation. The two local axes and the two local reference directions are
## therefore generally different vectors (the fork is rotated by the rake angle, the swingarm by its tilt), but the world
## frames coincide, the hinge starts with zero anchor error, zero axis error and zero angle.
##
## Failure policy (identical to SimWorld.add_body / add_constraint / HingeJointConstraint.from_world): build() returns null
## and reports the reason with push_error. Nothing partial survives: if anything fails after the first registration, exactly
## the bodies and constraints this call added are removed from the world again. A world passed in keeps all earlier content.
## Body ids of removed bodies are not reused (SimWorld's id counter only grows).

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")
const SimMassProperties = preload("res://Simulation/Core/sim_mass_properties.gd")
const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const MotorcycleAssembly = preload("res://Simulation/Motorcycle/motorcycle_assembly.gd")

## Construction self-check: world anchors of a hinge must agree within this distance. Pivot coordinates of order 1 m carry
## float32 rounding of ~1e-7 m per operation (local = R^-1 (P - x), world = x + R local: a few operations), so honest
## input yields ~2e-7 m; 1e-5 m (10 micrometres) is ~50x that and still far below any physical clearance. A configuration
## placed so far from the origin that float32 cannot represent the pivot to this accuracy is refused instead.
const ANCHOR_TOLERANCE: float = 1.0e-5
## Construction self-check: the two world axes of a hinge must agree within this angle (rad). Direction errors are float32
## rotations of unit vectors (~1e-7); 1e-5 rad leaves ~100x margin.
const AXIS_TOLERANCE: float = 1.0e-5


## Builds the assembly. `world` is optional: with null a new SimWorld (default SimConfig, gravity included) is created; pass
## your own world to choose its SimConfig or to add the motorcycle to existing content. Returns null on any failure.
static func build(config: MotorcycleConfig, world: SimWorld = null) -> MotorcycleAssembly:
	if config == null:
		push_error("MotorcycleBuilder.build: configuration is null.")
		return null
	var errors: PackedStringArray = config.validate()
	if not errors.is_empty():
		push_error("MotorcycleBuilder.build: invalid configuration: %s" % "; ".join(errors))
		return null
	var target: SimWorld = world if world != null else SimWorld.new()

	var added_bodies: Array = []
	var added_constraints: Array = []
	var assembly: MotorcycleAssembly = MotorcycleAssembly.new()
	assembly.world = target
	assembly.config = config

	# 1. Bodies: valid mass properties first, then registration. Any failure rolls everything back.
	var bodies: Array = []
	for spec in config.body_specs():
		var mass_properties: SimMassProperties = SimMassProperties.new(spec.mass, spec.local_inertia)
		if not mass_properties.valid or mass_properties.is_static():
			return _fail(target, added_bodies, added_constraints, "body '%s' has invalid mass properties: %s" % [spec.body_name, mass_properties.error_message])
		var body: SimRigidBody = SimRigidBody.new(mass_properties, spec.position, spec.orientation)
		body.body_name = spec.body_name
		if target.add_body(body) == null:
			return _fail(target, added_bodies, added_constraints, "SimWorld refused body '%s'" % spec.body_name)
		added_bodies.append(body)
		bodies.append(body)
	assembly.chassis = bodies[0]
	assembly.steering = bodies[1]
	assembly.swingarm = bodies[2]
	assembly.front_wheel = bodies[3]
	assembly.rear_wheel = bodies[4]

	# 2. Hinges (body A, body B, spec), registered in the same world as their bodies.
	var graph: Array = [
		[assembly.chassis, assembly.steering, config.steering_head],
		[assembly.chassis, assembly.swingarm, config.swingarm_pivot],
		[assembly.steering, assembly.front_wheel, config.front_axle],
		[assembly.swingarm, assembly.rear_wheel, config.rear_axle],
	]
	var joints: Array = []
	for link in graph:
		var spec = link[2]
		var joint = HingeJoint.from_world(link[0], link[1], spec.point, spec.axis, hinge_reference(spec.axis))
		if joint == null:
			return _fail(target, added_bodies, added_constraints, "hinge '%s' could not be created from the configured pivot and axis" % spec.joint_name)
		if target.add_constraint(joint) == null:
			return _fail(target, added_bodies, added_constraints, "SimWorld refused hinge '%s'" % spec.joint_name)
		added_constraints.append(joint)
		joints.append(joint)
	assembly.steering_joint = joints[0]
	assembly.swingarm_joint = joints[1]
	assembly.front_axle_joint = joints[2]
	assembly.rear_axle_joint = joints[3]

	# 3. Self-check of the construction contract (coincident anchors, aligned axes).
	var contract_error: String = construction_error(assembly)
	if contract_error != "":
		return _fail(target, added_bodies, added_constraints, contract_error)
	return assembly


## "" if every hinge of `assembly` currently has coincident world anchors (within ANCHOR_TOLERANCE) and aligned world axes
## (within AXIS_TOLERANCE), otherwise a description. build() applies it to the finished assembly; it is public so the check
## itself can be tested (honest input cannot trip it: float32 pivot coordinates are self-consistent even far from the origin).
## It describes the current pose, so it only means "as constructed" before the first step.
static func construction_error(assembly: MotorcycleAssembly) -> String:
	var anchor_error: float = assembly.max_anchor_error()
	if not (anchor_error <= ANCHOR_TOLERANCE):
		return "hinge anchors do not coincide at construction (max error %s m > %s m)" % [anchor_error, ANCHOR_TOLERANCE]
	var axis_error: float = assembly.max_axis_error()
	if not (axis_error <= AXIS_TOLERANCE):
		return "hinge axes are not aligned at construction (max error %s rad > %s rad)" % [axis_error, AXIS_TOLERANCE]
	return ""


## Deterministic world-space reference direction perpendicular to `axis` (the hinge's zero-angle direction): the world basis
## vector least aligned with the axis (X before Y before Z on ties), projected onto the plane perpendicular to the axis and
## normalized. Never singular because the least-aligned basis vector always has |axis . e| <= 1/sqrt(3).
## The same world vector is given to both bodies of a hinge; from_world converts it into each body's local frame.
static func hinge_reference(axis: Vector3) -> Vector3:
	var best: Vector3 = Vector3.RIGHT
	var best_dot: float = absf(axis.x)
	if absf(axis.y) < best_dot:
		best = Vector3.UP
		best_dot = absf(axis.y)
	if absf(axis.z) < best_dot:
		best = Vector3.BACK
	return (best - axis * axis.dot(best)).normalized()


static func _fail(world: SimWorld, added_bodies: Array, added_constraints: Array, reason: String):
	for constraint in added_constraints:
		world.remove_constraint(constraint)
	for body in added_bodies:
		world.remove_body(body)
	push_error("MotorcycleBuilder.build: %s; the world was restored (no partial assembly)." % reason)
	return null
