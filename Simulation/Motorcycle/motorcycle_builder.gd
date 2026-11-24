extends RefCounted
## Builds the motorcycle assembly from a MotorcycleConfig: six independent dynamic rigid bodies (chassis, steering assembly,
## fork slider, swingarm, front wheel, rear wheel) registered in a SimWorld, four generic HingeJointConstraints, one generic
## PrismaticJointConstraint (steering assembly <-> fork slider, along the fork axis), the front spring-damper and the rear
## spring-damper (chassis <-> swingarm, a direct force pair, no extra body or joint), both registered as generic force providers. Static functions only; scene-tree independent; no motorcycle-specific behaviour exists in
## Simulation/Core or Simulation/Solver.
##
## Joint frames are NOT typed in per body. Each hinge is created with HingeJointConstraint.from_world(a, b, point, axis,
## reference): the world pivot, world axis and ONE world reference direction (perpendicular to the axis) are converted into
## each body's own local frame using that body's orientation. The two local axes and the two local reference directions are
## therefore generally different vectors (the fork is rotated by the rake angle, the swingarm by its tilt), but the world
## frames coincide, the hinge starts with zero anchor error, zero axis error and zero angle. The prismatic fork joint is built the
## same way (PrismaticJointConstraint.from_world with the fork axis and a world reference direction): zero lateral error, zero
## axis error, zero twist and translation 0 at the reference pose. The suspension seats are converted from world points too.
##
## Failure policy (identical to SimWorld.add_body / add_constraint / add_force_provider / the from_world factories): build() returns
## null and reports the reason with push_error. The complete configuration is validated before anything is registered. Nothing
## partial survives: if anything fails after the first registration, exactly the bodies, constraints and force provider this call
## added are removed from the world again. A world passed in keeps all earlier content.
## Body ids of removed bodies are not reused (SimWorld's id counter only grows).

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")
const SimMassProperties = preload("res://Simulation/Core/sim_mass_properties.gd")
const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")
const PrismaticJoint = preload("res://Simulation/Solver/prismatic_joint_constraint.gd")
const FrontSuspension = preload("res://Simulation/Motorcycle/front_suspension.gd")
const RearSuspension = preload("res://Simulation/Motorcycle/rear_suspension.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const MotorcycleAssembly = preload("res://Simulation/Motorcycle/motorcycle_assembly.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")
const TireGroundContact = preload("res://Simulation/Motorcycle/tire_ground_contact.gd")

## Construction self-check: world anchors of a hinge must agree within this distance. Pivot coordinates of order 1 m carry
## float32 rounding of ~1e-7 m per operation (local = R^-1 (P - x), world = x + R local: a few operations), so honest
## input yields ~2e-7 m; 1e-5 m (10 micrometres) is ~50x that and still far below any physical clearance. A configuration
## placed so far from the origin that float32 cannot represent the pivot to this accuracy is refused instead.
const ANCHOR_TOLERANCE: float = 1.0e-5
## Construction self-check: the two world axes of a hinge must agree within this angle (rad). Direction errors are float32
## rotations of unit vectors (~1e-7); 1e-5 rad leaves ~100x margin.
const AXIS_TOLERANCE: float = 1.0e-5
## Construction self-check of the fork joint: lateral offset and twist/axis errors are float32 rotations of metre-sized
## vectors, bounded like the hinge anchors/axes; the translation coordinate must be zero at the reference pose.
const FORK_LATERAL_TOLERANCE: float = ANCHOR_TOLERANCE


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
	var added_providers: Array = []
	var assembly: MotorcycleAssembly = MotorcycleAssembly.new()
	assembly.world = target
	assembly.config = config

	# 1. Bodies: valid mass properties first, then registration. Any failure rolls everything back.
	var bodies: Array = []
	for spec in config.body_specs():
		var mass_properties: SimMassProperties = SimMassProperties.new(spec.mass, spec.local_inertia)
		if not mass_properties.valid or mass_properties.is_static():
			return _fail(target, added_bodies, added_constraints, added_providers, "body '%s' has invalid mass properties: %s" % [spec.body_name, mass_properties.error_message])
		var body: SimRigidBody = SimRigidBody.new(mass_properties, spec.position, spec.orientation)
		body.body_name = spec.body_name
		if target.add_body(body) == null:
			return _fail(target, added_bodies, added_constraints, added_providers, "SimWorld refused body '%s'" % spec.body_name)
		added_bodies.append(body)
		bodies.append(body)
	assembly.chassis = bodies[0]
	assembly.steering = bodies[1]
	assembly.fork_slider = bodies[2]
	assembly.swingarm = bodies[3]
	assembly.front_wheel = bodies[4]
	assembly.rear_wheel = bodies[5]

	# 2. Hinges (body A, body B, spec), registered in the same world as their bodies. The front axle hangs on the fork slider.
	var graph: Array = [
		[assembly.chassis, assembly.steering, config.steering_head],
		[assembly.chassis, assembly.swingarm, config.swingarm_pivot],
		[assembly.fork_slider, assembly.front_wheel, config.front_axle],
		[assembly.swingarm, assembly.rear_wheel, config.rear_axle],
	]
	var joints: Array = []
	for link in graph:
		var spec = link[2]
		var joint = HingeJoint.from_world(link[0], link[1], spec.point, spec.axis, hinge_reference(spec.axis))
		if joint == null:
			return _fail(target, added_bodies, added_constraints, added_providers, "hinge '%s' could not be created from the configured pivot and axis" % spec.joint_name)
		if target.add_constraint(joint) == null:
			return _fail(target, added_bodies, added_constraints, added_providers, "SimWorld refused hinge '%s'" % spec.joint_name)
		added_constraints.append(joint)
		joints.append(joint)
	assembly.steering_joint = joints[0]
	assembly.swingarm_joint = joints[1]
	assembly.front_axle_joint = joints[2]
	assembly.rear_axle_joint = joints[3]

	# 3. Fork: prismatic joint steering <-> fork slider along the fork axis (positive = extension), then the spring-damper.
	var fork_spec = config.fork_travel
	var fork = PrismaticJoint.from_world(assembly.steering, assembly.fork_slider, fork_spec.point, fork_spec.axis, hinge_reference(fork_spec.axis))
	if fork == null:
		return _fail(target, added_bodies, added_constraints, added_providers, "prismatic joint '%s' could not be created from the configured point and axis" % fork_spec.joint_name)
	var suspension_spec = config.front_suspension
	fork.limit_enabled = suspension_spec.travel_limits_enabled
	fork.lower_limit = suspension_spec.lower_travel_limit
	fork.upper_limit = suspension_spec.upper_travel_limit
	if not fork.is_valid():
		return _fail(target, added_bodies, added_constraints, added_providers, "prismatic joint '%s' rejected its travel limits: %s" % [fork_spec.joint_name, "; ".join(fork.validate())])
	if target.add_constraint(fork) == null:
		return _fail(target, added_bodies, added_constraints, added_providers, "SimWorld refused prismatic joint '%s'" % fork_spec.joint_name)
	added_constraints.append(fork)
	assembly.fork_joint = fork
	var suspension = FrontSuspension.from_world(fork, suspension_spec.upper_attachment, suspension_spec.lower_attachment,
		suspension_spec.stiffness, suspension_spec.damping, suspension_spec.rest_displacement)
	if suspension == null:
		return _fail(target, added_bodies, added_constraints, added_providers, "front suspension could not be created from the configured seats and parameters")
	if target.add_force_provider(suspension) == null:
		return _fail(target, added_bodies, added_constraints, added_providers, "SimWorld refused the front suspension")
	added_providers.append(suspension)
	assembly.front_suspension = suspension

	# 3b. Rear spring-damper: a force pair between chassis and swingarm at the two attachment points (no body, no joint).
	var rear_spec = config.rear_suspension
	var rear_suspension = RearSuspension.from_world(assembly.chassis, assembly.swingarm, rear_spec.chassis_attachment, rear_spec.swingarm_attachment,
		rear_spec.stiffness, rear_spec.damping, rear_spec.rest_length)
	if rear_suspension == null:
		return _fail(target, added_bodies, added_constraints, added_providers, "rear suspension could not be created from the configured attachments and parameters")
	if target.add_force_provider(rear_suspension) == null:
		return _fail(target, added_bodies, added_constraints, added_providers, "SimWorld refused the rear suspension")
	added_providers.append(rear_suspension)
	assembly.rear_suspension = rear_suspension

	# 3c. Optional ground (M2.4.1): one static body and one WheelGroundContact per wheel (ground = body A, wheel = body B).
	if config.ground != null:
		var ground_spec = config.ground
		var ground_body: SimRigidBody = SimRigidBody.new(SimMassProperties.new(0.0), ground_spec.point, Quaternion.IDENTITY)
		ground_body.body_name = "ground"
		if target.add_body(ground_body) == null:
			return _fail(target, added_bodies, added_constraints, added_providers, "SimWorld refused the ground body")
		added_bodies.append(ground_body)
		assembly.ground = ground_body
		var wheel_links: Array = [[assembly.front_wheel, config.front_wheel_radius, "front", config.front_tire], [assembly.rear_wheel, config.rear_wheel_radius, "rear", config.rear_tire]]
		var contacts: Array = []
		for link in wheel_links:
			# M2.4.2: a wheel with a tyre gets a TireGroundContact (the tyre model owns the tangential response; the tyre is copied so
			# later edits of the configuration do not reach the simulation), otherwise the generic M2.4.1 WheelGroundContact.
			var contact = null
			if link[3] != null:
				contact = TireGroundContact.create_with_tire(ground_body, link[0], link[1], ground_spec.point, ground_spec.normal, ground_spec.friction, link[3].duplicate_spec(), ground_spec.activation_distance)
			else:
				contact = WheelGroundContact.create(ground_body, link[0], link[1], ground_spec.point, ground_spec.normal, ground_spec.friction, ground_spec.activation_distance)
			if contact == null:
				return _fail(target, added_bodies, added_constraints, added_providers, "the %s wheel ground contact could not be created" % link[2])
			if target.add_constraint(contact) == null:
				return _fail(target, added_bodies, added_constraints, added_providers, "SimWorld refused the %s wheel ground contact" % link[2])
			added_constraints.append(contact)
			contacts.append(contact)
		assembly.front_wheel_contact = contacts[0]
		assembly.rear_wheel_contact = contacts[1]
		if config.front_tire != null:
			assembly.front_tire = contacts[0]
		if config.rear_tire != null:
			assembly.rear_tire = contacts[1]

	# 3d. Finite-width tyre geometry (M2.5.1): validated with the configuration above; stored as independent copies, not yet used by the contacts.
	if config.front_tire_geometry != null:
		assembly.front_tire_geometry = config.front_tire_geometry.duplicate_spec()
	if config.rear_tire_geometry != null:
		assembly.rear_tire_geometry = config.rear_tire_geometry.duplicate_spec()

	# 4. Self-check of the construction contract (coincident anchors, aligned axes).
	var contract_error: String = construction_error(assembly)
	if contract_error != "":
		return _fail(target, added_bodies, added_constraints, added_providers, contract_error)
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
	if assembly.fork_joint != null:
		var fork_error: Vector3 = assembly.fork_error()
		if not (fork_error.x <= FORK_LATERAL_TOLERANCE):
			return "the fork joint has a lateral offset at construction (%s m > %s m)" % [fork_error.x, FORK_LATERAL_TOLERANCE]
		if not (fork_error.y <= AXIS_TOLERANCE and fork_error.z <= AXIS_TOLERANCE):
			return "the fork joint axes/twist are not aligned at construction (axis %s rad, twist %s rad > %s rad)" % [fork_error.y, fork_error.z, AXIS_TOLERANCE]
		if not (absf(assembly.fork_joint.translation()) <= FORK_LATERAL_TOLERANCE):
			return "the fork joint is not at translation 0 at construction (%s m)" % assembly.fork_joint.translation()
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


static func _fail(world: SimWorld, added_bodies: Array, added_constraints: Array, added_providers: Array, reason: String):
	for provider in added_providers:
		world.remove_force_provider(provider)
	for constraint in added_constraints:
		world.remove_constraint(constraint)
	for body in added_bodies:
		world.remove_body(body)
	push_error("MotorcycleBuilder.build: %s; the world was restored (no partial assembly)." % reason)
	return null
