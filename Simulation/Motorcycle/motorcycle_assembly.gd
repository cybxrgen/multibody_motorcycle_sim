extends RefCounted
## Result of MotorcycleBuilder.build(): the world and NAMED references to the six bodies, the four hinges, the fork's prismatic
## joint, the front suspension and the rear suspension, so tests and presentation never index arrays or rely on body ids. A successfully built
## assembly is complete: it is only ever constructed with all six bodies, all five joints and both suspensions registered in `world`.
## When the configuration has a ground (M2.4.1) the static ground body and both wheel contacts are registered too.
## Owns nothing physical: the bodies, constraints and the force provider belong to `world` (SimWorld owns all physical state).

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")
const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")
const PrismaticJoint = preload("res://Simulation/Solver/prismatic_joint_constraint.gd")
const FrontSuspension = preload("res://Simulation/Motorcycle/front_suspension.gd")
const RearSuspension = preload("res://Simulation/Motorcycle/rear_suspension.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")
const TireGroundContact = preload("res://Simulation/Motorcycle/tire_ground_contact.gd")

var world: SimWorld
## The configuration the assembly was built from (kept for geometry queries; not re-read by the simulation).
var config

var chassis: SimRigidBody
## Steering stem and upper fork structure (chassis side of the fork).
var steering: SimRigidBody
## Lower fork structure and front-wheel carrier; translates along the fork axis relative to `steering`.
var fork_slider: SimRigidBody
var swingarm: SimRigidBody
var front_wheel: SimRigidBody
var rear_wheel: SimRigidBody

## chassis <-> steering, about the steering axis.
var steering_joint: HingeJoint
## chassis <-> swingarm, about the swingarm pivot axis.
var swingarm_joint: HingeJoint
## fork slider <-> front wheel, about the front axle.
var front_axle_joint: HingeJoint
## swingarm <-> rear wheel, about the rear axle.
var rear_axle_joint: HingeJoint


## steering <-> fork slider, translation along the fork axis (positive = extension).
var fork_joint: PrismaticJoint
## The front spring-damper (a SimForceProvider registered in `world`) between steering and fork slider.
var front_suspension: FrontSuspension
## The rear spring-damper (a SimForceProvider registered in `world`) between chassis and swingarm.
var rear_suspension: RearSuspension

## M2.4.1 ground contact (all null when the configuration has no ground). The ground is a STATIC body registered in `world`; each
## contact is a WheelGroundContact (body A = ground, body B = the wheel) registered in `world.constraints`.
var ground: SimRigidBody
var front_wheel_contact: WheelGroundContact
var rear_wheel_contact: WheelGroundContact
## M2.4.2 tyre models: the TireGroundContact of a wheel whose configuration has a tyre (the SAME object as that wheel's contact, so a
## tyre always references its wheel and its contact); null for a wheel without a tyre.
var front_tire: TireGroundContact
var rear_tire: TireGroundContact
## M2.5.1 finite-width tyre cross-sections (copies of the configuration, null when not configured). Geometry only: the wheel contacts
## above do not use them yet.
var front_tire_geometry
var rear_tire_geometry


## The six bodies: chassis, steering, fork slider, swingarm, front wheel, rear wheel.
func bodies() -> Array:
	return [chassis, steering, fork_slider, swingarm, front_wheel, rear_wheel]


## The four HINGES: steering head, swingarm pivot, front axle, rear axle.
func joints() -> Array:
	return [steering_joint, swingarm_joint, front_axle_joint, rear_axle_joint]


## All five constraints in registration order: the four hinges, then the fork's prismatic joint.
func constraints() -> Array:
	return [steering_joint, swingarm_joint, front_axle_joint, rear_axle_joint, fork_joint]


## The two force providers in registration order: front suspension, rear suspension.
func force_providers() -> Array:
	return [front_suspension, rear_suspension]


func total_mass() -> float:
	var total: float = 0.0
	for body in bodies():
		total += body.mass
	return total


## Largest distance (m) between the two world anchors of any hinge. Zero at construction up to float32 rounding.
func max_anchor_error() -> float:
	var worst: float = 0.0
	for joint in joints():
		worst = maxf(worst, joint.linear_error().length())
	return worst


## Largest angle (rad) between the two world axes of any hinge. Zero at construction up to float32 rounding.
func max_axis_error() -> float:
	var worst: float = 0.0
	for joint in joints():
		worst = maxf(worst, joint.axis_error_angle())
	return worst


## Largest violation of the fork's five constrained directions: lateral offset (m) and, separately, axis (rad) and twist (rad).
## Zero at construction up to float32 rounding. Returns Vector3(lateral m, axis rad, |twist| rad).
func fork_error() -> Vector3:
	return Vector3(fork_joint.lateral_error().length(), fork_joint.axis_error_angle(), absf(fork_joint.twist_error_angle()))


## True while the ground and both wheel contacts are part of the assembly.
func has_ground() -> bool:
	return ground != null and front_wheel_contact != null and rear_wheel_contact != null


## The two wheel contacts in the order front, rear (empty without ground).
func ground_contacts() -> Array:
	return [front_wheel_contact, rear_wheel_contact] if has_ground() else []


## The tyre models in the order front, rear (a wheel without a tyre is skipped).
func tires() -> Array:
	var result: Array = []
	for tire in [front_tire, rear_tire]:
		if tire != null:
			result.append(tire)
	return result


## Enables or disables the tangential response of every tyre (the normal contact stays active: a frictionless contact). Does nothing
## for a wheel without a tyre. A disabled tyre carries no impulse (they are discarded, so re-enabling starts cold).
func set_tire_forces_enabled(enabled: bool) -> void:
	for tire in tires():
		tire.tire_forces_enabled = enabled
		if not enabled:
			tire.reset_warm_start()


## Enables or disables both wheel contacts. Disabling discards their carried impulses (reset_warm_start), so re-enabling starts cold
## and a disabled contact can never act. Does nothing without a ground.
func set_ground_contacts_enabled(enabled: bool) -> void:
	for contact in ground_contacts():
		contact.enabled = enabled
		if not enabled:
			contact.reset_warm_start()


## Removes both wheel contacts and the ground body from the world; the six motorcycle bodies, five joints and both suspensions stay.
## No orphaned constraint or body remains. Does nothing without a ground.
func remove_ground() -> void:
	if not has_ground():
		return
	for contact in ground_contacts():
		world.remove_constraint(contact)
	world.remove_body(ground)
	front_wheel_contact = null
	rear_wheel_contact = null
	front_tire = null
	rear_tire = null
	ground = null
