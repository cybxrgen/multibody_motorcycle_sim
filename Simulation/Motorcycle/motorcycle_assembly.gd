extends RefCounted
## Result of MotorcycleBuilder.build(): the world and NAMED references to the six bodies, the four hinges, the fork's prismatic
## joint and the front suspension, so tests and presentation never index arrays or rely on body ids. A successfully built
## assembly is complete: it is only ever constructed with all six bodies, all five joints and the suspension registered in `world`.
## Owns nothing physical: the bodies, constraints and the force provider belong to `world` (SimWorld owns all physical state).

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")
const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")
const PrismaticJoint = preload("res://Simulation/Solver/prismatic_joint_constraint.gd")
const FrontSuspension = preload("res://Simulation/Motorcycle/front_suspension.gd")

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


## The six bodies: chassis, steering, fork slider, swingarm, front wheel, rear wheel.
func bodies() -> Array:
	return [chassis, steering, fork_slider, swingarm, front_wheel, rear_wheel]


## The four HINGES: steering head, swingarm pivot, front axle, rear axle.
func joints() -> Array:
	return [steering_joint, swingarm_joint, front_axle_joint, rear_axle_joint]


## All five constraints in registration order: the four hinges, then the fork's prismatic joint.
func constraints() -> Array:
	return [steering_joint, swingarm_joint, front_axle_joint, rear_axle_joint, fork_joint]


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
