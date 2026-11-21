extends RefCounted
## Result of MotorcycleBuilder.build(): the world and NAMED references to the five bodies and four hinges, so tests and
## presentation never index arrays or rely on body ids. A successfully built assembly is complete: it is only ever
## constructed with all five bodies and all four joints registered in `world`.
## Owns nothing physical: the bodies and constraints belong to `world` (SimWorld owns all physical state).

const SimWorld = preload("res://Simulation/Core/sim_world.gd")
const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")
const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")

var world: SimWorld
## The configuration the assembly was built from (kept for geometry queries; not re-read by the simulation).
var config

var chassis: SimRigidBody
var steering: SimRigidBody
var swingarm: SimRigidBody
var front_wheel: SimRigidBody
var rear_wheel: SimRigidBody

## chassis <-> steering, about the steering axis.
var steering_joint: HingeJoint
## chassis <-> swingarm, about the swingarm pivot axis.
var swingarm_joint: HingeJoint
## steering <-> front wheel, about the front axle.
var front_axle_joint: HingeJoint
## swingarm <-> rear wheel, about the rear axle.
var rear_axle_joint: HingeJoint


## The five bodies: chassis, steering, swingarm, front wheel, rear wheel.
func bodies() -> Array:
	return [chassis, steering, swingarm, front_wheel, rear_wheel]


## The four hinges: steering head, swingarm pivot, front axle, rear axle.
func joints() -> Array:
	return [steering_joint, swingarm_joint, front_axle_joint, rear_axle_joint]


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
