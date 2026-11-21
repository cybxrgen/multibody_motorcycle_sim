extends RefCounted
## Typed, scene-tree-independent configuration of the rigid motorcycle assembly (M2.1): five rigid bodies and the four
## hinges that connect them. Plain data plus validate(); the builder (motorcycle_builder.gd) turns it into a simulation.
##
## Body graph (hinge between A and B, A listed first):
##   chassis --steering_head--> steering (fork + stem + front-wheel carrier, ONE rigid body in M2.1)
##   chassis --swingarm_pivot-> swingarm
##   steering --front_axle---> front_wheel
##   swingarm --rear_axle----> rear_wheel
##
## All geometry is given ONCE, in world coordinates at the reference pose (every hinge angle zero): body COM positions and
## orientations, and for every hinge a pivot point and an axis. Local anchors, local axes and the hinge reference
## directions are derived by the builder (HingeJointConstraint.from_world), never typed in per body.
## SI units; right-handed, +Y up, local -Z forward, +X right (PHYSICS_CONVENTIONS.md).
##
## Nothing is clamped or repaired: validate() reports every violated rule and the builder refuses an invalid configuration.

const BodySpec = preload("res://Simulation/Motorcycle/motorcycle_body_spec.gd")
const JointSpec = preload("res://Simulation/Motorcycle/motorcycle_joint_spec.gd")
const SimMassProperties = preload("res://Simulation/Core/sim_mass_properties.gd")
const HingeJoint = preload("res://Simulation/Solver/hinge_joint_constraint.gd")

## Unit-length tolerance of a hinge axis: the hinge constraint's own input tolerance (8 float32 ulps), so a configuration
## that passes here is never refused by HingeJointConstraint.
const AXIS_UNIT_TOLERANCE: float = HingeJoint.DIRECTION_TOLERANCE
## | |q| - 1 | allowed for a body orientation: the same bound SimWorld applies to stored orientations (float32 rounding of a
## normalized quaternion is ~6e-8).
const ORIENTATION_NORM_TOLERANCE: float = 1.0e-5
## A wheel axle must be an eigenvector of the wheel's local inertia: |I u - (u.I u) u| <= tol * |I u|. R D R^T carries a few
## float32 ulps (~1e-6 relative); a genuine misorientation of 1 degree of a wheel whose moments differ by 30 % gives ~5e-3.
const WHEEL_AXLE_PRINCIPAL_TOLERANCE: float = 1.0e-4

var chassis: BodySpec = BodySpec.new("chassis")
var steering: BodySpec = BodySpec.new("steering")
var swingarm: BodySpec = BodySpec.new("swingarm")
var front_wheel: BodySpec = BodySpec.new("front_wheel")
var rear_wheel: BodySpec = BodySpec.new("rear_wheel")

var steering_head: JointSpec = JointSpec.new("steering_head")
var swingarm_pivot: JointSpec = JointSpec.new("swingarm_pivot")
var front_axle: JointSpec = JointSpec.new("front_axle")
var rear_axle: JointSpec = JointSpec.new("rear_axle")

## Wheel proxy / geometry dimensions (m). Used by presentation and by wheel_inertia() in fixtures; the physical
## quantity is `local_inertia` of the wheel body spec.
var front_wheel_radius: float = 0.0
var rear_wheel_radius: float = 0.0
var front_wheel_width: float = 0.0
var rear_wheel_width: float = 0.0


## The five body specs in the fixed order chassis, steering, swingarm, front wheel, rear wheel.
func body_specs() -> Array:
	return [chassis, steering, swingarm, front_wheel, rear_wheel]


## The four joint specs in the fixed order steering head, swingarm pivot, front axle, rear axle.
func joint_specs() -> Array:
	return [steering_head, swingarm_pivot, front_axle, rear_axle]


## Solid or hollow circular cylinder (a wheel) with its axle along the BODY-LOCAL X axis, about its COM:
##   I_axle      = m/2 (r_o^2 + r_i^2)
##   I_transverse = m/12 (3 (r_o^2 + r_i^2) + w^2)     (the two axes perpendicular to the axle)
## r_o outer radius, r_i inner radius (0 = solid disc), w width. Limits: thin ring (r_i -> r_o, w -> 0) gives m r^2 and
## m r^2 / 2; solid disc gives m r^2 / 2 and m (3 r^2 + w^2) / 12.
static func wheel_inertia(p_mass: float, outer_radius: float, inner_radius: float, width: float) -> Basis:
	var r2: float = outer_radius * outer_radius + inner_radius * inner_radius
	var axial: float = 0.5 * p_mass * r2
	var transverse: float = p_mass / 12.0 * (3.0 * r2 + width * width)
	return Basis.from_scale(Vector3(axial, transverse, transverse))


## One message per violated rule; empty means valid. Names the offending body / joint.
func validate() -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	var seen_names: Dictionary = {}
	var specs: Array = body_specs()
	for i in specs.size():
		var spec = specs[i]
		if spec == null:
			errors.append("body spec #%d is null" % i)
			continue
		var label: String = "body '%s'" % spec.body_name
		if spec.body_name == "":
			errors.append("body spec #%d has an empty name" % i)
		elif seen_names.has(spec.body_name):
			errors.append("%s: duplicate body name" % label)
		seen_names[spec.body_name] = true
		if not (is_finite(spec.mass) and spec.mass > 0.0):
			errors.append("%s: mass must be finite and > 0 (got %s); a mass <= 0 would make the body static" % [label, spec.mass])
		var inertia_error: String = SimMassProperties.validate_inertia(spec.local_inertia)
		if inertia_error != "":
			errors.append("%s: invalid inertia: %s" % [label, inertia_error])
		if not spec.position.is_finite():
			errors.append("%s: position must be finite (got %s)" % [label, spec.position])
		if not spec.orientation.is_finite() or absf(spec.orientation.length() - 1.0) > ORIENTATION_NORM_TOLERANCE:
			errors.append("%s: orientation must be a finite unit quaternion (got %s)" % [label, spec.orientation])
	for box_spec in [chassis, steering, swingarm]:
		if box_spec != null and not (box_spec.visual_size.is_finite() and box_spec.visual_size.x > 0.0 and box_spec.visual_size.y > 0.0 and box_spec.visual_size.z > 0.0):
			errors.append("body '%s': visual_size must be finite and > 0 on every axis (got %s)" % [box_spec.body_name, box_spec.visual_size])
	_check_positive(errors, "front_wheel_radius", front_wheel_radius)
	_check_positive(errors, "rear_wheel_radius", rear_wheel_radius)
	_check_positive(errors, "front_wheel_width", front_wheel_width)
	_check_positive(errors, "rear_wheel_width", rear_wheel_width)

	for spec in joint_specs():
		if spec == null:
			errors.append("a joint spec is null")
			continue
		var label: String = "joint '%s'" % spec.joint_name
		if spec.joint_name == "":
			errors.append("a joint spec has an empty name")
		if not spec.point.is_finite():
			errors.append("%s: point must be finite (got %s)" % [label, spec.point])
		if not spec.axis.is_finite():
			errors.append("%s: axis must be finite (got %s)" % [label, spec.axis])
		elif absf(spec.axis.length() - 1.0) > AXIS_UNIT_TOLERANCE:
			errors.append("%s: axis must be a unit vector (length %s)" % [label, spec.axis.length()])
	_check_wheel_axle(errors, front_wheel, front_axle)
	_check_wheel_axle(errors, rear_wheel, rear_axle)
	return errors


func is_valid() -> bool:
	return validate().is_empty()


static func _check_positive(errors: PackedStringArray, field: String, value: float) -> void:
	if not (is_finite(value) and value > 0.0):
		errors.append("%s must be finite and > 0 (got %s)" % [field, value])


## The wheel's axle, expressed in the wheel's own frame, must be a principal axis of its inertia (the wheel is spun about
## it). Only checked when the inputs are themselves usable; their own rules report the rest.
func _check_wheel_axle(errors: PackedStringArray, wheel: BodySpec, axle: JointSpec) -> void:
	if wheel == null or axle == null or not axle.axis.is_finite() or axle.axis.length() < 0.5 or not wheel.orientation.is_finite():
		return
	if SimMassProperties.validate_inertia(wheel.local_inertia) != "" or wheel.orientation.length() < 0.5:
		return
	var u: Vector3 = (wheel.orientation.normalized().inverse() * axle.axis).normalized()
	var iu: Vector3 = wheel.local_inertia * u
	var along: float = u.dot(iu)
	var residual: float = (iu - u * along).length()
	if residual > WHEEL_AXLE_PRINCIPAL_TOLERANCE * iu.length():
		errors.append("body '%s': axle '%s' is not a principal axis of the wheel inertia (relative residual %s > %s)" % [
			wheel.body_name, axle.joint_name, residual / iu.length(), WHEEL_AXLE_PRINCIPAL_TOLERANCE])
