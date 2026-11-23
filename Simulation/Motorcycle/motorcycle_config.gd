extends RefCounted
## Typed, scene-tree-independent configuration of the motorcycle assembly (M2.1 rigid assembly, M2.2 front telescopic fork,
## M2.3 rear suspension): six rigid bodies, four hinges, one prismatic joint, the front spring-damper and the rear spring-damper. Plain data plus validate(); the builder
## (motorcycle_builder.gd) turns it into a simulation.
##
## Body graph (joint between A and B, A listed first):
##   chassis --steering_head (hinge)--> steering (steering stem and upper fork structure)
##   steering --fork_travel (prismatic, along the fork axis)--> fork_slider (lower fork structure and front-wheel carrier)
##   fork_slider --front_axle (hinge)--> front_wheel
##   chassis --swingarm_pivot (hinge)--> swingarm
##   swingarm --rear_axle (hinge)--> rear_wheel
## The front spring-damper acts between steering and fork_slider along the fork axis (`front_suspension`).
## The rear spring-damper acts directly between chassis and swingarm along the line joining its two attachment points
## (`rear_suspension`); it adds no body and no joint.
##
## All geometry is given ONCE, in world coordinates at the reference pose (every hinge angle and the fork displacement zero):
## body COM positions and orientations, and for every joint a point and an axis. Local anchors, local axes and the reference
## directions are derived by the builder (HingeJointConstraint.from_world, PrismaticJointConstraint.from_world), never typed in
## per body.
## SI units; right-handed, +Y up, local -Z forward, +X right (PHYSICS_CONVENTIONS.md).
##
## Nothing is clamped or repaired: validate() reports every violated rule and the builder refuses an invalid configuration.

const BodySpec = preload("res://Simulation/Motorcycle/motorcycle_body_spec.gd")
const JointSpec = preload("res://Simulation/Motorcycle/motorcycle_joint_spec.gd")
const SuspensionSpec = preload("res://Simulation/Motorcycle/motorcycle_suspension_spec.gd")
const RearSuspensionSpec = preload("res://Simulation/Motorcycle/motorcycle_rear_suspension_spec.gd")
const RearSuspension = preload("res://Simulation/Motorcycle/rear_suspension.gd")
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
## The fork axis must be (anti)parallel to the steering axis: |a x f| <= tol. Both are unit vectors given to float32 precision
## (8 ulps each, 1e-6), so 1e-5 rad leaves an order of magnitude of margin and still rejects a one-hundredth-of-a-degree tilt.
const FORK_AXIS_PARALLEL_TOLERANCE: float = 1.0e-5
## The two spring seats must lie on one line parallel to the fork axis (central force pair); the same bound as the suspension
## component's own check (see front_suspension.gd).
const SEAT_COLLINEAR_TOLERANCE: float = 1.0e-5

var chassis: BodySpec = BodySpec.new("chassis")
var steering: BodySpec = BodySpec.new("steering")
var fork_slider: BodySpec = BodySpec.new("fork_slider")
var swingarm: BodySpec = BodySpec.new("swingarm")
var front_wheel: BodySpec = BodySpec.new("front_wheel")
var rear_wheel: BodySpec = BodySpec.new("rear_wheel")

var steering_head: JointSpec = JointSpec.new("steering_head")
var swingarm_pivot: JointSpec = JointSpec.new("swingarm_pivot")
var front_axle: JointSpec = JointSpec.new("front_axle")
var rear_axle: JointSpec = JointSpec.new("rear_axle")
## The fork's prismatic joint, between `steering` (A) and `fork_slider` (B). `axis` points from the steering assembly toward the
## slider (down the fork), so a positive coordinate is extension. It must be (anti)parallel to the steering-head axis (rake).
var fork_travel: JointSpec = JointSpec.new("fork_travel")
var front_suspension: SuspensionSpec = SuspensionSpec.new()
## The rear spring-damper between `chassis` and `swingarm` (two world attachment points, k, c, rest length).
var rear_suspension: RearSuspensionSpec = RearSuspensionSpec.new()

## Wheel proxy / geometry dimensions (m). Used by presentation and by wheel_inertia() in fixtures; the physical
## quantity is `local_inertia` of the wheel body spec.
var front_wheel_radius: float = 0.0
var rear_wheel_radius: float = 0.0
var front_wheel_width: float = 0.0
var rear_wheel_width: float = 0.0


## The six body specs in the fixed order chassis, steering, fork slider, swingarm, front wheel, rear wheel.
func body_specs() -> Array:
	return [chassis, steering, fork_slider, swingarm, front_wheel, rear_wheel]


## The four HINGE specs in the fixed order steering head, swingarm pivot, front axle, rear axle (the prismatic joint is
## `fork_travel`).
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
	for box_spec in [chassis, steering, fork_slider, swingarm]:
		if box_spec != null and not (box_spec.visual_size.is_finite() and box_spec.visual_size.x > 0.0 and box_spec.visual_size.y > 0.0 and box_spec.visual_size.z > 0.0):
			errors.append("body '%s': visual_size must be finite and > 0 on every axis (got %s)" % [box_spec.body_name, box_spec.visual_size])
	_check_positive(errors, "front_wheel_radius", front_wheel_radius)
	_check_positive(errors, "rear_wheel_radius", rear_wheel_radius)
	_check_positive(errors, "front_wheel_width", front_wheel_width)
	_check_positive(errors, "rear_wheel_width", rear_wheel_width)

	for spec in joint_specs() + [fork_travel]:
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
	_check_fork(errors)
	_check_rear_suspension(errors)
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


## Fork axis and suspension rules. Only checked when their inputs are themselves usable; the generic rules above report the rest.
func _check_fork(errors: PackedStringArray) -> void:
	if fork_travel == null or steering_head == null or not fork_travel.axis.is_finite() or not steering_head.axis.is_finite() \
			or fork_travel.axis.length() < 0.5 or steering_head.axis.length() < 0.5:
		return
	var fork_axis: Vector3 = fork_travel.axis.normalized()
	var skew: float = fork_axis.cross(steering_head.axis.normalized()).length()
	if skew > FORK_AXIS_PARALLEL_TOLERANCE:
		errors.append("joint 'fork_travel': the fork axis must be parallel to the steering axis (rake), |a x f| = %s > %s" % [skew, FORK_AXIS_PARALLEL_TOLERANCE])
	var spec: SuspensionSpec = front_suspension
	if spec == null:
		errors.append("front suspension spec is null")
		return
	if not (spec.upper_attachment.is_finite() and spec.lower_attachment.is_finite()):
		errors.append("front suspension: attachment points must be finite")
	else:
		var separation: Vector3 = spec.lower_attachment - spec.upper_attachment
		var lateral: float = (separation - fork_axis * separation.dot(fork_axis)).length()
		if lateral > SEAT_COLLINEAR_TOLERANCE:
			errors.append("front suspension: the two attachment points must lie on one line parallel to the fork axis (lateral offset %s m > %s m)" % [lateral, SEAT_COLLINEAR_TOLERANCE])
	if not (is_finite(spec.stiffness) and spec.stiffness >= 0.0):
		errors.append("front suspension: stiffness must be finite and >= 0 N/m (got %s)" % spec.stiffness)
	if not (is_finite(spec.damping) and spec.damping >= 0.0):
		errors.append("front suspension: damping must be finite and >= 0 N s/m (got %s)" % spec.damping)
	if not is_finite(spec.rest_displacement):
		errors.append("front suspension: rest displacement must be finite (got %s)" % spec.rest_displacement)
	if not (is_finite(spec.lower_travel_limit) and is_finite(spec.upper_travel_limit)):
		errors.append("front suspension: travel limits must be finite (lower %s, upper %s)" % [spec.lower_travel_limit, spec.upper_travel_limit])
	elif spec.travel_limits_enabled and not (spec.lower_travel_limit <= 0.0 and 0.0 <= spec.upper_travel_limit):
		errors.append("front suspension: with travel limits enabled, lower <= 0 <= upper is required so the reference pose lies inside the travel (lower %s, upper %s)" % [spec.lower_travel_limit, spec.upper_travel_limit])


## Rear spring-damper rules (single home of the rules; RearSuspension.validate() repeats them on the built object).
func _check_rear_suspension(errors: PackedStringArray) -> void:
	var spec: RearSuspensionSpec = rear_suspension
	if spec == null:
		errors.append("rear suspension spec is null")
		return
	if not (spec.chassis_attachment.is_finite() and spec.swingarm_attachment.is_finite()):
		errors.append("rear suspension: attachment points must be finite")
	else:
		var separation: float = (spec.swingarm_attachment - spec.chassis_attachment).length()
		# Finite coordinates can still overflow the length computation (float32): +INF >= MIN_LENGTH is true, so finiteness is checked first.
		if not is_finite(separation):
			errors.append("rear suspension: the attachment separation is not finite (%s m): the shock direction is undefined" % separation)
		elif separation < RearSuspension.MIN_LENGTH:
			errors.append("rear suspension: the attachment points are %s m apart (< %s m): the shock direction is undefined" % [separation, RearSuspension.MIN_LENGTH])
	if not (is_finite(spec.stiffness) and spec.stiffness >= 0.0):
		errors.append("rear suspension: stiffness must be finite and >= 0 N/m (got %s)" % spec.stiffness)
	if not (is_finite(spec.damping) and spec.damping >= 0.0):
		errors.append("rear suspension: damping must be finite and >= 0 N s/m (got %s)" % spec.damping)
	if not (is_finite(spec.rest_length) and spec.rest_length > 0.0):
		errors.append("rear suspension: rest length must be finite and > 0 m (got %s)" % spec.rest_length)
