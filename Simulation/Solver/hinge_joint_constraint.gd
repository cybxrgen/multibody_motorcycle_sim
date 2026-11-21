extends "res://Simulation/Solver/sim_constraint.gd"
## Hinge (revolute) joint: 5 constrained DOF, 1 free rotation about the hinge axis, with optional
## angular limits and a velocity motor. Generic rigid-body constraint (no vehicle/rider logic).
## Either body may be static. All Jacobian vectors are world space; lever arms start at each body's COM.
##
## ---- Frame data (all LOCAL to the respective body; see PHYSICS_CONVENTIONS.md, "Hinge joint") ----
##   local_anchor_a/b     hinge pivot point
##   local_axis_a/b       unit hinge axis; the two world axes coincide when the joint is satisfied
##   local_reference_a/b  unit direction perpendicular to the axis; defines the zero angle:
##                        angle == 0  <=>  R_a*reference_a == R_b*reference_b (axes aligned)
## Use HingeJoint.from_world() to build a joint whose zero angle is the bodies' current pose.
##
## ---- World quantities (R = body orientation) ----
##   r_a = R_a*anchor_a, r_b = R_b*anchor_b      a = R_a*axis_a,  b = R_b*axis_b
##   p = R_a*reference_a (rigid in A),  q = a x p   ->  (p, q, a) is a right-handed orthonormal frame fixed in A
##
## ---- Rows (always 8; inactive rows have impulse bounds [0, 0] and therefore do nothing) ----
##  0..2 anchor      C = (x_b + r_b) - (x_a + r_a), rows along world X, Y, Z (identical to BallJoint)
##                   Jv_a = -e, Jw_a = -(r_a x e), Jv_b = +e, Jw_b = r_b x e,  bias = 0 (physical), split_bias = (beta/dt) C.e
##  3,4  axis align  C_p = p.b, C_q = q.b. Both are zero exactly when b = +-a. For a small misalignment
##                   they equal the rotation of b away from a about -q / +p, i.e. a sine-type angle error.
##                   Exact derivative (p, q rotate with A):  dC_p/dt = (w_a x p).b + p.(w_b x b)
##                   =>  Jw_a = p x b,  Jw_b = b x p   (same for q). Jv = 0, bias = 0 (physical), split_bias = (beta/dt) C.
##                   Both Jw are perpendicular to b (and to a when aligned), so rotation about the hinge axis is
##                   not constrained. p and q come from A's own reference direction, so the basis is never
##                   singular (no "pick any perpendicular" fallback) and works for any hinge orientation.
##                   Known limit: b = -a also satisfies C = 0 (axes anti-parallel). The joint is only
##                   meaningful for misalignment well below 90 degrees, which holds for any joint that
##                   starts satisfied.
##  5    motor       Cdot = (w_b - w_a).a = d(angle)/dt.  Jw_a = -a, Jw_b = +a.  bias = -target_speed.
##                   Impulse bounds +-(max_torque * dt). Positive speed => positive angle rate.
##  6    lower limit C_l = angle - lower >= 0. Jw_a = -a, Jw_b = +a. Bounds [0, +INF): the impulse can only
##                   push the angle up.
##  7    upper limit C_u = upper - angle >= 0. Jw_a = +a, Jw_b = -a (Cdot = -d(angle)/dt). Bounds [0, +INF).
##                   Limit biases (a unilateral row enforces Cdot + bias >= 0):
##                     C >= 0 (inside): bias = C/dt   physical, speculative: the row only engages if the angle would cross the
##                                      limit within this step (angle + rate*dt < limit); not a correction, so no beta.
##                                      The split channel is inactive.
##                     C <  0 (violated): physical bias = 0 (the limit may not be pushed further: Cdot >= 0), and the split
##                                      channel corrects the violation: split_bias = (beta/dt) C, split bounds [0, +INF).
##                   The physical bias is continuous at C = 0 (C/dt -> 0).
##
## ---- Motor / limit cooperation ----
## Before the motor row is built its target speed is clamped into the rate interval the enabled limit rows admit:
##   rate >= -bias_lower  and  rate <= +bias_upper   (the PHYSICAL limit biases).
## The motor then never asks for a velocity a limit row forbids, so the two rows never push in opposite
## directions (a motor commanded into a stop settles on the speculative approach rate, then 0 at the stop)
## and the motor can never push through a stop. A violated limit has physical bias 0, so the motor cannot drive further into
## it; the violation itself is removed by the split channel, which the motor never sees. The clamp uses the same biases as the limit rows.
##
## ---- Continuous angle ----
## angle() = atan2(w.q, w.p) with w = R_b*reference_b: the signed rotation of B's reference about A's axis,
## positive right-handed about +a (B turns counter-clockwise seen from +a relative to A). Only the DIFFERENCE between
## two successive wrapped values is accumulated, wrapped to [-PI, PI), so the reported angle has no jump at +-PI
## and keeps counting through any number of turns. The tracker needs a sample at least once per half turn of
## relative rotation; every solver step samples it, so it holds while |relative rate| * dt < PI. Direct pose edits
## (teleporting) between samples that exceed half a turn alias to the nearest equivalent angle.
## A tilt of b away from a changes the angle only at second order (w stays in the p/q plane to first order).
##
## ---- Torque cap ----
## Rows are impulses. motor_max_torque [N m] -> impulse bound motor_max_torque * dt with dt = config.fixed_timestep.

const ROW_ANCHOR: int = 0
const ROW_AXIS_P: int = 3
const ROW_AXIS_Q: int = 4
const ROW_MOTOR: int = 5
const ROW_LOWER: int = 6
const ROW_UPPER: int = 7
const ROW_COUNT: int = 8

## Unit-length / perpendicularity tolerance for caller-supplied direction vectors: 8 float32 ulps (8 * 1.19e-7),
## the error a normalized or rotated Godot vector can legitimately carry. Anything beyond is invalid input, not rounding.
const DIRECTION_TOLERANCE: float = 1.0e-6

var local_anchor_a: Vector3 = Vector3.ZERO
var local_anchor_b: Vector3 = Vector3.ZERO
var local_axis_a: Vector3 = Vector3.RIGHT
var local_axis_b: Vector3 = Vector3.RIGHT
var local_reference_a: Vector3 = Vector3.UP
var local_reference_b: Vector3 = Vector3.UP

## Both limits are enabled together; radians on the continuous angle scale; lower <= upper, both finite.
var limit_enabled: bool = false
var lower_limit: float = 0.0
var upper_limit: float = 0.0

var motor_enabled: bool = false
## rad/s, positive = positive angle rate (see header).
var motor_target_speed: float = 0.0
## N m, >= 0. 0 behaves as a disabled motor.
var motor_max_torque: float = 0.0

var _angle: float = 0.0
var _last_wrapped: float = 0.0
var _r_a: Vector3 = Vector3.ZERO
var _r_b: Vector3 = Vector3.ZERO


func _init(p_body_a: SimRigidBody, p_body_b: SimRigidBody, p_local_anchor_a: Vector3, p_local_anchor_b: Vector3,
		p_local_axis_a: Vector3, p_local_axis_b: Vector3, p_local_reference_a: Vector3, p_local_reference_b: Vector3) -> void:
	super(p_body_a, p_body_b)
	local_anchor_a = p_local_anchor_a
	local_anchor_b = p_local_anchor_b
	local_axis_a = p_local_axis_a
	local_axis_b = p_local_axis_b
	local_reference_a = p_local_reference_a
	local_reference_b = p_local_reference_b
	for i in ROW_COUNT:
		add_row()
	var errors: PackedStringArray = validate()
	if not errors.is_empty():
		push_error("HingeJoint: invalid configuration: %s" % "; ".join(errors))
		return
	# Input is valid within DIRECTION_TOLERANCE: remove the residual rounding so the frames are exactly orthonormal.
	local_axis_a = local_axis_a.normalized()
	local_axis_b = local_axis_b.normalized()
	local_reference_a = (local_reference_a - local_axis_a * local_axis_a.dot(local_reference_a)).normalized()
	local_reference_b = (local_reference_b - local_axis_b * local_axis_b.dot(local_reference_b)).normalized()
	_last_wrapped = _measure_wrapped_angle()
	_angle = _last_wrapped


## Builds a hinge from WORLD data given the bodies' current poses. The anchor, axis and reference direction are
## converted to each body's local frame, so the current pose is the zero-angle configuration and the joint starts with
## no error. `world_reference` must be a unit vector perpendicular to `world_axis`.
## Returns null (with an error) if the input is invalid.
static func from_world(p_body_a: SimRigidBody, p_body_b: SimRigidBody, world_anchor: Vector3, world_axis: Vector3,
		world_reference: Vector3):
	if p_body_a == null or p_body_b == null:
		push_error("HingeJoint.from_world: bodies must not be null.")
		return null
	var inv_a: Quaternion = p_body_a.orientation.inverse()
	var inv_b: Quaternion = p_body_b.orientation.inverse()
	var joint = (load("res://Simulation/Solver/hinge_joint_constraint.gd")).new(
		p_body_a, p_body_b,
		inv_a * (world_anchor - p_body_a.position), inv_b * (world_anchor - p_body_b.position),
		inv_a * world_axis, inv_b * world_axis,
		inv_a * world_reference, inv_b * world_reference)
	return joint if joint.is_valid() else null


func validate() -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if body_a == null or body_b == null:
		errors.append("hinge bodies must not be null")
		return errors
	if body_a == body_b:
		errors.append("hinge must connect two different bodies")
	if body_a.is_static() and body_b.is_static():
		errors.append("hinge between two static bodies has nothing to solve")
	if not (_finite(local_anchor_a) and _finite(local_anchor_b)):
		errors.append("hinge anchors must be finite")
	_check_direction(errors, local_axis_a, "axis A")
	_check_direction(errors, local_axis_b, "axis B")
	_check_direction(errors, local_reference_a, "reference A")
	_check_direction(errors, local_reference_b, "reference B")
	if _finite(local_axis_a) and _finite(local_reference_a) and absf(local_axis_a.dot(local_reference_a)) > DIRECTION_TOLERANCE:
		errors.append("reference A must be perpendicular to axis A (dot = %s)" % local_axis_a.dot(local_reference_a))
	if _finite(local_axis_b) and _finite(local_reference_b) and absf(local_axis_b.dot(local_reference_b)) > DIRECTION_TOLERANCE:
		errors.append("reference B must be perpendicular to axis B (dot = %s)" % local_axis_b.dot(local_reference_b))
	if not (is_finite(lower_limit) and is_finite(upper_limit)):
		errors.append("limits must be finite (lower %s, upper %s)" % [lower_limit, upper_limit])
	elif limit_enabled and lower_limit > upper_limit:
		errors.append("lower_limit must be <= upper_limit (lower %s, upper %s)" % [lower_limit, upper_limit])
	if not is_finite(motor_target_speed):
		errors.append("motor_target_speed must be finite (got %s)" % motor_target_speed)
	if not (is_finite(motor_max_torque) and motor_max_torque >= 0.0):
		errors.append("motor_max_torque must be finite and >= 0 (got %s)" % motor_max_torque)
	return errors


static func _finite(v: Vector3) -> bool:
	return is_finite(v.x) and is_finite(v.y) and is_finite(v.z)


static func _check_direction(errors: PackedStringArray, v: Vector3, label: String) -> void:
	if not _finite(v):
		errors.append("%s must be finite" % label)
	elif absf(v.length() - 1.0) > DIRECTION_TOLERANCE:
		errors.append("%s must be a unit vector (length %s)" % [label, v.length()])


# --- Queries ---

func world_anchor_a() -> Vector3:
	return body_a.position + body_a.orientation * local_anchor_a


func world_anchor_b() -> Vector3:
	return body_b.position + body_b.orientation * local_anchor_b


## C = anchor_b - anchor_a in world space; zero when the anchors coincide. (Name shared with BallJoint.)
func linear_error() -> Vector3:
	return world_anchor_b() - world_anchor_a()


func world_axis_a() -> Vector3:
	return body_a.orientation * local_axis_a


func world_axis_b() -> Vector3:
	return body_b.orientation * local_axis_b


## Angle between the two hinge axes in radians (>= 0), robust for small values (atan2 of |a x b| and a.b).
func axis_error_angle() -> float:
	var a: Vector3 = world_axis_a()
	var b: Vector3 = world_axis_b()
	return atan2(a.cross(b).length(), a.dot(b))


## Continuous hinge angle in radians (see header). Samples the tracker: calling it more often is harmless,
## calling it less often than once per half turn of relative rotation aliases (the solver samples every step).
func angle() -> float:
	var wrapped: float = _measure_wrapped_angle()
	_angle += wrapf(wrapped - _last_wrapped, -PI, PI)
	_last_wrapped = wrapped
	return _angle


## Relative angular velocity about the hinge axis, (w_b - w_a).a in rad/s: the rate of angle().
func angle_rate() -> float:
	return (body_b.angular_velocity - body_a.angular_velocity).dot(world_axis_a())


## Last step's accumulated impulses of the motor / limit rows (N m s). Meaningful for the last step only.
func motor_impulse() -> float:
	return rows[ROW_MOTOR].accumulated_impulse


func lower_limit_impulse() -> float:
	return rows[ROW_LOWER].accumulated_impulse


func upper_limit_impulse() -> float:
	return rows[ROW_UPPER].accumulated_impulse


## Impulse bound currently set on the motor row (motor_max_torque * dt after the last prepare, 0 if inactive).
func motor_impulse_bound() -> float:
	return rows[ROW_MOTOR].max_impulse


func _measure_wrapped_angle() -> float:
	var a: Vector3 = body_a.orientation * local_axis_a
	var p: Vector3 = body_a.orientation * local_reference_a
	var q: Vector3 = a.cross(p)
	var w: Vector3 = body_b.orientation * local_reference_b
	return atan2(w.dot(q), w.dot(p))


# --- Solver interface ---

func _update_rows(dt: float, config) -> void:
	var beta_over_dt: float = config.baumgarte_factor / dt
	_r_a = body_a.orientation * local_anchor_a
	_r_b = body_b.orientation * local_anchor_b
	var a: Vector3 = body_a.orientation * local_axis_a
	var b: Vector3 = body_b.orientation * local_axis_b
	var p: Vector3 = body_a.orientation * local_reference_a
	var q: Vector3 = a.cross(p)
	var theta: float = angle()

	# Anchor rows.
	var c: Vector3 = (body_b.position + _r_b) - (body_a.position + _r_a)
	var axes: Array = [Vector3.RIGHT, Vector3.UP, Vector3.BACK]
	for i in 3:
		var e: Vector3 = axes[i]
		var row = rows[ROW_ANCHOR + i]
		row.linear_a = -e
		row.angular_a = -_r_a.cross(e)
		row.linear_b = e
		row.angular_b = _r_b.cross(e)
		row.bias = 0.0
		row.softness = 0.0
		row.min_impulse = -INF
		row.max_impulse = INF
		_set_split(row, beta_over_dt * c.dot(e), -INF, INF)

	# Axis alignment rows.
	_set_axis_row(rows[ROW_AXIS_P], p, b, beta_over_dt * p.dot(b))
	_set_axis_row(rows[ROW_AXIS_Q], q, b, beta_over_dt * q.dot(b))

	# Limit biases (physical; also needed to clamp the motor target).
	var bias_lower: float = 0.0
	var bias_upper: float = 0.0
	var gap_lower: float = theta - lower_limit
	var gap_upper: float = upper_limit - theta
	if limit_enabled:
		bias_lower = _limit_bias(gap_lower, dt)
		bias_upper = _limit_bias(gap_upper, dt)

	# Motor row.
	var motor_row = rows[ROW_MOTOR]
	_set_hinge_axis_row(motor_row, a, 1.0)
	motor_row.split_enabled = false
	var motor_active: bool = motor_enabled and motor_max_torque > 0.0
	if motor_active:
		var target: float = motor_target_speed
		if limit_enabled:
			target = minf(maxf(target, -bias_lower), bias_upper)
		motor_row.bias = -target
		var impulse_bound: float = motor_max_torque * dt
		motor_row.min_impulse = -impulse_bound
		motor_row.max_impulse = impulse_bound
	else:
		_deactivate(motor_row)

	# Limit rows.
	var lower_row = rows[ROW_LOWER]
	var upper_row = rows[ROW_UPPER]
	_set_hinge_axis_row(lower_row, a, 1.0)
	_set_hinge_axis_row(upper_row, a, -1.0)
	if limit_enabled:
		lower_row.bias = bias_lower
		upper_row.bias = bias_upper
		lower_row.min_impulse = 0.0
		lower_row.max_impulse = INF
		upper_row.min_impulse = 0.0
		upper_row.max_impulse = INF
		# Positional correction only for a violated limit (split bounds [0, +INF): it can only push back inside).
		_set_limit_split(lower_row, gap_lower, beta_over_dt)
		_set_limit_split(upper_row, gap_upper, beta_over_dt)
	else:
		_deactivate(lower_row)
		_deactivate(upper_row)


## Physical limit bias: speculative C/dt while the limit is not violated, 0 once it is (continuous at C = 0). The positional
## correction of a violated limit is not part of it; it goes through the split channel (_set_limit_split).
static func _limit_bias(c: float, dt: float) -> float:
	return c / dt if c >= 0.0 else 0.0


## Split channel of a limit row: active only while violated (C < 0), bias (beta/dt) C (negative), bounds [0, +INF).
static func _set_limit_split(row, c: float, beta_over_dt: float) -> void:
	if c < 0.0:
		_set_split(row, beta_over_dt * c, 0.0, INF)
	else:
		row.split_enabled = false


static func _set_split(row, split_bias: float, split_min: float, split_max: float) -> void:
	row.split_enabled = true
	row.split_bias = split_bias
	row.split_min_impulse = split_min
	row.split_max_impulse = split_max


## Axis alignment row: physical bias 0, positional correction `split_bias` through the split channel.
static func _set_axis_row(row, direction: Vector3, b: Vector3, split_bias: float) -> void:
	row.linear_a = Vector3.ZERO
	row.linear_b = Vector3.ZERO
	row.angular_a = direction.cross(b)
	row.angular_b = b.cross(direction)
	row.bias = 0.0
	row.softness = 0.0
	row.min_impulse = -INF
	row.max_impulse = INF
	_set_split(row, split_bias, -INF, INF)


## Row along the hinge axis: Cdot = direction_sign * (w_b - w_a).a.
static func _set_hinge_axis_row(row, a: Vector3, direction_sign: float) -> void:
	row.linear_a = Vector3.ZERO
	row.linear_b = Vector3.ZERO
	row.angular_a = -a * direction_sign
	row.angular_b = a * direction_sign
	row.softness = 0.0


## Inactive row: bounds [0, 0] make ConstraintRow clamp any carried impulse to zero (nothing is warm started,
## nothing is applied, no stale impulse survives to the step in which the feature is re-enabled).
static func _deactivate(row) -> void:
	row.bias = 0.0
	row.min_impulse = 0.0
	row.max_impulse = 0.0
	row.split_enabled = false
