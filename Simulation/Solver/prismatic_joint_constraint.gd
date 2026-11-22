extends "res://Simulation/Solver/sim_constraint.gd"
## Prismatic (slider) joint: 1 free DOF (translation along the joint axis), 5 constrained DOF (2 lateral translations,
## 3 relative rotations), with optional finite travel limits. Generic rigid-body constraint (no vehicle/rider logic, no
## scene tree). Either body may be static (not both). All Jacobian vectors are world space; lever arms start at each COM.
##
## ---- Frame data (all LOCAL to the respective body; same pattern as HingeJointConstraint) ----
##   local_anchor_a/b     a point of the joint axis line, on each body
##   local_axis_a/b       unit joint axis; the two world axes coincide when the joint is satisfied
##   local_reference_a/b  unit direction perpendicular to the axis; fixes the relative twist (zero twist <=>
##                        R_a*reference_a == R_b*reference_b with aligned axes)
## PrismaticJointConstraint.from_world() builds a joint whose current pose is the reference configuration (translation 0).
##
## ---- World quantities (R = body orientation) ----
##   r_a = R_a*anchor_a, r_b = R_b*anchor_b
##   a = R_a*axis_a,  p = R_a*reference_a,  q = a x p   ->  (p, q, a) is a right-handed orthonormal frame fixed in A
##   b = R_b*axis_b,  w = R_b*reference_b                ->  B's axis and reference
##   d = (x_b + r_b) - (x_a + r_a)                       offset of B's anchor from A's anchor
##
## ---- Translation coordinate (signed) ----
##   s = d . a   [m]. Positive when B's anchor is displaced from A's anchor along +a (the A-side axis direction).
##   s = 0 at the configuration given to from_world() (both anchors are the same world point there). Rate:
##   s_dot = a . d_dot + d . (w_a x a)  (the axis turns with A); translation_rate() evaluates it exactly.
##
## ---- Rows (always 7; inactive rows have impulse bounds [0, 0] and therefore do nothing) ----
##  0,1  lateral    C_p = d . p, C_q = d . q  (anchor offset perpendicular to the axis; the frame (p, q) turns with A).
##                  For e in {p, q}: C_dot = e . d_dot + d . (w_a x e), so
##                    Jv_a = -e, Jw_a = -(r_a x e) + (e x d) = -((r_a + d) x e), Jv_b = +e, Jw_b = r_b x e.
##                  The e x d term is NOT zero: with an axial offset s a rotation of A swings the axis line sideways by s.
##                  Physical bias 0, split bias (beta/dt) C, bilateral. Jv.a = 0: axial translation is not touched.
##  2,3  axis       C_p' = p . b, C_q' = q . b (zero exactly when b = +-a; sine-type error of B's axis away from A's).
##                  Jw_a = p x b, Jw_b = b x p (same for q). Jv = 0. Physical bias 0, split bias (beta/dt) C, bilateral.
##  4    twist      C_t = w . q (sine of the relative rotation about the axis; w = p at the reference).
##                  C_t_dot = w_b . (w x q) + w_a . (q x w)  =>  Jw_b = w x q (= +a at the reference), Jw_a = q x w (= -a).
##                  Physical bias 0, split bias (beta/dt) C_t, bilateral. Rows 2-4 together lock all three rotations.
##  5    lower limit C_l = s - lower >= 0. Jv_b = +a, Jw_b = r_b x a, Jv_a = -a, Jw_a = -((r_a + d) x a)  (C_dot = s_dot).
##                  Bounds [0, +INF): the impulse can only push s up (a force on B along +a, on A along -a).
##  6    upper limit C_u = upper - s >= 0. The negated Jacobian of row 5 (C_dot = -s_dot). Bounds [0, +INF): it can only push
##                  s down (a force on B along -a, on A along +a).
##                  Limit biases (a unilateral row enforces C_dot + bias >= 0), identical in form to the hinge limits:
##                    C >= 0 (inside): physical bias C/dt, speculative: the row only engages if the coordinate would cross the
##                                     limit within this step (s + s_dot dt < lower); not a correction. Split channel inactive.
##                    C <  0 (violated): physical bias 0 (the coordinate may not move further out: C_dot >= 0) and the split
##                                     channel corrects the violation: split_bias = (beta/dt) C (negative), bounds [0, +INF).
##
## ---- Free axis ----
##  No row has an axial component except the two limit rows. With limits disabled (or while inside them with room to spare)
##  the axial relative translation is free and carries no impulse: there is no motor, spring or damper in this joint.
##
## ---- Impulse signs ----
##  lower_limit_impulse() >= 0: impulse along +a on B (and -a on A). upper_limit_impulse() >= 0: along -a on B (+a on A).
##
## ---- Known limits (shared with the hinge) ----
##  b = -a and a twist of pi also satisfy the sine-type rows (C = 0), so the joint is only meaningful for a relative rotation well
##  below 90 degrees, which holds for any joint that starts satisfied. Rows are linearized once per step (frozen Jacobian).

const ROW_LATERAL_P: int = 0
const ROW_LATERAL_Q: int = 1
const ROW_AXIS_P: int = 2
const ROW_AXIS_Q: int = 3
const ROW_TWIST: int = 4
const ROW_LOWER: int = 5
const ROW_UPPER: int = 6
const ROW_COUNT: int = 7

## Unit-length / perpendicularity tolerance for caller-supplied direction vectors: 8 float32 ulps (8 * 1.19e-7), the same
## bound as HingeJointConstraint. Anything beyond is invalid input, not rounding.
const DIRECTION_TOLERANCE: float = 1.0e-6

var local_anchor_a: Vector3 = Vector3.ZERO
var local_anchor_b: Vector3 = Vector3.ZERO
var local_axis_a: Vector3 = Vector3.RIGHT
var local_axis_b: Vector3 = Vector3.RIGHT
var local_reference_a: Vector3 = Vector3.UP
var local_reference_b: Vector3 = Vector3.UP

## Both limits are enabled together; metres on the translation coordinate s; lower <= upper, both finite.
var limit_enabled: bool = false
var lower_limit: float = 0.0
var upper_limit: float = 0.0


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
		push_error("PrismaticJoint: invalid configuration: %s" % "; ".join(errors))
		return
	# Input is valid within DIRECTION_TOLERANCE: remove the residual rounding so the frames are exactly orthonormal.
	local_axis_a = local_axis_a.normalized()
	local_axis_b = local_axis_b.normalized()
	local_reference_a = (local_reference_a - local_axis_a * local_axis_a.dot(local_reference_a)).normalized()
	local_reference_b = (local_reference_b - local_axis_b * local_axis_b.dot(local_reference_b)).normalized()


## Builds a prismatic joint from WORLD data given the bodies' current poses: `world_anchor` (a point of the axis line),
## `world_axis` (unit, positive translation direction) and `world_reference` (unit, perpendicular to the axis). The current pose
## is the reference configuration: translation 0, no lateral offset, aligned axes, zero twist. Returns null (with an error) if
## the input is invalid.
static func from_world(p_body_a: SimRigidBody, p_body_b: SimRigidBody, world_anchor: Vector3, world_axis: Vector3,
		world_reference: Vector3):
	if p_body_a == null or p_body_b == null:
		push_error("PrismaticJoint.from_world: bodies must not be null.")
		return null
	var inv_a: Quaternion = p_body_a.orientation.inverse()
	var inv_b: Quaternion = p_body_b.orientation.inverse()
	var joint = (load("res://Simulation/Solver/prismatic_joint_constraint.gd")).new(
		p_body_a, p_body_b,
		inv_a * (world_anchor - p_body_a.position), inv_b * (world_anchor - p_body_b.position),
		inv_a * world_axis, inv_b * world_axis,
		inv_a * world_reference, inv_b * world_reference)
	return joint if joint.is_valid() else null


func validate() -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if body_a == null or body_b == null:
		errors.append("prismatic joint bodies must not be null")
		return errors
	if body_a == body_b:
		errors.append("prismatic joint must connect two different bodies")
	if body_a.is_static() and body_b.is_static():
		errors.append("prismatic joint between two static bodies has nothing to solve")
	if not (_finite(local_anchor_a) and _finite(local_anchor_b)):
		errors.append("prismatic joint anchors must be finite")
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


## d = anchor_b - anchor_a in world space (axial part s plus the lateral error).
func linear_error() -> Vector3:
	return world_anchor_b() - world_anchor_a()


## World joint axis (A side): the positive translation direction.
func world_axis_a() -> Vector3:
	return body_a.orientation * local_axis_a


func world_axis_b() -> Vector3:
	return body_b.orientation * local_axis_b


## Signed translation coordinate s = d . a in metres (see header).
func translation() -> float:
	return linear_error().dot(world_axis_a())


## Anchor offset perpendicular to the axis (world vector); zero when the lateral rows are satisfied.
func lateral_error() -> Vector3:
	var a: Vector3 = world_axis_a()
	var d: Vector3 = linear_error()
	return d - a * d.dot(a)


## Angle between the two joint axes in radians (>= 0), robust for small values.
func axis_error_angle() -> float:
	var a: Vector3 = world_axis_a()
	var b: Vector3 = world_axis_b()
	return atan2(a.cross(b).length(), a.dot(b))


## Signed relative twist about the axis in radians (atan2 form, wrapped to [-PI, PI]); zero when the twist row is satisfied.
func twist_error_angle() -> float:
	var a: Vector3 = world_axis_a()
	var p: Vector3 = body_a.orientation * local_reference_a
	var q: Vector3 = a.cross(p)
	var w: Vector3 = body_b.orientation * local_reference_b
	return atan2(w.dot(q), w.dot(p))


## Exact rate of the translation coordinate: s_dot = a . d_dot + d . (w_a x a), in m/s.
func translation_rate() -> float:
	var a: Vector3 = world_axis_a()
	var r_a: Vector3 = body_a.orientation * local_anchor_a
	var r_b: Vector3 = body_b.orientation * local_anchor_b
	var d: Vector3 = linear_error()
	var d_dot: Vector3 = (body_b.linear_velocity + body_b.angular_velocity.cross(r_b)) - (body_a.linear_velocity + body_a.angular_velocity.cross(r_a))
	return a.dot(d_dot) + d.dot(body_a.angular_velocity.cross(a))


## Last step's accumulated limit impulses (N s), >= 0. Meaningful for the last step only.
func lower_limit_impulse() -> float:
	return rows[ROW_LOWER].accumulated_impulse


func upper_limit_impulse() -> float:
	return rows[ROW_UPPER].accumulated_impulse


# --- Solver interface ---

func _update_rows(dt: float, config) -> void:
	var beta_over_dt: float = config.baumgarte_factor / dt
	var r_a: Vector3 = body_a.orientation * local_anchor_a
	var r_b: Vector3 = body_b.orientation * local_anchor_b
	var a: Vector3 = body_a.orientation * local_axis_a
	var p: Vector3 = body_a.orientation * local_reference_a
	var q: Vector3 = a.cross(p)
	var b: Vector3 = body_b.orientation * local_axis_b
	var w: Vector3 = body_b.orientation * local_reference_b
	var d: Vector3 = (body_b.position + r_b) - (body_a.position + r_a)

	# Lateral rows: anchor offset along A's moving frame vectors p and q.
	_set_lateral_row(rows[ROW_LATERAL_P], p, r_a, r_b, d, beta_over_dt)
	_set_lateral_row(rows[ROW_LATERAL_Q], q, r_a, r_b, d, beta_over_dt)

	# Axis alignment rows (identical to the hinge's).
	_set_rotation_row(rows[ROW_AXIS_P], p.cross(b), b.cross(p), beta_over_dt * p.dot(b))
	_set_rotation_row(rows[ROW_AXIS_Q], q.cross(b), b.cross(q), beta_over_dt * q.dot(b))

	# Twist row: w.q, w rigid in B, q rigid in A.
	_set_rotation_row(rows[ROW_TWIST], q.cross(w), w.cross(q), beta_over_dt * w.dot(q))

	# Limit rows: Jacobian of s_dot (lower) and its negative (upper).
	var s: float = d.dot(a)
	var lower_row = rows[ROW_LOWER]
	var upper_row = rows[ROW_UPPER]
	_set_axial_row(lower_row, a, r_a, r_b, d, 1.0)
	_set_axial_row(upper_row, a, r_a, r_b, d, -1.0)
	if limit_enabled:
		var gap_lower: float = s - lower_limit
		var gap_upper: float = upper_limit - s
		lower_row.bias = _limit_bias(gap_lower, dt)
		upper_row.bias = _limit_bias(gap_upper, dt)
		lower_row.min_impulse = 0.0
		lower_row.max_impulse = INF
		upper_row.min_impulse = 0.0
		upper_row.max_impulse = INF
		_set_limit_split(lower_row, gap_lower, beta_over_dt)
		_set_limit_split(upper_row, gap_upper, beta_over_dt)
	else:
		_deactivate(lower_row)
		_deactivate(upper_row)


## Physical limit bias: speculative C/dt while the limit is not violated, 0 once it is (continuous at C = 0). The positional
## correction of a violated limit goes through the split channel (_set_limit_split).
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


## Lateral row along the A-frame vector `e`: C = d . e.  Jw_a includes (e x d) because e turns with A.
static func _set_lateral_row(row, e: Vector3, r_a: Vector3, r_b: Vector3, d: Vector3, beta_over_dt: float) -> void:
	row.linear_a = -e
	row.angular_a = -((r_a + d).cross(e))
	row.linear_b = e
	row.angular_b = r_b.cross(e)
	row.bias = 0.0
	row.softness = 0.0
	row.min_impulse = -INF
	row.max_impulse = INF
	_set_split(row, beta_over_dt * d.dot(e), -INF, INF)


## Pure angular row (no linear part): bilateral, physical bias 0, positional correction through the split channel.
static func _set_rotation_row(row, angular_a: Vector3, angular_b: Vector3, split_bias: float) -> void:
	row.linear_a = Vector3.ZERO
	row.linear_b = Vector3.ZERO
	row.angular_a = angular_a
	row.angular_b = angular_b
	row.bias = 0.0
	row.softness = 0.0
	row.min_impulse = -INF
	row.max_impulse = INF
	_set_split(row, split_bias, -INF, INF)


## Row of the translation rate: C_dot = sign * s_dot, with s_dot = a.(v_b + w_b x r_b - v_a - w_a x r_a) + w_a . (a x d).
static func _set_axial_row(row, a: Vector3, r_a: Vector3, r_b: Vector3, d: Vector3, direction_sign: float) -> void:
	row.linear_a = -a * direction_sign
	row.angular_a = -((r_a + d).cross(a)) * direction_sign
	row.linear_b = a * direction_sign
	row.angular_b = r_b.cross(a) * direction_sign
	row.softness = 0.0


## Inactive row: bounds [0, 0] make ConstraintRow clamp any carried impulse to zero.
static func _deactivate(row) -> void:
	row.bias = 0.0
	row.min_impulse = 0.0
	row.max_impulse = 0.0
	row.split_enabled = false
