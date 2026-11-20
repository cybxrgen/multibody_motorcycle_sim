extends "res://Simulation/Solver/sim_constraint.gd"
## Generic one-point contact between two rigid bodies: unilateral normal constraint + isotropic Coulomb friction.
## The caller supplies the contact geometry and creates, refreshes and removes the constraint (no collision detection here).
## Either body may be static (not both). No vehicle/tire/road logic. Equations: PHYSICS_CONVENTIONS.md ("Contact").
##
## ---- Contact data ----
##   local_anchor_a / local_anchor_b   contact point on each body's surface, in that body's LOCAL frame
##   local_normal_a                    unit contact normal in the LOCAL frame of body A; points from A toward B
##   friction                          Coulomb coefficient mu >= 0 (finite)
##   tangent_rotation                  rotation (rad) of the tangent basis about the normal; physical results do not
##                                     depend on it (see "Tangent basis")
## World quantities (R = body orientation):
##   pa = x_a + R_a*anchor_a, pb = x_b + R_b*anchor_b,  n = R_a*normal_a
##   gap g = n . (pb - pa):  g > 0 separated, g < 0 penetrating (consequence of "n points from A toward B")
##   shared contact point p = (pa + pb)/2 (midpoint; equals both anchors when g = 0 and the surfaces touch at a point).
##   All Jacobians use lever arms r_a = p - x_a, r_b = p - x_b, so the equal-and-opposite impulses act at the same point.
##   Any tangential offset between pa and pb is not constrained (the contact never pulls the points together sideways).
##   Normal relative velocity  vn = n . (v_b(p) - v_a(p)):  vn > 0 separating, vn < 0 approaching.
##
## ---- Rows (always 3) ----
##  0  normal     Jv_a = -n, Jw_a = -(r_a x n), Jv_b = +n, Jw_b = r_b x n.  Cdot = vn.  Impulse bounds [0, +INF):
##                the contact can push B along +n (and A along -n) but never pull.
##                Physical row solve enforces  Cdot + bias >= 0, i.e. vn >= -bias:
##                  g >= 0: bias = g/dt        speculative: vn >= -g/dt, the bodies may approach at most fast enough to close
##                                             the gap exactly this step; if they are slower, no impulse (the row is inactive)
##                  g <  0: bias = 0           the bodies may not approach further (vn >= 0); no positional term, so the
##                                             physical impulse never adds separating velocity
##                The physical bias is continuous at g = 0 (g/dt -> 0), as for the hinge limit rows.
##                Split channel (M1.3, positional correction only), active only while penetrating (g < 0):
##                  split_bias = (beta/dt) g (negative, beta = config.baumgarte_factor), bounds [0, +INF): the split velocities
##                  separate the bodies at vn_split >= beta*|g|/dt. This moves the pose and never the physical velocity.
##  1,2 tangent   t1, t2 in the plane perpendicular to n; same Jacobian pattern as row 0 with t instead of n,
##                bias 0 (friction drives the tangential relative velocity at p to zero). They are NOT solved as scalar
##                rows: the pair is solved as one 2x2 block (solve_iteration) and projected onto a disk.
##
## ---- Friction (isotropic Coulomb: |lambda_t| <= mu * lambda_n) ----
##  lambda_t = (lt1, lt2) are the accumulated tangent impulses (rows 1, 2), |lambda_t| their Euclidean norm; lambda_n is the
##  CURRENT accumulated normal impulse. Per solver iteration (solve_iteration):
##   1. solve the normal row (scalar, clamped to [0, INF));
##   2. K = [J_i M^-1 J_j^T]_{ij}, c = (Cdot_t1, Cdot_t2);  y = lambda_t - K^-1 c  (unconstrained block update);
##   3. lambda_t <- argmin_{|x| <= mu*lambda_n} (x - y)^T K (x - y)  (K-metric projection onto the disk; y itself if inside).
##      The K-metric (not the Euclidean) projection is the exact block Gauss-Seidel minimizer of the friction subproblem and
##      keeps the result independent of the tangent basis. It is computed by bisection on the multiplier nu in
##      x(nu) = (K + nu I)^-1 K y, whose norm decreases monotonically in nu; the upper end of the bracket is used, so the bound is
##      never exceeded. mu*lambda_n = 0 gives lambda_t = 0: no normal impulse, no friction.
##   4. apply only the change of lambda_t to the bodies.
##  Sticking = y inside the disk (the tangential velocity is driven to zero); sliding = the disk boundary is active.
##
## ---- Tangent basis ----
##  e = world axis with the smallest |n . e| (so n x e is never degenerate); t1 = normalize(n x e), t2 = n x t1; (t1, t2, n) is
##  right-handed. Both are then rotated about n by tangent_rotation. Because the block solve and the disk are rotation invariant,
##  the physical result does not depend on this choice.
##
## ---- Warm starting ----
##  The normal row is warm started like every row (impulse scaled by warm_start_factor, clamped to [0, INF)). The friction
##  impulse is stored between steps as a WORLD vector (the basis may change when n changes), scaled by warm_start_factor,
##  projected into the CURRENT tangent plane (normal component removed), and then radially clipped to the disk of radius
##  mu * (warm-started lambda_n) before it is applied. So the carried friction impulse is admissible for the current normal
##  impulse and friction coefficient. Without warm starting both impulses start at zero.

const ROW_NORMAL: int = 0
const ROW_TANGENT_1: int = 1
const ROW_TANGENT_2: int = 2
const ROW_COUNT: int = 3

## Unit-length tolerance for the caller-supplied normal: 8 float32 ulps, as for the hinge directions.
const DIRECTION_TOLERANCE: float = 1.0e-6
## Bisection steps for the friction projection (bracket width shrinks by 2^-60, below float64 resolution of the result).
const PROJECTION_ITERATIONS: int = 60

var local_anchor_a: Vector3 = Vector3.ZERO
var local_anchor_b: Vector3 = Vector3.ZERO
var local_normal_a: Vector3 = Vector3.UP
var friction: float = 0.0
var tangent_rotation: float = 0.0

var _point: Vector3 = Vector3.ZERO
var _normal: Vector3 = Vector3.UP
var _gap: float = 0.0
var _t1: Vector3 = Vector3.RIGHT
var _t2: Vector3 = Vector3.BACK
## Friction impulse of the last solved step as a world vector (the warm-start state).
var _friction_impulse: Vector3 = Vector3.ZERO
var _k00: float = 0.0
var _k01: float = 0.0
var _k11: float = 0.0
var _k_valid: bool = false


func _init(p_body_a: SimRigidBody, p_body_b: SimRigidBody, p_local_anchor_a: Vector3, p_local_anchor_b: Vector3,
		p_local_normal_a: Vector3, p_friction: float) -> void:
	super(p_body_a, p_body_b)
	local_anchor_a = p_local_anchor_a
	local_anchor_b = p_local_anchor_b
	local_normal_a = p_local_normal_a
	friction = p_friction
	for i in ROW_COUNT:
		add_row()
	var errors: PackedStringArray = validate()
	if not errors.is_empty():
		push_error("ContactConstraint: invalid configuration: %s" % "; ".join(errors))
		return
	# Valid within DIRECTION_TOLERANCE: remove the residual rounding so the normal is exactly unit length.
	local_normal_a = local_normal_a.normalized()


## Builds a contact from WORLD data given the bodies' current poses: `world_point_a` / `world_point_b` are the contact points
## on A and B, `world_normal` the unit normal from A toward B. Returns null (with an error) if the input is invalid.
static func from_world(p_body_a: SimRigidBody, p_body_b: SimRigidBody, world_point_a: Vector3, world_point_b: Vector3,
		world_normal: Vector3, p_friction: float):
	if p_body_a == null or p_body_b == null:
		push_error("ContactConstraint.from_world: bodies must not be null.")
		return null
	var inv_a: Quaternion = p_body_a.orientation.inverse()
	var inv_b: Quaternion = p_body_b.orientation.inverse()
	var contact = (load("res://Simulation/Solver/contact_constraint.gd")).new(
		p_body_a, p_body_b,
		inv_a * (world_point_a - p_body_a.position), inv_b * (world_point_b - p_body_b.position),
		inv_a * world_normal, p_friction)
	return contact if contact.is_valid() else null


func validate() -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if body_a == null or body_b == null:
		errors.append("contact bodies must not be null")
		return errors
	if body_a == body_b:
		errors.append("contact must connect two different bodies")
	if body_a.is_static() and body_b.is_static():
		errors.append("contact between two static bodies has nothing to solve")
	if not (_finite(local_anchor_a) and _finite(local_anchor_b)):
		errors.append("contact anchors must be finite")
	if not _finite(local_normal_a):
		errors.append("contact normal must be finite")
	elif local_normal_a.length() == 0.0:
		errors.append("contact normal must be nonzero")
	elif absf(local_normal_a.length() - 1.0) > DIRECTION_TOLERANCE:
		errors.append("contact normal must be a unit vector (length %s)" % local_normal_a.length())
	if not (is_finite(friction) and friction >= 0.0):
		errors.append("friction must be finite and >= 0 (got %s)" % friction)
	if not is_finite(tangent_rotation):
		errors.append("tangent_rotation must be finite (got %s)" % tangent_rotation)
	return errors


static func _finite(v: Vector3) -> bool:
	return is_finite(v.x) and is_finite(v.y) and is_finite(v.z)


# --- Queries (use the current body poses / the last solved step) ---

func world_point_a() -> Vector3:
	return body_a.position + body_a.orientation * local_anchor_a


func world_point_b() -> Vector3:
	return body_b.position + body_b.orientation * local_anchor_b


## Contact normal in world space, from A toward B.
func world_normal() -> Vector3:
	return (body_a.orientation * local_normal_a).normalized()


## Shared world contact point (midpoint of the two anchor points).
func world_contact_point() -> Vector3:
	return (world_point_a() + world_point_b()) * 0.5


## Signed gap along the normal: > 0 separated, < 0 penetrating.
func gap() -> float:
	return world_normal().dot(world_point_b() - world_point_a())


## Relative velocity of B's contact point minus A's, at the shared point, world space.
func relative_contact_velocity() -> Vector3:
	var p: Vector3 = world_contact_point()
	return body_b.velocity_at_point(p) - body_a.velocity_at_point(p)


## Normal relative velocity: > 0 separating, < 0 approaching.
func normal_velocity() -> float:
	return relative_contact_velocity().dot(world_normal())


## Tangential (slip) relative velocity: the relative contact velocity without its normal component.
func tangential_velocity() -> Vector3:
	var n: Vector3 = world_normal()
	var v: Vector3 = relative_contact_velocity()
	return v - n * v.dot(n)


## Last step's accumulated normal impulse (N s), always >= 0. Meaningful for the last step only.
func normal_impulse() -> float:
	return rows[ROW_NORMAL].accumulated_impulse


## Last step's accumulated friction impulse on B as a world vector (perpendicular to the normal).
func friction_impulse_world() -> Vector3:
	return _t1 * rows[ROW_TANGENT_1].accumulated_impulse + _t2 * rows[ROW_TANGENT_2].accumulated_impulse


## |lambda_t|, the norm of the accumulated tangent impulse (the quantity bounded by mu * lambda_n).
func friction_impulse_magnitude() -> float:
	var l1: float = rows[ROW_TANGENT_1].accumulated_impulse
	var l2: float = rows[ROW_TANGENT_2].accumulated_impulse
	return sqrt(l1 * l1 + l2 * l2)


## Current Coulomb bound mu * lambda_n.
func friction_limit() -> float:
	return friction * rows[ROW_NORMAL].accumulated_impulse


# --- Solver interface ---

func _update_rows(dt: float, config) -> void:
	var pa: Vector3 = world_point_a()
	var pb: Vector3 = world_point_b()
	_point = (pa + pb) * 0.5
	_normal = world_normal()
	_gap = _normal.dot(pb - pa)
	_build_tangent_basis()
	var r_a: Vector3 = _point - body_a.position
	var r_b: Vector3 = _point - body_b.position

	var normal_row = rows[ROW_NORMAL]
	_set_row(normal_row, _normal, r_a, r_b)
	normal_row.bias = _gap / dt if _gap >= 0.0 else 0.0
	normal_row.min_impulse = 0.0
	normal_row.max_impulse = INF
	if _gap < 0.0:
		normal_row.split_enabled = true
		normal_row.split_bias = (config.baumgarte_factor / dt) * _gap
		normal_row.split_min_impulse = 0.0
		normal_row.split_max_impulse = INF
	else:
		normal_row.split_enabled = false

	_set_row(rows[ROW_TANGENT_1], _t1, r_a, r_b)
	_set_row(rows[ROW_TANGENT_2], _t2, r_a, r_b)
	for i in [ROW_TANGENT_1, ROW_TANGENT_2]:
		var row = rows[i]
		row.bias = 0.0
		row.min_impulse = -INF
		row.max_impulse = INF

	_k00 = _block_k(rows[ROW_TANGENT_1], rows[ROW_TANGENT_1])
	_k01 = _block_k(rows[ROW_TANGENT_1], rows[ROW_TANGENT_2])
	_k11 = _block_k(rows[ROW_TANGENT_2], rows[ROW_TANGENT_2])
	var det: float = _k00 * _k11 - _k01 * _k01
	_k_valid = det > 0.0 and is_finite(det) and (_k00 + _k11) > config.min_row_denominator


func prepare(dt: float, config) -> void:
	_update_rows(dt, config)
	var normal_row = rows[ROW_NORMAL]
	normal_row.prepare(config.min_row_denominator)
	normal_row.warm_start(config.warm_starting, config.warm_start_factor)
	var row_1 = rows[ROW_TANGENT_1]
	var row_2 = rows[ROW_TANGENT_2]
	row_1.accumulated_impulse = 0.0
	row_2.accumulated_impulse = 0.0
	if not config.warm_starting:
		return
	# Carried friction impulse: scale, project into the current tangent plane, clip to the current Coulomb disk.
	var carried: Vector3 = _friction_impulse * config.warm_start_factor
	carried -= _normal * carried.dot(_normal)
	var l1: float = carried.dot(_t1)
	var l2: float = carried.dot(_t2)
	var limit: float = friction * normal_row.accumulated_impulse
	var length: float = sqrt(l1 * l1 + l2 * l2)
	if length > limit:
		var scale: float = limit / length if length > 0.0 else 0.0
		l1 *= scale
		l2 *= scale
	row_1.accumulated_impulse = l1
	row_2.accumulated_impulse = l2
	row_1.apply_impulse(l1)
	row_2.apply_impulse(l2)


## Normal row first, so the friction bound uses the current normal impulse, then the coupled friction block.
## The friction bound reads the PHYSICAL normal row's accumulated impulse only; the split normal impulse lives in a separate
## field (split_accumulated_impulse) that friction never sees.
func solve_iteration() -> void:
	rows[ROW_NORMAL].solve()
	_solve_friction()


## Split pass: only the normal row has a split channel (friction is physical), and only while penetrating.
func solve_split_iteration() -> void:
	rows[ROW_NORMAL].solve_split()


func finish() -> void:
	_friction_impulse = friction_impulse_world()


func _solve_friction() -> void:
	if not _k_valid:
		return
	var row_1 = rows[ROW_TANGENT_1]
	var row_2 = rows[ROW_TANGENT_2]
	var c1: float = row_1.compute_velocity()
	var c2: float = row_2.compute_velocity()
	var det: float = _k00 * _k11 - _k01 * _k01
	# y = lambda_t - K^-1 c
	var y1: float = row_1.accumulated_impulse - (_k11 * c1 - _k01 * c2) / det
	var y2: float = row_2.accumulated_impulse - (-_k01 * c1 + _k00 * c2) / det
	var limit: float = friction * rows[ROW_NORMAL].accumulated_impulse
	var x: Array = _project_to_disk(y1, y2, limit)
	var d1: float = x[0] - row_1.accumulated_impulse
	var d2: float = x[1] - row_2.accumulated_impulse
	row_1.accumulated_impulse = x[0]
	row_2.accumulated_impulse = x[1]
	row_1.apply_impulse(d1)
	row_2.apply_impulse(d2)


## argmin over |x| <= radius of (x - y)^T K (x - y), K = [[k00, k01], [k01, k11]] (symmetric positive definite).
## Returns y when it is inside the disk. Lagrange condition K (x - y) + nu x = 0  =>  x(nu) = (K + nu I)^-1 K y with |x(nu)|
## strictly decreasing in nu >= 0; bisect |x(nu)| = radius and return the end of the bracket that is inside the disk.
func _project_to_disk(y1: float, y2: float, radius: float) -> Array:
	if radius <= 0.0:
		return [0.0, 0.0]
	if y1 * y1 + y2 * y2 <= radius * radius:
		return [y1, y2]
	var lo: float = 0.0
	var hi: float = _k00 + _k11  # trace >= largest eigenvalue
	var guard: int = 0
	while _disk_norm(hi, y1, y2) > radius and guard < 200:
		hi *= 2.0
		guard += 1
	for i in PROJECTION_ITERATIONS:
		var mid: float = 0.5 * (lo + hi)
		if _disk_norm(mid, y1, y2) > radius:
			lo = mid
		else:
			hi = mid
	return _disk_point(hi, y1, y2)


func _disk_point(nu: float, y1: float, y2: float) -> Array:
	var r1: float = _k00 * y1 + _k01 * y2
	var r2: float = _k01 * y1 + _k11 * y2
	var m00: float = _k00 + nu
	var m11: float = _k11 + nu
	var det: float = m00 * m11 - _k01 * _k01
	return [(m11 * r1 - _k01 * r2) / det, (-_k01 * r1 + m00 * r2) / det]


func _disk_norm(nu: float, y1: float, y2: float) -> float:
	var x: Array = _disk_point(nu, y1, y2)
	return sqrt(x[0] * x[0] + x[1] * x[1])


func _build_tangent_basis() -> void:
	var ax: float = absf(_normal.x)
	var ay: float = absf(_normal.y)
	var az: float = absf(_normal.z)
	var e: Vector3 = Vector3.RIGHT
	if ay <= ax and ay <= az:
		e = Vector3.UP
	elif az <= ax and az <= ay:
		e = Vector3.BACK
	var t1: Vector3 = _normal.cross(e).normalized()
	var t2: Vector3 = _normal.cross(t1)
	var c: float = cos(tangent_rotation)
	var s: float = sin(tangent_rotation)
	_t1 = t1 * c + t2 * s
	_t2 = t2 * c - t1 * s


static func _set_row(row, direction: Vector3, r_a: Vector3, r_b: Vector3) -> void:
	row.linear_a = -direction
	row.angular_a = -r_a.cross(direction)
	row.linear_b = direction
	row.angular_b = r_b.cross(direction)
	row.softness = 0.0


## K_ij = J_i M^-1 J_j^T for two rows of this constraint (current world inverse masses / inertias).
func _block_k(ri, rj) -> float:
	return body_a.inverse_mass * ri.linear_a.dot(rj.linear_a) \
		+ ri.angular_a.dot(body_a.inverse_inertia_world * rj.angular_a) \
		+ body_b.inverse_mass * ri.linear_b.dot(rj.linear_b) \
		+ ri.angular_b.dot(body_b.inverse_inertia_world * rj.angular_b)
