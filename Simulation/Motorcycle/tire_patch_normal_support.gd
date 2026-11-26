extends "res://Simulation/Solver/sim_constraint.gd"
## M2.5.3 Stage B: area-weighted compliant NORMAL support of one finite-width tyre on a static ground plane.
## ONE aggregate constraint per wheel (body A = static ground, body B = wheel) that owns one scalar compliant normal row per
## physically compressed tyre-surface sample of TirePatchSampler (Stage A). Motorcycle-level; the generic solver, ContactConstraint,
## ConstraintRow and SimWorld are NOT changed: the rows use the existing `softness` (regularisation) and `bias` fields.
## It carries NO tangential row, no slip, no rolling resistance and no tyre-force model, and it is not registered by the motorcycle
## builder (Stage D). Units and equations: PHYSICS_CONVENTIONS.md ("Tyre patch normal support (M2.5.3 Stage B)").
##
## ---- Physical model (per sample i, represented surface area A_i in m^2, from Stage A) ----
##   gap       g_i = n . (x_i - p0)        signed distance of the sample from the plane along the ground normal n (m); g_i < 0 compressed
##   compression   delta_i = -g_i          (m); a sample is PHYSICALLY ACTIVE if and only if g_i < 0 (strictly) at the start of the step
##   relative normal velocity  vn_i = n . (v_wheel(x_i) - v_ground(x_i))   (m/s), > 0 separating
##   effective stiffness  k_i = k_A A_i   (N/m)      k_A = `stiffness_density`  [N/m^3 = Pa/m: pressure per metre of compression]
##   effective damping    c_i = c_A A_i   (N s/m)    c_A = `damping_density`    [N s/m^3 = Pa s/m: pressure per m/s]
##   sample force along n:  F_i = max(0, k_i delta_i - c_i vn_i)   (N), applied at x_i (the wheel gets +n F_i, the ground -n F_i)
## The aggregate load is N = sum_i F_i (N), the aggregate moment about a point q is sum_i (x_i - q) x n F_i. Because k_i and c_i scale
## with A_i, N -> k_A * integral(delta dA) + c_A * integral(-vn dA) as the sampling is refined: sampling density never multiplies the
## stiffness (a fixed per-sample spring would). The sum of the weights is the tread area, not the projected footprint: k_A is a
## stiffness per unit TREAD SURFACE area and the force is taken along the ground normal (documented approximation).
##
## ---- Solver form (implicit spring-damper through the existing soft row) ----
## Backward Euler for one sample over the fixed step dt: lambda = dt F(g + dt vn'), lambda in N s, vn' the end-of-step normal velocity.
##   lambda = -dt k_i g - dt (c_i + dt k_i) vn'     <=>     vn' + gamma lambda + bias = 0,
##   gamma = 1 / (dt (c_i + dt k_i)),   bias = dt k_i gamma g = k_i g / (c_i + dt k_i).
## This is exactly the row equation Cdot + bias + softness * lambda = 0 of ConstraintRow (softness = gamma, effective mass 1/(K + gamma)),
## with the row bound lambda in [0, +INF) (the ground pushes, never pulls). The load of a sample is lambda_i / dt (N); the solver
## impulse of a step therefore IS the physical load times dt, never a stored force. gamma is not a tuning value: it follows from
## k_A, c_A, A_i and dt. Unconditionally stable for any positive k_i, c_i (implicit). The implicit look-ahead adds a numerical damping dt k_i, so the
## effective damping per sample is c_i + dt k_i (c_A + dt k_A per unit area): for k_A = 4e6 N/m^3 and dt = 1/120 s that is 3.3e4 N s/m^3.
##
## ---- Spin about the axle is excluded from the normal rows (rolling resistance is NOT hidden in the normal support) ----
## The tread is a surface of revolution about the axle u: a spin about u maps the surface onto itself, so it cannot change where the surface
## meets the plane, and the sample velocity it induces is TANGENT to the surface. Letting that velocity enter vn_i would make every
## tread sample at the front of the footprint "approach" the ground and every sample at the rear "recede" at omega * z_i, so the damper and
## the implicit look-ahead (dt k vn') would load the front of the footprint more than the rear: a hysteretic moment about the axle, i.e. a
## rolling-resistance-like moment of 13 % to 20 % of N R in a probe at 30 to 60 rad/s (c_A = 0 to 1e5), tied to dt, and +28 % to +416 % load. That belongs to
## the tyre model (Stage C), not to the normal support. Each sample row therefore uses the angular Jacobian with its axle component
## removed:  J_w = (r x n) - u (u . (r x n)),  so Cdot = n . v_c + (omega - (omega . u) u) . (r x n)  (the spin does not enter) and the
## row exerts no moment about u. The single-point contact has the same property (its contact point is directly below the hub). For the
## quasi-static footprint the removed part sums to zero anyway: the footprint is symmetric about the plane through u and the in-plane
## direction of n, so the force line meets the axle. `applied_wheel_moment()` is what the wheel receives; `total_moment_world()` is the
## geometric moment of the same forces (equal for the footprint, up to that axle component).
##
## ---- Excessive penetration (bottoming) ----
## A sample compressed by more than `max_compression` (m) at the start of the step is BOTTOMED: its row becomes the rigid non-penetration
## row of the generic contact (gamma = 0, bias = 0, vn' >= 0, bound [0, +INF)) with a split-impulse positional correction toward the
## compression `max_compression`: split_bias = (beta/dt)(g + max_compression), beta = config.baumgarte_factor, split bounds [0, +INF).
## The correction moves the pose and never adds physical velocity. A bottomed sample's impulse is physical load like any other (a hard
## stop), so the load is not continuous at max_compression. Penetration is bounded by max_compression plus at most ONE step of
## approach (|vn| dt): a sample is classified from the start-of-step gap, and there is deliberately NO speculative row (see below).
##
## ---- Speculative contact is not part of this model (M2.4.2.1 lifecycle preserved) ----
## A compliant sample needs no speculative (g > 0) collision-prevention row, and adding one would stop the tyre at zero compression so
## that the compliant support could never act. Samples with g >= 0 therefore have NO row, NO impulse, NO physical load and are NOT
## counted as active; positive-gap approach produces no response, and there is nothing to exclude from the load by bookkeeping.
##
## ---- Lifecycle (every fixed step, in prepare()) ----
## The samples are re-derived from the start-of-step pose (TirePatchSampler, Stage A; the profile is never re-derived here). Samples are
## indexed (lateral_index * circumferential_count + angular_index) and rotate with the wheel, so the index names a material point of
## the tyre: a row whose sample stays compressed keeps its accumulated impulse as the warm start; a row whose sample is no longer
## compressed is DROPPED with its impulse. When no sample is compressed (complete separation), the sampling is invalid, or the pose is
## non-finite, ALL rows and carried impulses are discarded and the load is zero: nothing stale survives into a later contact, whose
## first step starts cold (identical to a freshly created patch). SimWorld's abort calls reset_warm_start(), which clears everything.
## Non-finite carried impulses are discarded before they are applied.

const TireGeometry = preload("res://Simulation/Motorcycle/tire_geometry.gd")
const TirePatchSampler = preload("res://Simulation/Motorcycle/tire_patch_sampler.gd")

## Parameter bounds: keep every product (k_A * A_i, c_A * A_i, dt * k_i, ...) far from overflow. They are rule limits, not tuning values.
const MAX_STIFFNESS_DENSITY: float = 1.0e10
const MAX_DAMPING_DENSITY: float = 1.0e9
## The bottoming compression may be at most this fraction of the wheel radius.
const MAX_COMPRESSION_RATIO: float = 0.5
## Same unit-length tolerance as TirePatchSampler / TireGeometry for the plane normal.
const UNIT_TOLERANCE: float = 1.0e-5
## Wheel axle in the wheel's own frame (project convention: body-local X).
const AXLE_LOCAL: Vector3 = Vector3.RIGHT

var geometry: TireGeometry
var wheel_radius: float = 0.0
## A point on the plane and its unit normal n (world frame; the plane is static; n points away from the ground).
var plane_point: Vector3 = Vector3.ZERO
var plane_normal: Vector3 = Vector3.UP
## k_A (N/m^3): normal stiffness per unit tread area per metre of compression. Not a tyre-model, friction or slip stiffness.
var stiffness_density: float = 0.0
## c_A (N s/m^3): normal damping per unit tread area per m/s of approach.
var damping_density: float = 0.0
## Compression (m) beyond which a sample is bottomed (hard stop).
var max_compression: float = 0.0
var lateral_count: int = 0
var circumferential_count: int = 0

## True when the last prepared step had at least one physically compressed sample.
var contact_active: bool = false
## Why the last step produced no support ("" when it had some): the sampling failure reason or "no compressed sample".
var last_reason: String = ""

var _dt: float = 0.0
## Persistent rows keyed by sample index (the warm-start state). `rows` (base class) holds the ACTIVE rows of the last prepare, in
## ascending sample index (deterministic).
var _rows_by_index: Dictionary = {}
var _active_index: Array = []
var _active_position: Array = []
var _active_gap: Array = []
var _active_area: Array = []
var _active_bottomed: Array = []
## Angular Jacobian of each active row on the wheel (axle component removed), for applied_wheel_moment().
var _active_moment_arm: Array = []
## Physical load (N) of each active sample and the aggregate, from the last solved step.
var _sample_load: Array = []
var _load: float = 0.0


func _init(p_ground: SimRigidBody, p_wheel: SimRigidBody, p_geometry: TireGeometry, p_wheel_radius: float, p_plane_point: Vector3,
		p_plane_normal: Vector3, p_stiffness_density: float, p_damping_density: float, p_max_compression: float,
		p_lateral_count: int, p_circumferential_count: int) -> void:
	super(p_ground, p_wheel)
	geometry = p_geometry
	wheel_radius = p_wheel_radius
	plane_point = p_plane_point
	plane_normal = p_plane_normal
	stiffness_density = p_stiffness_density
	damping_density = p_damping_density
	max_compression = p_max_compression
	lateral_count = p_lateral_count
	circumferential_count = p_circumferential_count


## Builds the patch support of a wheel on a static ground plane. Returns null (with an error) for invalid input.
static func create(ground: SimRigidBody, wheel: SimRigidBody, p_geometry: TireGeometry, p_wheel_radius: float, p_plane_point: Vector3,
		p_plane_normal: Vector3, p_stiffness_density: float, p_damping_density: float, p_max_compression: float,
		p_lateral_count: int, p_circumferential_count: int):
	if ground == null or wheel == null:
		push_error("TirePatchNormalSupport.create: bodies must not be null.")
		return null
	var support = (load("res://Simulation/Motorcycle/tire_patch_normal_support.gd")).new(ground, wheel, p_geometry, p_wheel_radius,
			p_plane_point, p_plane_normal, p_stiffness_density, p_damping_density, p_max_compression, p_lateral_count, p_circumferential_count)
	var errors: PackedStringArray = support.validate()
	if not errors.is_empty():
		push_error("TirePatchNormalSupport.create: %s" % "; ".join(errors))
		return null
	support.plane_normal = p_plane_normal.normalized()
	return support


## One message per violated rule; empty = valid. The single home of the rules of this constraint's own data.
func validate() -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if body_a == null or body_b == null:
		errors.append("tire patch: bodies must not be null")
		return errors
	if body_a == body_b:
		errors.append("tire patch: the ground and the wheel must be different bodies")
	if not body_a.is_static():
		errors.append("tire patch: the ground body must be static")
	if body_b.is_static():
		errors.append("tire patch: the wheel body must be dynamic")
	var radius_ok: bool = is_finite(wheel_radius) and wheel_radius > 0.0
	if not radius_ok:
		errors.append("tire patch: wheel radius must be finite and > 0 (got %s)" % wheel_radius)
	if geometry == null:
		errors.append("tire patch: tire geometry must not be null")
	else:
		errors.append_array(geometry.validate(wheel_radius if radius_ok else 0.0))
	if not plane_point.is_finite():
		errors.append("tire patch: plane point must be finite (got %s)" % plane_point)
	if not plane_normal.is_finite():
		errors.append("tire patch: plane normal must be finite (got %s)" % plane_normal)
	elif absf(plane_normal.length() - 1.0) > UNIT_TOLERANCE:
		errors.append("tire patch: plane normal must be a unit vector (length %s)" % plane_normal.length())
	errors.append_array(TirePatchSampler.validate_counts(lateral_count, circumferential_count))
	if not (is_finite(stiffness_density) and stiffness_density > 0.0 and stiffness_density <= MAX_STIFFNESS_DENSITY):
		errors.append("tire patch: stiffness density must be finite and in (0, %s] N/m^3 (got %s)" % [MAX_STIFFNESS_DENSITY, stiffness_density])
	if not (is_finite(damping_density) and damping_density >= 0.0 and damping_density <= MAX_DAMPING_DENSITY):
		errors.append("tire patch: damping density must be finite and in [0, %s] N s/m^3 (got %s)" % [MAX_DAMPING_DENSITY, damping_density])
	if not (is_finite(max_compression) and max_compression > 0.0):
		errors.append("tire patch: maximum compression must be finite and > 0 m (got %s)" % max_compression)
	elif radius_ok and max_compression > MAX_COMPRESSION_RATIO * wheel_radius:
		errors.append("tire patch: maximum compression %s m must not exceed %s of the wheel radius (%s m)"
				% [max_compression, MAX_COMPRESSION_RATIO, MAX_COMPRESSION_RATIO * wheel_radius])
	return errors


## Implicit spring-damper coefficients of one sample: {gamma, bias_per_gap}. gamma = 1/(dt (c + dt k)) (row softness, m/(N s^2)) and
## bias = bias_per_gap * g with bias_per_gap = dt k gamma = k / (c + dt k) (1/s). NaN for non-finite or non-positive input.
static func soft_coefficients(stiffness: float, damping: float, dt: float) -> Dictionary:
	if not (is_finite(stiffness) and stiffness > 0.0 and is_finite(damping) and damping >= 0.0 and is_finite(dt) and dt > 0.0):
		return {"gamma": NAN, "bias_per_gap": NAN}
	var denominator: float = dt * (damping + dt * stiffness)
	return {"gamma": 1.0 / denominator, "bias_per_gap": dt * stiffness / denominator}


# --- Queries (results of the LAST solved step; geometry is the start-of-step state of that step) ---

## PHYSICAL normal load of the patch (N): the sum of the active samples' impulses / dt. Zero when no sample is compressed.
func normal_load() -> float:
	return _load


## True when at least one sample was physically compressed in the last prepared step.
func is_touching() -> bool:
	return contact_active


## Number of physically active (compressed) samples of the last prepared step.
func active_sample_count() -> int:
	return rows.size()


## Active samples compressed beyond max_compression (hard stop).
func bottomed_sample_count() -> int:
	var count: int = 0
	for b in _active_bottomed:
		if b:
			count += 1
	return count


## Sum of the represented areas of the active samples (m^2): the quadrature of the contact footprint on the tread.
func contact_area() -> float:
	var total: float = 0.0
	for a in _active_area:
		total += a
	return total


## Deepest start-of-step compression (m) of the last prepared step; 0 without active samples.
func max_penetration() -> float:
	var deepest: float = 0.0
	for g in _active_gap:
		deepest = maxf(deepest, -g)
	return deepest


## Aggregate force on the wheel (N, world): n * N. Along the ground normal by construction (no tangential response here).
func total_force_world() -> Vector3:
	return plane_normal * _load


## Aggregate moment on the wheel (N m, world) about the world point `about`: sum (x_i - about) x n F_i over the active samples.
func total_moment_world(about: Vector3) -> Vector3:
	var moment: Vector3 = Vector3.ZERO
	for k in _active_position.size():
		moment += (_active_position[k] - about).cross(plane_normal) * _sample_load[k]
	return moment


## Moment the patch applies to the wheel about its centre (N m, world): sum over the active samples of the axle-free arm
## (r x n) - u (u . (r x n)) times the sample load. Its component along the axle is zero by construction (see the header).
func applied_wheel_moment() -> Vector3:
	var moment: Vector3 = Vector3.ZERO
	for k in _active_moment_arm.size():
		moment += _active_moment_arm[k] * _sample_load[k]
	return moment


## Load-weighted centre of the sample loads (world); NaN vector when there is no load.
func centre_of_pressure() -> Vector3:
	if not (_load > 0.0):
		return Vector3(NAN, NAN, NAN)
	var weighted: Vector3 = Vector3.ZERO
	for k in _active_position.size():
		weighted += _active_position[k] * _sample_load[k]
	return weighted / _load


## One Dictionary per active sample, ascending sample index: index, position (world), gap (m), compression (m), area (m^2),
## load (N, last solved step), bottomed (bool).
func sample_report() -> Array:
	var report: Array = []
	for k in _active_index.size():
		report.append({"index": _active_index[k], "position": _active_position[k], "gap": _active_gap[k], "compression": -_active_gap[k],
			"area": _active_area[k], "load": _sample_load[k] if k < _sample_load.size() else 0.0, "bottomed": _active_bottomed[k]})
	return report


## Number of rows that carry a warm-start impulse between steps (the persistent per-sample state). Zero after separation.
func carried_row_count() -> int:
	return _rows_by_index.size()


## Carried (warm-start) impulse of the sample with this index (N s); 0 when the sample has no persistent row.
func carried_impulse(index: int) -> float:
	var row = _rows_by_index.get(index)
	return row.accumulated_impulse if row != null else 0.0


## Sum of |accumulated impulse| (N s) over the persistent rows. Zero after separation (no stale impulse survives).
func carried_impulse_total() -> float:
	var total: float = 0.0
	for index in _rows_by_index:
		total += absf(_rows_by_index[index].accumulated_impulse)
	return total


# --- Solver interface ---

## Re-derives the samples and rebuilds the ACTIVE rows (Jacobian, softness, bias, bounds, split correction) from the start-of-step state.
func _update_rows(dt: float, config) -> void:
	_dt = dt
	var result: Dictionary = TirePatchSampler.sample(body_b.position, body_b.orientation, geometry, wheel_radius, plane_point, plane_normal,
			lateral_count, circumferential_count)
	if not (result["valid"] and is_finite(dt) and dt > 0.0):
		last_reason = String(result["reason"]) if not result["valid"] else "invalid time step"
		_clear_state()
		return
	var n: Vector3 = plane_normal
	var axle: Vector3 = (body_b.orientation * AXLE_LOCAL).normalized()
	var kept: Dictionary = {}
	var new_rows: Array = []
	_active_index = []
	_active_position = []
	_active_gap = []
	_active_area = []
	_active_bottomed = []
	_active_moment_arm = []
	for s in result["samples"]:
		var gap: float = s["gap"]
		if not (gap < 0.0):
			continue  # positive (or zero) gap: no physical support, no row, no speculative response
		var area: float = s["weight"]
		var position: Vector3 = s["position"]
		var stiffness: float = stiffness_density * area
		var damping: float = damping_density * area
		var coefficients: Dictionary = soft_coefficients(stiffness, damping, dt)
		var gamma: float = coefficients["gamma"]
		var bias: float = coefficients["bias_per_gap"] * gap
		var bottomed: bool = -gap > max_compression
		var split_bias: float = (config.baumgarte_factor / dt) * (gap + max_compression)
		if not (is_finite(gamma) and is_finite(bias) and is_finite(split_bias)):
			continue  # degenerate sample: skipped like an uncompressed one, its carried state is dropped
		var index: int = int(s["lateral_index"]) * circumferential_count + int(s["angular_index"])
		var row = _rows_by_index.get(index)
		if row == null:
			row = ConstraintRow.new(body_a, body_b)
		elif not is_finite(row.accumulated_impulse):
			row.reset_impulses()
		var r_a: Vector3 = position - body_a.position
		var r_b: Vector3 = position - body_b.position
		var arm: Vector3 = r_b.cross(n)
		arm -= axle * axle.dot(arm)  # spin about the axle neither enters vn nor produces a moment (see the header)
		row.linear_a = -n
		row.angular_a = -r_a.cross(n)  # the ground is static: it takes no velocity change, the term only keeps the row symmetric
		row.linear_b = n
		row.angular_b = arm
		row.min_impulse = 0.0
		row.max_impulse = INF
		if bottomed:
			row.softness = 0.0
			row.bias = 0.0
			row.split_enabled = true
			row.split_bias = split_bias
			row.split_min_impulse = 0.0
			row.split_max_impulse = INF
		else:
			row.softness = gamma
			row.bias = bias
			row.split_enabled = false
		kept[index] = row
		new_rows.append(row)
		_active_index.append(index)
		_active_position.append(position)
		_active_gap.append(gap)
		_active_area.append(area)
		_active_bottomed.append(bottomed)
		_active_moment_arm.append(arm)
	if new_rows.is_empty():
		last_reason = "no compressed sample"
		_clear_state()
		return
	last_reason = ""
	contact_active = true
	_rows_by_index = kept  # rows of samples that are no longer compressed are dropped with their impulses
	rows = new_rows


func solve_iteration() -> void:
	if contact_active:
		super.solve_iteration()


func solve_split_iteration() -> void:
	if contact_active:
		super.solve_split_iteration()


## Physical load of every active sample = its accumulated impulse / dt (never a stored force).
func finish() -> void:
	_sample_load = []
	_load = 0.0
	if not contact_active or not (_dt > 0.0):
		return
	var total: float = 0.0
	for row in rows:
		var load: float = row.accumulated_impulse / _dt
		_sample_load.append(load)
		total += load
	if not is_finite(total):
		_clear_state()
		return
	_load = total


## SimWorld calls it after an aborted step: every carried impulse and every derived result is discarded.
func reset_warm_start() -> void:
	super()
	_clear_state()


## Discards every row, carried impulse and derived result (separation, invalid input, aborted step).
func _clear_state() -> void:
	contact_active = false
	_rows_by_index = {}
	rows = []
	_active_index = []
	_active_position = []
	_active_gap = []
	_active_area = []
	_active_bottomed = []
	_active_moment_arm = []
	_sample_load = []
	_load = 0.0
