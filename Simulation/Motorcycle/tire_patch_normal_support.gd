extends "res://Simulation/Solver/sim_constraint.gd"
## M2.5.3 Stage B: area-weighted compliant NORMAL support of one finite-width tyre on a static ground plane.
## ONE aggregate constraint per wheel (body A = static ground, body B = wheel) that owns one scalar compliant normal row per
## physically compressed tyre-surface sample of TirePatchSampler (Stage A). Motorcycle-level; the generic solver, ContactConstraint,
## ConstraintRow and SimWorld are NOT changed: the rows are VelocityJacobianRow (a ConstraintRow subclass that adds a velocity-side angular
## Jacobian) and use the existing `softness` (regularisation) and `bias` fields.
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
## ---- Two Jacobians per row: the physical wrench and the compression-rate map (VelocityJacobianRow) ----
## IMPULSE SIDE (the physical wrench). A normal impulse lambda_i n acting at the sample position x_i on the wheel (centre p, r_i = x_i - p) is
## the wrench (force n lambda_i, moment r_i x n lambda_i about p), so the impulse Jacobian is exactly  J_imp_lin = n,  J_imp_ang = r_i x n,
## with no component removed: the angular impulse the solver applies (warm start and every solve) is the moment of the normal impulse at its
## actual sample position, and `total_moment_world(wheel.position)` and `applied_wheel_moment()` are the same sum  sum_i (r_i x n) F_i
## (applied_wheel_moment reads the installed rows' impulse angular Jacobians, total_moment_world the sample positions; they agree to
## rounding). The axle component (r_i x n) . u of a sample moment is real: it is the fore-aft offset of the sample's line of action from the
## wheel centre times the load (zero for the quasi-static footprint, which is symmetric fore-aft, and non-zero whenever the loads are fore-aft
## asymmetric).
## VELOCITY SIDE (compression rate and effective mass). The tread is a surface of revolution about the axle u: a spin maps the surface onto
## itself, so the velocity that a spin induces at a sample, (omega . u)(u . (r_i x n)) in the row rate, is the TANGENT material
## velocity of the tread, not a rate of compression. Left in the rate, it makes the damper and the implicit look-ahead (dt k vn') load the
## leading samples of the footprint more than the trailing ones (probe, full Jacobian for everything, 17x96 samples, 2 cm compression, rolling
## at 30 and 60 rad/s: load +27 % and +76 % with c_A = 0, +185 % and +412 % with c_A = 1e5, and a moment about the axle of 13 % to 20 % of
## N R, a hidden rolling resistance). The row therefore measures and weighs the compression with the axle-projected angular Jacobian
##   J_vel_lin = n,   J_vel_ang = (r_i x n) - u (u . (r_i x n)),
##   vn_i = J_vel . v   (compute_velocity, from the CURRENT velocities in every iteration),   K_i = (1/m) |n|^2 + J_vel_ang . I_w^-1 . J_vel_ang
## (compute_k; effective mass 1/(K_i + gamma_i)). The two maps are deliberately different (VelocityJacobianRow): J_vel_ang is the rate map, J_imp_ang
## is the real moment, and neither is derived from the other. Because J_vel_ang is perpendicular to u, no change of the axle spin enters vn_i
## or K_i, not at preparation and not in later sequential iterations when the patch's own axle moment (or another constraint) changes the
## spin; u is a principal axis of the wheel inertia (a PRECONDITION that validate() enforces, see below), so the axle component of the wrench
## also does not leak into the off-axle angular velocity through I_w^-1. A previous version removed the initial spin from a frozen row bias; that left the effective mass
## and the later iterations on the full Jacobian (an axle-spin change still fed back into vn_i) and is gone: the bias is only the implicit
## spring term k_i g_i / (c_i + dt k_i). The Jacobians and u are those of the start of the step (frozen-Jacobian limitation of the project);
## the velocities are live. For bottomed (rigid) rows the same velocity map applies, so a spinning bottomed tyre is not pushed by tangential
## tread velocity either. Split correction: the split row uses the velocity-side Jacobian to measure and to apply (a pose correction is the gradient of the
## gap, which has no axle component; the physical wrench never enters the split velocities). No tangential row exists here: the in-plane and
## spin responses of the tyre are not modelled by this constraint (Stage C).
##
## ---- Precondition: the wheel axle is a principal axis of the wheel inertia (enforced by validate()) ----
## The effective mass uses J_vel M^-1 J_vel^T, while the exact response of the row rate to a unit impulse is J_vel M^-1 J_imp^T. They differ by
## J_vel_ang . I_w^-1 u (u . (r_i x n)): zero for every sample if and only if I_w^-1 u is parallel to u, i.e. u is a principal axis of the
## wheel inertia (equivalently I u = lambda u). SimMassProperties accepts full non-diagonal tensors, so this is NOT implied by a valid body:
## validate() rejects a wheel whose body-frame tensor couples the axle (body-local +X) to the transverse axes (axle_principal_axis_error). The
## check is made in the BODY frame, where the axle is exactly (1, 0, 0) and the tensor is the stored one: the world orientation never enters
## (I_w u = R I_local R^T R e_x = R I_local e_x, so the property is the same in every orientation, and a world-frame test would only add the
## float32 rounding of R I R^T). The wheel inertia is never projected, diagonalised or repaired: a wheel that violates the precondition is
## reported by validate() and SimWorld refuses to step it (the check is part of the ordinary constraint validation / preflight).

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
## A physically compressed sample whose DERIVED coefficients (gamma, bias, split bias) are not finite makes the whole patch INVALID for that step
## (e.g. an extremely small stiffness density with zero damping overflows gamma = 1/(dt (c + dt k)) although every input is finite and
## accepted by validate(): the bound depends on the sample area and the time step, which validate() cannot know). The patch is then cleared
## exactly like complete separation (no row, no load, no carried impulse, contact_active = false) and `last_reason` names the coefficient and
## the sample; the wheel receives nothing from the patch. A sample is never skipped as if it were uncompressed.

const TireGeometry = preload("res://Simulation/Motorcycle/tire_geometry.gd")
const TirePatchSampler = preload("res://Simulation/Motorcycle/tire_patch_sampler.gd")
const VelocityJacobianRow = preload("res://Simulation/Solver/velocity_jacobian_row.gd")

## Parameter bounds: keep every product (k_A * A_i, c_A * A_i, dt * k_i, ...) far from overflow. They are rule limits, not tuning values.
const MAX_STIFFNESS_DENSITY: float = 1.0e10
const MAX_DAMPING_DENSITY: float = 1.0e9
## The bottoming compression may be at most this fraction of the wheel radius.
const MAX_COMPRESSION_RATIO: float = 0.5
## Same unit-length tolerance as TirePatchSampler / TireGeometry for the plane normal.
const UNIT_TOLERANCE: float = 1.0e-5
## Wheel axle in the wheel's own frame (project convention: body-local X).
const AXLE_LOCAL: Vector3 = Vector3.RIGHT
## Principal-axis precondition: the products of inertia that couple the axle to the transverse axes (I_xy, I_xz of the body-frame tensor,
## symmetrised as SimMassProperties does for the stored inverse) may not exceed this fraction of the largest element of the tensor. It
## is the symmetry tolerance of SimMassProperties (1e-5, about 84 float32 ulps), which already bounds the rounding noise of a tensor
## assembled through general rotations (measured at most 2.2e-7 of the largest element over 75 such tensors, Tests O5, 45 times below
## the tolerance); a genuine coupling is orders of magnitude larger. The effective-mass error that a tolerated coupling causes is of
## the relative size coupling / axial moment (about 1e-5 or less), far below the float32 solver noise.
const AXLE_PRINCIPAL_REL_TOL: float = 1.0e-5

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
## Why the last step produced no support ("" when it had some): the sampling failure reason, "invalid time step", "no compressed sample" or
## "invalid derived coefficient ..." naming the coefficient and the sample.
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
	else:
		# Only a dynamic (hence valid) wheel has an inertia tensor to check; an invalid body is already reported by the rules above
		# and by SimWorld's mass-property check, and its zeroed tensor must not add a misleading second message.
		var axle_error: String = axle_principal_axis_error(body_b.local_inertia)
		if axle_error != "":
			errors.append(axle_error)
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


## "" when the wheel axle (AXLE_LOCAL, the body-local +X axis) is a principal axis of the body-frame inertia tensor `local_inertia`
## (I e_x parallel to e_x, i.e. I_xy = I_xz = 0), otherwise the validation message. Pure and deterministic: the verdict and the text depend only on
## the tensor. The tensor is compared in the body frame (never the world frame, see the header) and in float64 scalars; the coupling is
## the symmetrised product of inertia 0.5 (I_ab + I_ba), the same symmetrisation SimMassProperties applies to the inverse it stores, and
## the tolerance is AXLE_PRINCIPAL_REL_TOL times the largest element, so the verdict is independent of the physical scale of the tensor.
## Element (row r, column c) of a Godot Basis is basis[c][r], so I_xy is x.y (and y.x), I_xz is x.z (and z.x). A NaN coupling fails the comparison
## and is reported; finiteness of the whole tensor is the business of SimMassProperties.validate_inertia(), which every dynamic body has passed.
static func axle_principal_axis_error(local_inertia: Basis) -> String:
	var scale: float = 0.0
	for column in [local_inertia.x, local_inertia.y, local_inertia.z]:
		scale = maxf(scale, maxf(absf(column.x), maxf(absf(column.y), absf(column.z))))
	var coupling_xy: float = 0.5 * (local_inertia.x.y + local_inertia.y.x)
	var coupling_xz: float = 0.5 * (local_inertia.x.z + local_inertia.z.x)
	var coupling: float = sqrt(coupling_xy * coupling_xy + coupling_xz * coupling_xz)
	var tolerance: float = AXLE_PRINCIPAL_REL_TOL * scale
	if coupling <= tolerance:
		return ""
	return ("tire patch: the tyre patch normal-support model requires the wheel axle (body-local +X) to be a principal axis of the wheel inertia "
			+ "tensor, but the tensor couples the axle to the transverse axes (products of inertia I_xy = %s, I_xz = %s kg m^2; coupling %s kg m^2 "
			+ "exceeds the tolerance %s kg m^2 = %s of the largest tensor element). Define the wheel inertia in a body frame whose +X axis is the "
			+ "axle principal axis (I_xy = I_xz = 0), for example a diagonal tensor with the axial moment first; the inertia is not modified here.") % [
			coupling_xy, coupling_xz, coupling, tolerance, AXLE_PRINCIPAL_REL_TOL]


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


## Moment the patch applies to the wheel about its centre (N m, world): sum over the active rows of the installed wheel-side IMPULSE angular
## Jacobian times the sample load, i.e. what the solver's angular impulse delivers (domega = I_w^-1 sum_i J_ang_i lambda_i). It equals
## total_moment_world(wheel.position) up to rounding because J_ang_i = r_i x n.
func applied_wheel_moment() -> Vector3:
	var moment: Vector3 = Vector3.ZERO
	for k in mini(rows.size(), _sample_load.size()):
		moment += rows[k].angular_b * _sample_load[k]
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
	var axle: Vector3 = (body_b.orientation * AXLE_LOCAL).normalized()  # start-of-step axle u (frozen over the step, like the Jacobians)
	var kept: Dictionary = {}
	var new_rows: Array = []
	_active_index = []
	_active_position = []
	_active_gap = []
	_active_area = []
	_active_bottomed = []
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
		# A compressed sample with a non-finite derived coefficient is an invalid patch, never an inactive sample.
		var bad: String = "gamma" if not is_finite(gamma) else ("bias" if not is_finite(bias) else ("split bias" if not is_finite(split_bias) else ""))
		if bad != "":
			last_reason = "invalid derived coefficient: %s is not finite for compressed sample %d (lateral %d, angular %d; area %s m^2, stiffness %s N/m, damping %s N s/m, dt %s s)" % [
					bad, int(s["lateral_index"]) * circumferential_count + int(s["angular_index"]), int(s["lateral_index"]), int(s["angular_index"]),
					area, stiffness, damping, dt]
			_clear_state()
			return
		var index: int = int(s["lateral_index"]) * circumferential_count + int(s["angular_index"])
		var row = _rows_by_index.get(index)
		if row == null:
			row = VelocityJacobianRow.new(body_a, body_b)
		elif not is_finite(row.accumulated_impulse):
			row.reset_impulses()
		var r_a: Vector3 = position - body_a.position
		var r_b: Vector3 = position - body_b.position
		var moment_b: Vector3 = r_b.cross(n)  # r x n: the full moment of a normal impulse at the sample
		row.linear_a = -n
		row.angular_a = -r_a.cross(n)  # the ground is static: it takes no velocity change, the term only keeps the row symmetric
		row.velocity_angular_a = row.angular_a  # static ground: nothing to project
		row.linear_b = n
		row.angular_b = moment_b  # IMPULSE side: the physical wrench, never projected (see the header)
		row.velocity_angular_b = moment_b - axle * axle.dot(moment_b)  # VELOCITY side: spin about the axle is not compression
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
	_sample_load = []
	_load = 0.0
