extends "res://Simulation/Solver/sim_constraint.gd"
## M2.5.3 Stage B: area-weighted compliant NORMAL support of one finite-width tyre on a static ground plane.
## ONE aggregate constraint per wheel (body A = static ground, body B = wheel) that owns one scalar compliant normal row per
## physically compressed tyre-surface sample of TirePatchSampler (Stage A). Motorcycle-level; the generic solver, ContactConstraint,
## ConstraintRow and SimWorld are NOT changed: the rows are VelocityJacobianRow (a ConstraintRow subclass that adds a velocity-side angular
## Jacobian) and use the existing `softness` (regularisation) and `bias` fields.
## Without a tyre specification (`tire == null`, the Stage B configuration built by `create()`) it carries NO tangential row, no slip, no
## rolling resistance and no tyre-force model. With one (`create_with_tire()`, Stage C, see "Stage C" below) the SAME constraint is the sole
## owner of the wheel's tangential and rolling-resistance response. MotorcycleBuilder registers one per wheel that has a MotorcycleTirePatchSpec (Stage D), INSTEAD of the
## one-point TireGroundContact, never next to it. Units and
## equations: PHYSICS_CONVENTIONS.md ("Tyre patch normal support (M2.5.3 Stage B)" and "Tyre patch force integration (M2.5.3 Stage C)").
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
## validate() rejects a wheel whose body-frame tensor couples the axle (body-local +X) to the transverse axes, measured by the scale-independent
## residual rho = |I u - u (u^T I u)| / |I u| against AXLE_PRINCIPAL_RESIDUAL_TOL (axle_principal_axis_error). The
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
## ---- Stage C: tyre-force integration (the tyre model on the aggregate physical state of the patch) ----
## OWNERSHIP. The patch constraint owns the tangential response because the response needs the CURRENT accumulated normal impulses of the
## rows of this very constraint in every solver iteration; a second registered constraint could not see them in the same iteration without a
## solver change. There is exactly ONE tangential response and ONE rolling-resistance response per wheel, no generic friction row and no
## per-sample traction. TireModel keeps the axes, the slip definition, the combined-slip law and the rolling-resistance law (shared
## static helpers, also used by the one-point TireGroundContact); MotorcycleTireSpec keeps every parameter and its meaning.
## AGGREGATE STATE, re-derived from the current rows in EVERY iteration (after the sequential normal rows of that iteration):
##   Lambda_n = sum_i lambda_i  (N s), N = Lambda_n / dt,   p_eff = sum_i lambda_i p_i / Lambda_n   (p_i: Stage A sample positions of the
##   start-of-step pose), valid only for finite Lambda_n > 0 (otherwise the safe no-load result: no point, no tangential response).
##   v_eff = v + omega x (p_eff - x) from the CURRENT velocities. For a rigid wheel this IS the lambda-weighted mean of the sample velocities
##   (v(p) is affine in p and the weights sum to one), so no per-sample velocity is evaluated. The approximation is the single resultant
##   force: all samples are driven by the weighted mean slip, so the variation of the slip across the patch (yaw-rate and roll-rate
##   "spin slip", differing tread speeds of cambered shoulders) and any moment about the contact normal (self-aligning, turn slip) are not modelled.
## TANGENT FRAME. TireModel.axes(u, n) with the start-of-step axle u: e_x = (n x u)/|n x u| (forward), e_y = e_x x n (right), in the
## ground plane, orthonormal, following camber and yaw; degenerate (|n x u| < MIN_PLANE_PROJECTION) => no tyre force (the normal support stays).
## SLIP. s_x = -(v_eff . e_x) + (R_eff - R) Omega, s_y = -(v_eff . e_y), Omega = -(omega . u) (TireModel.slip, evaluated at p_eff and
## with the live velocities in each iteration); reference speed D = sqrt(v_hub^2 + v_ref^2) frozen at the START of the solve (before the
## normal warm start), exactly as the one-point contact does.
## SOLVER ORDER in one iteration: (1) all normal rows (Stage B, unchanged); (2) Lambda_n, p_eff from their current impulses; (3) the carried
## tangential impulse is RE-SEATED at p_eff (its angular impulse is replaced by (p_eff - x) x L, so the applied wrench is always the
## resultant force at the current effective point); (4) the 2x2 implicit response with G = S M^-1 P^T built at p_eff
## (TireModel.implicit_tangential_impulse, the M2.4.2 equations, phi lagged one iteration) and the ONE friction ellipse of the aggregate
## Lambda_n (one common reduction factor); (5) the change of the resultant is applied to the wheel as the linear impulse L = e_x
## lambda_x + e_y lambda_y and the angular impulse (p_eff - x) x L; the ground is static. The velocity-side (axle-projected) Jacobian of
## the normal rows plays no part here: the tangential wrench is the physical force at p_eff. (6) Rolling resistance: M = c_rr N R_eff
## psi(R_eff Omega / v_rr) about the axle against the frozen spin, M dt applied as the difference to what this step applied (so it follows Lambda_n
## as the rows converge), once per wheel, outside the ellipse, never warm started, exactly zero without positive load.
## WARM START. The converged tangential impulse is carried as a world vector, projected into the next tangent plane, scaled by
## warm_start_factor and clipped into the ellipse of the warm-started aggregate load, then applied at the warm-started p_eff.
## LIFECYCLE. No positive aggregate normal impulse (separation, invalid sampling, abort, reset), a non-finite aggregate or derived solver
## value, disabled tyre forces or degenerate axes: tangential impulse, rolling impulse, reported forces and the carried impulse are zero and
## nothing of the aggregate point or velocity is kept. A non-finite aggregate state is never repaired by skipping samples: the tyre response
## is withdrawn in that iteration and finish() clears the whole patch with `last_reason`. If that withdrawal is itself not representable in float32
## (a wheel state near the float32 limit), nothing is written or cleared: the response stays recorded as what the wheel carries, the failure stays
## terminal, `last_reason` and validate() report the numerical fault (SimWorld then refuses to step) until reset_warm_start() (see _withdrawal_fault).

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
const TireModel = preload("res://Simulation/Motorcycle/tire_model.gd")
const MotorcycleTireSpec = preload("res://Simulation/Motorcycle/motorcycle_tire_spec.gd")

## Parameter bounds: keep every product (k_A * A_i, c_A * A_i, dt * k_i, ...) far from overflow. They are rule limits, not tuning values.
const MAX_STIFFNESS_DENSITY: float = 1.0e10
const MAX_DAMPING_DENSITY: float = 1.0e9
## The bottoming compression may be at most this fraction of the wheel radius.
const MAX_COMPRESSION_RATIO: float = 0.5
## Same unit-length tolerance as TirePatchSampler / TireGeometry for the plane normal.
const UNIT_TOLERANCE: float = 1.0e-5
## Wheel axle in the wheel's own frame (project convention: body-local X).
const AXLE_LOCAL: Vector3 = Vector3.RIGHT
## Principal-axis precondition, scale-independent residual: with u the axle in the body frame and I the (symmetrised) body-frame inertia tensor,
##   rho = |I u - u (u^T I u)| / |I u|   (the sine of the angle by which the inertia turns the axle away from itself),
## which for u = (1, 0, 0) is sqrt(I_xy^2 + I_xz^2) / sqrt(I_xx^2 + I_xy^2 + I_xz^2). A wheel whose rho exceeds this tolerance is rejected.
## Why this measure: the relative error of the row effective mass caused by a non-principal axle is of the order of rho (the error term is
## J_vel_ang . I^-1 u (u . (r_i x n)) with I u = I_xx u + c, |c| = rho |I u|, and I^-1 u = (u - I^-1 c) / I_xx), so rho bounds it whatever the
## ratio of axial to transverse moments; the former test compared |c| with the largest element of the tensor, which for a small axial moment and
## large transverse moments let |c| / I_xx be as large as 1e-5 times the transverse-to-axial ratio. Measured in float64 over random sample
## geometry (axial : transverse moments from 1000:1 to 1:1000), the relative effective-mass error at rho = 1e-4 was at most 4.4e-4.
## Float32 level: a tensor assembled through general rotations carries rounding in I_xy, I_xz of about 1.5e-7 times the largest TRANSVERSE moment
## (it scales with the largest element, not with I_xx), so rho is about 2.7e-7 or less for a wheel whose axial moment is the largest or
## comparable (measured at most 2.7e-7 over 126 tensors per ratio, up to 1:10000), 1.3e-5 for a 1:100 axial-to-transverse wheel and
## 1.5e-4 at 1:1000. 1e-4 is therefore 370 times above the roundoff of ordinary wheels and covers assembled tensors down to an axial moment of
## roughly 1/600 of the transverse ones; a genuine coupling is orders of magnitude larger. It equals MotorcycleConfig.WHEEL_AXLE_PRINCIPAL_TOLERANCE
## (the same residual), so a wheel accepted by the configuration is no longer refused here. A tensor so anisotropic that its assembled roundoff
## exceeds this tolerance has an axle that float32 cannot tell from a non-principal one; give such a wheel as a diagonal tensor in the body frame.
const AXLE_PRINCIPAL_RESIDUAL_TOL: float = 1.0e-4

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

## Stage C. The tyre of this patch: an OWNED copy of a MotorcycleTireSpec (null = normal support only, the Stage B configuration) and the
## surface friction coefficient mu_s of the ground from which the ellipse limits mu_x = grip_x mu_s, mu_y = grip_y mu_s follow.
var tire: MotorcycleTireSpec = null
var surface_friction: float = 0.0
## When false the tangential and rolling-resistance responses are zero; the normal support stays available.
var tire_forces_enabled: bool = true

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

# Stage C state. Frame and reference values are frozen at the start of the solve (like every Jacobian); the impulses are the solve state.
var _axes_valid: bool = false
var _e_x: Vector3 = Vector3.ZERO
var _e_y: Vector3 = Vector3.ZERO
var _axle: Vector3 = Vector3.RIGHT
var _mu_x: float = 0.0
var _mu_y: float = 0.0
var _reference_speed: float = 1.0
var _spin: float = 0.0  # start-of-solve spin -(w.u), rad/s
## Aggregate of the current normal rows (see _update_aggregate()).
var _agg_impulse: float = 0.0
var _agg_point: Vector3 = Vector3.ZERO
var _agg_valid: bool = false
## Tangential impulse in tyre axes (N s) and the wrench the response has applied to the wheel in this step (linear impulse, angular impulse about the COM).
var _tire_lx: float = 0.0
var _tire_ly: float = 0.0
var _applied_linear: Vector3 = Vector3.ZERO
var _applied_angular: Vector3 = Vector3.ZERO
var _carried_world: Vector3 = Vector3.ZERO
var _rolling_impulse: float = 0.0  # rolling-resistance angular impulse about the axle u applied in this step (N m s)
var _tire_failed: bool = false
## Terminal numerical fault of the failure path: "" normally; otherwise the description of a tyre response that could NOT be withdrawn because the
## inverse of what it applied is not representable in float32 (see _withdraw_tire_response()). While it is set the wheel still carries the recorded
## wrench, the record (_applied_*, _rolling_impulse, _tire_lx/_tire_ly) is HELD (never cleared, so it keeps describing what the wheel carries),
## no tyre response is applied (_tire_failed stays true), and validate() reports it so SimWorld's preflight refuses further steps. Only
## reset_warm_start() (an aborted step, or the caller after repairing the wheel state) releases it.
var _withdrawal_fault: String = ""


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


## Stage C: the patch support of a wheel together with its tyre. `p_tire` is validated against the wheel radius and COPIED (later edits of the
## caller's object never reach the running patch); `p_surface_friction` is the ground coefficient mu_s (> 0). Returns null (with an error) for
## invalid input, with the same checks as create().
static func create_with_tire(ground: SimRigidBody, wheel: SimRigidBody, p_geometry: TireGeometry, p_wheel_radius: float, p_plane_point: Vector3,
		p_plane_normal: Vector3, p_stiffness_density: float, p_damping_density: float, p_max_compression: float,
		p_lateral_count: int, p_circumferential_count: int, p_tire: MotorcycleTireSpec, p_surface_friction: float):
	if ground == null or wheel == null:
		push_error("TirePatchNormalSupport.create_with_tire: bodies must not be null.")
		return null
	if p_tire == null:
		push_error("TirePatchNormalSupport.create_with_tire: the tire specification must not be null.")
		return null
	var support = (load("res://Simulation/Motorcycle/tire_patch_normal_support.gd")).new(ground, wheel, p_geometry, p_wheel_radius,
			p_plane_point, p_plane_normal, p_stiffness_density, p_damping_density, p_max_compression, p_lateral_count, p_circumferential_count)
	support.tire = p_tire.duplicate_spec()
	support.surface_friction = p_surface_friction
	var errors: PackedStringArray = support.validate()
	if not errors.is_empty():
		push_error("TirePatchNormalSupport.create_with_tire: %s" % "; ".join(errors))
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
	if tire != null:
		# Stage C: the existing tyre rules (parameter bounds, rolling radius against the wheel radius) and the surface friction.
		errors.append_array(tire.validate(wheel_radius if radius_ok else 0.0))
		if not (is_finite(surface_friction) and surface_friction > 0.0):
			errors.append("tire patch: surface friction must be finite and > 0 (got %s)" % surface_friction)
	if _withdrawal_fault != "":
		# A runtime fault, not bad data: reported here because validate() is the channel through which SimWorld's preflight refuses to step.
		errors.append(_withdrawal_fault)
	return errors


## "" when the wheel axle (AXLE_LOCAL, the body-local +X axis) is a principal axis of the body-frame inertia tensor `local_inertia`, otherwise the
## validation message. The criterion is the dimensionless residual rho = |I u - u (u^T I u)| / |I u| <= AXLE_PRINCIPAL_RESIDUAL_TOL (see the constant).
## Pure and deterministic: the verdict and the text depend only on the tensor. The tensor is evaluated in the body frame (never the world frame, see the
## header) and in float64 scalars. I u is the first column of the tensor symmetrised as SimMassProperties symmetrises the inverse it stores:
## (I_xx, 0.5 (I_xy + I_yx), 0.5 (I_xz + I_zx)); u^T I u = I_xx, so the off-axle part of I u is the pair of products of inertia. Element (row r,
## column c) of a Godot Basis is basis[c][r], so I_xy is x.y (and y.x), I_xz is x.z (and z.x). The tensor is never modified.
## The three components are divided by the largest magnitude before the norms are taken, so the residual is independent of the scale of the tensor and
## cannot overflow or underflow. A tensor that gives no usable residual (a non-finite entry, a zero I u, an axial moment I_xx <= 0) is REPORTED, never
## accepted: no division by zero is performed and a NaN residual fails the comparison. A valid dynamic body (SimMassProperties.validate_inertia) always
## has a finite, positive I_xx, so these branches are defensive.
static func axle_principal_axis_error(local_inertia: Basis) -> String:
	var axial: float = local_inertia.x.x
	var coupling_xy: float = 0.5 * (local_inertia.x.y + local_inertia.y.x)
	var coupling_xz: float = 0.5 * (local_inertia.x.z + local_inertia.z.x)
	var residual: float = NAN
	if is_finite(axial) and is_finite(coupling_xy) and is_finite(coupling_xz) and axial > 0.0:
		var largest: float = maxf(axial, maxf(absf(coupling_xy), absf(coupling_xz)))
		var a: float = axial / largest
		var x: float = coupling_xy / largest
		var z: float = coupling_xz / largest
		var off_axle: float = sqrt(x * x + z * z)
		residual = off_axle / sqrt(a * a + off_axle * off_axle)
	if is_finite(residual) and residual <= AXLE_PRINCIPAL_RESIDUAL_TOL:
		return ""
	return ("tire patch: the tyre patch normal-support model requires the wheel axle (body-local +X) to be a principal axis of the wheel inertia "
			+ "tensor, but the tensor couples the axle to the transverse axes (axial moment I_xx = %s, products of inertia I_xy = %s, I_xz = %s kg m^2; "
			+ "principal-axis residual |I u - u (u.I u)| / |I u| = %s exceeds the tolerance %s, a scale-independent ratio; a residual of nan means no "
			+ "usable residual exists for this tensor). Define the wheel inertia in a body frame whose +X axis is the "
			+ "axle principal axis (I_xy = I_xz = 0), for example a diagonal tensor with the axial moment first; the inertia is not modified here.") % [
			axial, coupling_xy, coupling_xz, residual, AXLE_PRINCIPAL_RESIDUAL_TOL]


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


# --- Stage C queries (aggregate physical state and tyre response; forces are the LAST solved step's results) ---

## Sums the CURRENT accumulated normal impulses of the active rows (so it is valid inside a solver iteration, not only after finish()) into
## _agg_impulse, and the impulse-weighted mean of the Stage A sample positions into _agg_point. No division unless the impulse is finite and > 0;
## the no-load result is zero impulse, Vector3.ZERO and _agg_valid = false (never NaN, never a stale point).
func _update_aggregate() -> void:
	var total: float = 0.0
	var sx: float = 0.0
	var sy: float = 0.0
	var sz: float = 0.0
	for k in mini(rows.size(), _active_position.size()):
		var lambda_i: float = rows[k].accumulated_impulse
		var p: Vector3 = _active_position[k]
		total += lambda_i
		sx += lambda_i * p.x
		sy += lambda_i * p.y
		sz += lambda_i * p.z
	_agg_impulse = total
	_agg_point = Vector3.ZERO
	_agg_valid = false
	if is_finite(total) and total > 0.0:
		var point: Vector3 = Vector3(sx / total, sy / total, sz / total)
		if point.is_finite():
			_agg_point = point
			_agg_valid = true


## Aggregate PHYSICAL normal impulse (N s) of the current rows: the sum of their accumulated impulses. Zero without active support.
func aggregate_normal_impulse() -> float:
	_update_aggregate()
	return _agg_impulse if contact_active and is_finite(_agg_impulse) else 0.0


## Aggregate physical normal load N = Lambda_n / dt (N). Zero without active support.
func aggregate_normal_load() -> float:
	return aggregate_normal_impulse() / _dt if _dt > 0.0 else 0.0


## True when the aggregate state is usable: positive finite aggregate normal impulse and a finite effective point.
func has_aggregate_contact() -> bool:
	_update_aggregate()
	return contact_active and _agg_valid


## Effective contact point (world): the normal-impulse weighted mean of the sample positions. Vector3.ZERO (with has_aggregate_contact() false) without load.
func effective_contact_point() -> Vector3:
	_update_aggregate()
	return _agg_point if contact_active and _agg_valid else Vector3.ZERO


## Rigid-body velocity at the effective contact point from the CURRENT wheel velocities (v + w x (p_eff - x)); Vector3.ZERO without load.
func effective_contact_velocity() -> Vector3:
	_update_aggregate()
	if not (contact_active and _agg_valid):
		return Vector3.ZERO
	return body_b.linear_velocity + body_b.angular_velocity.cross(_agg_point - body_b.position)


## Longitudinal axis (forward, projected on the ground) and lateral axis (right) of the last prepared step; zero for degenerate axes, without a
## tyre and without support.
func longitudinal_axis() -> Vector3:
	return _e_x if _axes_valid and contact_active else Vector3.ZERO


func lateral_axis() -> Vector3:
	return _e_y if _axes_valid and contact_active else Vector3.ZERO


## Slip of the wheel from its CURRENT state evaluated at the effective contact point (TireModel.slip with this tyre; {valid = false, reason}
## without a tyre, without aggregate load or for degenerate geometry).
func current_slip() -> Dictionary:
	if tire == null:
		return {"valid": false, "reason": "no tire specification"}
	_update_aggregate()
	if not (contact_active and _agg_valid):
		return {"valid": false, "reason": "no positive aggregate normal impulse"}
	return TireModel.slip(body_b.position, body_b.orientation, body_b.linear_velocity, body_b.angular_velocity, wheel_radius, tire.rolling_radius,
			plane_point, plane_normal, tire.low_speed_reference, _agg_point)


## Tangential tyre force on the wheel (N) along e_x / e_y in the last solved step (after the ellipse scaling; rolling resistance excluded).
func longitudinal_force() -> float:
	return _tire_lx / _dt if _dt > 0.0 else 0.0


func lateral_force() -> float:
	return _tire_ly / _dt if _dt > 0.0 else 0.0


## Total tangential tyre force on the wheel as a world vector (N).
func tangential_force_world() -> Vector3:
	return _applied_linear / _dt if _dt > 0.0 else Vector3.ZERO


## Moment about the wheel centre (N m, world) that the tangential response applied: (p_eff - x) x F, the resultant force at the effective point.
func tangential_moment_world() -> Vector3:
	return _applied_angular / _dt if _dt > 0.0 else Vector3.ZERO


## Rolling-resistance moment on the wheel about its axle u (N m): positive along +u, i.e. against the spin of a forward-rolling wheel.
func rolling_resistance_torque() -> float:
	return _rolling_impulse / _dt if _dt > 0.0 else 0.0


## Diagnostic: the longitudinal force at the tread equal to the rolling-resistance moment, -M / R_eff (the effect is applied as the moment).
func rolling_resistance_force() -> float:
	return -rolling_resistance_torque() / tire.rolling_radius if tire != null else 0.0


## Friction limits mu_x N and mu_y N of the last step (N), from the aggregate load; zero without a tyre or load.
func longitudinal_force_limit() -> float:
	return _mu_x * _load if tire != null and _axes_valid else 0.0


func lateral_force_limit() -> float:
	return _mu_y * _load if tire != null and _axes_valid else 0.0


## Friction-ellipse utilisation sqrt((Fx/(mu_x N))^2 + (Fy/(mu_y N))^2) of the aggregate response in the last step, in [0, 1]; 0 without load.
func friction_utilization() -> float:
	var ln: float = _load * _dt
	if not (ln > 0.0 and _mu_x > 0.0 and _mu_y > 0.0 and tire != null and _axes_valid):
		return 0.0
	var ux: float = _tire_lx / (_mu_x * ln)
	var uy: float = _tire_ly / (_mu_y * ln)
	return sqrt(ux * ux + uy * uy)


func friction_coefficient_longitudinal() -> float:
	return tire.grip_longitudinal * surface_friction if tire != null else 0.0


func friction_coefficient_lateral() -> float:
	return tire.grip_lateral * surface_friction if tire != null else 0.0


## Carried (warm-start) tangential tyre impulse (N s, world vector) that the next step would apply; zero after separation, reset and without a tyre.
func carried_tire_impulse() -> Vector3:
	return _carried_world


## Number of tangential responses applied through this constraint: one (one resultant per wheel) when the tyre acts, otherwise zero.
func tangential_response_count() -> int:
	return 1 if tire != null and tire_forces_enabled and _axes_valid and contact_active else 0


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


## Stage C: the tyre's start-of-solve frame (axes, reference speed, spin) is read BEFORE the normal warm start, so the frozen values are those of
## the start of the solve exactly as in the one-point contact; the tangential warm start follows the warm-started normal rows.
func prepare(dt: float, config) -> void:
	_capture_tire_frame()
	super.prepare(dt, config)
	_prepare_tire(config)


## Normal rows first (Stage B, unchanged), then the ONE tangential and the ONE rolling-resistance response of the patch, from the aggregate
## of those rows as they are NOW (same iteration).
func solve_iteration() -> void:
	if contact_active:
		super.solve_iteration()
		_solve_tire_iteration()


func solve_split_iteration() -> void:
	if contact_active:
		super.solve_split_iteration()


## Physical load of every active sample = its accumulated impulse / dt (never a stored force).
func finish() -> void:
	_sample_load = []
	_load = 0.0
	if not contact_active or not (_dt > 0.0):
		return
	if _tire_failed:
		# A failed response has either withdrawn everything it applied (and applied nothing since: it is terminal for the solve), which makes the
		# call below a no-op, or it could not (_withdrawal_fault): then the record is held, not cleared, because the wheel still carries it.
		_withdraw_or_hold()
		last_reason = _withdrawal_fault if _withdrawal_fault != "" else "non-finite aggregate tire state or derived tyre solver value"
		_clear_state()
		return
	var total: float = 0.0
	for row in rows:
		var load: float = row.accumulated_impulse / _dt
		_sample_load.append(load)
		total += load
	if not is_finite(total):
		_fail_tire_response()
		last_reason = _withdrawal_fault if _withdrawal_fault != "" else "non-finite aggregate tire state or derived tyre solver value"
		_clear_state()
		return
	_load = total
	var tire_on: bool = tire != null and tire_forces_enabled and _axes_valid
	_carried_world = (_e_x * _tire_lx + _e_y * _tire_ly) if tire_on else Vector3.ZERO


## SimWorld calls it after an aborted step: every carried impulse and every derived result is discarded.
func reset_warm_start() -> void:
	super()
	_withdrawal_fault = ""  # the only release of a withdrawal fault (the wheel state was restored by the abort or repaired by the caller)
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
	_clear_tire_state()


# --- Stage C: tyre response ---

## Discards the whole tyre state: tangential impulse, applied wrench, carried impulse, rolling impulse, aggregate and the frame of the last step.
## EXCEPT a held withdrawal fault (see `_withdrawal_fault`): the wheel still carries that response, so its record, the tangential impulse that
## produced it and the failure flag survive every clear except reset_warm_start(); the aggregate, the frame and the carried (warm start) impulse
## are still discarded, so nothing of it is carried into a later solve.
func _clear_tire_state() -> void:
	_axes_valid = false
	_agg_impulse = 0.0
	_agg_point = Vector3.ZERO
	_agg_valid = false
	_carried_world = Vector3.ZERO
	if _withdrawal_fault != "":
		return
	_tire_lx = 0.0
	_tire_ly = 0.0
	_applied_linear = Vector3.ZERO
	_applied_angular = Vector3.ZERO
	_rolling_impulse = 0.0
	_tire_failed = false


## Tyre axes, reference speed, spin and friction limits from the start-of-solve state (frozen for the step; before any impulse of this step).
func _capture_tire_frame() -> void:
	_axes_valid = false
	if tire == null or not tire_forces_enabled:
		return
	var u: Vector3 = (body_b.orientation * AXLE_LOCAL).normalized()
	var tyre_axes: Dictionary = TireModel.axes(u, plane_normal)
	if not (tyre_axes["valid"] and body_b.linear_velocity.is_finite() and body_b.angular_velocity.is_finite()):
		return
	_axle = u
	_e_x = tyre_axes["longitudinal"]
	_e_y = tyre_axes["lateral"]
	_mu_x = tire.grip_longitudinal * surface_friction
	_mu_y = tire.grip_lateral * surface_friction
	var hub: float = body_b.linear_velocity.dot(_e_x)
	_reference_speed = sqrt(hub * hub + tire.low_speed_reference * tire.low_speed_reference)
	_spin = -body_b.angular_velocity.dot(u)
	_axes_valid = true


## Resets the solve state of the tyre response and applies the warm start: the carried tangential impulse, projected into the current tangent
## plane and scaled into the current ellipse, at the effective point of the warm-started normal rows. Cold when there is no load.
func _prepare_tire(config) -> void:
	if _withdrawal_fault != "":
		# A held withdrawal fault is terminal until reset_warm_start(): no response is applied and the record of what the wheel still carries stays.
		_tire_failed = true
		_carried_world = Vector3.ZERO
		_agg_impulse = 0.0
		_agg_point = Vector3.ZERO
		_agg_valid = false
		last_reason = _withdrawal_fault
		return
	_tire_lx = 0.0
	_tire_ly = 0.0
	_applied_linear = Vector3.ZERO
	_applied_angular = Vector3.ZERO
	_rolling_impulse = 0.0
	_tire_failed = false
	_agg_impulse = 0.0
	_agg_point = Vector3.ZERO
	_agg_valid = false
	if not contact_active or tire == null or not tire_forces_enabled or not _axes_valid:
		_carried_world = Vector3.ZERO
		_axes_valid = _axes_valid and contact_active
		return
	var carried: Vector3 = _carried_world * config.warm_start_factor
	_carried_world = Vector3.ZERO
	if not config.warm_starting or not carried.is_finite():
		return
	_update_aggregate()
	if not _agg_valid:
		return
	carried -= plane_normal * carried.dot(plane_normal)
	var clipped: Vector2 = TireModel.into_ellipse(carried.dot(_e_x), carried.dot(_e_y), _agg_impulse, _mu_x, _mu_y)
	_apply_tangential_wrench(clipped.x, clipped.y, _agg_point)


## Makes the wrench that the tangential response has applied to the wheel equal to the resultant force (lambda_x e_x + lambda_y e_y) acting at
## `point`: the linear impulse L and the angular impulse (point - x) x L. Only the DIFFERENCE to what was applied before is added to the wheel,
## so the impulse follows both the amount and the point of application as the normal rows converge. The ground is static and takes nothing.
func _apply_tangential_wrench(lambda_x: float, lambda_y: float, point: Vector3) -> void:
	if _tire_failed:
		return  # a failed response is terminal for the solve (see _fail_tire_response())
	# Validate-then-apply: every candidate is derived first and the wheel and the bookkeeping are touched only when ALL of them are finite. Finite
	# scalar impulses can still overflow here (a Vector3 is float32), so the derived impulses, their changes, the velocity changes and the resulting
	# velocities are checked, not just the inputs. An invalid candidate is a tyre failure (never clamped to zero, never half applied); a valid zero
	# response is finite and passes. The record is therefore always finite (_withdraw_tire_response() still validates it and the rollback it derives).
	var valid: bool = is_finite(lambda_x) and is_finite(lambda_y) and point.is_finite()
	var linear: Vector3 = Vector3.ZERO
	var angular: Vector3 = Vector3.ZERO
	var velocity: Vector3 = Vector3.ZERO
	var spin: Vector3 = Vector3.ZERO
	if valid:
		linear = _e_x * lambda_x + _e_y * lambda_y
		angular = (point - body_b.position).cross(linear)
		var d_linear: Vector3 = linear - _applied_linear
		var d_angular: Vector3 = angular - _applied_angular
		var d_velocity: Vector3 = d_linear * body_b.inverse_mass
		var d_spin: Vector3 = body_b.inverse_inertia_world * d_angular
		velocity = body_b.linear_velocity + d_velocity
		spin = body_b.angular_velocity + d_spin
		valid = (linear.is_finite() and angular.is_finite() and d_linear.is_finite() and d_angular.is_finite() and d_velocity.is_finite()
				and d_spin.is_finite() and velocity.is_finite() and spin.is_finite())
	if not valid:
		_fail_tire_response()
		return
	body_b.linear_velocity = velocity
	body_b.angular_velocity = spin
	_applied_linear = linear
	_applied_angular = angular
	_tire_lx = lambda_x
	_tire_ly = lambda_y


## One tangential and rolling-resistance update from the aggregate of the normal rows as they are right now (see the Stage C header).
func _solve_tire_iteration() -> void:
	if tire == null or not tire_forces_enabled or not _axes_valid:
		return
	if _tire_failed:
		return  # terminal for the rest of this solve: no later iteration applies any tangential or rolling-resistance response
	_update_aggregate()
	if not _agg_valid:
		# A non-finite aggregate is a failure, never a skipped sample.
		if not is_finite(_agg_impulse):
			_fail_tire_response()
			return
		# Zero aggregate load: withdraw whatever this step had applied (a valid zero response; the next iteration may find load again).
		_apply_tangential_wrench(0.0, 0.0, body_b.position)
		_solve_rolling_resistance(0.0)
		return
	var ln: float = _agg_impulse
	# Re-seat the carried tangential impulse at the CURRENT effective point before the slip is measured.
	_apply_tangential_wrench(_tire_lx, _tire_ly, _agg_point)
	if _tire_failed:
		return  # the re-seat itself can fail (see _apply_tangential_wrench())
	var r: Vector3 = _agg_point - body_b.position
	var v_c: Vector3 = body_b.linear_velocity + body_b.angular_velocity.cross(r)
	var rr_offset: float = tire.rolling_radius - wheel_radius
	# Slip velocities of the tread at p_eff (TireModel.slip): s_x = -(v_c.e_x) + (R_eff - R) spin with spin = -(w.u); s_y = -(v_c.e_y).
	var s_x: float = -v_c.dot(_e_x) - rr_offset * _axle.dot(body_b.angular_velocity)
	var s_y: float = -v_c.dot(_e_y)
	# Exact 2x2 response G = S M^-1 P^T of the slip velocities to unit tangential impulses at p_eff (impulse rows P: linear e, angular r x e).
	var p_x_ang: Vector3 = r.cross(_e_x)
	var p_y_ang: Vector3 = r.cross(_e_y)
	var sx_ang: Vector3 = -p_x_ang - _axle * rr_offset
	var sy_ang: Vector3 = -p_y_ang
	var im: float = body_b.inverse_mass
	var ip_x: Vector3 = body_b.inverse_inertia_world * p_x_ang
	var ip_y: Vector3 = body_b.inverse_inertia_world * p_y_ang
	var g00: float = im * (-_e_x).dot(_e_x) + sx_ang.dot(ip_x)
	var g01: float = im * (-_e_x).dot(_e_y) + sx_ang.dot(ip_y)
	var g10: float = im * (-_e_y).dot(_e_x) + sy_ang.dot(ip_x)
	var g11: float = im * (-_e_y).dot(_e_y) + sy_ang.dot(ip_y)
	var target: Vector2 = Vector2.ZERO
	if _mu_x > 0.0 and _mu_y > 0.0:
		target = TireModel.implicit_tangential_impulse(s_x, s_y, ln, _tire_lx, _tire_ly, g00, g01, g10, g11, tire.longitudinal_stiffness,
				tire.lateral_stiffness, _mu_x, _mu_y, _reference_speed)
	if not (is_finite(s_x) and is_finite(s_y) and is_finite(g00) and is_finite(g01) and is_finite(g10) and is_finite(g11) and target.is_finite()):
		_fail_tire_response()
		return
	_apply_tangential_wrench(target.x, target.y, _agg_point)
	_solve_rolling_resistance(ln)


## Rolling-resistance angular impulse about the axle, a function of the current AGGREGATE normal impulse and the frozen spin; the difference to
## what was already applied this step is applied (so it follows the load as the rows converge, and is never warm started). One per wheel.
## Every derived value is validated BEFORE the wheel is touched: the target, the change to the applied impulse, the angular-velocity change (a
## Vector3 is float32, so a finite float64 target can still overflow here) and the resulting angular velocity. An invalid value is a tyre failure
## (_fail_tire_response()), never clamped to zero: a valid zero response (no load, zero spin) is a different thing and stays valid.
func _solve_rolling_resistance(ln: float) -> void:
	if _tire_failed:
		return
	var target: float = 0.0
	if ln > 0.0:
		# TireModel.rolling_resistance_force(v, N, c, v_rr) = -c N psi(v / v_rr); the moment about +u opposes a positive spin.
		target = -TireModel.rolling_resistance_force(tire.rolling_radius * _spin, ln, tire.rolling_resistance, tire.rolling_resistance_speed) * tire.rolling_radius
	var delta: float = target - _rolling_impulse
	var d_omega: Vector3 = body_b.inverse_inertia_world * (_axle * delta)
	var omega: Vector3 = body_b.angular_velocity + d_omega
	if not (is_finite(target) and is_finite(delta) and d_omega.is_finite() and omega.is_finite()):
		_fail_tire_response()
		return
	_rolling_impulse = target
	body_b.angular_velocity = omega


## The ONE tyre-failure path. The tyre response of this solve is marked failed (TERMINAL until the next prepare() or a clear: every later iteration
## returns before applying anything) and whatever it has applied to the wheel is withdrawn, so the wheel is left as the normal rows alone made it.
## If that withdrawal is not representable the fault is held instead (_withdraw_or_hold()): the failure is then a terminal NUMERICAL FAULT.
func _fail_tire_response() -> void:
	_tire_failed = true
	_withdraw_or_hold()


## Withdraws the recorded response, or, when its inverse cannot be represented, holds the record and raises the terminal numerical fault.
## Never clamps and never half applies. A fault already held is left as it is (the record is not touched a second time).
func _withdraw_or_hold() -> void:
	if _withdrawal_fault != "":
		return
	if not _withdraw_tire_response():
		_withdrawal_fault = "tire patch: the tyre response could not be withdrawn because the inverse of the recorded wrench is not representable (non-finite recorded impulse or wheel velocity, or a rollback beyond the float32 range; linear impulse %s, angular impulse %s, rolling impulse %s). The wheel keeps the recorded response and the record is held; reset_warm_start() releases it after the wheel state is repaired" % [_applied_linear, _applied_angular, _rolling_impulse]


## Removes from the wheel the tangential wrench and the rolling-resistance moment that this solve has applied, using the bookkeeping that recorded
## them (the exact inverse of what _apply_tangential_wrench() and _solve_rolling_resistance() added), then zeroes that bookkeeping. Returns true when
## the wheel no longer carries the response (including when nothing was recorded).
## VALIDATE-THEN-WRITE, like both response paths: the record, the wheel velocities, every velocity change and both resulting velocities are checked
## BEFORE either velocity is written, so the rollback is atomic (linear, tangential angular and rolling angular parts together, never one without the
## others) and a Vector3 (float32) overflow cannot reach the wheel. A finite wheel state near the float32 limit can still have a non-representable
## inverse, and a clamp would not be a rollback (the record would no longer equal what the wheel carries), so in that case NOTHING is written or cleared
## and false is returned; the caller turns that into the held numerical fault. A non-finite record is not "never applied" (that was only true while
## nothing could tamper with it): it cannot be inverted, hence also a fault.
func _withdraw_tire_response() -> bool:
	if _applied_linear == Vector3.ZERO and _applied_angular == Vector3.ZERO and _rolling_impulse == 0.0:
		_tire_lx = 0.0
		_tire_ly = 0.0
		return true  # nothing recorded, nothing to invert
	var roll_impulse: Vector3 = _axle * _rolling_impulse
	var d_linear: Vector3 = _applied_linear * body_b.inverse_mass
	var d_spin: Vector3 = body_b.inverse_inertia_world * _applied_angular
	var d_roll: Vector3 = body_b.inverse_inertia_world * roll_impulse
	var velocity: Vector3 = body_b.linear_velocity - d_linear
	# REVERSE ORDER of the application (tangential first, rolling second): the rolling moment comes off first, then the tangential one. float32
	# sums are not associative: when the tangential contribution is large, the forward sum (w + d_tan) + d_roll may have lost a small d_roll, and
	# removing d_tan first would then leave a residual of d_roll in the spin; undoing the steps in reverse order cancels exactly that rounding.
	var spin_rolling: Vector3 = body_b.angular_velocity - d_roll
	var spin: Vector3 = spin_rolling - d_spin
	if not (_applied_linear.is_finite() and _applied_angular.is_finite() and is_finite(_rolling_impulse) and _axle.is_finite()
			and body_b.linear_velocity.is_finite() and body_b.angular_velocity.is_finite() and roll_impulse.is_finite()
			and d_linear.is_finite() and d_spin.is_finite() and d_roll.is_finite() and velocity.is_finite() and spin_rolling.is_finite() and spin.is_finite()):
		return false
	body_b.linear_velocity = velocity
	body_b.angular_velocity = spin
	_applied_linear = Vector3.ZERO
	_applied_angular = Vector3.ZERO
	_tire_lx = 0.0
	_tire_ly = 0.0
	_rolling_impulse = 0.0
	return true
