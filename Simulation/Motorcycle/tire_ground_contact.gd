extends "res://Simulation/Motorcycle/wheel_ground_contact.gd"
## Wheel-ground contact WITH a tyre (M2.4.2): the unilateral normal response of the generic ContactConstraint plus a tyre model that is
## the ONLY owner of the tangential (traction) response. Motorcycle-level; the generic solver is unchanged. Mathematics and sign
## conventions: tire_model.gd and PHYSICS_CONVENTIONS.md ("Tyre behaviour"); ordering: ARCHITECTURE.md ("Tyre model (M2.4.2)").
##
## ---- Friction ownership (no duplicate friction) ----
## The generic isotropic Coulomb block of ContactConstraint is bypassed completely: _prepare_active() never warm starts it,
## solve_iteration() never calls it and _solve_friction() is an empty override, so no generic friction impulse can ever exist for this
## contact. The two tangent rows are re-aimed at the tyre axes (row 1 = longitudinal e_x, row 2 = lateral e_y) and carry ONLY the tyre
## impulse. `friction` (inherited) is the SURFACE coefficient mu_s from which the tyre limits mu_x = grip_x mu_s, mu_y = grip_y mu_s
## follow; it is never applied as a Coulomb disk. With tire_forces_enabled = false the tangential impulse is zero (frictionless
## contact): the normal response stays active, the tyre contributes nothing and nothing else replaces it.
##
## ---- One solver iteration (after the physical normal row of the SAME iteration) ----
##  1. lambda_n = accumulated normal impulse now; the normal load of this step is N = lambda_n / dt (never a stale value; zero when
##     the contact is inactive because an inactive contact discards its impulses).
##  2. s = current slip velocities (S . body velocity, including every impulse applied so far), sigma = s / D, rho, phi(rho).
##  3. Implicit (backward-Euler) tangential impulse: lambda = A sigma(lambda) + lambda_rr with A = diag(phi k_x lambda_n, phi k_y lambda_n) and
##     sigma(lambda) = (s + G (lambda - lambda_cur)) / D linearised through the exact 2x2 response G = S M^-1 P^T of the wheel
##     (S = slip rows, P = impulse rows); solved in closed form. phi is lagged one iteration (a monotone fixed point with
##     contraction factor < 1/3 per iteration, see the derivation in PHYSICS_CONVENTIONS.md).
##  4. The slip impulse is radially scaled into the friction ellipse (lambda_x/(mu_x lambda_n))^2 + (lambda_y/(mu_y lambda_n))^2 <= 1
##     and applied at the contact point (normal-row point) through rows 1 and 2: the same impulse gives the wheel its linear AND
##     angular response; the ground is static and takes nothing.
##  5. Rolling resistance is a SEPARATE effect: a moment about the axle, M = c_rr N R_eff psi(R_eff spin / v_rr), opposing the spin
##     (the normal force acts ahead of the wheel centre of a rolling tyre). It is applied as an angular impulse M dt on the wheel from
##     the same load N, is outside the slip ellipse (it is not a traction force) and vanishes with the spin and with the load. As a
##     contact FORCE it would be cancelled by the slip force of a free wheel and would not decelerate it (rejected approach).
## Why implicit: with an explicit tyre force and the wheel's low inertia about its axle the effective stiffness c R^2 dt / I exceeds 2
## for the prototype loads (unstable); the implicit solve is unconditionally stable for the slip stiffness.
##
## ---- Speculative contact is not tyre contact (M2.4.2.1) ----
## The generic normal row stays active while the wheel has a small positive gap and approaches (speculative bias g/dt, unchanged): its
## positive impulse is COLLISION PREVENTION, not physical load. The tyre therefore acts only while the wheel is GEOMETRICALLY in
## contact: gap g (start-of-step geometry) <= TOUCH_TOLERANCE (0.1 mm, inherited from WheelGroundContact; penetration included). While
## the contact is active but not touching: no tyre warm start, no tangential impulse, no rolling resistance, the carried tyre impulse
## and rolling-resistance impulse are cleared (no state leaks across the air gap) and normal_load() / the force limits / the
## utilisation report zero physical load. The normal row, its split-impulse correction and the generic contact are not touched.
##
## ---- Lifecycle ----
## The geometry, the tyre axes, the slip rows and the reference speed are re-derived from the start-of-step state at every fixed step
## (prepare()). A contact that is inactive (wheel separated or degenerate) discards ALL carried impulses (reset_warm_start()), so no
## normal load, slip or tyre force survives separation, and the first step after re-contact starts cold.

const TireModel = preload("res://Simulation/Motorcycle/tire_model.gd")
const MotorcycleTireSpec = preload("res://Simulation/Motorcycle/motorcycle_tire_spec.gd")

var tire: MotorcycleTireSpec
## When false the tangential response is zero (the normal response stays).
var tire_forces_enabled: bool = true

var _e_x: Vector3 = Vector3.ZERO
var _e_y: Vector3 = Vector3.ZERO
var _axes_valid: bool = false
var _dt: float = 0.0
var _mu_x: float = 0.0
var _mu_y: float = 0.0
var _reference_speed: float = 1.0
var _hub_speed: float = 0.0
var _spin: float = 0.0  # start-of-solve spin -(w.u), rad/s
var _axle: Vector3 = Vector3.RIGHT
var _sx_lin: Vector3 = Vector3.ZERO
var _sx_ang: Vector3 = Vector3.ZERO
var _sy_lin: Vector3 = Vector3.ZERO
var _sy_ang: Vector3 = Vector3.ZERO
var _g00: float = 0.0
var _g01: float = 0.0
var _g10: float = 0.0
var _g11: float = 0.0
var _tire_impulse_world: Vector3 = Vector3.ZERO
## True when the last prepared step had the wheel geometrically in contact (gap <= TOUCH_TOLERANCE); only then does the tyre act and
## does the normal impulse count as physical load. False while separated (speculative normal row only) and while inactive.
var _touching: bool = false
var _rolling_angular_impulse: float = 0.0  # rolling-resistance angular impulse about the axle u applied in the last step (N m s)


func _init(p_ground: SimRigidBody, p_wheel: SimRigidBody, p_local_anchor_a: Vector3, p_local_anchor_b: Vector3, p_local_normal_a: Vector3,
		p_surface_friction: float, p_wheel_radius: float, p_plane_point: Vector3, p_plane_normal: Vector3, p_activation_distance: float,
		p_tire: MotorcycleTireSpec) -> void:
	tire = p_tire  # set BEFORE the generic constructor validates the contact (validate() checks it)
	super(p_ground, p_wheel, p_local_anchor_a, p_local_anchor_b, p_local_normal_a, p_surface_friction, p_wheel_radius, p_plane_point, p_plane_normal, p_activation_distance)


## Builds a tyre contact from the wheel's CURRENT pose; null (with an error) for invalid input or degenerate geometry.
## `surface_friction` is the ground's coefficient mu_s; `p_tire` is validated against the wheel radius.
static func create_with_tire(ground: SimRigidBody, wheel: SimRigidBody, p_wheel_radius: float, p_plane_point: Vector3, p_plane_normal: Vector3,
		surface_friction: float, p_tire: MotorcycleTireSpec, p_activation_distance: float = 0.05):
	if ground == null or wheel == null:
		push_error("TireGroundContact.create_with_tire: bodies must not be null.")
		return null
	if p_tire == null:
		push_error("TireGroundContact.create_with_tire: the tire specification must not be null.")
		return null
	var errors: PackedStringArray = input_errors(p_wheel_radius, p_plane_point, p_plane_normal, p_activation_distance)
	errors.append_array(p_tire.validate(p_wheel_radius))
	if not errors.is_empty():
		push_error("TireGroundContact.create_with_tire: %s" % "; ".join(errors))
		return null
	var n: Vector3 = p_plane_normal.normalized()
	var geometry: Dictionary = compute_geometry(wheel.position, wheel.orientation, p_wheel_radius, p_plane_point, n)
	if not geometry.valid:
		push_error("TireGroundContact.create_with_tire: %s" % geometry.reason)
		return null
	var inv_a: Quaternion = ground.orientation.inverse()
	var inv_b: Quaternion = wheel.orientation.inverse()
	var contact = (load("res://Simulation/Motorcycle/tire_ground_contact.gd")).new(ground, wheel,
		inv_a * (geometry.ground_point - ground.position), inv_b * (geometry.wheel_point - wheel.position), inv_a * n, surface_friction,
		p_wheel_radius, p_plane_point, n, p_activation_distance, p_tire)
	contact.geometry_valid = true
	return contact if contact.is_valid() else null


func validate() -> PackedStringArray:
	var errors: PackedStringArray = super.validate()
	if tire == null:
		errors.append("the tire specification must not be null")
	else:
		errors.append_array(tire.validate(wheel_radius))
	return errors


# --- Queries (read-only; forces and load are the LAST solved step's results, slip is evaluated from the CURRENT body state) ---

## PHYSICAL normal load of the last solved step (N) = normal impulse / dt while the wheel is geometrically touching the ground
## (gap <= TOUCH_TOLERANCE). Zero while the contact is inactive, before the first step, and while the wheel is separated: the
## positive normal impulse of an active speculative contact is collision prevention (see `speculative_normal_impulse()`), not load.
func normal_load() -> float:
	return rows[ROW_NORMAL].accumulated_impulse / _dt if _dt > 0.0 and _touching else 0.0


## True when the last prepared step had the wheel geometrically in contact, i.e. the tyre was allowed to act.
func is_touching() -> bool:
	return _touching


## Normal impulse of the generic row in the last solved step (N s) while the wheel is separated and the speculative contact is
## active: the collision-prevention response. Zero while touching (that impulse is then the physical load, `normal_load()`).
func speculative_normal_impulse() -> float:
	return rows[ROW_NORMAL].accumulated_impulse if contact_active and not _touching else 0.0


## Longitudinal axis (forward, projected on the ground) and lateral axis (right) of the last prepared step.
func longitudinal_axis() -> Vector3:
	return _e_x


func lateral_axis() -> Vector3:
	return _e_y


## Tangential slip force on the wheel (N) along e_x / e_y in the last solved step (after the ellipse scaling; rolling resistance excluded).
func longitudinal_force() -> float:
	return rows[ROW_TANGENT_1].accumulated_impulse / _dt if _dt > 0.0 else 0.0


func lateral_force() -> float:
	return rows[ROW_TANGENT_2].accumulated_impulse / _dt if _dt > 0.0 else 0.0


## Rolling-resistance moment on the wheel about its axle u (N m) in the last solved step: positive along +u, i.e. against the
## spin of a forward-rolling wheel (which turns about -u). Zero for a stationary wheel, without load or without contact.
func rolling_resistance_torque() -> float:
	return _rolling_angular_impulse / _dt if _dt > 0.0 else 0.0


## The longitudinal force at the tread that equals the rolling-resistance moment (N, along e_x, signed): -M / R_eff, so it is negative
## (backward) while rolling forward. Diagnostic only: the effect is applied as the moment.
func rolling_resistance_force() -> float:
	return -rolling_resistance_torque() / tire.rolling_radius


## Total tangential force on the wheel as a world vector (N).
func tangential_force_world() -> Vector3:
	return _e_x * longitudinal_force() + _e_y * lateral_force()


## Force limits mu_x N and mu_y N of the last step (N).
func longitudinal_force_limit() -> float:
	return _mu_x * normal_load()


func lateral_force_limit() -> float:
	return _mu_y * normal_load()


## Friction-ellipse utilisation sqrt((Fx/(mu_x N))^2 + (Fy/(mu_y N))^2) of the last step, in [0, 1]; 0 without load.
func friction_utilization() -> float:
	var ln: float = rows[ROW_NORMAL].accumulated_impulse if _touching else 0.0
	if not (ln > 0.0 and _mu_x > 0.0 and _mu_y > 0.0):
		return 0.0
	var ux: float = rows[ROW_TANGENT_1].accumulated_impulse / (_mu_x * ln)
	var uy: float = rows[ROW_TANGENT_2].accumulated_impulse / (_mu_y * ln)
	return sqrt(ux * ux + uy * uy)


## Effective limit coefficients mu_x = grip_x mu_s and mu_y = grip_y mu_s (from the current tyre and surface values).
func friction_coefficient_longitudinal() -> float:
	return tire.grip_longitudinal * friction


func friction_coefficient_lateral() -> float:
	return tire.grip_lateral * friction


## Slip of the wheel from its CURRENT state (TireModel.slip with the contact's tyre parameters; valid = false for degenerate geometry).
func current_slip() -> Dictionary:
	return TireModel.slip(body_b.position, body_b.orientation, body_b.linear_velocity, body_b.angular_velocity, wheel_radius,
		tire.rolling_radius, plane_point, plane_normal, tire.low_speed_reference)


# --- Solver interface ---

func _update_rows(dt: float, config) -> void:
	super._update_rows(dt, config)
	_dt = dt
	# Geometric contact of THIS step (start-of-step gap, the one the normal row uses): the tyre acts only then.
	_touching = _gap <= TOUCH_TOLERANCE
	var u: Vector3 = (body_b.orientation * WHEEL_AXLE_LOCAL).normalized()
	var tyre_axes: Dictionary = TireModel.axes(u, _normal)
	_axes_valid = tyre_axes.valid
	if not _axes_valid:
		return
	_e_x = tyre_axes.longitudinal
	_e_y = tyre_axes.lateral
	_t1 = _e_x
	_t2 = _e_y
	var r_a: Vector3 = _point - body_a.position
	var r_b: Vector3 = _point - body_b.position
	_set_row(rows[ROW_TANGENT_1], _e_x, r_a, r_b)
	_set_row(rows[ROW_TANGENT_2], _e_y, r_a, r_b)
	for i in [ROW_TANGENT_1, ROW_TANGENT_2]:
		rows[i].bias = 0.0
		rows[i].min_impulse = -INF
		rows[i].max_impulse = INF
	_mu_x = tire.grip_longitudinal * friction
	_mu_y = tire.grip_lateral * friction
	# Slip rows S (slip velocity = S . (v, w) of the wheel; the ground is static): s_x = -(v_c.e_x) + (R_eff - R) spin, s_y = -(v_c.e_y),
	# with v_c.e = row_e . velocity (the tangent rows ARE the contact-point velocity Jacobians) and spin = -(w.u).
	var p_x = rows[ROW_TANGENT_1]
	var p_y = rows[ROW_TANGENT_2]
	_sx_lin = -p_x.linear_b
	_sx_ang = -p_x.angular_b - u * (tire.rolling_radius - wheel_radius)
	_sy_lin = -p_y.linear_b
	_sy_ang = -p_y.angular_b
	var im: float = body_b.inverse_mass
	var inv_i: Basis = body_b.inverse_inertia_world
	var ip_x: Vector3 = inv_i * p_x.angular_b
	var ip_y: Vector3 = inv_i * p_y.angular_b
	_g00 = im * _sx_lin.dot(p_x.linear_b) + _sx_ang.dot(ip_x)
	_g01 = im * _sx_lin.dot(p_y.linear_b) + _sx_ang.dot(ip_y)
	_g10 = im * _sy_lin.dot(p_x.linear_b) + _sy_ang.dot(ip_x)
	_g11 = im * _sy_lin.dot(p_y.linear_b) + _sy_ang.dot(ip_y)
	# Reference speed and rolling resistance factor from the start-of-solve hub velocity (frozen for the step, like every Jacobian).
	_hub_speed = body_b.linear_velocity.dot(_e_x)
	_reference_speed = sqrt(_hub_speed * _hub_speed + tire.low_speed_reference * tire.low_speed_reference)
	_axle = u
	_spin = -body_b.angular_velocity.dot(u)


func _prepare_active(dt: float, config) -> void:
	_update_rows(dt, config)
	var normal_row = rows[ROW_NORMAL]
	normal_row.prepare(config.min_row_denominator)
	normal_row.warm_start(config.warm_starting, config.warm_start_factor)
	rows[ROW_TANGENT_1].accumulated_impulse = 0.0
	rows[ROW_TANGENT_2].accumulated_impulse = 0.0
	_rolling_angular_impulse = 0.0
	if not _touching:
		# Speculative contact across an air gap: no tyre state may survive (cold tyre; the normal row above keeps its generic warm start).
		_tire_impulse_world = Vector3.ZERO
		return
	if not config.warm_starting or not _axes_valid or not tire_forces_enabled:
		return
	# Carried tyre impulse: scale, project into the current tangent plane, express in the current tyre axes, scale into the current ellipse.
	var carried: Vector3 = _tire_impulse_world * config.warm_start_factor
	carried -= _normal * carried.dot(_normal)
	var clipped: Vector2 = _into_ellipse(carried.dot(_e_x), carried.dot(_e_y), normal_row.accumulated_impulse)
	rows[ROW_TANGENT_1].accumulated_impulse = clipped.x
	rows[ROW_TANGENT_2].accumulated_impulse = clipped.y
	rows[ROW_TANGENT_1].apply_impulse(clipped.x)
	rows[ROW_TANGENT_2].apply_impulse(clipped.y)


## Normal row first (the load for this iteration), then the tyre's tangential response. Never the generic friction block.
func solve_iteration() -> void:
	if not contact_active:
		return
	rows[ROW_NORMAL].solve()
	_solve_tire()


## The generic Coulomb friction block belongs to ContactConstraint; a tyre contact has none (see the file header).
func _solve_friction() -> void:
	pass


func finish() -> void:
	if contact_active:
		_tire_impulse_world = _e_x * rows[ROW_TANGENT_1].accumulated_impulse + _e_y * rows[ROW_TANGENT_2].accumulated_impulse


func reset_warm_start() -> void:
	super()
	_touching = false
	_tire_impulse_world = Vector3.ZERO
	_rolling_angular_impulse = 0.0


## Scales (lx, ly) radially into the friction ellipse of the normal impulse `ln` (zero for ln <= 0 or a zero limit): TireModel.into_ellipse.
func _into_ellipse(lx: float, ly: float, ln: float) -> Vector2:
	return TireModel.into_ellipse(lx, ly, ln, _mu_x, _mu_y)


func _solve_tire() -> void:
	var row_x = rows[ROW_TANGENT_1]
	var row_y = rows[ROW_TANGENT_2]
	var ln: float = rows[ROW_NORMAL].accumulated_impulse
	var cur_x: float = row_x.accumulated_impulse
	var cur_y: float = row_y.accumulated_impulse
	var target: Vector2 = Vector2.ZERO
	if _touching and tire_forces_enabled and _axes_valid and ln > 0.0 and _mu_x > 0.0 and _mu_y > 0.0:
		var s_x: float = _sx_lin.dot(body_b.linear_velocity) + _sx_ang.dot(body_b.angular_velocity)
		var s_y: float = _sy_lin.dot(body_b.linear_velocity) + _sy_ang.dot(body_b.angular_velocity)
		target = TireModel.implicit_tangential_impulse(s_x, s_y, ln, cur_x, cur_y, _g00, _g01, _g10, _g11, tire.longitudinal_stiffness,
			tire.lateral_stiffness, _mu_x, _mu_y, _reference_speed)
	row_x.accumulated_impulse = target.x
	row_y.accumulated_impulse = target.y
	row_x.apply_impulse(target.x - cur_x)
	row_y.apply_impulse(target.y - cur_y)
	_solve_rolling_resistance(ln)


## Rolling-resistance angular impulse about the axle, a function of the current normal impulse and the frozen spin; the difference to
## the amount already applied this step is applied (so it follows the load as the normal row converges, and is never warm started).
func _solve_rolling_resistance(ln: float) -> void:
	var target: float = 0.0
	if _touching and tire_forces_enabled and _axes_valid and ln > 0.0:
		# TireModel.rolling_resistance_force(v, N, c, v_rr) = -c N psi(v / v_rr); the moment about +u opposes a positive spin.
		target = -TireModel.rolling_resistance_force(tire.rolling_radius * _spin, ln, tire.rolling_resistance, tire.rolling_resistance_speed) * tire.rolling_radius
	var delta: float = target - _rolling_angular_impulse
	_rolling_angular_impulse = target
	body_b.angular_velocity += body_b.inverse_inertia_world * (_axle * delta)
