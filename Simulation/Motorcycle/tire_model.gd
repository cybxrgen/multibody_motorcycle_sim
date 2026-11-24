extends RefCounted
## Pure tyre mathematics (M2.4.2): slip from a rigid-body state, the combined-slip force law and rolling resistance. No state, no
## world, no solver: every function is static and finite for every finite input. The solver integration (an implicit tangential
## response inside the contact solve) is TireGroundContact; this file holds the definitions the tests check independently.
## Full derivation and sign conventions: PHYSICS_CONVENTIONS.md ("Tyre behaviour").
##
## ---- Tyre axes (SAE-like: x forward, y right, z down = -n) ----
##   n  plane normal (up), u  wheel axle in world coordinates (body-local +X, i.e. the wheel's right),
##   e_x = (n x u) / |n x u|          longitudinal axis: the rolling direction projected on the ground (forward for the prototype),
##   e_y = (u - n (n.u)) / |u - n (n.u)| = e_x x n   lateral axis: the axle projected on the ground, pointing to the wheel's right.
##   Both are exact for a tilted (cambered) wheel and defined while |n x u| >= MIN_PLANE_PROJECTION.
## ---- Slip (contact-point velocity relative to the static ground) ----
##   v_c = v + w x (p_c - x)          velocity of the wheel material at the contact point p_c (COM position x, COM velocity v)
##   hub speed   v_hub = v . e_x       spin  Omega = -(w . u)  (positive for forward rolling: the wheel turns about -u)
##   slip velocities (tread relative to ground):   s_x = -(v_c . e_x) + (R_eff - R) Omega,   s_y = -(v_c . e_y)
##   reference speed  D = sqrt(v_hub^2 + v_ref^2)      slip  sigma_x = s_x / D,  sigma_y = s_y / D     (finite at zero speed)
##   sigma_x > 0: DRIVING (tread faster than the ground, force forward); sigma_x < 0: BRAKING (force backward; locked wheel at
##   forward speed v: sigma_x -> -1). sigma_y > 0: the contact point slides to the wheel's LEFT, the restoring force points RIGHT.
## ---- Force law (friction ellipse with a smooth brush-type saturation) ----
##   e_x' = k_x sigma_x / mu_x,  e_y' = k_y sigma_y / mu_y,  rho = |(e_x', e_y')|
##   g(rho) = rho - rho^2/3 + rho^3/27 (rho < 3), 1 (rho >= 3);   phi(rho) = g(rho) / rho   (phi(0) = 1)
##   F_x = phi k_x N sigma_x,  F_y = phi k_y N sigma_y   =>   (F_x/(mu_x N))^2 + (F_y/(mu_y N))^2 = g(rho)^2 <= 1 always.

const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")

const MIN_PLANE_PROJECTION: float = WheelGroundContact.MIN_PLANE_PROJECTION
const WHEEL_AXLE_LOCAL: Vector3 = Vector3.RIGHT


## Tyre axes for the axle `u` (unit) and plane normal `n` (unit). {valid, longitudinal, lateral, projection} with
## projection = |n x u|; valid only when it is >= MIN_PLANE_PROJECTION and the inputs are finite.
static func axes(u: Vector3, n: Vector3) -> Dictionary:
	var result: Dictionary = {"valid": false, "longitudinal": Vector3.ZERO, "lateral": Vector3.ZERO, "projection": 0.0}
	if not (u.is_finite() and n.is_finite()):
		return result
	var cross: Vector3 = n.cross(u)
	var length: float = cross.length()
	result.projection = length
	if not (length >= MIN_PLANE_PROJECTION):
		return result
	var e_x: Vector3 = cross / length
	result.longitudinal = e_x
	result.lateral = e_x.cross(n)
	result.valid = true
	return result


## Slip of one wheel from its rigid-body state, with the contact point velocity evaluated literally as v + w x (p_c - x).
## `contact_point` = null uses the wheel's lowest point (WheelGroundContact geometry). Returns a Dictionary:
##   valid, reason, longitudinal_axis, lateral_axis, contact_point, contact_velocity (Vector3), tangential_velocity_x / _y (v_c . e),
##   hub_speed, spin, reference_speed, slip_velocity_x / _y (m/s), slip_x, slip_y (dimensionless).
static func slip(wheel_position: Vector3, wheel_orientation: Quaternion, linear_velocity: Vector3, angular_velocity: Vector3,
		geometry_radius: float, rolling_radius: float, plane_point: Vector3, plane_normal: Vector3, low_speed_reference: float,
		contact_point = null) -> Dictionary:
	var result: Dictionary = {"valid": false, "reason": "", "longitudinal_axis": Vector3.ZERO, "lateral_axis": Vector3.ZERO,
		"contact_point": Vector3.ZERO, "contact_velocity": Vector3.ZERO, "tangential_velocity_x": 0.0, "tangential_velocity_y": 0.0,
		"hub_speed": 0.0, "spin": 0.0, "reference_speed": maxf(low_speed_reference, 0.0), "slip_velocity_x": 0.0, "slip_velocity_y": 0.0,
		"slip_x": 0.0, "slip_y": 0.0}
	if not (linear_velocity.is_finite() and angular_velocity.is_finite() and is_finite(rolling_radius) and is_finite(low_speed_reference) and low_speed_reference > 0.0):
		result.reason = "non-finite velocity or tyre parameter"
		return result
	var geometry: Dictionary = WheelGroundContact.compute_geometry(wheel_position, wheel_orientation, geometry_radius, plane_point, plane_normal)
	if not geometry.valid:
		result.reason = geometry.reason
		return result
	var u: Vector3 = geometry.axle
	var tyre_axes: Dictionary = axes(u, plane_normal)
	if not tyre_axes.valid:
		result.reason = "degenerate tyre axes"
		return result
	var p_c: Vector3 = geometry.wheel_point if contact_point == null else contact_point
	var v_c: Vector3 = linear_velocity + angular_velocity.cross(p_c - wheel_position)
	var e_x: Vector3 = tyre_axes.longitudinal
	var e_y: Vector3 = tyre_axes.lateral
	var spin: float = -angular_velocity.dot(u)
	var hub: float = linear_velocity.dot(e_x)
	var d: float = sqrt(hub * hub + low_speed_reference * low_speed_reference)
	var vcx: float = v_c.dot(e_x)
	var vcy: float = v_c.dot(e_y)
	var s_x: float = -vcx + (rolling_radius - geometry_radius) * spin
	var s_y: float = -vcy
	result.valid = true
	result.longitudinal_axis = e_x
	result.lateral_axis = e_y
	result.contact_point = p_c
	result.contact_velocity = v_c
	result.tangential_velocity_x = vcx
	result.tangential_velocity_y = vcy
	result.hub_speed = hub
	result.spin = spin
	result.reference_speed = d
	result.slip_velocity_x = s_x
	result.slip_velocity_y = s_y
	result.slip_x = s_x / d
	result.slip_y = s_y / d
	return result


## g(rho): normalised force magnitude of the brush-type saturation, 0 <= g <= 1, g(0) = 0, g'(0) = 1, g(rho >= 3) = 1.
static func saturation(rho: float) -> float:
	if rho <= 0.0:
		return 0.0
	if rho >= 3.0:
		return 1.0
	return rho - rho * rho / 3.0 + rho * rho * rho / 27.0


## phi(rho) = g(rho) / rho, the factor by which the linear force is reduced (1 at rho = 0, 1/rho for rho >= 3).
static func reduction(rho: float) -> float:
	if rho <= 1.0e-9:
		return 1.0
	return saturation(rho) / rho


## Combined-slip force for the slips (slip_x, slip_y), normal load N >= 0 (newtons) and the friction limits mu_x, mu_y > 0.
## Returns {fx, fy, utilization, rho, phi}; utilization = sqrt((fx/(mu_x N))^2 + (fy/(mu_y N))^2) in [0, 1]. Zero force for
## N <= 0 or a non-positive limit. Not clamped per axis: the one common reduction factor phi keeps (fx, fy) inside the ellipse.
static func force(slip_x: float, slip_y: float, normal_load: float, mu_x: float, mu_y: float, k_x: float, k_y: float) -> Dictionary:
	var result: Dictionary = {"fx": 0.0, "fy": 0.0, "utilization": 0.0, "rho": 0.0, "phi": 1.0}
	if not (normal_load > 0.0 and mu_x > 0.0 and mu_y > 0.0):
		return result
	var ex: float = k_x * slip_x / mu_x
	var ey: float = k_y * slip_y / mu_y
	var rho: float = sqrt(ex * ex + ey * ey)
	var phi: float = reduction(rho)
	result.fx = phi * k_x * normal_load * slip_x
	result.fy = phi * k_y * normal_load * slip_y
	result.rho = rho
	result.phi = phi
	result.utilization = saturation(rho)
	return result


## Rolling resistance along the longitudinal axis (newtons, signed): -c_rr N psi(v / v_rr), psi(x) = x / sqrt(1 + x^2).
## Opposes the rolling direction, proportional to the normal load, exactly zero at zero rolling speed, zero without load.
static func rolling_resistance_force(rolling_speed: float, normal_load: float, coefficient: float, speed_scale: float) -> float:
	if not (normal_load > 0.0 and coefficient > 0.0 and speed_scale > 0.0):
		return 0.0
	var x: float = rolling_speed / speed_scale
	return -coefficient * normal_load * x / sqrt(1.0 + x * x)
