extends "res://Tests/sim_test_case.gd"
## M2.4.2 A: slip calculation. Expected values are derived by hand from the definitions (PHYSICS_CONVENTIONS.md, "Tyre behaviour"), not
## by calling the implementation's own helper: the wheel axle is +X, forward is -Z, up is +Y, so a forward-rolling wheel turns about -X
## (angular velocity (-v/R, 0, 0)) and the contact point at the bottom, r = (0, -R, 0), has velocity v_com + w x r.

const Rig = preload("res://Tests/tire_rig.gd")
const TireModel = preload("res://Simulation/Motorcycle/tire_model.gd")
const TireGroundContact = preload("res://Simulation/Motorcycle/tire_ground_contact.gd")

const R: float = 0.32
const VREF: float = 0.5


func slip(pos: Vector3, q: Quaternion, v: Vector3, w: Vector3, r_eff: float = R) -> Dictionary:
	return TireModel.slip(pos, q, v, w, R, r_eff, Vector3.ZERO, Vector3.UP, VREF)


func upright(v: Vector3, w: Vector3, r_eff: float = R) -> Dictionary:
	return slip(Vector3(0.0, R, 0.0), Quaternion.IDENTITY, v, w, r_eff)


func test_a1_stationary_wheel_has_zero_slip_and_finite_denominator() -> void:
	var s: Dictionary = upright(Vector3.ZERO, Vector3.ZERO)
	check(s.valid, "valid geometry")
	check_near(s.slip_x, 0.0, 0.0, "slip_x of a stationary wheel")
	check_near(s.slip_y, 0.0, 0.0, "slip_y of a stationary wheel")
	check_near(s.reference_speed, VREF, 1.0e-7, "the reference speed is the low-speed reference at rest (no division by zero)")
	check_vec3_near(s.contact_velocity, Vector3.ZERO, 0.0, "contact point at rest")


func test_a2_pure_rolling_has_zero_slip() -> void:
	# v = 7 m/s forward (-Z), spin about -X at v/R: the contact point velocity is (0,0,-7) + (-7/R,0,0) x (0,-R,0) = (0,0,-7) + (0,0,-7)*(-1)... = 0.
	var v: float = 7.0
	var s: Dictionary = upright(Vector3(0.0, 0.0, -v), Vector3(-v / R, 0.0, 0.0))
	check_vec3_near(s.contact_velocity, Vector3.ZERO, 1.0e-5, "a rolling wheel's contact point is at rest relative to the ground")
	check_near(s.slip_x, 0.0, 1.0e-6, "pure rolling: zero longitudinal slip")
	check_near(s.slip_y, 0.0, 1.0e-6, "pure rolling: zero lateral slip")
	check_near(s.hub_speed, v, 1.0e-6, "hub speed along the longitudinal axis (forward positive)")
	check_near(s.spin, v / R, 1.0e-5, "spin is positive for forward rolling")


func test_a3_rolling_radius_defines_the_zero_slip_spin() -> void:
	# A tyre with R_eff = 0.30 on a 0.32 m geometry rolls freely at spin = v / R_eff (kinematic rolling radius): zero slip there, not at v/R.
	var v: float = 6.0
	var r_eff: float = 0.30
	var s_free: Dictionary = upright(Vector3(0.0, 0.0, -v), Vector3(-v / r_eff, 0.0, 0.0), r_eff)
	check_near(s_free.slip_x, 0.0, 1.0e-6, "zero slip at spin = v / R_eff")
	var s_geo: Dictionary = upright(Vector3(0.0, 0.0, -v), Vector3(-v / R, 0.0, 0.0), r_eff)
	# spin v/R: tread speed R_eff * v/R = v * 0.9375 -> slip velocity -0.375 m/s
	check_near(s_geo.slip_velocity_x, r_eff * v / R - v, 1.0e-5, "slip velocity with the wrong spin: R_eff * spin - v")
	check(s_geo.slip_x < 0.0, "a wheel turning slower than v / R_eff is braking")


func test_a4_driven_wheel_has_positive_slip() -> void:
	# spin 10 % above rolling at v = 8: tread speed 8.8, slip velocity +0.8, D = sqrt(64 + 0.25).
	var v: float = 8.0
	var s: Dictionary = upright(Vector3(0.0, 0.0, -v), Vector3(-1.1 * v / R, 0.0, 0.0))
	check_near(s.slip_velocity_x, 0.8, 1.0e-5, "driving slip velocity")
	check_near(s.slip_x, 0.8 / sqrt(64.25), 1.0e-6, "driving slip ratio, normalised by sqrt(v^2 + v_ref^2)")
	check(s.slip_x > 0.0, "driving slip is positive")
	check_near(s.slip_y, 0.0, 1.0e-6, "no lateral slip")


func test_a5_braking_and_locked_wheel_have_negative_slip() -> void:
	var v: float = 8.0
	var half: Dictionary = upright(Vector3(0.0, 0.0, -v), Vector3(-0.5 * v / R, 0.0, 0.0))
	check_near(half.slip_x, -4.0 / sqrt(64.25), 1.0e-6, "half-speed wheel: slip velocity -4 m/s")
	var locked: Dictionary = upright(Vector3(0.0, 0.0, -20.0), Vector3.ZERO)
	check_near(locked.slip_x, -20.0 / sqrt(400.25), 1.0e-6, "locked wheel at 20 m/s: slip -> -1")
	check(locked.slip_x < -0.999 and locked.slip_x > -1.0, "a locked wheel at high speed approaches -1 and never goes below it")
	var reverse: Dictionary = upright(Vector3(0.0, 0.0, 4.0), Vector3(2.0 / R, 0.0, 0.0))
	# Rolling backwards at 4 m/s with spin only half of the rolling value (w = +2/R about +X): hub speed -4, tread speed -2: slip +2/... positive:
	check_near(reverse.hub_speed, -4.0, 1.0e-6, "backward hub speed is negative")
	check_near(reverse.slip_velocity_x, R * (-2.0 / R) - (-4.0), 1.0e-5, "reverse braking: tread -2, hub -4: slip velocity +2 (it brakes a backward wheel)")


func test_a6_lateral_slip_sign_and_size() -> void:
	# Wheel rolling forward at 10 m/s while its hub also moves 2 m/s to the RIGHT (+X): the contact point slides right, sigma_y < 0
	# (positive lateral slip = the contact slides to the LEFT), force to the left restores.
	var s: Dictionary = upright(Vector3(2.0, 0.0, -10.0), Vector3(-10.0 / R, 0.0, 0.0))
	check_near(s.slip_velocity_y, -2.0, 1.0e-5, "contact point slides right: slip velocity -2")
	check_near(s.slip_y, -2.0 / sqrt(100.25), 1.0e-6, "lateral slip = -v_c.e_y / D")
	check_near(s.slip_x, 0.0, 1.0e-6, "no longitudinal slip")
	var left: Dictionary = upright(Vector3(-2.0, 0.0, -10.0), Vector3(-10.0 / R, 0.0, 0.0))
	check_near(left.slip_y, 2.0 / sqrt(100.25), 1.0e-6, "mirror image: hub moving left gives positive lateral slip")


func test_a7_combined_slip_is_the_independent_pair() -> void:
	var s: Dictionary = upright(Vector3(1.5, 0.0, -6.0), Vector3(-0.8 * 6.0 / R, 0.0, 0.0))
	# braking 20 %: tread 4.8 vs hub 6 -> -1.2 m/s; lateral: contact slides right at 1.5 -> -1.5 m/s
	var d: float = sqrt(36.0 + 0.25)
	check_near(s.slip_x, -1.2 / d, 1.0e-6, "combined: longitudinal part")
	check_near(s.slip_y, -1.5 / d, 1.0e-6, "combined: lateral part")


func test_a8_yawed_wheel_resolves_velocity_in_its_own_axes() -> void:
	# Yaw 40 degrees about +Y: axle u = (cos, 0, -sin); forward e_x = n x u = (-sin, 0, -cos); right e_y = u (derived by hand).
	var psi: float = deg_to_rad(40.0)
	var q: Quaternion = Quaternion(Vector3.UP, psi)
	var forward: Vector3 = Vector3(-sin(psi), 0.0, -cos(psi))
	var right: Vector3 = Vector3(cos(psi), 0.0, -sin(psi))
	var v: Vector3 = forward * 9.0 + right * 1.0
	var w: Vector3 = -right * (9.0 * 0.9 / R)  # braking 10 %
	var s: Dictionary = slip(Vector3(0.0, R, 0.0), q, v, w)
	var d: float = sqrt(81.0 + 0.25)
	check_vec3_near(s.longitudinal_axis, forward, 1.0e-6, "longitudinal axis of a yawed wheel")
	check_vec3_near(s.lateral_axis, right, 1.0e-6, "lateral axis of a yawed wheel")
	check_near(s.slip_x, (0.9 * 9.0 - 9.0) / d, 1.0e-5, "braking slip is independent of the heading")
	check_near(s.slip_y, -1.0 / d, 1.0e-5, "lateral slip is independent of the heading")


func test_a9_roll_rate_contributes_to_the_contact_point_velocity() -> void:
	# Angular velocity about the forward axis (-Z) of 1.5 rad/s, hub at rest: w x r = (0,0,-1.5) x (0,-R,0) = (-1.5 R, 0, 0): the contact
	# point moves LEFT at 1.5 R = 0.48 m/s, i.e. lateral slip velocity +0.48 (omitting w x r would give zero).
	var s: Dictionary = upright(Vector3.ZERO, Vector3(0.0, 0.0, -1.5))
	check_vec3_near(s.contact_velocity, Vector3(-1.5 * R, 0.0, 0.0), 1.0e-6, "contact point velocity includes w x r")
	check_near(s.slip_velocity_y, 1.5 * R, 1.0e-6, "roll rate gives lateral slip velocity")
	check_near(s.slip_x, 0.0, 1.0e-7, "and no longitudinal slip")
	# A yaw rate about the vertical axis does not move the lowest point of an upright wheel.
	var yaw: Dictionary = upright(Vector3.ZERO, Vector3(0.0, 2.0, 0.0))
	check_vec3_near(yaw.contact_velocity, Vector3.ZERO, 1.0e-6, "yaw rate leaves an upright contact point at rest")


func test_a10_spin_alone_at_standstill_gives_regularised_driving_slip() -> void:
	# Spinning in place at 5 rad/s (tread 1.6 m/s): slip = 1.6 / v_ref = 3.2, finite (the rolling-speed formulation would divide by zero).
	var s: Dictionary = upright(Vector3.ZERO, Vector3(-5.0, 0.0, 0.0))
	check_near(s.slip_velocity_x, 5.0 * R, 1.0e-6, "tread speed of a wheel spinning in place")
	check_near(s.slip_x, 5.0 * R / VREF, 1.0e-6, "slip normalised by the low-speed reference only")
	check(is_finite(s.slip_x) and is_finite(s.slip_y), "finite")


func test_a11_tilted_wheel_uses_projected_axle_and_exact_geometry() -> void:
	# Camber 25 degrees about the forward axis. The lateral axis is the axle projected on the ground, the contact point is the
	# lowest point of the tilted circle. Rolling "free" speed is v = R * cos... derived: the lowest point of a tilted rolling circle has
	# zero velocity when w x r cancels v: use the library geometry for the point and check the velocity decomposition by hand.
	var camber: float = deg_to_rad(25.0)
	var q: Quaternion = Quaternion(Vector3(0.0, 0.0, -1.0), camber)
	var u: Vector3 = q * Vector3.RIGHT
	var projected: Vector3 = (u - Vector3.UP * u.y).normalized()
	var geometry: Dictionary = TireModel.slip(Vector3(0.0, R * cos(camber), 0.0), q, Vector3.ZERO, Vector3.ZERO, R, R, Vector3.ZERO, Vector3.UP, VREF)
	check_vec3_near(geometry.lateral_axis, projected, 1.0e-6, "lateral axis = projected axle")
	check_vec3_near(geometry.longitudinal_axis, Vector3(0.0, 0.0, -1.0), 1.0e-6, "longitudinal axis stays along -Z for a roll about -Z")
	check_near(geometry.contact_point.y, 0.0, 1.0e-6, "the lowest point of the tilted circle is on the plane (centre height R cos(camber))")
	# Hub speed 5 m/s forward, spin chosen so the contact point is at rest along e_x: Omega R = v (the e_x contact velocity only
	# contains v and R * spin for any tilt: (r x e_x) = R u).
	var rolling: Dictionary = TireModel.slip(Vector3(0.0, R * cos(camber), 0.0), q, Vector3(0.0, 0.0, -5.0), -u * (5.0 / R), R, R, Vector3.ZERO, Vector3.UP, VREF)
	check_near(rolling.slip_x, 0.0, 1.0e-5, "free rolling of a tilted wheel has zero longitudinal slip")
	# Roll rate of a tilted wheel: contact lateral velocity = -R cos(camber) * (w . e_x) (the r x e_y = -R s e_x identity with s = cos(camber)).
	var rolled: Dictionary = TireModel.slip(Vector3(0.0, R * cos(camber), 0.0), q, Vector3.ZERO, Vector3(0.0, 0.0, -1.0), R, R, Vector3.ZERO, Vector3.UP, VREF)
	check_near(rolled.slip_velocity_y, R * cos(camber), 1.0e-5, "tilted wheel: roll rate lateral contact velocity R cos(camber) per rad/s")


func test_a12_degenerate_and_invalid_inputs_are_reported_not_propagated() -> void:
	var flat: Dictionary = slip(Vector3(0.0, 0.1, 0.0), Quaternion(Vector3.BACK, PI / 2.0), Vector3.ZERO, Vector3.ZERO)
	check(not flat.valid and flat.reason != "", "a wheel lying flat has no tyre axes: invalid with a reason")
	check_near(flat.slip_x, 0.0, 0.0, "and reports zero slip, never NaN")
	var nan: Dictionary = upright(Vector3(NAN, 0.0, 0.0), Vector3.ZERO)
	check(not nan.valid, "non-finite velocity is rejected")
	var bad_ref: Dictionary = TireModel.slip(Vector3(0.0, R, 0.0), Quaternion.IDENTITY, Vector3.ZERO, Vector3.ZERO, R, R, Vector3.ZERO, Vector3.UP, 0.0)
	check(not bad_ref.valid, "a zero low-speed reference would reintroduce the zero denominator: rejected")
	for case in [[Vector3.ZERO, Vector3.ZERO], [Vector3(1e-12, 0.0, 1e-12), Vector3.ZERO], [Vector3(0, 0, -300.0), Vector3(0.0, 0.0, 0.0)], [Vector3(50, 0, -50), Vector3(-900.0, 3.0, 4.0)]]:
		var s: Dictionary = upright(case[0], case[1])
		check(s.valid and is_finite(s.slip_x) and is_finite(s.slip_y) and s.reference_speed >= VREF, "finite slip for %s" % [case])


func test_a13_axes_function_handles_tilt_and_rejects_degenerate() -> void:
	var up: Dictionary = TireModel.axes(Vector3.RIGHT, Vector3.UP)
	check_vec3_near(up.longitudinal, Vector3(0.0, 0.0, -1.0), 1.0e-7, "upright: forward is -Z")
	check_vec3_near(up.lateral, Vector3.RIGHT, 1.0e-7, "upright: lateral is +X (right)")
	check(not TireModel.axes(Vector3.UP, Vector3.UP).valid, "axle parallel to the normal: invalid")
	check(not TireModel.axes(Vector3(NAN, 0, 0), Vector3.UP).valid, "non-finite: invalid")
	var tilted: Dictionary = TireModel.axes((Quaternion(Vector3(0, 0, -1), 0.6) * Vector3.RIGHT), Vector3.UP)
	check_near(tilted.longitudinal.dot(Vector3.UP), 0.0, 1.0e-6, "longitudinal axis lies in the ground plane")
	check_near(tilted.lateral.dot(Vector3.UP), 0.0, 1.0e-6, "lateral axis lies in the ground plane")
	check_near(tilted.longitudinal.dot(tilted.lateral), 0.0, 1.0e-6, "and they are orthogonal")


func test_a14_solver_slip_rows_equal_the_state_formula() -> void:
	# The solver works with Jacobian rows; they must reproduce TireModel.slip (literal v + w x r) for arbitrary states: this is what
	# catches an omitted angular term or a wrong axle orientation in the solver path.
	var cases: Array = [
		{"yaw": 0.0, "camber": 0.0, "forward_speed": 6.0, "lateral_speed": 1.0, "spin_ratio": 0.8},
		{"yaw": 0.7, "camber": 0.3, "forward_speed": 3.0, "lateral_speed": -2.0, "spin_ratio": 1.3, "omega_extra": Vector3(0.4, -0.6, 0.9)},
		{"yaw": -1.2, "camber": -0.5, "forward_speed": -2.0, "lateral_speed": 0.5, "spin_ratio": 0.2, "omega_extra": Vector3(-1.0, 2.0, 0.3), "front": true},
		{"yaw": 0.0, "camber": 0.0, "forward_speed": 0.0, "lateral_speed": 0.0, "spin": 4.0},
	]
	for c in cases:
		c.spec_overrides = {"rolling_radius": 0.29 if not c.get("front", false) else 0.30}
		var rig: Dictionary = Rig.wheel_rig(c)
		var contact = rig.contact
		contact.prepare(Rig.DT, rig.world.config)
		check(contact.contact_active, "contact active for case %s" % [c])
		var wheel = rig.wheel
		var by_rows_x: float = contact._sx_lin.dot(wheel.linear_velocity) + contact._sx_ang.dot(wheel.angular_velocity)
		var by_rows_y: float = contact._sy_lin.dot(wheel.linear_velocity) + contact._sy_ang.dot(wheel.angular_velocity)
		# contact_point = null would use the wheel point; the contact acts at the (gap-shifted) shared point, equal here (gap 0).
		var s: Dictionary = TireModel.slip(wheel.position, wheel.orientation, wheel.linear_velocity, wheel.angular_velocity, rig.radius,
			rig.spec.rolling_radius, Vector3.ZERO, Vector3.UP, rig.spec.low_speed_reference, contact._point)
		check_near(by_rows_x, s.slip_velocity_x, 2.0e-5, "slip velocity x by rows vs state formula (case %s)" % [c])
		check_near(by_rows_y, s.slip_velocity_y, 2.0e-5, "slip velocity y by rows vs state formula (case %s)" % [c])
		check_vec3_near(contact.longitudinal_axis(), s.longitudinal_axis, 1.0e-6, "same longitudinal axis")
		var cs: Dictionary = contact.current_slip()
		check_near(cs.slip_x, s.slip_x, 1.0e-4, "current_slip() agrees (gap-shifted lever is the only difference)")
