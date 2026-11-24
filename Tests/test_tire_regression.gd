extends "res://Tests/sim_test_case.gd"
## M2.4.2 H: regression sensitivity. Each MUTANT below is a deliberately wrong TireGroundContact (one defect each). A PROBE is a
## physical check written against the correct behaviour; every probe must pass for the real contact and FAIL for the mutant it targets.
## This shows that the suite would notice reversed signs, a missing angular-velocity term, a wrong axle or rolling radius, force without
## contact, a stale load, independent per-axis clamping, rolling resistance that does not vanish at rest, and duplicated friction.

const Rig = preload("res://Tests/tire_rig.gd")
const TireGroundContact = preload("res://Simulation/Motorcycle/tire_ground_contact.gd")
const TireModel = preload("res://Simulation/Motorcycle/tire_model.gd")

const DT: float = 1.0 / 120.0


class ReversedLongitudinal extends "res://Simulation/Motorcycle/tire_ground_contact.gd":
	func _solve_tire() -> void:
		super()
		var current: float = rows[ROW_TANGENT_1].accumulated_impulse
		rows[ROW_TANGENT_1].apply_impulse(-2.0 * current)
		rows[ROW_TANGENT_1].accumulated_impulse = -current


class ReversedLateral extends "res://Simulation/Motorcycle/tire_ground_contact.gd":
	func _solve_tire() -> void:
		super()
		var current: float = rows[ROW_TANGENT_2].accumulated_impulse
		rows[ROW_TANGENT_2].apply_impulse(-2.0 * current)
		rows[ROW_TANGENT_2].accumulated_impulse = -current


class OmittedAngularVelocity extends "res://Simulation/Motorcycle/tire_ground_contact.gd":
	func _update_rows(dt: float, config) -> void:
		super._update_rows(dt, config)
		_sx_ang = Vector3.ZERO
		_sy_ang = Vector3.ZERO


class WrongAxle extends "res://Simulation/Motorcycle/tire_ground_contact.gd":
	# Takes the body-local Z axis (not X) as the axle when building the tyre axes.
	func _update_rows(dt: float, config) -> void:
		super._update_rows(dt, config)
		var wrong: Dictionary = TireModel.axes(body_b.orientation * Vector3.BACK, _normal)
		if wrong.valid:
			_e_x = wrong.longitudinal
			_e_y = wrong.lateral
			_set_row(rows[ROW_TANGENT_1], _e_x, _point - body_a.position, _point - body_b.position)
			_set_row(rows[ROW_TANGENT_2], _e_y, _point - body_a.position, _point - body_b.position)


class WrongRollingRadius extends "res://Simulation/Motorcycle/tire_ground_contact.gd":
	# Ignores the effective rolling radius: always uses the geometric radius.
	func _update_rows(dt: float, config) -> void:
		super._update_rows(dt, config)
		_sx_ang = -rows[ROW_TANGENT_1].angular_b


class ForceWithoutContact extends "res://Simulation/Motorcycle/tire_ground_contact.gd":
	# Applies a longitudinal impulse from the slip even when the contact is inactive (constant load m g assumed).
	func solve_iteration() -> void:
		if not contact_active:
			var u: Vector3 = body_b.orientation * Vector3.RIGHT
			body_b.angular_velocity += body_b.inverse_inertia_world * (u * -0.05)
			return
		super()


class StaleLoad extends "res://Simulation/Motorcycle/tire_ground_contact.gd":
	# Does not discard its carried impulses when the contact becomes inactive.
	func reset_warm_start() -> void:
		pass


class IndependentClamp extends "res://Simulation/Motorcycle/tire_ground_contact.gd":
	# Per-axis law: each axis saturates at its OWN full limit independently (no shared traction budget).
	func _solve_tire() -> void:
		var ln: float = rows[ROW_NORMAL].accumulated_impulse
		if not (ln > 0.0):
			return
		var s_x: float = _sx_lin.dot(body_b.linear_velocity) + _sx_ang.dot(body_b.angular_velocity)
		var s_y: float = _sy_lin.dot(body_b.linear_velocity) + _sy_ang.dot(body_b.angular_velocity)
		var want_x: float = clampf(tire.longitudinal_stiffness * ln * s_x / _reference_speed, -_mu_x * ln, _mu_x * ln)
		var want_y: float = clampf(tire.lateral_stiffness * ln * s_y / _reference_speed, -_mu_y * ln, _mu_y * ln)
		rows[ROW_TANGENT_1].apply_impulse(want_x - rows[ROW_TANGENT_1].accumulated_impulse)
		rows[ROW_TANGENT_2].apply_impulse(want_y - rows[ROW_TANGENT_2].accumulated_impulse)
		rows[ROW_TANGENT_1].accumulated_impulse = want_x
		rows[ROW_TANGENT_2].accumulated_impulse = want_y


class ConstantRollingResistance extends "res://Simulation/Motorcycle/tire_ground_contact.gd":
	# Magnitude c_rr N R at any non-zero spin (sign function instead of the smooth psi).
	func _solve_rolling_resistance(ln: float) -> void:
		var target: float = 0.0
		if tire_forces_enabled and ln > 0.0 and absf(_spin) > 0.0:
			target = signf(_spin) * tire.rolling_resistance * ln * tire.rolling_radius
		var delta: float = target - _rolling_angular_impulse
		_rolling_angular_impulse = target
		body_b.angular_velocity += body_b.inverse_inertia_world * (_axle * delta)


class DuplicateFriction extends "res://Simulation/Motorcycle/tire_ground_contact.gd":
	# The tyre solve PLUS a second, full-strength generic box-Coulomb friction on the same rows.
	func _solve_tire() -> void:
		super()
		var ln: float = rows[ROW_NORMAL].accumulated_impulse
		var limits: Array = [friction * ln, friction * ln]
		for k in 2:
			var row = rows[ROW_TANGENT_1 + k]
			var denominator: float = body_b.inverse_mass * row.linear_b.dot(row.linear_b) + row.angular_b.dot(body_b.inverse_inertia_world * row.angular_b)
			if denominator <= 0.0:
				continue
			var wanted: float = row.accumulated_impulse - row.compute_velocity() / denominator
			var clamped: float = clampf(wanted, -limits[k], limits[k])
			row.apply_impulse(clamped - row.accumulated_impulse)
			row.accumulated_impulse = clamped


# ------------------------------------------------------------------ probes (true = behaves correctly)

func probe_rows_match_state(script) -> bool:
	for c in [
		{"yaw": 0.7, "camber": 0.3, "forward_speed": 3.0, "lateral_speed": -2.0, "spin_ratio": 1.3, "omega_extra": Vector3(0.4, -0.6, 0.9)},
		{"yaw": 0.0, "camber": 0.0, "forward_speed": 6.0, "lateral_speed": 1.0, "spin_ratio": 0.8},
	]:
		c.spec_overrides = {"rolling_radius": 0.29}
		c.contact_script = script
		var rig: Dictionary = Rig.wheel_rig(c)
		var contact = rig.contact
		contact.prepare(DT, rig.world.config)
		var wheel = rig.wheel
		var s: Dictionary = TireModel.slip(wheel.position, wheel.orientation, wheel.linear_velocity, wheel.angular_velocity, rig.radius,
			rig.spec.rolling_radius, Vector3.ZERO, Vector3.UP, rig.spec.low_speed_reference, contact._point)
		var sx: float = contact._sx_lin.dot(wheel.linear_velocity) + contact._sx_ang.dot(wheel.angular_velocity)
		var sy: float = contact._sy_lin.dot(wheel.linear_velocity) + contact._sy_ang.dot(wheel.angular_velocity)
		if absf(sx - s.slip_velocity_x) > 1.0e-4 or absf(sy - s.slip_velocity_y) > 1.0e-4:
			return false
		if (contact.longitudinal_axis() - s.longitudinal_axis).length() > 1.0e-5 or (contact.lateral_axis() - s.lateral_axis).length() > 1.0e-5:
			return false
	return true


func probe_directions(script) -> bool:
	var driven: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spin_ratio": 1.1, "contact_script": script})
	driven.world.step()
	var braked: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spin_ratio": 0.9, "contact_script": script})
	braked.world.step()
	var side: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "lateral_speed": 0.5, "contact_script": script})
	side.world.step()
	var other: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "lateral_speed": -0.5, "contact_script": script})
	other.world.step()
	return driven.contact.longitudinal_force() > 1.0 and braked.contact.longitudinal_force() < -1.0 \
		and side.contact.lateral_force() < -1.0 and other.contact.lateral_force() > 1.0


func probe_no_action_without_contact(script) -> bool:
	var rig: Dictionary = Rig.wheel_rig({"height": 0.2, "forward_speed": 5.0, "spin_ratio": 0.5, "contact_script": script})
	var spin: float = -rig.wheel.angular_velocity.dot(rig.u)
	for i in 5:
		rig.world.step()
	return absf(-rig.wheel.angular_velocity.dot(rig.u) - spin) < 1.0e-3 and rig.contact.longitudinal_force() == 0.0


func probe_no_stale_state_after_separation(script) -> bool:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spin_ratio": 0.5, "lateral_speed": 0.5, "contact_script": script})
	for i in 4:
		rig.world.step()
	rig.wheel.position += Vector3(0.0, 0.2, 0.0)
	rig.world.step()
	var c = rig.contact
	return c.normal_load() == 0.0 and c.longitudinal_force() == 0.0 and c.lateral_force() == 0.0 and c._tire_impulse_world == Vector3.ZERO


func probe_combined_envelope(script) -> bool:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 8.0, "lateral_speed": 3.0, "spin": 0.0, "spec_overrides": {"lateral_stiffness": 15.0}, "contact_script": script})
	rig.world.step()
	var c = rig.contact
	var n: float = c.normal_load()
	var ux: float = c.longitudinal_force() / (0.8 * n)
	var uy: float = c.lateral_force() / (0.8 * n)
	return ux * ux + uy * uy <= 1.0 + 1.0e-3


func probe_rolling_resistance_vanishes_at_rest(script) -> bool:
	var rig: Dictionary = Rig.wheel_rig({"spin": 0.002 / 0.32, "contact_script": script})  # 2 mm/s tread speed
	rig.world.step()
	var full: float = 0.015 * rig.contact.normal_load() * rig.radius
	return absf(rig.contact.rolling_resistance_torque()) < 0.05 * full


func probe_wheel_free_rolling_keeps_speed_without_resistance(script) -> bool:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spec_overrides": {"rolling_resistance": 0.0}, "contact_script": script})
	for i in 60:
		rig.world.step()
	return absf(rig.wheel.linear_velocity.dot(rig.e_x) - 6.0) < 1.0e-3


func test_h1_the_probes_accept_the_real_contact() -> void:
	for probe in ["probe_rows_match_state", "probe_directions", "probe_no_action_without_contact", "probe_no_stale_state_after_separation",
			"probe_combined_envelope", "probe_rolling_resistance_vanishes_at_rest", "probe_wheel_free_rolling_keeps_speed_without_resistance"]:
		check(call(probe, TireGroundContact), "real contact passes %s" % probe)


func test_h2_reversed_longitudinal_force_sign_is_detected() -> void:
	check(not probe_directions(ReversedLongitudinal), "reversed Fx sign")


func test_h3_reversed_lateral_force_sign_is_detected() -> void:
	check(not probe_directions(ReversedLateral), "reversed Fy sign")


func test_h4_omitted_wheel_angular_velocity_in_slip_is_detected() -> void:
	check(not probe_rows_match_state(OmittedAngularVelocity), "slip rows without the angular term")


func test_h5_wrong_axle_orientation_is_detected() -> void:
	check(not probe_rows_match_state(WrongAxle), "tyre axes built from the wrong axle")


func test_h6_wrong_rolling_radius_is_detected() -> void:
	check(not probe_rows_match_state(WrongRollingRadius), "slip that ignores R_eff")
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spec_overrides": {"rolling_radius": 0.30}, "contact_script": WrongRollingRadius})
	rig.world.step()
	var good: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spec_overrides": {"rolling_radius": 0.30}})
	good.world.step()
	check(absf(rig.contact.longitudinal_force() - good.contact.longitudinal_force()) > 5.0, "and its force differs measurably from the correct one")


func test_h7_force_without_contact_is_detected() -> void:
	check(not probe_no_action_without_contact(ForceWithoutContact), "tyre action on a flying wheel")


func test_h8_stale_load_after_separation_is_detected() -> void:
	check(not probe_no_stale_state_after_separation(StaleLoad), "carried impulses survive separation")


func test_h9_independent_per_axis_clamping_is_detected() -> void:
	check(not probe_combined_envelope(IndependentClamp), "Fx and Fy clamped separately exceed the ellipse")


func test_h10_rolling_resistance_that_does_not_vanish_at_rest_is_detected() -> void:
	check(not probe_rolling_resistance_vanishes_at_rest(ConstantRollingResistance), "sign() instead of a smooth rise")


func test_h11_duplicate_generic_and_tyre_friction_is_detected() -> void:
	check(not probe_combined_envelope(DuplicateFriction), "a second full-strength friction on top of the tyre force")


func test_h12_a_wheel_with_a_tyre_has_no_generic_friction_path() -> void:
	# Structural guarantee: the generic friction hook of the tyre contact does nothing and the tyre contact never accumulates a
	# generic friction impulse; with the tyre disabled the contact is frictionless (not generic Coulomb).
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spin": 0.0})
	rig.contact.tire_forces_enabled = false
	for i in 10:
		rig.world.step()
	check_near(rig.wheel.linear_velocity.dot(rig.e_x), 6.0, 1.0e-6, "disabled tyre: an M2.4.1 Coulomb contact would have slowed the locked wheel; this one does not")
	check_near(rig.contact.friction_impulse_magnitude(), 0.0, 0.0, "no tangential impulse at all")
	rig.contact.tire_forces_enabled = true
	var before: float = rig.wheel.linear_velocity.dot(rig.e_x)
	rig.world.step()
	check(rig.wheel.linear_velocity.dot(rig.e_x) < before, "re-enabled: the tyre brakes the sliding wheel again")
