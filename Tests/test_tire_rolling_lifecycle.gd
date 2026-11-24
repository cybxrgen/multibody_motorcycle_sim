extends "res://Tests/sim_test_case.gd"
## M2.4.2 E (rolling resistance) and F (contact and fixed-step lifecycle) on a single prototype wheel (Tests/tire_rig.gd).
## Rolling resistance is the axle moment M = c_rr N R_eff psi(R_eff spin / v_rr), psi(x) = x / sqrt(1 + x^2) (c_rr 0.015, v_rr 0.2 m/s).
## Free-wheel deceleration reference (hand derivation, rolling relation m a = F, I alpha = M - F R with alpha = a / R):
##   a = M / (R m (1 + k)) = c_rr g psi / (1 + k),  k = I_axle / (m R^2) = 0.78125.

const Rig = preload("res://Tests/tire_rig.gd")
const GroundRig = preload("res://Tests/ground_rig.gd")

const DT: float = 1.0 / 120.0
const C_RR: float = 0.015


func psi(x: float) -> float:
	return x / sqrt(1.0 + x * x)


func test_e1_resistance_opposes_the_rolling_direction() -> void:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 5.0})
	var c = rig.contact
	var v0: float = rig.wheel.linear_velocity.dot(rig.e_x)
	for i in 30:
		rig.world.step()
	check(c.rolling_resistance_torque() > 0.0, "forward rolling: the moment acts along +axle, against the spin (%s N m)" % c.rolling_resistance_torque())
	check(c.rolling_resistance_force() < 0.0, "the equivalent tread force points backward")
	check(rig.wheel.linear_velocity.dot(rig.e_x) < v0, "a free rolling wheel slows down")
	var back: Dictionary = Rig.wheel_rig({"forward_speed": -5.0})
	for i in 30:
		back.world.step()
	check(back.contact.rolling_resistance_torque() < 0.0, "backward rolling: the moment reverses")
	check(back.contact.rolling_resistance_force() > 0.0, "and the equivalent force points forward")
	check(back.wheel.linear_velocity.dot(back.e_x) > -5.0, "a wheel rolling backward slows down too")


func test_e2_resistance_is_proportional_to_the_normal_load() -> void:
	var torques: Array = []
	var loads: Array = []
	for scale in [1.0, 2.0, 3.0]:
		var rig: Dictionary = Rig.wheel_rig({"forward_speed": 5.0, "gravity_scale": scale})
		rig.world.step()
		torques.append(rig.contact.rolling_resistance_torque())
		loads.append(rig.contact.normal_load())
	check_near(torques[1] / torques[0], 2.0, 2.0e-3, "twice the load, twice the resistance")
	check_near(torques[2] / torques[0], 3.0, 3.0e-3, "three times the load, three times the resistance")
	# Absolute value: c_rr N R psi(R spin / v_rr) with spin = 5 / R -> psi(25).
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 5.0})
	rig.world.step()
	var n: float = rig.contact.normal_load()
	check_near(rig.contact.rolling_resistance_torque(), C_RR * n * rig.radius * psi(25.0), 5.0e-3 * C_RR * n * rig.radius, "M = c_rr N R psi(v / v_rr)")
	check_near(torques[0] / loads[0], C_RR * rig.radius * psi(25.0), 1.0e-4, "M / N is a constant of the tyre, not a fixed brake torque")


func test_e3_resistance_goes_smoothly_to_zero_with_the_rolling_speed() -> void:
	var previous: float = -1.0
	var saturation: float = 0.0
	for tread in [0.0, 0.005, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 5.0]:
		var rig: Dictionary = Rig.wheel_rig({"spin": tread / 0.32})
		rig.world.step()
		var m: float = rig.contact.rolling_resistance_torque()
		check(m >= previous - 1.0e-9, "non-decreasing with the rolling speed (tread %s m/s: %s N m)" % [tread, m])
		if tread == 5.0:
			saturation = m
		previous = m
		if tread == 0.0:
			check_near(m, 0.0, 0.0, "exactly zero at zero rolling speed")
	var small: Dictionary = Rig.wheel_rig({"spin": 0.02 / 0.32})
	small.world.step()
	check_near(small.contact.rolling_resistance_torque() / saturation, psi(0.1) / psi(25.0), 0.06 * psi(0.1), "a 2 cm/s roll gives psi(0.1) = 10 %% of the full resistance, not a constant")
	var negative: Dictionary = Rig.wheel_rig({"spin": -0.02 / 0.32})
	negative.world.step()
	check_near(negative.contact.rolling_resistance_torque(), -small.contact.rolling_resistance_torque(), 1.0e-6, "odd in the rolling direction (no jump through zero)")


func test_e4_no_resistance_without_contact() -> void:
	var rig: Dictionary = Rig.wheel_rig({"height": 0.2, "forward_speed": 5.0})
	var spin: float = -rig.wheel.angular_velocity.dot(rig.u)
	for i in 20:
		rig.world.step()
	check(not rig.contact.contact_active, "separated")
	check_near(rig.contact.rolling_resistance_torque(), 0.0, 0.0, "no resistance torque")
	check_near(-rig.wheel.angular_velocity.dot(rig.u), spin, 1.0e-9, "the spin of a flying wheel is not reduced")


func test_e5_a_stationary_wheel_feels_no_persistent_force() -> void:
	var rig: Dictionary = Rig.wheel_rig({})
	for i in 240:
		rig.world.step()
	var c = rig.contact
	check_near(c.rolling_resistance_torque(), 0.0, 0.0, "no resistance torque at rest")
	check_near(c.longitudinal_force(), 0.0, 1.0e-4, "no longitudinal force at rest")
	check_near(c.lateral_force(), 0.0, 1.0e-4, "no lateral force at rest")
	check(rig.wheel.linear_velocity.length() < 1.0e-5 and rig.wheel.angular_velocity.length() < 1.0e-5, "the wheel does not creep (v %s, w %s)" % [rig.wheel.linear_velocity.length(), rig.wheel.angular_velocity.length()])
	check_near(c.normal_load(), rig.mass * Rig.G, 0.002 * rig.mass * Rig.G, "while still carrying its weight")


func test_e6_free_rolling_deceleration_matches_the_hand_derivation_and_zero_coefficient_means_no_drag() -> void:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 3.0})
	for i in 120:
		rig.world.step()
	var k: float = GroundRig.axle_inertia_factor()
	var expected: float = C_RR * Rig.G * psi(15.0) / (1.0 + k) * 1.0
	var drop: float = 3.0 - rig.wheel.linear_velocity.dot(rig.e_x)
	check_near(drop, expected, 0.05 * expected, "speed lost in 1 s = c_rr g psi / (1 + k) (%s m/s)" % expected)
	var free: Dictionary = Rig.wheel_rig({"forward_speed": 3.0, "spec_overrides": {"rolling_resistance": 0.0}})
	for i in 120:
		free.world.step()
	check_near(free.wheel.linear_velocity.dot(free.e_x), 3.0, 1.0e-3, "c_rr = 0: a free rolling wheel keeps its speed (no hidden drag from the tyre or the contact)")
	check_near(free.contact.rolling_resistance_torque(), 0.0, 0.0, "and reports no moment")


func test_e7_resistance_is_a_separate_effect_outside_the_slip_ellipse() -> void:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 5.0, "spin_ratio": 1.0})
	for i in 60:
		rig.world.step()
	check(absf(rig.contact.rolling_resistance_torque()) > 0.5, "the moment exists")
	check(rig.contact.friction_utilization() < 0.02, "while the slip force stays tiny: resistance is not hidden in the traction budget")


# ---------------------------------------------------------------- F: lifecycle

func test_f1_tyre_behaviour_begins_when_valid_contact_is_established() -> void:
	var rig: Dictionary = Rig.wheel_rig({"height": 0.15, "forward_speed": 5.0, "spin_ratio": 0.5})
	var c = rig.contact
	var first_active: int = -1
	for i in 120:
		rig.world.step()
		if c.contact_active and first_active < 0:
			first_active = i
		if first_active < 0:
			check(c.normal_load() == 0.0 and c.longitudinal_force() == 0.0 and c.rolling_resistance_torque() == 0.0, "no load or force before contact (step %d)" % i)
	check(first_active > 0, "the contact becomes active while falling (step %d)" % first_active)
	check(c.normal_load() >= 0.0, "load is never negative")
	var seen_force: bool = false
	for i in 10:
		rig.world.step()
		seen_force = seen_force or absf(c.longitudinal_force()) > 1.0
	check(seen_force, "braking slip produces tyre force once the wheel is down")


func test_f2_losing_contact_discards_load_and_forces_and_recontact_restores_them() -> void:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spin_ratio": 0.5, "lateral_speed": 0.5})
	var c = rig.contact
	var wheel = rig.wheel
	for i in 4:
		rig.world.step()
	check(c.normal_load() > 100.0 and absf(c.longitudinal_force()) > 10.0 and absf(c.lateral_force()) > 1.0, "tyre acting before separation")
	wheel.position += Vector3(0.0, 0.2, 0.0)  # lift the wheel clear (test input)
	var omega: Vector3 = wheel.angular_velocity
	rig.world.step()
	check(not c.contact_active, "separated")
	check_near(c.normal_load(), 0.0, 0.0, "no stale normal load after separation")
	check_near(c.longitudinal_force(), 0.0, 0.0, "no stale longitudinal force")
	check_near(c.lateral_force(), 0.0, 0.0, "no stale lateral force")
	check_near(c.friction_utilization(), 0.0, 0.0, "no stale utilisation")
	check_near(c.rolling_resistance_torque(), 0.0, 0.0, "no stale rolling moment")
	check_vec3_near(c._tire_impulse_world, Vector3.ZERO, 0.0, "the carried (warm-start) tyre impulse is discarded")
	# Free flight of a spinning wheel with a small roll rate precesses gyroscopically (~1e-2 rad/s here): compare the axle spin, not the vector.
	check_near(-wheel.angular_velocity.dot(rig.u), -omega.dot(rig.u), 1.0e-3, "and the spin of the flying wheel is unchanged (no tyre action)")
	# Re-contact with the same slip: behaves like a fresh contact in the same state (cold start).
	wheel.position = Vector3(0.0, rig.radius, 0.0)
	wheel.linear_velocity = rig.e_x * 6.0 + rig.e_y * 0.5
	wheel.angular_velocity = -rig.u * (0.5 * 6.0 / rig.radius)
	rig.world.step()
	var fresh: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spin_ratio": 0.5, "lateral_speed": 0.5})
	fresh.world.step()
	check(c.contact_active and c.normal_load() > 100.0, "re-contact: active with load")
	check_near(c.longitudinal_force(), fresh.contact.longitudinal_force(), 0.02 * absf(fresh.contact.longitudinal_force()), "re-contact longitudinal force equals a fresh contact's")
	check_near(c.lateral_force(), fresh.contact.lateral_force(), 0.02 * absf(fresh.contact.lateral_force()), "re-contact lateral force equals a fresh contact's")


func test_f3_each_internal_fixed_step_of_one_advance_refreshes_the_tyre() -> void:
	var a: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spin_ratio": 0.6, "lateral_speed": 0.7})
	var b: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spin_ratio": 0.6, "lateral_speed": 0.7})
	var refreshes: int = a.contact.refresh_count
	var steps: int = a.world.advance(5.5 * DT)
	check_near(steps, 5.0, 0.0, "advance(5.5 dt) runs five fixed steps")
	check_near(a.contact.refresh_count - refreshes, 5.0, 0.0, "the contact geometry was refreshed once per internal step")
	for i in 5:
		b.world.step()
	check_vec3_near(a.wheel.linear_velocity, b.wheel.linear_velocity, 0.0, "one advance of five steps equals five single steps (velocity)")
	check_vec3_near(a.wheel.angular_velocity, b.wheel.angular_velocity, 0.0, "(angular velocity)")
	check_vec3_near(a.wheel.position, b.wheel.position, 0.0, "(position)")
	check_near(a.contact.longitudinal_force(), b.contact.longitudinal_force(), 0.0, "(tyre force of the last step)")
	# Slip really is re-evaluated each step: it is not the value of the first step.
	var first: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spin_ratio": 0.6})
	first.world.step()
	var later: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spin_ratio": 0.6})
	later.world.advance(12.5 * DT)
	check(absf(later.contact.current_slip().slip_x) < 0.5 * absf(first.contact.current_slip().slip_x), "slip after 12 internal steps has decayed (a once-per-frame tyre would not show this)")


func test_f4_wheel_translation_and_rotation_update_the_slip_every_step() -> void:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spin": 0.0})
	var previous_slip: float = absf(rig.contact.current_slip().slip_x)
	var previous_spin: float = 0.0
	for i in 20:
		rig.world.step()
		var s: Dictionary = rig.contact.current_slip()
		check(absf(s.slip_x) < previous_slip + 1.0e-9, "braking slip magnitude falls as the wheel spins up (step %d)" % i)
		check(s.spin > previous_spin, "spin rises under the sliding force (step %d)" % i)
		previous_slip = absf(s.slip_x)
		previous_spin = s.spin
	check(rig.wheel.position.dot(-Vector3.BACK) > 0.5, "and the wheel translated forward")


func test_f5_sustained_operation_stays_finite_and_inside_the_envelope() -> void:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 8.0, "spin_ratio": 0.8, "lateral_speed": 1.0, "spec_overrides": {"grip_lateral": 0.7}})
	var c = rig.contact
	var worst: float = 0.0
	var finite: bool = true
	for i in 1800:  # 15 s
		var phase: int = (i / 120) % 4
		if phase == 1:
			Rig.drive_torque(rig.wheel, rig.u, 60.0)
		elif phase == 2:
			Rig.drive_torque(rig.wheel, rig.u, -80.0)
		if i % 240 == 100:
			rig.wheel.linear_velocity += Vector3(1.5, 0.0, 0.0)
		if i % 400 == 399:
			rig.wheel.position += Vector3(0.0, 0.12, 0.0)  # a hop: contact loss and re-contact
		rig.world.step()
		var q: Array = [c.normal_load(), c.longitudinal_force(), c.lateral_force(), c.rolling_resistance_torque(), c.friction_utilization()]
		for value in q:
			finite = finite and is_finite(value)
		worst = maxf(worst, c.friction_utilization())
		finite = finite and rig.wheel.position.is_finite() and rig.wheel.linear_velocity.is_finite() and rig.wheel.angular_velocity.is_finite()
	check(finite, "every force, load and body state stayed finite over 1800 steps")
	check(worst <= 1.0 + 1.0e-5, "utilisation never exceeded 1 (max %s)" % worst)
	check(rig.world.numerical_fault == "", "no numerical fault was raised")


func test_f6_disabled_tyre_is_a_frictionless_normal_contact() -> void:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 5.0, "spin_ratio": 0.5, "lateral_speed": 1.0})
	rig.contact.tire_forces_enabled = false
	var v0: Vector3 = rig.wheel.linear_velocity
	for i in 30:
		rig.world.step()
	check_near(rig.contact.longitudinal_force(), 0.0, 0.0, "no longitudinal force")
	check_near(rig.contact.lateral_force(), 0.0, 0.0, "no lateral force")
	check_near(rig.contact.rolling_resistance_torque(), 0.0, 0.0, "no rolling resistance")
	check_near(rig.contact.normal_load(), rig.mass * Rig.G, 0.002 * rig.mass * Rig.G, "the normal contact still carries the weight")
	check_near(rig.wheel.linear_velocity.x, v0.x, 1.0e-6, "nothing (no generic friction either) changes the horizontal speed")
	check_near(rig.wheel.linear_velocity.z, v0.z, 1.0e-6, "in either direction")
