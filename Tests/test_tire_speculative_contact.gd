extends "res://Tests/sim_test_case.gd"
## M2.4.2.1: the tyre must not act across an air gap.
##
## The generic normal row stays active while the wheel has a small POSITIVE gap and approaches the ground (speculative bias g/dt, M2.4.1).
## Its positive impulse is collision prevention, not physical load. These tests exercise exactly the boundary the M2.4.2 suite missed:
## an ACTIVE contact with 0 < gap <= activation distance and a positive speculative normal impulse (the earlier tests used a wheel
## clearly outside the activation distance, where the contact is inactive and discards everything).
##
## Single prototype wheel (Tests/tire_rig.gd), 120 Hz, default solver settings. First step from the set-up state, so the expectations are
## closed forms: gravity acts first (v_y = v_y0 - g dt), then the normal row of an upright wheel (contact point below the COM, no lever
## arm) gives  lambda_n = m (-(v_y0 - g dt) - gap/dt)  whenever that is positive.
##   A speculative approach: normal response active, no tyre force, no rolling resistance, normal_load() = 0, nothing leaks into the wheel.
##   B no tangential warm start across separation: established tyre state is cleared and not applied across the gap.
##   C actual contact is unchanged (load, forces, rolling resistance, penetration, the touch boundary) and re-contact starts cold.
##   H regression sensitivity: an in-suite mutant with the old behaviour (any positive normal impulse is load) fails the probes.

const Rig = preload("res://Tests/tire_rig.gd")
const TireGroundContact = preload("res://Simulation/Motorcycle/tire_ground_contact.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")

const DT: float = 1.0 / 120.0
const G: float = 9.81
## Positive gap inside the 5 cm activation distance and far above the 0.1 mm touch tolerance (the geometry is float32: 1e-7 resolution).
const GAP: float = 0.002


## The old (M2.4.2) behaviour: any active contact counts as touching, so a speculative normal impulse is treated as tyre load.
class ActsOnSpeculativeLoad extends "res://Simulation/Motorcycle/tire_ground_contact.gd":
	func _update_rows(dt: float, config) -> void:
		super._update_rows(dt, config)
		_touching = true


## Wheel with the tyre fully loaded by slip (forward 6 m/s, half the rolling spin, 0.5 m/s sideways), `gap` above the plane and a
## vertical velocity `vy`. Everything else as Rig.wheel_rig.
func approach_rig(front: bool, gap: float, vy: float, script = null) -> Dictionary:
	var opts: Dictionary = {"front": front, "height": gap, "forward_speed": 6.0, "spin_ratio": 0.5, "lateral_speed": 0.5}
	if script != null:
		opts["contact_script"] = script
	var rig: Dictionary = Rig.wheel_rig(opts)
	rig.wheel.linear_velocity += Vector3(0.0, vy, 0.0)
	return rig


func spin_of(rig: Dictionary) -> float:
	return -rig.wheel.angular_velocity.dot(rig.u)


# ---------------------------------------------------------------- A speculative approach

func test_a1_speculative_contact_keeps_the_normal_response_but_no_tyre_action() -> void:
	for front in [false, true]:
		var label: String = "front" if front else "rear"
		var rig: Dictionary = approach_rig(front, GAP, -2.0)
		var c = rig.contact
		var wheel = rig.wheel
		check(c.signed_gap() > WheelGroundContact.TOUCH_TOLERANCE * 10.0 and c.signed_gap() < c.activation_distance, "%s: positive gap inside the activation distance (%s m)" % [label, c.signed_gap()])
		var forward: float = wheel.linear_velocity.dot(rig.e_x)
		var sideways: float = wheel.linear_velocity.dot(rig.e_y)
		var spin: float = spin_of(rig)
		rig.world.step()
		var expected: float = rig.mass * (-(-2.0 - G * DT) - GAP / DT)
		check(c.contact_active, "%s: the speculative contact is active" % label)
		check(c.signed_gap() > -WheelGroundContact.TOUCH_TOLERANCE, "%s: no penetration after the step" % label)
		check(c.normal_impulse() > 0.0, "%s: a POSITIVE speculative normal impulse exists while the geometric gap is positive (%s N s)" % [label, c.normal_impulse()])
		check_near(c.normal_impulse(), expected, 1.0e-3 * expected, "%s: it equals m (v_down - gap/dt): the generic speculative response" % label)
		check_near(c.speculative_normal_impulse(), c.normal_impulse(), 0.0, "%s: reported as the speculative response" % label)
		check_near(wheel.linear_velocity.y, -GAP / DT, 1.0e-3, "%s: the wheel is braked to close exactly the remaining gap (no tunnelling, no extra bounce)" % label)
		check(not c.is_touching(), "%s: not touching" % label)
		check_near(c.normal_load(), 0.0, 0.0, "%s: normal_load() is zero: the speculative impulse is not physical tyre load" % label)
		check_near(c.longitudinal_force(), 0.0, 0.0, "%s: no longitudinal tyre force" % label)
		check_near(c.lateral_force(), 0.0, 0.0, "%s: no lateral tyre force" % label)
		check_vec3_near(c.tangential_force_world(), Vector3.ZERO, 0.0, "%s: no tangential force vector" % label)
		check_near(c.rolling_resistance_torque(), 0.0, 0.0, "%s: no rolling resistance" % label)
		check_near(c.rolling_resistance_force(), 0.0, 0.0, "%s: no equivalent rolling-resistance force" % label)
		check_near(c.friction_utilization(), 0.0, 0.0, "%s: no utilisation of the friction ellipse" % label)
		check_near(c.longitudinal_force_limit(), 0.0, 0.0, "%s: no longitudinal force limit without physical load" % label)
		check_near(c.lateral_force_limit(), 0.0, 0.0, "%s: no lateral force limit without physical load" % label)
		check_near(c.rows[c.ROW_TANGENT_1].accumulated_impulse, 0.0, 0.0, "%s: tangent row 1 holds no impulse" % label)
		check_near(c.rows[c.ROW_TANGENT_2].accumulated_impulse, 0.0, 0.0, "%s: tangent row 2 holds no impulse" % label)
		# Nothing reached the wheel through the tangential channel: the horizontal motion and the spin are exactly those of free flight.
		check_near(wheel.linear_velocity.dot(rig.e_x), forward, 1.0e-5, "%s: forward velocity unchanged (no traction across the gap)" % label)
		check_near(wheel.linear_velocity.dot(rig.e_y), sideways, 1.0e-5, "%s: lateral velocity unchanged" % label)
		check_near(spin_of(rig), spin, 1.0e-3, "%s: spin unchanged (no slip force, no rolling resistance)" % label)


func test_a2_fast_approach_is_still_stopped_and_stays_free_of_tyre_force() -> void:
	# 20 m/s towards the plane from 4 cm: only the speculative row can stop it this step, and it must (collision prevention preserved).
	var rig: Dictionary = approach_rig(false, 0.04, -20.0)
	var c = rig.contact
	rig.world.step()
	check(c.contact_active and c.normal_impulse() > 0.0, "active with a large speculative impulse")
	check_near(rig.wheel.linear_velocity.y, -0.04 / DT, 1.0e-3, "stopped exactly at the plane (v = -gap/dt)")
	check(c.signed_gap() > -WheelGroundContact.TOUCH_TOLERANCE, "no tunnelling")
	check_near(c.normal_load(), 0.0, 0.0, "no physical load")
	check_near(c.longitudinal_force(), 0.0, 0.0, "no longitudinal force")
	check_near(c.lateral_force(), 0.0, 0.0, "no lateral force")
	check_near(c.rolling_resistance_torque(), 0.0, 0.0, "no rolling resistance")


func test_a3_a_slow_approach_has_no_speculative_impulse_and_no_tyre_action_either() -> void:
	# Gap 2 mm, falling at 0.1 m/s (slower than gap/dt = 0.24 m/s): the row stays inactive; still no tyre force.
	var rig: Dictionary = approach_rig(false, GAP, -0.1)
	var c = rig.contact
	rig.world.step()
	check(c.contact_active, "inside the activation distance")
	check_near(c.normal_impulse(), 0.0, 0.0, "no impulse needed this step")
	check_near(c.normal_load(), 0.0, 0.0, "no load")
	check_near(c.longitudinal_force(), 0.0, 0.0, "no longitudinal force")
	check_near(c.lateral_force(), 0.0, 0.0, "no lateral force")
	check_near(c.rolling_resistance_torque(), 0.0, 0.0, "no rolling resistance")


# ---------------------------------------------------------------- B no tangential warm start across separation

func test_b1_established_tyre_state_is_neither_warm_started_nor_applied_across_a_gap() -> void:
	for front in [false, true]:
		var label: String = "front" if front else "rear"
		var rig: Dictionary = approach_rig(front, 0.0, 0.0)
		var c = rig.contact
		var wheel = rig.wheel
		for i in 4:
			rig.world.step()
		check(c.is_touching() and c.normal_load() > 100.0, "%s: actual contact with load before the interval" % label)
		check(absf(c.longitudinal_force()) > 10.0 and absf(c.lateral_force()) > 1.0 and c.rolling_resistance_torque() > 0.0, "%s: nonzero tangential response and rolling resistance accumulated" % label)
		check(c._tire_impulse_world.length() > 0.1 and c._rolling_angular_impulse > 0.0, "%s: carried tyre state exists (what a warm start would reuse)" % label)
		# Move into a positive gap with the speculative contact still active and approaching (test input).
		wheel.position += Vector3(0.0, GAP, 0.0)
		wheel.linear_velocity.y = -2.0
		var forward: float = wheel.linear_velocity.dot(rig.e_x)
		var sideways: float = wheel.linear_velocity.dot(rig.e_y)
		var spin: float = spin_of(rig)
		rig.world.step()
		check(c.contact_active and c.normal_impulse() > 0.0 and c.signed_gap() > -WheelGroundContact.TOUCH_TOLERANCE, "%s: speculative contact still active with a positive normal impulse" % label)
		check(not c.is_touching(), "%s: separated" % label)
		check_vec3_near(c._tire_impulse_world, Vector3.ZERO, 0.0, "%s: the carried tangential impulse is cleared, not warm started" % label)
		check_near(c._rolling_angular_impulse, 0.0, 0.0, "%s: the rolling-resistance state is cleared" % label)
		check_near(c.rolling_resistance_torque(), 0.0, 0.0, "%s: no rolling resistance" % label)
		check_near(c.longitudinal_force(), 0.0, 0.0, "%s: no longitudinal force" % label)
		check_near(c.lateral_force(), 0.0, 0.0, "%s: no lateral force" % label)
		check_near(c.normal_load(), 0.0, 0.0, "%s: the physical normal-load diagnostic is zero" % label)
		check_near(wheel.linear_velocity.dot(rig.e_x), forward, 1.0e-5, "%s: the previous tangential impulse was not applied (forward velocity)" % label)
		check_near(wheel.linear_velocity.dot(rig.e_y), sideways, 1.0e-5, "%s: (lateral velocity)" % label)
		check_near(spin_of(rig), spin, 1.0e-3, "%s: (spin)" % label)


func test_b2_state_stays_clear_over_a_longer_separated_speculative_interval() -> void:
	# A wheel kept just above the plane by a test input for several steps, each time approaching: zero tyre response every step.
	var rig: Dictionary = approach_rig(false, 0.0, 0.0)
	var c = rig.contact
	for i in 4:
		rig.world.step()
	var worst: float = 0.0
	var speculative_steps: int = 0
	for i in 6:
		rig.wheel.position.y = rig.radius + GAP  # test input: held at a 2 mm gap
		rig.wheel.linear_velocity.y = -2.0
		rig.world.step()
		if c.contact_active and c.normal_impulse() > 0.0:
			speculative_steps += 1
		worst = maxf(worst, absf(c.longitudinal_force()) + absf(c.lateral_force()) + absf(c.rolling_resistance_torque()) + c.normal_load() + c._tire_impulse_world.length())
	check(speculative_steps == 6, "every step had an active speculative contact with positive normal impulse (%d of 6)" % speculative_steps)
	check_near(worst, 0.0, 0.0, "no tyre force, rolling resistance, load or carried state in any of them")


# ---------------------------------------------------------------- C actual contact and re-contact

func test_c1_actual_contact_is_unchanged() -> void:
	var rig: Dictionary = approach_rig(false, 0.0, 0.0)
	var c = rig.contact
	var v0: float = rig.wheel.linear_velocity.dot(rig.e_x)
	for i in 4:
		rig.world.step()
	check(c.contact_active and c.is_touching(), "touching")
	check_near(c.speculative_normal_impulse(), 0.0, 0.0, "the normal impulse of a touching wheel is load, not a speculative response")
	check_near(c.normal_load(), rig.mass * G, 0.02 * rig.mass * G, "the physical load carries the weight")
	check(absf(c.longitudinal_force()) > 10.0 and absf(c.lateral_force()) > 1.0, "slip produces longitudinal and lateral tyre forces")
	check(c.longitudinal_force() < 0.0 and c.lateral_force() < 0.0, "with the usual signs: braking slip pushes backward, sideways motion is opposed")
	check(c.rolling_resistance_torque() > 0.0, "rolling resistance acts")
	check(c.friction_utilization() > 0.0 and c.friction_utilization() <= 1.0 + 1.0e-6, "utilisation inside the ellipse")
	check(rig.wheel.linear_velocity.dot(rig.e_x) < v0, "the wheel is slowed by the tyre")


func test_c2_penetration_and_the_touch_boundary() -> void:
	# Penetrating wheels are geometrically in contact; the boundary is the 0.1 mm touch tolerance (documented in WheelGroundContact).
	var expect: Dictionary = {-0.001: true, 0.00005: true, 0.0002: false, 0.001: false}
	for gap in expect:
		var rig: Dictionary = approach_rig(false, gap, -1.0)
		var c = rig.contact
		rig.world.step()
		var touching: bool = expect[gap]
		check(c.contact_active and c.normal_impulse() > 0.0, "gap %s m: active contact with a positive normal impulse" % gap)
		check(c.is_touching() == touching, "gap %s m: touching = %s" % [gap, touching])
		if touching:
			check(c.normal_load() > 0.0 and absf(c.longitudinal_force()) > 1.0 and absf(c.lateral_force()) > 0.1 and c.rolling_resistance_torque() > 0.0, "gap %s m: the tyre acts" % gap)
		else:
			check(c.normal_load() == 0.0 and c.longitudinal_force() == 0.0 and c.lateral_force() == 0.0 and c.rolling_resistance_torque() == 0.0, "gap %s m: the tyre does not act" % gap)


func test_c3_recontact_after_a_speculative_interval_resumes_without_stale_impulses() -> void:
	var rig: Dictionary = approach_rig(false, 0.0, 0.0)
	var c = rig.contact
	var wheel = rig.wheel
	for i in 4:
		rig.world.step()
	wheel.position += Vector3(0.0, GAP, 0.0)
	wheel.linear_velocity.y = -2.0
	rig.world.step()  # speculative interval: no tyre action
	check(not c.is_touching() and c.longitudinal_force() == 0.0, "the interval was speculative")
	# Restore actual contact in the state of a fresh contact (as in test_tire_rolling_lifecycle f2) and compare with it.
	wheel.position = Vector3(0.0, rig.radius, 0.0)
	wheel.linear_velocity = rig.e_x * 6.0 + rig.e_y * 0.5
	wheel.angular_velocity = -rig.u * (0.5 * 6.0 / rig.radius)
	rig.world.step()
	var fresh: Dictionary = approach_rig(false, 0.0, 0.0)
	fresh.world.step()
	check(c.is_touching() and c.normal_load() > 100.0, "re-contact: touching with load")
	# The GENERIC normal row legitimately warm starts from the speculative impulse (M2.4.1 behaviour, unchanged), so the load differs from
	# the fresh contact's by about 1 % (measured 145.5 N against 147.2 N). The tyre state is cold: compare the force per unit load (2 %).
	var n: float = c.normal_load()
	var n_fresh: float = fresh.contact.normal_load()
	check_near(n, n_fresh, 0.03 * n_fresh, "the load differs from a fresh contact's only by the generic normal warm start")
	check_near(c.longitudinal_force() / n, fresh.contact.longitudinal_force() / n_fresh, 0.02 * absf(fresh.contact.longitudinal_force() / n_fresh), "longitudinal force per load equals a fresh contact's")
	check_near(c.lateral_force() / n, fresh.contact.lateral_force() / n_fresh, 0.02 * absf(fresh.contact.lateral_force() / n_fresh), "lateral force per load equals a fresh contact's")
	check_near(c.rolling_resistance_torque() / n, fresh.contact.rolling_resistance_torque() / n_fresh, 0.02 * fresh.contact.rolling_resistance_torque() / n_fresh, "rolling resistance per load equals a fresh contact's")
	check_vec3_near(c._tire_impulse_world, c._e_x * c.rows[c.ROW_TANGENT_1].accumulated_impulse + c._e_y * c.rows[c.ROW_TANGENT_2].accumulated_impulse, 1.0e-6, "the carried tyre impulse is only what this step produced")


func test_c4_natural_landing_resumes_the_tyre_on_the_step_after_touchdown() -> void:
	# A wheel falling onto the plane (no manual state reset): no tyre action while it is in the air or speculative, tyre response after touchdown.
	var rig: Dictionary = approach_rig(false, 0.03, 0.0)
	var c = rig.contact
	var acted_in_air: bool = false
	var touched: bool = false
	var acted_after: bool = false
	for i in 40:
		rig.world.step()
		var acting: bool = absf(c.longitudinal_force()) + absf(c.lateral_force()) + c.rolling_resistance_torque() > 0.0
		if c.is_touching():
			touched = true
			acted_after = acted_after or acting
		else:
			acted_in_air = acted_in_air or acting
	check(not acted_in_air, "no tyre force or rolling resistance on any step without geometric contact (including the speculative landing step)")
	check(touched and acted_after, "the tyre acts normally once the wheel touches")


# ---------------------------------------------------------------- H regression sensitivity

func probe_no_tyre_action_during_speculative_approach(script) -> bool:
	var rig: Dictionary = approach_rig(false, GAP, -2.0, script)
	var c = rig.contact
	rig.world.step()
	return c.contact_active and c.normal_impulse() > 0.0 and c.normal_load() == 0.0 and c.longitudinal_force() == 0.0 \
		and c.lateral_force() == 0.0 and c.rolling_resistance_torque() == 0.0


func probe_no_warm_start_across_separation(script) -> bool:
	var rig: Dictionary = approach_rig(false, 0.0, 0.0, script)
	var c = rig.contact
	for i in 4:
		rig.world.step()
	rig.wheel.position += Vector3(0.0, GAP, 0.0)
	rig.wheel.linear_velocity.y = -2.0
	var forward: float = rig.wheel.linear_velocity.dot(rig.e_x)
	rig.world.step()
	return c.normal_impulse() > 0.0 and c._tire_impulse_world == Vector3.ZERO and c._rolling_angular_impulse == 0.0 \
		and absf(rig.wheel.linear_velocity.dot(rig.e_x) - forward) < 1.0e-5


func test_h1_the_probes_accept_the_real_contact() -> void:
	check(probe_no_tyre_action_during_speculative_approach(TireGroundContact), "real contact passes the speculative-approach probe")
	check(probe_no_warm_start_across_separation(TireGroundContact), "real contact passes the no-warm-start probe")


func test_h2_the_probes_reject_the_old_behaviour() -> void:
	check(not probe_no_tyre_action_during_speculative_approach(ActsOnSpeculativeLoad), "old behaviour (speculative impulse treated as load) is rejected by the approach probe")
	check(not probe_no_warm_start_across_separation(ActsOnSpeculativeLoad), "old behaviour is rejected by the warm-start probe")
