extends "res://Tests/sim_test_case.gd"
## M2.4.2 B (longitudinal), C (lateral), D (combined slip): the force law (TireModel.force, pure) and the tyre contact in the solver
## (TireGroundContact on a single prototype wheel, Tests/tire_rig.gd). Expected values come from hand derivations quoted in each test
## (the friction ellipse, the rolling relation F = T / (R (1 + k)) of a driven or braked free wheel, the locked-wheel force mu N).
## Prototype tyre: k_x 15, k_y 10, grip 1/1 on mu_s = 0.8 (so mu_x = mu_y = 0.8), v_ref 0.5 m/s, 120 Hz, 20 iterations.

const Rig = preload("res://Tests/tire_rig.gd")
const GroundRig = preload("res://Tests/ground_rig.gd")
const TireModel = preload("res://Simulation/Motorcycle/tire_model.gd")

const DT: float = 1.0 / 120.0
const MU: float = 0.8
const KX: float = 15.0
const KY: float = 10.0


func law(sx: float, sy: float, n: float = 200.0, mux: float = MU, muy: float = MU) -> Dictionary:
	return TireModel.force(sx, sy, n, mux, muy, KX, KY)


# ---------------------------------------------------------------- force law (pure)

func test_b1_law_small_slip_is_linear_with_the_hand_computed_reduction() -> void:
	# sigma_x = 0.01: rho = 15 * 0.01 / 0.8 = 0.1875, phi = 1 - rho/3 + rho^2/27 = 1 - 0.0625 + 0.0013021 = 0.9388, F = phi * 15 * 200 * 0.01.
	var f: Dictionary = law(0.01, 0.0)
	check_near(f.fx, 0.9388021 * 15.0 * 200.0 * 0.01, 1.0e-3, "longitudinal force at small slip")
	check_near(f.fy, 0.0, 0.0, "no lateral force for pure longitudinal slip")
	# The slope at zero slip is the stiffness k_x N.
	var tiny: Dictionary = law(1.0e-6, 0.0)
	check_near(tiny.fx / 1.0e-6, KX * 200.0, 1.0, "slope at the origin is k_x N")
	check_near(law(0.0, 0.0).fx, 0.0, 0.0, "force is zero at zero slip")


func test_b2_law_signs_driving_forward_braking_backward() -> void:
	check(law(0.05, 0.0).fx > 0.0, "driving slip gives forward force")
	check(law(-0.05, 0.0).fx < 0.0, "braking slip gives backward force")
	check_near(law(0.05, 0.0).fx, -law(-0.05, 0.0).fx, 1.0e-9, "odd in the slip")
	check(law(0.0, 0.05).fy > 0.0, "positive lateral slip (contact slides left) gives a force to the right")
	check(law(0.0, -0.05).fy < 0.0, "negative lateral slip gives a force to the left")
	check_near(law(0.0, 0.05).fy, -law(0.0, -0.05).fy, 1.0e-9, "odd in the lateral slip")


func test_b3_law_increases_then_saturates_at_mu_n() -> void:
	var previous: float = -1.0
	var saturated_at: float = 3.0 * MU / KX  # rho = 3 -> sigma = 0.16
	for i in 41:
		var sigma: float = i * 0.01
		var f: float = law(sigma, 0.0).fx
		check(f >= previous - 1.0e-9, "non-decreasing in slip at %s" % sigma)
		check(f <= MU * 200.0 + 1.0e-6, "bounded by mu N at %s" % sigma)
		previous = f
	check_near(law(saturated_at, 0.0).fx, MU * 200.0, 1.0e-4, "the force reaches mu N at rho = 3 (sigma = 0.16)")
	check_near(law(1.0, 0.0).fx, MU * 200.0, 1.0e-9, "and stays at mu N for larger slip")
	check_near(law(-1.0, 0.0).fx, -MU * 200.0, 1.0e-9, "braking saturates at -mu N")
	check_near(law(0.0, 3.0).fy, MU * 200.0, 1.0e-9, "lateral saturation at mu N")


func test_b4_law_scales_with_normal_load_and_vanishes_without() -> void:
	check_near(law(0.05, 0.02, 400.0).fx, 2.0 * law(0.05, 0.02, 200.0).fx, 1.0e-6, "force is proportional to the normal load")
	check_near(law(0.05, 0.02, 400.0).fy, 2.0 * law(0.05, 0.02, 200.0).fy, 1.0e-6, "lateral force too")
	check_near(law(0.5, 0.5, 0.0).fx, 0.0, 0.0, "no load, no force")
	check_near(law(0.5, 0.5, -5.0).fy, 0.0, 0.0, "negative load (never produced by a unilateral contact) gives no force")
	check_near(law(0.5, 0.5, 200.0, 0.0, 0.8).fx, 0.0, 0.0, "zero friction limit, no force")


func test_d1_law_stays_inside_the_friction_ellipse_everywhere() -> void:
	var worst: float = 0.0
	for mu_y in [0.8, 0.4]:
		for i in 41:
			for j in 41:
				var f: Dictionary = law(-2.0 + i * 0.1, -2.0 + j * 0.1, 250.0, MU, mu_y)
				var ux: float = f.fx / (MU * 250.0)
				var uy: float = f.fy / (mu_y * 250.0)
				worst = maxf(worst, ux * ux + uy * uy)
	check(worst <= 1.0 + 1.0e-9, "(Fx/(mu_x N))^2 + (Fy/(mu_y N))^2 <= 1 over the whole slip plane (max %s)" % worst)
	check(worst > 0.999, "and the envelope is actually reached (max %s)" % worst)


func test_d2_law_is_not_an_independent_per_axis_clamp() -> void:
	# Independent per-axis clamping of these demands would give (-mu N, -mu N), utilisation sqrt(2).
	var f: Dictionary = law(-2.0, -3.0)  # normalised demands k_x*2/mu = 37.5 and k_y*3/mu = 37.5: equal
	check(f.utilization <= 1.0 + 1.0e-9, "utilisation <= 1 for large combined slip")
	check_near(f.fx, -MU * 200.0 / sqrt(2.0), 1.0e-3 * 200.0, "equal normalised demand shares the budget: Fx = -mu N / sqrt(2)")
	check_near(f.fy, -MU * 200.0 / sqrt(2.0), 1.0e-3 * 200.0, "and Fy = -mu N / sqrt(2)")
	var ux: float = f.fx / (MU * 200.0)
	var uy: float = f.fy / (MU * 200.0)
	check_near(sqrt(ux * ux + uy * uy), 1.0, 1.0e-9, "the total force sits on the ellipse")


func test_d3_law_braking_reduces_lateral_capacity_and_lateral_demand_reduces_traction() -> void:
	var previous: float = INF
	for sx in [0.0, -0.05, -0.1, -0.3, -1.0]:
		var fy: float = law(sx, 0.1).fy
		check(fy < previous, "lateral force falls as braking slip grows (sigma_x %s: %s)" % [sx, fy])
		previous = fy
	previous = INF
	for sy in [0.0, 0.05, 0.1, 0.3, 1.0]:
		var fx: float = law(0.1, sy).fx
		check(fx < previous, "traction falls as lateral slip grows (sigma_y %s: %s)" % [sy, fx])
		previous = fx
	check(law(-1.0, 0.1).fy < 0.2 * law(0.0, 0.1).fy, "locked-wheel braking leaves only a small part of the lateral force")


func test_d4_law_anisotropic_limits_use_each_axis_own_scale() -> void:
	var f: Dictionary = TireModel.force(-1.0, -1.0, 200.0, 0.8, 0.4, KX, KY)
	# Normalised demands e_x = 15 * -1 / 0.8 = -18.75, e_y = 10 * -1 / 0.4 = -25, rho = 31.25 (saturated): F = (mu_x N e_x/rho, mu_y N e_y/rho).
	check_near(f.fx, -0.8 * 200.0 * 18.75 / 31.25, 1.0e-6, "anisotropic ellipse, longitudinal share")
	check_near(f.fy, -0.4 * 200.0 * 25.0 / 31.25, 1.0e-6, "anisotropic ellipse, lateral share")


# ---------------------------------------------------------------- solver level: longitudinal

func axle_k() -> float:
	return GroundRig.axle_inertia_factor()


func test_b5_driving_torque_gives_forward_force_and_matching_wheel_response() -> void:
	# Free wheel on the ground, drive torque T about the axle. Rolling relation (hand derivation): hub m a = F, axle I alpha = T - F R,
	# alpha = a / R  =>  F = T / (R (1 + k)),  k = I_axle / (m R^2) = 0.78125.
	var rig: Dictionary = Rig.wheel_rig({"spec_overrides": {"rolling_resistance": 0.0}})
	var wheel = rig.wheel
	var contact = rig.contact
	var torque: float = 20.0
	for i in 60:
		Rig.drive_torque(wheel, rig.u, torque)
		rig.world.step()
	var expected: float = torque / (rig.radius * (1.0 + axle_k()))
	check_near(contact.longitudinal_force(), expected, 0.03 * expected, "driving force = T / (R (1 + k)) (%s N)" % expected)
	check(contact.longitudinal_force() > 0.0 and contact.current_slip().slip_x > 0.0, "forward force and positive driving slip")
	check(-wheel.linear_velocity.dot(Vector3(0, 0, 1)) > 0.5 and wheel.linear_velocity.dot(rig.e_x) > 0.5, "the wheel accelerates forward")
	check(-wheel.angular_velocity.dot(rig.u) > 0.0, "and its spin is positive (forward rolling)")
	check_near(contact.lateral_force(), 0.0, 1.0e-3, "no lateral force")
	# Angular response of one step: I_axle * dOmega / dt = T - F * R (contact at the bottom, lever R).
	var spin_before: float = -wheel.angular_velocity.dot(rig.u)
	var f_before: float = contact.longitudinal_force()
	Rig.drive_torque(wheel, rig.u, torque)
	rig.world.step()
	var alpha: float = (-wheel.angular_velocity.dot(rig.u) - spin_before) / DT
	var i_axle: float = rig.mass * rig.radius * rig.radius * axle_k()
	check_near(alpha * i_axle, torque - contact.longitudinal_force() * rig.radius, 0.02 * torque, "axle torque balance of the step: I alpha = T - F R")
	check(contact.friction_utilization() < 0.5, "well inside the friction limit")


func test_b6_braking_torque_gives_backward_force() -> void:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 12.0, "spec_overrides": {"rolling_resistance": 0.0}})
	var wheel = rig.wheel
	var contact = rig.contact
	var torque: float = 20.0
	for i in 60:
		Rig.drive_torque(wheel, rig.u, -torque)
		rig.world.step()
	var expected: float = torque / (rig.radius * (1.0 + axle_k()))
	check_near(contact.longitudinal_force(), -expected, 0.03 * expected, "braking force = -T / (R (1 + k))")
	check(contact.current_slip().slip_x < 0.0, "braking slip is negative")
	check(wheel.linear_velocity.dot(rig.e_x) < 12.0 - 0.5, "the hub decelerates")
	check(-wheel.angular_velocity.dot(rig.u) < 12.0 / rig.radius, "and the spin falls below the rolling value")


func test_b7_locked_wheel_force_is_exactly_mu_n_against_the_motion() -> void:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 6.0, "spin": 0.0})
	rig.world.step()
	var contact = rig.contact
	var n: float = contact.normal_load()
	check_near(n, rig.mass * Rig.G, 0.002 * rig.mass * Rig.G, "normal load carries the weight (m g = %s N)" % (rig.mass * Rig.G))
	check_near(contact.longitudinal_force(), -MU * n, 1.0e-3 * MU * n, "a sliding wheel carries mu N backward")
	check_near(contact.friction_utilization(), 1.0, 1.0e-4, "on the ellipse")
	check_near(contact.lateral_force(), 0.0, 1.0e-2, "and no lateral force")


func test_b8_force_vanishes_with_slip_and_grows_with_it() -> void:
	var previous: float = -1.0
	var first: float = 0.0
	for i in 8:
		var delta: float = i * 0.0015  # spin 0 .. 1.05 % above rolling, forward 5 m/s
		var rig: Dictionary = Rig.wheel_rig({"forward_speed": 5.0, "spin_ratio": 1.0 + delta, "spec_overrides": {"rolling_resistance": 0.0}})
		rig.world.step()
		var f: float = rig.contact.longitudinal_force()
		if i == 0:
			first = f
		check(f >= previous - 1.0e-6, "force does not decrease as the driving slip grows (delta %s: %s N)" % [delta, f])
		check(f <= MU * rig.contact.normal_load() + 1.0e-3, "force bounded by mu N")
		previous = f
	check_near(first, 0.0, 0.5, "force at (almost) zero slip is about zero (%s N)" % first)
	check(previous > 5.0, "and clearly non-zero at one percent slip (%s N)" % previous)


func test_b9_forces_never_exceed_the_load_dependent_envelope() -> void:
	var worst: float = 0.0
	var count: int = 0
	for gscale in [0.5, 1.0, 2.0]:
		for spin in [0.0, 0.4, 1.0, 1.6, 4.0]:
			for lateral in [0.0, 1.5, -3.0]:
				var rig: Dictionary = Rig.wheel_rig({"gravity_scale": gscale, "forward_speed": 7.0, "spin_ratio": spin, "lateral_speed": lateral,
					"spec_overrides": {"grip_lateral": 0.6}})
				for i in 3:
					rig.world.step()
					var c = rig.contact
					var n: float = c.normal_load()
					count += 1
					worst = maxf(worst, c.friction_utilization())
					check(absf(c.longitudinal_force()) <= c.friction_coefficient_longitudinal() * n * (1.0 + 1.0e-4) + 1.0e-6, "|Fx| <= mu_x N (gscale %s spin %s lat %s)" % [gscale, spin, lateral])
					check(absf(c.lateral_force()) <= c.friction_coefficient_lateral() * n * (1.0 + 1.0e-4) + 1.0e-6, "|Fy| <= mu_y N")
	check(worst <= 1.0 + 1.0e-5, "utilisation <= 1 in all %d sampled steps (max %s)" % [count, worst])


func test_b10_no_tire_force_without_contact() -> void:
	var rig: Dictionary = Rig.wheel_rig({"height": 0.2, "forward_speed": 5.0, "spin_ratio": 0.5, "lateral_speed": 1.0})
	var wheel = rig.wheel
	var omega: Vector3 = wheel.angular_velocity
	var vx: float = wheel.linear_velocity.x
	for i in 10:
		rig.world.step()
	var c = rig.contact
	check(not c.contact_active, "wheel 0.2 m above the ground (activation distance 0.05 m): contact inactive")
	check_near(c.normal_load(), 0.0, 0.0, "no normal load")
	check_near(c.longitudinal_force(), 0.0, 0.0, "no longitudinal force")
	check_near(c.lateral_force(), 0.0, 0.0, "no lateral force")
	check_near(c.rolling_resistance_torque(), 0.0, 0.0, "no rolling resistance")
	check_vec3_near(wheel.angular_velocity, omega, 1.0e-9, "the wheel spin is untouched (only gravity acts)")
	check_near(wheel.linear_velocity.x, vx, 1.0e-9, "and so is the sideways speed")


func test_b11_impulses_obey_newton_and_the_ground_stays_put() -> void:
	# One step with slip: dp(wheel) = (lambda_n n + F_x e_x + F_y e_y) dt + m g dt, dL(about COM) = r x (contact impulse) + rolling impulse * u.
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 5.0, "spin_ratio": 0.7, "lateral_speed": 0.8})
	rig.world.step()  # settle the warm start
	var wheel = rig.wheel
	var c = rig.contact
	var p0: Vector3 = wheel.linear_momentum()
	var i0: Basis = wheel.inertia_world
	var w0: Vector3 = wheel.angular_velocity
	var x0: Vector3 = wheel.position
	rig.world.step()
	var impulse: Vector3 = (Vector3.UP * c.normal_load() + c.longitudinal_axis() * c.longitudinal_force() + c.lateral_axis() * c.lateral_force()) * DT
	var gravity: Vector3 = Vector3(0.0, -Rig.G, 0.0) * rig.mass * DT
	check_vec3_near(wheel.linear_momentum() - p0, impulse + gravity, 2.0e-3 * impulse.length(), "linear momentum change = contact impulse + gravity impulse")
	var lever: Vector3 = c._point - x0  # the start-of-step contact point and COM (the contact acts from the start-of-step pose)
	var angular: Vector3 = lever.cross(impulse) + rig.u * (c.rolling_resistance_torque() * DT)
	# Tolerance 4 %: the implicit-midpoint gyroscopic term of the spinning wheel (omega x I omega, measured 2.7 % of the impulse here) is
	# in the velocity change but is not a contact impulse.
	check_vec3_near(i0 * (wheel.angular_velocity - w0), angular, 0.04 * angular.length() + 1.0e-4, "angular impulse about the COM = r x P + rolling moment")
	check_vec3_near(rig.ground.linear_velocity, Vector3.ZERO, 0.0, "the static ground gains no velocity")
	check_vec3_near(rig.ground.angular_velocity, Vector3.ZERO, 0.0, "nor spin")
	check_vec3_near(rig.ground.position, Vector3.ZERO, 0.0, "nor moves")


# ---------------------------------------------------------------- solver level: lateral

func test_c1_lateral_slip_gives_a_restoring_force_for_both_signs() -> void:
	for sign_value in [1.0, -1.0]:
		var rig: Dictionary = Rig.wheel_rig({"forward_speed": 5.0, "lateral_speed": 0.5 * sign_value})
		var c = rig.contact
		var vy0: float = rig.wheel.linear_velocity.dot(rig.e_y)
		rig.world.step()
		check(c.lateral_force() * sign_value < 0.0, "force opposes the lateral motion (sign %s: %s N)" % [sign_value, c.lateral_force()])
		check(c.current_slip().slip_y * sign_value < 0.0, "lateral slip has the sign of the (negated) sliding direction")
		# Short horizon only: a free wheel has no roll stability, so after ~0.3 s it falls over and the lateral motion is no longer a
		# clean decay (physical, not a tyre effect).
		var previous: float = absf(vy0)
		var monotone: bool = true
		for i in 12:
			rig.world.step()
			var vy: float = absf(rig.wheel.linear_velocity.dot(rig.e_y))
			if vy > previous + 1.0e-6:
				monotone = false
			previous = vy
		check(monotone, "the sideways speed decays monotonically over 0.1 s (restoring, no overshoot)")
		check(previous < 0.8 * absf(vy0), "and has fallen clearly below its start (%s of %s)" % [previous, vy0])


func test_c2_small_lateral_slip_force_is_proportional_to_slip_and_load() -> void:
	var forces: Array = []
	for lateral in [0.1, 0.2]:
		var rig: Dictionary = Rig.wheel_rig({"forward_speed": 10.0, "lateral_speed": lateral})
		rig.world.step()
		forces.append(-rig.contact.lateral_force())
	check(forces[0] > 0.0, "positive restoring magnitude")
	check_near(forces[1] / forces[0], 2.0, 0.08, "twice the sliding speed, twice the force (linear region, implicit lag within 8 %%)")
	var heavy: Dictionary = Rig.wheel_rig({"forward_speed": 10.0, "lateral_speed": 0.1, "gravity_scale": 2.0})
	heavy.world.step()
	var ratio: float = -heavy.contact.lateral_force() / forces[0]
	check(ratio > 1.5 and ratio < 2.05, "twice the load gives clearly more force, at most twice (the implicit response lowers the ratio: %s)" % ratio)


func test_c3_large_lateral_slip_saturates_at_mu_n() -> void:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 5.0, "lateral_speed": 5.0})
	rig.world.step()
	var c = rig.contact
	check_near(c.lateral_force(), -MU * c.normal_load(), 2.0e-3 * MU * c.normal_load(), "45 degree slip: lateral force = -mu N")
	check_near(c.friction_utilization(), 1.0, 1.0e-3, "on the ellipse")
	check_near(c.longitudinal_force(), 0.0, 0.02 * MU * c.normal_load(), "no longitudinal share without longitudinal slip")


func test_c4_tilted_and_yawed_wheels_keep_the_lateral_force_in_the_ground_plane_along_the_projected_axle() -> void:
	var magnitudes: Array = []
	for yaw in [0.0, 0.6, -1.1]:
		var rig: Dictionary = Rig.wheel_rig({"yaw": yaw, "camber": 0.35, "forward_speed": 6.0, "lateral_speed": 1.0})
		rig.world.step()
		var c = rig.contact
		var f: Vector3 = c.tangential_force_world()
		var projected: Vector3 = (rig.u - Vector3.UP * rig.u.y).normalized()
		check_near(f.dot(Vector3.UP), 0.0, 1.0e-4, "tangential force lies in the ground plane (yaw %s)" % yaw)
		check(f.dot(projected) < 0.0, "and points against the sideways motion, along the projected axle")
		check(f.cross(projected).length() < 0.05 * f.length(), "parallel to the projected axle (cross %s of %s)" % [f.cross(projected).length(), f.length()])
		check_vec3_near(c.lateral_axis(), projected, 1.0e-6, "the contact's lateral axis is the projected axle")
		magnitudes.append(c.lateral_force())
	check_near(magnitudes[1], magnitudes[0], 0.03 * absf(magnitudes[0]), "the same slip in the wheel's axes gives the same force whatever the heading")
	check_near(magnitudes[2], magnitudes[0], 0.03 * absf(magnitudes[0]), "also for another heading")


# ---------------------------------------------------------------- solver level: combined

func test_d5_locked_wheel_with_sideways_motion_slides_against_the_velocity_vector() -> void:
	# Equal normalised stiffness (k_x = k_y), so the gross-sliding direction of the law is the slip direction (with k_x != k_y the
	# direction in the normalised space is weighted by the stiffnesses; documented in PHYSICS_CONVENTIONS.md).
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 8.0, "lateral_speed": 3.0, "spin": 0.0, "spec_overrides": {"lateral_stiffness": 15.0}})
	rig.world.step()
	var c = rig.contact
	var n: float = c.normal_load()
	var norm: float = sqrt(64.0 + 9.0)
	check_near(c.longitudinal_force(), -MU * n * 8.0 / norm, 0.04 * MU * n, "Fx = -mu N cos(theta)")
	check_near(c.lateral_force(), -MU * n * 3.0 / norm, 0.04 * MU * n, "Fy = -mu N sin(theta)")
	check_near(c.friction_utilization(), 1.0, 2.0e-3, "total tangential force on the ellipse")
	var total: float = sqrt(c.longitudinal_force() * c.longitudinal_force() + c.lateral_force() * c.lateral_force())
	check(total <= MU * n * (1.0 + 1.0e-4), "total force never exceeds mu N (%s of %s)" % [total, MU * n])


func test_d6_anisotropic_limits_with_combined_loading_stay_inside_the_ellipse() -> void:
	var rig: Dictionary = Rig.wheel_rig({"forward_speed": 8.0, "lateral_speed": 3.0, "spin": 0.0, "spec_overrides": {"grip_lateral": 0.5}})
	rig.world.step()
	var c = rig.contact
	var n: float = c.normal_load()
	var mux: float = 0.8
	var muy: float = 0.4
	# Saturated: force direction in the normalised space is the slip direction: e = (-15*8/0.8, -10*3/0.4) / norm -> share of each axis.
	var ex: float = -KX * 8.0 / 0.8
	var ey: float = -KY * 3.0 / 0.4
	var rho: float = sqrt(ex * ex + ey * ey)
	check_near(c.longitudinal_force(), mux * n * ex / rho, 0.05 * mux * n, "anisotropic ellipse: longitudinal share")
	check_near(c.lateral_force(), muy * n * ey / rho, 0.05 * muy * n, "anisotropic ellipse: lateral share")
	var ux: float = c.longitudinal_force() / (mux * n)
	var uy: float = c.lateral_force() / (muy * n)
	check(ux * ux + uy * uy <= 1.0 + 1.0e-4, "inside the ellipse")
	check(absf(c.longitudinal_force()) < 0.99 * mux * n and absf(c.lateral_force()) < 0.99 * muy * n, "neither axis is clamped to its full limit independently")


func test_d7_braking_reduces_lateral_force_and_lateral_demand_reduces_traction() -> void:
	var previous: float = INF
	for ratio in [1.0, 0.9, 0.7, 0.3, 0.0]:
		var rig: Dictionary = Rig.wheel_rig({"forward_speed": 8.0, "lateral_speed": 1.0, "spin_ratio": ratio})
		rig.world.step()
		var fy: float = absf(rig.contact.lateral_force())
		check(fy < previous, "lateral force falls with braking (spin ratio %s: %s N)" % [ratio, fy])
		previous = fy
	previous = INF
	for lateral in [0.0, 1.0, 2.0, 4.0]:
		var rig: Dictionary = Rig.wheel_rig({"forward_speed": 8.0, "lateral_speed": lateral, "spin_ratio": 0.7})
		rig.world.step()
		var fx: float = absf(rig.contact.longitudinal_force())
		check(fx < previous, "traction falls with lateral demand (lateral speed %s: %s N)" % [lateral, fx])
		previous = fx


func test_d8_pure_cases_separate_cleanly() -> void:
	var longitudinal: Dictionary = Rig.wheel_rig({"forward_speed": 8.0, "spin_ratio": 0.9})
	longitudinal.world.step()
	check_near(longitudinal.contact.lateral_force(), 0.0, 1.0e-3, "pure longitudinal loading: no lateral force")
	var lateral: Dictionary = Rig.wheel_rig({"forward_speed": 8.0, "lateral_speed": 1.0})
	lateral.world.step()
	check(absf(lateral.contact.longitudinal_force()) < 0.01 * MU * lateral.contact.normal_load(), "pure lateral loading: only a negligible longitudinal force from the wheel's own roll coupling (%s N)" % lateral.contact.longitudinal_force())
