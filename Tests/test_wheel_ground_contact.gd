extends "res://Tests/sim_test_case.gd"
## M2.4.1 wheel-ground contact PHYSICS and fixed-step LIFECYCLE with the generic ContactConstraint doing the solving.
## Every scenario runs for both the front (R 0.31, 12 kg) and the rear (R 0.32, 15 kg) wheel.
## Tolerances (justified inline): float32 values of order 1 use 1e-5; a rest-state support impulse is exact to the solver's
## convergence (the normal row of one contact is solved exactly per iteration), so 1e-3 relative is generous; sliding-to-rolling
## compares a discrete 120 Hz solution with the continuous formula, so 1 % (an O(dt) effect: the transition falls inside one step).
##
## Roll stability is NOT tested here and is never faked: a single wheel on one point contact is only exercised for short durations
## (the roll instability of an upright disc grows as exp(t sqrt(g/R)), ~1e-7 rad of float32 noise stays below 1e-4 rad for 1 s).
## Two narrow point contacts do not give a complete tyre-width or rider-supported roll-stability model.

const Rig = preload("res://Tests/ground_rig.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")
const ContactConstraint = preload("res://Simulation/Solver/contact_constraint.gd")

const DT: float = 1.0 / 120.0
const G: float = 9.81
const TOL: float = 1.0e-5


func each_wheel(callable: Callable) -> void:
	for front in [true, false]:
		callable.call(front, "front" if front else "rear")


# --- Resting support and normal-impulse sign -------------------------------------------------------------------------------

func test_wheel_rests_on_the_plane_and_the_normal_force_balances_its_weight() -> void:
	each_wheel(func(front, label):
		var rig: Dictionary = Rig.wheel_rig(front, 0.8)
		var min_impulse: float = INF
		for i in 240:
			rig.world.step()
			min_impulse = minf(min_impulse, rig.contact.normal_impulse())
		var weight_impulse: float = rig.mass * G * DT
		check_near(rig.contact.normal_impulse(), weight_impulse, weight_impulse * 1.0e-3, "%s: per-step normal impulse = m g dt (force balances the weight)" % label)
		check_near(rig.wheel.position.y, rig.radius, 1.0e-4, "%s: the wheel stays at height R (no sinking)" % label)
		check(min_impulse >= 0.0, "%s: the normal impulse is never negative (min %s)" % [label, min_impulse])
		check_near(rig.wheel.linear_velocity.length(), 0.0, 1.0e-3, "%s: the wheel is at rest" % label)
		check(rig.contact.contact_state() == WheelGroundContact.STATE_TOUCHING, "%s: state touching" % label)
		check(rig.contact.contact_active, "%s: contact active" % label)
		check(rig.ground.linear_velocity == Vector3.ZERO and rig.ground.position == Vector3.ZERO, "%s: the ground neither moves nor acquires velocity" % label))


func test_normal_impulse_accelerates_the_wheel_to_a_halt_and_acts_at_the_contact_point() -> void:
	# A wheel dropped from a height h lands with v = sqrt(2 g h) (approx.). The impulse must remove the whole downward momentum
	# m v within the landing step(s) and apply no angular impulse for a contact directly below the COM.
	each_wheel(func(front, label):
		var rig: Dictionary = Rig.wheel_rig(front, 0.8, 0.0)  # placeholder, repositioned below
		rig.wheel.position = Vector3(0, rig.radius + 0.01, 0)
		rig.wheel.linear_velocity = Vector3(0, -3.0, 0)
		var total_impulse: float = 0.0
		for i in 12:
			rig.world.step()
			total_impulse += rig.contact.normal_impulse()
		# Momentum balance in y over the 12 steps: sum(lambda) - m g t = m (v_final - v_initial), v_final ~ 0, v_initial = -3.
		var expected: float = rig.mass * (0.0 - (-3.0)) + rig.mass * G * 12.0 * DT
		check_near(total_impulse, expected, expected * 0.02, "%s: total normal impulse = momentum change plus weight impulse" % label)
		check_near(rig.wheel.angular_velocity.length(), 0.0, 1.0e-3, "%s: a contact below the COM exerts no torque" % label)
		check(rig.wheel.linear_velocity.y > -0.05, "%s: the wheel does not pass through the ground" % label))


func test_penetration_is_corrected_through_the_split_channel() -> void:
	# Start 20 mm into the ground, at rest, zero gravity so only the contact acts. Physical row: vn >= 0 (no approach), so the
	# physical velocity gets NO separating component; the split pass moves the pose by beta*|g| per step: g1 = (1 - beta) g0.
	each_wheel(func(front, label):
		var rig: Dictionary = Rig.wheel_rig(front, 0.8, 0.0, Vector3.ZERO)
		rig.wheel.position = Vector3(0, rig.radius - 0.02, 0)
		var beta: float = rig.world.config.baumgarte_factor
		rig.world.step()
		check_near(rig.contact.signed_gap(), -0.02 * (1.0 - beta), 1.0e-5, "%s: one step removes beta of the penetration (split correction)" % label)
		check_near(rig.wheel.linear_velocity.y, 0.0, 1.0e-4, "%s: no separating velocity was added (the split channel moves the pose only)" % label)
		check_near(rig.contact.normal_impulse(), 0.0, 1.0e-4, "%s: the physical normal impulse stays zero: nothing approaches" % label)
		for i in 60:
			rig.world.step()
		check(absf(rig.contact.signed_gap()) < 1.0e-4, "%s: the penetration decays to zero (gap %s)" % [label, rig.contact.signed_gap()])
		check_near(rig.wheel.linear_velocity.y, 0.0, 1.0e-3, "%s: the wheel never acquired a bounce velocity" % label))


func test_no_attractive_normal_impulse_when_separated() -> void:
	each_wheel(func(front, label):
		var rig: Dictionary = Rig.wheel_rig(front, 0.8, 0.0)
		rig.wheel.position = Vector3(0, rig.radius + 0.5, 0)
		rig.wheel.linear_velocity = Vector3(0, 2.0, 0)
		var worst: float = 0.0
		var min_vy: float = INF
		for i in 60:
			rig.world.step()
			worst = maxf(worst, absf(rig.contact.normal_impulse()) + rig.contact.friction_impulse_magnitude())
			min_vy = minf(min_vy, rig.wheel.linear_velocity.y)
		check(worst == 0.0, "%s: a separated, receding wheel receives no contact impulse at all (%s)" % [label, worst])
		# Free flight: v_y(t) = 2 - g t exactly (semi-implicit Euler steps of g dt); no force at a distance.
		check_near(rig.wheel.linear_velocity.y, 2.0 - G * 60.0 * DT, 1.0e-4, "%s: only gravity acts while separated" % label))


# --- Friction -----------------------------------------------------------------------------------------------------------

func test_friction_decelerates_a_sliding_wheel_until_it_rolls_at_the_analytic_speed() -> void:
	# Sliding wheel, forward speed s0 along -Z, no spin. Linear: m ds/dt = -mu m g. Angular: I dw/dt = mu m g R, rolling when
	# w R = s. With k = I/(m R^2): spin-up w R = (mu g / k) t, so t* = s0 / (mu g (1 + 1/k)) and s_final = s0 / (1 + k).
	var mu: float = 0.8
	var s0: float = 4.0
	each_wheel(func(front, label):
		var rig: Dictionary = Rig.wheel_rig(front, mu)
		rig.wheel.linear_velocity = Vector3(0, 0, -s0)
		var k: float = Rig.axle_inertia_factor()
		var worst_excess: float = 0.0
		var first_step_checked: bool = false
		for i in 90:
			rig.world.step()
			worst_excess = maxf(worst_excess, rig.contact.friction_impulse_magnitude() - rig.contact.friction_limit())
			if not first_step_checked:
				first_step_checked = true
				# During the first step the contact slides, so the friction impulse sits on the Coulomb boundary mu*lambda_n,
				# and it acts at the contact point below the COM: dw_x = -mu*lambda_n*R / I_axle.
				var lambda_n: float = rig.contact.normal_impulse()
				check_near(rig.contact.friction_impulse_magnitude(), mu * lambda_n, mu * lambda_n * 1.0e-3, "%s: sliding friction is on the Coulomb boundary" % label)
				var expected_wx: float = -mu * lambda_n * rig.radius / (k * rig.mass * rig.radius * rig.radius)
				check_near(rig.wheel.angular_velocity.x, expected_wx, absf(expected_wx) * 1.0e-3, "%s: friction at the contact point spins the wheel (angular response)" % label)
				check_near(rig.wheel.linear_velocity.z, -s0 + mu * lambda_n / rig.mass, 1.0e-4, "%s: friction opposes the sliding velocity" % label)
		var final_speed: float = -rig.wheel.linear_velocity.z
		check_near(final_speed, s0 / (1.0 + k), s0 / (1.0 + k) * 0.01, "%s: final rolling speed = s0 / (1 + k)" % label)
		check_near(rig.wheel.angular_velocity.x, -final_speed / rig.radius, 0.02 * final_speed / rig.radius, "%s: it ends in pure rolling (w = -v/R about +X)" % label)
		check(worst_excess <= 1.0e-9, "%s: the friction impulse never exceeded mu * lambda_n (excess %s)" % [label, worst_excess])
		check_near(rig.contact.tangential_velocity().length(), 0.0, 0.01, "%s: the contact point no longer slips" % label))


func test_the_coulomb_coefficient_sets_the_sliding_deceleration() -> void:
	# Sensitivity: the speed after 0.1 s of sliding is s0 - mu g t (t < t*): distinguishes mu = 0.4 from 0.8 and from a bound that
	# is 25 % too large.
	for mu in [0.4, 0.8]:
		var rig: Dictionary = Rig.wheel_rig(false, mu)
		rig.wheel.linear_velocity = Vector3(0, 0, -6.0)
		for i in 12:
			rig.world.step()
		check_near(-rig.wheel.linear_velocity.z, 6.0 - mu * G * 12.0 * DT, 0.02, "mu=%s: s = s0 - mu g t while sliding" % mu)


func test_zero_friction_produces_no_tangential_impulse() -> void:
	each_wheel(func(front, label):
		var rig: Dictionary = Rig.wheel_rig(front, 0.0)
		rig.wheel.linear_velocity = Vector3(0.7, 0, -4.0)
		var worst: float = 0.0
		for i in 120:
			rig.world.step()
			worst = maxf(worst, rig.contact.friction_impulse_magnitude())
		check(worst == 0.0, "%s: mu = 0 gives exactly zero friction impulse" % label)
		check_vec3_near(rig.wheel.linear_velocity, Vector3(0.7, 0, -4.0), 1.0e-4, "%s: the tangential velocity is untouched" % label)
		check_vec3_near(rig.wheel.angular_velocity, Vector3.ZERO, 1.0e-4, "%s: no spin is generated" % label))


func test_static_friction_holds_a_wheel_pushed_below_the_coulomb_limit() -> void:
	# A horizontal force F < mu m g applied at the COM cannot move a non-rolling wheel... the wheel does roll, so use a locked wheel:
	# block the spin through a huge axle inertia, then F < mu N must be balanced by friction (sticking).
	var rig: Dictionary = Rig.wheel_rig(false, 0.8)
	rig.wheel.set_mass_properties(SimMassProperties.new(15.0, Basis.from_scale(Vector3(1.0e6, 1.0e6, 1.0e6))))
	var force: float = 0.5 * 0.8 * 15.0 * G
	for i in 120:
		rig.wheel.apply_force(Vector3(0, 0, force))
		rig.world.step()
	check(absf(rig.wheel.linear_velocity.z) < 0.01, "a force of half the Coulomb limit does not move the wheel (v %s)" % rig.wheel.linear_velocity.z)
	check_near(rig.contact.friction_impulse_magnitude(), force * DT, force * DT * 0.02, "friction balances the applied force (sticking)")
	var big: Dictionary = Rig.wheel_rig(false, 0.8)
	big.wheel.set_mass_properties(SimMassProperties.new(15.0, Basis.from_scale(Vector3(1.0e6, 1.0e6, 1.0e6))))
	for i in 60:
		big.wheel.apply_force(Vector3(0, 0, 2.0 * 0.8 * 15.0 * G))
		big.world.step()
	check(big.wheel.linear_velocity.z > 1.0, "a force of twice the Coulomb limit slides the wheel (v %s)" % big.wheel.linear_velocity.z)
	check(big.contact.friction_impulse_magnitude() <= big.contact.friction_limit() + 1.0e-9, "the friction impulse stays inside the Coulomb disk while sliding")


# --- Rolling -------------------------------------------------------------------------------------------------------------

func test_controlled_rolling_has_no_gross_penetration_and_the_contact_point_migrates() -> void:
	each_wheel(func(front, label):
		var rig: Dictionary = Rig.wheel_rig(front, 0.8)
		var speed: float = 3.0
		rig.wheel.linear_velocity = Vector3(0, 0, -speed)
		rig.wheel.angular_velocity = Vector3(-speed / rig.radius, 0, 0)  # rolling forward: negative rate about +X
		var worst_gap: float = 0.0
		var anchors: Array = []
		for i in 120:
			rig.world.step()
			worst_gap = maxf(worst_gap, absf(rig.contact.signed_gap()))
			if i % 30 == 29:
				anchors.append(rig.contact.local_anchor_b)
		check(worst_gap < 1.0e-3, "%s: rolling for 1 s never penetrates or lifts by 1 mm (worst |gap| %s)" % [label, worst_gap])
		check_near(rig.wheel.position.z, -speed * 1.0, 0.03, "%s: the wheel advanced v t without rolling resistance (no tyre model exists)" % label)
		check_near(-rig.wheel.linear_velocity.z, speed, 0.02, "%s: pure rolling keeps its speed" % label)
		check(rig.contact.tangential_velocity().length() < 0.02, "%s: rolling is slip-free (contact point velocity %s)" % [label, rig.contact.tangential_velocity().length()])
		# The material point under the contact turned by w t = (v/R) t radians: the successive local anchors differ by that angle.
		for j in range(1, anchors.size()):
			var turned: float = anchors[j - 1].angle_to(anchors[j])
			check_near(turned, speed / rig.radius * 0.25, 0.01, "%s: the contact point moved %s rad around the rim in 0.25 s" % [label, turned])
		check(anchors[0].distance_to(anchors[1]) > 0.1, "%s: the contact point is not fixed to one material point of the tyre" % label)
		check(absf(rig.wheel.orientation.get_euler().z) < 1.0e-3 and absf(rig.wheel.angular_velocity.y) < 1.0e-3, "%s: no roll or yaw developed during the 1 s test" % label))


func test_sustained_contact_never_produces_a_non_finite_state() -> void:
	each_wheel(func(front, label):
		var rig: Dictionary = Rig.wheel_rig(front, 0.8)
		rig.wheel.linear_velocity = Vector3(0, 0, -5.0)
		rig.wheel.angular_velocity = Vector3(-5.0 / rig.radius, 0, 0)
		var ok: bool = true
		for i in 600:
			ok = rig.world.step() and ok
		check(ok and rig.world.numerical_fault == "", "%s: 5 s of contact ran without a numerical fault" % label)
		check(rig.wheel.position.is_finite() and rig.wheel.linear_velocity.is_finite() and rig.wheel.angular_velocity.is_finite(), "%s: finite state" % label)
		check(rig.contact.normal_impulse() >= 0.0 and is_finite(rig.contact.normal_impulse()), "%s: finite non-negative impulse" % label))


# --- Contact lifecycle: separation, ghost impulses, re-contact -----------------------------------------------------------------

func test_separation_discards_carried_impulses_and_recontact_starts_cold() -> void:
	each_wheel(func(front, label):
		var rig: Dictionary = Rig.wheel_rig(front, 0.8)
		rig.wheel.linear_velocity = Vector3(0, 0, -2.0)
		for i in 60:
			rig.world.step()
		check(rig.contact.normal_impulse() > 0.0 and rig.contact.friction_impulse_magnitude() >= 0.0, "%s: the wheel is loaded before the test" % label)
		# Lift the wheel 1 m (test-side pose change between steps), stale impulses are still stored in the constraint.
		rig.contact.rows[0].accumulated_impulse = 50.0
		rig.contact._friction_impulse = Vector3(5.0, 0.0, 5.0)
		rig.wheel.position += Vector3(0, 1.0, 0)
		var v_before: Vector3 = rig.wheel.linear_velocity
		rig.world.step()
		check(not rig.contact.contact_active, "%s: a wheel 1 m away is inactive" % label)
		check(rig.contact.normal_impulse() == 0.0 and rig.contact.friction_impulse_magnitude() == 0.0, "%s: no stale normal or friction impulse survives (ghost-free)" % label)
		check(rig.contact.friction_impulse_world() == Vector3.ZERO and rig.contact._friction_impulse == Vector3.ZERO, "%s: the carried friction vector was cleared" % label)
		check_vec3_near(rig.wheel.linear_velocity, v_before + Vector3(0, -G * DT, 0), 1.0e-5, "%s: only gravity changed the velocity: no ghost force at a distance" % label)
		check(rig.contact.contact_state() != WheelGroundContact.STATE_TOUCHING, "%s: state is not touching while separated" % label)
		# Fall back: re-contact must produce a real impulse, then rest.
		var touched: bool = false
		for i in 240:
			rig.world.step()
			touched = touched or rig.contact.contact_active and rig.contact.normal_impulse() > 0.0
		check(touched, "%s: the wheel makes contact again after falling back" % label)
		check_near(rig.wheel.position.y, rig.radius, 2.0e-3, "%s: and comes to rest on the plane (y %s)" % [label, rig.wheel.position.y])
		check_near(rig.contact.normal_impulse(), rig.mass * G * DT, rig.mass * G * DT * 0.02, "%s: re-contact support again balances the weight" % label))


func test_lifecycle_is_deterministic() -> void:
	var results: Array = []
	for run in 2:
		var rig: Dictionary = Rig.wheel_rig(true, 0.8, 1.0)
		rig.wheel.linear_velocity = Vector3(0.3, 0.0, -2.0)
		rig.wheel.angular_velocity = Vector3(-4.0, 0.0, 0.0)
		for i in 360:
			rig.world.step()
		results.append([rig.wheel.position, rig.wheel.linear_velocity, rig.wheel.angular_velocity, rig.contact.normal_impulse()])
	check(results[0][0] == results[1][0] and results[0][1] == results[1][1] and results[0][2] == results[1][2] and results[0][3] == results[1][3],
		"drop, landing, separation and re-contact are bit-identical between two runs")


func test_approaching_wheel_is_stopped_at_the_plane_not_beyond_it() -> void:
	# A fast approach from outside the activation distance must still be caught (speculative reach grows with the approach speed).
	each_wheel(func(front, label):
		var rig: Dictionary = Rig.wheel_rig(front, 0.8, 0.0)
		rig.wheel.position = Vector3(0, rig.radius + 0.30, 0)
		rig.wheel.linear_velocity = Vector3(0, -20.0, 0)  # 0.167 m per step: crosses 0.30 m in two steps
		var lowest: float = INF
		for i in 20:
			rig.world.step()
			lowest = minf(lowest, rig.contact.signed_gap())
		check(lowest > -0.01, "%s: the fast wheel is stopped at the plane (lowest gap %s)" % [label, lowest])
		check(rig.wheel.position.y > rig.radius - 0.01, "%s: it did not tunnel" % label))


func test_ground_cannot_become_dynamic_or_move() -> void:
	var rig: Dictionary = Rig.wheel_rig(false, 0.8)
	rig.wheel.linear_velocity = Vector3(0, -3.0, -1.0)
	for i in 120:
		rig.world.step()
	check(rig.ground.is_static(), "the ground body is static")
	check(rig.ground.linear_velocity == Vector3.ZERO and rig.ground.angular_velocity == Vector3.ZERO, "the ground acquired no velocity from contact")
	check(rig.ground.position == Vector3.ZERO and rig.ground.orientation == Quaternion.IDENTITY, "the ground did not move")
	# Regression: a ground made dynamic is refused by the world (no silent simulation of a movable plane).
	rig.ground.set_mass_properties(SimMassProperties.new(100.0, SimMassProperties.sphere_inertia(100.0, 1.0)))
	var before: int = rig.world.step_count
	check(rig.world.advance(DT) == 0 and rig.world.step_count == before, "a dynamic ground makes the world refuse to step")
	check(rig.world.last_error.contains("ground body must be static"), "the refusal names the cause (%s)" % rig.world.last_error)


# --- Fixed-step lifecycle through SimWorld.advance() -----------------------------------------------------------------------------

## Distance between the contact's current anchors and the anchors an independent evaluation of the geometry gives for the pose at
## the START of the last executed step (previous_state): the contact is prepared from the start-of-step pose.
func anchor_error_for_last_step(contact, wheel, radius: float) -> float:
	var q: Quaternion = wheel.previous_state.orientation
	var c: Vector3 = wheel.previous_state.position
	var u: Vector3 = q * Vector3.RIGHT
	var d: Vector3 = (Vector3.UP - u * u.y).normalized()
	var wheel_point: Vector3 = c - d * radius
	var expected_b: Vector3 = q.inverse() * (wheel_point - c)
	var expected_a: Vector3 = Vector3(wheel_point.x, 0.0, wheel_point.z)
	return maxf((contact.local_anchor_b - expected_b).length(), (contact.local_anchor_a - expected_a).length())


func test_advance_with_several_fixed_steps_refreshes_the_contact_at_every_step() -> void:
	each_wheel(func(front, label):
		var rig: Dictionary = Rig.wheel_rig(front, 0.8)
		var speed: float = 6.0
		rig.wheel.linear_velocity = Vector3(0, 0, -speed)
		rig.wheel.angular_velocity = Vector3(-speed / rig.radius, 0, 0)
		var refreshes_before: int = rig.contact.refresh_count
		var steps: int = rig.world.advance(5.0 * DT + 1.0e-6)
		check(steps == 5, "%s: one frame ran five fixed steps (got %d)" % [label, steps])
		check(rig.contact.refresh_count - refreshes_before == 5, "%s: the geometry was refreshed once per fixed step, 5 times (got %d)" % [label, rig.contact.refresh_count - refreshes_before])
		check(anchor_error_for_last_step(rig.contact, rig.wheel, rig.radius) < 1.0e-5, "%s: the anchors belong to the pose at the start of the last step, not to the frame start" % label)
		# The wheel moved 5 * v dt = 0.25 m in the frame: an anchor refreshed once per frame would be 4 steps (0.2 m) stale.
		check(absf(rig.contact.local_anchor_a.z - rig.wheel.previous_state.position.z) < 1.0e-5, "%s: the ground anchor follows the translation (z %s vs start-of-step %s)" % [label, rig.contact.local_anchor_a.z, rig.wheel.previous_state.position.z])
		check(absf(rig.contact.local_anchor_a.z) > 0.15, "%s: and it is far from where it started (so a stale anchor would be detected)" % label)
		# Frames that run zero steps must not refresh anything.
		var count: int = rig.contact.refresh_count
		check(rig.world.advance(0.0005) == 0 and rig.contact.refresh_count == count, "%s: a frame without a fixed step does not touch the contact" % label)
		# Frames of different lengths: refresh count tracks the executed steps exactly.
		var executed: int = 0
		for frame in [0.003, 0.02, 0.0166, 0.05, 0.008]:
			executed += rig.world.advance(frame)
		check(rig.contact.refresh_count - count == executed, "%s: refreshes = executed fixed steps over variable frames (%d vs %d)" % [label, rig.contact.refresh_count - count, executed]))


func test_stale_anchor_implementations_are_detected_by_the_lifecycle_check() -> void:
	# Regression sensitivity: a plain generic ContactConstraint whose anchors are set once (the forbidden design) fails the same
	# check that the lifecycle test applies to WheelGroundContact.
	var rig: Dictionary = Rig.wheel_rig(false, 0.8)
	var plain = ContactConstraint.from_world(rig.ground, rig.wheel, Vector3.ZERO, Vector3(0, 0.0, 0), Vector3.UP, 0.8)
	rig.world.remove_constraint(rig.contact)
	rig.world.add_constraint(plain)
	rig.wheel.linear_velocity = Vector3(0, 0, -6.0)
	rig.wheel.angular_velocity = Vector3(-6.0 / rig.radius, 0, 0)
	rig.world.advance(5.0 * DT + 1.0e-6)
	var error: float = anchor_error_for_last_step(plain, rig.wheel, rig.radius)
	check(error > 0.1, "a contact that is never re-seated is 0.1+ m off after a 5-step frame (error %s): the lifecycle check rejects it" % error)
	# The same wheel with the fixed anchor also fails physically: after rolling by about one radian the anchor is high on the rim.
	var rig2: Dictionary = Rig.wheel_rig(false, 0.8)
	var plain2 = ContactConstraint.from_world(rig2.ground, rig2.wheel, Vector3.ZERO, Vector3(0, 0.0, 0), Vector3.UP, 0.8)
	rig2.world.remove_constraint(rig2.contact)
	rig2.world.add_constraint(plain2)
	rig2.wheel.linear_velocity = Vector3(0, 0, -3.0)
	rig2.wheel.angular_velocity = Vector3(-3.0 / rig2.radius, 0, 0)
	for i in 120:
		rig2.world.step()
	check(absf(rig2.wheel.position.y - rig2.radius) > 0.01, "a contact point fixed in the wheel frame makes the wheel leave the correct height (y %s vs R %s)" % [rig2.wheel.position.y, rig2.radius])


func test_reversed_normal_and_wrong_radius_are_detected() -> void:
	# Reversed normal (wheel as body A): the generic contact then pushes the wheel INTO the ground; the wheel falls through.
	var rig: Dictionary = Rig.wheel_rig(false, 0.8)
	rig.world.remove_constraint(rig.contact)
	var reversed = ContactConstraint.from_world(rig.wheel, rig.ground, Vector3(0, -rig.radius, 0), Vector3.ZERO, Vector3.UP, 0.8)
	rig.world.add_constraint(reversed)
	for i in 120:
		rig.world.step()
	check(rig.wheel.position.y < rig.radius - 0.1, "a reversed contact normal lets the wheel through the ground (y %s): the rest-height check rejects it" % rig.wheel.position.y)
	# Wrong radius: 20 mm too small; the wheel then rests 20 mm too low and the 1e-4 rest-height tolerance rejects it.
	var wrong: Dictionary = Rig.wheel_rig(false, 0.8)
	wrong.contact.wheel_radius = wrong.radius - 0.02
	for i in 240:
		wrong.world.step()
	check(absf(wrong.wheel.position.y - wrong.radius) > 0.015, "an incorrect wheel radius shifts the resting height by the same amount (y %s)" % wrong.wheel.position.y)


# --- Plane placement ---------------------------------------------------------------------------------------------------------

func test_the_wheel_rests_on_an_offset_plane() -> void:
	var rig: Dictionary = Rig.wheel_rig(false, 0.8, -1.0, Vector3(0, -9.81, 0), 0.25)
	for i in 240:
		rig.world.step()
	check_near(rig.wheel.position.y, 0.25 + rig.radius, 1.0e-4, "the wheel rests R above a plane offset to y = 0.25")
	check_near(rig.contact.normal_impulse(), rig.mass * G * DT, rig.mass * G * DT * 1.0e-3, "support force balances the weight on the offset plane")
