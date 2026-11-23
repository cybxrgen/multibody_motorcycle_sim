extends "res://Tests/sim_test_case.gd"
## M2.4.1 ground plane and wheel contact GEOMETRY (WheelGroundContact.compute_geometry / refresh / create) and configuration rules.
## Expected values are analytic (derived below, never read back from the implementation).
## Tolerance: float32 vector quantities of order 1 m: 2e-6 (a few ulps of ~1.2e-7 relative, several operations).

const Rig = preload("res://Tests/ground_rig.gd")
const WheelGroundContact = preload("res://Simulation/Motorcycle/wheel_ground_contact.gd")
const MotorcycleConfig = preload("res://Simulation/Motorcycle/motorcycle_config.gd")
const MotorcyclePrototype = preload("res://Simulation/Motorcycle/motorcycle_prototype.gd")
const MotorcycleBuilder = preload("res://Simulation/Motorcycle/motorcycle_builder.gd")
const GroundSpec = preload("res://Simulation/Motorcycle/motorcycle_ground_spec.gd")

const TOL: float = 2.0e-6
const R: float = 0.32


func geometry(pos: Vector3, rot: Quaternion, radius: float = R, plane_point: Vector3 = Vector3.ZERO, normal: Vector3 = Vector3.UP) -> Dictionary:
	return WheelGroundContact.compute_geometry(pos, rot, radius, plane_point, normal)


func test_horizontal_plane_signed_distance_and_upright_contact_point() -> void:
	# Plane y = 0, normal +Y, wheel centre at (0.4, 0.5, -1.2): lowest point (0.4, 0.5 - R, -1.2), gap 0.5 - R, ground point below it.
	var g: Dictionary = geometry(Vector3(0.4, 0.5, -1.2), Quaternion.IDENTITY)
	check(g.valid, "upright wheel: valid geometry")
	check_vec3_near(g.wheel_point, Vector3(0.4, 0.5 - R, -1.2), TOL, "lowest point is one radius straight below the centre")
	check_vec3_near(g.ground_point, Vector3(0.4, 0.0, -1.2), TOL, "ground point is the plane point directly below")
	check_vec3_near(g.normal, Vector3.UP, 0.0, "normal is +Y")
	check_near(g.gap, 0.5 - R, TOL, "signed gap = centre height - R")
	check_vec3_near(g.axle, Vector3.RIGHT, TOL, "axle is the world X axis")
	check_near(geometry(Vector3(0, R, 0), Quaternion.IDENTITY).gap, 0.0, TOL, "centre at height R touches")
	check(geometry(Vector3(0, R - 0.01, 0), Quaternion.IDENTITY).gap < 0.0, "centre below R penetrates (negative gap)")


func test_ground_plane_offset() -> void:
	# Plane through (5, 0.2, 7), normal +Y: signed distance of a point is y - 0.2 independent of x and z.
	var g: Dictionary = geometry(Vector3(0.0, 0.9, 0.0), Quaternion.IDENTITY, R, Vector3(5.0, 0.2, 7.0))
	check_near(g.gap, 0.9 - R - 0.2, TOL, "gap measured from the offset plane")
	check_vec3_near(g.ground_point, Vector3(0.0, 0.2, 0.0), TOL, "ground point lies on the offset plane below the wheel")
	var low: Dictionary = geometry(Vector3(0.0, 0.2 + R, 0.0), Quaternion.IDENTITY, R, Vector3(5.0, 0.2, 7.0))
	check_near(low.gap, 0.0, TOL, "wheel resting on the offset plane touches")


func test_inclined_plane_normal_and_signed_distance() -> void:
	# A general unit normal (the representation is not limited to +Y): n = (0, 0.6, 0.8) through the origin, wheel with axle X.
	# in-plane direction of n for u = X is n itself (n.u = 0); lowest point = c - R n; gap = n.(c - R n) = n.c - R.
	var n: Vector3 = Vector3(0.0, 0.6, 0.8)
	var c: Vector3 = Vector3(0.1, 1.0, 0.5)
	var g: Dictionary = geometry(c, Quaternion.IDENTITY, R, Vector3.ZERO, n)
	check_near(g.gap, n.dot(c) - R, TOL, "gap along a general normal")
	check_vec3_near(g.wheel_point, c - n * R, TOL, "lowest point along a general normal")
	check_vec3_near(g.ground_point, g.wheel_point - n * g.gap, TOL, "ground point is the orthogonal projection")


func test_spin_about_the_axle_moves_the_local_anchor_but_not_the_world_point() -> void:
	# Rotation theta about the axle: the world contact point stays at the bottom, the material point migrates. For R_x(theta) the
	# body-frame offset of the bottom point is R_x(-theta) (0, -R, 0) = (0, -R cos t, +R sin t).
	for theta in [0.0, 0.5, 1.2, 2.5, -0.9]:
		var rig: Dictionary = Rig.wheel_rig(false, 0.5)
		rig.wheel.orientation = Quaternion(Vector3.RIGHT, theta)
		rig.contact.refresh()
		check_vec3_near(rig.contact.world_point_b(), Vector3(0.0, 0.0, 0.0), TOL, "theta=%s: world contact point stays on the ground below the centre" % theta)
		check_vec3_near(rig.contact.local_anchor_b, Vector3(0.0, -R * cos(theta), R * sin(theta)), TOL, "theta=%s: material point migrated by the spin angle" % theta)
		check_near(rig.contact.local_anchor_b.length(), R, TOL, "theta=%s: anchor stays on the rim (radius R)" % theta)
		check_vec3_near(rig.contact.world_normal(), Vector3.UP, TOL, "theta=%s: normal stays +Y" % theta)


func test_camber_tilt_lowest_point() -> void:
	# Axle tilted by phi about Z: u = (cos p, sin p, 0). n projected onto the wheel plane: cos p (-sin p, cos p, 0), so the unit
	# direction is d = (-sin p, cos p, 0); lowest point c - R d = c + (R sin p, -R cos p, 0); its height is c_y - R cos p.
	for phi in [0.0, 0.2, 0.6, -0.5, 1.2]:
		var c: Vector3 = Vector3(0.3, 0.7, -0.2)
		var g: Dictionary = geometry(c, Quaternion(Vector3.BACK, phi))
		check(g.valid, "phi=%s: valid" % phi)
		check_vec3_near(g.wheel_point, c + Vector3(R * sin(phi), -R * cos(phi), 0.0), TOL, "phi=%s: lowest point of a cambered wheel" % phi)
		check_near(g.gap, c.y - R * cos(phi), TOL, "phi=%s: gap uses R cos(camber)" % phi)
		check_near(g.wheel_point.distance_to(c), R, TOL, "phi=%s: contact point stays on the rim circle" % phi)
	# Pitch rotation about the (horizontal) Y... a yaw about Y does not change the axle's angle to the plane: same gap as upright.
	var yawed: Dictionary = geometry(Vector3(0.0, 0.6, 0.0), Quaternion(Vector3.UP, 0.8))
	check_near(yawed.gap, 0.6 - R, TOL, "yaw leaves the lowest point height unchanged")


func test_wheel_radius_enters_the_gap_exactly() -> void:
	for radius in [0.31, 0.32, 0.45]:
		check_near(geometry(Vector3(0, 0.5, 0), Quaternion.IDENTITY, radius).gap, 0.5 - radius, TOL, "gap = height - radius for radius %s" % radius)
	var difference: float = geometry(Vector3(0, 0.5, 0), Quaternion.IDENTITY, 0.31).gap - geometry(Vector3(0, 0.5, 0), Quaternion.IDENTITY, 0.32).gap
	check_near(difference, 0.01, TOL, "a 10 mm radius change moves the gap by 10 mm")


func test_local_world_conversion_with_general_poses() -> void:
	# A wheel with a general orientation (camber + yaw + spin) and a non-trivial position: the anchors stored in each body's local
	# frame must map back to the geometric world points, and the ground anchor to the plane.
	var rig: Dictionary = Rig.wheel_rig(true, 0.5, 0.4, Vector3.ZERO, 0.1)
	var rot: Quaternion = Quaternion(Vector3(0.3, 0.8, -0.4).normalized(), 0.35)
	rig.wheel.position = Vector3(1.1, 0.55, -2.3)
	rig.wheel.orientation = rot
	rig.contact.refresh()
	var u: Vector3 = rot * Vector3.RIGHT
	var d: Vector3 = (Vector3.UP - u * u.y).normalized()
	var expected_wheel_point: Vector3 = rig.wheel.position - d * rig.radius
	check_vec3_near(rig.contact.world_point_b(), expected_wheel_point, TOL, "world point of the wheel anchor equals c - R d")
	check_vec3_near(rig.contact.world_point_a(), Vector3(expected_wheel_point.x, 0.1, expected_wheel_point.z), TOL, "ground anchor is the projection onto the plane y = 0.1")
	check_near(rig.contact.gap(), expected_wheel_point.y - 0.1, TOL, "ContactConstraint gap equals the signed distance of the lowest point")
	check_near(rig.contact.signed_gap(), expected_wheel_point.y - 0.1, TOL, "signed_gap() query agrees")
	check_vec3_near(rig.wheel.orientation * rig.contact.local_anchor_b + rig.wheel.position, expected_wheel_point, TOL, "local anchor is rotation-consistent")


func test_contact_normal_points_from_ground_to_wheel() -> void:
	# ContactConstraint convention: normal from body A toward body B. A = ground, B = wheel, so the normal is +n (up).
	var rig: Dictionary = Rig.wheel_rig(false, 0.5, 0.5)
	check(rig.contact.body_a == rig.ground and rig.contact.body_b == rig.wheel, "A is the ground, B the wheel")
	check_vec3_near(rig.contact.world_normal(), Vector3.UP, TOL, "normal from ground toward wheel is +Y")
	check(rig.contact.gap() > 0.0, "wheel above the plane gives a positive gap")


func test_contact_point_follows_translation() -> void:
	# The ground-side anchor is the plane point below the wheel: it moves with the wheel's horizontal position.
	var rig: Dictionary = Rig.wheel_rig(false, 0.5, 0.5)
	for x in [0.0, 0.7, -2.0]:
		rig.wheel.position = Vector3(x, 0.5, x * 0.5)
		rig.contact.refresh()
		check_vec3_near(rig.contact.local_anchor_a, Vector3(x, 0.0, x * 0.5), TOL, "ground anchor below the wheel at x=%s" % x)


func test_degenerate_axle_parallel_to_normal() -> void:
	# Wheel lying flat: axle = +Y = the normal; the in-plane projection of n vanishes. Never normalised; reported as invalid.
	var flat: Dictionary = geometry(Vector3(0, 0.5, 0), Quaternion(Vector3.BACK, PI / 2.0))
	check(not flat.valid, "axle parallel to the normal is degenerate")
	check(flat.reason.contains("degenerate"), "the reason names the degeneracy (%s)" % flat.reason)
	check(flat.wheel_point == Vector3.ZERO and flat.ground_point == Vector3.ZERO, "no point is produced from a vanishing vector")
	# Just inside the domain: sin(angle) = 0.2 (> 0.1) is valid and finite; just outside (0.05) is degenerate.
	var inside: Dictionary = geometry(Vector3(0, 0.5, 0), Quaternion(Vector3.BACK, PI / 2.0 - asin(0.2)))
	check(inside.valid and inside.wheel_point.is_finite(), "near-flat wheel inside the documented domain stays valid and finite")
	var outside: Dictionary = geometry(Vector3(0, 0.5, 0), Quaternion(Vector3.BACK, PI / 2.0 - asin(0.05)))
	check(not outside.valid, "near-flat wheel beyond the documented domain is rejected")
	var rig: Dictionary = Rig.wheel_rig(false, 0.5, 0.5)
	rig.wheel.orientation = Quaternion(Vector3.BACK, PI / 2.0)
	check(not rig.contact.refresh(), "refresh() reports the degenerate geometry")
	check(not rig.contact.geometry_valid, "geometry_valid is cleared")
	check(is_nan(rig.contact.signed_gap()), "signed_gap() is NaN, never a fabricated number")
	check(rig.contact.contact_state() == WheelGroundContact.STATE_INACTIVE, "state is inactive")
	check(rig.contact.local_anchor_b.is_finite() and rig.contact.local_normal_a.is_finite(), "anchors stay finite (untouched)")


func test_degenerate_wheel_never_feeds_non_finite_values_to_the_solver() -> void:
	var rig: Dictionary = Rig.wheel_rig(false, 0.5, 0.5)
	rig.wheel.orientation = Quaternion(Vector3.BACK, PI / 2.0)
	rig.wheel.position = Vector3(0, 0.1, 0)
	var steps_ok: bool = true
	for i in 30:
		steps_ok = rig.world.step() and steps_ok
	check(steps_ok and rig.world.numerical_fault == "", "stepping with a flat wheel is safe (the contact is inactive)")
	check(rig.wheel.position.is_finite() and rig.wheel.linear_velocity.is_finite(), "state stays finite")
	check(not rig.contact.contact_active and rig.contact.normal_impulse() == 0.0, "no impulse from a degenerate contact")


func test_create_rejects_invalid_input() -> void:
	var rig: Dictionary = Rig.wheel_rig(false, 0.5, 0.5)
	var g = rig.ground
	var wh = rig.wheel
	check(WheelGroundContact.create(null, wh, R, Vector3.ZERO, Vector3.UP, 0.5) == null, "null ground rejected")
	check(WheelGroundContact.create(g, null, R, Vector3.ZERO, Vector3.UP, 0.5) == null, "null wheel rejected")
	check(WheelGroundContact.create(g, wh, 0.0, Vector3.ZERO, Vector3.UP, 0.5) == null, "zero radius rejected")
	check(WheelGroundContact.create(g, wh, -0.3, Vector3.ZERO, Vector3.UP, 0.5) == null, "negative radius rejected")
	check(WheelGroundContact.create(g, wh, NAN, Vector3.ZERO, Vector3.UP, 0.5) == null, "NaN radius rejected")
	check(WheelGroundContact.create(g, wh, INF, Vector3.ZERO, Vector3.UP, 0.5) == null, "infinite radius rejected")
	check(WheelGroundContact.create(g, wh, R, Vector3(NAN, 0, 0), Vector3.UP, 0.5) == null, "non-finite plane point rejected")
	check(WheelGroundContact.create(g, wh, R, Vector3.ZERO, Vector3(0, 2, 0), 0.5) == null, "non-unit normal rejected")
	check(WheelGroundContact.create(g, wh, R, Vector3.ZERO, Vector3.ZERO, 0.5) == null, "zero normal rejected")
	check(WheelGroundContact.create(g, wh, R, Vector3.ZERO, Vector3(0, NAN, 0), 0.5) == null, "NaN normal rejected")
	check(WheelGroundContact.create(g, wh, R, Vector3.ZERO, Vector3.UP, -0.1) == null, "negative friction rejected")
	check(WheelGroundContact.create(g, wh, R, Vector3.ZERO, Vector3.UP, NAN) == null, "NaN friction rejected")
	check(WheelGroundContact.create(g, wh, R, Vector3.ZERO, Vector3.UP, 0.5, -1.0) == null, "negative activation distance rejected")
	check(WheelGroundContact.create(g, wh, R, Vector3.ZERO, Vector3.UP, 0.5, NAN) == null, "NaN activation distance rejected")
	check(WheelGroundContact.create(wh, wh, R, Vector3.ZERO, Vector3.UP, 0.5) == null, "same body twice rejected")
	var flat_wheel = make_sphere(5.0, 0.3, Vector3(0, 0.5, 0))
	flat_wheel.orientation = Quaternion(Vector3.BACK, PI / 2.0)
	check(WheelGroundContact.create(g, flat_wheel, R, Vector3.ZERO, Vector3.UP, 0.5) == null, "degenerate initial geometry rejected")
	check(WheelGroundContact.create(g, wh, R, Vector3.ZERO, Vector3.UP, 0.0) != null, "zero friction is valid")


func test_validate_requires_a_static_ground_and_a_dynamic_wheel() -> void:
	var rig: Dictionary = Rig.wheel_rig(false, 0.5, 0.5)
	check(rig.contact.is_valid(), "a fresh contact is valid")
	var errors_before: int = rig.contact.validate().size()
	rig.ground.set_mass_properties(SimMassProperties.new(5.0, SimMassProperties.sphere_inertia(5.0, 0.2)))
	check(rig.contact.validate().size() > errors_before, "a dynamic ground is reported as invalid")
	var message: String = "; ".join(rig.contact.validate())
	check(message.contains("ground body must be static"), "the message names the rule (%s)" % message)
	rig.contact.wheel_radius = NAN
	check("; ".join(rig.contact.validate()).contains("wheel radius"), "mutating the radius to NaN is caught by validate()")


func test_config_ground_rules() -> void:
	check(MotorcyclePrototype.config().ground == null, "the plain prototype has no ground (M2.1-M2.3 assembly unchanged)")
	check(MotorcyclePrototype.config_with_ground().is_valid(), "the prototype with ground is valid")
	var g = MotorcyclePrototype.config_with_ground().ground
	check_vec3_near(g.normal, Vector3.UP, 0.0, "prototype ground normal is +Y")
	check_vec3_near(g.point, Vector3.ZERO, 0.0, "prototype ground plane passes through the origin")
	check_near(g.friction, MotorcyclePrototype.GROUND_FRICTION, 0.0, "prototype friction constant")
	var cases: Array = [
		["NaN point", func(s): s.point = Vector3(NAN, 0, 0)],
		["infinite point", func(s): s.point = Vector3(0, INF, 0)],
		["NaN normal", func(s): s.normal = Vector3(0, NAN, 0)],
		["zero normal", func(s): s.normal = Vector3.ZERO],
		["non-unit normal", func(s): s.normal = Vector3(0, 1.5, 0)],
		["negative friction", func(s): s.friction = -0.1],
		["NaN friction", func(s): s.friction = NAN],
		["infinite friction", func(s): s.friction = INF],
		["negative activation distance", func(s): s.activation_distance = -0.01],
		["NaN activation distance", func(s): s.activation_distance = NAN],
		["wheel axle parallel to the ground normal", func(s): s.normal = Vector3.RIGHT],
	]
	for entry in cases:
		var config: MotorcycleConfig = MotorcyclePrototype.config_with_ground()
		entry[1].call(config.ground)
		check(not config.is_valid(), "invalid ground rejected by validate(): %s" % entry[0])
		var world = make_world()
		check(MotorcycleBuilder.build(config, world) == null, "build() refuses it: %s" % entry[0])
		check(world.bodies.is_empty() and world.constraints.is_empty() and world.force_providers.is_empty(), "nothing registered after refusing: %s" % entry[0])
	var zero_mu: MotorcycleConfig = MotorcyclePrototype.config_with_ground()
	zero_mu.ground.friction = 0.0
	check(zero_mu.is_valid(), "zero friction is a valid ground")
