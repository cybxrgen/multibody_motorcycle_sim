extends "res://Tests/sim_test_case.gd"
## Mass, inertia, world-space transforms, force-vs-impulse semantics.


func test_mass_and_inverse_mass_consistent() -> void:
	for m in [0.1, 1.0, 12.5, 250.0]:
		var mp = SimMassProperties.new(m, SimMassProperties.sphere_inertia(m, 0.3))
		check_near(mp.mass * mp.inverse_mass, 1.0, 1e-12, "m * m^-1 = 1 for m=%s" % m)
		check_basis_near(mp.local_inertia * mp.inverse_local_inertia, Basis.IDENTITY, 1e-6, "I * I^-1 = identity for m=%s" % m)


func test_zero_mass_is_static() -> void:
	var b = make_static(Vector3(1, 1, 1))
	check(b.is_static(), "zero mass -> static")
	check_near(b.mass, 0.0, 0.0, "mass zero")
	check_near(b.inverse_mass, 0.0, 0.0, "inverse mass exactly zero")
	check_basis_near(b.inverse_inertia_world, Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO), 0.0, "inverse inertia zero")
	b.apply_impulse(Vector3(5, 5, 5))
	b.apply_angular_impulse(Vector3(5, 5, 5))
	b.apply_impulse_at_point(Vector3(1, 2, 3), Vector3(4, 5, 6))
	check_vec3_near(b.linear_velocity, Vector3.ZERO, 0.0, "impulses do not move static bodies")
	check_vec3_near(b.angular_velocity, Vector3.ZERO, 0.0, "angular impulses do not spin static bodies")


func test_principal_inertia_formulas() -> void:
	# Box m=12, size (1,2,3): Ixx = (12/12)(4+9)=13, Iyy = 1+9 = 10, Izz = 1+4 = 5.
	var box: Basis = SimMassProperties.box_inertia(12.0, Vector3(1, 2, 3))
	check_basis_near(box, Basis.from_scale(Vector3(13, 10, 5)), 1e-12, "solid box principal moments")
	# Sphere m=5, r=2: 2/5 * 5 * 4 = 8.
	var sph: Basis = SimMassProperties.sphere_inertia(5.0, 2.0)
	check_basis_near(sph, Basis.from_scale(Vector3(8, 8, 8)), 1e-12, "solid sphere moment")


func test_world_inertia_for_rotated_body() -> void:
	var local: Basis = SimMassProperties.diagonal_inertia(Vector3(1, 2, 3))
	# +90 deg about Z maps local X -> world Y: world diag becomes (2, 1, 3).
	var b = make_body(1.0, local, Vector3.ZERO, Quaternion(Vector3.BACK, PI / 2.0))
	check_basis_near(b.inertia_world, Basis.from_scale(Vector3(2, 1, 3)), 1e-6, "Rz(90) swaps Ixx and Iyy")
	check_basis_near(b.inertia_world * b.inverse_inertia_world, Basis.IDENTITY, 1e-6, "I_w * I_w^-1 = identity")
	# General rotation: R^T I_w R must give back the local tensor; I_w must be symmetric.
	var q: Quaternion = Quaternion(Vector3(1, 2, -1).normalized(), 0.83)
	var c = make_body(2.0, local, Vector3.ZERO, q)
	var r: Basis = Basis(q)
	check_basis_near(r.transposed() * c.inertia_world * r, local, 1e-5, "R^T I_w R = I_local")
	check_near(c.inertia_world.x.y, c.inertia_world.y.x, 1e-12, "I_w symmetric (xy)")
	check_near(c.inertia_world.x.z, c.inertia_world.z.x, 1e-12, "I_w symmetric (xz)")
	check_near(c.inertia_world.y.z, c.inertia_world.z.y, 1e-12, "I_w symmetric (yz)")
	# Trace is rotation invariant.
	var tr_local: float = local.x.x + local.y.y + local.z.z
	var tr_world: float = c.inertia_world.x.x + c.inertia_world.y.y + c.inertia_world.z.z
	check_near(tr_world, tr_local, 1e-5, "trace invariant under rotation")
	# Re-assigning the orientation refreshes the cache.
	c.orientation = Quaternion.IDENTITY
	check_basis_near(c.inertia_world, local, 1e-12, "cache follows orientation setter")


func test_full_symmetric_inertia_supported() -> void:
	# A non-diagonal tensor built as R diag R^T must work as local inertia.
	var r: Basis = Basis(Quaternion(Vector3(0.3, 1, 0.2).normalized(), 0.6))
	var full: Basis = r * Basis.from_scale(Vector3(1, 2, 3)) * r.transposed()
	var mp = SimMassProperties.new(1.0, full)
	check(not mp.is_static(), "non-diagonal tensor accepted")
	check_basis_near(mp.local_inertia * mp.inverse_local_inertia, Basis.IDENTITY, 1e-6, "inverse of full tensor")


func test_force_at_offset_produces_torque() -> void:
	var b = make_sphere(2.0, 0.5, Vector3(2, 0, 0))
	# r = (3,0,0) - (2,0,0) = (1,0,0); F = (0,0,10); tau = r x F = (0,-10,0)
	b.apply_force_at_point(Vector3(0, 0, 10), Vector3(3, 0, 0))
	check_vec3_near(b.force_accumulator, Vector3(0, 0, 10), 1e-12, "force accumulated")
	check_vec3_near(b.torque_accumulator, Vector3(0, -10, 0), 1e-12, "torque = (p - x_com) x F")
	# A force through the COM produces no torque.
	var c = make_sphere(1.0, 0.5, Vector3(5, 5, 5))
	c.apply_force_at_point(Vector3(3, 2, 1), Vector3(5, 5, 5))
	check_vec3_near(c.torque_accumulator, Vector3.ZERO, 1e-12, "no torque through COM")
	b.clear_accumulators()
	check_vec3_near(b.force_accumulator, Vector3.ZERO, 0.0, "force cleared")
	check_vec3_near(b.torque_accumulator, Vector3.ZERO, 0.0, "torque cleared")


func test_impulse_is_instantaneous_and_not_a_force() -> void:
	var b = make_sphere(2.0, 0.5, Vector3(2, 0, 0))  # I = 0.4*2*0.25 = 0.2
	b.apply_impulse_at_point(Vector3(0, 0, 10), Vector3(3, 0, 0))
	check_vec3_near(b.linear_velocity, Vector3(0, 0, 5), 1e-12, "dv = J/m")
	check_vec3_near(b.angular_velocity, Vector3(0, -50, 0), 1e-4, "domega = I^-1 (r x J) = (0,-10,0)/0.2")
	check_vec3_near(b.force_accumulator, Vector3.ZERO, 0.0, "impulse does not touch force accumulator")
	check_vec3_near(b.torque_accumulator, Vector3.ZERO, 0.0, "impulse does not touch torque accumulator")
	# Same magnitude as a force: an impulse J equals a force J/dt applied for one step.
	var w = make_world(Vector3.ZERO)
	var f = make_sphere(2.0, 0.5, Vector3(2, 0, 0))
	w.add_body(f)
	var dt: float = w.config.fixed_timestep
	f.apply_force_at_point(Vector3(0, 0, 10) / dt, Vector3(3, 0, 0))
	w.step()
	check_vec3_near(f.linear_velocity, Vector3(0, 0, 5), 1e-6, "force J/dt over one step == impulse J (linear)")
	check_vec3_near(f.angular_velocity, Vector3(0, -50, 0), 1e-4, "force J/dt over one step == impulse J (angular)")


# --- M0.1: inertia tensor validation ---
# Tolerances: Basis is float32 (eps 1.19e-7). I * I^-1 is compared to identity at 1e-5, which covers
# the condition numbers used here (<= ~1e4 * 1.2e-7 ~ 1e-3 worst case only for the deliberately
# ill-conditioned boundary tensors, which are validated but not multiplied back).

func _rotated_tensor(principal: Vector3) -> Basis:
	var r: Basis = Basis(Quaternion(Vector3(0.3, 1, 0.2).normalized(), 0.6))
	return r * Basis.from_scale(principal) * r.transposed()


func _check_rejected(inertia: Basis, label: String) -> void:
	check(SimMassProperties.validate_inertia(inertia) != "", "validate_inertia rejects: " + label)
	var mp = SimMassProperties.new(1.0, inertia)
	check(not mp.valid, "constructor flags invalid: " + label)
	check(mp.error_message != "", "error message set: " + label)
	check_near(mp.inverse_mass, 0.0, 0.0, "no inverse mass leaks from invalid: " + label)
	check_basis_near(mp.inverse_local_inertia, Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO), 0.0, "no inverse inertia leaks: " + label)


func _check_accepted(inertia: Basis, label: String) -> void:
	check(SimMassProperties.validate_inertia(inertia) == "", "validate_inertia accepts: " + label)
	var mp = SimMassProperties.new(1.0, inertia)
	check(mp.valid and mp.error_message == "", "constructor valid: " + label)
	check(not mp.is_static(), "dynamic: " + label)


func test_valid_diagonal_inertia_accepted() -> void:
	var inertia: Basis = SimMassProperties.diagonal_inertia(Vector3(1, 2, 3))
	_check_accepted(inertia, "diag(1,2,3)")
	var mp = SimMassProperties.new(2.0, inertia)
	check_basis_near(mp.local_inertia * mp.inverse_local_inertia, Basis.IDENTITY, 1e-5, "I * I^-1 = identity")


func test_valid_nondiagonal_inertia_accepted() -> void:
	var full: Basis = _rotated_tensor(Vector3(1, 2, 3))
	check(absf(full.x.y) > 0.1, "tensor really is non-diagonal")
	_check_accepted(full, "R diag(1,2,3) R^T (float32 rounding asymmetry tolerated)")


func test_negative_principal_moments_rejected() -> void:
	var flipped: Basis = Basis.from_scale(Vector3(-1, -1, 1))
	check(flipped.determinant() > 0.0, "premise: determinant of diag(-1,-1,1) is positive")
	_check_rejected(flipped, "diag(-1,-1,1)")
	_check_rejected(Basis.from_scale(Vector3(-1, 2, 3)), "diag(-1,2,3)")
	_check_rejected(Basis.from_scale(Vector3(-1, -2, -3)), "diag(-1,-2,-3)")


func test_singular_inertia_rejected() -> void:
	_check_rejected(Basis.from_scale(Vector3(1, 1, 0)), "diag(1,1,0)")
	_check_rejected(Basis(Vector3(1, 2, 3), Vector3(2, 4, 6), Vector3(3, 6, 9)), "symmetric rank-1 tensor")
	_check_rejected(_rotated_tensor(Vector3(1, 2, 0)), "rotated singular tensor")
	_check_rejected(Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO), "zero tensor")


func test_nonsymmetric_inertia_rejected() -> void:
	# Column x = (2,1,0): element (row1,col0)=1 but (row0,col1)=0.
	var skew: Basis = Basis(Vector3(2, 1, 0), Vector3(0, 2, 0), Vector3(0, 0, 2))
	check(skew.determinant() > 0.0, "premise: nonsymmetric example has positive determinant")
	_check_rejected(skew, "nonsymmetric tensor")
	check(SimMassProperties.validate_inertia(skew).contains("symmetric"), "reason names symmetry")


func test_positive_determinant_but_invalid_rejected() -> void:
	var a: Basis = Basis.from_scale(Vector3(-2, -3, 4))
	var b: Basis = _rotated_tensor(Vector3(1, -2, -3))
	check(a.determinant() > 0.0 and b.determinant() > 0.0, "premise: both have positive determinant")
	_check_rejected(a, "diag(-2,-3,4)")
	_check_rejected(b, "rotated indefinite tensor")


func test_small_scale_positive_definite_accepted() -> void:
	# 1 g body with 1-3 mm edges: I ~ 1e-12 kg m^2; far below the old absolute determinant threshold.
	var tiny: Basis = SimMassProperties.box_inertia(1.0e-3, Vector3(1.0e-3, 2.0e-3, 3.0e-3))
	check(tiny.determinant() < 1.0e-18, "premise: old absolute threshold would have rejected this")
	_check_accepted(tiny, "1 g box with mm edges")
	var mp = SimMassProperties.new(1.0e-3, tiny)
	check_basis_near(mp.local_inertia * mp.inverse_local_inertia, Basis.IDENTITY, 1e-5, "small-scale I * I^-1")
	_check_accepted(SimMassProperties.diagonal_inertia(Vector3(1.0e-30, 2.0e-30, 3.0e-30)), "diag ~1e-30")


func test_large_scale_positive_definite_accepted() -> void:
	_check_accepted(SimMassProperties.diagonal_inertia(Vector3(1.0e12, 2.0e12, 3.0e12)), "diag ~1e12")
	var mp = SimMassProperties.new(1.0e6, _rotated_tensor(Vector3(1.0e20, 2.0e20, 3.0e20)))
	check(mp.valid, "full tensor ~1e20 valid")
	check_basis_near(mp.local_inertia * mp.inverse_local_inertia, Basis.IDENTITY, 1e-5, "large-scale I * I^-1")


func test_validation_is_scale_invariant() -> void:
	for k in [1.0e-30, 1.0e-20, 1.0e-9, 1.0e-3, 1.0, 1.0e3, 1.0e9, 1.0e12]:
		var tag: String = " (scale %s)" % k
		check(SimMassProperties.validate_inertia(_rotated_tensor(Vector3(1, 2, 3)) * k) == "", "valid stays valid" + tag)
		check(SimMassProperties.validate_inertia(_rotated_tensor(Vector3(1, -2, -3)) * k) != "", "indefinite stays invalid" + tag)
		check(SimMassProperties.validate_inertia(Basis.from_scale(Vector3(1, 1, 0)) * k) != "", "singular stays invalid" + tag)
		check(SimMassProperties.validate_inertia(Basis.from_scale(Vector3(1, 1, 1.0e-7)) * k) != "", "ill-conditioned stays invalid" + tag)


func test_conditioning_boundary() -> void:
	# For diag(1,1,x) the reciprocal condition number is exactly x; threshold is 1e-5.
	check(SimMassProperties.validate_inertia(Basis.from_scale(Vector3(1, 1, 1.0e-4))) == "", "rcond 1e-4 accepted")
	check(SimMassProperties.validate_inertia(Basis.from_scale(Vector3(1, 1, 2.0e-5))) == "", "rcond 2e-5 accepted")
	check(SimMassProperties.validate_inertia(Basis.from_scale(Vector3(1, 1, 5.0e-6))) != "", "rcond 5e-6 rejected")
	check(SimMassProperties.validate_inertia(Basis.from_scale(Vector3(1, 1, 1.0e-7))) != "", "rcond 1e-7 rejected")
	# A slender rod (r=1 cm, L=1 m) is physically valid and well within the limit.
	var rod: Basis = Basis.from_scale(Vector3(0.5 * 0.01 * 0.01, 1.0 / 12.0, 1.0 / 12.0))
	check(SimMassProperties.validate_inertia(rod) == "", "slender rod accepted")


func test_nonfinite_tensor_rejected() -> void:
	_check_rejected(Basis.from_scale(Vector3(NAN, 1, 1)), "NaN diagonal")
	_check_rejected(Basis.from_scale(Vector3(INF, 1, 1)), "+inf diagonal")
	_check_rejected(Basis.from_scale(Vector3(1, -INF, 1)), "-inf diagonal")
	_check_rejected(Basis(Vector3(1, NAN, 0), Vector3(NAN, 1, 0), Vector3(0, 0, 1)), "NaN off-diagonal")
	_check_rejected(Basis(Vector3(1, 0, 0), Vector3(0, 1, INF), Vector3(0, INF, 1)), "inf off-diagonal")


func test_nonfinite_or_unrepresentable_mass_rejected() -> void:
	for m in [NAN, INF, -INF]:
		var mp = SimMassProperties.new(m, Basis.IDENTITY)
		check(not mp.valid, "non-finite mass %s rejected" % m)
		check_near(mp.inverse_mass, 0.0, 0.0, "no inverse mass for %s" % m)
	var sub = SimMassProperties.new(pow(2.0, -1070.0), Basis.IDENTITY)
	check(not sub.valid, "subnormal mass with infinite inverse rejected")


func test_static_convention_preserved() -> void:
	# Finite mass <= 0 is static and valid; the inertia argument is ignored (unchanged M0 convention).
	for m in [0.0, -1.0, -1.0e-30]:
		var mp = SimMassProperties.new(m, Basis.from_scale(Vector3(-1, -1, -1)))
		check(mp.valid and mp.error_message == "", "mass %s -> valid static" % m)
		check(mp.is_static(), "mass %s is static" % m)
		check_near(mp.inverse_mass, 0.0, 0.0, "inverse mass exactly zero for %s" % m)
	check(SimMassProperties.new().valid, "default-constructed properties are valid static")


func test_world_and_body_refuse_invalid_mass_properties() -> void:
	var bad = SimMassProperties.new(1.0, Basis.from_scale(Vector3(-1, -1, 1)))
	var w = make_world()
	check(w.add_body(SimRigidBody.new(bad)) == null, "add_body returns null for invalid mass properties")
	check(w.bodies.is_empty(), "invalid body was not added")
	var b = make_sphere(2.0, 0.5)
	check(not b.set_mass_properties(bad), "set_mass_properties refuses invalid input")
	check_near(b.mass_properties.mass, 2.0, 0.0, "previous mass properties retained")
	check(b.set_mass_properties(SimMassProperties.new(3.0, SimMassProperties.sphere_inertia(3.0, 0.5))), "valid replacement accepted")
	check_near(b.mass_properties.mass, 3.0, 0.0, "replacement applied")
	# Direct field assignment bypasses the setter; step() must still refuse to simulate it.
	w.add_body(b)
	b.mass_properties = bad
	check(not w.step(), "step refused when a body holds invalid mass properties")
	check(w.step_count == 0, "no step counted")
