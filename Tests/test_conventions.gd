extends "res://Tests/sim_test_case.gd"
## Verifies the Godot math conventions that PHYSICS_CONVENTIONS.md relies on.
## If any of these fail on a new Godot version, the whole physics core must be
## re-audited before trusting other test results.


func test_right_handed_axes() -> void:
	check_vec3_near(Vector3.RIGHT.cross(Vector3.UP), Vector3.BACK, 1e-12, "+X x +Y = +Z (right-handed)")
	check_vec3_near(Vector3.FORWARD, Vector3(0, 0, -1), 1e-12, "forward is -Z")


func test_positive_rotation_is_counter_clockwise() -> void:
	# +90 deg about +Y (seen from +Y looking down) turns forward (-Z) toward -X.
	var q: Quaternion = Quaternion(Vector3.UP, PI / 2.0)
	check_vec3_near(q * Vector3.FORWARD, Vector3(-1, 0, 0), 1e-6, "Ry(+90) maps -Z to -X")
	# +90 deg about +Z maps +X to +Y.
	var qz: Quaternion = Quaternion(Vector3.BACK, PI / 2.0)
	check_vec3_near(qz * Vector3.RIGHT, Vector3.UP, 1e-6, "Rz(+90) maps +X to +Y")


func test_quaternion_product_order() -> void:
	# (a * b) rotates by b first, then a (Hamilton product, active rotation).
	var a: Quaternion = Quaternion(Vector3.UP, 0.7)
	var b: Quaternion = Quaternion(Vector3.RIGHT, -0.4)
	var v: Vector3 = Vector3(0.3, -1.2, 2.0)
	check_vec3_near((a * b) * v, a * (b * v), 1e-6, "(a*b)*v == a*(b*v)")
	var differs: bool = ((a * b) * v - (b * a) * v).length() > 1e-3
	check(differs, "quaternion product is not commutative for these rotations")


func test_basis_matches_quaternion() -> void:
	var q: Quaternion = Quaternion(Vector3(1, 2, 3).normalized(), 1.1)
	var v: Vector3 = Vector3(-0.5, 0.25, 2.0)
	check_vec3_near(Basis(q) * v, q * v, 1e-6, "Basis(q)*v == q*v")


func test_basis_product_order() -> void:
	var a: Basis = Basis(Quaternion(Vector3.UP, 0.7))
	var b: Basis = Basis(Quaternion(Vector3.RIGHT, -0.4))
	var v: Vector3 = Vector3(0.3, -1.2, 2.0)
	check_vec3_near((a * b) * v, a * (b * v), 1e-6, "(A*B)*v == A*(B*v)")


func test_basis_from_scale_is_diagonal() -> void:
	var d: Basis = Basis.from_scale(Vector3(2, 3, 5))
	check_vec3_near(d * Vector3(1, 1, 1), Vector3(2, 3, 5), 1e-12, "from_scale is diagonal")
	check_vec3_near(d * Vector3.UP, Vector3(0, 3, 0), 1e-12, "from_scale does not mix axes")


func test_basis_inverse_and_transpose() -> void:
	var r: Basis = Basis(Quaternion(Vector3(0.2, 1.0, -0.4).normalized(), 0.9))
	check_basis_near(r * r.transposed(), Basis.IDENTITY, 1e-6, "R * R^T = I")
	var m: Basis = r * Basis.from_scale(Vector3(2, 3, 5)) * r.transposed()
	check_basis_near(m * m.inverse(), Basis.IDENTITY, 1e-6, "M * M^-1 = I")
