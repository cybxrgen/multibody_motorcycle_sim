extends RefCounted
## Mass and inertia about the center of mass, expressed in the body frame.
## A finite mass <= 0 means "static": inverse mass and inverse inertia are exactly zero.
## The inertia tensor is a full 3x3 Basis, so non-diagonal tensors are supported.
##
## Validity (M0.1): bad input never yields a valid-looking body. `valid` is false and
## `error_message` explains why when the mass is non-finite/unrepresentable or a dynamic
## body's inertia fails validate_inertia(). An invalid instance has zero inverse mass and
## inertia, and is refused by SimWorld.add_body() and SimRigidBody.set_mass_properties().

## Symmetry tolerance, relative to the largest tensor element. Basis is float32
## (machine epsilon 2^-23 = 1.19e-7); a tensor assembled as R*D*R^T carries a few ulps of
## asymmetry, so ~84 ulps (1e-5) is allowed. Genuine asymmetry is orders of magnitude larger.
const SYMMETRY_REL_TOL: float = 1.0e-5
## Minimum reciprocal condition number (1-norm) of the scale-normalized tensor. The solver
## inverts the tensor in float32; below ~1e-5 fewer than ~2 digits of the inverse survive.
const MIN_RECIPROCAL_CONDITION: float = 1.0e-5

var mass: float = 0.0
var inverse_mass: float = 0.0
var local_inertia: Basis = Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)
var inverse_local_inertia: Basis = Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)
## False when construction rejected the input; see error_message.
var valid: bool = true
var error_message: String = ""


func _init(p_mass: float = 0.0, p_local_inertia: Basis = Basis.IDENTITY) -> void:
	if not is_finite(p_mass):
		_reject("mass must be finite (got %s)" % p_mass)
		return
	if p_mass <= 0.0:
		return  # static body; the inertia argument is ignored
	var reason: String = validate_inertia(p_local_inertia)
	if reason != "":
		_reject("dynamic body has invalid inertia: " + reason)
		return
	var inv_m: float = 1.0 / p_mass
	if not is_finite(inv_m):
		_reject("mass %s is too small: inverse mass is not finite" % p_mass)
		return
	var inv_i: Basis = _inverse_float64(p_local_inertia)
	if not (is_finite(inv_i.x.x) and is_finite(inv_i.y.y) and is_finite(inv_i.z.z)) or inv_i.x.x <= 0.0 or inv_i.y.y <= 0.0 or inv_i.z.z <= 0.0:
		_reject("inertia inverse is not representable in float32 (tensor magnitude out of range)")
		return
	mass = p_mass
	inverse_mass = inv_m
	local_inertia = p_local_inertia
	inverse_local_inertia = inv_i


## Inverse of a (validated, symmetric) tensor computed in float64 on the scale-normalized tensor,
## then stored as float32. Basis.inverse() works in float32 and overflows once I^3 or I^2 exceeds
## ~3.4e38 (tensors above ~1e12), although the true inverse is perfectly representable.
static func _inverse_float64(inertia: Basis) -> Basis:
	var scale: float = 0.0
	for v in [inertia.x.x, inertia.x.y, inertia.x.z, inertia.y.x, inertia.y.y, inertia.y.z, inertia.z.x, inertia.z.y, inertia.z.z]:
		scale = maxf(scale, absf(v))
	var n00: float = inertia.x.x / scale
	var n11: float = inertia.y.y / scale
	var n22: float = inertia.z.z / scale
	var n01: float = 0.5 * (inertia.y.x + inertia.x.y) / scale
	var n02: float = 0.5 * (inertia.z.x + inertia.x.z) / scale
	var n12: float = 0.5 * (inertia.z.y + inertia.y.z) / scale
	var c00: float = n11 * n22 - n12 * n12
	var c01: float = n02 * n12 - n01 * n22
	var c02: float = n01 * n12 - n02 * n11
	var c11: float = n00 * n22 - n02 * n02
	var c12: float = n01 * n02 - n00 * n12
	var c22: float = n00 * n11 - n01 * n01
	var det: float = n00 * c00 + n01 * c01 + n02 * c02
	var f: float = 1.0 / (det * scale)
	return Basis(Vector3(c00, c01, c02) * f, Vector3(c01, c11, c12) * f, Vector3(c02, c12, c22) * f)


func _reject(reason: String) -> void:
	valid = false
	error_message = reason
	mass = 0.0
	inverse_mass = 0.0
	local_inertia = Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)
	inverse_local_inertia = Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)
	push_error("SimMassProperties: " + reason)


## Returns "" if `inertia` is a valid dynamic-body inertia tensor, otherwise the reason.
## Checks, in order: finite; symmetric within SYMMETRY_REL_TOL of the largest element;
## positive definite (Cholesky with strictly positive pivots); well conditioned
## (reciprocal 1-norm condition number >= MIN_RECIPROCAL_CONDITION).
## The tensor is first normalized by its largest |element| and all arithmetic is float64,
## so the verdict is independent of the physical scale of the tensor (tested over 1e-30..1e12).
static func validate_inertia(inertia: Basis) -> String:
	# Element (row r, col c) of a Godot Basis is basis[c][r].
	var a00: float = inertia.x.x
	var a10: float = inertia.x.y
	var a20: float = inertia.x.z
	var a01: float = inertia.y.x
	var a11: float = inertia.y.y
	var a21: float = inertia.y.z
	var a02: float = inertia.z.x
	var a12: float = inertia.z.y
	var a22: float = inertia.z.z
	var scale: float = 0.0
	for v in [a00, a10, a20, a01, a11, a21, a02, a12, a22]:
		if not is_finite(v):
			return "tensor contains non-finite values"
		scale = maxf(scale, absf(v))
	if scale <= 0.0:
		return "tensor is zero"
	var tol: float = SYMMETRY_REL_TOL * scale
	if absf(a01 - a10) > tol or absf(a02 - a20) > tol or absf(a12 - a21) > tol:
		return "tensor is not symmetric (relative tolerance %s)" % SYMMETRY_REL_TOL
	# Normalized, symmetrized entries.
	var n00: float = a00 / scale
	var n11: float = a11 / scale
	var n22: float = a22 / scale
	var n01: float = 0.5 * (a01 + a10) / scale
	var n02: float = 0.5 * (a02 + a20) / scale
	var n12: float = 0.5 * (a12 + a21) / scale
	# Cholesky A = L L^T; every pivot must be strictly positive.
	if n00 <= 0.0:
		return "tensor is not positive definite (non-positive leading moment)"
	var l00: float = sqrt(n00)
	var l10: float = n01 / l00
	var l20: float = n02 / l00
	var d1: float = n11 - l10 * l10
	if d1 <= 0.0:
		return "tensor is not positive definite (non-positive pivot 2)"
	var l21: float = (n12 - l20 * l10) / sqrt(d1)
	var d2: float = n22 - l20 * l20 - l21 * l21
	if d2 <= 0.0:
		return "tensor is not positive definite (non-positive pivot 3)"
	# Reciprocal condition number from the cofactor inverse (det = n00 * d1 * d2 > 0 here).
	var det: float = n00 * d1 * d2
	var c00: float = n11 * n22 - n12 * n12
	var c01: float = n02 * n12 - n01 * n22
	var c02: float = n01 * n12 - n02 * n11
	var c11: float = n00 * n22 - n02 * n02
	var c12: float = n01 * n02 - n00 * n12
	var c22: float = n00 * n11 - n01 * n01
	var norm_a: float = maxf(absf(n00) + absf(n01) + absf(n02), maxf(absf(n01) + absf(n11) + absf(n12), absf(n02) + absf(n12) + absf(n22)))
	var norm_inv: float = maxf(absf(c00) + absf(c01) + absf(c02), maxf(absf(c01) + absf(c11) + absf(c12), absf(c02) + absf(c12) + absf(c22))) / det
	var rcond: float = 1.0 / (norm_a * norm_inv)
	if not (rcond >= MIN_RECIPROCAL_CONDITION):
		return "tensor is numerically singular (reciprocal condition %s < %s)" % [rcond, MIN_RECIPROCAL_CONDITION]
	return ""


func is_static() -> bool:
	return inverse_mass == 0.0


## I_world = R * I_local * R^T, where R is the local->world rotation.
func world_inertia(rotation: Basis) -> Basis:
	return rotation * local_inertia * rotation.transposed()


## I_world^-1 = R * I_local^-1 * R^T
func world_inverse_inertia(rotation: Basis) -> Basis:
	return rotation * inverse_local_inertia * rotation.transposed()


## Diagonal tensor from principal moments (body frame axes are the principal axes).
static func diagonal_inertia(principal_moments: Vector3) -> Basis:
	return Basis.from_scale(principal_moments)


## Solid box with full side lengths `size` (x, y, z).
static func box_inertia(p_mass: float, size: Vector3) -> Basis:
	var k: float = p_mass / 12.0
	return Basis.from_scale(Vector3(
		k * (size.y * size.y + size.z * size.z),
		k * (size.x * size.x + size.z * size.z),
		k * (size.x * size.x + size.y * size.y)))


## Solid sphere.
static func sphere_inertia(p_mass: float, radius: float) -> Basis:
	var i: float = 0.4 * p_mass * radius * radius
	return Basis.from_scale(Vector3(i, i, i))
