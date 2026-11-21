extends RefCounted
## Semi-implicit (symplectic) Euler integration of rigid bodies with an implicit-midpoint gyroscopic update.
##
## Velocity update (uses accumulated force/torque and the CURRENT orientation):
##   v'      = v + dt * F / m
##   omega*  = omega + dt * I_w^-1 * tau                         (torque kick, unchanged from the explicit scheme)
##   omega'  = omega* + R * I^-1 * (L_b' - L_b)                  (gyroscopic part, implicit midpoint, see below)
## Gyroscopic part. In the body frame (R = local->world rotation, I = local inertia) torque-free rotation obeys Euler's
## equations for the body-frame angular momentum L_b = I R^T omega:
##   dL_b/dt = L_b x I^-1 L_b.
## One step is the implicit midpoint rule, L_b' = L_b + dt * Lm x I^-1 Lm with Lm = (L_b + L_b') / 2, solved by Newton iteration
## in float64 (3x3 system, quadratic convergence, typically 3-4 iterations). Both quadratic invariants of the free rotation,
## |L_b|^2 and T = 1/2 L_b . I^-1 L_b, are conserved EXACTLY by the midpoint rule (up to the solve tolerance and float32 storage of
## the result), because (Lm x y) . Lm = 0 and (Lm x I^-1 Lm) . I^-1 Lm = 0 for the symmetric positive definite I^-1. Since
## T = 1/2 L . I^-1 L lies between |L|^2 / (2 I_max) and |L|^2 / (2 I_min), |omega| cannot grow without bound for any dt. The former
## explicit update raised both invariants every step (second-order term +dt^2/2 c . I^-1 c with c = omega x I omega, never
## negative), which feeds on itself and ends in overflow (PHYSICS_CONVENTIONS.md, "Rotational dynamics").
## Position update (uses the NEW velocities, which is what makes it semi-implicit):
##   x' = x + dt * (v' + v_split)
##   q' = exp(0.5 * dt * (omega' + omega_split)) * q     (world-frame rotation increment, exact for constant omega)
## v_split / omega_split are the split-impulse (positional correction) velocities of this step; they move the pose but are
## never added to the stored physical velocities (PHYSICS_CONVENTIONS.md, "Split-impulse stabilization"). Both are zero for
## a body no split impulse touched, which reproduces the previous update exactly.
##
## Assumptions / limitations (also in ARCHITECTURE.md and PHYSICS_CONVENTIONS.md):
##  - First-order accurate overall (torque is a first-order kick; the orientation is advanced with the end-of-step omega, not the
##    midpoint omega). The magnitude of the spin angular momentum and the rotational kinetic energy of a torque-free body are
##    conserved, but the world-frame DIRECTION of L is not: it errs by O(dt), bounded (measured, see PROJECT_STATE.md).
##  - The Newton solve is bounded; if it does not converge (|omega| dt far outside the regime of interest) integrate_velocities()
##    returns false and leaves the angular velocity untouched, so SimWorld can report the failure instead of integrating garbage.
##  - Orientation is re-normalized after every update (guards floating-point drift only).

const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")

## Newton iteration cap and convergence criterion of the gyroscopic solve: stop when the Newton step is below
## GYRO_TOLERANCE * |L_b|. 1e-12 is ~4 orders below the float32 resolution of the stored result (6e-8) and ~4 orders above the
## float64 rounding floor of the iteration; the quadratic convergence needs 3-5 iterations for |omega| dt <= 0.5.
const GYRO_MAX_ITERATIONS: int = 24
const GYRO_TOLERANCE: float = 1.0e-12


## Returns false (angular velocity untouched, linear velocity already updated) when the implicit gyroscopic solve fails
## (non-finite input, singular Newton matrix or no convergence); true otherwise or for a static body.
static func integrate_velocities(body: SimRigidBody, dt: float) -> bool:
	if body.is_static():
		return true
	body.linear_velocity += body.force_accumulator * (body.inverse_mass * dt)
	var w: Vector3 = body.angular_velocity + body.inverse_inertia_world * (body.torque_accumulator * dt)
	var rotation: Basis = Basis(body.orientation)
	var l_body: Vector3 = body.local_inertia * (rotation.transposed() * w)
	var delta_l: PackedFloat64Array = solve_free_rotation(l_body, body.inverse_local_inertia, dt)
	if delta_l.is_empty():
		return false
	# omega change = R I^-1 (L' - L): only the (small) increment is transformed, so a body that does not change (zero angular
	# velocity, principal-axis or isotropic spin) keeps its angular velocity bit for bit and no frame-transform rounding accumulates.
	var dw_body: Vector3 = body.inverse_local_inertia * Vector3(delta_l[0], delta_l[1], delta_l[2])
	body.angular_velocity = w + rotation * dw_body
	return true


## Implicit-midpoint step of Euler's equations dL/dt = L x W L (W = inverse inertia, symmetric positive definite) in the body
## frame. Returns the increment L' - L as float64 (3 values), or an empty array if the input is not finite, the Newton matrix is
## singular or the iteration did not converge. Exposed for tests (invariants, convergence); has no side effects.
static func solve_free_rotation(l: Vector3, inv_inertia: Basis, dt: float) -> PackedFloat64Array:
	var lx: float = l.x
	var ly: float = l.y
	var lz: float = l.z
	var l2: float = lx * lx + ly * ly + lz * lz
	if not is_finite(l2) or not is_finite(dt):
		return PackedFloat64Array()
	if l2 == 0.0:
		return PackedFloat64Array([0.0, 0.0, 0.0])
	# Symmetrized inverse inertia entries (the validated tensor is symmetric to ~1e-5; symmetric W is what the invariants need).
	var w00: float = inv_inertia.x.x
	var w11: float = inv_inertia.y.y
	var w22: float = inv_inertia.z.z
	var w01: float = 0.5 * (inv_inertia.y.x + inv_inertia.x.y)
	var w02: float = 0.5 * (inv_inertia.z.x + inv_inertia.x.z)
	var w12: float = 0.5 * (inv_inertia.z.y + inv_inertia.y.z)
	var tol2: float = GYRO_TOLERANCE * GYRO_TOLERANCE * l2
	var h: float = 0.5 * dt
	var x0: float = lx
	var x1: float = ly
	var x2: float = lz
	for _it in GYRO_MAX_ITERATIONS:
		var m0: float = 0.5 * (lx + x0)
		var m1: float = 0.5 * (ly + x1)
		var m2: float = 0.5 * (lz + x2)
		var y0: float = w00 * m0 + w01 * m1 + w02 * m2
		var y1: float = w01 * m0 + w11 * m1 + w12 * m2
		var y2: float = w02 * m0 + w12 * m1 + w22 * m2
		# Residual f = L + dt (m x y) - x.
		var f0: float = lx + dt * (m1 * y2 - m2 * y1) - x0
		var f1: float = ly + dt * (m2 * y0 - m0 * y2) - x1
		var f2: float = lz + dt * (m0 * y1 - m1 * y0) - x2
		# Jacobian J = (dt/2) ([m]x W - [y]x) - I, with [a]x b = a x b.
		var a00: float = -m2 * w01 + m1 * w02
		var a01: float = -m2 * w11 + m1 * w12 + y2
		var a02: float = -m2 * w12 + m1 * w22 - y1
		var a10: float = m2 * w00 - m0 * w02 - y2
		var a11: float = m2 * w01 - m0 * w12
		var a12: float = m2 * w02 - m0 * w22 + y0
		var a20: float = -m1 * w00 + m0 * w01 + y1
		var a21: float = -m1 * w01 + m0 * w11 - y0
		var a22: float = -m1 * w02 + m0 * w12
		var j00: float = h * a00 - 1.0
		var j01: float = h * a01
		var j02: float = h * a02
		var j10: float = h * a10
		var j11: float = h * a11 - 1.0
		var j12: float = h * a12
		var j20: float = h * a20
		var j21: float = h * a21
		var j22: float = h * a22 - 1.0
		# Solve J d = -f by cofactors.
		var c00: float = j11 * j22 - j12 * j21
		var c01: float = j12 * j20 - j10 * j22
		var c02: float = j10 * j21 - j11 * j20
		var det: float = j00 * c00 + j01 * c01 + j02 * c02
		if not is_finite(det) or det == 0.0:
			return PackedFloat64Array()
		var inv_det: float = 1.0 / det
		var r0: float = -f0
		var r1: float = -f1
		var r2: float = -f2
		var d0: float = (r0 * c00 + r1 * (j02 * j21 - j01 * j22) + r2 * (j01 * j12 - j02 * j11)) * inv_det
		var d1: float = (r0 * c01 + r1 * (j00 * j22 - j02 * j20) + r2 * (j02 * j10 - j00 * j12)) * inv_det
		var d2: float = (r0 * c02 + r1 * (j01 * j20 - j00 * j21) + r2 * (j00 * j11 - j01 * j10)) * inv_det
		x0 += d0
		x1 += d1
		x2 += d2
		var step2: float = d0 * d0 + d1 * d1 + d2 * d2
		if not is_finite(step2):
			return PackedFloat64Array()
		if step2 <= tol2:
			return PackedFloat64Array([x0 - lx, x1 - ly, x2 - lz])
	return PackedFloat64Array()


static func integrate_positions(body: SimRigidBody, dt: float) -> void:
	if body.is_static():
		return
	body.position += (body.linear_velocity + body.split_linear_velocity) * dt
	var w: Vector3 = body.angular_velocity + body.split_angular_velocity
	var speed: float = w.length()
	# A non-finite angular velocity must not reach the Quaternion constructor (its engine error hides the real failure):
	# the rotation is skipped and SimWorld reports the non-finite state with the body identity. Not a repair.
	if is_finite(speed) and speed * dt > 1.0e-12:
		var delta: Quaternion = Quaternion(w / speed, speed * dt)
		body.orientation = delta * body.orientation  # setter normalizes and refreshes world inertia
