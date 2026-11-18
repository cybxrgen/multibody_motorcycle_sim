extends RefCounted
## One scalar constraint row: J = [linear_a, angular_a, linear_b, angular_b] (world frame).
## Cdot = J . v. A scalar impulse lambda applies dv = M^-1 Jv lambda, domega = I_w^-1 Jw lambda.
## See PHYSICS_CONVENTIONS.md ("Constraint Jacobian conventions").

const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")

var body_a: SimRigidBody
var body_b: SimRigidBody

var linear_a: Vector3 = Vector3.ZERO
var angular_a: Vector3 = Vector3.ZERO
var linear_b: Vector3 = Vector3.ZERO
var angular_b: Vector3 = Vector3.ZERO

## Velocity-level bias added to Cdot (e.g. Baumgarte beta/dt * C).
var bias: float = 0.0
## Regularization gamma >= 0 (soft constraint); 0 = rigid.
var softness: float = 0.0
## 1 / (K + softness), computed by prepare(). 0 for degenerate rows.
var effective_mass: float = 0.0
var accumulated_impulse: float = 0.0
var min_impulse: float = -INF
var max_impulse: float = INF


func _init(p_body_a: SimRigidBody, p_body_b: SimRigidBody) -> void:
	body_a = p_body_a
	body_b = p_body_b


func compute_velocity() -> float:
	return linear_a.dot(body_a.linear_velocity) + angular_a.dot(body_a.angular_velocity) \
		+ linear_b.dot(body_b.linear_velocity) + angular_b.dot(body_b.angular_velocity)


## K = J M^-1 J^T using the bodies' current world inverse inertia.
func compute_k() -> float:
	return body_a.inverse_mass * linear_a.dot(linear_a) \
		+ angular_a.dot(body_a.inverse_inertia_world * angular_a) \
		+ body_b.inverse_mass * linear_b.dot(linear_b) \
		+ angular_b.dot(body_b.inverse_inertia_world * angular_b)


func prepare(min_denominator: float) -> void:
	var denominator: float = compute_k() + softness
	effective_mass = 1.0 / denominator if denominator > min_denominator else 0.0


## Applies carried-over impulse (warm start) or resets it.
func warm_start(use_warm_start: bool, factor: float) -> void:
	if use_warm_start:
		accumulated_impulse = clampf(accumulated_impulse * factor, min_impulse, max_impulse)
		apply_impulse(accumulated_impulse)
	else:
		accumulated_impulse = 0.0


func apply_impulse(magnitude: float) -> void:
	body_a.linear_velocity += linear_a * (body_a.inverse_mass * magnitude)
	body_a.angular_velocity += body_a.inverse_inertia_world * (angular_a * magnitude)
	body_b.linear_velocity += linear_b * (body_b.inverse_mass * magnitude)
	body_b.angular_velocity += body_b.inverse_inertia_world * (angular_b * magnitude)


## One projected Gauss-Seidel update. The accumulated impulse is clamped, only the clamped delta is applied.
func solve() -> void:
	var cdot: float = compute_velocity()
	var delta: float = -effective_mass * (cdot + bias + softness * accumulated_impulse)
	var new_total: float = clampf(accumulated_impulse + delta, min_impulse, max_impulse)
	delta = new_total - accumulated_impulse
	accumulated_impulse = new_total
	apply_impulse(delta)
