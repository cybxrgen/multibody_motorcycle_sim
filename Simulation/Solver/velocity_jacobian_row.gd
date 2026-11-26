extends "res://Simulation/Solver/constraint_row.gd"
## A ConstraintRow whose VELOCITY-SIDE angular Jacobian differs from its IMPULSE-SIDE (wrench) angular Jacobian.
## Opt-in: no existing constraint uses it, and ConstraintRow itself is unchanged, so every other row keeps its behaviour. The
## tyre patch (TirePatchNormalSupport) is its only user. Equations and rationale: PHYSICS_CONVENTIONS.md ("Velocity-side and
## impulse-side Jacobians").
##
## Two different maps share one scalar row:
##   impulse side (physical wrench, inherited fields linear_a/b, angular_a/b):  a scalar impulse lambda changes the bodies by
##       dv = M^-1 J_lin lambda,  domega = I_w^-1 J_ang lambda          (warm start, solve(), accumulated impulse)
##   velocity side (what the row measures and how stiff it is, fields linear_a/b and velocity_angular_a/b = J_v_ang):
##       Cdot   = J_lin . v + J_v_ang . omega                           (compute_velocity)
##       K      = J_lin M^-1 J_lin^T + J_v_ang I_w^-1 J_v_ang           (compute_k; effective mass = 1/(K + softness))
## The linear part is shared. A velocity-side angular Jacobian that differs from the wrench is not a mistake: the wrench is the real
## moment of the impulse (a force at the sample point), while Cdot must be a rate that the model defines to exclude a rigid
## motion that does not change the constrained quantity (for the tyre: the spin about the axle, which only slides the tread
## along itself).
## The effective mass uses J_v on both sides, J_v M^-1 J_v^T, as specified for the tyre. The exact response of Cdot to a unit impulse
## is J_v M^-1 J_i^T; the two agree whenever J_v I_w^-1 (J_i_ang - J_v_ang) = 0, which holds for the tyre because the axle u is a
## principal axis of the wheel inertia and J_v_ang is perpendicular to u (documented limitation otherwise).
## Split channel: the split impulse is a POSE correction, not a physical impulse, so it uses the velocity-side Jacobian on both
## sides (measure and apply). That is the gradient of the constrained quantity with respect to the pose: a rotation about the
## axle does not change the surface of revolution and the correction therefore never spins the wheel. The physical wrench never
## enters the split velocities, so the physical/split separation of the solver is preserved.
## Nothing here is cached: Cdot is evaluated from the CURRENT velocities in every iteration, so velocity changes made earlier in the
## same step (including the spin change caused by this row's own wrench) enter exactly through J_v.

## Velocity-side angular Jacobians of body A and body B. Owners must set them with every Jacobian update (they are not derived from
## angular_a/b); a row of this class that never receives them measures no angular velocity.
var velocity_angular_a: Vector3 = Vector3.ZERO
var velocity_angular_b: Vector3 = Vector3.ZERO


func compute_velocity() -> float:
	return linear_a.dot(body_a.linear_velocity) + velocity_angular_a.dot(body_a.angular_velocity) \
		+ linear_b.dot(body_b.linear_velocity) + velocity_angular_b.dot(body_b.angular_velocity)


## K = J_v M^-1 J_v^T (current world inverse inertia).
func compute_k() -> float:
	return body_a.inverse_mass * linear_a.dot(linear_a) \
		+ velocity_angular_a.dot(body_a.inverse_inertia_world * velocity_angular_a) \
		+ body_b.inverse_mass * linear_b.dot(linear_b) \
		+ velocity_angular_b.dot(body_b.inverse_inertia_world * velocity_angular_b)


func compute_split_velocity() -> float:
	return linear_a.dot(body_a.split_linear_velocity) + velocity_angular_a.dot(body_a.split_angular_velocity) \
		+ linear_b.dot(body_b.split_linear_velocity) + velocity_angular_b.dot(body_b.split_angular_velocity)


## Pose correction along the velocity-side Jacobian (never the physical wrench); changes only the split velocities.
func apply_split_impulse(magnitude: float) -> void:
	body_a.split_linear_velocity += linear_a * (body_a.inverse_mass * magnitude)
	body_a.split_angular_velocity += body_a.inverse_inertia_world * (velocity_angular_a * magnitude)
	body_b.split_linear_velocity += linear_b * (body_b.inverse_mass * magnitude)
	body_b.split_angular_velocity += body_b.inverse_inertia_world * (velocity_angular_b * magnitude)
