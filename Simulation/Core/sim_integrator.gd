extends RefCounted
## Semi-implicit (symplectic) Euler integration of rigid bodies.
##
## Velocity update (uses accumulated force/torque and the CURRENT orientation):
##   v'     = v + dt * F / m
##   omega' = omega + dt * I_w^-1 (tau - omega x (I_w omega))        <- explicit gyroscopic term
## Position update (uses the NEW velocities, which is what makes it semi-implicit):
##   x' = x + dt * (v' + v_split)
##   q' = exp(0.5 * dt * (omega' + omega_split)) * q     (world-frame rotation increment, exact for constant omega)
## v_split / omega_split are the split-impulse (positional correction) velocities of this step; they move the pose but are
## never added to the stored physical velocities (PHYSICS_CONVENTIONS.md, "Split-impulse stabilization"). Both are zero for
## a body no split impulse touched, which reproduces the previous update exactly.
##
## Assumptions / limitations (also in ARCHITECTURE.md):
##  - First-order accurate. The explicit gyroscopic term does not conserve kinetic energy or angular
##    momentum exactly for asymmetric bodies spinning fast relative to 1/dt; error is O(dt).
##    An implicit gyroscopic solve is deferred until tests show it is needed.
##  - Orientation is re-normalized after every update (guards floating-point drift only).

const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")


static func integrate_velocities(body: SimRigidBody, dt: float) -> void:
	if body.is_static():
		return
	body.linear_velocity += body.force_accumulator * (body.inverse_mass * dt)
	var w: Vector3 = body.angular_velocity
	var gyroscopic: Vector3 = w.cross(body.inertia_world * w)
	body.angular_velocity += body.inverse_inertia_world * ((body.torque_accumulator - gyroscopic) * dt)


static func integrate_positions(body: SimRigidBody, dt: float) -> void:
	if body.is_static():
		return
	body.position += (body.linear_velocity + body.split_linear_velocity) * dt
	var w: Vector3 = body.angular_velocity + body.split_angular_velocity
	var speed: float = w.length()
	if speed * dt > 1.0e-12:
		var delta: Quaternion = Quaternion(w / speed, speed * dt)
		body.orientation = delta * body.orientation  # setter normalizes and refreshes world inertia
