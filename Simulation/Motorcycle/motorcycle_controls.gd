extends "res://Simulation/Core/sim_force_provider.gd"
## Basic direct controls of the motorcycle (M2.5.2): normalized throttle, brake and steering inputs turned into physics every FIXED
## step. A direct input-to-physics interface, NOT a powertrain, engine, brake system, rider or controller (no PID, no countersteering, no
## balancing). It is a generic SimForceProvider: the world calls apply_forces() once per fixed step (never once per render frame), it only
## uses the bodies' torque API and the existing steering hinge motor, it never writes poses or velocities, and the physics code never
## reads the keyboard (the scene translates keys into set_inputs()).
##
## Inputs (validated, never clamped): throttle in [0, 1], brake in [0, 1], steering in [-1, 1] (+1 = steer LEFT: positive steering-hinge
## angle, see PHYSICS_CONVENTIONS.md). set_inputs() refuses an invalid set (finite and in range) and keeps the old values; validate()
## reports invalid values written directly, which makes SimWorld refuse to step.
##
## ---- Throttle: axle torque on the rear wheel ----
## tau = throttle * max_drive_torque about the rear axle u (the wheel's own +X): torque -tau u on the wheel (forward rolling is a NEGATIVE
## rate about +X), and the equal and opposite +tau u on the swingarm that carries the axle (an internal pair, no net external torque).
## No force is ever applied to the chassis or the wheel: the forward force on the motorcycle exists only because the tyre model turns the
## resulting slip into traction. Zero throttle applies exactly zero torque.
##
## ---- Brake: torque opposing the wheel's rotation, both wheels ----
## For each wheel, w_rel = (w_wheel - w_carrier) . u is the rate of the axle hinge (carrier = fork slider / swingarm). The brake torque
## magnitude is  T = min(brake * max_brake_torque_wheel, STOP_FRACTION I_red |w_rel| / dt)  with I_red = 1 / (1 / I_wheel_axle + 1 / I_carrier_axle) the
## reduced inertia about the axle; the torque is  -sign(w_rel) T u  on the wheel and the opposite on the carrier. Hence the brake
##  * opposes the actual spin (sign of w_rel), is zero at w_rel = 0 (cannot reverse a stationary wheel),
##  * can at most remove STOP_FRACTION (1/2) of the relative rotation per step (the cap), so it decays to rest and never past it,
##  * has power  -T |w_rel| <= 0: it never injects energy.
## The friction limit of the braked wheel is the tyre model's (combined slip ellipse); the brake never applies a chassis force.
##
## ---- Throttle and brake together ----
## Both torques are applied (their sum acts on the rear wheel). There is no priority and no interlock: the brake caps at the spin that
## exists at the start of the step, the drive adds its own torque.
##
## ---- Steering: the steering hinge motor ----
## target rate = steering * max_steering_rate (rad/s) with torque cap max_steering_torque (N m), set on the existing steering HingeJoint's
## motor at every fixed step. Releasing the input (steering = 0) keeps the motor enabled with target rate 0: the steering rotation is
## actively held at rest (a rate lock limited by the torque cap; it is not an angle controller and not a self-centering). The hinge's
## own angle limits still apply (the motor target is clamped into the rate the limit rows admit, the motor never pushes through a stop)
## and the motor is part of the hinge constraint, so the hinge itself is never bypassed.
## Disabling the provider (enabled = false) or removing it leaves the last motor command in the hinge: use release_motor().

## Fraction of the one-step stopping torque the brake may use. The two-body reduced inertia is only a prediction (the hinge and suspension
## couple the carrier to the rest of the machine; measured overshoot with the full cap: ~1 % of the initial spin), so the cap leaves a factor 2
## of margin: the relative rotation decays geometrically and never changes sign.
const STOP_FRACTION: float = 0.5

var throttle: float = 0.0
var brake: float = 0.0
var steering: float = 0.0

var spec = null
var chassis = null
var steering_body = null
var fork_slider = null
var swingarm = null
var front_wheel = null
var rear_wheel = null
var steering_joint = null

## Torque / command applied in the last evaluated step (read-only diagnostics).
var last_drive_torque: float = 0.0
var last_front_brake_torque: float = 0.0
var last_rear_brake_torque: float = 0.0


## Builds the controls for an assembly (a MotorcycleAssembly) and a control spec. Returns null (with an error) for invalid input.
static func create(assembly, p_spec):
	if assembly == null or p_spec == null:
		push_error("MotorcycleControls.create: assembly and spec must not be null.")
		return null
	var errors: PackedStringArray = p_spec.validate()
	if not errors.is_empty():
		push_error("MotorcycleControls.create: %s" % "; ".join(errors))
		return null
	var controls = (load("res://Simulation/Motorcycle/motorcycle_controls.gd")).new()
	controls.spec = p_spec.duplicate_spec()
	controls.chassis = assembly.chassis
	controls.steering_body = assembly.steering
	controls.fork_slider = assembly.fork_slider
	controls.swingarm = assembly.swingarm
	controls.front_wheel = assembly.front_wheel
	controls.rear_wheel = assembly.rear_wheel
	controls.steering_joint = assembly.steering_joint
	return controls


func involved_bodies() -> Array:
	return [chassis, steering_body, fork_slider, swingarm, front_wheel, rear_wheel]


## Validated assignment of all three inputs; false (nothing changed) when any value is not finite or out of range.
func set_inputs(p_throttle: float, p_brake: float, p_steering: float) -> bool:
	if not input_errors(p_throttle, p_brake, p_steering).is_empty():
		return false
	throttle = p_throttle
	brake = p_brake
	steering = p_steering
	return true


static func input_errors(p_throttle: float, p_brake: float, p_steering: float) -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if not (is_finite(p_throttle) and p_throttle >= 0.0 and p_throttle <= 1.0):
		errors.append("throttle must be finite and in [0, 1] (got %s)" % p_throttle)
	if not (is_finite(p_brake) and p_brake >= 0.0 and p_brake <= 1.0):
		errors.append("brake must be finite and in [0, 1] (got %s)" % p_brake)
	if not (is_finite(p_steering) and p_steering >= -1.0 and p_steering <= 1.0):
		errors.append("steering must be finite and in [-1, 1] (got %s)" % p_steering)
	return errors


func validate() -> PackedStringArray:
	var errors: PackedStringArray = input_errors(throttle, brake, steering)
	if spec == null:
		errors.append("controls: spec is null")
	else:
		errors.append_array(spec.validate())
	return errors


## Zero inputs (the neutral command).
func release_inputs() -> void:
	throttle = 0.0
	brake = 0.0
	steering = 0.0


## Disables the steering motor again (target 0, torque 0, motor off). The steering then moves freely under the physics.
func release_motor() -> void:
	if steering_joint != null:
		steering_joint.motor_enabled = false
		steering_joint.motor_target_speed = 0.0
		steering_joint.motor_max_torque = 0.0


func apply_forces(dt: float, _config) -> void:
	last_drive_torque = 0.0
	last_front_brake_torque = 0.0
	last_rear_brake_torque = 0.0
	# Drive: axle torque on the rear wheel, reaction on the swingarm.
	var axle: Vector3 = rear_wheel.orientation * Vector3.RIGHT
	last_drive_torque = throttle * spec.max_drive_torque
	if last_drive_torque > 0.0:
		rear_wheel.apply_torque(-axle * last_drive_torque)
		swingarm.apply_torque(axle * last_drive_torque)
	# Brakes (both wheels).
	last_front_brake_torque = _apply_brake(front_wheel, fork_slider, brake * spec.max_front_brake_torque, dt)
	last_rear_brake_torque = _apply_brake(rear_wheel, swingarm, brake * spec.max_rear_brake_torque, dt)
	# Steering: hinge motor command (fixed-step cadence: set here once per step).
	steering_joint.motor_enabled = true
	steering_joint.motor_target_speed = steering * spec.max_steering_rate
	steering_joint.motor_max_torque = spec.max_steering_torque


## Applies the opposing brake torque (see header); returns its magnitude.
func _apply_brake(wheel, carrier, commanded: float, dt: float) -> float:
	if commanded <= 0.0 or dt <= 0.0:
		return 0.0
	var u: Vector3 = wheel.orientation * Vector3.RIGHT
	var rate: float = (wheel.angular_velocity - carrier.angular_velocity).dot(u)
	if rate == 0.0 or not is_finite(rate):
		return 0.0
	var i_wheel: float = u.dot(wheel.inertia_world * u)
	var i_carrier: float = u.dot(carrier.inertia_world * u)
	var i_reduced: float = 1.0 / (1.0 / i_wheel + 1.0 / i_carrier)
	var magnitude: float = minf(commanded, STOP_FRACTION * i_reduced * absf(rate) / dt)
	var direction: float = -signf(rate)
	wheel.apply_torque(u * direction * magnitude)
	carrier.apply_torque(-u * direction * magnitude)
	return magnitude
