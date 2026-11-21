extends RefCounted
## Generic rigid body. `position` is the center of mass in world coordinates.
## All vectors are world-frame unless the name says "local". See PHYSICS_CONVENTIONS.md.

const SimMassProperties = preload("res://Simulation/Core/sim_mass_properties.gd")
const SimBodyState = preload("res://Simulation/Core/sim_body_state.gd")

var id: int = -1
var body_name: String = ""

var mass_properties: SimMassProperties = SimMassProperties.new()

var mass: float:
	get:
		return mass_properties.mass
var inverse_mass: float:
	get:
		return mass_properties.inverse_mass
var local_inertia: Basis:
	get:
		return mass_properties.local_inertia
var inverse_local_inertia: Basis:
	get:
		return mass_properties.inverse_local_inertia

var position: Vector3 = Vector3.ZERO
## Local->world unit quaternion. Assigning re-normalizes and refreshes the cached world inertia.
var orientation: Quaternion = Quaternion.IDENTITY:
	set(value):
		orientation = value.normalized()
		update_world_inertia()
var linear_velocity: Vector3 = Vector3.ZERO
var angular_velocity: Vector3 = Vector3.ZERO

## Split-impulse (pseudo) velocities: SOLVER SCRATCH, not physical state. They carry positional constraint correction only
## (M1.3, PHYSICS_CONVENTIONS.md "Split-impulse stabilization"): they are zeroed before the solver of every fixed step, written
## only by the split solve, added to the physical velocities only inside the position integration, and zeroed again right after
## it. They are never part of momentum, kinetic energy, motor/friction/contact velocity or the published state.
var split_linear_velocity: Vector3 = Vector3.ZERO
var split_angular_velocity: Vector3 = Vector3.ZERO

var force_accumulator: Vector3 = Vector3.ZERO
var torque_accumulator: Vector3 = Vector3.ZERO

## Cached; always consistent with `orientation`.
var inertia_world: Basis = Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)
var inverse_inertia_world: Basis = Basis(Vector3.ZERO, Vector3.ZERO, Vector3.ZERO)

## State at the start of the last step and state published at its end (for render interpolation).
var previous_state: SimBodyState = SimBodyState.new()
var published_state: SimBodyState = SimBodyState.new()


func _init(p_mass_properties: SimMassProperties = null, p_position: Vector3 = Vector3.ZERO, p_orientation: Quaternion = Quaternion.IDENTITY) -> void:
	if p_mass_properties != null:
		mass_properties = p_mass_properties
	position = p_position
	orientation = p_orientation
	reset_published_state()


## Returns false (keeping the current mass properties) if the argument is null or invalid.
func set_mass_properties(p_mass_properties: SimMassProperties) -> bool:
	if p_mass_properties == null or not p_mass_properties.valid:
		push_error("SimRigidBody.set_mass_properties: rejected invalid mass properties.")
		return false
	mass_properties = p_mass_properties
	update_world_inertia()
	return true


func is_static() -> bool:
	return mass_properties.inverse_mass == 0.0


func update_world_inertia() -> void:
	var r: Basis = Basis(orientation)
	inertia_world = mass_properties.world_inertia(r)
	inverse_inertia_world = mass_properties.world_inverse_inertia(r)


## Call after teleporting a body (initial setup) so interpolation has no stale history.
func reset_published_state() -> void:
	previous_state.capture_from(self)
	published_state.capture_from(self)


func get_transform() -> Transform3D:
	return Transform3D(Basis(orientation), position)


# --- Forces and torques (accumulate over the next step, then cleared) ---

func apply_force(force: Vector3) -> void:
	force_accumulator += force


func apply_torque(torque: Vector3) -> void:
	torque_accumulator += torque


func apply_force_at_point(force: Vector3, world_point: Vector3) -> void:
	force_accumulator += force
	torque_accumulator += (world_point - position).cross(force)


func clear_accumulators() -> void:
	force_accumulator = Vector3.ZERO
	torque_accumulator = Vector3.ZERO


func clear_split_velocity() -> void:
	split_linear_velocity = Vector3.ZERO
	split_angular_velocity = Vector3.ZERO


# --- Impulses (instantaneous velocity changes; never touch the accumulators) ---

func apply_impulse(impulse: Vector3) -> void:
	linear_velocity += impulse * mass_properties.inverse_mass


func apply_angular_impulse(angular_impulse: Vector3) -> void:
	angular_velocity += inverse_inertia_world * angular_impulse


func apply_impulse_at_point(impulse: Vector3, world_point: Vector3) -> void:
	apply_impulse(impulse)
	apply_angular_impulse((world_point - position).cross(impulse))


# --- Queries ---

func velocity_at_point(world_point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(world_point - position)


func linear_momentum() -> Vector3:
	return linear_velocity * mass_properties.mass


## Angular momentum about the COM: I_world * omega.
func angular_momentum_about_com() -> Vector3:
	return inertia_world * angular_velocity


func kinetic_energy() -> float:
	return 0.5 * mass_properties.mass * linear_velocity.length_squared() \
		+ 0.5 * angular_velocity.dot(inertia_world * angular_velocity)
