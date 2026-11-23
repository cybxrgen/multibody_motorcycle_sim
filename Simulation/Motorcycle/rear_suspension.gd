extends "res://Simulation/Core/sim_force_provider.gd"
## Rear spring-damper (M2.3): a direct linear spring and linear viscous damper between the motorcycle chassis and the swingarm,
## acting along the INSTANTANEOUS line joining two attachment points (one fixed in each body). Scene-tree independent.
## Motorcycle-specific, so it lives here; it uses only the generic SimForceProvider hook of SimWorld (no motorcycle branch in
## Core or Solver) and adds no joint, constraint or body: the shock is an explicit force pair, nothing else.
##
## ---- Geometry (world frame, evaluated from the CURRENT body state) ----
##   p_c, p_s   world attachment points: fixed points of the chassis and of the swingarm (local_attachment_chassis/_swingarm)
##   d = p_s - p_c,  l = |d|,  n = d / l      n points from the chassis attachment toward the swingarm attachment
##   l_0                                      configured spring rest length (m, > 0), explicit (not derived from the pose)
##   v(p) = v_com + w x (p - x_com)           velocity of a body-fixed point; includes the rotational contribution
##   l_dot = n . (v(p_s) - v(p_c))            extension rate of the shock (m/s), > 0 while it lengthens
## ---- Law ----
##   F = -k (l - l_0) - c l_dot               scalar force along n (N); k >= 0 N/m, c >= 0 N s/m
##   force on the swingarm:  F_s = +F n  at p_s          force on the chassis:  F_c = -F n  at p_c
##   Restoring: l > l_0 gives F < 0 (the swingarm is pulled toward the chassis, the shock shortens); l < l_0 gives F > 0.
##   Damping F_c = -c l_dot always opposes the change of length. The direction n is recomputed from the attachment geometry at
##   every step: it is not tied to any body axis, so it follows the swingarm as it rotates and the shock is never assumed vertical.
##   Because each force acts AT its attachment point, each body also receives the torque (p - x_com) x F about its own COM.
##   The pair is equal and opposite and collinear (both lie on the line p_c p_s), so the total force is zero AND the total torque
##   about any point is zero: an isolated pair conserves linear and angular momentum.
## ---- Energy ----
##   Spring energy U = 1/2 k (l - l_0)^2. Power of the force pair P = F_s.v(p_s) + F_c.v(p_c) = F l_dot; the damping part is
##   -c l_dot^2 <= 0 (dissipative) and the spring part is -dU/dt. The provider only adds forces; it never edits poses or
##   velocities. Evaluated once per FIXED step from the start-of-step state, so the discrete system is the same symplectic-Euler
##   map as the front suspension, with the same time-step stability condition on k and c (PHYSICS_CONVENTIONS.md, "Rear
##   suspension spring-damper"); that condition is not a hidden correction and is not checked at run time.
## ---- Degenerate geometry ----
##   n is undefined for l -> 0. The provider refuses l < MIN_LENGTH: validate() reports it (so add_force_provider and the world's
##   preflight refuse it; nothing is clamped or repaired), and direction() / the force queries return zero and apply_forces()
##   applies nothing there, so a non-finite direction or force can never be produced.

## Smallest attachment separation (m) accepted. Pose arithmetic in float32 (local = R^-1 (P - x), world = x + R local) carries
## ~2e-7 m for metre-sized coordinates, so the direction n = d / l has an error of ~2e-7 / l rad; at l = 1 mm that is ~2e-4 rad,
## and 1 mm is ~5000x the position rounding and far below any real shock. Shorter separations are refused, never clamped.
const MIN_LENGTH: float = 1.0e-3

var chassis_body = null
var swingarm_body = null
var local_attachment_chassis: Vector3 = Vector3.ZERO
var local_attachment_swingarm: Vector3 = Vector3.ZERO
## N/m, finite, >= 0.
var stiffness: float = 0.0
## N s/m, finite, >= 0.
var damping: float = 0.0
## m, finite, > 0: the length l_0 at which the spring force is zero.
var rest_length: float = 0.0


func _init(p_chassis_body, p_swingarm_body, p_local_attachment_chassis: Vector3, p_local_attachment_swingarm: Vector3,
		p_stiffness: float, p_damping: float, p_rest_length: float) -> void:
	chassis_body = p_chassis_body
	swingarm_body = p_swingarm_body
	local_attachment_chassis = p_local_attachment_chassis
	local_attachment_swingarm = p_local_attachment_swingarm
	stiffness = p_stiffness
	damping = p_damping
	rest_length = p_rest_length


## Builds the component from WORLD attachment points at the reference pose (the bodies' current poses): each point becomes a
## fixed local point of its body. Returns null (with an error) if the bodies or the data are invalid.
static func from_world(p_chassis_body, p_swingarm_body, world_attachment_chassis: Vector3, world_attachment_swingarm: Vector3,
		p_stiffness: float, p_damping: float, p_rest_length: float):
	if p_chassis_body == null or p_swingarm_body == null:
		push_error("RearSuspension.from_world: the chassis and swingarm bodies must not be null.")
		return null
	if not (world_attachment_chassis.is_finite() and world_attachment_swingarm.is_finite()):
		push_error("RearSuspension.from_world: attachment points must be finite.")
		return null
	var suspension = (load("res://Simulation/Motorcycle/rear_suspension.gd")).new(
		p_chassis_body, p_swingarm_body,
		p_chassis_body.orientation.inverse() * (world_attachment_chassis - p_chassis_body.position),
		p_swingarm_body.orientation.inverse() * (world_attachment_swingarm - p_swingarm_body.position),
		p_stiffness, p_damping, p_rest_length)
	var errors: PackedStringArray = suspension.validate()
	if not errors.is_empty():
		push_error("RearSuspension.from_world: invalid rear suspension: %s" % "; ".join(errors))
		return null
	return suspension


func involved_bodies() -> Array:
	return [chassis_body, swingarm_body]


func validate() -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if chassis_body == null or swingarm_body == null:
		errors.append("rear suspension needs a chassis body and a swingarm body")
		return errors
	if chassis_body == swingarm_body:
		errors.append("rear suspension must act between two different bodies")
	if chassis_body.is_static() and swingarm_body.is_static():
		errors.append("rear suspension between two static bodies has nothing to act on")
	var attachments_finite: bool = local_attachment_chassis.is_finite() and local_attachment_swingarm.is_finite()
	if not attachments_finite:
		errors.append("rear suspension attachment points must be finite")
	if not (is_finite(stiffness) and stiffness >= 0.0):
		errors.append("rear suspension stiffness must be finite and >= 0 N/m (got %s)" % stiffness)
	if not (is_finite(damping) and damping >= 0.0):
		errors.append("rear suspension damping must be finite and >= 0 N s/m (got %s)" % damping)
	if not (is_finite(rest_length) and rest_length > 0.0):
		errors.append("rear suspension rest length must be finite and > 0 m (got %s)" % rest_length)
	if attachments_finite:
		# Current geometry. A non-finite length (corrupt body pose) is left to the world's own pose validation.
		var separation: float = length()
		if is_finite(separation) and separation < MIN_LENGTH:
			errors.append("rear suspension attachment points are %s m apart (< %s m): the shock direction is undefined" % [separation, MIN_LENGTH])
	return errors


# --- Queries (read-only; evaluated from the CURRENT body state) ---

func attachment_chassis_world() -> Vector3:
	return chassis_body.position + chassis_body.orientation * local_attachment_chassis


func attachment_swingarm_world() -> Vector3:
	return swingarm_body.position + swingarm_body.orientation * local_attachment_swingarm


## Current shock length l = |p_s - p_c| (m).
func length() -> float:
	return (attachment_swingarm_world() - attachment_chassis_world()).length()


## True while the geometry defines a direction (l >= MIN_LENGTH; false for non-finite poses too).
func has_direction() -> bool:
	return length() >= MIN_LENGTH


## Unit vector n from the chassis attachment toward the swingarm attachment; Vector3.ZERO where it is undefined (l < MIN_LENGTH).
func direction() -> Vector3:
	var d: Vector3 = attachment_swingarm_world() - attachment_chassis_world()
	var l: float = d.length()
	if not (l >= MIN_LENGTH):
		return Vector3.ZERO
	return d / l


## Extension l - l_0 (m): > 0 when the shock is longer than its rest length, < 0 when compressed.
func extension() -> float:
	return length() - rest_length


## l_dot = n . (v(p_s) - v(p_c)) (m/s), > 0 while the shock lengthens. Zero where n is undefined.
func relative_extension_rate() -> float:
	var n: Vector3 = direction()
	return n.dot(swingarm_body.velocity_at_point(attachment_swingarm_world()) - chassis_body.velocity_at_point(attachment_chassis_world()))


## -k (l - l_0): the spring part of F (N), along +n on the swingarm.
func spring_force() -> float:
	if not has_direction():
		return 0.0
	return -stiffness * extension()


## -c l_dot: the damping part of F (N), along +n on the swingarm.
func damping_force() -> float:
	return -damping * relative_extension_rate()


## F = spring + damping: the scalar force on the swingarm along +n (and on the chassis along -n), in newtons.
func axial_force() -> float:
	return spring_force() + damping_force()


## 1/2 k (l - l_0)^2 in joules.
func spring_energy() -> float:
	var e: float = extension()
	return 0.5 * stiffness * e * e


## Power of the damping force pair, -c l_dot^2 <= 0 (W).
func damping_power() -> float:
	var rate: float = relative_extension_rate()
	return -damping * rate * rate


## Applies F n at p_s to the swingarm and -F n at p_c to the chassis (equal and opposite, AT the attachment points, so each body
## also receives the torque about its COM). Static bodies are skipped exactly like gravity is. Called by SimWorld once per
## fixed step; applies nothing where the direction is undefined.
func apply_forces(_dt: float, _config) -> void:
	var n: Vector3 = direction()
	if n == Vector3.ZERO:
		return
	var force: Vector3 = n * axial_force()
	if not swingarm_body.is_static():
		swingarm_body.apply_force_at_point(force, attachment_swingarm_world())
	if not chassis_body.is_static():
		chassis_body.apply_force_at_point(-force, attachment_chassis_world())
