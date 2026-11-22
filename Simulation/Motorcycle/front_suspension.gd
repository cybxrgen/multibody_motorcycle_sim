extends "res://Simulation/Core/sim_force_provider.gd"
## Front telescopic-fork spring-damper (M2.2): a linear spring and a linear viscous damper acting along the axis of the fork's
## prismatic joint, between the steering assembly (body A of the joint) and the fork slider (body B). Scene-tree independent.
## Motorcycle-specific, so it lives here; it uses only the generic SimForceProvider hook of SimWorld (no motorcycle branch in
## Core or Solver) and the generic PrismaticJointConstraint (read-only: it reads the joint's world axis, never edits it).
##
## ---- Geometry ----
##   u    unit world fork axis = joint.world_axis_a(); it points from the steering assembly toward the slider, so a positive
##        displacement EXTENDS the fork (B moves along +u, away from A).
##   p_a, p_b   world attachment points (spring seats): fixed points of body A and body B (local_attachment_a/b).
##   l = (p_b - p_a) . u         axial separation of the seats; l0 = its value at the reference pose (from_world()).
##   x = l - l0                  suspension displacement (m) from the reference pose; x > 0 extension, x < 0 compression.
##                               For a satisfied joint x equals the prismatic coordinate s (the seats are rigid in A and B).
##   v = u . (v(p_b) - v(p_a))   relative velocity of the seats along the axis (m/s), v(p) = v_com + w x (p - x_com).
##                               It is the power-conjugate rate of the force pair below, so the damping power is exactly -c v^2.
##   x_rest                      rest displacement: the spring is unloaded at x = x_rest (explicit, not assumed 0).
## ---- Law ----
##   F = -k (x - x_rest) - c v        scalar force along u acting on B (N); k >= 0 N/m, c >= 0 N s/m.
##   force on B:  +F u at p_b         force on A: -F u at p_a        (equal and opposite, collinear when the seats lie on one line
##                                     parallel to u; from_world() refuses seats that do not, because a non-central pair would
##                                     inject angular momentum into an isolated system)
##   Restoring: x > x_rest gives F < 0 (pulls B back toward A, i.e. compresses); x < x_rest gives F > 0. Damping: F_c = -c v
##   always opposes the relative axial motion.
##   Because the forces are applied AT the seats, each body also receives the torque (p - x_com) x F u about its COM. The force
##   has no component perpendicular to u, so it never acts along the five directions the prismatic joint constrains (the torque it
##   causes on a body is the ordinary effect of an off-COM line of action; the joint's rows resist the resulting motion).
## ---- Energy ----
##   Spring energy U = 1/2 k (x - x_rest)^2; damping power P = F_c v = -c v^2 <= 0. The provider only adds forces; it never
##   edits poses or velocities. The force is evaluated once per FIXED step from the state at the start of the step and enters the
##   semi-implicit Euler update, so the discrete system is the symplectic-Euler map (bounded energy error, no secular energy gain
##   from the spring for omega dt < 2) with an explicit damping term that dissipates for c dt / mu < 2, mu the reduced mass of
##   the two bodies along the axis. Stability of the force evaluation is therefore a time-step condition on k and c, not a
##   hidden correction (PHYSICS_CONVENTIONS.md, "Front suspension spring-damper").

const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")
const PrismaticJoint = preload("res://Simulation/Solver/prismatic_joint_constraint.gd")

## Largest distance (m) of the two seats from a common line parallel to the axis that from_world() accepts. Float32 pose
## arithmetic of metre-sized coordinates carries ~2e-7 m; 1e-5 m is ~50x that and far below any physical offset.
const COLLINEAR_TOLERANCE: float = 1.0e-5

var joint = null
var local_attachment_a: Vector3 = Vector3.ZERO
var local_attachment_b: Vector3 = Vector3.ZERO
## N/m, finite, >= 0.
var stiffness: float = 0.0
## N s/m, finite, >= 0.
var damping: float = 0.0
## m, finite: the displacement x at which the spring force is zero.
var rest_displacement: float = 0.0
## m: axial seat separation at the reference pose (set by from_world(); any finite value is allowed for direct construction).
var reference_separation: float = 0.0


func _init(p_joint, p_local_attachment_a: Vector3, p_local_attachment_b: Vector3, p_stiffness: float, p_damping: float,
		p_rest_displacement: float, p_reference_separation: float = 0.0) -> void:
	joint = p_joint
	local_attachment_a = p_local_attachment_a
	local_attachment_b = p_local_attachment_b
	stiffness = p_stiffness
	damping = p_damping
	rest_displacement = p_rest_displacement
	reference_separation = p_reference_separation


## Builds the component from WORLD seat positions at the reference pose (the bodies' current poses). The seats become fixed
## local points of the joint's bodies and the current axial separation becomes l0. Returns null (with an error) if the input is
## invalid or the seats are not on one line parallel to the joint axis.
static func from_world(p_joint, world_attachment_a: Vector3, world_attachment_b: Vector3, p_stiffness: float, p_damping: float,
		p_rest_displacement: float):
	if p_joint == null or p_joint.body_a == null or p_joint.body_b == null:
		push_error("FrontSuspension.from_world: the prismatic joint and its bodies must not be null.")
		return null
	if not (world_attachment_a.is_finite() and world_attachment_b.is_finite()):
		push_error("FrontSuspension.from_world: attachment points must be finite.")
		return null
	var body_a = p_joint.body_a
	var body_b = p_joint.body_b
	var axis: Vector3 = p_joint.world_axis_a()
	var separation: Vector3 = world_attachment_b - world_attachment_a
	var lateral: float = (separation - axis * separation.dot(axis)).length()
	if lateral > COLLINEAR_TOLERANCE:
		push_error("FrontSuspension.from_world: the attachment points must lie on one line parallel to the fork axis (lateral offset %s m > %s m): a non-central force pair would add angular momentum." % [lateral, COLLINEAR_TOLERANCE])
		return null
	var suspension = (load("res://Simulation/Motorcycle/front_suspension.gd")).new(
		p_joint,
		body_a.orientation.inverse() * (world_attachment_a - body_a.position),
		body_b.orientation.inverse() * (world_attachment_b - body_b.position),
		p_stiffness, p_damping, p_rest_displacement, separation.dot(axis))
	return suspension if suspension.is_valid() else null


func involved_bodies() -> Array:
	return [joint.body_a, joint.body_b] if joint != null else []


func validate() -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if joint == null or joint.body_a == null or joint.body_b == null:
		errors.append("front suspension needs a prismatic joint with two bodies")
		return errors
	if joint.body_a == joint.body_b:
		errors.append("front suspension must act between two different bodies")
	if joint.body_a.is_static() and joint.body_b.is_static():
		errors.append("front suspension between two static bodies has nothing to act on")
	if not (local_attachment_a.is_finite() and local_attachment_b.is_finite()):
		errors.append("suspension attachment points must be finite")
	if not (is_finite(stiffness) and stiffness >= 0.0):
		errors.append("stiffness must be finite and >= 0 N/m (got %s)" % stiffness)
	if not (is_finite(damping) and damping >= 0.0):
		errors.append("damping must be finite and >= 0 N s/m (got %s)" % damping)
	if not is_finite(rest_displacement):
		errors.append("rest displacement must be finite (got %s)" % rest_displacement)
	if not is_finite(reference_separation):
		errors.append("reference separation must be finite (got %s)" % reference_separation)
	return errors


# --- Queries (read-only; evaluated from the CURRENT body state) ---

func attachment_a_world() -> Vector3:
	return joint.body_a.position + joint.body_a.orientation * local_attachment_a


func attachment_b_world() -> Vector3:
	return joint.body_b.position + joint.body_b.orientation * local_attachment_b


## Unit fork axis u (steering assembly toward slider).
func axis() -> Vector3:
	return joint.world_axis_a()


## Suspension displacement x = (p_b - p_a) . u - l0 (m); > 0 extension, < 0 compression.
func displacement() -> float:
	return (attachment_b_world() - attachment_a_world()).dot(axis()) - reference_separation


## Displacement from the rest position, x - x_rest (m).
func displacement_from_rest() -> float:
	return displacement() - rest_displacement


## Relative axial velocity of the seats, u . (v(p_b) - v(p_a)) (m/s), > 0 while extending.
func relative_velocity() -> float:
	return axis().dot(joint.body_b.velocity_at_point(attachment_b_world()) - joint.body_a.velocity_at_point(attachment_a_world()))


## -k (x - x_rest): the spring part of F (N), along +u on B.
func spring_force() -> float:
	return -stiffness * displacement_from_rest()


## -c v: the damping part of F (N), along +u on B.
func damping_force() -> float:
	return -damping * relative_velocity()


## F = spring + damping, the scalar force on B along +u (and on A along -u) in newtons.
func axial_force() -> float:
	return spring_force() + damping_force()


## 1/2 k (x - x_rest)^2 in joules.
func spring_energy() -> float:
	return 0.5 * stiffness * displacement_from_rest() * displacement_from_rest()


## Power of the damping force pair, -c v^2 <= 0 (W).
func damping_power() -> float:
	var v: float = relative_velocity()
	return -damping * v * v


## Applies F u at p_b to body B and -F u at p_a to body A (equal and opposite, at the seats, so each body also receives the
## torque about its COM). Static bodies are skipped exactly like gravity is. Called by SimWorld once per fixed step.
func apply_forces(_dt: float, _config) -> void:
	var u: Vector3 = axis()
	var force: Vector3 = u * axial_force()
	var p_a: Vector3 = attachment_a_world()
	var p_b: Vector3 = attachment_b_world()
	if not joint.body_b.is_static():
		joint.body_b.apply_force_at_point(force, p_b)
	if not joint.body_a.is_static():
		joint.body_a.apply_force_at_point(-force, p_a)
