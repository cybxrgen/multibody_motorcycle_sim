extends RefCounted
## Base class: a constraint between two bodies made of scalar rows.
## Subclasses override _update_rows() to refresh Jacobians/bias from current body poses.
## The base implementation leaves rows untouched, so rows can also be filled in by hand.

const ConstraintRow = preload("res://Simulation/Solver/constraint_row.gd")
const SimRigidBody = preload("res://Simulation/Core/sim_rigid_body.gd")

var body_a: SimRigidBody
var body_b: SimRigidBody
var rows: Array = []
var enabled: bool = true


func _init(p_body_a: SimRigidBody, p_body_b: SimRigidBody) -> void:
	body_a = p_body_a
	body_b = p_body_b


func add_row() -> ConstraintRow:
	var row: ConstraintRow = ConstraintRow.new(body_a, body_b)
	rows.append(row)
	return row


## Rule check for the constraint's own data. One message per violated rule; empty means valid.
## The base class has no data to check. SimWorld refuses to add (add_constraint) or to step
## (preflight) while a constraint reports errors, so invalid input is never silently simulated.
func validate() -> PackedStringArray:
	return PackedStringArray()


func is_valid() -> bool:
	return validate().is_empty()


## Prepare for the current step: refresh Jacobians, effective masses, and warm start.
func prepare(dt: float, config) -> void:
	_update_rows(dt, config)
	for row in rows:
		row.prepare(config.min_row_denominator)
		row.warm_start(config.warm_starting, config.warm_start_factor)


func _update_rows(_dt: float, _config) -> void:
	pass


## One solver iteration over this constraint. The default solves every row in order (scalar projected Gauss-Seidel).
## A constraint whose rows are coupled (e.g. contact friction, where two tangent rows share one disk-shaped admissible
## region that depends on another row's impulse) overrides this to order and project them itself. The solver stays generic:
## it only calls this hook.
func solve_iteration() -> void:
	for row in rows:
		row.solve()


## One iteration of the split-impulse (positional correction) pass over this constraint. Same structure as solve_iteration():
## the default solves every row's split channel in order (rows with split_enabled = false do nothing). It only touches the
## bodies' split velocities. A constraint whose physical solve is coupled does not need to override this unless its split rows
## are coupled too (none are).
func solve_split_iteration() -> void:
	for row in rows:
		row.solve_split()


func finish() -> void:
	pass
