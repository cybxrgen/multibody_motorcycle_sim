extends Resource
## Centralized simulation parameters. No numerical tuning values live anywhere else.

@export var gravity: Vector3 = Vector3(0.0, -9.81, 0.0)

@export_group("Time stepping")
## Fixed physics step in seconds.
@export var fixed_timestep: float = 1.0 / 120.0
## Frame time is clamped to this before entering the accumulator (spiral-of-death guard #1).
@export var max_frame_time: float = 0.1
## Hard cap of simulation steps per advance() call (guard #2). Remaining backlog is dropped.
@export var max_steps_per_frame: int = 10

@export_group("Constraint solver")
## Sequential-impulse / projected Gauss-Seidel iterations per step.
@export var solver_iterations: int = 20
## Baumgarte factor beta: fraction of position error removed per step. Applied through the split-impulse channel
## (split bias = beta/dt * C), so it corrects pose without adding physical velocity.
@export var baumgarte_factor: float = 0.2
## Re-apply last step's accumulated impulses before iterating.
@export var warm_starting: bool = true
## Scale applied to carried-over impulses (1 = full warm start).
@export var warm_start_factor: float = 1.0
## Rows with K + softness below this are treated as having zero effective mass (degenerate).
@export var min_row_denominator: float = 1.0e-12


## Centralized validation. Returns one message per violated rule; empty means valid.
## Nothing is clamped or corrected: callers (SimWorld) refuse to simulate while invalid.
## Ranges: dt, max_frame_time, min_row_denominator finite and > 0; max_steps_per_frame and
## solver_iterations >= 1; baumgarte_factor and warm_start_factor in [0, 1] (beta > 1 would
## over-correct position error within one step; a warm-start scale > 1 amplifies stale impulses).
func validate() -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if not (is_finite(gravity.x) and is_finite(gravity.y) and is_finite(gravity.z)):
		errors.append("gravity must be finite (got %s)" % gravity)
	if not _is_positive_finite(fixed_timestep):
		errors.append("fixed_timestep must be finite and > 0 (got %s)" % fixed_timestep)
	if not _is_positive_finite(max_frame_time):
		errors.append("max_frame_time must be finite and > 0 (got %s)" % max_frame_time)
	if max_steps_per_frame < 1:
		errors.append("max_steps_per_frame must be >= 1 (got %s)" % max_steps_per_frame)
	if solver_iterations < 1:
		errors.append("solver_iterations must be >= 1 (got %s)" % solver_iterations)
	if not _is_in_unit_range(baumgarte_factor):
		errors.append("baumgarte_factor must be in [0, 1] (got %s)" % baumgarte_factor)
	if not _is_in_unit_range(warm_start_factor):
		errors.append("warm_start_factor must be in [0, 1] (got %s)" % warm_start_factor)
	if not _is_positive_finite(min_row_denominator):
		errors.append("min_row_denominator must be finite and > 0 (got %s)" % min_row_denominator)
	return errors


func is_valid() -> bool:
	return validate().is_empty()


static func _is_positive_finite(x: float) -> bool:
	return is_finite(x) and x > 0.0


static func _is_in_unit_range(x: float) -> bool:
	return is_finite(x) and x >= 0.0 and x <= 1.0
