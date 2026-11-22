extends RefCounted
## Base class: a generic per-step force provider (a spring, a damper, a drag law, ...). Scene-tree independent.
##
## Why it exists (ARCHITECTURE.md, D21): forces applied to a body are consumed and cleared by ONE fixed step, but a frame can run
## several fixed steps (SimWorld.advance), and a state-dependent force such as a spring-damper must be re-evaluated from the
## current state at every one of them. A caller that applies the force once per frame would apply it to the first step only and
## evaluate it at the render rate. A provider registered in the world is asked once per fixed step instead.
##
## Contract:
##  * apply_forces() is called once per fixed step, after gravity has been accumulated and before the velocities are
##    integrated, with the state at the START of the step. It may only call the bodies' force/torque API
##    (apply_force, apply_force_at_point, apply_torque). It must not write poses or velocities, and must not read split velocities.
##  * involved_bodies() lists every body the provider reads or loads. SimWorld requires each to be a registered member
##    (object identity), re-checks it at every step (preflight) and drops the provider when one of them is removed.
##  * validate() reports invalid data (one message per violated rule; empty = valid). SimWorld refuses to register a provider
##    (add_force_provider) or to step (preflight) while it reports errors. Nothing is clamped or repaired.
##  * A disabled provider (enabled = false) is skipped.
## Forces applied here are cleared with the other accumulators at the end of the step, and discarded by an aborted step.

var enabled: bool = true


## Every body this provider reads or applies loads to. Default: none.
func involved_bodies() -> Array:
	return []


func validate() -> PackedStringArray:
	return PackedStringArray()


func is_valid() -> bool:
	return validate().is_empty()


## Accumulates this step's forces/torques into the bodies. `dt` is the fixed timestep, `config` the world's SimConfig.
func apply_forces(_dt: float, _config) -> void:
	pass
