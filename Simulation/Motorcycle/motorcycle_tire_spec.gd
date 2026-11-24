extends RefCounted
## Specification of one tyre (M2.4.2): plain data plus validate(), scene-tree independent. One instance per wheel; the front and the
## rear tyre are independent objects. The values are ILLUSTRATIVE PROTOTYPE parameters of a deliberately simple model
## (tire_model.gd), not measured tyre data. Equations and sign conventions: PHYSICS_CONVENTIONS.md ("Tyre behaviour").
##
## Limits of the tangential force (friction ellipse) are mu_x = grip_longitudinal * mu_surface and mu_y = grip_lateral * mu_surface,
## where mu_surface is the ground specification's friction coefficient: the surface supplies the base coefficient and the tyre the
## (anisotropic) scale. The generic Coulomb friction of ContactConstraint is NOT used by a wheel that has a tyre.

## Effective (kinematic) rolling radius R_eff (m): free rolling has tread speed R_eff * spin = hub speed. Must lie in
## [MIN_RADIUS_RATIO, 1] times the contact geometry radius of the wheel (a loaded tyre never rolls on more than its free radius).
var rolling_radius: float = 0.0
## Longitudinal slip stiffness k_x (dimensionless): small-slip force Fx = k_x * N * slip_x (N = normal load).
var longitudinal_stiffness: float = 0.0
## Lateral slip stiffness k_y (dimensionless, per unit lateral slip ~ per radian): small-slip force Fy = k_y * N * slip_y.
var lateral_stiffness: float = 0.0
## Dimensionless scale of the surface friction coefficient giving the longitudinal / lateral force limit (> 0).
var grip_longitudinal: float = 1.0
var grip_lateral: float = 1.0
## Rolling-resistance coefficient c_rr (dimensionless, >= 0): force magnitude c_rr * N at rolling speeds >> rolling_resistance_speed.
var rolling_resistance: float = 0.0
## Speed scale (m/s, > 0) over which rolling resistance rises from zero: F = c_rr N psi(v / v_rr), psi(x) = x / sqrt(1 + x^2).
var rolling_resistance_speed: float = 0.2
## Low-speed regularisation (m/s, > 0): slip denominators are sqrt(v_x^2 + v_ref^2), finite and smooth at zero speed.
var low_speed_reference: float = 0.5

## A wheel's rolling radius may not be smaller than this fraction of its geometric radius.
const MIN_RADIUS_RATIO: float = 0.5


## One message per violated rule; empty = valid. `wheel_radius` is the contact geometry radius of the wheel this tyre belongs to.
func validate(wheel_radius: float = 0.0) -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	if not (is_finite(rolling_radius) and rolling_radius > 0.0):
		errors.append("tire: rolling radius must be finite and > 0 m (got %s)" % rolling_radius)
	elif wheel_radius > 0.0 and is_finite(wheel_radius) and not (rolling_radius >= MIN_RADIUS_RATIO * wheel_radius and rolling_radius <= wheel_radius):
		errors.append("tire: rolling radius %s m must lie in [%s, %s] m (0.5 to 1.0 times the wheel radius)" % [rolling_radius, MIN_RADIUS_RATIO * wheel_radius, wheel_radius])
	for pair in [["longitudinal stiffness", longitudinal_stiffness], ["lateral stiffness", lateral_stiffness],
			["longitudinal grip", grip_longitudinal], ["lateral grip", grip_lateral],
			["rolling-resistance speed", rolling_resistance_speed], ["low-speed reference", low_speed_reference]]:
		if not (is_finite(pair[1]) and pair[1] > 0.0):
			errors.append("tire: %s must be finite and > 0 (got %s)" % pair)
	if not (is_finite(rolling_resistance) and rolling_resistance >= 0.0):
		errors.append("tire: rolling-resistance coefficient must be finite and >= 0 (got %s)" % rolling_resistance)
	elif is_finite(grip_longitudinal) and rolling_resistance > 0.5 * grip_longitudinal:
		errors.append("tire: rolling-resistance coefficient %s must not exceed half of the longitudinal grip %s (it is a small loss, not a traction limit)" % [rolling_resistance, grip_longitudinal])
	return errors


func is_valid(wheel_radius: float = 0.0) -> bool:
	return validate(wheel_radius).is_empty()


## An independent copy (callers may mutate it).
func duplicate_spec():
	var copy = (load("res://Simulation/Motorcycle/motorcycle_tire_spec.gd")).new()
	copy.rolling_radius = rolling_radius
	copy.longitudinal_stiffness = longitudinal_stiffness
	copy.lateral_stiffness = lateral_stiffness
	copy.grip_longitudinal = grip_longitudinal
	copy.grip_lateral = grip_lateral
	copy.rolling_resistance = rolling_resistance
	copy.rolling_resistance_speed = rolling_resistance_speed
	copy.low_speed_reference = low_speed_reference
	return copy
