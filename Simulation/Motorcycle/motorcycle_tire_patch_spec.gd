extends RefCounted
## Specification of the distributed contact patch of ONE tyre (M2.5.3 Stage D): the sampling grid and the compliant normal-support
## parameters that TirePatchNormalSupport needs and that no other specification owns. Plain data plus validate(), scene-tree independent.
## One instance per wheel; the front and the rear patch are independent objects. The tyre cross-section comes from the wheel's
## TireGeometry and the tangential model from the wheel's MotorcycleTireSpec (their ownership is unchanged); the surface friction comes from
## the ground. A wheel with a patch specification is given a TirePatchNormalSupport INSTEAD of the one-point TireGroundContact (never both).
## The values are ILLUSTRATIVE prototype parameters, not measured tyre data. Equations: PHYSICS_CONVENTIONS.md ("Tyre patch normal support").
## The rules repeat TirePatchNormalSupport.validate() for exactly these fields so that MotorcycleConfig.validate() refuses a configuration
## the builder could not construct; the bounds are the constraint's own constants (single source).

const TirePatchNormalSupport = preload("res://Simulation/Motorcycle/tire_patch_normal_support.gd")
const TirePatchSampler = preload("res://Simulation/Motorcycle/tire_patch_sampler.gd")

## Samples across the tread width (TirePatchSampler.MIN/MAX_LATERAL_COUNT).
var lateral_count: int = 9
## Samples around the circumference (TirePatchSampler.MIN/MAX_CIRCUMFERENTIAL_COUNT).
var circumferential_count: int = 48
## k_A (N/m^3): normal stiffness per unit tread area per metre of compression.
var stiffness_density: float = 4.0e6
## c_A (N s/m^3): normal damping per unit tread area per m/s.
var damping_density: float = 1.0e5
## Compression (m) beyond which a sample is bottomed; at most half of the wheel radius.
var max_compression: float = 0.05


func _init(p_lateral_count: int = 9, p_circumferential_count: int = 48, p_stiffness_density: float = 4.0e6,
		p_damping_density: float = 1.0e5, p_max_compression: float = 0.05) -> void:
	lateral_count = p_lateral_count
	circumferential_count = p_circumferential_count
	stiffness_density = p_stiffness_density
	damping_density = p_damping_density
	max_compression = p_max_compression


## One message per violated rule; empty = valid. `wheel_radius` is the contact radius of the wheel this patch belongs to.
func validate(wheel_radius: float = 0.0) -> PackedStringArray:
	var errors: PackedStringArray = TirePatchSampler.validate_counts(lateral_count, circumferential_count)
	if not (is_finite(stiffness_density) and stiffness_density > 0.0 and stiffness_density <= TirePatchNormalSupport.MAX_STIFFNESS_DENSITY):
		errors.append("tire patch: stiffness density must be finite and in (0, %s] N/m^3 (got %s)" % [TirePatchNormalSupport.MAX_STIFFNESS_DENSITY, stiffness_density])
	if not (is_finite(damping_density) and damping_density >= 0.0 and damping_density <= TirePatchNormalSupport.MAX_DAMPING_DENSITY):
		errors.append("tire patch: damping density must be finite and in [0, %s] N s/m^3 (got %s)" % [TirePatchNormalSupport.MAX_DAMPING_DENSITY, damping_density])
	if not (is_finite(max_compression) and max_compression > 0.0):
		errors.append("tire patch: maximum compression must be finite and > 0 m (got %s)" % max_compression)
	elif wheel_radius > 0.0 and is_finite(wheel_radius) and max_compression > TirePatchNormalSupport.MAX_COMPRESSION_RATIO * wheel_radius:
		errors.append("tire patch: maximum compression %s m must not exceed %s of the wheel radius (%s m)"
				% [max_compression, TirePatchNormalSupport.MAX_COMPRESSION_RATIO, TirePatchNormalSupport.MAX_COMPRESSION_RATIO * wheel_radius])
	return errors


func is_valid(wheel_radius: float = 0.0) -> bool:
	return validate(wheel_radius).is_empty()


## An independent copy (callers may mutate it).
func duplicate_spec():
	return (load("res://Simulation/Motorcycle/motorcycle_tire_patch_spec.gd")).new(lateral_count, circumferential_count, stiffness_density,
			damping_density, max_compression)
