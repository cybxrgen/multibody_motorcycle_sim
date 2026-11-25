extends RefCounted
## Limits of the basic direct controls (M2.5.2): plain data plus validate(). NOT a powertrain, brake system or rider: only the
## maxima that scale the normalized inputs of MotorcycleControls (illustrative values, not measured).
##
##   max_drive_torque       N m  axle torque on the rear wheel at throttle 1
##   max_front_brake_torque N m  brake torque on the front wheel at brake 1
##   max_rear_brake_torque  N m  brake torque on the rear wheel at brake 1
##   max_steering_rate      rad/s target angular rate of the steering hinge at full steering input
##   max_steering_torque    N m  torque cap of the steering hinge motor
##   steering_limit         rad  symmetric steering-hinge angle limit (|angle| <= limit); 0 = no limit configured

var max_drive_torque: float = 0.0
var max_front_brake_torque: float = 0.0
var max_rear_brake_torque: float = 0.0
var max_steering_rate: float = 0.0
var max_steering_torque: float = 0.0
var steering_limit: float = 0.0


func _init(p_drive: float = 0.0, p_front_brake: float = 0.0, p_rear_brake: float = 0.0, p_rate: float = 0.0, p_steer_torque: float = 0.0, p_limit: float = 0.0) -> void:
	max_drive_torque = p_drive
	max_front_brake_torque = p_front_brake
	max_rear_brake_torque = p_rear_brake
	max_steering_rate = p_rate
	max_steering_torque = p_steer_torque
	steering_limit = p_limit


func validate() -> PackedStringArray:
	var errors: PackedStringArray = PackedStringArray()
	for pair in [["max_drive_torque", max_drive_torque], ["max_front_brake_torque", max_front_brake_torque], ["max_rear_brake_torque", max_rear_brake_torque],
			["max_steering_rate", max_steering_rate], ["max_steering_torque", max_steering_torque], ["steering_limit", steering_limit]]:
		if not (is_finite(pair[1]) and pair[1] >= 0.0):
			errors.append("controls: %s must be finite and >= 0 (got %s)" % [pair[0], pair[1]])
	if is_finite(steering_limit) and steering_limit >= PI:
		errors.append("controls: steering_limit must be < pi rad (got %s)" % steering_limit)
	return errors


func duplicate_spec():
	return (load("res://Simulation/Motorcycle/motorcycle_control_spec.gd")).new(max_drive_torque, max_front_brake_torque, max_rear_brake_torque, max_steering_rate, max_steering_torque, steering_limit)
