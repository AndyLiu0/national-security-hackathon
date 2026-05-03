extends Node
class_name SensorPressure

# ---------------------------------------------------------------------------
# Stereo differential air-pressure sensors mounted at the glider's wingtips.
# The two transducers see slightly different overpressure from a passing
# atmospheric shock; the sum is intensity, the difference encodes lateral
# bearing (TDoA-style). Always on, very low power, plasma-blackout immune.
# ---------------------------------------------------------------------------

@export var noise: float = 0.04
@export var lateral_gain: float = 0.5
# Near-field overpressure scaling: peak ~1.0 when the HCM passes ~300 m off
# the wingtip at hypersonic speed. Falls as 1/r so a close pass produces a
# sharp, unmistakable spike in both transducers.
@export var near_field_k: float = 320.0
@export var near_field_floor_m: float = 200.0

var rng := RandomNumberGenerator.new()

var is_powered: bool = true
var left_dp: float = 0.0
var right_dp: float = 0.0
var sum_p: float = 0.0
var diff_p: float = 0.0    # left - right

# Cached so the HUD/model can see the bow-shock contribution explicitly.
var near_field_pa: float = 0.0

func _ready() -> void:
	rng.randomize()

func sample(argus_pos_m: Vector3, argus_heading_rad: float, target_pos_m: Vector3, target_speed_mps: float, _dt: float) -> void:
	var rel := target_pos_m - argus_pos_m
	var dist := rel.length()
	var range_factor: float = pow(clamp(1.0 - dist / SimConstants.INFRASOUND_RANGE_M, 0.0, 1.0), 1.8)
	var mach: float = max(target_speed_mps, 1.0) / 340.0
	var far_field: float = range_factor * (1.0 - exp(-(mach * mach) / 90.0))

	# Near-field N-wave: when the missile is supersonic AND close, the
	# wingtip transducers see a sharp 1/r overpressure peak.
	var super_factor: float = max(0.0, sqrt(mach) - 1.0)
	near_field_pa = near_field_k * super_factor / max(dist, near_field_floor_m)
	# Smooth roll-off beyond ~8 km so the boom only registers nearby.
	near_field_pa *= clamp(1.0 - dist / 8000.0, 0.0, 1.0)

	var base: float = far_field + near_field_pa

	var rel_az: float = atan2(rel.z, rel.x) - argus_heading_rad
	var lateral: float = sin(rel_az)

	left_dp  = base * (0.5 + lateral_gain * lateral) + rng.randfn(0.0, noise)
	right_dp = base * (0.5 - lateral_gain * lateral) + rng.randfn(0.0, noise)
	left_dp  = clamp(left_dp, 0.0, 4.0)
	right_dp = clamp(right_dp, 0.0, 4.0)
	sum_p = left_dp + right_dp
	diff_p = left_dp - right_dp
