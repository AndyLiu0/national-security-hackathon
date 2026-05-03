extends Node
class_name SensorSWIR

# ---------------------------------------------------------------------------
# Short-Wave Infrared (SWIR) array.
#
# Per PDF: "anomaly triggers the Short-Wave Infrared (SWIR) array for
# thermal verification". SWIR sees the HCM's thermal plume / aerodynamic
# heating skin glow. It returns a thermal-bearing pair with much tighter
# noise than infrasound, plus a coarse range estimate from the apparent
# plume intensity (assuming a calibrated emitter model).
#
# Crucially, SWIR is unaffected by RF blackout (plasma sheathing actually
# *increases* IR signature), making the IR + acoustic combination the core
# of ARGUS's blackout-tolerant tracking.
# ---------------------------------------------------------------------------

@export var bearing_noise_rad: float = deg_to_rad(0.8)
@export var range_noise_frac: float = 0.18
var rng := RandomNumberGenerator.new()

var is_powered: bool = false       # gated by power manager
var thermal_intensity: float = 0.0 # 0..1
var bearing_rad: float = 0.0
var elevation_rad: float = 0.0
var range_estimate_m: float = 0.0
var has_lock: bool = false

func _ready() -> void:
	rng.randomize()

func sample(argus_pos_m: Vector3, target_pos_m: Vector3, target_thermal: float, _dt: float) -> void:
	# Always compute the physical signal so the HUD bar reflects what the
	# detector would see if it were powered. Power gating only affects
	# `has_lock` so the fusion/power-manager pipeline behaviour is unchanged.
	var rel := target_pos_m - argus_pos_m
	var dist := rel.length()
	if dist > SimConstants.SWIR_RANGE_M:
		thermal_intensity = 0.0
		has_lock = false
		return

	# Apparent intensity blends a 1/r^2 photon-flux model (dominant at close
	# range) with a softened range-fraction term that keeps the bar climbing
	# all the way out to SWIR_RANGE_M, so distance variation is obvious in
	# the sensor stack instead of pinning at 0 past ~10 km.
	var emitted := target_thermal
	var inv_sq: float = emitted / pow(max(dist / 8000.0, 0.4), 2.0)
	var range_frac: float = pow(clamp(1.0 - dist / SimConstants.SWIR_RANGE_M, 0.0, 1.0), 0.9)
	var apparent: float = tanh(0.9 * inv_sq + 0.6 * emitted * range_frac)
	thermal_intensity = lerp(thermal_intensity, clamp(apparent + rng.randfn(0.0, 0.025), 0.0, 1.0), 0.20)
	has_lock = is_powered and thermal_intensity > 0.18

	# Bearing is high-fidelity for a focal-plane IR array.
	var true_bearing := atan2(rel.z, rel.x)
	bearing_rad = true_bearing + rng.randfn(0.0, bearing_noise_rad)
	var horiz: float = Vector2(rel.x, rel.z).length()
	elevation_rad = atan2(rel.y, max(horiz, 1.0)) + rng.randfn(0.0, bearing_noise_rad)

	# Coarse range from intensity inversion (calibrated emitter model).
	if has_lock and emitted > 0.01:
		var inv_range: float = sqrt(emitted / max(thermal_intensity, 1e-3)) * 10000.0
		range_estimate_m = inv_range * (1.0 + rng.randfn(0.0, range_noise_frac))
	else:
		range_estimate_m = 0.0
