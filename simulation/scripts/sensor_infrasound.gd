extends Node
class_name SensorInfrasound

# ---------------------------------------------------------------------------
# Passive infrasound / atmospheric pressure sensor.
#
# Per PDF: "low-power state using passive infrasound monitoring until an
# anomaly triggers the SWIR array". This sensor is the always-on tripwire.
# It detects upward-propagating shockwaves (from the HCM's bow shock and
# its acoustic signature). It is long-range but low-fidelity: it gives a
# coarse bearing and an anomaly score, NOT a position. Plasma sheathing
# does NOT block infrasound (it's an EM phenomenon), so this sensor is
# uniquely valuable in the RF-blackout window.
# ---------------------------------------------------------------------------

@export var bearing_noise_rad: float = deg_to_rad(8.0)
@export var detection_floor: float = 0.05
var rng := RandomNumberGenerator.new()

# Latest measurement, consumed by the fusion node.
var anomaly_score: float = 0.0      # 0..1
var bearing_rad: float = 0.0        # azimuth from ARGUS, world frame
var elevation_rad: float = 0.0      # rough up-angle (very noisy)
var is_powered: bool = true         # always on per tiered architecture
var last_update_t: float = 0.0

func _ready() -> void:
	rng.randomize()

func sample(argus_pos_m: Vector3, target_pos_m: Vector3, target_speed_mps: float, dt: float) -> void:
	last_update_t += dt
	if not is_powered:
		anomaly_score = 0.0
		return

	var rel := target_pos_m - argus_pos_m
	var dist := rel.length()

	# Inverse-square-ish acoustic attenuation, with a long usable range
	# because infrasound carries enormous distances in the stratosphere.
	var range_factor: float = clamp(1.0 - dist / SimConstants.INFRASOUND_RANGE_M, 0.0, 1.0)

	# Shock energy scales with speed^2; HCM is loud, subsonic targets aren't.
	var shock_energy: float = pow(max(target_speed_mps, 1.0) / 340.0, 2.0)
	var raw: float = range_factor * (shock_energy / (1.0 + shock_energy)) * 1.6

	# Background atmospheric noise.
	raw += rng.randfn(0.0, 0.06)
	anomaly_score = clamp(raw, 0.0, 1.0)
	if anomaly_score < detection_floor:
		anomaly_score = 0.0

	# Bearing: real direction + Gaussian noise. Elevation is very coarse.
	var true_bearing := atan2(rel.z, rel.x)
	bearing_rad = true_bearing + rng.randfn(0.0, bearing_noise_rad)
	var horiz: float = Vector2(rel.x, rel.z).length()
	elevation_rad = atan2(rel.y, max(horiz, 1.0)) + rng.randfn(0.0, deg_to_rad(15.0))
