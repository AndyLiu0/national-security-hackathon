extends Node3D
class_name HCMTarget

# ---------------------------------------------------------------------------
# Hypersonic Cruise Missile (HCM) target.
#
# Per PDF "Adversary Modeling": programmed with advanced evasion heuristics —
# randomized glide maneuvers and erratic altitude changes. Also models the
# RF blackout (plasma sheathing) window where active emissions / RF tracking
# are degraded; ARGUS's passive multi-modal stack must keep custody through
# this window.
# ---------------------------------------------------------------------------

@export var cruise_speed_mps: float = SimConstants.HCM_CRUISE_MPS
@export var alt_band: Vector2 = Vector2(SimConstants.HCM_ALT_MIN_M, SimConstants.HCM_ALT_MAX_M)

# Evasion heuristic parameters.
@export var maneuver_period_s: float = 4.5
@export var lateral_g: float = 6.0
@export var altitude_jitter_m: float = 1500.0

var velocity_mps: Vector3 = Vector3.ZERO
var truth_position_m: Vector3 = Vector3.ZERO  # "true" world-frame position in meters
var heading_rad: float = 0.0
var _maneuver_timer: float = 0.0
var _target_alt_m: float = 22000.0
var _lateral_bias: float = 0.0
var rng := RandomNumberGenerator.new()

# Plasma-sheathing RF blackout — engaged when accelerating hard or peak-cruise.
var plasma_blackout: bool = false
var thermal_signature: float = 1.0   # normalized SWIR plume intensity

func _ready() -> void:
	rng.randomize()
	reset_track()

func reset_track() -> void:
	# Spawn outside EO range (45 km) but inside infrasound range so the
	# engagement starts cold but the action is on-screen quickly.
	var start_dist: float = 80000.0
	var bearing: float = rng.randf_range(0.0, TAU)
	truth_position_m = Vector3(cos(bearing) * start_dist, rng.randf_range(alt_band.x, alt_band.y), sin(bearing) * start_dist)
	# Inbound heading toward origin with a randomized offset.
	var inbound := -truth_position_m
	inbound.y = 0.0
	heading_rad = atan2(inbound.z, inbound.x) + rng.randf_range(-0.4, 0.4)
	velocity_mps = Vector3(cos(heading_rad), 0.0, sin(heading_rad)) * cruise_speed_mps
	_target_alt_m = rng.randf_range(alt_band.x, alt_band.y)
	_maneuver_timer = 0.0
	plasma_blackout = false

func _physics_process(delta: float) -> void:
	_step_evasion(delta)
	_step_kinematics(delta)
	_update_signatures(delta)
	_update_render_transform()

func _step_evasion(delta: float) -> void:
	# Randomized glide maneuvers: re-roll lateral bias and altitude target on
	# a Poisson-ish cadence. This is the "erratic" part the PDF asks for.
	_maneuver_timer -= delta
	if _maneuver_timer <= 0.0:
		_maneuver_timer = maneuver_period_s * rng.randf_range(0.4, 1.6)
		_lateral_bias = rng.randf_range(-1.0, 1.0)
		_target_alt_m = clamp(_target_alt_m + rng.randf_range(-altitude_jitter_m, altitude_jitter_m), alt_band.x, alt_band.y)

	# Apply lateral acceleration as a heading-rate change (coordinated turn).
	var accel_lateral: float = lateral_g * 9.81 * _lateral_bias
	var safe_speed: float = max(cruise_speed_mps, 1.0)
	var turn_rate: float = accel_lateral / safe_speed
	heading_rad += turn_rate * delta

func _step_kinematics(delta: float) -> void:
	# Hold cruise speed; nudge altitude toward _target_alt_m.
	var horiz := Vector3(cos(heading_rad), 0.0, sin(heading_rad)) * cruise_speed_mps
	var alt_err := _target_alt_m - truth_position_m.y
	var vy: float = clamp(alt_err * 0.15, -180.0, 180.0)   # ~600 m/s climb cap
	velocity_mps = Vector3(horiz.x, vy, horiz.z)
	truth_position_m += velocity_mps * delta

	# Recycle if the missile flies far past the loiter point.
	if truth_position_m.length() > 120000.0:
		reset_track()

func _update_signatures(_delta: float) -> void:
	# Plasma sheathing engages above Mach ~6 dynamic pressure or during
	# aggressive maneuvers. Approximated here from lateral g-load + speed.
	var maneuver_load: float = absf(_lateral_bias)
	plasma_blackout = (cruise_speed_mps > 1500.0) and (maneuver_load > 0.55)
	# SWIR thermal plume scales with speed cubed (drag heating proxy),
	# normalized to ~1.0 at HCM_CRUISE_MPS.
	thermal_signature = pow(cruise_speed_mps / SimConstants.HCM_CRUISE_MPS, 3.0)

func _update_render_transform() -> void:
	position = truth_position_m / SimConstants.RENDER_SCALE
	look_at(position + velocity_mps.normalized(), Vector3.UP)
