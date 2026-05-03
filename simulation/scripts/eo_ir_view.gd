extends Node3D
class_name EOIRView

# ---------------------------------------------------------------------------
# 360° azimuth camera rig. Four wide-angle (rectilinear) cameras tile the
# horizon with overlap: yaw offsets 0°/90°/180°/-90° relative to glider
# heading, FOV ~100° each → 400° of azimuth coverage (≈25° overlap per
# seam, no blind sector). Each camera is pitched slightly downward so the
# HCM operating band (18–25 km altitude, well below the 60–90 kft glider)
# falls near the centre of the frame.
#
# Why not fisheye/tetrahedral: at FOV ≥160° Godot's perspective projection
# warps distant point-targets into invisible smears at the edges, and two
# of the four lobes pointed at empty sky/ground. The azimuth tiling keeps
# every camera rectilinear (clear, undistorted projection) while still
# covering the full ring around the airframe — which is the only direction
# an HCM can approach from.
#
# The four EO viewports collapse to ONE panoramic descriptor at
# sensor_fusion / ml_bridge time (same for the four IR viewports), feeding
# one CNN per band in model_v2.ArgusFusionNet — matching architecture.svg.
# ---------------------------------------------------------------------------

@export var hcm_path: NodePath
@export var argus_path: NodePath

# Per-camera FOV (vertical, degrees). 4 × 100° → ~400° azimuth, ~25° overlap.
const CAM_FOV_DEG: float = 100.0
# Far clip — must reach the HCM (~110 km away at first detection).
const CAM_FAR: float = 250000.0
# Near clip set BEYOND the airframe's bounding sphere (wing half-span ≈ 18 m,
# fuselage half-length ≈ 3.5 m). With near = 25 m, no part of the glider can
# ever enter any camera's frustum, so wing/fuselage clipping artifacts are
# physically impossible regardless of where the cameras sit.
const CAM_NEAR: float = 25.0
# Downward pitch so the HCM band sits centred in frame, not at the bottom.
const CAM_PITCH_RAD: float = -0.26  # ~-15°

# Yaw offsets (radians) from glider heading: front, starboard, aft, port.
const YAW_OFFSETS := [0.0, PI * 0.5, PI, -PI * 0.5]
const TAGS := ["F", "R", "B", "L"]

# All four cameras co-mount at the airframe centre. Because CAM_NEAR (25 m)
# is larger than the airframe's bounding sphere, mount offsets are unnecessary
# for occlusion safety — keeping them at zero also stops parallax jitter when
# the glider banks.
const MOUNT_OFFSET_M: float = 0.0

var hcm: Node3D
var argus: Node3D
var eo_cams: Array[Camera3D] = []
var ir_cams: Array[Camera3D] = []

func _ready() -> void:
	hcm = get_node_or_null(hcm_path) as Node3D
	argus = get_node_or_null(argus_path) as Node3D
	for tag in TAGS:
		var eov := get_node_or_null("EOViewport_%s" % tag) as SubViewport
		var irv := get_node_or_null("IRViewport_%s" % tag) as SubViewport
		if eov:
			var c := eov.get_node_or_null("EOCamera_%s" % tag) as Camera3D
			if c:
				_configure_camera(c)
				eo_cams.append(c)
		if irv:
			var c := irv.get_node_or_null("IRCamera_%s" % tag) as Camera3D
			if c:
				_configure_camera(c)
				ir_cams.append(c)

func _configure_camera(c: Camera3D) -> void:
	c.fov = CAM_FOV_DEG
	c.near = CAM_NEAR
	c.far = CAM_FAR
	c.keep_aspect = Camera3D.KEEP_HEIGHT
	# Render only layer 1; HUD-only objects (EstimateMarker, debug glyphs)
	# live on layer 2 so they don't pollute the EO/IR feeds.
	c.cull_mask = 1

func _process(_dt: float) -> void:
	if argus == null:
		return
	# Body-fixed azimuth tiling: each lobe sits at heading + YAW_OFFSETS[i]
	# with a slight downward pitch. No target tracking — orientation depends
	# only on the glider's heading, so the rig provides genuine 360° coverage.
	var heading: float = argus.heading_rad if "heading_rad" in argus else 0.0
	var cp: float = cos(CAM_PITCH_RAD)
	var sp: float = sin(CAM_PITCH_RAD)
	for i in YAW_OFFSETS.size():
		var yaw: float = heading + YAW_OFFSETS[i]
		var dir_world := Vector3(cos(yaw) * cp, sp, sin(yaw) * cp)
		var cam_pos: Vector3 = argus.global_position
		var aim: Vector3 = cam_pos + dir_world * 1000.0
		if i < eo_cams.size():
			eo_cams[i].global_position = cam_pos
			eo_cams[i].look_at(aim, Vector3.UP)
		if i < ir_cams.size():
			ir_cams[i].global_position = cam_pos
			ir_cams[i].look_at(aim, Vector3.UP)
