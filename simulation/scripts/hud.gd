class_name HUD
extends CanvasLayer

var _uav_labels: Dictionary = {}
var _uav_bars: Dictionary = {}
var _fused_label: Label
var _fused_bar: ProgressBar
var _status_label: Label
var _mode_label: Label
var _uav_container: VBoxContainer

func _ready() -> void:
	_build_ui()

func _build_ui() -> void:
	var panel := PanelContainer.new()
	panel.set_anchors_preset(Control.PRESET_TOP_LEFT)
	panel.position = Vector2(12, 12)
	panel.custom_minimum_size = Vector2(320, 0)
	add_child(panel)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override("separation", 4)
	panel.add_child(vbox)

	var title := Label.new()
	title.text = "UAV SENSOR FUSION SYSTEM"
	title.add_theme_font_size_override("font_size", 15)
	vbox.add_child(title)

	vbox.add_child(_separator())

	_mode_label = Label.new()
	_mode_label.text = "Trajectory: STRAIGHT"
	vbox.add_child(_mode_label)

	_status_label = Label.new()
	_status_label.text = "Status: SCANNING"
	vbox.add_child(_status_label)

	vbox.add_child(_separator())

	var sec_label := Label.new()
	sec_label.text = "PER-UAV DETECTIONS"
	sec_label.add_theme_font_size_override("font_size", 12)
	vbox.add_child(sec_label)

	_uav_container = VBoxContainer.new()
	_uav_container.add_theme_constant_override("separation", 6)
	vbox.add_child(_uav_container)

	vbox.add_child(_separator())

	var fused_title := Label.new()
	fused_title.text = "SYSTEM CONFIDENCE (fused)"
	fused_title.add_theme_font_size_override("font_size", 13)
	vbox.add_child(fused_title)

	_fused_label = Label.new()
	_fused_label.text = "0.00"
	_fused_label.add_theme_font_size_override("font_size", 22)
	vbox.add_child(_fused_label)

	_fused_bar = _make_bar()
	vbox.add_child(_fused_bar)

	vbox.add_child(_separator())

	var controls := Label.new()
	controls.text = "[L] Launch  [R] Reset  [T] Trajectory mode\n[1] Side  [2] Top  [3] Chase"
	controls.add_theme_color_override("font_color", Color(0.65, 0.65, 0.65))
	controls.add_theme_font_size_override("font_size", 11)
	vbox.add_child(controls)

func register_uav(uav_id: int) -> void:
	var row := VBoxContainer.new()
	_uav_container.add_child(row)

	var label := Label.new()
	label.text = "UAV %d — IR:0.00 OPT:0.00 PRS:0.00  fused:0.00" % uav_id
	label.add_theme_font_size_override("font_size", 11)
	row.add_child(label)

	var bar := _make_bar()
	bar.custom_minimum_size.y = 8
	row.add_child(bar)

	_uav_labels[uav_id] = label
	_uav_bars[uav_id] = bar

func update_uav_detection(uav_id: int, data: Dictionary) -> void:
	if not _uav_labels.has(uav_id):
		return
	var label: Label = _uav_labels[uav_id]
	var bar: ProgressBar = _uav_bars[uav_id]
	var fused: float = data.get("fused", 0.0)
	label.text = "UAV %d — IR:%.2f OPT:%.2f PRS:%.2f  fused:%.2f" % [
		uav_id,
		data.get("ir", 0.0),
		data.get("optical", 0.0),
		data.get("pressure", 0.0),
		fused,
	]
	bar.value = fused * 100.0
	if fused >= 0.7:
		label.add_theme_color_override("font_color", Color.RED)
	elif fused >= DataFusion.DETECTION_THRESHOLD:
		label.add_theme_color_override("font_color", Color.YELLOW)
	else:
		label.add_theme_color_override("font_color", Color.WHITE)

func update_fused_confidence(confidence: float) -> void:
	_fused_label.text = "%.2f" % confidence
	_fused_bar.value = confidence * 100.0
	if confidence >= DataFusion.DETECTION_THRESHOLD:
		_fused_label.add_theme_color_override("font_color", Color.RED)
		_status_label.text = "Status: TARGET DETECTED"
		_status_label.add_theme_color_override("font_color", Color.RED)
	else:
		_fused_label.add_theme_color_override("font_color", Color.WHITE)
		_status_label.text = "Status: SCANNING"
		_status_label.add_theme_color_override("font_color", Color.WHITE)

func update_trajectory_mode(mode_name: String) -> void:
	_mode_label.text = "Trajectory: " + mode_name

# ── helpers ──────────────────────────────────────────────────────────────────

func _separator() -> HSeparator:
	return HSeparator.new()

func _make_bar() -> ProgressBar:
	var bar := ProgressBar.new()
	bar.min_value = 0.0
	bar.max_value = 100.0
	bar.value = 0.0
	bar.custom_minimum_size = Vector2(0, 14)
	bar.show_percentage = false
	return bar
