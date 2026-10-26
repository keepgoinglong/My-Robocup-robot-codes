"""Generate the six-frame right-turn baseline without editing the supplied JSON."""
from pathlib import Path
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Actions" / "my_actionsfinalTURNRIGHT.json"
OUTPUT = ROOT / "Inc" / "right_turn_frames.h"

data = json.loads(SOURCE.read_text(encoding="utf-8-sig"))
gait = json.loads((ROOT / "Actions" / "my_actionsfinalorigin4.json").read_text(encoding="utf-8-sig"))
assert data["period_ms"] == 10
assert len(data["frames"]) == 6

lines = [
    "/* Generated from Actions/my_actionsfinalTURNRIGHT.json; do not hand-edit. */",
    "/* SHA256: %s */" % hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    "#ifndef RIGHT_TURN_FRAMES_H",
    "#define RIGHT_TURN_FRAMES_H",
    "#define RIGHT_TURN_FRAME_COUNT 6U",
    "static const ActionFrame k_right_turn_frames[RIGHT_TURN_FRAME_COUNT] = {",
]
delays = []
durations = []
for index, frame in enumerate(data["frames"]):
    positions = [frame["positions"][str(i)] for i in range(1, 15)]
    for joint, position in enumerate(positions, 1):
        lo, hi = data["limits"][str(joint)]
        assert lo <= position <= hi
        gait_lo, gait_hi = gait["limits"][str(joint)]
        assert gait_lo <= position <= gait_hi, (index, joint, position)
    expected_hip = 3 if index < 3 else 2
    assert positions[expected_hip - 1] == 1948
    assert positions[(2 if expected_hip == 3 else 3) - 1] == 2048
    lines.append(
        "  {{%s}, %dU, %dU, %dU, %dU, 0U, %dU},"
        % (
            ", ".join(map(str, positions)),
            frame["duration_ms"], frame["hold_ms"], frame["speed"], frame["acc"],
            int(index in (1, 4)),
        )
    )
    timing = frame.get("timing", {})
    delays.append([timing.get(str(i), {}).get("delay_ms", 0) for i in range(1, 15)])
    durations.append([
        timing.get(str(i), {}).get("duration_ms", frame["duration_ms"])
        for i in range(1, 15)
    ])
lines.append("};")
lines.append("static const uint32_t k_right_turn_delay[RIGHT_TURN_FRAME_COUNT][ROBOT_SERVO_COUNT] = {")
for row in delays:
    lines.append("  {%s}," % ", ".join("%dU" % value for value in row))
lines.append("};")
lines.append("static const uint32_t k_right_turn_joint_duration[RIGHT_TURN_FRAME_COUNT][ROBOT_SERVO_COUNT] = {")
for row in durations:
    lines.append("  {%s}," % ", ".join("%dU" % value for value in row))
lines.extend(["};", "#endif", ""])
OUTPUT.write_text("\n".join(lines), encoding="utf-8")
print("Generated exact six-frame right-turn baseline:", OUTPUT)
