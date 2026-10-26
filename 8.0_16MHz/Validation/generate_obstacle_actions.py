"""Generate the small-step and obstacle-over action tables without editing JSON sources."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load(name, expected):
    path = ROOT / "Actions" / name
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    assert data["period_ms"] == 10 and len(data["frames"]) == expected
    assert set(data["limits"]) == {str(i) for i in range(1, 15)}
    return path, data


def timing(frame, servo_id):
    item = frame.get("timing", {}).get(str(servo_id), {})
    return int(item.get("delay_ms", 0)), int(item.get("duration_ms", frame["duration_ms"]))


def emit_table(lines, prefix, data):
    macro = prefix[2:].upper() if prefix.startswith("k_") else prefix.upper()
    lines.append("static const ActionFrame %s_frames[%s_FRAME_COUNT] = {" % (prefix, macro))
    for index, frame in enumerate(data["frames"]):
        pos = [int(frame["positions"][str(i)]) for i in range(1, 15)]
        for i, value in enumerate(pos, 1):
            lo, hi = data["limits"][str(i)]
            assert lo <= value <= hi
        duration = max([int(frame["duration_ms"])] + [sum(timing(frame, i)) for i in range(1, 15)])
        landing = int("落地" in frame["name"])
        lines += ["  /* %d: %s */" % (index, frame["name"]),
                  "  {{%s}, %dU, %dU, %dU, %dU, %dU, %dU}," % (
                      ", ".join(map(str, pos)), duration, int(frame["hold_ms"]),
                      int(frame["speed"]), int(frame["acc"]), int(bool(frame["wait_reached"])), landing)]
    lines.append("};")
    for suffix, select in (("delay", 0), ("joint_duration", 1)):
        lines.append("static const uint32_t %s_%s[%s_FRAME_COUNT][ROBOT_SERVO_COUNT] = {" %
                     (prefix, suffix, macro))
        for frame in data["frames"]:
            lines.append("  {%s}," % ", ".join("%dU" % timing(frame, i)[select] for i in range(1, 15)))
        lines.append("};")


def generate():
    small_path, small = load("my_actionsfinalshort3.json", 6)
    over_path, over = load("my_actions strideover4.json", 16)
    lines = ["/* Generated from the two user-supplied JSON files; do not hand-edit targets/timing. */",
             "/* small SHA256: %s */" % hashlib.sha256(small_path.read_bytes()).hexdigest(),
             "/* over  SHA256: %s */" % hashlib.sha256(over_path.read_bytes()).hexdigest(),
             "#ifndef OBSTACLE_ACTION_FRAMES_H", "#define OBSTACLE_ACTION_FRAMES_H",
             "#include \"action_frames.h\"", "#define SMALL_FRAME_COUNT 6U",
             "#define OVER_FRAME_COUNT 16U", "#define SMALL_RIGHT_LANDING 1U",
             "#define SMALL_LEFT_LANDING 4U"]
    lines.append("static const uint16_t k_obstacle_limits[ROBOT_SERVO_COUNT][2] = {")
    for i in range(1, 15):
        lo = min(small["limits"][str(i)][0], over["limits"][str(i)][0])
        hi = max(small["limits"][str(i)][1], over["limits"][str(i)][1])
        lines.append("  {%d, %d}, /* ID%d */" % (lo, hi, i))
    lines.append("};")
    emit_table(lines, "k_small", small)
    emit_table(lines, "k_over", over)
    lines += ["#endif", ""]
    (ROOT / "Inc" / "obstacle_action_frames.h").write_text("\n".join(lines), encoding="utf-8")
    print("Generated 6 small-step and 16 obstacle-over frames.")


if __name__ == "__main__":
    generate()
