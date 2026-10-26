import json
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class RightTurnReferenceTests(unittest.TestCase):
    def test_generated_frames_match_supplied_json_exactly(self):
        data = json.loads((ROOT / "Actions" / "my_actionsfinalTURNRIGHT.json").read_text(encoding="utf-8-sig"))
        header = (ROOT / "Inc" / "right_turn_frames.h").read_text(encoding="utf-8")
        for frame in data["frames"]:
            positions = ", ".join(str(frame["positions"][str(i)]) for i in range(1, 15))
            row = "{{%s}, %dU, %dU, %dU, %dU" % (
                positions, frame["duration_ms"], frame["hold_ms"], frame["speed"], frame["acc"]
            )
            self.assertIn(row, header)

    def test_right_angle_changes_only_support_hip(self):
        motion = (ROOT / "Src" / "robot_motion.c").read_text(encoding="utf-8")
        self.assertIn("s_motion = k_right_turn_frames[right_index];", motion)
        self.assertIn("s_motion.position[hip] = (uint16_t)(2048 + s_requested_correction);", motion)
        block = motion.split("} else if (use_right) {", 1)[1].split("} else {", 1)[0]
        writes = re.findall(r"s_motion\.position\[([^\]]+)\]\s*=", block)
        self.assertEqual(writes, ["hip"])


if __name__ == "__main__":
    unittest.main()
