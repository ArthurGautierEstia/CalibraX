import unittest
from pathlib import Path

from models.tool_config_file import ToolConfigFile


class ToolConfigFileTests(unittest.TestCase):
    def test_missing_evaluated_robot_axis_colliders_is_rejected(self):
        with self.assertRaises(ValueError):
            ToolConfigFile.from_dict(
                {
                    "name": "Demo",
                    "tool": {"x": 1, "y": 2, "z": 3, "a": 4, "b": 5, "c": 6},
                    "tool_cad_model": "",
                    "tool_cad_offset_rz": 0.0,
                    "tool_colliders": [],
                }
            )

    def test_save_and_reload_preserves_evaluated_robot_axis_colliders(self):
        output_path = Path("tests") / "_tmp_tool_profile.json"
        profile = ToolConfigFile.from_dict(
            {
                "name": "Demo",
                "tool": {"x": 0, "y": 0, "z": 0, "a": 0, "b": 0, "c": 0},
                "tool_cad_model": "",
                "tool_cad_offset_rz": 0.0,
                "tool_colliders": [],
                "evaluated_robot_axis_colliders": [False, False, False, True, True, True],
            }
        )

        try:
            profile.save(str(output_path))
            reloaded = ToolConfigFile.load(str(output_path))

            self.assertEqual(
                [False, False, False, True, True, True],
                reloaded.evaluated_robot_axis_colliders,
            )
        finally:
            output_path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
