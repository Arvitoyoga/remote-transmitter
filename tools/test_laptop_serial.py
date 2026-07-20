import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from laptop_serial import map_sim_to_joystick_data, parse_sim_line


class LaptopSerialTests(unittest.TestCase):
    def test_parse_sim_line_maps_all_finger_buttons(self):
        data = parse_sim_line("SIM,1000,200,300,400,1,0,1,0,1,0,1,0")
        self.assertIsNotNone(data)

        joystick = map_sim_to_joystick_data(data)
        self.assertAlmostEqual(joystick["left_x"], -((400 / 1984.0) * 2.0 - 1.0))
        self.assertAlmostEqual(joystick["left_y"], -((1000 / 1984.0) * 2.0 - 1.0))
        self.assertAlmostEqual(joystick["right_x"], -((200 / 1984.0) * 2.0 - 1.0))
        self.assertAlmostEqual(joystick["right_y"], -((300 / 1984.0) * 2.0 - 1.0))
        self.assertTrue(joystick["left_point"])
        self.assertTrue(joystick["left_ring"])
        self.assertTrue(joystick["right_point"])
        self.assertTrue(joystick["right_ring"])
        self.assertFalse(joystick["left_middle"])
        self.assertFalse(joystick["right_middle"])


if __name__ == "__main__":
    unittest.main()
