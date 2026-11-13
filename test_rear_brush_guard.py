import importlib.util
import pathlib
import unittest


module_path = pathlib.Path(__file__).with_name('rear_brush_guard.py')
spec = importlib.util.spec_from_file_location('rear_brush_guard', module_path)
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


def blocked(start, end):
    return guard.path_enters_zone(start, end, 55, 132, 290, 343, 20)


class RearBrushGuardGeometryTests(unittest.TestCase):
    def test_crossing_between_safe_endpoints_is_blocked(self):
        self.assertTrue(blocked((40, 320, 2), (150, 320, 2)))
        self.assertTrue(blocked((175, 175, 2), (100, 320, 2)))

    def test_diagonal_xyz_path_is_checked_at_intersection(self):
        self.assertTrue(blocked((40, 320, 20), (150, 320, 2)))
        self.assertFalse(blocked((40, 320, 20), (150, 320, 20)))
        self.assertFalse(blocked((40, 320, 22), (150, 320, 20)))

    def test_z_trim_requires_margin_on_travel_moves(self):
        # PRINT_START applies a -0.03 mm Z trim after contact homing.
        self.assertTrue(blocked((175, 155, 19.97), (50, 350, 19.97)))
        self.assertFalse(blocked((175, 155, 20.97), (50, 350, 20.97)))

    def test_lowering_inside_zone_is_blocked_but_raising_is_allowed(self):
        self.assertTrue(blocked((100, 320, 20), (100, 320, 2)))
        self.assertFalse(blocked((100, 320, 2), (100, 320, 20)))

    def test_cleaning_and_print_routes_are_allowed(self):
        self.assertFalse(blocked((71, 350, 20), (71, 350, 5.2)))
        self.assertFalse(blocked((71, 350, 5.2), (107, 344, 5.2)))
        self.assertFalse(blocked((107, 344, 5.2), (71, 344, 5.2)))
        self.assertFalse(blocked((89, 347, 20), (175, 175, 20)))
        self.assertFalse(blocked((89, 347, 20), (140, 347, 20)))
        self.assertFalse(blocked((140, 347, 20), (140, 175, 20)))
        self.assertFalse(blocked((89, 347, 11), (168, 348, 11)))

    def test_probe_locations_and_extrusion_only_are_allowed(self):
        self.assertFalse(blocked((25, 280, 5), (310, 280, 5)))
        self.assertFalse(blocked((100, 280, 5), (100, 280, 5)))

    def test_edges_and_measured_positions(self):
        self.assertTrue(blocked((57, 293, 2), (130, 293, 2)))
        self.assertTrue(blocked((130, 343, 6), (130, 342, 6)))
        self.assertFalse(blocked((130, 344, 5.2), (71, 344, 5.2)))

    def test_rejected_move_resets_gcode_position(self):
        class FakeKinematics:
            def get_status(self, eventtime):
                return {'homed_axes': 'xyz'}

        class FakeToolhead:
            def get_position(self):
                return [40, 320, 2, 0]

            def get_kinematics(self):
                return FakeKinematics()

        class FakeGcodeMove:
            resets = 0

            def reset_last_position(self):
                self.resets += 1

        class FakePrinter:
            command_error = RuntimeError

        obj = object.__new__(guard.RearBrushGuard)
        obj.x_min, obj.x_max = 55, 132
        obj.y_min, obj.y_max = 290, 343
        obj.z_max = 20
        obj.toolhead = FakeToolhead()
        obj.gcode_move = FakeGcodeMove()
        obj.printer = FakePrinter()
        with self.assertRaisesRegex(RuntimeError, 'Rear brush guard blocked'):
            obj.check_move([150, 320, 2, 0])
        self.assertEqual(obj.gcode_move.resets, 1)


if __name__ == '__main__':
    unittest.main()
