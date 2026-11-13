"""Reject toolhead paths through the rear-left brush clearance volume.

This module is kept in the printer config Git repository and linked into
Klipper's extras directory. The check is on the complete XYZ line segment,
after any G-code coordinate and bed-mesh transforms.
"""


def path_enters_zone(start, end, x_min, x_max, y_min, y_max, z_max):
    """Return whether an XYZ segment intersects the protected volume.

    The upper Z face is open: travel exactly at z_max is permitted. A purely
    vertical lift is always permitted so a toolhead can escape the zone.
    """
    if (start[0] == end[0] and start[1] == end[1]
            and end[2] > start[2]):
        return False
    if (start[0] == end[0] and start[1] == end[1]
            and start[2] == end[2]):
        return False
    if min(start[2], end[2]) >= z_max:
        return False

    lo, hi = 0.0, 1.0
    for axis, lower, upper in ((0, x_min, x_max), (1, y_min, y_max)):
        origin = start[axis]
        delta = end[axis] - origin
        if delta == 0.0:
            if origin < lower or origin > upper:
                return False
            continue
        first = (lower - origin) / delta
        last = (upper - origin) / delta
        lo = max(lo, min(first, last))
        hi = min(hi, max(first, last))
        if lo > hi:
            return False

    z_at_entry = start[2] + (end[2] - start[2]) * lo
    z_at_exit = start[2] + (end[2] - start[2]) * hi
    return min(z_at_entry, z_at_exit) < z_max


class RearBrushGuard:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.x_min = config.getfloat('x_min')
        self.x_max = config.getfloat('x_max')
        self.y_min = config.getfloat('y_min')
        self.y_max = config.getfloat('y_max')
        self.z_max = config.getfloat('z_max')
        if self.x_min >= self.x_max or self.y_min >= self.y_max:
            raise config.error('rear_brush_guard requires increasing XY bounds')
        self.toolhead = None
        self.gcode_move = None
        self.original_move = None
        self.original_drip_move = None
        self.printer.register_event_handler('klippy:ready', self.handle_ready)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('CHECK_REAR_BRUSH_GUARD', self.cmd_check,
                               desc='Check a proposed XYZ path without moving')

    def handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self.original_move = self.toolhead.move
        self.original_drip_move = self.toolhead.drip_move
        self.toolhead.move = self.guarded_move
        self.toolhead.drip_move = self.guarded_drip_move
        # Without bed_mesh, GcodeMove may hold the pre-guard bound method.
        if self.gcode_move.move_with_transform == self.original_move:
            self.gcode_move.move_with_transform = self.guarded_move

    def get_status(self, eventtime):
        return {
            'active': (self.toolhead is not None
                       and self.toolhead.move == self.guarded_move
                       and self.toolhead.drip_move == self.guarded_drip_move),
            'x_min': self.x_min, 'x_max': self.x_max,
            'y_min': self.y_min, 'y_max': self.y_max,
            'z_max': self.z_max,
        }

    def enters_zone(self, start, end):
        return path_enters_zone(start, end, self.x_min, self.x_max,
                                self.y_min, self.y_max, self.z_max)

    def check_move(self, end):
        start = self.toolhead.get_position()
        if not self.enters_zone(start, end):
            return
        # Klipper cannot know physical XYZ coordinates until all axes are
        # homed. Existing G28 and Beacon homing routines handle that phase.
        homed = self.toolhead.get_kinematics().get_status(0.)['homed_axes']
        if not all(axis in homed for axis in 'xyz'):
            return
        # GcodeMove updates its logical target before calling toolhead.move.
        # Restore it to the last accepted position after a rejected command.
        self.gcode_move.reset_last_position()
        raise self.printer.command_error(
            'Rear brush guard blocked XYZ move from '
            '(%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f); '
            'raise Z to %.1f before crossing X%.1f..%.1f Y%.1f..%.1f'
            % (start[0], start[1], start[2], end[0], end[1], end[2],
               self.z_max, self.x_min, self.x_max, self.y_min, self.y_max))

    def guarded_move(self, newpos, speed):
        self.check_move(newpos)
        return self.original_move(newpos, speed)

    def guarded_drip_move(self, newpos, speed, drip_completion):
        self.check_move(newpos)
        return self.original_drip_move(newpos, speed, drip_completion)

    def cmd_check(self, gcmd):
        start = [gcmd.get_float('X0'), gcmd.get_float('Y0'),
                 gcmd.get_float('Z0')]
        end = [gcmd.get_float('X1'), gcmd.get_float('Y1'),
               gcmd.get_float('Z1')]
        outcome = 'BLOCKED' if self.enters_zone(start, end) else 'allowed'
        gcmd.respond_info('Rear brush path %s: %s -> %s'
                          % (outcome, start, end))


def load_config(config):
    return RearBrushGuard(config)
