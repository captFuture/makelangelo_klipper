# Polar Drawing Machine Kinematics for Klipper
# Repository: https://github.com/captFuture/makelangelo_klipper
#
# Each stepper position = belt length in mm.
# Klipper moves both steppers together; we override calc_position
# and set_position to handle the polargraph math in Python.
# The C solver (cartesian x) is used only for step generation --
# it never sees actual XY coordinates, only belt lengths.
#
# Coordinate systems:
#   WORLD: origin at midpoint between motors on motor axis, Y increases down
#   DRAWING: origin at (draw_margin_left right of left motor,
#                       draw_margin_top  below motor axis)
#            X right, Y down -- matches Makelangelo software convention
#
# Homing:
#   Both motors drive simultaneously toward longer belt (positive direction).
#   Longer belt = gondola descends = counterweight rises to MAX endstop.
#   position_endstop = hypotenuse_length_at_home = belt length at endstop.
#   homing_positive_dir: true in both stepper configs.
#   homing_retract_dist: 0 (retract handled by G28 macro instead).
#
# Pen changer: placeholder for future carousel hardware.

import math
import logging
import stepper


class PolarDrawingKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()

        # ── Machine dimensions ────────────────────────────────────────────────

        self.motor_distance  = config.getfloat('motor_distance', above=0.)
        self.hypotenuse_home = config.getfloat('hypotenuse_length_at_home', 1035.0)
        self.max_belt_length = config.getfloat('max_belt_length', 1400.0)
        self.min_belt_length = config.getfloat('min_belt_length', 100.0)

        # ── Drawing origin ────────────────────────────────────────────────────

        self.draw_margin_left = config.getfloat('draw_margin_left', 115.0)
        self.draw_margin_top  = config.getfloat('draw_margin_top',  115.0)
        self.draw_width       = config.getfloat('draw_width',  594.0)
        self.draw_height      = config.getfloat('draw_height', 420.0)
        self.homing_speed     = config.getfloat('homing_speed', 50.0)

        # ── Derived geometry ──────────────────────────────────────────────────

        half_w = self.motor_distance / 2.0
        # Drawing origin in world coordinates
        self.draw_origin_wx = -half_w + self.draw_margin_left
        self.draw_origin_wy =  self.draw_margin_top
        # World Y of gondola when both endstops trigger
        self.home_y_world   = math.sqrt(
            max(self.hypotenuse_home**2 - half_w**2, 0.0))
        # Gondola position in DRAWING coordinates after homing
        self.homed_drawing_x = 0.0             - self.draw_origin_wx
        self.homed_drawing_y = self.home_y_world - self.draw_origin_wy

        # ── Load rails ────────────────────────────────────────────────────────
        # stepper position = belt length in mm
        # cartesian_stepper_alloc with 'x' is used only for step generation;
        # all coordinate math is done in Python (calc_position / set_position).

        self.rails = [
            stepper.LookupMultiRail(config.getsection('stepper_left')),
            stepper.LookupMultiRail(config.getsection('stepper_right')),
        ]
        for rail in self.rails:
            rail.setup_itersolve('cartesian_stepper_alloc', b'x')
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())

        self.is_homing = False

        # Drawing area for Mainsail display
        self.axes_min = toolhead.Coord([0., 0., 0., 0.])
        self.axes_max = toolhead.Coord([self.draw_width, self.draw_height, 0., 0.])

        # ── Pen changer placeholder ───────────────────────────────────────────
        self._init_pen_changer(config)

        # Register so macros can access printer['polardrawing'].xxx
        self.printer.add_object('polardrawing', self)

        logging.info(
            "PolarDrawing: motor_distance=%.1f  hypotenuse_home=%.1f  "
            "home_y_world=%.1f  homed_drawing=(%.1f, %.1f)  "
            "draw_area=%.0fx%.0f mm",
            self.motor_distance, self.hypotenuse_home,
            self.home_y_world, self.homed_drawing_x, self.homed_drawing_y,
            self.draw_width, self.draw_height)

    # ── Pen changer placeholder ───────────────────────────────────────────────

    def _init_pen_changer(self, config):
        """
        Future hardware:
          pen_changer_angle_sensor_pin -- ADC/encoder for carousel position
          pen_changer_servo_pin        -- 360-deg continuous rotation servo
        All keys are optional; omit to disable.
        """
        self.pen_changer_angle_pin = config.get('pen_changer_angle_sensor_pin', None)
        self.pen_changer_servo_pin = config.get('pen_changer_servo_pin', None)
        self.pen_count             = config.getint('pen_count', 4, minval=1)
        self.current_pen           = 0
        self.pen_changer_enabled   = (self.pen_changer_angle_pin is not None
                                      and self.pen_changer_servo_pin is not None)

    def _do_pen_change(self, target_pen):
        """
        PLACEHOLDER: rotate carousel to target_pen slot.
        Implement when angle sensor and servo are fitted.
        """
        logging.info("PolarDrawing: pen change %d -> %d (NOT YET IMPLEMENTED)",
                     self.current_pen, target_pen)
        self.current_pen = target_pen

    # ── Inverse kinematics: drawing XY -> belt lengths ────────────────────────

    def _xy_to_belts(self, dx, dy):
        """Convert drawing position to (left_belt_mm, right_belt_mm)."""
        wx     = self.draw_origin_wx + dx
        wy     = self.draw_origin_wy + dy
        half_w = self.motor_distance / 2.0
        left_l  = math.sqrt((wx + half_w)**2 + wy**2)
        right_l = math.sqrt((wx - half_w)**2 + wy**2)
        return left_l, right_l

    # ── Forward kinematics: belt lengths -> drawing XY ────────────────────────

    def _belts_to_xy(self, left_l, right_l):
        """Convert (left_belt_mm, right_belt_mm) to drawing position."""
        W           = self.motor_distance
        x_from_left = (left_l**2 - right_l**2 + W**2) / (2.0 * W)
        y_sq        = left_l**2 - x_from_left**2
        world_y     = math.sqrt(max(y_sq, 0.0))
        world_x     = x_from_left - W / 2.0
        return world_x - self.draw_origin_wx, world_y - self.draw_origin_wy

    # ── Klipper kinematics interface ──────────────────────────────────────────

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        """
        Called by Klipper to compute toolhead XYZ from stepper positions.
        Stepper positions are belt lengths in mm.
        """
        left_l  = stepper_positions[self.rails[0].get_name()]
        right_l = stepper_positions[self.rails[1].get_name()]
        dx, dy  = self._belts_to_xy(left_l, right_l)
        return [dx, dy, 0.0]

    def set_position(self, newpos, homing_axes):
        """
        Called by Klipper to force a known position (drawing coordinates).
        Converts to belt lengths and sets each stepper directly.
        """
        dx, dy          = newpos[0], newpos[1]
        left_l, right_l = self._xy_to_belts(dx, dy)
        for s in self.rails[0].get_steppers():
            s.set_position([left_l,  0., 0.])
        for s in self.rails[1].get_steppers():
            s.set_position([right_l, 0., 0.])

    def home(self, homing_state):
        """
        Home both motors simultaneously.

        Stepper position = belt length in mm.
        forcepos = position_min (short belt, gondola high, counterweight low)
        homepos  = position_endstop = hypotenuse_home (longer belt,
                   gondola lower, counterweight UP to MAX endstop)

        homing_positive_dir: true in config drives from forcepos to homepos
        (increasing belt length = counterweight rises).

        homing_retract_dist: 0 in config disables Klipper's built-in retract,
        which does not work correctly for polargraph geometry.
        The G28 macro handles moving away from the endstop after homing.
        """
        self.is_homing = True

        axes = [a for a in homing_state.get_axes() if a < len(self.rails)]
        if axes:
            hi      = self.rails[0].get_homing_info()
            pos_min = self.rails[0].get_range()[0]
            pos_end = hi.position_endstop

            # Both rails in a single home_rails call = simultaneous homing.
            # forcepos[0] = left belt start, forcepos[1] = right belt start.
            forcepos = [pos_min, pos_min, 0., 0.]
            homepos  = [pos_end, pos_end, 0., 0.]

            homing_state.home_rails(
                [self.rails[a] for a in axes], forcepos, homepos)

        self.is_homing = False

        # Override toolhead position to drawing coordinates.
        # Without this Klipper would report belt-length coordinates.
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.set_position(
            [self.homed_drawing_x, self.homed_drawing_y, 0., 0.],
            homing_axes='xy')
        logging.info(
            "PolarDrawing: homing complete, drawing pos=(%.2f, %.2f)",
            self.homed_drawing_x, self.homed_drawing_y)

    def clear_homing_state(self, clear_axes):
        pass

    def check_move(self, move):
        """Reject moves outside the drawing area (not checked during homing)."""
        if self.is_homing:
            return
        dx, dy = move.end_pos[0], move.end_pos[1]
        if (dx < 0. or dx > self.draw_width or
                dy < 0. or dy > self.draw_height):
            raise move.move_error(
                "PolarDrawing: (%.2f, %.2f) outside drawing area "
                "X=[0, %.0f] Y=[0, %.0f]"
                % (dx, dy, self.draw_width, self.draw_height))

    def get_status(self, eventtime):
        """Expose machine state to macros via printer['polardrawing']."""
        return {
            'homed_axes':          'xy',
            'axis_minimum':        self.axes_min,
            'axis_maximum':        self.axes_max,
            'motor_distance':      self.motor_distance,
            'hypotenuse_home':     self.hypotenuse_home,
            'home_y_world':        self.home_y_world,
            'draw_origin_wx':      self.draw_origin_wx,
            'draw_origin_wy':      self.draw_origin_wy,
            'homed_drawing_x':     self.homed_drawing_x,
            'homed_drawing_y':     self.homed_drawing_y,
            'draw_width':          self.draw_width,
            'draw_height':         self.draw_height,
            'current_pen':         self.current_pen,
            'pen_count':           self.pen_count,
            'pen_changer_enabled': self.pen_changer_enabled,
        }


def load_kinematics(toolhead, config):
    """Entry point called by Klipper when 'kinematics: polardrawing' is set."""
    return PolarDrawingKinematics(toolhead, config.getsection('polardrawing'))