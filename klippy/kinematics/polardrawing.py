# Polar Drawing Machine Kinematics for Klipper
# Project: makelangelo_klipper
# Repository: https://github.com/captFuture/makelangelo_klipper
#
# FILE PLACEMENT:
#   Copy to: ~/klipper/klippy/kinematics/polardrawing.py
#
# ── Coordinate systems ────────────────────────────────────────────────────────
#
#  WORLD coordinates  (internal, used for belt math only)
#    Origin:   midpoint between motors, on the motor axis
#    Left motor  at world (-motor_distance/2, 0)
#    Right motor at world (+motor_distance/2, 0)
#    Y increases downward
#
#  DRAWING coordinates  (what G-code sees, what Klipper reports)
#    Origin (0,0): draw_margin_left mm right of left motor,
#                  draw_margin_top  mm below motor axis
#    X increases right, Y increases down  (same orientation as world)
#    In world:  draw_origin_wx = -motor_distance/2 + draw_margin_left
#               draw_origin_wy =  draw_margin_top
#
#  Relationship:
#    world_x = draw_origin_wx + drawing_x
#    world_y = draw_origin_wy + drawing_y
#
# ── Homing ────────────────────────────────────────────────────────────────────
#
#  Both motors drive toward shorter belt simultaneously.
#  Counterweights rise and hit MAX endstops.
#  Belt length at trigger = hypotenuse_length_at_home.
#  Homed world position = (0, home_y_world)
#  Klipper is told the DRAWING coordinates of that position.
#
# ── Belt direction ────────────────────────────────────────────────────────────
#
#  Gondola moves DOWN  -> belt length INCREASES -> counterweight UP
#  Gondola moves UP    -> belt length DECREASES -> counterweight DOWN
#  Homing drives toward SHORTER belt (gondola up, counterweight up to endstop)
#
# ── Pen changer (placeholder) ─────────────────────────────────────────────────
#
#  Future hardware:
#    - Angle sensor (potentiometer or encoder) for carousel position feedback
#    - 360-degree continuous rotation servo to rotate the carousel
#  See _init_pen_changer() and _do_pen_change() below.

import math
import logging
import stepper
import homing


class PolarDrawingKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()

        # ── Machine dimensions ────────────────────────────────────────────────

        # Distance between motor shaft centres (mm).
        self.motor_distance = config.getfloat('motor_distance', above=0.)

        # Belt length when both counterweights trigger MAX endstops (mm).
        self.hypotenuse_home = config.getfloat(
            'hypotenuse_length_at_home', 1035.0, above=0.)

        # Maximum belt length (gondola at drawing bottom).
        self.max_belt_length = config.getfloat(
            'max_belt_length', self.hypotenuse_home + 300.0, above=0.)

        # Physical minimum belt length (safety limit).
        self.min_belt_length = config.getfloat('min_belt_length', 100.0, above=0.)

        # ── Drawing origin offset ─────────────────────────────────────────────

        self.draw_margin_left = config.getfloat('draw_margin_left', 115.0, above=0.)
        self.draw_margin_top  = config.getfloat('draw_margin_top',  115.0, above=0.)
        self.draw_width       = config.getfloat('draw_width',  594.0, above=0.)
        self.draw_height      = config.getfloat('draw_height', 420.0, above=0.)

        # Homing speed (mm/s).
        self.homing_speed = config.getfloat('homing_speed', 50.0, above=0.)

        # Retract distance after endstop triggers (mm).
        self.homing_retract_dist = config.getfloat('homing_retract_dist', 10.0)

        # ── Derived geometry ──────────────────────────────────────────────────

        half_w = self.motor_distance / 2.0
        self.draw_origin_wx = -half_w + self.draw_margin_left
        self.draw_origin_wy =  self.draw_margin_top
        self.home_y_world   = math.sqrt(
            max(self.hypotenuse_home ** 2 - half_w ** 2, 0.0))
        self.homed_drawing_x = 0.0             - self.draw_origin_wx
        self.homed_drawing_y = self.home_y_world - self.draw_origin_wy

        # ── Load steppers (winch.py pattern -- PrinterStepper) ───────────────
        # PrinterStepper is what this Klipper version uses for cable kinematics.
        # endstop_pin, position_min/max/endstop, homing_positive_dir are read
        # from the stepper config sections and used explicitly during homing.

        ppins = self.printer.lookup_object('pins')
        trapq = toolhead.get_trapq()
        self.steppers = []
        self.endstops = []
        self.endstop_names = []

        for name, axis in [('stepper_left', b'x'), ('stepper_right', b'y')]:
            sc = config.getsection(name)
            s = stepper.PrinterStepper(sc)
            s.setup_itersolve('cartesian_stepper_alloc', axis)
            s.set_trapq(trapq)
            self.steppers.append(s)

            # Set stepper position limits from config
            # (position_min, position_max, position_endstop)
            s.set_position([sc.getfloat('position_endstop'), 0., 0.])

            # Register endstop pin manually
            endstop_pin = sc.get('endstop_pin')
            mcu_endstop = ppins.setup_pin('endstop', endstop_pin)
            mcu_endstop.add_stepper(s)
            self.endstops.append(mcu_endstop)
            self.endstop_names.append(name)

        # ── Pen changer placeholder ───────────────────────────────────────────
        self._init_pen_changer(config)

        # ── Axes min/max for Mainsail display ─────────────────────────────────
        self.axes_min = toolhead.Coord(0., 0., 0., 0.)
        self.axes_max = toolhead.Coord(self.draw_width, self.draw_height, 0., 0.)

        logging.info(
            "PolarDrawing: motor_distance=%.1f  hypotenuse_home=%.1f  "
            "draw_origin_world=(%.1f, %.1f)  home_y_world=%.1f  "
            "homed_drawing=(%.1f, %.1f)  draw_area=%.0f x %.0f mm",
            self.motor_distance, self.hypotenuse_home,
            self.draw_origin_wx, self.draw_origin_wy,
            self.home_y_world,
            self.homed_drawing_x, self.homed_drawing_y,
            self.draw_width, self.draw_height,
        )

    # ── Pen changer placeholder ───────────────────────────────────────────────

    def _init_pen_changer(self, config):
        self.pen_changer_angle_pin = config.get('pen_changer_angle_sensor_pin', None)
        self.pen_changer_servo_pin = config.get('pen_changer_servo_pin', None)
        self.pen_count   = config.getint('pen_count', 4, minval=1)
        self.current_pen = 0
        self.pen_changer_enabled = (
            self.pen_changer_angle_pin is not None and
            self.pen_changer_servo_pin is not None
        )

    def _do_pen_change(self, target_pen):
        """PLACEHOLDER: rotate carousel to target_pen slot."""
        logging.info(
            "PolarDrawing: pen change %d -> %d requested (NOT YET IMPLEMENTED)",
            self.current_pen, target_pen
        )
        self.current_pen = target_pen

    # ── Coordinate transforms ─────────────────────────────────────────────────

    def _drawing_to_world(self, dx, dy):
        return self.draw_origin_wx + dx, self.draw_origin_wy + dy

    def _world_to_drawing(self, wx, wy):
        return wx - self.draw_origin_wx, wy - self.draw_origin_wy

    # ── Inverse kinematics: drawing XY -> belt lengths ────────────────────────

    def _xy_to_belts(self, dx, dy):
        wx, wy = self._drawing_to_world(dx, dy)
        half_w = self.motor_distance / 2.0
        left_len  = math.sqrt((wx + half_w) ** 2 + wy ** 2)
        right_len = math.sqrt((wx - half_w) ** 2 + wy ** 2)
        return left_len, right_len

    # ── Forward kinematics: belt lengths -> drawing XY ────────────────────────

    def _belts_to_xy(self, left_len, right_len):
        W = self.motor_distance
        x_from_left = (left_len ** 2 - right_len ** 2 + W ** 2) / (2.0 * W)
        y_sq    = left_len ** 2 - x_from_left ** 2
        world_y = math.sqrt(max(y_sq, 0.0))
        world_x = x_from_left - W / 2.0
        return self._world_to_drawing(world_x, world_y)

    # ── Klipper kinematics interface ──────────────────────────────────────────

    def get_steppers(self):
        return list(self.steppers)

    def calc_position(self, stepper_positions):
        """Compute toolhead XYZ from current stepper belt lengths."""
        left_len  = stepper_positions[self.steppers[0].get_name()]
        right_len = stepper_positions[self.steppers[1].get_name()]
        dx, dy = self._belts_to_xy(left_len, right_len)
        return [dx, dy, 0.0]

    def set_position(self, newpos, homing_axes):
        """Force a known position -- convert drawing XY to belt lengths."""
        left_len, right_len = self._xy_to_belts(newpos[0], newpos[1])
        self.steppers[0].set_position([left_len,  0.0, 0.0])
        self.steppers[1].set_position([right_len, 0.0, 0.0])

    def home(self, homing_state):
        """
        Homing sequence:
          1. Both motors drive simultaneously toward endstops.
             Each endstop is a MAX endstop (counterweight at top).
             Driving toward shorter belt raises gondola and counterweight.
          2. Both endstops trigger. Belt lengths = hypotenuse_home.
          3. We tell Klipper the gondola drawing position from geometry.
          4. G0 X0 Y0 moves gondola to drawing top-left corner.
        """
        # Build a homing move for both steppers simultaneously
        # homing_state.home_rails is not available; use the lower-level API.
        # We drive both steppers to their endstops using a Homing object.
        homing_obj = homing_state
        # Tell homing which axes we are homing
        homing_obj.set_axes([0, 1])

        # Drive both steppers toward their endstops simultaneously
        # endstop position = hypotenuse_home (position_endstop in config)
        # We move in the direction that shortens the belt (negative direction)
        # The endstop will stop the motor when counterweight hits it.
        endstop_list = list(zip(self.endstops, self.endstop_names))
        homing_obj.home_motors(
            endstop_list,
            forcepos=True,
            speed=self.homing_speed,
            second_speed=self.homing_speed / 2.0,
            retract=self.homing_retract_dist,
            retract_speed=self.homing_speed / 2.0,
            pos_endstop=self.hypotenuse_home,
        )

        # Set homed position in drawing coordinates
        homing_obj.set_homed_position(
            [self.homed_drawing_x, self.homed_drawing_y, 0.0])
        logging.info(
            "PolarDrawing: homing complete -- gondola at drawing (%.2f, %.2f)",
            self.homed_drawing_x, self.homed_drawing_y
        )

    def clear_homing_state(self, clear_axes):
        pass

    def check_move(self, move):
        """Reject moves outside drawing area or beyond belt length limits."""
        dx, dy = move.end_pos[0], move.end_pos[1]
        if (dx < 0.0 or dx > self.draw_width or
                dy < 0.0 or dy > self.draw_height):
            raise move.move_error(
                "PolarDrawing: destination (%.2f, %.2f) outside drawing area "
                "X=[0, %.1f]  Y=[0, %.1f]"
                % (dx, dy, self.draw_width, self.draw_height)
            )
        left_len, right_len = self._xy_to_belts(dx, dy)
        if (left_len  > self.max_belt_length or
                right_len > self.max_belt_length or
                left_len  < self.min_belt_length or
                right_len < self.min_belt_length):
            raise move.move_error(
                "PolarDrawing: belt lengths out of range at (%.2f, %.2f) "
                "left=%.1f mm  right=%.1f mm  (allowed: %.1f - %.1f mm)"
                % (dx, dy, left_len, right_len,
                   self.min_belt_length, self.max_belt_length)
            )

    def get_status(self, eventtime):
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