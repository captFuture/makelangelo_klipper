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
#  Belt length at trigger = hypotenuse_length_at_home (position_endstop).
#  Homed world position = (0, home_y_world)
#  Klipper is told the DRAWING coordinates of that position.
#
# ── Belt direction ────────────────────────────────────────────────────────────
#
#  Gondola moves DOWN  -> belt length INCREASES -> counterweight UP
#  Gondola moves UP    -> belt length DECREASES -> counterweight DOWN
#  Homing drives toward SHORTER belt (gondola up, counterweight up to endstop)
#  homing_positive_dir: false in stepper config
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
        self.draw_origin_wx  = -half_w + self.draw_margin_left
        self.draw_origin_wy  =  self.draw_margin_top
        self.home_y_world    = math.sqrt(
            max(self.hypotenuse_home ** 2 - half_w ** 2, 0.0))
        self.homed_drawing_x = 0.0               - self.draw_origin_wx
        self.homed_drawing_y = self.home_y_world  - self.draw_origin_wy

        # ── Load belt rails (cartesian.py pattern) ────────────────────────────
        # stepper.LookupMultiRail reads endstop_pin, position_min/max/endstop,
        # homing_positive_dir from the stepper config section.
        # setup_itersolve connects the rail to the C motion planner.
        # set_trapq connects to the toolhead motion queue.

        self.rails = [
            stepper.LookupMultiRail(config.getsection('stepper_left')),
            stepper.LookupMultiRail(config.getsection('stepper_right')),
        ]
        for rail, axis in zip(self.rails, 'xy'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())

        # Homing limits -- (1.0, -1.0) means not yet homed
        self.limits = [(1.0, -1.0), (1.0, -1.0)]

        # ── Drawing area for Mainsail display ─────────────────────────────────
        self.axes_min = toolhead.Coord([0., 0., 0., 0.])
        self.axes_max = toolhead.Coord(
            [self.draw_width, self.draw_height, 0., 0.])

        # ── Pen changer placeholder ───────────────────────────────────────────
        self._init_pen_changer(config)

        # Register as a printer object so macros can access
        # printer['polardrawing'].motor_distance etc.
        self.printer.add_object('polardrawing', self)

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
        self.pen_changer_angle_pin = config.get(
            'pen_changer_angle_sensor_pin', None)
        self.pen_changer_servo_pin = config.get(
            'pen_changer_servo_pin', None)
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
        """Drawing position -> (left_belt_mm, right_belt_mm)."""
        wx, wy = self._drawing_to_world(dx, dy)
        half_w    = self.motor_distance / 2.0
        left_len  = math.sqrt((wx + half_w) ** 2 + wy ** 2)
        right_len = math.sqrt((wx - half_w) ** 2 + wy ** 2)
        return left_len, right_len

    # ── Forward kinematics: belt lengths -> drawing XY ────────────────────────

    def _belts_to_xy(self, left_len, right_len):
        """(left_belt_mm, right_belt_mm) -> drawing position."""
        W = self.motor_distance
        x_from_left = (left_len ** 2 - right_len ** 2 + W ** 2) / (2.0 * W)
        y_sq    = left_len ** 2 - x_from_left ** 2
        world_y = math.sqrt(max(y_sq, 0.0))
        world_x = x_from_left - W / 2.0
        return self._world_to_drawing(world_x, world_y)

    # ── Klipper kinematics interface ──────────────────────────────────────────

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        """Compute toolhead XYZ from current stepper belt lengths."""
        left_len  = stepper_positions[self.rails[0].get_name()]
        right_len = stepper_positions[self.rails[1].get_name()]
        dx, dy = self._belts_to_xy(left_len, right_len)
        return [dx, dy, 0.0]

    def set_position(self, newpos, homing_axes):
        """Force a known position -- convert drawing XY to belt lengths."""
        for rail in self.rails:
            rail.set_position(newpos)
        for axis_name in homing_axes:
            axis = 'xy'.index(axis_name)
            self.limits[axis] = self.rails[axis].get_range()

    def home(self, homing_state):
        """
        Home both belt motors simultaneously.

        Both rails drive toward shorter belt (homing_positive_dir: false).
        Counterweights rise and hit their MAX endstops simultaneously.
        Belt length at endstop = position_endstop = hypotenuse_home.

        After homing, set_position() is called by Klipper with the homed
        position from calc_position(). We then override the toolhead position
        to drawing coordinates in the G28 macro (GO_HOME after homing).
        """
        for axis in homing_state.get_axes():
            rail = self.rails[axis]
            hi   = rail.get_homing_info()

            # forcepos: where to force the toolhead before the homing move.
            # For homing_positive_dir: false (drive toward negative / shorter
            # belt), forcepos must be ABOVE position_endstop so the move
            # goes downward toward the endstop.
            position_min, position_max = rail.get_range()
            homepos  = [None, None, None, None]
            forcepos = [None, None, None, None]
            homepos[axis]  = hi.position_endstop
            if hi.positive_dir:
                forcepos[axis] = (hi.position_endstop
                                  - 1.5 * (hi.position_endstop - position_min))
            else:
                forcepos[axis] = (hi.position_endstop
                                  + 1.5 * (position_max - hi.position_endstop))

            homing_state.home_rails([rail], forcepos, homepos)

    def clear_homing_state(self, clear_axes):
        for axis, axis_name in enumerate('xy'):
            if axis_name in clear_axes:
                self.limits[axis] = (1.0, -1.0)

    def check_move(self, move):
        """Reject moves outside drawing area or beyond belt length limits."""
        # Skip bounds check if not yet homed -- homing moves use raw belt
        # coordinates which are outside the drawing area by definition.
        if self.limits[0][0] > self.limits[0][1]:
            return  # not homed, allow all moves

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