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
#    Left motor  at world (-motor_distance/2, 0, 0)
#    Right motor at world (+motor_distance/2, 0, 0)
#    Y increases downward
#
#  DRAWING coordinates  (what G-code sees, what Klipper reports)
#    Origin (0,0): draw_margin_left mm right of left motor,
#                  draw_margin_top  mm below motor axis
#    X increases right, Y increases down
#    Motor anchors in drawing coordinates:
#      left:  X = -draw_margin_left,      Z = -draw_margin_top
#      right: X = motor_distance - draw_margin_left,  Z = -draw_margin_top
#    (Y=0 for both -- motors are in the XZ plane of the drawing space)
#
# ── Itersolve ─────────────────────────────────────────────────────────────────
#
#  Uses winch_stepper_alloc(anchor_x, anchor_y, anchor_z) from Klipper's C
#  solver. Each motor is a winch at a fixed anchor pulling a cable (belt) to
#  the gondola. The solver computes Euclidean distance from anchor to toolhead
#  position -- exactly the belt length formula. X/Z are used for the 2D plane,
#  anchor_y=0 for both motors.
#
# ── Homing ────────────────────────────────────────────────────────────────────
#
#  homing_positive_dir: true -- homes toward position_max (longer belt).
#  Longer belt = gondola descends = counterweight rises to MAX endstop.
#  After homing, belt lengths = position_endstop = hypotenuse_length_at_home.
#  Toolhead is then set to drawing coordinates (homed_drawing_x, homed_drawing_y).
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
        # = position_endstop in both stepper configs.
        self.hypotenuse_home = config.getfloat(
            'hypotenuse_length_at_home', 1035.0, above=0.)

        # Maximum belt length (gondola at drawing bottom).
        self.max_belt_length = config.getfloat(
            'max_belt_length', self.hypotenuse_home + 300.0, above=0.)

        # Physical minimum belt length (safety limit).
        self.min_belt_length = config.getfloat('min_belt_length', 100.0, above=0.)

        # ── Drawing origin offset ─────────────────────────────────────────────

        # mm from left motor to left edge of drawing area
        self.draw_margin_left = config.getfloat('draw_margin_left', 115.0, above=0.)
        # mm from motor axis down to top edge of drawing area
        self.draw_margin_top  = config.getfloat('draw_margin_top',  115.0, above=0.)
        # Drawing area size (DIN A2 landscape = 594 x 420 mm)
        self.draw_width       = config.getfloat('draw_width',  594.0, above=0.)
        self.draw_height      = config.getfloat('draw_height', 420.0, above=0.)

        # Homing speed (mm/s).
        self.homing_speed = config.getfloat('homing_speed', 50.0, above=0.)

        # Retract distance after endstop triggers (mm).
        self.homing_retract_dist = config.getfloat('homing_retract_dist', 10.0)

        # ── Derived geometry ──────────────────────────────────────────────────

        half_w = self.motor_distance / 2.0

        # Drawing origin in world coordinates
        self.draw_origin_wx  = -half_w + self.draw_margin_left
        self.draw_origin_wy  =  self.draw_margin_top

        # World Y of gondola when both endstops trigger
        self.home_y_world = math.sqrt(
            max(self.hypotenuse_home ** 2 - half_w ** 2, 0.0))

        # Gondola position in DRAWING coordinates after homing
        self.homed_drawing_x = 0.0              - self.draw_origin_wx
        self.homed_drawing_y = self.home_y_world - self.draw_origin_wy

        # Motor anchor positions in drawing coordinates for winch_stepper_alloc.
        # The solver uses (anchor_x, anchor_y=0, anchor_z) where Z is our Y axis.
        # left motor drawing X  = world_x - draw_origin_wx = -half_w - draw_origin_wx
        #                       = -half_w - (-half_w + draw_margin_left) = -draw_margin_left
        # left motor drawing Z  = world_y=0 - draw_origin_wy = -draw_margin_top
        self.anchor_left_x  = -self.draw_margin_left
        self.anchor_left_z  = -self.draw_margin_top
        self.anchor_right_x =  self.motor_distance - self.draw_margin_left
        self.anchor_right_z = -self.draw_margin_top

        # ── Load belt rails ───────────────────────────────────────────────────
        # LookupMultiRail reads endstop_pin, position_min/max/endstop,
        # homing_positive_dir from the stepper config section.
        # winch_stepper_alloc computes Euclidean distance from anchor to
        # toolhead -- exactly the belt length. Anchor Y = 0 (2D machine).

        self.rails = [
            stepper.LookupMultiRail(config.getsection('stepper_left')),
            stepper.LookupMultiRail(config.getsection('stepper_right')),
        ]
        self.rails[0].setup_itersolve(
            'winch_stepper_alloc',
            self.anchor_left_x, 0., self.anchor_left_z)
        self.rails[1].setup_itersolve(
            'winch_stepper_alloc',
            self.anchor_right_x, 0., self.anchor_right_z)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())

        # Homing limits -- (1.0, -1.0) = not yet homed (min > max convention)
        self.limits = [(1.0, -1.0), (1.0, -1.0)]
        self.is_homing = False

        # ── Drawing area bounds for Mainsail display ──────────────────────────
        self.axes_min = toolhead.Coord([0., 0., 0., 0.])
        self.axes_max = toolhead.Coord(
            [self.draw_width, self.draw_height, 0., 0.])

        # ── Pen changer placeholder ───────────────────────────────────────────
        self._init_pen_changer(config)

        # Register as printer object so macros can access
        # printer['polardrawing'].motor_distance etc.
        self.printer.add_object('polardrawing', self)

        logging.info(
            "PolarDrawing: motor_distance=%.1f  hypotenuse_home=%.1f  "
            "draw_origin_world=(%.1f, %.1f)  home_y_world=%.1f  "
            "homed_drawing=(%.1f, %.1f)  draw_area=%.0f x %.0f mm  "
            "anchors: left=(%.1f, 0, %.1f)  right=(%.1f, 0, %.1f)",
            self.motor_distance, self.hypotenuse_home,
            self.draw_origin_wx, self.draw_origin_wy,
            self.home_y_world,
            self.homed_drawing_x, self.homed_drawing_y,
            self.draw_width, self.draw_height,
            self.anchor_left_x, self.anchor_left_z,
            self.anchor_right_x, self.anchor_right_z,
        )

    # ── Pen changer placeholder ───────────────────────────────────────────────

    def _init_pen_changer(self, config):
        """
        Pen changer placeholder.
        Future hardware:
          pen_changer_angle_sensor_pin -- ADC/encoder for carousel position
          pen_changer_servo_pin        -- 360-deg continuous rotation servo
        Config keys (all optional -- omit to disable):
          pen_changer_angle_sensor_pin: PA1
          pen_changer_servo_pin:        PH4
          pen_count:                    4
        """
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
        """
        PLACEHOLDER: rotate carousel to target_pen slot.
        Implementation when hardware is ready:
          1. Read current angle from pen_changer_angle_pin.
          2. Calculate rotation: target_angle = target_pen * 360 / pen_count
          3. Drive servo until angle sensor confirms target_angle.
          4. Update self.current_pen.
        """
        logging.info(
            "PolarDrawing: pen change %d -> %d requested (NOT YET IMPLEMENTED)",
            self.current_pen, target_pen
        )
        self.current_pen = target_pen

    # ── Inverse kinematics: drawing XY -> belt lengths ────────────────────────

    def _xy_to_belts(self, dx, dy):
        """Drawing position -> (left_belt_mm, right_belt_mm)."""
        wx, wy = self.draw_origin_wx + dx, self.draw_origin_wy + dy
        half_w    = self.motor_distance / 2.0
        left_len  = math.sqrt((wx + half_w) ** 2 + wy ** 2)
        right_len = math.sqrt((wx - half_w) ** 2 + wy ** 2)
        return left_len, right_len

    # ── Forward kinematics: belt lengths -> drawing XY ────────────────────────

    def _belts_to_xy(self, left_len, right_len):
        """(left_belt_mm, right_belt_mm) -> drawing position (triangulation)."""
        W = self.motor_distance
        x_from_left = (left_len ** 2 - right_len ** 2 + W ** 2) / (2.0 * W)
        y_sq    = left_len ** 2 - x_from_left ** 2
        world_y = math.sqrt(max(y_sq, 0.0))
        world_x = x_from_left - W / 2.0
        return world_x - self.draw_origin_wx, world_y - self.draw_origin_wy

    # ── Klipper kinematics interface ──────────────────────────────────────────

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        """
        Called by Klipper to compute toolhead XYZ from stepper positions.
        With winch_stepper_alloc the C solver provides belt lengths directly.
        We convert belt lengths back to drawing coordinates.
        """
        left_len  = stepper_positions[self.rails[0].get_name()]
        right_len = stepper_positions[self.rails[1].get_name()]
        dx, dy = self._belts_to_xy(left_len, right_len)
        return [dx, dy, 0.0]

    def set_position(self, newpos, homing_axes):
        """
        Called by Klipper to force a known position.
        Sets rail positions in drawing coordinates (the C solver handles
        conversion to belt lengths via the winch anchor geometry).
        """
        for rail in self.rails:
            rail.set_position(newpos)
        if homing_axes:
            for axis_name in homing_axes:
                if axis_name in 'xy':
                    axis = 'xy'.index(axis_name)
                    self.limits[axis] = self.rails[axis].get_range()

    def home(self, homing_state):
        """
        Homing sequence:
          1. Both rails drive toward position_max (homing_positive_dir: true).
             Longer belt = gondola descends = counterweight rises to MAX endstop.
          2. Both MAX endstops trigger. Belt = position_endstop = hypotenuse_home.
          3. Toolhead position is overridden to drawing coordinates.
          4. G0 X0 Y0 moves gondola to top-left corner of paper.
        """
        self.is_homing = True
        # Home both rails simultaneously -- critical for polargraph:
        # sequential homing causes one counterweight to hit its endstop
        # while the other is still moving, leading to skipping and chaos.
        axes = [a for a in homing_state.get_axes() if a < len(self.rails)]
        if axes:
            homepos  = [None, None, None, None]
            forcepos = [None, None, None, None]
            for axis in axes:
                rail = self.rails[axis]
                hi   = rail.get_homing_info()
                position_min, position_max = rail.get_range()
                homepos[axis] = hi.position_endstop
                if hi.positive_dir:
                    forcepos[axis] = (hi.position_endstop
                                      - 1.5 * (hi.position_endstop - position_min))
                else:
                    forcepos[axis] = (hi.position_endstop
                                      + 1.5 * (position_max - hi.position_endstop))
            # Single call with both rails = simultaneous homing move
            homing_state.home_rails(
                [self.rails[a] for a in axes], forcepos, homepos)

        self.is_homing = False
        # Override toolhead to drawing coordinates after homing.
        # Without this, Klipper would report belt-length coordinates.
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.set_position(
            [self.homed_drawing_x, self.homed_drawing_y, 0., 0.],
            homing_axes='xy'
        )
        logging.info(
            "PolarDrawing: homing complete -- gondola at drawing (%.2f, %.2f)",
            self.homed_drawing_x, self.homed_drawing_y
        )

    def clear_homing_state(self, clear_axes):
        """Reset homing state when axes are cleared (e.g. after a crash)."""
        for axis, axis_name in enumerate('xy'):
            if axis_name in clear_axes:
                self.limits[axis] = (1.0, -1.0)

    def check_move(self, move):
        """Reject moves outside drawing area or beyond belt length limits."""
        # Allow all moves during homing or before first home
        if self.is_homing or self.limits[0][0] > self.limits[0][1]:
            return

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
        """Expose machine state to Klipper macros via printer['polardrawing']."""
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