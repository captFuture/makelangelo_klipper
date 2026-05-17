# Polar Drawing Machine Kinematics for Klipper
# Project: makelangelo_klipper
# Repository: https://github.com/captFuture/makelangelo_klipper
#
# FILE PLACEMENT:
#   Copy to: ~/makelangelo_klipper/klippy/kinematics/polardrawing.py
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
#  Both motors drive toward shorter belt (homing_positive_dir: false).
#  Counterweights rise and hit MAX endstops simultaneously.
#  Belt length at trigger = hypotenuse_length_at_home.
#  Homed world position = (0, home_y_world) where:
#    home_y_world = sqrt(hypotenuse_home^2 - (motor_distance/2)^2)
#  Klipper is then told the DRAWING coordinates of that position:
#    homed_drawing_x = 0           - draw_origin_wx  = motor_distance/2 - draw_margin_left
#    homed_drawing_y = home_y_world - draw_origin_wy  = home_y_world - draw_margin_top
#
# ── Belt direction ────────────────────────────────────────────────────────────
#
#  Gondola moves DOWN  -> world_y increases -> belt lengths INCREASE
#  Gondola moves UP    -> world_y decreases -> belt lengths DECREASE
#  hypotenuse_home is the minimum belt length at homed gondola X=0.
#  Belt length at the drawing top-left corner (X=0, Y=0) is shorter than
#  hypotenuse_home because the drawing origin is ABOVE the homed position.
#
# ── Pen changer (placeholder) ─────────────────────────────────────────────────
#
#  Future hardware:
#    - Angle sensor (potentiometer or encoder) for carousel position feedback
#    - 360-degree continuous rotation servo to rotate the carousel
#  Both are parsed from config and stored. See _init_pen_changer() below.

import math
import logging
import stepper


class PolarDrawingKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()

        # ── Machine dimensions ────────────────────────────────────────────────

        # Distance between motor shaft centres (mm).
        # Measure: centre of left drive pulley to centre of right drive pulley.
        self.motor_distance = config.getfloat('motor_distance', above=0.)

        # Belt length when both counterweights trigger their MAX endstops (mm).
        # Measured: motor pulley to pen tip with gondola in homed position.
        self.hypotenuse_home = config.getfloat(
            'hypotenuse_length_at_home', 1035.0, above=0.)

        # Maximum belt length (gondola at bottom of drawing area).
        self.max_belt_length = config.getfloat(
            'max_belt_length', self.hypotenuse_home + 300.0, above=0.)

        # Physical minimum belt length (safety limit, gondola near one motor).
        self.min_belt_length = config.getfloat('min_belt_length', 100.0, above=0.)

        # ── Drawing origin offset ─────────────────────────────────────────────

        # Distance from left motor to drawing left edge (mm).
        # Drawing X=0 is this many mm to the right of the left motor.
        self.draw_margin_left = config.getfloat('draw_margin_left', 115.0, above=0.)

        # Distance from motor axis to drawing top edge (mm).
        # Drawing Y=0 is this many mm below the motor axis.
        self.draw_margin_top = config.getfloat('draw_margin_top', 115.0, above=0.)

        # Drawing area dimensions (mm).  DIN A2 landscape = 594 x 420 mm.
        self.draw_width  = config.getfloat('draw_width',  594.0, above=0.)
        self.draw_height = config.getfloat('draw_height', 420.0, above=0.)

        # Homing speed (mm/s).
        self.homing_speed = config.getfloat('homing_speed', 50.0, above=0.)

        # Retract distance after endstop triggers (mm).
        self.homing_retract_dist = config.getfloat('homing_retract_dist', 10.0)

        # ── Derived geometry ──────────────────────────────────────────────────

        half_w = self.motor_distance / 2.0

        # Drawing origin in world coordinates
        self.draw_origin_wx = -half_w + self.draw_margin_left
        self.draw_origin_wy =  self.draw_margin_top

        # World Y of gondola when both endstops are triggered
        self.home_y_world = math.sqrt(
            max(self.hypotenuse_home ** 2 - half_w ** 2, 0.0)
        )

        # Homed gondola position in DRAWING coordinates
        # (what Klipper is told after homing completes)
        self.homed_drawing_x =  0.0              - self.draw_origin_wx
        self.homed_drawing_y =  self.home_y_world - self.draw_origin_wy

        # ── Load belt steppers as Rails ───────────────────────────────────────
        # PrinterRail (not PrinterStepper) is required because:
        #   - It handles endstop_pin, position_min/max/endstop in the config
        #   - It provides get_homing_info() needed by the homing system
        #   - It supports homing_positive_dir
        # Each rail drives one belt. We use the cartesian itersolve solver
        # since our inverse kinematics is handled entirely in Python
        # (calc_position / set_position / _xy_to_belts).

        self.rails = []
        trapq = toolhead.get_trapq()
        for name, axis in [('stepper_left', b'x'), ('stepper_right', b'y')]:
            rail = stepper.PrinterRail(config.getsection(name))
            rail.setup_itersolve('cartesian_stepper_alloc', axis)
            rail.set_trapq(trapq)
            self.rails.append(rail)

        # self.steppers: flat list of all MCU stepper objects across all rails
        # (each rail may have multiple steppers in multi-motor setups)
        self.steppers = [s for rail in self.rails for s in rail.get_steppers()]

        # ── Pen changer placeholder ───────────────────────────────────────────
        self._init_pen_changer(config)

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
        """
        Pen changer placeholder.

        Future hardware:
          pen_changer_angle_sensor_pin -- ADC/encoder reading carousel rotation
          pen_changer_servo_pin        -- 360-deg continuous rotation servo

        When hardware is fitted:
          1. Uncomment both pins in [polardrawing] in printer.cfg.
          2. Uncomment [servo pen_changer] and the angle sensor section.
          3. Implement _do_pen_change() below.

        Config keys (all optional):
          pen_changer_angle_sensor_pin: PA1   # carousel angle feedback
          pen_changer_servo_pin:        PH4   # 360-deg servo
          pen_count:                    4     # slots on carousel
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
          2. Calculate rotation needed: target_angle = target_pen * 360 / pen_count
          3. Drive pen_changer_servo until angle sensor confirms target_angle.
          4. Update self.current_pen.
        """
        logging.info(
            "PolarDrawing: pen change %d -> %d requested (NOT YET IMPLEMENTED)",
            self.current_pen, target_pen
        )
        self.current_pen = target_pen

    # ── Coordinate transforms ─────────────────────────────────────────────────

    def _drawing_to_world(self, dx, dy):
        """Convert drawing coordinates to world coordinates."""
        return self.draw_origin_wx + dx, self.draw_origin_wy + dy

    def _world_to_drawing(self, wx, wy):
        """Convert world coordinates to drawing coordinates."""
        return wx - self.draw_origin_wx, wy - self.draw_origin_wy

    # ── Inverse kinematics: drawing XY -> belt lengths ────────────────────────

    def _xy_to_belts(self, dx, dy):
        """
        Drawing position (dx, dy) -> (left_belt_mm, right_belt_mm).
        Belt = Euclidean distance from motor anchor to gondola in world space.
        """
        wx, wy = self._drawing_to_world(dx, dy)
        half_w = self.motor_distance / 2.0
        left_len  = math.sqrt((wx + half_w) ** 2 + wy ** 2)
        right_len = math.sqrt((wx - half_w) ** 2 + wy ** 2)
        return left_len, right_len

    # ── Forward kinematics: belt lengths -> drawing XY ────────────────────────

    def _belts_to_xy(self, left_len, right_len):
        """
        (left_belt_mm, right_belt_mm) -> drawing position (dx, dy).
        Triangulation using law of cosines, then offset to drawing origin.
        """
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
        left_len  = stepper_positions[self.rails[0].get_name()]
        right_len = stepper_positions[self.rails[1].get_name()]
        dx, dy = self._belts_to_xy(left_len, right_len)
        return [dx, dy, 0.0]

    def set_position(self, newpos, homing_axes):
        """Force a known position -- convert drawing XY to belt lengths."""
        left_len, right_len = self._xy_to_belts(newpos[0], newpos[1])
        self.rails[0].set_position([left_len,  0.0, 0.0])
        self.rails[1].set_position([right_len, 0.0, 0.0])

    def home(self, homing_state):
        """
        Homing sequence:
          1. Both rails home simultaneously toward their endstops.
             homing_positive_dir: false drives toward shorter belt
             (gondola rises, counterweights go up, hit MAX endstops).
          2. Klipper latches belt length at endstop = position_endstop.
          3. We set the toolhead drawing position from the known geometry.
          4. G0 X0 Y0 then moves gondola to drawing top-left corner.
        """
        # home_rails drives both rails to their endstops using
        # homing_positive_dir and position_endstop from the config
        forcepos = list(self._get_home_position())
        homing_state.home_rails(self.rails, forcepos, self._get_home_position())
        # Tell Klipper where the gondola is in drawing coordinates
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.set_position(
            [self.homed_drawing_x, self.homed_drawing_y, 0.0, 0.0],
            homing_axes=(0, 1)
        )
        logging.info(
            "PolarDrawing: homing complete -- gondola at drawing (%.2f, %.2f)",
            self.homed_drawing_x, self.homed_drawing_y
        )

    def _get_home_position(self):
        """Return the toolhead position to force during homing approach."""
        # For homing_positive_dir: false, forcepos should be beyond position_max
        # so the homing move drives in the negative direction (toward endstop)
        return [self.homed_drawing_x, self.homed_drawing_y, 0.0]

    def clear_homing_state(self, clear_axes):
        """Called by Klipper to reset homing state on specified axes."""
        # Nothing to reset -- homing state is managed by the rails
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
        """Expose machine state to Klipper macros via printer['polardrawing']."""
        return {
            'homed_axes':          'xy',
            'motor_distance':      self.motor_distance,
            'hypotenuse_home':     self.hypotenuse_home,
            'home_y_world':        self.home_y_world,
            'draw_origin_wx':      self.draw_origin_wx,
            'draw_origin_wy':      self.draw_origin_wy,
            'homed_drawing_x':     self.homed_drawing_x,
            'homed_drawing_y':     self.homed_drawing_y,
            'draw_width':          self.draw_width,
            'draw_height':         self.draw_height,
            # Pen changer
            'current_pen':         self.current_pen,
            'pen_count':           self.pen_count,
            'pen_changer_enabled': self.pen_changer_enabled,
        }


def load_kinematics(toolhead, config):
    """Entry point called by Klipper when 'kinematics: polardrawing' is set."""
    # config here is the [printer] section -- we need the [polardrawing] section
    return PolarDrawingKinematics(toolhead, config.getsection('polardrawing'))