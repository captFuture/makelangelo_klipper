# Polar Drawing Machine Kinematics for Klipper
# Repository: https://github.com/captFuture/makelangelo_klipper
#
# Based on winch.py pattern -- each motor is a cable winch at a fixed anchor.
# winch_stepper_alloc(ax, ay, az) computes belt length as Euclidean distance
# from anchor to toolhead position -- exactly the polargraph belt formula.
#
# Coordinate system (DRAWING space = toolhead space):
#   Origin (0,0): draw_margin_left mm right of left motor,
#                 draw_margin_top  mm below motor axis
#   X increases right, Y increases down
#   Motor anchors in drawing space:
#     left:  (-draw_margin_left,  -draw_margin_top, 0)
#     right: (motor_distance - draw_margin_left, -draw_margin_top, 0)
#
# Homing:
#   Both endstops are wired as MCU endstops and registered manually.
#   Home sequence drives both motors until both endstops trigger.
#   After homing, gondola is at drawing position (homed_drawing_x, homed_drawing_y).
#
# Pen changer: placeholder for future carousel hardware.

import math
import logging
import stepper


class PolarDrawingKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()

        # ── Machine dimensions ────────────────────────────────────────────────
        self.motor_distance   = config.getfloat('motor_distance', above=0.)
        self.hypotenuse_home  = config.getfloat('hypotenuse_length_at_home', 1035.0)
        self.max_belt_length  = config.getfloat('max_belt_length', 1400.0)
        self.min_belt_length  = config.getfloat('min_belt_length', 100.0)
        self.draw_margin_left = config.getfloat('draw_margin_left', 115.0)
        self.draw_margin_top  = config.getfloat('draw_margin_top',  115.0)
        self.draw_width       = config.getfloat('draw_width',  594.0)
        self.draw_height      = config.getfloat('draw_height', 420.0)
        self.homing_speed     = config.getfloat('homing_speed', 50.0)

        # ── Derived geometry ──────────────────────────────────────────────────
        half_w = self.motor_distance / 2.0

        # Motor anchor positions in DRAWING coordinates
        # left motor:  world (-half_w, 0) -> drawing (-half_w - draw_origin_wx, -draw_origin_wy)
        #            = (-half_w - (-half_w + draw_margin_left), -draw_margin_top)
        #            = (-draw_margin_left, -draw_margin_top)
        self.anchor_left  = (-self.draw_margin_left,
                              0.,
                             -self.draw_margin_top)
        self.anchor_right = (self.motor_distance - self.draw_margin_left,
                              0.,
                             -self.draw_margin_top)

        # Gondola world-Y when both endstops trigger
        self.home_y_world    = math.sqrt(
            max(self.hypotenuse_home**2 - half_w**2, 0.0))
        # Drawing origin in world: (-half_w + draw_margin_left, draw_margin_top)
        draw_origin_wx = -half_w + self.draw_margin_left
        draw_origin_wy =  self.draw_margin_top
        # Gondola drawing position after homing (world X=0, world Y=home_y_world)
        self.homed_drawing_x = 0.0             - draw_origin_wx
        self.homed_drawing_y = self.home_y_world - draw_origin_wy

        # ── Load steppers (winch.py pattern) ─────────────────────────────────
        # PrinterStepper + winch_stepper_alloc: the C solver computes
        # Euclidean distance from anchor to toolhead = belt length.
        # Toolhead moves in drawing (XZ) plane; anchor_y=0 for both motors.
        self.steppers = []
        anchors = [self.anchor_left, self.anchor_right]
        for name, anchor in zip(['stepper_left', 'stepper_right'], anchors):
            s = stepper.PrinterStepper(config.getsection(name))
            s.setup_itersolve('winch_stepper_alloc', *anchor)
            s.set_trapq(toolhead.get_trapq())
            self.steppers.append(s)

        # ── Endstops ──────────────────────────────────────────────────────────
        # Register endstop pins manually -- PrinterStepper doesn't handle them.
        ppins = self.printer.lookup_object('pins')
        self.endstops = []
        for i, name in enumerate(['stepper_left', 'stepper_right']):
            sc  = config.getsection(name)
            pin = sc.get('endstop_pin')
            mcu_es = ppins.setup_pin('endstop', pin)
            mcu_es.add_stepper(self.steppers[i])
            self.endstops.append((mcu_es, name))

        # Homing info from stepper configs
        self.homing_speed_cfg = config.getfloat('homing_speed', 50.0)

        # ── Drawing area for Mainsail ─────────────────────────────────────────
        self.axes_min = toolhead.Coord([0., 0., 0., 0.])
        self.axes_max = toolhead.Coord([self.draw_width, self.draw_height, 0., 0.])

        # ── Pen changer placeholder ───────────────────────────────────────────
        self._init_pen_changer(config)
        self.printer.add_object('polardrawing', self)

        logging.info(
            "PolarDrawing: motor_distance=%.1f  hypotenuse_home=%.1f  "
            "home_y_world=%.1f  homed_drawing=(%.1f, %.1f)  "
            "anchors: left=%s  right=%s",
            self.motor_distance, self.hypotenuse_home,
            self.home_y_world, self.homed_drawing_x, self.homed_drawing_y,
            self.anchor_left, self.anchor_right)

    # ── Pen changer placeholder ───────────────────────────────────────────────

    def _init_pen_changer(self, config):
        self.pen_changer_angle_pin = config.get('pen_changer_angle_sensor_pin', None)
        self.pen_changer_servo_pin = config.get('pen_changer_servo_pin', None)
        self.pen_count           = config.getint('pen_count', 4, minval=1)
        self.current_pen         = 0
        self.pen_changer_enabled = (self.pen_changer_angle_pin is not None
                                    and self.pen_changer_servo_pin is not None)

    def _do_pen_change(self, target_pen):
        logging.info("PolarDrawing: pen change %d->%d (NOT YET IMPLEMENTED)",
                     self.current_pen, target_pen)
        self.current_pen = target_pen

    # ── Klipper interface ─────────────────────────────────────────────────────

    def get_steppers(self):
        return list(self.steppers)

    def calc_position(self, stepper_positions):
        """
        Forward kinematics: belt lengths -> drawing XY.
        The winch solver stores belt lengths in stepper_positions.
        We triangulate to find gondola position.
        """
        left_l  = stepper_positions['stepper_left']
        right_l = stepper_positions['stepper_right']
        W = self.motor_distance
        # Triangulation (law of cosines)
        # Anchors are at drawing X = -draw_margin_left and X = motor_distance - draw_margin_left
        # Distance between anchors in X = motor_distance
        ax_l = self.anchor_left[0]   # = -draw_margin_left
        ax_r = self.anchor_right[0]  # = motor_distance - draw_margin_left
        az   = self.anchor_left[2]   # = -draw_margin_top (same for both)
        # Solve for gondola (dx, dz) in drawing space
        # left_l^2  = (dx - ax_l)^2 + dz^2  ... wait, anchors use (x,y,z) but gondola is 2D
        # winch uses distance in 3D: sqrt((gx-ax)^2 + (gy-ay)^2 + (gz-az)^2)
        # gondola y=0 always, anchor y=0 always, so:
        # left_l^2  = (dx - ax_l)^2 + (dz - az)^2
        # right_l^2 = (dx - ax_r)^2 + (dz - az)^2
        # Subtract: left_l^2 - right_l^2 = (dx-ax_l)^2 - (dx-ax_r)^2
        #         = dx^2 - 2*dx*ax_l + ax_l^2 - dx^2 + 2*dx*ax_r - ax_r^2
        #         = 2*dx*(ax_r - ax_l) + ax_l^2 - ax_r^2
        #         = 2*dx*W + ax_l^2 - ax_r^2
        # dx = (left_l^2 - right_l^2 - ax_l^2 + ax_r^2) / (2*W)
        dx = (left_l**2 - right_l**2 - ax_l**2 + ax_r**2) / (2.0 * W)
        dz_sq = left_l**2 - (dx - ax_l)**2
        dz = math.sqrt(max(dz_sq, 0.0)) + az  # az is negative, so dz = drawing Y
        return [dx, dz, 0.0]

    def set_position(self, newpos, homing_axes):
        """
        Klipper calls this to force a known position.
        newpos is in drawing coordinates [dx, dy, dz, e].
        The winch solver computes belt lengths from anchor to newpos automatically
        when we call set_position on the stepper with the drawing position.
        """
        for s in self.steppers:
            s.set_position(newpos)

    def home(self, homing_state):
        """
        Homing using the winch.py / manual endstop pattern.
        Both motors drive simultaneously until both endstops trigger.
        Then we set the known homed position in drawing coordinates.
        """
        # Tell homing system which axes we are homing
        homing_state.set_axes([0, 1])

        # Drive both steppers toward endstops simultaneously
        # We use the Homing object directly like winch.py does
        from extras import homing as homing_mod
        toolhead = self.printer.lookup_object('toolhead')

        # Force current position to a point far from endstop
        # so the homing move has room to travel
        # In drawing space, move to a position where belts are short
        # (gondola high = belts short = far from MAX endstop)
        # We use the homed position itself as the target (homepos)
        # and force a start below it
        curpos = list(toolhead.get_position())
        # Force position to drawing coords of short belt (gondola low = long belt)
        # We just need the homing move to go in the right direction
        # Start: gondola at bottom (long belts), End: gondola at home (shorter belts)
        # But our endstops trigger at LONG belt (counterweight UP = gondola DOWN)
        # So homepos = homed_drawing position, forcepos = somewhere below it

        # Use HomingMove directly
        hmove = homing_mod.HomingMove(self.printer, self.endstops)
        # Move to homed drawing position -- endstops will stop it
        homepos = [self.homed_drawing_x, self.homed_drawing_y, 0., 0.]
        # Forcepos: start from a position below home (larger Y = longer belts)
        forcepos = [self.homed_drawing_x,
                    self.homed_drawing_y + self.draw_height,
                    0., 0.]
        toolhead.set_position(forcepos)
        hmove.homing_move(homepos, self.homing_speed)

        # Set homed position
        homing_state.set_homed_position(
            [self.homed_drawing_x, self.homed_drawing_y, 0.])
        logging.info("PolarDrawing: homing done, drawing pos=(%.2f, %.2f)",
                     self.homed_drawing_x, self.homed_drawing_y)

    def clear_homing_state(self, clear_axes):
        pass

    def check_move(self, move):
        dx, dy = move.end_pos[0], move.end_pos[1]
        if (dx < 0. or dx > self.draw_width or
                dy < 0. or dy > self.draw_height):
            raise move.move_error(
                "PolarDrawing: (%.2f, %.2f) outside drawing area "
                "X=[0, %.0f] Y=[0, %.0f]"
                % (dx, dy, self.draw_width, self.draw_height))

    def get_status(self, eventtime):
        return {
            'homed_axes':          'xy',
            'axis_minimum':        self.axes_min,
            'axis_maximum':        self.axes_max,
            'motor_distance':      self.motor_distance,
            'hypotenuse_home':     self.hypotenuse_home,
            'home_y_world':        self.home_y_world,
            'draw_origin_wx':      self.anchor_left[0],
            'draw_origin_wy':      -self.anchor_left[2],
            'homed_drawing_x':     self.homed_drawing_x,
            'homed_drawing_y':     self.homed_drawing_y,
            'draw_width':          self.draw_width,
            'draw_height':         self.draw_height,
            'current_pen':         self.current_pen,
            'pen_count':           self.pen_count,
            'pen_changer_enabled': self.pen_changer_enabled,
        }


def load_kinematics(toolhead, config):
    return PolarDrawingKinematics(toolhead, config.getsection('polardrawing'))
    return PolarDrawingKinematics(toolhead, config.getsection('polardrawing'))