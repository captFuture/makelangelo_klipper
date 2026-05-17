# Polar Drawing Machine Kinematics for Klipper
# Repository: https://github.com/captFuture/makelangelo_klipper
#
# Architecture:
#   stepper_left  uses cartesian_stepper_alloc('x') -- position[0] = left belt mm
#   stepper_right uses cartesian_stepper_alloc('y') -- position[1] = right belt mm
#
#   The C solver moves steppers linearly in their axis.
#   set_position() receives drawing coords and converts to belt lengths.
#   calc_position() receives belt lengths and converts to drawing coords.
#   home_rails() forcepos/homepos must be in belt-length space:
#     [left_belt, right_belt, 0, 0]
#
# Homing:
#   forcepos = [position_min, position_min, 0, 0]  (short belts)
#   homepos  = [position_endstop, position_endstop, 0, 0]  (hypotenuse_home)
#   homing_positive_dir: true -- drives from short to long belt
#   Longer belt = gondola down = counterweight UP to MAX endstop
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
        self.draw_margin_left = config.getfloat('draw_margin_left', 115.0)
        self.draw_margin_top  = config.getfloat('draw_margin_top',  115.0)
        self.draw_width       = config.getfloat('draw_width',  594.0)
        self.draw_height      = config.getfloat('draw_height', 420.0)
        self.homing_speed     = config.getfloat('homing_speed', 50.0)

        # ── Derived geometry ──────────────────────────────────────────────────
        half_w = self.motor_distance / 2.0
        self.draw_origin_wx  = -half_w + self.draw_margin_left
        self.draw_origin_wy  =  self.draw_margin_top
        self.home_y_world    = math.sqrt(
            max(self.hypotenuse_home**2 - half_w**2, 0.0))
        self.homed_drawing_x = 0.0              - self.draw_origin_wx
        self.homed_drawing_y = self.home_y_world - self.draw_origin_wy

        # ── Load rails ────────────────────────────────────────────────────────
        # stepper_left  -> axis 'x' -> toolhead position[0] = left belt length
        # stepper_right -> axis 'y' -> toolhead position[1] = right belt length
        self.rails = [
            stepper.LookupMultiRail(config.getsection('stepper_left')),
            stepper.LookupMultiRail(config.getsection('stepper_right')),
        ]
        self.rails[0].setup_itersolve('cartesian_stepper_alloc', b'x')
        self.rails[1].setup_itersolve('cartesian_stepper_alloc', b'y')
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())

        self.is_homing = False
        self.axes_min = toolhead.Coord([0., 0., 0., 0.])
        self.axes_max = toolhead.Coord([self.draw_width, self.draw_height, 0., 0.])

        self._init_pen_changer(config)
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

    # ── Kinematics math ───────────────────────────────────────────────────────

    def _xy_to_belts(self, dx, dy):
        """Drawing coords -> (left_belt_mm, right_belt_mm)."""
        wx      = self.draw_origin_wx + dx
        wy      = self.draw_origin_wy + dy
        half_w  = self.motor_distance / 2.0
        left_l  = math.sqrt((wx + half_w)**2 + wy**2)
        right_l = math.sqrt((wx - half_w)**2 + wy**2)
        return left_l, right_l

    def _belts_to_xy(self, left_l, right_l):
        """(left_belt_mm, right_belt_mm) -> drawing coords."""
        W           = self.motor_distance
        x_from_left = (left_l**2 - right_l**2 + W**2) / (2.0 * W)
        y_sq        = left_l**2 - x_from_left**2
        world_y     = math.sqrt(max(y_sq, 0.0))
        world_x     = x_from_left - W / 2.0
        return world_x - self.draw_origin_wx, world_y - self.draw_origin_wy

    # ── Klipper interface ─────────────────────────────────────────────────────

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        """
        Klipper calls this with stepper positions.
        stepper_left  position[0] = left belt length (mm)
        stepper_right position[1] = right belt length (mm)
        """
        left_l  = stepper_positions[self.rails[0].get_name()]
        right_l = stepper_positions[self.rails[1].get_name()]
        dx, dy  = self._belts_to_xy(left_l, right_l)
        return [dx, dy, 0.0]

    def set_position(self, newpos, homing_axes):
        """
        Klipper calls this with [x, y, z, e] in BELT-LENGTH space during
        homing moves (forcepos/homepos), and in DRAWING space after homing.

        During homing: newpos[0]=left_belt, newpos[1]=right_belt
        After homing:  we call toolhead.set_position with drawing coords,
                       which calls back here -- so we must handle both cases.

        We detect which space we're in by checking is_homing.
        """
        if self.is_homing:
            # newpos is belt lengths -- set steppers directly
            left_l  = newpos[0]
            right_l = newpos[1]
        else:
            # newpos is drawing coordinates -- convert to belt lengths
            left_l, right_l = self._xy_to_belts(newpos[0], newpos[1])
        for s in self.rails[0].get_steppers():
            s.set_position([left_l,  0., 0.])
        for s in self.rails[1].get_steppers():
            s.set_position([right_l, 0., 0.])

    def home(self, homing_state):
        """
        Home both motors simultaneously.

        forcepos and homepos are in BELT-LENGTH space:
          forcepos = [position_min, position_min, 0, 0]
          homepos  = [hypotenuse_home, hypotenuse_home, 0, 0]

        The C solver moves each stepper linearly from forcepos to homepos.
        stepper_left  moves position[0]: pos_min -> hypotenuse_home
        stepper_right moves position[1]: pos_min -> hypotenuse_home
        Both belts get longer -> both counterweights rise -> endstops trigger.
        """
        self.is_homing = True
        axes = [a for a in homing_state.get_axes() if a < len(self.rails)]
        if axes:
            pos_min = self.rails[0].get_range()[0]
            pos_end = self.rails[0].get_homing_info().position_endstop

            forcepos = [pos_min, pos_min, 0., 0.]
            homepos  = [pos_end, pos_end, 0., 0.]

            homing_state.home_rails(
                [self.rails[a] for a in axes], forcepos, homepos)

        self.is_homing = False

        # Now set toolhead to drawing coordinates
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.set_position(
            [self.homed_drawing_x, self.homed_drawing_y, 0., 0.],
            homing_axes='xy')
        logging.info("PolarDrawing: homing done, drawing pos=(%.2f, %.2f)",
                     self.homed_drawing_x, self.homed_drawing_y)

    def clear_homing_state(self, clear_axes):
        pass

    def check_move(self, move):
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
    return PolarDrawingKinematics(toolhead, config.getsection('polardrawing'))