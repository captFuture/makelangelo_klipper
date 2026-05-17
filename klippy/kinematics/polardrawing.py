# Polar Drawing Machine Kinematics for Klipper
# Repository: https://github.com/captFuture/makelangelo_klipper

import math
import logging
import stepper

class PolarDrawingKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()

        self.motor_distance   = config.getfloat('motor_distance', above=0.)
        self.hypotenuse_home  = config.getfloat('hypotenuse_length_at_home', 1060.0)
        self.max_belt_length  = config.getfloat('max_belt_length', 1450.0)
        self.min_belt_length  = config.getfloat('min_belt_length', 100.0)
        self.draw_margin_left = config.getfloat('draw_margin_left', 115.0)
        self.draw_margin_top  = config.getfloat('draw_margin_top',  150.0)
        self.draw_width       = config.getfloat('draw_width',  420.0)
        self.draw_height      = config.getfloat('draw_height', 594.0)
        self.homing_speed     = config.getfloat('homing_speed', 50.0)

        half_w = self.motor_distance / 2.0

        # FIX: Ankerpunkte strikt auf die 2D-Fläche (Z=0) gezwungen!
        self.anchor_left  = (-self.draw_margin_left, -self.draw_margin_top, 0.)
        self.anchor_right = (self.motor_distance - self.draw_margin_left, -self.draw_margin_top, 0.)

        self.home_y_world    = math.sqrt(
            max(self.hypotenuse_home**2 - half_w**2, 0.0))
        draw_origin_wx = -half_w + self.draw_margin_left
        draw_origin_wy =  self.draw_margin_top
        
        self.homed_drawing_x = 0.0              - draw_origin_wx
        self.homed_drawing_y = self.home_y_world - draw_origin_wy

        self.steppers = []
        anchors = [self.anchor_left, self.anchor_right]
        for name, anchor in zip(['stepper_left', 'stepper_right'], anchors):
            s = stepper.PrinterStepper(config.getsection(name))
            s.setup_itersolve('winch_stepper_alloc', *anchor)
            s.set_trapq(toolhead.get_trapq())
            self.steppers.append(s)

        ppins = self.printer.lookup_object('pins')
        self.endstops = []
        for i, name in enumerate(['stepper_left', 'stepper_right']):
            sc  = config.getsection(name)
            pin = sc.get('endstop_pin')
            mcu_es = ppins.setup_pin('endstop', pin)
            mcu_es.add_stepper(self.steppers[i])
            self.endstops.append((mcu_es, name))

        self.homing_speed_cfg = config.getfloat('homing_speed', 50.0)

        limit_x_min = -self.draw_margin_left - 100.0
        limit_x_max = self.motor_distance + 100.0
        limit_y_min = -self.draw_margin_top - 10.0
        limit_y_max = self.homed_drawing_y + 50.0
        
        self.axes_min = toolhead.Coord([limit_x_min, limit_y_min, 0., 0.])
        self.axes_max = toolhead.Coord([limit_x_max, limit_y_max, 0., 0.])

        self._init_pen_changer(config)
        self.printer.add_object('polardrawing', self)

    def _init_pen_changer(self, config):
        self.pen_changer_angle_pin = config.get('pen_changer_angle_sensor_pin', None)
        self.pen_changer_servo_pin = config.get('pen_changer_servo_pin', None)
        self.pen_count           = config.getint('pen_count', 4, minval=1)
        self.current_pen         = 0
        self.pen_changer_enabled = (self.pen_changer_angle_pin is not None
                                    and self.pen_changer_servo_pin is not None)

    def get_steppers(self):
        return list(self.steppers)

    def calc_position(self, stepper_positions):
        left_l  = stepper_positions['stepper_left']
        right_l = stepper_positions['stepper_right']
        W = self.motor_distance
        
        ax_l = self.anchor_left[0]
        ax_r = self.anchor_right[0]
        ay   = self.anchor_left[1] # FIX: Y-Achse korrekt mappen
        
        dx = (left_l**2 - right_l**2 - ax_l**2 + ax_r**2) / (2.0 * W)
        
        # FIX: Saubere 2D Pythagoras Berechnung für die vertikale Achse
        dy_sq = left_l**2 - (dx - ax_l)**2
        dy = math.sqrt(max(dy_sq, 0.0)) + ay
        return [dx, dy, 0.0]

    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)

    def home(self, homing_state):
        homing_state.set_axes([0, 1])

        from extras import homing as homing_mod
        toolhead = self.printer.lookup_object('toolhead')

        hmove = homing_mod.HomingMove(self.printer, self.endstops)
        
        fake_target = [self.homed_drawing_x, self.homed_drawing_y + 2000.0, 0., 0.]
        
        forcepos = [self.homed_drawing_x,
                    -self.draw_margin_top + 1.0,
                    0., 0.]
        toolhead.set_position(forcepos)
        
        hmove.homing_move(fake_target, self.homing_speed)

        homing_state.set_homed_position(
            [self.homed_drawing_x, self.homed_drawing_y, 0.])
        logging.info("PolarDrawing: homing done, drawing pos=(%.2f, %.2f)",
                     self.homed_drawing_x, self.homed_drawing_y)

    def clear_homing_state(self, clear_axes):
        pass

    def check_move(self, move):
        dx, dy = move.end_pos[0], move.end_pos[1]
        if dy < -self.draw_margin_top:
            raise move.move_error(
                "PolarDrawing: Target Y (%.2f) is above the motors. Move denied." % dy)

    def get_status(self, eventtime):
        return {
            'homed_axes':          'xy',
            'axis_minimum':        self.axes_min,
            'axis_maximum':        self.axes_max,
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