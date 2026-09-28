# Polar Drawing Machine Kinematics for Klipper
# Repository: https://github.com/captFuture/makelangelo_klipper
#
# Design intent: this file stays a stable, machine-agnostic driver.
# Only TWO physical measurements are required to configure a new
# machine in [polardrawing] of printer.cfg:
#
#   motor_distance  -- distance between the two belt pivot points
#                       (top line), center to center. Easy to measure
#                       directly with a tape measure.
#   home_drop       -- straight vertical distance from that top line
#                       down to the center of the gondola (where the
#                       pen sits), measured AT THE HOME POSITION (both
#                       endstops triggered). Hold a tape measure level
#                       with the top line and drop it straight down to
#                       the pen -- no diagonal/triangulated measurement
#                       needed.
#
# Everything else (draw margins, draw width/height, max_belt_length,
# min_belt_length) is AUTO-DERIVED from those two numbers using the
# percentage-based defaults below. Every auto-derived value can still be
# overridden explicitly in printer.cfg (e.g. draw_width, draw_margin_top)
# if a specific machine needs hand-tuning -- an explicit config value
# always wins over the computed default.
#
# draw_margin_left is NOT a config option: it is always computed from
# draw_width, never set independently, so the two can never drift apart:
#   draw_margin_left = (motor_distance - draw_width) / 2
# This requires draw_width < motor_distance (draw_margin_left > 0).

import math
import logging
import stepper

class PolarDrawingKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()

        # --- The only two required physical measurements -------------------
        self.motor_distance = config.getfloat('motor_distance', above=0.)
        self.home_drop      = config.getfloat('home_drop', above=0.)

        # --- Everything below has a sensible default, all overridable ------

        # How much headroom to keep below the physical endstop-triggered
        # belt length when computing max_belt_length / the drawable area,
        # so a draw move never runs right up to the endstop trigger point.
        belt_safety_buffer = config.getfloat('belt_safety_buffer', 20.0, minval=0.)

        # How much headroom to keep above the belt length at the nearest
        # drawable corner when computing min_belt_length, so a draw move
        # never runs right up to a pivot.
        min_belt_safety_buffer = config.getfloat('min_belt_safety_buffer', 20.0, minval=0.)

        # Default margins as a fraction of motor_distance / home_drop.
        # 0.13 reproduces the values manually tuned on "BendArr"
        # (130mm side margin on a 1000mm motor_distance, ~206mm top
        # margin on a ~1583mm home_drop) -- a reasonable starting point
        # for a new machine, not a guarantee; verify with a corner test.
        side_margin_percent = config.getfloat('side_margin_percent', 0.13,
                                               minval=0., maxval=0.45)
        top_margin_percent  = config.getfloat('top_margin_percent', 0.13,
                                               minval=0., maxval=0.9)

        self.homing_speed = config.getfloat('homing_speed', 50.0)

        # Homing overtravel: how far past the expected home position the
        # homing move is allowed to search before Klipper gives up. Kept
        # small on purpose -- a large overtravel makes homing moves run
        # long when the gondola starts far from home, which was
        # suspected to correlate with "Internal error in stepcompress"
        # crashes.
        self.homing_overtravel = config.getfloat('homing_overtravel', 400.0)

        # Small offset used to force the toolhead's believed position
        # just off the upper travel limit before the homing move starts.
        self.home_approach_offset = config.getfloat('home_approach_offset', 1.0)

        # Padding added around the belt-length-derived travel range when
        # reporting axis_minimum/axis_maximum (used by UIs like Mainsail,
        # not enforced as a hard limit -- check_move()'s belt-length
        # check is the actual safety enforcement).
        self.axis_margin_x        = config.getfloat('axis_margin_x', 100.0)
        self.axis_margin_y_top    = config.getfloat('axis_margin_y_top', 10.0)
        self.axis_margin_y_bottom = config.getfloat('axis_margin_y_bottom', 50.0)

        # Set while a homing move is in flight. The homing move's target
        # is deliberately placed past the real home position (see
        # homing_overtravel below) so the move is stopped by the physical
        # endstop rather than by reaching its target -- check_move() must
        # not reject that intentionally-unreachable target, so it skips
        # its checks entirely while this is True.
        self.homing_active = False

        half_w = self.motor_distance / 2.0

        # home_y_world is now just home_drop directly -- no Pythagoras
        # needed since we measure the vertical drop, not the diagonal
        # belt length.
        self.home_y_world = self.home_drop

        # The diagonal belt length at home is still useful internally
        # (it's the true physical maximum -- where the endstop
        # triggers), even though it's no longer a config input.
        hypotenuse_home = math.sqrt(half_w**2 + self.home_drop**2)

        # max_belt_length: true endstop-triggered belt length, minus a
        # safety buffer. Depends only on motor_distance and home_drop.
        #   max_belt_length = sqrt((motor_distance/2)^2 + home_drop^2) - belt_safety_buffer
        auto_max_belt_length = hypotenuse_home - belt_safety_buffer
        self.max_belt_length = config.getfloat('max_belt_length', auto_max_belt_length)

        # draw_width: either an explicit override, or auto from
        # side_margin_percent. draw_margin_left is always derived from
        # whichever draw_width ends up being -- never set independently,
        # so it can't drift out of sync with draw_width.
        auto_width_default = self.motor_distance * (1.0 - 2.0 * side_margin_percent)
        self.draw_width = config.getfloat('draw_width', auto_width_default)

        self.draw_margin_left = (self.motor_distance - self.draw_width) / 2.0
        if self.draw_margin_left <= 0.:
            raise config.error(
                "PolarDrawing: draw_width (%.1f) must be less than "
                "motor_distance (%.1f) -- draw_margin_left would be zero "
                "or negative." % (self.draw_width, self.motor_distance))

        auto_top_margin = top_margin_percent * self.home_drop
        self.draw_margin_top = config.getfloat('draw_margin_top', auto_top_margin)

        # Solve for the tallest draw_height whose far (bottom) corner
        # still stays within max_belt_length -- same geometry check as
        # check_move() performs at runtime, done once here to pick a
        # safe default.
        reach_x = self.draw_width + self.draw_margin_left
        under_sqrt = self.max_belt_length**2 - reach_x**2
        if under_sqrt <= 0.:
            raise config.error(
                "PolarDrawing: draw_width leaves no room for a valid "
                "draw_height at this max_belt_length -- reduce draw_width "
                "or check motor_distance/home_drop.")
        auto_height = math.sqrt(under_sqrt) - self.draw_margin_top
        if auto_height <= 0.:
            raise config.error(
                "PolarDrawing: computed draw_height is not positive -- "
                "top_margin_percent is too large for this machine's "
                "home_drop. Reduce top_margin_percent.")

        self.draw_height = config.getfloat('draw_height', auto_height)

        # min_belt_length: belt length at the nearest drawable corner
        # (hypotenuse of draw_margin_left and draw_margin_top), minus a
        # safety buffer, so a move never runs right up to a pivot.
        #   min_belt_length = hypot(draw_margin_left, draw_margin_top) - min_belt_safety_buffer
        near_corner = math.hypot(self.draw_margin_left, self.draw_margin_top)
        auto_min_belt_length = max(0.0, near_corner - min_belt_safety_buffer)
        self.min_belt_length = config.getfloat('min_belt_length', auto_min_belt_length,
                                                minval=0.)

        # Sanity check: warn (don't hard-fail) if the resulting top
        # corners would require a belt shorter than min_belt_length --
        # only reachable if min_belt_length was overridden explicitly,
        # since the auto default always stays below near_corner.
        if near_corner < self.min_belt_length:
            logging.warning(
                "PolarDrawing: top corners are only %.1fmm from a pivot, "
                "below min_belt_length (%.1fmm). check_move() will reject "
                "moves into the top corners -- lower min_belt_length or "
                "increase top_margin_percent/draw_margin_top.",
                near_corner, self.min_belt_length)

        # Sanity check: warn if draw_margin_top + draw_height reaches (or
        # passes) home_drop -- that means the configured drawing area's
        # bottom edge is at or below the physical home position itself,
        # which is only possible if home_drop, draw_margin_top or
        # draw_height are inconsistent with each other (e.g. home_drop
        # was changed without re-checking overrides carried over from a
        # different home_drop).
        if self.draw_margin_top + self.draw_height >= self.home_drop:
            logging.warning(
                "PolarDrawing: draw_margin_top (%.1f) + draw_height (%.1f) "
                "= %.1f is >= home_drop (%.1f) -- the bottom of the "
                "configured drawing area is at or beyond the physical home "
                "position. Check that draw_margin_top/draw_height/"
                "max_belt_length still match this machine's home_drop.",
                self.draw_margin_top, self.draw_height,
                self.draw_margin_top + self.draw_height, self.home_drop)

        # Anchor points are forced strictly onto the 2D plane (Z=0).
        self.anchor_left  = (-self.draw_margin_left, -self.draw_margin_top, 0.)
        self.anchor_right = (self.motor_distance - self.draw_margin_left, -self.draw_margin_top, 0.)

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

        limit_x_min = -self.draw_margin_left - self.axis_margin_x
        limit_x_max = self.motor_distance + self.axis_margin_x
        limit_y_min = -self.draw_margin_top - self.axis_margin_y_top
        limit_y_max = self.homed_drawing_y + self.axis_margin_y_bottom

        self.axes_min = toolhead.Coord([limit_x_min, limit_y_min, 0., 0.])
        self.axes_max = toolhead.Coord([limit_x_max, limit_y_max, 0., 0.])

        logging.info(
            "PolarDrawing: motor_distance=%.1f home_drop=%.1f -> "
            "draw_margin_left=%.1f draw_margin_top=%.1f "
            "draw_width=%.1f draw_height=%.1f max_belt_length=%.1f",
            self.motor_distance, self.home_drop, self.draw_margin_left,
            self.draw_margin_top, self.draw_width, self.draw_height,
            self.max_belt_length)

        self.printer.add_object('polardrawing', self)

    def get_steppers(self):
        return list(self.steppers)

    def calc_position(self, stepper_positions):
        left_l  = stepper_positions['stepper_left']
        right_l = stepper_positions['stepper_right']
        W = self.motor_distance

        ax_l = self.anchor_left[0]
        ax_r = self.anchor_right[0]
        ay   = self.anchor_left[1]  # both anchors share the same Y

        dx = (left_l**2 - right_l**2 - ax_l**2 + ax_r**2) / (2.0 * W)

        # Plain 2D Pythagoras for the vertical component.
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

        # Homing direction: gondola DOWN => belts LONGER => counterweights
        # UP => endstops trigger. home_drop is measured AT that triggered
        # position, i.e. it's the maximum reach, not the minimum.
        fake_target = [self.homed_drawing_x,
                       self.homed_drawing_y + self.homing_overtravel, 0., 0.]

        forcepos = [self.homed_drawing_x,
                    -self.draw_margin_top + self.home_approach_offset,
                    0., 0.]
        toolhead.set_position(forcepos)

        # fake_target is intentionally past the real home position (by
        # homing_overtravel) so the move is stopped by the endstop, not
        # by reaching its target. check_move() must not reject that
        # unreachable-by-design target -- disable its checks for the
        # duration of this move.
        self.homing_active = True
        try:
            hmove.homing_move(fake_target, self.homing_speed)
        finally:
            self.homing_active = False

        homing_state.set_homed_position(
            [self.homed_drawing_x, self.homed_drawing_y, 0.])
        logging.info("PolarDrawing: homing done, drawing pos=(%.2f, %.2f)",
                     self.homed_drawing_x, self.homed_drawing_y)

    def clear_homing_state(self, clear_axes):
        pass

    def check_move(self, move):
        if self.homing_active:
            # The homing move's target is deliberately unreachable (see
            # home() above) -- the endstop is the real safety mechanism
            # for that move, not this check.
            return

        dx, dy = move.end_pos[0], move.end_pos[1]
        if dy < -self.draw_margin_top:
            raise move.move_error(
                "PolarDrawing: target Y (%.2f) is above the motors. Move denied." % dy)

        # Enforce the configured/derived belt-length safety range. Only
        # checked at the move's end position (not continuously along the
        # path), but this is still real protection where previously
        # min_belt_length / max_belt_length were read from config and
        # never used.
        ax_l, ay_l = self.anchor_left[0], self.anchor_left[1]
        ax_r, ay_r = self.anchor_right[0], self.anchor_right[1]
        left_len  = math.hypot(dx - ax_l, dy - ay_l)
        right_len = math.hypot(dx - ax_r, dy - ay_r)

        if left_len < self.min_belt_length or right_len < self.min_belt_length:
            raise move.move_error(
                "PolarDrawing: target (%.2f, %.2f) would require a belt shorter "
                "than min_belt_length (%.1f mm). Move denied."
                % (dx, dy, self.min_belt_length))

        if left_len > self.max_belt_length or right_len > self.max_belt_length:
            raise move.move_error(
                "PolarDrawing: target (%.2f, %.2f) would require a belt longer "
                "than max_belt_length (%.1f mm). Move denied."
                % (dx, dy, self.max_belt_length))

    def get_status(self, eventtime):
        return {
            'homed_axes':      'xy',
            'axis_minimum':    self.axes_min,
            'axis_maximum':    self.axes_max,
            'homed_drawing_x': self.homed_drawing_x,
            'homed_drawing_y': self.homed_drawing_y,
            'draw_width':      self.draw_width,
            'draw_height':     self.draw_height,
        }

def load_kinematics(toolhead, config):
    return PolarDrawingKinematics(toolhead, config.getsection('polardrawing'))
