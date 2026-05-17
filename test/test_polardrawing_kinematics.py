#!/usr/bin/env python3
"""
Polar Drawing Machine -- Kinematics Math Tests
Runs WITHOUT Klipper, directly on your PC or Raspberry Pi.

Usage:
    python3 test_polardrawing_kinematics.py

Edit the machine parameters section to match your machine.
All tests should pass before connecting any hardware.
"""

import math
import sys

# ── Kinematics functions (mirrored from polardrawing.py) ──────────────────────

def drawing_to_world(dx, dy, draw_origin_wx, draw_origin_wy):
    return draw_origin_wx + dx, draw_origin_wy + dy

def world_to_drawing(wx, wy, draw_origin_wx, draw_origin_wy):
    return wx - draw_origin_wx, wy - draw_origin_wy

def xy_to_belts(dx, dy, motor_distance, draw_origin_wx, draw_origin_wy):
    """Drawing (dx, dy) -> (left_belt_mm, right_belt_mm)."""
    wx, wy = drawing_to_world(dx, dy, draw_origin_wx, draw_origin_wy)
    half_w    = motor_distance / 2.0
    left_len  = math.sqrt((wx + half_w) ** 2 + wy ** 2)
    right_len = math.sqrt((wx - half_w) ** 2 + wy ** 2)
    return left_len, right_len

def belts_to_xy(left_len, right_len, motor_distance, draw_origin_wx, draw_origin_wy):
    """(left_belt_mm, right_belt_mm) -> drawing (dx, dy)."""
    W = motor_distance
    x_from_left = (left_len ** 2 - right_len ** 2 + W ** 2) / (2.0 * W)
    y_sq    = left_len ** 2 - x_from_left ** 2
    world_y = math.sqrt(max(y_sq, 0.0))
    world_x = x_from_left - W / 2.0
    return world_to_drawing(world_x, world_y, draw_origin_wx, draw_origin_wy)

# ── Machine parameters -- edit these to match YOUR machine ────────────────────

MOTOR_DISTANCE  = 650.0    # mm -- pulley centre to pulley centre
HYPOTENUSE_HOME = 1035.0   # mm -- belt length when endstops trigger

# Drawing origin offset from machine frame
DRAW_MARGIN_LEFT = 115.0   # mm -- from left motor to left edge of drawing area
DRAW_MARGIN_TOP  = 115.0   # mm -- from motor axis down to top edge of drawing area

# Drawing area -- DIN A2 landscape
DRAW_WIDTH  = 594.0        # mm
DRAW_HEIGHT = 420.0        # mm

# Belt limits
MAX_BELT_LENGTH = 1400.0   # mm
MIN_BELT_LENGTH = 100.0    # mm -- physical safety minimum

# ── Derived values (do not edit) ─────────────────────────────────────────────

HALF_W         = MOTOR_DISTANCE / 2.0
HOME_Y_WORLD   = math.sqrt(max(HYPOTENUSE_HOME ** 2 - HALF_W ** 2, 0.0))
DRAW_ORIGIN_WX = -HALF_W + DRAW_MARGIN_LEFT
DRAW_ORIGIN_WY =  DRAW_MARGIN_TOP
HOMED_DX       =  0.0            - DRAW_ORIGIN_WX
HOMED_DY       =  HOME_Y_WORLD   - DRAW_ORIGIN_WY
TOLERANCE      = 0.001   # mm

# ── Tests ─────────────────────────────────────────────────────────────────────

def test_coordinate_system():
    """Verify drawing origin maps to the correct world position."""
    print("\n-- Coordinate system --")
    # Drawing (0,0) must map to world (draw_origin_wx, draw_origin_wy)
    wx, wy = drawing_to_world(0.0, 0.0, DRAW_ORIGIN_WX, DRAW_ORIGIN_WY)
    wx_ok = abs(wx - DRAW_ORIGIN_WX) < TOLERANCE
    wy_ok = abs(wy - DRAW_ORIGIN_WY) < TOLERANCE
    print(f"  Drawing (0, 0):")
    print(f"    World X: {wx:.3f}  expected {DRAW_ORIGIN_WX:.3f}  {'PASS' if wx_ok else 'FAIL'}")
    print(f"    World Y: {wy:.3f}  expected {DRAW_ORIGIN_WY:.3f}  {'PASS' if wy_ok else 'FAIL'}")

    # Drawing (0,0) must be 115mm right of left motor
    left_motor_x = -HALF_W
    x_from_left_motor = wx - left_motor_x
    margin_ok = abs(x_from_left_motor - DRAW_MARGIN_LEFT) < TOLERANCE
    print(f"  Drawing origin is {x_from_left_motor:.1f} mm right of left motor "
          f"(expected {DRAW_MARGIN_LEFT:.1f})  {'PASS' if margin_ok else 'FAIL'}")

    # Drawing (0,0) must be 115mm below motor axis
    y_from_motor = wy
    top_ok = abs(y_from_motor - DRAW_MARGIN_TOP) < TOLERANCE
    print(f"  Drawing origin is {y_from_motor:.1f} mm below motor axis "
          f"(expected {DRAW_MARGIN_TOP:.1f})  {'PASS' if top_ok else 'FAIL'}")

    return wx_ok and wy_ok and margin_ok and top_ok

def test_home_position():
    """
    After homing, belts = HYPOTENUSE_HOME.
    Forward kinematics must return the correct drawing coordinates.
    """
    print("\n-- Home position --")
    dx, dy = belts_to_xy(HYPOTENUSE_HOME, HYPOTENUSE_HOME,
                         MOTOR_DISTANCE, DRAW_ORIGIN_WX, DRAW_ORIGIN_WY)
    dx_ok = abs(dx - HOMED_DX) < TOLERANCE
    dy_ok = abs(dy - HOMED_DY) < TOLERANCE
    print(f"  Belt lengths at home: L={HYPOTENUSE_HOME}  R={HYPOTENUSE_HOME}")
    print(f"  Drawing coordinates:  X={dx:.3f}  Y={dy:.3f}")
    print(f"  Expected:             X={HOMED_DX:.3f}  Y={HOMED_DY:.3f}")
    print(f"  X: {'PASS' if dx_ok else 'FAIL'}   Y: {'PASS' if dy_ok else 'FAIL'}")
    print(f"  (Gondola is {HOME_Y_WORLD:.1f} mm below motor axis at homing)")
    if dy > DRAW_HEIGHT:
        print(f"  NOTE: homed Y ({dy:.1f}) is below the drawing area ({DRAW_HEIGHT:.1f})")
        print(f"        After homing, move to G0 X0 Y0 to reach the drawing top-left corner.")
    return dx_ok and dy_ok

def test_drawing_corners():
    """
    Verify belt lengths at all four drawing corners.
    All must be within [MIN_BELT_LENGTH, MAX_BELT_LENGTH].
    """
    print("\n-- Drawing corners --")
    ok = True
    corners = [
        (0.0,        0.0,         "top-left    (0, 0)"),
        (DRAW_WIDTH, 0.0,         f"top-right   ({DRAW_WIDTH:.0f}, 0)"),
        (0.0,        DRAW_HEIGHT, f"bottom-left (0, {DRAW_HEIGHT:.0f})"),
        (DRAW_WIDTH, DRAW_HEIGHT, f"bottom-right ({DRAW_WIDTH:.0f}, {DRAW_HEIGHT:.0f})"),
    ]
    for dx, dy, label in corners:
        l, r = xy_to_belts(dx, dy, MOTOR_DISTANCE, DRAW_ORIGIN_WX, DRAW_ORIGIN_WY)
        wx, wy = drawing_to_world(dx, dy, DRAW_ORIGIN_WX, DRAW_ORIGIN_WY)
        in_range = (MIN_BELT_LENGTH <= l <= MAX_BELT_LENGTH and
                    MIN_BELT_LENGTH <= r <= MAX_BELT_LENGTH)
        print(f"  {'PASS' if in_range else 'FAIL'}  {label:30s}"
              f"  world=({wx:6.1f},{wy:6.1f})"
              f"  L={l:7.2f}  R={r:7.2f}"
              f"  {'OK' if in_range else '*** OUT OF RANGE ***'}")
        ok = ok and in_range
    return ok

def _roundtrip(dx, dy, label=""):
    """Drawing XY -> belts -> drawing XY must be identical."""
    l, r     = xy_to_belts(dx, dy, MOTOR_DISTANCE, DRAW_ORIGIN_WX, DRAW_ORIGIN_WY)
    dx2, dy2 = belts_to_xy(l, r,   MOTOR_DISTANCE, DRAW_ORIGIN_WX, DRAW_ORIGIN_WY)
    ex, ey   = abs(dx - dx2), abs(dy - dy2)
    ok = ex < TOLERANCE and ey < TOLERANCE
    print(f"  {'PASS' if ok else 'FAIL'}  {label:30s}"
          f"  ({dx:7.1f},{dy:6.1f})"
          f" -> L={l:8.2f} R={r:8.2f}"
          f" -> ({dx2:7.1f},{dy2:6.1f})"
          f"  err=({ex:.5f},{ey:.5f})")
    return ok

def test_roundtrips():
    """Roundtrip accuracy across the drawing area."""
    print("\n-- Roundtrip tests (drawing XY -> belts -> drawing XY) --")
    positions = [
        (0.0,          0.0,           "origin (top-left)"),
        (DRAW_WIDTH,   0.0,           "top-right"),
        (0.0,          DRAW_HEIGHT,   "bottom-left"),
        (DRAW_WIDTH,   DRAW_HEIGHT,   "bottom-right"),
        (DRAW_WIDTH/2, DRAW_HEIGHT/2, "centre"),
        (100.0,        50.0,          "near top-left"),
        (500.0,        350.0,         "near bottom-right"),
        (DRAW_WIDTH/2, 0.0,           "top-centre"),
        (DRAW_WIDTH/2, DRAW_HEIGHT,   "bottom-centre"),
    ]
    return all(_roundtrip(dx, dy, label) for dx, dy, label in positions)

def test_symmetry():
    """
    Horizontal symmetry around the machine centre.
    Points equidistant from the machine centreline but on opposite sides
    must have swapped belt lengths.
    Machine centreline in drawing coords: X = HALF_W - DRAW_MARGIN_LEFT
    """
    print("\n-- Horizontal symmetry --")
    # Machine centre in drawing coordinates
    cx = HALF_W - DRAW_MARGIN_LEFT   # = 325 - 115 = 210 mm
    ok = True
    for offset in [50, 100, 150, 200]:
        y = DRAW_HEIGHT / 2
        l1, r1 = xy_to_belts(cx + offset, y, MOTOR_DISTANCE, DRAW_ORIGIN_WX, DRAW_ORIGIN_WY)
        l2, r2 = xy_to_belts(cx - offset, y, MOTOR_DISTANCE, DRAW_ORIGIN_WX, DRAW_ORIGIN_WY)
        sym = abs(l1 - r2) < TOLERANCE and abs(r1 - l2) < TOLERANCE
        print(f"  {'PASS' if sym else 'FAIL'}  offset=+/-{offset:3d} mm from centre  "
              f"right side: L={l1:.2f} R={r1:.2f}   "
              f"left side:  L={l2:.2f} R={r2:.2f}")
        ok = ok and sym
    return ok

def print_machine_summary():
    print("\n-- Machine geometry summary --")
    print(f"  Motor distance:              {MOTOR_DISTANCE:.1f} mm")
    print(f"  Hypotenuse at home:          {HYPOTENUSE_HOME:.1f} mm")
    print(f"  Gondola below motor axis:    {HOME_Y_WORLD:.1f} mm  (at homing)")
    print(f"  Drawing margin left:         {DRAW_MARGIN_LEFT:.1f} mm")
    print(f"  Drawing margin top:          {DRAW_MARGIN_TOP:.1f} mm")
    print(f"  Drawing origin (world):      X={DRAW_ORIGIN_WX:.1f}  Y={DRAW_ORIGIN_WY:.1f}")
    print(f"  Drawing area:                {DRAW_WIDTH:.0f} x {DRAW_HEIGHT:.0f} mm  (DIN A2 landscape)")
    print(f"  Homed position (drawing):    X={HOMED_DX:.1f}  Y={HOMED_DY:.1f} mm")
    print(f"  After homing:                G0 X0 Y0 moves gondola to top-left corner of paper")

# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 72)
    print("Polar Drawing Machine -- Kinematics Math Tests")
    print(f"  motor_distance   = {MOTOR_DISTANCE} mm")
    print(f"  hypotenuse_home  = {HYPOTENUSE_HOME} mm")
    print(f"  draw_margin_left = {DRAW_MARGIN_LEFT} mm")
    print(f"  draw_margin_top  = {DRAW_MARGIN_TOP} mm")
    print(f"  draw area        = {DRAW_WIDTH} x {DRAW_HEIGHT} mm  (DIN A2 landscape)")
    print("=" * 72)

    results = [
        test_coordinate_system(),
        test_home_position(),
        test_drawing_corners(),
        test_roundtrips(),
        test_symmetry(),
    ]

    print_machine_summary()

    passed = sum(results)
    total  = len(results)
    print("\n" + "=" * 72)
    print(f"Result: {passed}/{total} test groups passed")
    if passed == total:
        print("All tests PASSED.")
    else:
        print("FAILURES detected -- review output above before connecting hardware.")
    print("=" * 72)
    sys.exit(0 if passed == total else 1)