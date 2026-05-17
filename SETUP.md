# polardrawing — Complete Setup, Flash & Test Guide

**Klipper fork for the Makelangelo polar drawing machine**  
RUMBA board (ATmega2560) + Raspberry Pi + RepRapDiscount Smart Controller

---

## Table of contents

1. [What you need](#1-what-you-need)
2. [Repository structure and file placement](#2-repository-structure-and-file-placement)
3. [Step 1 — Prepare the Raspberry Pi](#step-1--prepare-the-raspberry-pi)
4. [Step 2 — Fork Klipper and add the kinematics file](#step-2--fork-klipper-and-add-the-kinematics-file)
5. [Step 3 — Install Klipper stack with KIAUH](#step-3--install-klipper-stack-with-kiauh)
6. [Step 4 — Compile firmware for the RUMBA](#step-4--compile-firmware-for-the-rumba)
7. [Step 5 — Flash the RUMBA board](#step-5--flash-the-rumba-board)
8. [Step 6 — Place project files](#step-6--place-project-files)
9. [Step 7 — Edit printer.cfg](#step-7--edit-printercfg)
10. [Step 8 — Start Klipper](#step-8--start-klipper)
11. [Step 9 — Wiring reference](#step-9--wiring-reference)
12. [Step 10 — Test sequence](#step-10--test-sequence)
13. [Step 11 — First plot](#step-11--first-plot)
14. [Troubleshooting](#troubleshooting)
15. [Machine geometry reference](#machine-geometry-reference)

---

## 1. What you need

### Hardware
| Item | Details |
|---|---|
| Makelangelo drawing machine | Motors 650 mm apart, 1035 mm belt (motor to pen tip at home) |
| RUMBA board | ATmega2560, connected via USB to the Pi |
| Raspberry Pi | Model 3B or newer |
| RepRapDiscount Smart Controller | 20×4 HD44780 LCD, rotary encoder, beeper, reset button |
| Two 10-pin ribbon cables | Supplied with the Smart Controller |
| USB Mini-B cable | Pi → RUMBA |
| MicroSD card | 8 GB minimum for the Pi |

### Software (all installed during this guide)
- Raspberry Pi OS Lite 64-bit
- KIAUH → installs Klipper, Moonraker, Mainsail, avrdude

---

## 2. Repository structure and file placement

```
GitHub: github.com/captFuture/makelangelo_klipper   (your Klipper fork)
│
├── klippy/
│   └── kinematics/
│       └── polardrawing.py          ← NEW: polargraph kinematics
│                                        alongside cartesian.py, delta.py …
├── config/
│   └── printer_polardrawing_rumba.cfg  ← reference config
├── test_polardrawing_kinematics.py
└── docs/
    └── SETUP.md                     ← this file


Raspberry Pi filesystem
│
├── ~/makelangelo_klipper/                       running Klipper (your fork)
│   └── klippy/kinematics/
│       └── polardrawing.py          ← must be here
│
└── ~/printer_data/config/
    ├── printer.cfg                  ← active config (copied & edited)
    └── mainsail.cfg                 ← auto-created by KIAUH
```

---

## Step 1 — Prepare the Raspberry Pi

### Flash the OS

1. Download **Raspberry Pi Imager** → https://www.raspberrypi.com/software/
2. Choose **Raspberry Pi OS Lite (64-bit)**
3. In the imager settings (gear icon):
   - Hostname: `makelangelo`
   - Enable SSH with password
   - Username: `pi`, set a password
   - Configure Wi-Fi if needed
4. Flash, insert SD card, power on

### Connect and update

```bash
ssh pi@makelangelo.local

sudo apt update && sudo apt upgrade -y
sudo reboot
```

---

## Step 2 — Fork Klipper and add the kinematics file

### On GitHub

1. Go to https://github.com/Klipper3d/klipper
2. Click **Fork** — name it `makelangelo_klipper`

### Add polardrawing.py to your fork

On your PC:
```bash
git clone https://github.com/captFuture/makelangelo_klipper.git
# copy polardrawing.py into the repo
cp polardrawing.py polardrawing/klippy/kinematics/
cd polardrawing
git add klippy/kinematics/polardrawing.py
git commit -m "Add polardrawing kinematics"
git push
```

Or use the GitHub web interface: navigate to `klippy/kinematics/`, click **Add file → Upload file**.

### Clone onto the Raspberry Pi

```bash
# On the Pi — this directory IS the running Klipper installation
git clone https://github.com/captFuture/makelangelo_klipper.git ~/makelangelo_klipper
```

---

## Step 3 — Install Klipper stack with KIAUH

```bash
# On the Pi
git clone https://github.com/dw-0/kiauh.git ~/kiauh
~/kiauh/kiauh.sh
```

Select **1) Install** and install in this order:

| # | Component | Note |
|---|---|---|
| 1 | **Klipper** | When asked for the Klipper repo path, enter `~/makelangelo_klipper` |
| 2 | **Moonraker** | API server — required by Mainsail |
| 3 | **Mainsail** | Web UI |

After installation, open **http://makelangelo.local** in your browser.
It will show errors until `printer.cfg` is in place — that is expected.

---

## Step 4 — Compile firmware for the RUMBA

```bash
cd ~/makelangelo_klipper
make menuconfig
```

Set **exactly** these options (arrow keys to navigate, space to toggle):

```
[*] Enable extra low-level configuration options

    Micro-controller Architecture  →  Atmel AVR
    Processor model                →  atmega2560
    Communication interface        →  UART0
    Baud rate for serial port      →  250000
```

Everything else: leave at default. Press **Q** then **Y** to save.

```bash
make clean && make
```

Success ends with:
```
  Creating hex file out/klipper.elf.hex
```

---

## Step 5 — Flash the RUMBA board

### Connect the RUMBA via USB

```bash
ls /dev/serial/by-id/
```

Output looks like:
```
usb-Arduino__www.arduino.cc__0042_55636313530351611171-if00
```

Copy the full path — you need it in `printer.cfg`.

### Flash

```bash
avrdude -c stk500v2 \
        -p atmega2560 \
        -P /dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_YOUR_ID-if00 \
        -u \
        -Uflash:w:out/klipper.elf.hex:i
```

Successful output:
```
avrdude: Device signature = 0x1e9801 (probably m2560)
avrdude: writing flash (28672 bytes) ...
avrdude done.  Thank you.
```

> **Permission error?** Run:
> ```bash
> sudo usermod -aG dialout pi
> # log out and back in, then retry
> ```

---

## Step 6 — Place project files

### Kinematics module — verify it is present

```bash
ls ~/makelangelo_klipper/klippy/kinematics/polardrawing.py
```

If missing:
```bash
# copy from your local checkout or download from your GitHub fork
cp /path/to/polardrawing.py ~/makelangelo_klipper/klippy/kinematics/
```

### Active printer config

```bash
cp ~/makelangelo_klipper/config/printer_polardrawing_rumba.cfg \
   ~/printer_data/config/printer.cfg
```

---

## Step 7 — Edit printer.cfg

```bash
nano ~/printer_data/config/printer.cfg
```

### Required changes

**Serial port** — paste your actual device ID from Step 5:
```ini
[mcu]
serial: /dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_YOUR_ID-if00
```

**Machine dimensions** — confirm or update:
```ini
[polardrawing]
motor_distance: 650.0              # pulley centre to pulley centre (mm)
hypotenuse_length_at_home: 1035.0  # motor pulley to pen tip when homed (mm)
draw_margin_left: 115.0            # mm from left motor to left paper edge
draw_margin_top:  115.0            # mm from motor axis to top paper edge
draw_width:  594.0                 # DIN A2 landscape
draw_height: 420.0
```

**Belt travel range** — set to match your frame:
```ini
[stepper_left]
position_max: 1400.0    # increase if gondola needs to travel further down

[stepper_right]
position_max: 1400.0
```

**Right motor direction** — check during Test 5:
```ini
[stepper_right]
dir_pin: PL0    # add ! prefix (→ !PL0) if motor drives the wrong way
```

### Restart Klipper after every printer.cfg edit

```bash
sudo systemctl restart klipper
```

Or use the **Restart** button in Mainsail.

---

## Step 8 — Start Klipper

```bash
sudo systemctl restart klipper
sudo systemctl restart moonraker

# Watch the log for errors
sudo journalctl -u klipper -f
```

Open **http://makelangelo.local** — the status bar should show **Ready**.

### Common startup errors

| Error | Fix |
|---|---|
| `Unable to open serial port` | Wrong `serial:` path — check Step 5 |
| `Unknown kinematics type 'polardrawing'` | `polardrawing.py` not in `~/makelangelo_klipper/klippy/kinematics/` |
| `Option 'motor_distance' must be specified` | `[polardrawing]` section missing from `printer.cfg` |
| `mcu 'mcu': Unable to connect` | RUMBA not flashed, or USB cable issue |

---

## Step 9 — Wiring reference

### Smart Controller ribbon cables

The Smart Controller and RUMBA both have connectors labelled EXP1 and EXP2.

> **Important:** On most RUMBA revisions the silkscreen labels are swapped.
> Connect:
> - Smart Controller **EXP1** → RUMBA **EXP2**
> - Smart Controller **EXP2** → RUMBA **EXP1**
>
> If LCD shows only backlight (no characters): swap the two cables.

### All pins at a glance

| Signal | ATmega port | RUMBA connector |
|---|---|---|
| Left motor STEP | PC0 | X_STEP |
| Left motor DIR | PL1 | X_DIR |
| Left motor ENABLE | PA7 | X_EN |
| **Left endstop (MAX)** | PE3 | X_MAX |
| Right motor STEP | PC6 | Y_STEP |
| Right motor DIR | PL0 | Y_DIR |
| Right motor ENABLE | PA7 | Y_EN (shared) |
| **Right endstop (MAX)** | PJ1 | Y_MAX |
| Pen servo signal | PH3 | FAN0 |
| LCD RS | PD2 | via EXP |
| LCD Enable | PL7 | via EXP |
| LCD D4 / D5 / D6 / D7 | PD3 / PD7 / PG0 / PG1 | via EXP |
| Encoder A / B | PB5 / PB6 | via EXP |
| Encoder click | PL6 | via EXP |
| Beeper | PG5 | via EXP |
| Kill / Reset button | PG0 | via EXP |

### Endstop logic

- Both are **MAX** endstops — the counterweight travels **up** and presses the switch
- `^` in `endstop_pin` activates the internal pull-up resistor
- Normal state: **OPEN** (gondola low, counterweight not touching switch)
- Homed state: **TRIGGERED** (counterweight pressing switch)

---

## Step 10 — Test sequence

Run in order. Stop if a test fails — fix it before proceeding.

### Test 1 — Klipper responds

In the Mainsail console:
```
M115
```
Response includes `FIRMWARE_VERSION` and `MACHINE_TYPE`.

---

### Test 2 — Kinematics module loaded

```
POLARDRAWING_INFO
```

Expected output:
```
=== Polar Drawing Machine ===
Motor distance:          650.0 mm
Belt at home:            1035.0 mm
Home Y (calculated):     982.6 mm below motor axis
Drawing area X:          -315.0 to 315.0 mm
Drawing area Y:          0 to ~300 mm
Current pen:             0 of 4
Pen changer:             placeholder (not yet fitted)
```

---

### Test 3 — LCD, beeper, and encoder

LCD should show the Klipper status screen automatically.

```
BEEP
```
One short beep → beeper wired correctly.

Rotate the encoder knob → menu should scroll on the LCD.
Press the encoder → enters menu item.
Press the reset button → Klipper emergency stop triggers (red banner in Mainsail — click **Restart** to recover).

---

### Test 4 — Endstops (no motion)

With gondola low (counterweights away from switches):
```
QUERY_ENDSTOPS
```
Expected:
```
x:OPEN  y:OPEN
```

Manually press each switch by hand:
```
QUERY_ENDSTOPS
```
Expected: the pressed side shows `TRIGGERED`.

If a switch shows `TRIGGERED` at rest: remove `!` from its `endstop_pin`.
If a switch never shows `TRIGGERED`: check wiring, add `^` for pull-up.

---

### Test 5 — Motor directions (no belts)

```
STEPPER_BUZZ STEPPER=stepper_left
```

The left motor shaft oscillates. Observe which way the belt feed goes in the **positive** direction (belt should lengthen = gondola descends = counterweight rises).

```
STEPPER_BUZZ STEPPER=stepper_right
```

Same check for the right motor.

**To invert a motor:** add or remove `!` from its `dir_pin` in `printer.cfg`, then restart Klipper.

---

### Test 6 — Math test on the Pi

```bash
python3 ~/makelangelo_klipper/test_polardrawing_kinematics.py
```

All 5 groups must show **PASS**. Final summary:
```
Gondola below motor axis: 982.6 mm  (at homing)
Drawing area:             594 x 420 mm  (DIN A2 landscape)
Homed position (drawing): X=210.0  Y=867.6 mm
After homing:             G0 X0 Y0 moves gondola to top-left corner of paper
```

---

### Test 7 — First homing

Fit the belts. Manually position the gondola roughly in the centre of the machine.
Ensure nothing blocks the counterweights from rising freely.

```
G28
```

**Expected sequence:**
1. Pen servo raises (pen up)
2. Both motors drive simultaneously — gondola rises
3. Both counterweights hit their endstops — motors stop
4. Gondola moves to drawing origin `X=0 Y=0` (top-left corner of paper area)
5. LCD shows `X:0.00  Y:0.00`

**If only one motor stops:** that endstop is not triggering. Check wiring and `QUERY_ENDSTOPS`.

**If gondola descends instead of rising:** motor direction wrong. Toggle `!` on both `dir_pin` entries.

---

### Test 8 — Pen servo calibration

```
CALIBRATE_PEN UP=90 DOWN=40
```

The servo cycles between the two angles.
- `UP` angle: pen fully clear of paper
- `DOWN` angle: pen presses paper with light, even pressure

Adjust the numbers until both are correct. Update `PEN_UP` and `PEN_DOWN` macros in `printer.cfg` and restart Klipper.

---

### Test 9 — Draw a test rectangle

After successful homing:
```
G28
PEN_DOWN
G1 X594 Y0   F3000
G1 X594 Y420 F3000
G1 X0   Y420 F3000
G1 X0   Y0   F3000
PEN_UP
```

This draws the full DIN A2 border. Check:
- Lines are straight (not curved) → kinematics correct
- Corners are at 90° → geometry consistent
- Line weight is even → servo pressure correct
- Rectangle is the correct size (594 × 420 mm) → `motor_distance` and margins correct

---

## Step 11 — First plot

### Via Makelangelo software

Connect the Makelangelo software to the virtual serial port:
- **Port:** `/tmp/printer`
- **Baud:** `250000`

### Via Mainsail web interface

1. Open **http://makelangelo.local**
2. Drag a `.gcode` file into the file manager
3. Click **Print**

### G-code compatibility

| Makelangelo sends | Klipper action |
|---|---|
| `G28` | Home both motors, move to drawing origin |
| `G0 X Y F` / `G1 X Y F` | Move gondola |
| `G2` / `G3` | Arc interpolation |
| `M280 P0 S90` | Pen up |
| `M280 P0 S40` | Pen down |
| `M18` / `M84` | Disable motors |
| `CHANGE_PEN P0` | Pen slot (no-op until carousel fitted) |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| LCD: backlight only, no characters | EXP1/EXP2 swapped | Swap the two ribbon cables |
| LCD: garbage characters | Wrong `lcd_type` | Confirm `lcd_type: hd44780` |
| Encoder scrolls wrong direction | A/B pins swapped | Swap order of `encoder_pins` |
| Beeper silent | Wrong pin or no PWM | Check `pin: PG5`, `pwm: True` |
| Endstop always TRIGGERED | Inverted logic | Remove `!` from `endstop_pin` |
| Endstop never TRIGGERED | No pull-up / bad wiring | Add `^` to `endstop_pin`, check cable |
| Gondola moves wrong direction | `dir_pin` polarity | Toggle `!` on `dir_pin` |
| Drawing stretched vertically | `motor_distance` wrong | Re-measure pulley centre-to-centre |
| Drawing shifted on paper | Margins wrong | Re-measure `draw_margin_left` / `draw_margin_top` |
| Top-right corner unreachable | `max_belt_length` too low | Increase `max_belt_length` and `position_max` |
| `Unknown kinematics: polardrawing` | File not found | Check `~/makelangelo_klipper/klippy/kinematics/polardrawing.py` |
| `Option 'motor_distance' must be specified` | Wrong section name | Check `[polardrawing]` in `printer.cfg` |

---

## Machine geometry reference

Calculated from your measured values (not configurable — derived automatically):

```
Motor distance:               650.0 mm   (measured)
Belt at home:                 1035.0 mm  (measured)

Gondola at homing:
  World Y below motor axis:   982.6 mm
  Drawing coordinates:        X=210.0  Y=867.6
  → After homing, G0 X0 Y0 moves gondola to top-left corner of paper

Drawing origin (0,0) in world:
  X = -210.0 mm  (115 mm right of left motor)
  Y =  115.0 mm  (115 mm below motor axis)

DIN A2 landscape drawing area:
  X: 0 to 594 mm
  Y: 0 to 420 mm

Belt lengths at drawing corners:
  Top-left  ( 0,   0 ):  L=162.6 mm  R=547.2 mm
  Top-right (594,  0 ):  L=718.3 mm  R=129.3 mm
  Bot-left  ( 0, 420 ):  L=547.2 mm  R=756.6 mm
  Bot-right (594, 420):  L=888.2 mm  R=538.2 mm
```
