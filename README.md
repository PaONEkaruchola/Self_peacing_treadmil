# Self-Pacing Treadmill Controller

A Simulink-based real-time controller that lets a split-belt instrumented treadmill match the walker's own speed instead of forcing the walker to match a fixed belt speed. The belt speed is driven by an estimate of the walker's center-of-mass (COM) position and velocity, so as the person drifts forward or backward on the deck, the treadmill accelerates or decelerates to bring them back to a target position.

The estimator fuses two sources: COM acceleration measured from the force plates and step-by-step position/velocity computed from heel-strike events, combined in a Kalman filter. A parallel motion-capture path provides an independent position/velocity estimate for validation.

This repository contains the MATLAB Function block code extracted from the Simulink model, organized by the subsystem each block belongs to.

## What it does

- Detects heel-strike and toe-off events from vertical ground reaction force, with debouncing to reject false triggers during double support.
- Estimates COM position and velocity by fusing force-plate acceleration (continuous prediction) with per-step measurements (correction on each heel strike).
- Computes a target belt speed from a proportional-plus-velocity control law, with gains scheduled on the current belt speed.
- Builds and transmits binary command packets to the treadmill hardware, including per-belt speed, acceleration, incline, and a checksum.
- Includes a rapid-deceleration path for perturbation studies and a clean shutdown command that ramps both belts to zero.

## How it works

The signal flow, roughly in order:

**1. Force-plate conditioning.** Raw force-plate channels are scaled by a fixed calibration matrix (`Calibration_Matrices`) into forces and torques, then zero-offset removed by a rolling-average tare (`Zero_Cal_Timed5`).

**2. Gait event detection.** `Detect_Foot_Down` thresholds the vertical force (Fz) on each plate to flag when a foot is loaded. `Trigger_Generator_betterTrig` (`HS_filter`) debounces the raw down/up signal so brief dropouts near a false threshold crossing don't register as extra steps, and it timestamps rise (toe-off) and fall (heel-strike) events. `Pert_Safety` enforces a minimum time between accepted strikes so a single step can't be double-counted.

**3. State estimation (`Kalman_filter1`).** This is the core. A constant-acceleration Kalman filter runs every time step:
   - **Predict** using COM acceleration `a = (Fy_Left + Fy_Right) / mass`.
   - **Correct** only on a new heel strike, using the position of the foot at strike (derived from the center-of-pressure formula `-h*Fy + Tx) / Fz`) and the step velocity `step_length / step_time`. The correction compares the measured step values against the running average of the predicted state since the last strike.
   
   Outputs are the estimated COM position `p_e` and velocity `v_e`, plus the raw per-step measurements and the pure prediction for diagnostics.

**4. Motion-capture cross-check (`MotionCap_values`).** In parallel, marker position is differentiated and low-pass filtered (4th-order Butterworth, 4 Hz) to produce an independent velocity/acceleration estimate, and step velocity is computed on each heel strike. This path is for validation against the force-plate estimate. `marker_fileter` sorts raw marker channels into left/right hip, knee, and ankle by position, with hold-last-value logic for dropped markers; `TLA_convert` computes thigh/leg angles from hip and ankle positions.

**5. Speed control (`Speed_control` + `Gain_change`).** On each step, the target belt-speed change is `del_v = Gp*(p_off - p_e) + Gv*v_e`, i.e. push back toward the target position `p_off` and follow the walker's velocity. `Gain_change` schedules `Gp` and `Gv` linearly with the current belt speed. Target belt speed is clamped to a configured range and target acceleration is computed to hit that speed within `del_t_tgt`.

**6. Command output (`Send_Treadmill_Command`).** `Build_Packet` packs left/right speed (mm/s), acceleration (mm/s²), and incline into a 37-byte packet with a big-endian split of each 16-bit value and a bitwise-inverted checksum tail. `Build_Packet1` adds a fast-decel branch for perturbations. `Shutdown_Set_Speed` emits a zero-speed packet to stop both belts.

## Repository layout

All files live under `Extracted_M_Scripts/`. Names follow the Simulink hierarchy `<model>_<subsystem>_<block>.m`.

### Core estimator and control
| File | Role |
|------|------|
| `..._Subsystem2_Kalman_filter1.m` | COM position/velocity Kalman filter fusing force-plate acceleration and per-step measurements |
| `..._Subsystem2_Speed_control.m` | Proportional + velocity control law producing target belt speed and acceleration |
| `..._Subsystem2_Gain_change.m` | Speed-scheduled control gains `Gp`, `Gv` |

### Gait event detection
| File | Role |
|------|------|
| `..._Subsystem2_Trigger_Generator_Detect_Foot_Down.m` | Fz threshold to flag foot loaded |
| `..._Subsystem2_Trigger_Generator__betterTrig?.m` | Debounces raw foot-contact signal, timestamps rise/fall |
| `..._Subsystem2_Trigger_Generator_Subsystem*_Pert_Safety.m` | Minimum-interval guard on accepted heel strikes |
| `..._Subsystem2_Trigger_Generator_Subsystem*_Detect_Foot_Down1.m` | Per-belt copies of the foot-down detector |
| `..._Subsystem2_Trigger_Generator_Subsystem*__betterTrig?1.m` | Per-belt copies of the debounce filter |

### Force-plate conditioning
| File | Role |
|------|------|
| `..._Subsystem2_Subsystem_FP_Calibration_Calibration_Matrices.m` | Fixed 12×12 calibration scaling |
| `..._Subsystem2_Subsystem_Zero_Cal_Timed5_MATLAB_Function.m` | Rolling-average zero/tare |

### Motion-capture path
| File | Role |
|------|------|
| `..._Subsystem2_MotionCap_values.m` | Butterworth-filtered velocity/acceleration + step velocity from markers |
| `..._Subsystem2_MotionCap_values1.m` | Marker position offset |
| `..._MATLAB_Function1.m` | `marker_fileter`: sorts markers into hip/ankle, holds last value on dropout |
| `marker_fileter.m` | Extended marker filter including knee markers |
| `..._MATLAB_Function.m` | `TLA_convert`: thigh/leg angle from hip and ankle positions |

### Hardware command output
| File | Role |
|------|------|
| `..._Subsystem2_Send_Treadmill_Command_Build_Packet.m` | Builds the treadmill command packet |
| `..._Subsystem2_Send_Treadmill_Command_Build_Packet1.m` | Packet builder with perturbation fast-decel branch |
| `..._Subsystem2_Shutdown_Set_Speed1.m` | Zero-speed stop packet |
| `..._Subsystem2_MATLAB_Function.m` | Time-based belt speed ramp (fixed-speed protocol mode) |
| `eml_lib_MATLAB_Function.m` | Pass-through utility block |

## Key parameters

These are set in the model / block masks, not hard-coded, unless noted.

- **Kalman filter** (`Kalman_filter1`): process noise `sigma_a = 1.3647`; measurement noise `sigma_p = 0.0356`, `sigma_v = 0.1146`; CoP lever arm `h = 0.0095 m`. The deck-geometry offset `1.73 + 0.1695 + 0.1446 - 0.1110` in the heel-strike position is specific to this treadmill and must be re-measured for another rig.
- **Motion-capture filter** (`MotionCap_values`): sample rate 1000 Hz, 4th-order Butterworth low-pass at 4 Hz. Marker offset `+0.1589 m`.
- **Speed control**: target position `p_off`, belt-speed range `r_v_tm_t`, and target settling time `del_t_tgt` are model inputs. Minimum acceleration `a_min = 0.001`.
- **Packet builder**: speed and acceleration clamped to ±32.767; incline capped at 45°; speed in mm/s, acceleration in mm/s², incline in 0.01° units.
- **Startup**: the Kalman block calls `pause(30)` on first execution — a 30 s settling delay before estimation begins.

## Requirements

- MATLAB and Simulink _(fill in the version you built on, e.g. R2023b)_
- Signal Processing Toolbox (for `butter`/`filter` in the motion-capture path)
- Simulink Coder / real-time target _(fill in what you deployed to — e.g. Speedgoat, Simulink Real-Time)_

## Usage

> **Note:** these are the extracted MATLAB Function block bodies, not standalone runnable scripts. They document and version the model's logic. To run the controller you need the parent Simulink model (`.slx`), which is _(add a line here: not included / available on request / in the `model/` folder)_.

Each `.m` file corresponds to one MATLAB Function block. Block I/O signatures are the `function [...] = fcn(...)` line at the top of each file. To rebuild or inspect the model, open the `.slx` and the block bodies match these files.

## Safety notes

This code drives a physical treadmill that a person stands on. A few things worth flagging for anyone adapting it:

- The deck-geometry constants and force-plate calibration are rig-specific. Wrong values put the position estimate off and the belt will chase the wrong target.
- `Pert_Safety` and the debounce filter exist to prevent runaway commands from spurious step detection. Don't remove them.
- Always keep a hardware emergency stop within reach independent of this software.

## Background

Self-pacing (or "self-paced") treadmill control is used in gait research and rehabilitation to let subjects walk at a natural, self-selected speed while staying in a fixed lab position for motion capture and force measurement. This implementation was developed as _(one line on the context — course project / research lab / thesis work — your call)_.


## Contact

Pavan_Karuchola
GitHub: [PaONEkaruchola](https://github.com/PaONEkaruchola)
