# VIO for the ArduPlane - GPS loss switch-over

Goal: when GPS is lost, the plane keeps flying and navigating on visual-inertial odometry (VIO),
then returns to GPS when it comes back.

## What is here

| Path | What it is |
|---|---|
| `scripts/vio_gps_switch.lua` | The switch-over script (runs on the flight controller): GPS = EKF source set 1, VIO = set 2. Rejects a VIO that goes wrong and keeps the bank angle low without GPS |
| `scripts/archive/vio_gps_switch_v1.lua` | The first version (no VIO checks, no bank limit), kept for comparison tests |
| `params/vio_plane.parm` | The matching ArduPlane parameters |
| `tools/sitl.py` | Starts Mission Planner's `ArduPlane.exe` as instance 1 (TCP 5770) in `sitl/` - never touches Mission Planner's own `sitl\plane` folder |
| `tools/sitl_scripts/sim_vio_fault.lua` | Simulator only: makes the simulated VIO drift or run away (`VFT_*` parameters) |
| `tools/check_setup.py` | Boot check: script loads, VisOdom healthy, EKF on GPS |
| `tools/test_gps_loss.py` | Flies an AUTO mission, switches the simulated GPS off and on, saves the flight log |
| `tools/run_robustness_tests.ps1` | The four VIO-failure test runs (about 15 minutes) |
| `tools/run_rtl_tests.ps1` | The four return-home (RTL) test runs (about 15 minutes) |
| `tools/analyze_log.py` | True vs estimated position from the flight log (time-matched), bank angle, VIO checks |
| `tools/plot_run.py` | Plot of a run: track by navigation mode, position error, VIO checks, bank angle |
| `tools/compare_runs.py` | Several runs in one chart: position error against time since GPS loss |
| `results/<run>/` | Flight log, data, summary and plot of each test run |
| `venv/` | This project's own Python (pymavlink, matplotlib, numpy) |

## Run a test (PowerShell, from `C:\Users\funfo\vio\tools`)

```
..\venv\Scripts\python.exe test_gps_loss.py --name my_run
..\venv\Scripts\python.exe analyze_log.py my_run
..\venv\Scripts\python.exe plot_run.py my_run
```

Options: `--no-vio` (fly without VIO for comparison), `--gps-off-s 300`, `--param NAME=VALUE`
(e.g. simulator VIO noise via the `SIM_VICON_*` parameters), `--script PATH` (fly another version of the
switch-over script), and events counted from the GPS loss: `--wind-shift 60,9,310`, `--vio-drift 0.4,45`,
`--vio-dropout 100,8`, `--vio-runaway 40,3,120,30` (details: `test_gps_loss.py --help`).

## Results so far

- `vio_switch` (2026-09-28): GPS off for 3 min in AUTO, no wind. Switched to VIO 1.5 s after GPS loss,
  kept flying the mission at 100 m, switched back 10.5 s after GPS returned. Position error: 0.48 m on
  GPS, 0.17 m on the simulator's (perfect) VIO.
- Comparison in wind (2026-09-29): 6 m/s wind shifting to 9 m/s from another direction 1 min into a
  3 min GPS outage (`results/comparison.png`, made with `compare_runs.py`):

  | Run | Mean error | Worst | Notes |
  |---|---|---|---|
  | `wind_no_vio` | 325 m | 951 m | fine (<6 m) until the wind shift, then ~8 m/s drift |
  | `wind_real_vio` | 36 m | 72 m | VIO noise 0.5 m / 0.3 m/s, drift 0.4 m/s, 8 s dropout; wind shift has no effect |
  | `wind_perfect_vio` | 0.2 m | 0.5 m | |

  With VIO in use, the drifted position is kept for `VSW_GOOD_S` (10 s) after GPS returns.
- Mission Planner (2026-09-29): the same test in your own Mission Planner simulator worked - switched to
  VIO when `SIM_GPS1_ENABLE` was set to 0, kept flying the mission, switched back when it was set to 1.
  Your settings from before are in `backups/mp_before_vio_switch.param` (load it in Full Parameter List
  to go back); the mission is `missions/mp_rectangle_loop.waypoints`.

### When the VIO itself goes wrong (2026-09-29)

The first script trusted the VIO blindly - and real VIO can diverge, as OpenVINS did on the fixed-wing
recording (Phase B below). The script now watches the VIO while it is in use:

| Check | What it compares | Rejects when | Catches |
|---|---|---|---|
| IMU check | the EKF's velocity innovation test ratio: VIO velocity vs what the IMU predicts (1 = the EKF's own rejection gate) | above `VSW_INNOV` (1.0) for `VSW_REJ_T` (1 s) | fast runaways, within seconds |
| airspeed check | EKF ground velocity vs airspeed along the nose + the wind remembered at the switch | above `VSW_VEL_ERR` (12 m/s) for `VSW_REJ_T` | slow runaways the EKF follows |

A rejected VIO stays rejected until the GPS is back; meanwhile the EKF dead-reckons on airspeed and
compass. While the VIO navigates, the bank limit `ROLL_LIMIT_DEG` is lowered to `VSW_ROLL` (20 deg) so the
camera keeps seeing the ground, and the RTL circle `RTL_RADIUS` is widened to what the plane can fly at
that bank downwind (`AIRSPEED_CRUISE` + wind + 2 m/s). Both are restored when the GPS is back or the VIO
is rejected, and never saved. The script logs `VSW` (state, both checks, wind, bank limit, RTL radius)
10 times a second.

With neither GPS nor usable VIO (VIO rejected, missing, or in a dropout) for `VSW_RTL_S` (10 s), the
plane returns home (RTL) - but only if it is flying itself (AUTO, GUIDED, LOITER, TAKEOFF) and not
landing. It happens once per GPS outage, so if you switch away from RTL the script leaves you alone, and
the plane stays in RTL when the GPS comes back (switch back to AUTO to resume the mission). On dead
reckoning "home" is only as good as the dead reckoning: the plane circles where it *thinks* home is.

Script versions: v1 (`scripts/archive/vio_gps_switch_v1.lua`) had no checks; the runs below marked "new
script" used v2 (checks, bank limit also during dead reckoning, no RTL); the RTL runs further down use the
current version.

Test runs (`tools/run_robustness_tests.ps1`, plus `slow_runaway_old`): 3 min GPS outage, wind 6 m/s with
turbulence, VIO noise 0.5 m / 0.3 m/s and 0.4 m/s drift. `results/runaway_comparison.png` puts the four
runaway runs side by side; each run's `plot.png` shows the track, both checks and the bank angle.

| Run | VIO fault | Result |
|---|---|---|
| `runaway_old` | 40 s into the outage the VIO runs away (+3 m/s every second, up to 30 m/s); old script | never rejected: the plane flew off in a straight line, **4.1 km** off when GPS returned. The bad VIO had also bent the EKF's wind estimate, so after GPS returned ArduPilot disabled the healthy airspeed sensor, switched EKF lanes and ran on its backup attitude estimate (DCM) for 3 s |
| `runaway_new` | the same; new script | IMU check rejected it **3.2 s** after it began (23 m off at that moment), dead reckoning, **147 m** off when GPS returned |
| `slow_runaway_old` | slow runaway (+0.3 m/s every second); old script | never rejected: **2.7 km** off when GPS returned, and the same airspeed-sensor and EKF trouble afterwards |
| `slow_runaway_new` | the same; new script | the IMU check cannot see it (max 0.60); airspeed check rejected it after **43 s** (13 m/s off, 293 m off), but by then the EKF's wind estimate was ~5 m/s wrong, so dead reckoning drifted to **829 m** at GPS return |
| `wind_real_vio_new` | normal VIO, wind shift, 8 s dropout (as `wind_real_vio`) | **no false rejection**: IMU check max 0.61 (limit 1), airspeed check max 8.8 m/s (limit 12); same accuracy as before (36 m mean, 72 m worst) |

In every run with the new script the bank angle stayed within 21.7 deg while without GPS (60+ deg before)
and the limit was back at 65 deg afterwards.

Lessons:
- The EKF's own velocity innovation test ratio is the quickest sign of a VIO gone wrong. In normal runs it
  never stayed above 1 for more than one sample; a runaway pushes it over 1 within 1-2 s. After about
  10 s of rejecting the VIO, the EKF resets itself onto it - the script has to act before that.
- Once the EKF has swallowed bad VIO, its wind estimate is wrong too, and dead reckoning afterwards
  drifts by that error. Reject early. A slow runaway looks like a wind change to every sensor on the
  plane, so it is caught late; the airspeed check's margin to a real 8 m/s wind shift is only ~3 m/s.
- The IMU check's sensitivity depends on `VISO_POS_M_NSE` / `VISO_VEL_M_NSE` matching the real VIO
  (with 0.5 / 0.3 the fast runaway peaked at 1.6; with the default values it reached 2.9 within 4 s).
  Tune on the real plane from flights with the VIO logged but not used.
- With 20 deg bank the mission corners are flown wide (turn radius ~135 m at 22 m/s).

### Return home when GPS and VIO are both gone (2026-09-29)

Your choice: RTL. Runs with the current script (`tools/run_rtl_tests.ps1`, same conditions as above):

| Run | What happens | Result |
|---|---|---|
| `rtl_runaway` | VIO runs away 40 s into the outage | rejected 2.4 s after it began, dead reckoning, **RTL 10 s later**. Circles its home on the normal 80 m circle; really 11-174 m from home (dead-reckoning error), 89 m off when GPS returned |
| `rtl_no_vio` | no VIO at all, wind shift after 60 s | **RTL 11.5 s after GPS loss**, circles its believed home neatly (68-104 m) - but dead reckoning cannot know about the wind shift and drifts ~8 m/s: really up to 989 m away, 921 m off when GPS returned |
| `rtl_normal_vio` | normal VIO, wind shift, 8 s dropout | **no RTL**, no rejection (IMU check max 0.59, airspeed check max 9.0 m/s) |
| `rtl_long_dropout` | VIO drops out for 20 s | **RTL 10 s into the dropout**; the VIO came back and the plane flew home on it, circling on the widened 251 m circle (really 215-321 m from home; ArduPilot's own estimate agrees) |

Lessons:
- RTL on dead reckoning is only as good as the wind estimate: the plane circles where it *thinks* home is,
  and after a wind change that drifts downwind. Watch it and take over if it drifts.
- At 20 deg bank a circle has to be wide: 251 m at 22 m/s cruise in 6 m/s wind. With the normal 80 m
  circle the plane wandered up to 400 m from home, so the script widens `RTL_RADIUS` while the limit is
  on. For the Albabird at ~16 m/s it would be about (16 + 6 + 2)^2 / (9.81 x tan 20) = 160 m.
- The bank limit now applies only while the VIO navigates: in dead reckoning there is no camera to
  protect, and RTL then circles home at normal bank on the normal circle.

## Phase B - real VIO (Ubuntu 22.04 in WSL, ROS 2 Humble, OpenVINS in ~/ws_ov)

| Script | What it does |
|---|---|
| `setup/install_ros2_humble.sh` | ROS 2 Humble + OpenVINS dependencies (needs your Linux password) |
| `setup/build_openvins.sh` | Builds OpenVINS in ~/ws_ov (2 compile jobs, fits the 6 GB WSL limit in `C:\Users\funfo\.wslconfig`) |
| `setup/run_openvins_bag.sh BAG [CONFIG] [OUT]` | Runs OpenVINS on a ROS 2 recording, saves `ov_estimate.txt` |
| `tools/remote_zip.py` | Downloads single files out of big remote .zip archives (resumes, waits out rate limits) |
| `tools/bag_groundtruth.py` | Extracts the true path (e.g. Vicon) from a recording |
| `tools/eval_vio.py EST GT NAME` | Scores a run (ATE and drift after a start line-up) and plots it |

- EuRoC V1_01_easy (2026-09-29): ATE 0.155 m, drift 0.18 m after 139 s = 0.30 % of the 58.5 m flown
  (`results/euroc_v1_01_ext4/`).
- Lessons: play recordings from Ubuntu's own disk (`~/datasets`), not through `/mnt/c` - the slow
  Windows file bridge made OpenVINS silently lose IMU messages and diverge (`results/euroc_v1_01/`).
  OpenVINS listens to the IMU with ROS 2 SensorDataQoS (best effort, only 5 queued) - worth deepening on
  the Jetson. In ROS 2 its `save_total_state` needs `filepath_*` node parameters (the launch file does not
  pass them), and it sometimes ignores a stop request - `run_openvins_bag.sh` handles both.
  Its settings reader (OpenCV YAML) crashes with a segmentation fault if a comment contains a colon -
  `setup/check_config.sh CONFIG_DIR` tests a config folder in seconds.

### Fixed-wing test: InGVIO `fw_gvi_easy` (2026-09-29)

12 kg plane, ZED 2i stereo camera looking down, RTK GPS as truth; 175 s, 2.5 km, takeoff, 4 turns
up to ~30 deg bank at 45-70 m, landing. Data in Ubuntu `~/datasets/ingvio/fw_gvi_easy_ros2`
(CC BY-NC-SA 4.0). OpenVINS configs in `openvins_config/` (built from InGVIO's calibration).

| Run | Change | Result |
|---|---|---|
| `fw_gvi_easy` | InGVIO's IMU noise values | lost track after ~60 s |
| `_A` | camera calibration kept fixed | lost track |
| `_B` | A + start in the air (45 s) | lost track |
| `_C` | A + realistic IMU noise (`fw_zed2i_imu`) | **best**: right through takeoff and climb (lift-off height 15.6 vs 15.9 m), ~30 % too fast in the climb, lost track in the 2nd turn over a lake (64-77 s) |
| `_D` | C without SLAM landmarks | same as C |
| `_E` | C + start in the air | in-air start failed (speed 1.6 instead of 21 m/s) |

- The dataset authors report that GVINS (VINS-based) could not even initialize on these flights, and their
  own invariant-filter VIO drifts ~30 m RMS over the 2.5 km (about 1.2 %; 52-55 m on the harder flights).
- Lessons: IMU noise must be realistic (measure it standing still: `tools/check_imu.py`); banked turns
  over featureless ground or water are where VIO breaks; OpenVINS's in-flight start is unreliable - start
  VIO on the ground and keep it running.
- The GPS truth in these bags is ~970047 s (11 days) off the camera clock; `eval_vio.py --time-offset
  auto:970047.2` fine-tunes the offset.

## Gazebo - camera-based simulation (started 2026-09-29)

Goal: a real VIO (OpenVINS) working on rendered camera images while ArduPilot flies the plane, with the
same MAVLink bridge the Jetson will run. More realistic for the VIO and the Jetson software; still not
for image quality (no motion blur, vibration or exposure changes) or flight physics.

| Script (run inside Ubuntu) | What it does |
|---|---|
| `setup/install_gazebo_harmonic.sh` | Gazebo Harmonic, its ROS 2 Humble bridge, build dependencies (needs your password) |
| `setup/get_ardupilot_sources.sh` | ArduPilot (latest stable ArduPlane, 4.7.1) and ArduPilot's Gazebo plugin into `~` |
| `setup/build_ardupilot_gazebo.sh` | Builds the ArduPlane simulator and the Gazebo plugin (Python build tools in `~/venv-ardupilot`) |
| `setup/run_gazebo_zephyr.sh` | Opens Gazebo with the Zephyr flying wing and starts the simulator (MAVLink TCP 5760 / 5762) |
| `tools/make_scenery.py` | Builds the VIO test world into `~/vio_gazebo` (run with Ubuntu's `python3`); map preview in `results/scenery_map.jpg`; `--camera d455-800` (default: the D455's colour camera, whole 1280 x 800 sensor at 30/s, 91 deg wide), `d455` (its 848 x 480 mode) or `sim640` (the earlier 640 x 480 at 20/s) |
| `setup/run_gazebo_vio.sh` | Starts the VIO world: simulation server (physics + camera), the Gazebo window, the simulator (MAVLink TCP 5760 / 5762) |
| `setup/gazebo_window.sh` | Reopens the Gazebo window on a running simulation (closing it never stops the simulation) |
| `setup/stop_gazebo_vio.sh` | Stops the whole Gazebo simulation |
| `tools/gz_web_view.py` | Live view in the Windows browser, http://localhost:8080: chase camera (25 frames/s) and VIO camera |
| `tools/gz_camera_grab.py` | Saves frames from the plane's camera (Ubuntu's `python3`) into `results/gazebo_camera/` |
| `setup/record_gazebo_flight.sh NAME [S]` | Records the VIO camera, VIO IMU and true pose (through `ros_gz_bridge`) into a compressed ROS 2 bag `~/datasets/gazebo/NAME` |
| `tools/bag_truth_ros2.py BAG OUT` | Writes the true path from such a recording (Ubuntu, ROS 2 loaded) for `eval_vio.py` |
| `openvins_config/gz_zephyr/` | OpenVINS for the simulated camera: monocular, 640 x 480 pinhole (f = 320 px), VIO IMU in the camera module |
| `setup/openvins_variants_gz.sh` | Tuning experiments: variants of `gz_zephyr` run on the same recording |
| `tools/camera_flow.py` | Picture of how fast the ground slides across the VIO camera's image when it looks down, ahead or tilted (`--height`, `--speed`, `--tilt`) -> `results/camera_flow.png` |
| `tools/ros_rgb_to_mono.py` | Turns the simulator's colour pictures grey like a real mono camera (Gazebo's own grey format is far too dark); `record_gazebo_flight.sh` runs it |
| `setup/openvins_init_from_state.py` | Adds "start from a given state" to OpenVINS's ROS 2 node - from the flight controller live (`/ov_msckf/init_state`) or from a file (rebuild `ov_msckf` after it; `setup/openvins_init_from_state.patch` is the change) |
| `tools/bag_gt_init.py BAG OUT` | The true state of the VIO IMU from a recording, as OpenVINS's start state; checks the frame maths against the recorded IMU and picks the start (level flight) |
| `setup/openvins_cameras_gz.sh [BAG] [CAMS]` | Runs OpenVINS on each camera of one recording side by side, all started from the same state (`openvins_config/gz_down`, `gz_t45`, `gz_t30`) |
| `tools/compare_cameras.py STATE RUN ...` | Map and position error against distance flown for runs started from the true state (no lining up afterwards) |
| `tools/vio_bridge.py` | **The companion program (what the Jetson will run)**: starts OpenVINS from the flight controller's state while flying steadily (straight or a steady turn) with GPS, sends the VIO to ArduPilot (VISION_POSITION/SPEED_ESTIMATE, 20 Hz), keeps it fresh (restarts it while on GPS when its speed is off by > 0.7 m/s, or every 5 minutes), holds it back when the camera stalls and restarts it after (from ArduPilot's estimate if the GPS is gone - also after an OpenVINS crash), and tells the ground station ("VIO: ..."); `--mount gazebo` / `realsense-down`, `--lever`, `--imu-topic`, `--camera-topic` |
| `setup/run_vio_live.sh` | The VIO chain live in the simulation: Gazebo camera + VIO IMU -> grey pictures -> OpenVINS (restarted whenever it stops) <-> `vio_bridge.py` <-> ArduPilot (TCP 5763); start after `run_gazebo_vio.sh`; `VIO_CONFIG=gz_d455_800` (default), `gz_d455` or `gz_down` - to match the world's camera |
| `setup/run_vio_d455.sh` | **The same chain on the plane's Jetson** (not yet tested on hardware): RealSense D455 colour 1280 x 800 at 30/s (`RES=848x480` for the smaller mode) + its IMU at 400/s -> OpenVINS (`openvins_config/d455_800`, every 2nd picture) <-> `vio_bridge.py --mount realsense-down` <-> flight controller on a serial port; `MAVLINK`, `LEVER`, `RECORD=1` (records camera + IMU to replay flights later), `CAMERA_ONLY=1` - see "D455 bring-up" |
| `setup/jetson_install.sh` | One-shot Jetson setup (not yet tried on one): ROS 2 Humble, RealSense driver, OpenVINS at the desktop's exact version with the change, pymavlink, serial access, the start-on-boot service (off) |
| `setup/vio.service`, `setup/vio.env.example` | Start the VIO chain at boot (`sudo systemctl enable vio`), settings (port, camera position, resolution, recording) in `setup/vio.env` |
| `tools/preflight_check.py` | Checks the flight controller's VIO settings and the free space before the companion starts (run by `run_vio_d455.sh`); "VIO check: ..." to the ground station |
| `openvins_config/gz_d455/`, `openvins_config/d455/`, `openvins_config/d455_800/` | OpenVINS for the D455-like simulated camera at 848 x 480, and for the real D455 at 848 x 480 and 1280 x 800 (OpenVINS's own D455 numbers - scaled for 1280 x 800 - until your camera is calibrated; one camera, the 100 m settings) |
| `openvins_config/gz_d455_800/`, `openvins_config/variants/` | OpenVINS for the simulated D455 at its full 1280 x 800 (`make_scenery.py --camera d455-800`), and the variants of the side-by-side test (1280 x 800 or 848 x 480; 10, 15 or 30 pictures a second; 20 or 30 clones) |
| `setup/run_vio_variants.sh LOG_DIR [NAME ...]` | Side-by-side test: next to the VIO ArduPilot uses, one more OpenVINS per variant on the same live camera, all started from the same states at the same moments; logs their paths and the simulator's truth (`tools/ros_truth_log.py`) |
| `tools/compare_variants.py LOG_DIR` | Scores those variants against the truth: drift since each VIO (re)start after 30 s to 5 min, speed error -> table and `results/variants.png` |
| `tools/plot_vio_runs.py RUN=LABEL ...` | For runs of `gz_gps_loss.py`: VIO speed error while on GPS (with the companion's restarts) and ArduPilot's position error after the GPS loss |
| `tools/gz_gps_loss.py` | Closed-loop GPS-loss test in Gazebo: take-off, rectangle, GPS off, prints where ArduPilot thinks the plane is and where it really is; `--wind`, `--wind-shift`, `--far`, `--no-vio`, `--camera-stall S,DUR` (camera pictures stop, IMU goes on), `--kill-vio S` (OpenVINS crash) |
| `setup/gz_set_wind.sh SPEED FROM_DEG` | Sets / changes the wind in the running Gazebo world |
| `setup/ardupilot_gazebo_airspeed.py` | Makes ArduPilot's Gazebo plugin send the airspeed through the wind (otherwise the simulated airspeed sensor reads ground speed); rebuild the plugin after it |
| `tools/plot_rtl_compare.py RUN=LABEL ...` | Map of the true path and distance from home while the GPS was off, for several runs |
| `tools/gz_takeoff_loiter.py` | Starts a flight for you to take over: take-off, a straight leg home at 100 m (the VIO starts on it), LOITER over home, then lets go of the simulator |
| `tools/sim_watch.py` | Watches the Linux VM's free memory during a session, warns in Mission Planner at 10, 5 and 2 minutes left and stops the simulation cleanly before WSL runs out |

**Fly it yourself (Gazebo, with the VIO):** start `setup/run_gazebo_vio.sh` and `setup/run_vio_live.sh` in
Ubuntu, then `python tools/gz_takeoff_loiter.py` from Windows (and `tools/sim_watch.py` in Ubuntu after
it). Connect Mission Planner with **TCP, host 127.0.0.1, port 5762** (or MAVProxy:
`mavproxy.py --master=tcp:127.0.0.1:5762`; only one program per port); watch the cameras on
http://localhost:8080. GPS off / on: parameter `SIM_GPS1_ENABLE` 0 / 1. Wind, from PowerShell or cmd:
`C:\Users\funfo\vio\tools\wind.cmd 6 270` (6 m/s from the west; `0 0` = calm) - ArduPilot's own wind
estimate (what Mission Planner shows) takes ~30-60 s to catch up.
Modes that need no sticks: AUTO (the test rectangle), LOITER, GUIDED ("Fly to here"), RTL, CIRCLE, CRUISE
needs a stick - without a joystick FBWA/MANUAL/CRUISE get zero throttle. A session lasts about 1-2 hours
without the chase camera and with the 640 x 480 camera, about 45 minutes with the D455-like one (30 bigger
pictures a second: ~80 MB/min; `sim_watch.py` warns in Mission Planner before it has to stop); stop earlier with
`wsl -d Ubuntu-22.04 -- bash /mnt/c/Users/funfo/vio/setup/stop_gazebo_vio.sh`.

From Windows: `tools/gz_fly_zephyr.py` takes off (vertically, the Zephyr way) and flies the test
rectangle. `test_gps_loss.py --sitl wsl` runs the GPS-loss tests on the Ubuntu-built 4.7.1 simulator
instead of Mission Planner's.

- Step 1 (2026-09-29): Gazebo Harmonic 8.15 + ArduPlane 4.7.1 SITL in Ubuntu - the Zephyr took off and
  flew the rectangle at 100 m. Rendering runs on the RTX 4060 through WSLg (`MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`,
  otherwise the Intel graphics are used). The stock world is flat green with a runway - nothing for a
  camera to track - and runs ~2.3x faster than real time; the VIO world will need real time.
- Found on the way: Mission Planner's simulator is a 4.8.0 *development* build. `ahrs:get_wind()` does not
  exist in 4.7, so the switch-over script would have crashed on a 4.7 plane at the moment it switched to
  VIO; it now falls back to `ahrs:wind_estimate()`. Checked on 4.7.1 (`v471_rtl_runaway`): switch to VIO,
  runaway rejected 3.0 s after it began, RTL 10 s later, back on GPS 0.4 s after it returned.
- WSL quirks for the Ubuntu simulator (handled by `tools/sitl.py` and `setup/run_sitl_wsl.sh`): it must run
  in a folder on Ubuntu's own disk (`~/sitl_vio`; on `/mnt/c` it dies at start), and it must not be the
  process wsl.exe starts directly (then it dies as soon as a client connects).
- Step 2-1 (2026-09-29): the VIO world. `make_scenery.py` paints a 2 km x 2 km landscape around home into
  64 texture tiles at 25 cm per pixel - fields with crop rows and tramlines, ploughed and stubble fields,
  meadows, orchards, forest, hedges, two villages, roads and dirt tracks, a mown strip with a big N and E
  at home, and a lake under the north-east corner of the test rectangle - plus 938 3D trees and 91 houses
  for depth; seed 7, so it is the same every time. The Zephyr carries a downward camera (640 x 480,
  90 deg wide, 20 frames/s, top of the image = nose). Result: images that look like drone footage at
  ~19.5 frames/s, the simulation in real time (1.00x). Big meadows and plain crop fields have little
  texture and the lake almost none - realistic, and a fair test for the VIO.
- Lessons (WSL graphics): a Gazebo *window* drawn through WSL's GPU layer (d3d12) leaks memory - about
  600 MB a minute on the RTX, 120 on the Intel graphics - until Ubuntu runs out and Gazebo crashes inside
  the graphics driver (it did, after 5 minutes). The camera, rendered off-screen on the RTX, does not leak.
  So the simulation server and the window are now separate programs; the window draws on the Intel
  graphics, and a watchdog reopens it before its memory gets large (software drawing does not leak but
  was too slow to even appear). Linux windows did not show on the Windows screen at all until WSL was
  restarted (`wsl --shutdown`). The window's follow-camera must follow rigidly (`follow_pgain` 1.0 on
  `/gui/track`): the default lazy follow makes the plane jump around by 20-60 px (measured on
  screenshots); rigidly it is 4-9 px, ~22 new pictures per second. Smoothest of all is the browser view
  (`gz_web_view.py`): its chase camera is rendered inside the simulation, in step with the physics.
  Texture tiles need a border of their neighbours
  (2 m), or the graphics card blends opposite edges into thin lines along the tile seams. The simulator
  serves one client per TCP port and switches to the newest - a forgotten script still reconnecting
  in the background knocks the real one off.
- Step 2-2 (2026-09-29): OpenVINS on the simulated camera. The plane got the VIO's own IMU in the camera
  module (400 Hz, noise 0.004 rad/s and 0.04 m/s^2 per sample plus a random constant bias per axis - like
  a camera-module IMU; ArduPilot keeps its perfect 1 kHz IMU) and a grayscale camera; the true pose is
  published too. One flight recorded (`gz_flight1`: 7 min, 926 MB - take-off, the north leg along the
  lake, the turn over the water, the east leg). OpenVINS started by itself at the take-off jolt and
  tracked the whole flight without losing it, but: ~10-15 m off during take-off, climb and the first
  turns, then on the long straight legs the error grew quadratically to **727 m after 3.3 km (22 %)**.
  That shape is an accelerometer bias of ~0.03 m/s^2 that the filter cannot tell apart from a speed
  change - on a straight leg at constant speed one camera sees the direction of travel but not the
  scale. The simulated IMU's biases are 0.02 m/s^2, so this is the weak spot real planes have too.
  Tuning on the same recording (`setup/openvins_variants_gz.sh`):

  | Variant | ATE (lined up) | Drift at the end (3.3 km) |
  |---|---|---|
  | `gz_zephyr` (as above) | 263 m (8.1 %) | 727 m (22 %) |
  | biases free to wander (random walk x10) | diverged (km) | - |
  | **longer window (20 clones, 75 SLAM points)** | **160 m (4.9 %)** | **564 m (17 %)** - under 5 m for the first 100 s |
  | both | diverged (km) | - |

  The VIO also believed the plane was sinking steadily (down to 15 % of the true height): the height
  is the direction a downward camera sees worst (1 m/s of sink at 100 m only enlarges the picture by 1 %
  a second). So `vio_plane.parm` now takes only horizontal position and velocity from the VIO
  (`EK3_SRC2_VELZ 0`); height and vertical speed stay on the barometer.

  **Correction (step 2-3): most of that drift was our own setup, not the physics.** OpenVINS by default
  throws away every point further than 60 m (`fi_max_dist`), and at 100 m height that is the whole
  ground under a downward camera - at cruise height the VIO flew on its IMU alone (its speed uncertainty
  grew from 0.5 to 12 m/s between 146 and 266 s). The quadratic drift and the sinking came from that.
  The pictures were also far too dark (Gazebo's grey format skips the gamma step - ground mean ~15 of
  255). Both are fixed in step 2-3; keeping `EK3_SRC2_VELZ 0` (baro height) is still right.
- Gotchas: both cameras published their camera info on `/vio/camera_info` (Gazebo derives the name from
  the image topic) - the chase camera now has its own; while recording, the simulation stalls for
  moments (the server waits for rendering), which does not matter for the data (all in simulation time).
- Step 2-3 (2026-09-29): which way should the VIO camera look? (The goal restated: VIO is only the
  GPS-loss backup that gets the plane home and keeps it circling there; speed through the air comes
  from the airspeed sensor, height from the barometer - what the camera must add is the speed *over
  the ground*, i.e. the wind.) First a picture of the geometry (`tools/camera_flow.py`,
  `results/camera_flow.png`): straight ahead, half the picture is sky and the ground slides ~7x slower
  than under a downward camera. Then a test: the plane got two more cameras in the same module (same
  IMU), tilted 45 and 30 deg below the horizon (`make_scenery.py --tilts 30,45`), and one flight was
  recorded with all three (`gz_flight2`: 4.5 km, 454 s of VIO; ~14 pictures/s - Gazebo skipped
  renders under the load, the same frames for all cameras). OpenVINS ran on each camera with the same
  settings (`openvins_config/gz_down`, `gz_t45`, `gz_t30`: the 20-clone variant plus the distance limits
  raised to 3 km), all started from the same true state in level flight - on the real plane that is the
  flight controller's state while GPS still works (OpenVINS change: `setup/openvins_init_from_state.py`;
  runner: `setup/openvins_cameras_gz.sh`). No lining up afterwards (`tools/compare_cameras.py`):

  | Camera | Error at the end (4.5 km) | Largest error | Ground speed error (median / largest) |
  |---|---|---|---|
  | **straight down** | **24 m (0.5 %)** | **28 m** | **0.15 / 0.49 m/s** |
  | tilted 45 deg down | 51 m | 58 m | 0.33 / 0.81 m/s |
  | tilted 30 deg down | 6 m | 40 m | 0.38 / 0.94 m/s |
  | straight down, old settings (60 m limit) | 9.1 km | - | diverged |

  All three are good enough to find home; tilting did not help in the simulator - the downward camera
  was the steadiest and measured the ground speed about twice as well. What the simulator does not
  show: water and featureless fields (a downward camera sees only a 200 x 150 m patch; a tilted one
  sees kilometres ahead), motion blur and vibration - the logged-only flights with the real camera
  decide. Plots: `results/gz_flight2_cameras.png`, `results/gz_flight2_limit.png`.
- Gotchas (step 2-3): the Gazebo server leaks ~500 MB of memory per minute under WSL with three or four
  cameras on the RTX (the Intel graphics leak less but render ~1000x too slowly) - after ~10 minutes the
  Linux VM runs out and stops, so recordings are made from a freshly started server, with a guard that
  ends the recording cleanly (the one VM stop lost only the last minute; `ros2 bag reindex` then needs
  `compression_format: zstd` / `compression_mode: MESSAGE` put back into `metadata.yaml`); config files
  written by Windows Python get CRLF line ends, which broke the topic filter (the runner strips them now).
- Step 3 (2026-09-29): **the real VIO in the loop - GPS lost, the plane flies home on OpenVINS.** The
  companion program `tools/vio_bridge.py` (what the Jetson will run) waits for straight, level flight above
  40 m with GPS, hands the flight controller's own state to OpenVINS as its start (so the VIO works in
  ArduPilot's frame from the first moment - no take-off jolt needed), then sends the VIO to ArduPilot. The
  switch-over script got `VSW_GPS_RTL` (default 1): losing GPS with a healthy VIO now also means return
  home (only if the plane is flying itself, never while landing). The Gazebo world got wind (changeable
  in flight, `setup/gz_set_wind.sh`), and the Gazebo plugin now sends the airspeed through that wind -
  ArduPilot's simulator otherwise took a JSON model's wind as zero, so its airspeed sensor read ground
  speed. Test: `tools/gz_gps_loss.py` - rectangle at 100 m, GPS off 40 s after the VIO started, 3 minutes
  without GPS, 60 s into it the wind turns from 6 m/s from the west to 9 m/s from the north-west:

  | Run (3 minutes without GPS) | ArduPilot's position off by | Really this far from home while circling |
  |---|---|---|
  | **with the VIO, wind + wind change** (`gz_vio_rtl_wind`) | **mean 4 m, max 7 m** | **104-143 m** (its RTL circle was 102 m) |
  | without the VIO, same wind (`gz_dr_rtl_wind`) | mean 278 m, max 776 m | drifted away to **819 m** - ArduPilot thought 54-77 m |
  | with the VIO, no wind, before the two fixes below (`gz_vio_rtl_nowind`) | mean 17 m, max 20 m | 46-83 m (circle 60 m) |

  RTL started 1.5 s after the GPS loss; back on GPS 10.5 s after it returned; the VIO checks stayed quiet
  (IMU check max 0.10 of 1, airspeed check max 7.4 of 12 m/s - it grows with a wind change, because it
  compares with the wind remembered at the switch; a change much bigger than 7 m/s could trip it). Plot:
  `results/gz_rtl_wind_compare.png`. Fixes found on the way: (1) `VISO_TYPE 2` turns the whole VIO frame to
  match ArduPilot's heading at the first message - with a 1 deg mismatch, about a point 575 m away, that
  cost 8 m near home; the companion already starts the VIO in ArduPilot's frame, so `vio_plane.parm` now has
  `VISO_TYPE 1` (both types keep the VIO lined up with ArduPilot's position while on GPS, so switching
  never jumps); (2) the companion starts the VIO only in straight flight (bank < 5 deg, turn < 2 deg/s) -
  in a turn a little delay between attitude and IMU time already means a degree of heading (since the D455
  preparation it matches the two clocks and starts in steady turns too - see "D455 bring-up").
  **For the real plane:** `VISO_TYPE 1`, `ARMING_SKIPCHK 262144` (skip only the visual odometry arming
  check - the VIO starts in the air), the new script (`VSW_GPS_RTL`), and the Jetson on `SERIAL5`.
- Step 3, harder run (`gz_vio_rtl_lake`, `--far 690,860`): GPS lost 960 m out beyond the lake, home
  straight across the water, 40 s in the wind swung from 6 m/s from the north-west to 9 m/s from the
  north-north-east (a change of 8.5 m/s), 200 s without GPS: ArduPilot's position off by **mean 9 m, max 13 m**,
  the plane circled home 93-150 m out (circle 110 m); the VIO crossed the lake without trouble. Two warnings:
  (1) the script's airspeed check reached **10.8 of 12 m/s** - it compares with the wind remembered at the
  switch, so a large wind change nearly rejected a good VIO (it would then have flown on dead reckoning);
  (2) after the sudden wind change ArduPilot flew ~40 s at **7.5 m/s airspeed** (minimum 9, target 12): its
  EKF learns wind slowly (`EK3_WIND_P_NSE` 0.1), so the airspeed it predicted (16 m/s) disagreed with the
  sensor and it stopped using the sensor (`AHRS_OPTIONS` 0) - TECS then held the wrong speed. Not a VIO
  problem (the same happens with GPS), but worth tuning before real flights: a faster wind estimate
  (`EK3_WIND_P_NSE` higher) and an airspeed check that allows for real wind changes.
- World enlarged (2026-09-29, for flying it yourself): 5.1 km x 5.1 km - the central 2 km as before at
  25 cm per pixel, a ring of 84 tiles at 50 cm per pixel around it (all at 25 cm would need ~2 GB of
  graphics memory), 5 lakes, 7 villages with roads, 4 big forests, 30 dirt tracks, 6164 3D trees and 324
  houses; plain grass beyond, out to 8 km. **The session limit is solved:** the Gazebo server's WSL leak is
  per piece drawn in each camera picture (not the sky, not the shadows - tested). The ground used to be one
  mesh with all its tiles, so every picture drew every tile (with 149 tiles the Linux VM ran out after ~4
  minutes); now each tile and each 512 m block of trees and houses is its own visual, and Gazebo draws only
  what a camera sees: **~35-60 MB/min instead of 540-800, about 1-2 hours per session**. The chase camera
  (10 pictures a second, looking kilometres ahead) still draws many pieces - it is left out for long
  sessions (`make_scenery.py --no-chase`, `GZ_VIEW=vio`). `--keep-tiles` rebuilds without repainting.
- Gotcha (2026-09-29, C: drive full): every time the Gazebo server was stopped the polite way it crashed on
  its way out, and WSL wrote a ~230 MB crash dump into `%LOCALAPPDATA%\Temp\wsl-crashes` (it keeps up to 10).
  With the day's logs this filled C: during a flight: Ubuntu's filesystem switched to read-only, the
  simulation died, and only `wsl --shutdown` brought Ubuntu back (the filesystem recovered cleanly). Now
  `stop_gazebo_vio.sh` kills the server at once (no crash, no dump), and `sim_watch.py` also watches C:
  (warns below 1.5 GB, stops the simulation below 0.8 GB).
- Gotchas (step 3): ArduPlane refuses MAVLink message rates above its loop allows (50 Hz requests were
  "denied", 20 Hz works); the Gazebo memory leak limits a session to ~9 minutes, so each test starts a fresh
  simulation (`tools/gz_gps_loss.py` runs about 5 minutes); WSL expands `$variables` in `wsl -- bash -c '...'`
  before the inner shell sees them - use script files.

## D455 bring-up (prepared 2026-09-29)

The camera for the plane: **Intel RealSense D455**, used as one downward camera - its colour sensor has a
global shutter (no rolling-shutter bending in turns and vibration), 91 deg x 65 deg, and the IMU sits in the
same housing with a factory calibration. Order of choice among what is available: D455, then D435i (its left
infrared camera with the projector off - global shutter, 87 deg; the colour camera is rolling shutter), then
D435if (the same, but its infrared cameras see only near-infrared - check the picture outdoors first), then
OAK-D Pro (fine hardware, but a different driver and IMU, and its camera-IMU calibration is up to you). The
first three use the same scripts: the `realsense-down` mount fits any RealSense.

What was changed for it:

- **Simulator camera = the D455's colour camera**: its whole 1280 x 800 sensor at 30 pictures a second,
  91 deg x 65 deg (`make_scenery.py --camera d455-800`, the default; `d455` = its 848 x 480 mode, `sim640` =
  the earlier 640 x 480 at 20/s). The simulated IMU's noise already matched the D455's numbers in OpenVINS.
  OpenVINS settings: `gz_d455_800` / `gz_d455` (simulator), `d455_800` / `d455` (the real camera - OpenVINS's
  own sample D455 until yours is calibrated; one camera, the settings that let OpenVINS use features 100 m
  away, online calibration on). Why 1280 x 800 and every 2nd picture: "Choosing the camera settings" below.
- **The companion (`tools/vio_bridge.py`)**:
  - `--mount realsense-down` (looking straight down, top of the picture towards the nose), `--lever`
    (camera position from the flight controller), `--imu-topic`.
  - **Starts in a steady turn too**, not only in straight flight: the flight controller's samples carry its
    own clock, the offset to the camera's clock is learnt from their arrival, and the attitude is turned on
    at its measured rates to the exact moment of a camera IMU sample. Offline check in an 11 deg/s loiter
    turn: heading off by **0.008 deg** (taking the latest sample as it is: up to 0.57 deg).
  - **Keeps the VIO fresh** - the fix for the drift you saw after 20 minutes of loiter: while the plane surely
    flies on GPS (the switch-over script reports it, `VSW_ST` = 0, and the GPS has been good for 10 s),
    OpenVINS is restarted from the flight controller's state when its speed is more than 0.7 m/s off for 5 s,
    or every 5 minutes. A stopped OpenVINS is noticed (3 s without an estimate) and started again the same
    way. The pilot is told ("VIO: restart, ..."), ArduPilot through the messages' reset counter. Never while
    the VIO may be navigating; a restart takes a few seconds, during which a GPS loss would mean dead
    reckoning.
  - Logs how old each estimate is when it leaves (`age_ms`) - for `VISO_DELAY_MS` (below). OpenVINS carries
    its estimate forward to the latest IMU sample, so this is only the delivery delay: 0-2 ms in the
    simulator; with the serial link the default `VISO_DELAY_MS` 10 is likely right.
- **`setup/run_vio_live.sh`** keeps OpenVINS in a restart loop; **`setup/run_vio_d455.sh`** is the same chain
  for the Jetson with the real camera (checks the camera topics and the picture size before starting).
- `params/vio_plane.parm`: `SERIAL5_BAUD 921` (921600, the Jetson's serial speed).

### Choosing the camera settings - side-by-side test (2026-09-30)

Question: is 1280 x 800 better than 848 x 480 in real flight - and what is the best combination of everything?
How: `setup/run_vio_variants.sh` runs one OpenVINS per setting on the same live 1280 x 800 camera (the
848 x 480 ones get the picture cut and shrunk the way the D455 makes its 848 x 480 mode), all started from the
same state at the same moments and restarted together; `tools/compare_variants.py` scores each start by its
drift from the true path. Flight 1 (`gz_variants`): out to 1 km over the fine-textured middle of the world,
4-minute restarts, then 4 minutes without GPS - one long segment per setting that includes the flight home.
Flight 2 (`gz_variants2`): the rectangle around home for 18 minutes, 2-minute restarts, 8-9 segments per
setting. (Times are simulated time; with seven OpenVINS the simulation ran at ~0.77x real time.)

| OpenVINS setting | Flight 2: drift after 60 s | after 90 s | speed error | Flight 1: long segment after 5 min |
|---|---|---|---|---|
| **1280 x 800, every 2nd picture (15/s)** | **3.7 m** | **4.6 m** | **0.11 m/s** | **10.9 m** |
| 1280 x 800, every 3rd picture (10/s) | 4.3 m | 5.5 m | 0.13 m/s | 24.8 m |
| 1280 x 800, 15/s, 30 clones | 3.9 m | 5.4 m | 0.11 m/s | 11.6 m |
| 1280 x 800, every picture (30/s) | 5.9 m | 7.1 m | 0.16 m/s | 17.6 m |
| 848 x 480, 15/s | 8.1 m | 12.6 m | 0.15 m/s | 27.7 m |
| 848 x 480, 30/s | 11.1 m | 16.3 m | 0.20 m/s | 33.3 m |

- **1280 x 800 drifted 2-3x less than 848 x 480** at the same rate: 1.5x finer angles per pixel (0.07 instead
  of 0.11 deg), and the whole sensor is 65 deg tall instead of 60 (the 848 x 480 mode cuts top and bottom).
- **Every 2nd picture beat every picture** at both resolutions: the 20-clone window then spans 1.3 s instead of
  0.67 s - more parallax from 100 m up. Every 3rd (10/s) was nearly as good; 30 clones brought nothing.
- Closed loop (the VIO flying the plane home): 848 x 480 at 30/s vs 15/s on identical flights, GPS lost 1.45 km
  out over the coarse outer ring (`gz_d455_keepfresh`, `gz_d455_keepfresh_15hz`): mean 24 vs 30 m, max 47 vs
  50 m - no difference (`results/gz_d455_rate_compare.png`); 1280 x 800 at 15/s, GPS lost 1 km out over the
  fine middle (`gz_variants`): **mean 7 m, max 12 m**, circled home 104-146 m out (RTL circle 111 m).
- The drift grows mostly on straight legs: at constant speed a single camera + IMU cannot tell the scale
  (published: straight line 9.2 % scale error, circle 6.4 %, figure-eight 4.8 %).
- Cost: OpenVINS took 7-8 ms per 1280 x 800 picture running alone (18.5 ms with six other copies running)
  against 5.5 ms per 848 x 480 picture (i7-13700H). The Jetson's cores are roughly 3-4x slower: expect
  ~25-30 ms, well inside 15/s (66 ms). Measure it on the bench (OpenVINS prints the time per picture).
- What the simulator cannot say: it has no motion blur, vibration, exposure changes or sensor noise, and its
  ground is painted at 25 cm per pixel while the 1280 x 800 camera sees 16 cm per pixel from 100 m. Real ground
  has finer detail (more gain), blur and noise eat fine pixels (less gain). The shadow flights decide: record
  at 1280 x 800, 30/s (`RECORD=1`) and replay the settings offline.

**The combination to fly (so far):**
1. D455 colour camera, whole sensor 1280 x 800 at 30/s, straight down, top of the picture towards the nose
   (`run_vio_d455.sh` default `RES=1280x800`).
2. OpenVINS on every 2nd picture (15/s), 20 clones, 200 features (`openvins_config/d455_800`); every 3rd if
   the Jetson cannot keep up.
3. Camera: auto-exposure priority off, no frame sync, accelerometer 200 + gyro 400/s interpolated; look for
   blur on the bench and cap the exposure if needed.
4. Calibration: `rs-imu-calibration.py`, then Kalibr at 1280 x 800 (the scaled OpenVINS sample is only a start).
5. Mount: rigid to the flight controller, isolated from motor vibration, nothing of the plane in view.
6. Companion: steady-flight start with matched clocks, keep-fresh (every 5 min, or at 0.7 m/s speed error),
   `VISO_TYPE 1`.
7. ArduPilot: `VISO_DELAY_MS` from the shadow flights' `age_ms` (likely the default 10); `VISO_POS_M_NSE` - the EKF now and then
   rejects the VIO position while taking its speed, then snaps back to it (jumps of a few metres).
8. Not tested yet, possibly the biggest remaining gain: flying home in gentle S-turns instead of a straight
   line (the scale becomes visible), and a lower RTL altitude while on VIO (more parallax). Both change how the
   plane flies home - a decision for the pilot.

Gotchas found on the way: OpenVINS's ROS node crashes when stopped with SIGINT, and WSL keeps a 130-180 MB
dump of each crash (1.1 GB after an afternoon of VIO restarts) - every OpenVINS stop now uses SIGKILL.
Windows' page file grows to ~5.7 GB during simulations (C: free fell 5.8 -> 1.4 GB in 30 s) and shrinks after.
rclpy checks every byte in Python when `bytes` are assigned to `Image.data` (48 ms per 1280 x 800 picture - the
grey converter managed only 8 pictures a second) - `array.array('B', ...)` avoids it. The Zephyr's vertical
launch fails in a crosswind (rolled 77 deg at 4 m) - `gz_gps_loss.py` now sets `--wind` after the climb.

### Picture quality - blur and noise (2026-09-30)

The simulator's pictures are perfect; a real camera's are smeared by motion during the exposure and grainy in
dim light. `tools/ros_rgb_to_mono.py --blur-deg 0.6 --noise 8` made spoiled copies of the live 1280 x 800
picture (the blur runs along the direction the camera IMU's gyro says it turns; the 848 x 480 copies are shrunk
from the spoiled full picture, so the shrinking averages noise away as the camera's scaler does), and six
OpenVINS ran side by side on the rectangle, 2-minute restarts, 9 segments each (`gz_quality`):

| Drift after 30 / 60 / 90 s | 1280 x 800, 15/s | 848 x 480, 15/s |
|---|---|---|
| clean | 2.1 / 3.7 / **5.5 m** | 2.4 / 4.0 / **8.5 m** |
| 0.6 deg motion blur (8 px at 1280 x 800) | 2.2 / 2.3 / **4.3 m** | 1.7 / 5.8 / **6.3 m** |
| noise, 8 grey levels | 2.4 / 5.0 / **5.9 m** | 3.6 / 5.7 / **9.2 m** |

- 1280 x 800 stays ahead with blur and with noise - the finer pixels are not wasted.
- 0.6 deg of blur did no harm. That is what a 10 ms exposure smears at a 57 deg/s roll rate (5 ms at
  115 deg/s): cap the colour camera's exposure at ~5-10 ms and look at the pictures on the bench and in the
  shadow flights. Noise costs a little. (The differences are within the scatter between flights - read them
  as "tolerates", not as a ranking.)

### Flying home in S-turns - first results (2026-09-30, to be continued)

At constant speed in a straight line a single camera + IMU cannot tell the scale, and that is where the VIO
drifted most. `gz_gps_loss.py --mission weave --restart-at-legs` flies a straight 2 km leg across the middle of
the world and back along the same line in a zig-zag (75 m either side, a turn every 300 m, ~24 deg of heading
swing), round and round, with a fresh VIO at the start of every leg; the wind (6 m/s) blows across the line
so both kinds of leg have the same ground speed (a first try with the wind along the line mixed up the two).
`compare_variants.py --split-turning --skip-first` scores the legs apart (`gz_weave2`; 2 legs of each kind -
stopped early, so a first look):

| | straight legs | zig-zag legs |
|---|---|---|
| 1280 x 800, drift after 90 s / per km flown | 12.0 m / 27.1 m per km | **3.9 m / 5.0 m per km** |
| 848 x 480, drift after 90 s / per km flown | 5.2 m / 11.6 m per km | **2.9 m / 4.7 m per km** |

Zig-zagging cut the drift per km 2.5-5x. (On these few straight legs 848 x 480 happened to do better than
1280 x 800 - straight legs scatter a lot; the 9-segment tests above are the better guide for resolution.)
`scripts/vio_gps_switch.lua` has the option ready but **not yet flight-tested**: `VSW_WEAVE` (metres either side,
0 = off, the default) and `VSW_WEAVE_D` (metres between turns, 300) - on a GPS loss with a healthy VIO the plane
flies home in GUIDED through points alternately left and right of the line home, and switches to RTL for the
last 1.5 x `VSW_WEAVE_D`, or at once if the VIO is rejected or the GPS returns; a mode change by the pilot ends
it. Next: closed-loop GPS-loss flights with `--param VSW_WEAVE=75` against 0, then decide.

### Failure tests with the real VIO chain (2026-09-30)

The switch-over script's checks were tested earlier against a simulated faulty VIO; these tests break the real
chain instead - the way a RealSense on a Jetson can fail (USB frame dropouts are a known RealSense-on-Jetson
issue). Same flight each time: GPS lost 880 m out (`--far 700,-700`), 20 s into the flight home the camera's
pictures stop for 40 s while its IMU goes on (`gz_gps_loss.py --camera-stall 20,40`), 200 s without GPS.

| Run | What the companion did | Position error without GPS | The VIO afterwards |
|---|---|---|---|
| `gz_stall_s1` - before the fix | kept forwarding OpenVINS, which ran on the IMU alone | mean 14 m, max 25 m | came back with a 20 m jump (its window was 40 s old) |
| `gz_stall_s2` - hold back only | stopped forwarding after 1 s; ArduPilot flew on airspeed + wind estimate | mean 3 m, max 6 m | never came back this time - dead reckoning to the end (fine in steady wind; not in a changing one) |
| **`gz_stall_s3` - hold back + restart** | held back, then restarted OpenVINS as soon as pictures flowed again, started from ArduPilot's estimate without GPS; **and** OpenVINS was killed 110 s into the outage (`--kill-vio 110`) | **mean 7 m, max 11 m** | back 5 s after the camera, 4 s after the crash; no jump either time |

What changed in `tools/vio_bridge.py`:
- OpenVINS publishes its estimate on every IMU sample - with the pictures gone it carries on from the IMU alone,
  and ArduPilot's IMU check cannot see that (it is the IMU agreeing with itself). Its `/ov_msckf/poseimu` comes
  only after a camera update: without one for 1 s (`--stale-s`) nothing is forwarded ("VIO: no camera updates
  - held back"), and the switch-over script treats the VIO as missing.
- After a stall of 3 s or more (`--stall-restart-s`) OpenVINS is not trusted to carry on - it came back with a
  jump (S1), or not at all (S2), or fine (a bench repeat): once pictures flow again (the camera's own per-frame
  `--camera-topic`, `/d455/color/camera_info` on the plane) it is restarted.
- Without GPS that restart - and one after an OpenVINS crash - starts from ArduPilot's current estimate
  ("VIO: fresh start without GPS"; `--no-recovery` turns it off). The new VIO carries ArduPilot's position error
  from that moment on, but gives good speed and turns again, instead of dead reckoning to the end.
- `setup/run_vio_d455.sh` now keeps the camera driver in a restart loop too (a USB reset can take it down).
- Timing: OpenVINS needed 7-8 ms per 1280 x 800 picture when running alone on this PC (18.5 ms with six
  other copies in the side-by-side test) - so ~25-30 ms on the Jetson, well inside 15 pictures a second.

### On the Jetson (Orin Nano Super, JetPack 6 = Ubuntu 22.04)

Copy this folder over (e.g. to `~/vio`), then **`bash setup/jetson_install.sh`** (not yet tried on a Jetson -
it stops at the first error; asks for your password). It installs ROS 2 Humble (base), the RealSense driver,
OpenVINS at exactly the desktop's version (commit `6948812`, 2025-11-30 - the one the change was made for)
with the change applied and built, pymavlink, serial port access, and the start-on-boot service (switched off).
Then:
1. Log out and in again (serial port access). Fastest power mode: `sudo nvpmodel -q --verbose` lists them - pick
   MAXN SUPER; `sudo jetson_clocks`.
2. Check the camera: `rs-enumerate-devices` must list colour 1280x800 at 30 and the accelerometer and gyro. If
   the motion sensors are missing, build librealsense with `-DFORCE_RSUSB_BACKEND=true` (Intel's advice for
   Jetsons: no kernel patches). Update the camera firmware if `realsense-viewer` offers it. Known JetPack snag:
   its own OpenCV packages can clash with the one ROS wants - `dpkg -l | grep -i opencv` shows them.
3. Wiring: 40-pin header pin 8 (TX) to the flight controller's RX, pin 10 (RX) to its TX, pin 6 to ground;
   both sides 3.3 V, no 5 V between them. Find which `/dev/ttyTHS*` it is - on the right one this prints the
   flight controller's heartbeat:
   `python3 -c "from pymavlink import mavutil; print(mavutil.mavlink_connection('/dev/ttyTHS0', 921600).wait_heartbeat(timeout=5))"`
4. Settings in **`setup/vio.env`** (from `vio.env.example`): `MAVLINK` (port), `LEVER` (camera position),
   `RES`, `RECORD`.
5. `bash setup/run_vio_d455.sh` starts everything by hand. It first runs **`tools/preflight_check.py`**: the
   flight controller's VIO settings (`VISO_TYPE`, both source sets, scripting, the switch-over script loaded,
   the vision arming skip, which SERIALx is at 921600) and the free space - a table in `preflight.txt`, one line
   to the ground station ("VIO check: OK" or what is off). It only warns.
6. When the tests are done: `sudo systemctl enable vio` starts the chain at every boot (`setup/vio.service`,
   settings from `vio.env`; `journalctl -u vio -f` to watch). The camera driver and OpenVINS run in restart
   loops, so a USB reset or a crash does not end it.

### Calibration (before any flight)

1. IMU: `rs-imu-calibration.py` (librealsense tools) - stores the IMU calibration in the camera.
2. Camera and camera-IMU with Kalibr and an Aprilgrid board: record the camera as in flight
   (`CAMERA_ONLY=1 RECORD=1 bash setup/run_vio_d455.sh`), calibrate the camera (pinhole-radtan), then camera + IMU (with the
   noise numbers from `openvins_config/d455/kalibr_imu_chain.yaml`). Kalibr runs in its Docker image on the
   desktop (ROS 1: convert the ROS 2 bag with `rosbags-convert`). Put `intrinsics`, `distortion_coeffs`,
   `T_cam_imu` and `timeshift_cam_imu` into `openvins_config/d455/kalibr_imucam_chain.yaml`.
3. Each run of `run_vio_d455.sh` saves what the driver reports about the camera (`camera_info.txt`); the
   focal lengths should come out near 417 px (OpenVINS's sample D455) - far off means a different mode.

### Mounting

- Looking straight down, **top of the picture towards the nose** (= `realsense-down`); any other way round
  needs its own rotation in `MOUNTS` in `vio_bridge.py`. Measure where it sits relative to the flight
  controller - `LEVER=FORWARD,RIGHT,DOWN` in metres.
- Rigid against the flight controller (a degree of tilt between the two is a degree of heading error at each
  VIO start), but away from motor vibration - check the IMU in a recording at full throttle.
- Nothing of the plane in the picture (otherwise a mask), a clean lens, no extra window if avoidable; a USB 3
  port and a short cable with strain relief.

### Test order

1. **Bench** (Jetson, camera and flight controller on the table, no propeller): `bash setup/run_vio_d455.sh` -
   "camera up", "connected (realsense-down mount ...)" in `vio_bridge.log`, `ros2 topic hz /d455/imu` about
   400 and `/d455/color/image_raw` about 30. The VIO must *not* start (not armed, not flying).
2. **OpenVINS alone, hand-held, outdoors**: `CAMERA_ONLY=1 bash setup/run_vio_d455.sh`, and in a second terminal
   `ros2 run ov_msckf run_subscribe_msckf --ros-args -r __ns:=/ov_msckf -p config_path:=$PWD/openvins_config/d455_800/estimator_config.yaml`;
   hold still 2 s, then walk a loop with the camera looking at the ground: it should come back close to where it
   started. Its log prints the time per picture: at 15 pictures a second it must stay well under 66 ms, or use
   every 3rd picture (`track_frequency: 11.0` in `openvins_config/d455_800/estimator_config.yaml` - nearly as
   good in the side-by-side test).
3. **Shadow flights** - the VIO runs and is logged, the flight controller does not use it: `VSW_ENABLE 0`,
   `RECORD=1 bash setup/run_vio_d455.sh`. Afterwards `vio_bridge.csv` has the VIO and the flight controller
   side by side: the speed difference should stay well under 0.7 m/s and "VIO: restart" should be rare. Set
   `VISO_DELAY_MS` to the typical `age_ms` plus a few ms for the serial link. The recording also holds the
   flight controller's GPS-aided track (`/fc/odom`) and the VIO start states, so OpenVINS settings can be
   replayed and scored against GPS on the desktop, as in the simulator's side-by-side test.
4. **Switch-over flights** with a safety pilot: `VSW_ENABLE 1`, a transmitter switch set to "GPS Disable"
   (`RCx_OPTION 65`) to cut the GPS in flight - first near the field, then far enough for a real RTL.

Flight controller (real plane): `params/vio_plane.parm` (source sets, `VISO_TYPE 1`, the Jetson's port -
`SERIAL5` there is an example), `ARMING_SKIPCHK 262144` (skip only the visual-odometry arming check - the VIO
starts in the air), `SCR_ENABLE 1` and `scripts/vio_gps_switch.lua` in `APM/scripts` on the SD card.

## Road map

- **A - ArduPilot side, simulator**: switch-over script and parameters, realistic VIO errors and a no-VIO
  comparison, the same test in Mission Planner (all done); VIO checks, bank limit and RTL when GPS and
  VIO are both gone (done 2026-09-29).
- **B - real VIO before the Jetson**: Ubuntu 22.04 + ROS 2 Humble + OpenVINS on recorded flights,
  then its output into the simulator over MAVLink (the same messages the Jetson will send) - done in
  Gazebo step 3 (2026-09-29): GPS lost in changing wind, the plane flew home on OpenVINS and circled there.
- **C - Jetson + camera**: bench, then flights with VIO logged but not used, then real switch-over
  tests with a safety pilot. Needs an H743 flight controller (stock F405 firmware has no visual odometry).
  The logged-only flights are also where `VISO_POS_M_NSE` / `VISO_VEL_M_NSE` and the check limits get
  tuned: while on GPS, the EKF already compares the VIO with its GPS-aided estimate (innovations of the
  unused source), so the VIO can be scored on every flight. The Jetson should also send a quality value,
  so `VISO_QUAL_MIN` can drop VIO data that the VIO itself doesn't trust.
