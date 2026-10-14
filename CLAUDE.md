# MFE Driverless V1 — Developer Handoff

Read this before starting work. Update it at the end of any session where significant code changes were made.

---

## How to run

```bash
bash ~/Develop/MFE-Driverless-V1/scripts/docker_run.sh [track] [mode] [gui] [laps]

# Examples
bash ~/Develop/MFE-Driverless-V1/scripts/docker_run.sh peanut no_perception gui 0   # endless peanut
bash ~/Develop/MFE-Driverless-V1/scripts/docker_run.sh accel no_perception gui 1    # acceleration
bash ~/Develop/MFE-Driverless-V1/scripts/docker_run.sh small_track no_perception nogui 3
```

`docker_run.sh` calls `launch_sim.sh` inside the container — never run `launch_sim.sh` directly from the host.
After launch, a 6-pane tmux session opens. **Press Enter in pane 4** (mid-right) to start the car.

---

## Architecture

```
EUFS Gazebo sim
  └─ mfe_eufs_sim bridge        (adapts EUFS ↔ MFE topics)
       ├─ /planning/cones        → path_planner_node  →  /planning/centerline
       │                                                        └─ pure_pursuit_node  →  /control/command → bridge → Gazebo
       ├─ /ground_truth/state_odom → pure_pursuit_node (pose)
       └─ /lidar/points_raw       → finish_detector_node
```

Key files:

| File | Role |
|------|------|
| `ros2/src/mfe_path_planning/mfe_path_planning/path_planner_node.py` | ft-fsd path planner; hardcoded path for skidpad only |
| `ros2/src/mfe_control/mfe_control/pure_pursuit_node.py` | Pure pursuit + lookahead velocity profile |
| `ros2/src/mfe_bringup/launch/bringup.launch.py` | All node launch config, mission-specific params |
| `ros2/src/mfe_validation/mfe_validation/slam_node.py` | GTSAM cone-landmark SLAM (LiDAR-only validation stack) |
| `ros2/src/mfe_validation/mfe_validation/skidpad_planner.py` | Fits the known skidpad layout to SLAM landmarks, publishes `/planning/centerline` |
| `scripts/docker_run.sh` | Host-side launcher (run this) |
| `scripts/launch_sim.sh` | In-container tmux launcher (called by docker_run.sh) |
| `docs/` | Full reference: nodes.md, simulation.md, architecture.md, etc. |

---

## Mission types

| Track arg | Mission | Path planning | Notes |
|-----------|---------|---------------|-------|
| `accel` | acceleration | ft-fsd | Straight ~78 m |
| `skidpad` | skidpad | **hardcoded** | 2L+2R laps — ft-fsd can't express multi-lap sequence |
| `peanut` | peanut | ft-fsd (trackdrive, global) | Crossing figure-8; global planning over the full cone map resolves the crossing |
| `small_track`, `autocross`, etc. | autocross | ft-fsd (local) | Closed loop |

### Why peanut uses trackdrive global planning
The peanut track is a crossing figure-8 (verified from lap traces: the car crosses the waist on two diagonals per lap). The waist has a double blue wall — two rows 2-4 m apart, both "left" for their own passage. Local planning with ANY visibility radius makes the ft-fsd boundary trace jump walls there (measured: car led off-track twice per lap). `MissionTypes.trackdrive` with the FULL cone map (no visibility filter) resolves it: the global trace follows each wall around its own loop by ring continuity. Blue = left boundary, yellow = right boundary.

**Rule: hardcoded / track-specific paths are allowed ONLY for accel and skidpad.** Everything else must be generic cone-based planning — no track-name branches in the planner.

---

## Key implementation notes

### path_planner_node.py
- `self._global_planning` (true for trackdrive-type missions): pass the FULL cone map to ft-fsd, no visibility/behind filter. Local missions keep `CONE_VIS_R = 50 m` + 3 m behind-car filter.
- **Lap-1 lock-in** (global missions only): during lap 1 the node records a breadcrumb trail (points ≥0.5 m apart). When the car closes the loop (trail distance >60 m, max excursion from start >15 m, back within 4 m of start), `_build_locked_loop()` builds a full-lap centerline: walk the trail every ~1 m, take the midpoint of the nearest blue/yellow wall cones (each within 5 m; else keep the drive line), smooth (cyclic 3-point average), run `_min_curvature_path` once (safety 0.5), then a clearance guard reverts any waypoint the optimizer pushed within 1.0 m of a cone back to its raw centered position. The result is closed (path[0] == path[-1], so pure pursuit wraps) and published unchanged forever — per-cycle re-planning stops. Trail ordering is what makes this correct on self-crossing tracks (figure-8): midpoints are sequenced by where the car actually drove.
- **Locked loop is resampled to uniform ~1.0 m spacing** (2026-07-19): consecutive trail steps collapse onto the same wall-cone midpoint, so the raw locked line is only as dense as the wall-cone spacing (measured: 56 wpts for a 97.7 m peanut lap, 1.75 m avg). That coarseness caused 2.7 m cross-track at the SE hairpin (measured 4-18 cone hits/lap) and halves the index separation between figure-8 crossing passes toward pure pursuit's 30-wpt search window (the skidpad lap-skip failure mode). Resampling by linear interpolation along the closed polyline keeps path[0]==path[-1].
- Racing-line optimizer half-widths are capped at 2.0 m (`_W_MAX` in `_min_curvature_path`) — where a boundary wall has a gap (figure-8 crossing, missed detections) the nearest-cone distance blows up and the line would swing through the gap into cones (measured: 10-15 hits/lap at the peanut waist before the cap).
- Skidpad still uses a hardcoded path (in `_make_skidpad_path()`). The old hardcoded `_PEANUT_PATH` is removed — peanut is pure cone-based planning now.

### pure_pursuit_node.py
- `_path_idx` resets when the incoming path's **content** changes (first/middle/last waypoints compared with 1 cm epsilon): ft-fsd replanned paths differ every cycle → reset every time; bit-identical re-publishes (skidpad hardcoded, peanut locked loop) keep the index. Do NOT gate the reset on "first waypoint moved >1 m" — ft-fsd's first waypoint tracks the car and moves less than that per replan, so the index was kept on paths that had slid under the car, and the forward-only closest search could ratchet ahead.
- Closest-waypoint search window is **30** and must stay **below the waypoints-per-lap of sequenced self-crossing paths** (skidpad circles ≈ 50 wp/lap): at the central crossing the next segment's waypoints sit at the same physical spot, and a wider window makes the tracker jump segments and skip laps. Regression 2026-07-12: window was widened to 60 during the centerline overhaul → skidpad drove entry + 1 left lap + exit (~95 m instead of ~290 m). Verified by offline kinematic sim: window 60 → lobe sequence "L"; window 30 → "LLRR".
- The >10 m relocalization fallback searches **forward-only** (from `_path_idx` to path end). A full-path backward search snapped to colocated waypoints from an earlier skidpad lap at the crossing and made the car repeat a lap (measured: 5 circle laps instead of 4). Closed-loop wrap-around is a separate explicit branch.
- Wrap-around in `_find_lookahead_point`: only for **closed paths** (path[0] ≈ path[-1] within 3 m). Prevents accel car reversing at the finish line.
- Velocity profile: scans 40 waypoints ahead, computes Menger curvature, brakes if `v²-v_target² > 2·a_brake·d`.
- Stale-path failsafe: full brake if no path update for >2 s (planner blind, e.g. no cones in view). Without it the car once sailed 60+ m off-track on a stale path at max speed.

### bringup.launch.py
- `_make_pure_pursuit()`: mission-specific speed/lookahead (skidpad: 4.5 m/s / 3 m; peanut: 6 m/s / 4.5 m; accel: 13 m/s / 8 m). Steering limit 28° (EUFS sim clamps at ~30° internally, so higher values saturate).
- `max_lateral_accel = 4.0 m/s²` (base, all missions) — at 8.0 the velocity profile never braked for the tightest corners and the car ran wide (measured on peanut's right-lobe hairpin).
- `_make_finish_detector()`: peanut/autocross use `return_to_start_r=5 m`, `min_travel_m=60 m`. Acceleration and skidpad use **perception-based orange-gate detection**: the finish detector accumulates ORANGE_BIG cones from `/planning/cones`, clusters them into gates (single-linkage 8 m), and counts passes through a gate. Enter radius is **4.5 m** (leave at +2 m hysteresis) — tighter radii miss real passes because pure pursuit corner-cuts the skidpad crossing by up to ~3 m (measured: at 3.0 m the two left-lap crossings never fired). Pass debounce is 5 m of travel (10 m would eat skidpad's entry pass at ≈7 m); a gate the car spawns inside (accel start gate) is neutralized by init-inside logic instead. Accel: 1 pass, brake immediately. Skidpad: 5 passes (entry + 4 laps of the central gate at (14.4, 0), cones at (12.9,±1.8)/(15.9,±1.8)) + 17 m run-out to reach the stop zone (exit-corridor cones end at x≈35.4). `finish_x` is now an EMERGENCY fallback only (accel 85, skidpad 36), fires only if the gate is never perceived.
- Gate cones flow: bridge forwards `big_orange_cones` → ORANGE_BIG on `/planning/cones` (GT-bypass); the planner drops ORANGE_BIG/ORANGE_SMALL at intake so they never pollute the ft-fsd boundary trace (they used to be omitted at the bridge for that reason).
- **Endless mode (`laps=0`) disables the finish detector entirely** — the car never stops, and on non-loop tracks (skidpad/accel) it runs out of path at the end and stalls. Skidpad/accel should be run with laps=1.

### bridge_node.py (mfe_eufs_sim)
- ONE subscription to `/ground_truth/track` feeds both `/ground_truth/track_colored` and (GT-bypass mode) `/planning/cones`. Duplicate subscriptions to the same topic in one node were observed to **silently starve** — one callback delivered, the other never fired, so the planner went blind while track_colored kept flowing. Don't re-split them.

### boundary_extractor.py
- If the map←lidar TF lookup fails, LiDAR cones are dropped for that tick instead of publishing untransformed lidar-frame points mislabelled as map-frame cones.
- Camera cones are filtered to metric-only before use: pixel-mode detections (z = -1.0 sentinel, frame_id `camera_pixel`, published when the vision node has no depth/camera_info) are excluded, and all camera cones are dropped if the map←camera TF fails. Previously raw pixel values (e.g. (320, 240)) leaked in as garbage cone positions.
- TF lookups are stamp-aware: transform at the sensor-capture stamp (fallback: latest) so cones land where the car WAS when the sensor saw them — latest-only lookup smeared positions by car motion since capture (~0.3-0.6 m at 6 m/s with YOLO latency).
- Unmatched camera cones within `CAM_DEDUP_RADIUS` (1.2 m) of a LiDAR centroid are dropped, not appended — they are the same physical cone seen with camera error; appending created phantom duplicates between the 0.5 m match radius and camera position error.
- GT color fallback is now per-cone, not all-or-nothing: camera color wins when a LiDAR centroid matches a camera cone (0.5 m), otherwise the GT fallback colors it. Previously ANY camera cone disabled the GT fallback entirely — with the camera actually working (2026-07-18), that left most cones UNKNOWN (measured 24 of 28) and starved the planner of boundary colors. Sim-only: on hardware the GT topic is absent and camera-unmatched cones stay UNKNOWN.
- Camera-only cones (no LiDAR confirmation) are appended only within `CAM_ONLY_MAX_RANGE` (12 m) of the car (TF map←base_footprint; none appended if that TF is unavailable). Depth error grows with range and the planner's persistent global cone map never reconciles phantoms away.

### perception_evaluator
- `fusion_evaluator_node` subscribes `camera/camera_info` (relative) and bringup remaps it to `/zed/camera_info` — it previously hard-subscribed `/camera/camera_info`, which nothing publishes in sim, so pixel-mode matching never had intrinsics.
- LiDAR→pixel projection uses the `camera_frame` param (default `zed_depth_camera_frame`) and converts body→optical convention before applying intrinsics (EUFS "optical" frames are identity-rotated/body-convention; the old code projected against a nonexistent `camera_base` frame with optical math).

### cone_detection_node.py (vision)
- Frame enqueue is non-blocking (`put_nowait`, drop-oldest on full) — a ROS callback must never block the executor.
- YOLO inference now passes the configured `conf`/`iou` thresholds (previously defaults were silently used).
- 16UC1 depth is converted mm→m at decode time (a dead-code dtype check previously left uint16 depth in millimeters → 1000× range error on hardware sensors like the D435i; sim ZED publishes 32FC1 so this only bites on hardware).
- Depth indexing scales RGB pixel coords into depth-image coords — depth streams at a different resolution than RGB no longer misindex.
- `process_and_publish_detections` runs BEFORE `render_detection`: the mean-colour ROI must sample the clean frame, not one with green boxes drawn on it.
- `depth_detection_node.py` deleted — abandoned broken stub (crashed on init, never registered in setup.py), superseded by `depth_callback` here.
- **YOLO class→color mapping fixed** to the trained `best.pt` class order (0=blue, 1=yellow, 2=orange_cone SMALL, 3=large_orange_cone BIG, 4=unknown). Classes 2/3 were swapped — small oranges were published as ORANGE_BIG, which would pollute the finish detector's orange-gate clustering in perception mode.
- **Depth validity gate is `isfinite`, not `!isnan`**: Gazebo depth cameras publish **+inf** for no-return pixels. inf depth passed the old metric gate and back-projected to inf/NaN cone coordinates (`0 * inf = NaN`), which crashed the path planner (`round(NaN)` ValueError — observed 2026-07-18, first day the camera ever produced metric cones). ROI depth averaging also filters to finite values only. Defense in depth: boundary_extractor drops non-finite camera cones, and the planner's `_cones_cb` skips non-finite cones instead of dying.
- **Cones are stamped with the image CAPTURE time**, threaded through the frame queue as (frame, stamp) tuples. Publish-time stamps made stamp-aware TF consumers place cones where the car was AFTER YOLO inference (100-300 ms on CPU ≈ 1-2 m of motion at speed). First perception-mode drive (2026-07-18, publish-time stamps + no camera range cap) hit 37 cones in 83 m on small_track.

### lidar_cone_detector (C++)
- `lidar_perception_node`: cone-body Z-band is parameterized (`cone_z_min`/`cone_z_max`, defaults -1.10/-0.50 for the sim VLP-16 at ~1.0 m height — set from mounting height on hardware), used by the CPU (non-GPU) fallback build path.
- **GPU pipeline history (superseded, 2026-09-06):** an earlier session rewrote the GPU path onto raw `cudaSegmentation` + `cudaExtractCluster` calls with corrected `extract()` output semantics — that version was untested on Jetson and has since been superseded by a teammate's follow-on work on `origin/neil-fix` (33 commits, merged in this session): the GPU path now does `cudaFilter` (ROI + voxel grid) on GPU only, then downloads to CPU for `SACMODEL_PERPENDICULAR_PLANE` RANSAC ground removal (raw `cudaSegmentation` only supports an unconstrained plane model, which mis-fits a flat track), plane-relative height-band + Y-range cone isolation, CPU Euclidean clustering, and bounding-box cone-shape validation (rejects clusters too big to be a cone — FSAE DD.1.3.2 envelope). `min_intensity_`-based intensity filtering (retroreflective tape) was also added on top of this. The raw `cudaSegmentation`/`cudaExtractCluster` GPU-only path is gone; don't reintroduce it without addressing the flat-plane-fit bug the teammate's commits document.
- `cone_detector.cpp` also picked up geometry-based L/R cone assignment for LiDAR-only mode (no color from LiDAR alone) — not yet reviewed in detail this session.
- `lidar_preprocessor` node renamed from a copy-pasted "ground_plane_removal_node" (name collision) and preserves the input frame_id instead of stamping "map".
- Legacy `detected_cones` node now actually publishes `mfe_msgs/Track` on `cones` (color UNKNOWN; was computing centroids and discarding them). Needed `<depend>mfe_msgs</depend>` in package.xml.

### launch_sim.sh
- `BRIDGE_MAX_SPEED` must match the bringup pure-pursuit `max_speed` for the mission (peanut 6.0, autocross/trackdrive 8.0) — the bridge clamps commands, so a mismatch silently caps or exceeds intended speed.

### lib/
- `lib/gtsam` — git submodule, `borglab/gtsam` pinned to tag `4.2.0`. Reference/build-version pin only — the actual runtime dependency is the `gtsam==4.2` pip wheel (`pip install --user --no-deps gtsam`), installed separately in whatever environment runs `slam_node.py`. This submodule is not built or imported from; don't assume it needs compiling.
- `lib/cudaPCL` — the LiDAR+GTSAM-SLAM validation/testing harness (`eval/score_run.py`, `eval/render_video.py`, `scripts/run_slam_headless.sh`, etc.), vendored in as plain tracked files (not a submodule — it's this team's own code, no separate remote). Scripts that load a track's cone CSV resolve the EUFS tracks directory from the `EUFS_MASTER` env var (same variable the launch scripts already export) instead of a hardcoded absolute path — falls back to `~/Develop/MFE26-eufs-sim` if unset.
  - `lib/cudaPCL/validation/` (the `mfe_validation` ROS2 package: `slam_node.py`, `skidpad_planner.py`) is now a **stale duplicate** — the real copy lives at `ros2/src/mfe_validation/` and is built automatically by the normal `colcon build` over `ros2/src`, same as every other package. The old `lib/cudaPCL/validation/` copy still exists because other in-flight work may still be using the separately-merged `simws_local` workspace that symlinks to it; clean it up once that's no longer needed. Don't maintain both in parallel — fix bugs in `ros2/src/mfe_validation/`, not here.
  - `gtsam` is installed in `Docker/fs-driverless-sim/Dockerfile` (`pip3 install gtsam==4.2`) — no longer a manual `pip install --user --no-deps gtsam` step in a running container.
- Not to be confused with `ros2/src/mfe_perception/lidar_cone_detector/lib/cuda-pcl` — a separate, unrelated submodule (NVIDIA's `cuda-pcl` GPU point-cloud library), pre-existing, different purpose, different name (no capital P/no hyphen distinguishes `cudaPCL` from `cuda-pcl`).

---

## Current status

### Working
- `accel` ✓
- `small_track` / autocross ✓ (ft-fsd, multi-lap, finish detection; re-verified 2026-07-12: 1 lap, 0 cone hits, max deviation 1.9 m — after fixing a May-3 typo in MFE26-eufs-sim `small_track.launch` (`$(var pub_ground)truth)` → `$(var pub_ground_truth)`) that had crashed the EUFS launch for this track since then. That fix lives in the MFE26-eufs-sim repo (uncommitted).
- `skidpad` ✓ (hardcoded 2L+2R path; 2026-07-12: fixed lap-skipping regression — search window 60→30 — and switched finish to perception-based orange-gate passes. Verified in-sim headless: 5 gate passes at travel 10/65/120/175/230 m — exactly entry + 4 laps — gate-triggered brake at x=27, rest at x=28.98 inside the stop corridor, no coordinate involved)
- `peanut` — REGRESSED since 2026-07-11 (<1 hit/lap then; re-measured 2026-07-19: 4-18 hits/lap, cross-track up to 2.7 m at the right lobe SE hairpin, both during lap-1 exploration and on the locked loop; peanut was never re-verified after the 7-12 pure-pursuit changes). Locked-loop 1 m resampling fix applied 2026-07-19 — re-verification in progress, see below.

### Perception-node fix pass (2026-07-18)
All perception nodes audited and fixed (see Key implementation notes for details):
- vision node: 16UC1 depth mm→m, ROI colour sampled before box drawing, depth-resolution scaling; dead `depth_detection_node.py` deleted; `file_loader.py` FPS fallback fixed (was 1 frame / 33 s).
- boundary_extractor: stamp-aware TF, camera-cone dedup vs LiDAR centroids.
- fusion_evaluator: camera_info wiring (bringup remap → /zed/camera_info), projection frame + body→optical conversion.
- lidar C++: preprocessor node-name collision + frame_id, `cone_z_min/max` params, cuPCL GPU cluster semantics rewrite (untested — needs Jetson), legacy `detected_cones` now publishes Track.
- **Dockerfile (fs-driverless-sim): torch/ultralytics added (CPU wheels)** — the sim image finally has the vision stack, so the YOLO node can run in perception mode. CPU torch on purpose: the cu128 CUDA wheels (~4.5 GB) repeatedly failed to download from the PyTorch CDN, and CPU YOLO11n keeps up with the sim's ~10 Hz camera. Rebuild with `docker build -t mfe-driverless-sim Docker/fs-driverless-sim`.
- **VERIFIED END-TO-END (2026-07-18)**: `small_track perception nogui 1` — full lap completed, **0 cone hits**, mission FINISHED cleanly (return-to-start). Max deviation ~1.8 m at the same corner as the no_perception baseline (1.9 m). Chain: ZED camera trio → YOLO (CPU) + VLP-16 → C++ lidar node (~7.7 Hz) → boundary_extractor (all cones colored: camera first, GT fallback) → planner → pure pursuit → finish detector. First-ever perception-mode lap; before the capture-stamp + camera-range-cap fixes the same run hit 37 cones in 83 m.

### In progress (2026-09-06)
- Pulling `origin/neil-fix` via `git rebase origin/neil-fix`: brings in 33 teammate commits (LiDAR GPU/CPU pipeline rework, Mac Dockerfile fixes, cone filter tuning for FSAE DD.1.3.2 large-orange gate cones). Local tip was 1 commit ("Random fixes", 2026-08-08) ahead of the old origin.
- Rebase hit one conflict in `lidar_perception_node.cpp` (raw GPU cudaSegmentation/cudaCluster reintroduction vs. teammate's CPU-RANSAC replacement — see lidar_cone_detector notes above). Resolution in progress in-session; **rebase not yet continued/completed** as of this note — verify `git status` before assuming this is done.

### Known remaining issues
- Peanut: occasional cone hits (~4 per 5 laps) in the right lobe's SE corner — tuning candidates: pure pursuit lookahead, optimizer safety margin.
- autocross regression untested since `max_lateral_accel` 8→4 (will brake more in corners — safer but slower; retest lap times if they matter).
- Perception mode + gate finish: gate detection needs ORANGE_BIG on `/planning/cones`; in perception mode that depends on boundary_extractor's GT color fallback labelling them. Verified 2026-07-13: perception mode DOES put 3 ORANGE_BIG on /planning/cones via the GT fallback.
- **Perception mode findings (2026-07-13):** LiDAR chain works as wired (`/lidar/points_raw` → C++ `lidar_perception_node` → `/perception/cones_uncolored` @ ~7 Hz → boundary_extractor → colored `/planning/cones`). Gazebo classic camera sensors need a DISPLAY/X context to render — headless containers without `--env DISPLAY --volume /tmp/.X11-unix` get NO camera topics at all (the CPU-raycast lidar still works, which masks it). YOLO consumes the ZED **depth-camera trio** (`/zed/image_raw` + `/zed/depth/image_raw` + `/zed/camera_info` — same Gazebo sensor, pixel-aligned) with `output_body_frame:=true` because the EUFS "optical" TF frames are identity-rotated (body convention), and stamps `zed_depth_camera_frame`.
- GPU (cuPCL) path of `lidar_perception_node` compiles only on Jetson and the rewritten cluster extraction has not been run on hardware yet.
- Acceleration untested since the gate-finish change (gate pass at x≈78 should brake the car to rest ≈92; emergency fallback at 85).

---

## Update instructions
When ending a session with code changes, update the **Current status** section above and add any new design decisions to the relevant notes section.
