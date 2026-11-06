# Changelog

All notable changes to this repository are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the repository uses
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## 1.1.0 – Unreleased

### Added
- GitHub Actions CI: builds and tests the workspace in the `ros:humble` container on pull requests
  and pushes to `main`.
- `requirements.txt` with pinned pip-only dependencies (numpy, scipy, quadprog, gtsam, ultralytics).
- `scripts/bump_version.sh` to set one version across all packages.
- Repository README with build, test, launch and configuration instructions.
- Optional minimum-curvature race line from lap 2, controlled by `raceline_enable` (off by default).
- Shared helpers moved to `mfe_common` as part of the parallel refactor (in progress).

### Changed
- Boundary tracer: same-side wall check, phantom-cell re-grow, staggered-straight handling, and
  distinct no-path reasons.
- Workspace paths in the launch and build scripts can be overridden from the environment.
- Package descriptions filled in; one repo-wide version (1.1.0) across all packages.
- `mfe_state_estimation` now passes its flake8 and pep257 tests.

### Removed
- PSO race-line optimizer.
- Unused code: the Python LiDAR cone detector, `ekf_slam_node`, the `detected_cones` target, the
  unbuilt `cone_transformer_node` and `lidar_tf_broadcaster` launch entries, and the empty
  `mfe_base`, `mfe_perception_core` and `mfe_tests` packages.
- Stale `urinay` submodule entries and the tracked `.DS_Store` file.
