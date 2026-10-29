# Vendored: TUMFTM trajectory_planning_helpers (subset)

- Upstream: https://github.com/TUMFTM/trajectory_planning_helpers
- Commit: aa950f6045680366b789dbb855db8d59d54b1db5 (setup.py version 0.80)
- Licence: GNU Lesser General Public License v3.0 (see LICENSE in this directory)

Files copied unchanged except for the import lines noted below:

| File | Used for |
|---|---|
| opt_min_curv.py | min-curvature race line inside the wall corridor |
| calc_splines.py | closed spline and normal vectors for opt_min_curv |
| calc_head_curv_num.py | path curvature for the speed profile |
| calc_vel_profile.py | friction-ellipse speed profile |
| conv_filt.py, normalize_psi.py | helpers of the two files above |

Local changes (only these):

- `calc_vel_profile.py`, `calc_head_curv_num.py`: `import trajectory_planning_helpers.X` replaced by
  `from . import X`, and the matching `trajectory_planning_helpers.X.` prefixes removed, so the files work inside
  `mfe_path_planning.third_party.tph`.
- `opt_min_curv.py`: the top-level `import quadprog` is moved into the function that calls it, so importing the
  package does not need quadprog. quadprog is needed only by `mfe_path_planning.raceline.build`.

Not vendored: the plotting and `__main__` test blocks (matplotlib is imported only there).
