"""
Vendored subset of TUMFTM trajectory_planning_helpers (LGPL-3.0, see LICENSE in this directory).

Upstream https://github.com/TUMFTM/trajectory_planning_helpers, commit aa950f6 (version 0.80).
Files are unchanged except:
- calc_vel_profile.py, calc_head_curv_num.py: package imports made relative (`from . import X`).
- opt_min_curv.py: `import quadprog` moved into the function, so importing the package needs no quadprog.
The plotting and `__main__` test blocks are not included.
"""
