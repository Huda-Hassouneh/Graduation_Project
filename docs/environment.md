# Development environment (verified 2026-10-28)

| Item | Verified value | How verified |
|---|---|---|
| Host | Windows 11 + WSL 2, kernel 6.6.87.2-microsoft-standard-WSL2 | `uname` via session environment |
| Distribution | Ubuntu 24.04.5 LTS (`ros-jazzy`) | stated by user, consistent with package versions |
| Python | 3.12.3 (`/usr/bin/python3`) | `python3 --version` |
| NumPy / SciPy / Matplotlib | 1.26.4 / 1.11.4 / 3.6.3 | imported and printed `__version__` |
| PyYAML / pytest | 6.0.1 / 7.4.4 | imported and printed `__version__` |
| pandas | **not installed** | `ImportError` |
| pip / ensurepip | **not available** | `pip3: command not found`, `No module named 'ensurepip'` |
| ROS 2 | Jazzy at `/opt/ros/jazzy` | `source /opt/ros/jazzy/setup.bash && ros2 --help` |
| Gazebo | Gazebo Sim 8.15.0 (Harmonic) | `gz sim --version` |
| Gazebo Python bindings | `gz.transport13`, `gz.msgs10` in `/usr/lib/python3/dist-packages` (no ROS sourcing needed) | `python3 -c "import gz.transport13"` (2026-10-30) |
| Gazebo model path | not set in a fresh shell; `simulation/gazebo/env.sh` sets `GZ_SIM_RESOURCE_PATH` and `SDF_PATH` | `echo $GZ_SIM_RESOURCE_PATH` (2026-10-30) |
| GitHub CLI | **not installed** | `gh: command not found` |
| Git | 2.43.0 | `git --version` |

## Dependency decisions

* The scientific core uses only the system NumPy/SciPy/Matplotlib/PyYAML, so nothing was installed.
* pandas is not required: time series are written as CSV with the standard library / NumPy.
  If Parquet output or pandas-based analysis is wanted later, install `python3-pandas` with apt
  (requires sudo) and pin the version here.
* The package is not pip-installed; pytest finds it through `pythonpath = ["src"]` in
  `pyproject.toml`, and scripts are run with `PYTHONPATH=src`.
* No remote repository was created: `gh` is unavailable and no authentication was verified.
  To publish later: create `Graduation-project` on GitHub (names cannot contain spaces), then
  `git remote add origin <url> && git push -u origin main`.
