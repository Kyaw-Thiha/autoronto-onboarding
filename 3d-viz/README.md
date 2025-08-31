# 3d-viz

**Type:** `single`

This project is scaffolded to work seamlessly with the **ros-workspace** infra (Docker + ROS 2 dev environment).
For Docker/infra details and helper scripts like `workon.sh`, see: **ros-workspace** → https://github.com/Kyaw-Thiha/ros-workspace

---

## Contents

- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Repository Layout](#repository-layout)
- [Scripts Overview](#scripts-overview)
  - [scripts/sync.sh](#scriptssyncsh)
  - [scripts/freeze.sh](#scriptsfreezesh)
  - [scripts/build.sh](#scriptsbuildsh)
  - [scripts/clean.sh](#scriptscleansh)
  - [scripts/create_pkg.sh](#scriptscreate_pkgsh)
- [Lockfiles & Sources](#lockfiles--sources)
- [Common Tasks](#common-tasks)
- [Troubleshooting](#troubleshooting)

---

## Prerequisites

- You are using the Docker-based ROS environment from **ros-workspace** (link above).
- Inside the container you have:
  - ROS 2 (sourced automatically by our scripts).
  - `colcon`, `vcstool` (for meta projects).
- On host: a checkout of this repo (and the infra repo), plus the ability to mount this project into the container (see `workon.sh` in infra).

---

## Quick Start

On the host (outside the container), from your infra repo:

```bash
# mount this project into your ROS container
/path/to/ros-workspace/scripts/workon.sh "/home/kevin/Documents/Projects/autoronto-onboarding/3d-viz"
```

Once inside the container:

```bash
# (Only for multi-repo project) fetch sources defined in ws.repos
[[ -f ./ws.repos ]] && ./scripts/sync.sh

# build everything in ws/
./scripts/build.sh

# load the workspace environment into your current shell
source ws/install/setup.bash

# (now you can run/launch nodes, e.g.)
# ros2 run <package> <executable>
# ros2 launch <package> <launch_file.py>
```

> Single-repo projects: `./scripts/sync.sh` isn’t needed (no `ws.repos`).  
> Tip: The build script forwards any extra args to `colcon build`. For example:  
> `./scripts/build.sh --packages-select my_pkg --symlink-install`

---

## Repository Layout

- `ws/` — colcon workspace
  - `src/` — packages (committed in this repo)
  - `build/`, `install/`, `log/` — generated; ignored by git
- `.env` — per-project env overrides (e.g., `ROS_DOMAIN_ID`, RMW choice)
- `scripts/` — helper scripts for syncing, freezing, building, cleaning, and creating packages
- (meta only) `ws.repos` — list of external repos to import into `ws/src`
- (meta only) `ws.repos.lock` — pinned exact commits for reproducibility

---

## Scripts Overview

### scripts/sync.sh
**Use when:** \(meta projects\) You want to materialize or update `ws/src` from `ws.repos`.

**What it does:**
- Creates `ws/src` if needed.
- `vcs validate` the `ws.repos` file.
- `vcs import` to clone any missing repos into `ws/src`.
- `vcs pull` to update existing repos to their remote heads.

**Notes:**
- Idempotent — run anytime to bring sources up to date.
- Respects whatever ref/branch `ws.repos` lists. If you want exact pins, see `freeze.sh` below.

**Run:**
```bash
./scripts/sync.sh
```

---

### scripts/freeze.sh
**Use when:** \(meta projects\) You want to **pin** the current state of `ws/src` for reproducible builds.

**What it does:**
- Exports the **exact commits** of all repos in `ws/src` to `ws.repos.lock` via:
  `vcs export --exact ws/src > ws.repos.lock`.

**Workflow:**
1. Run `./scripts/sync.sh` to fetch/update sources.
2. Verify everything builds.
3. Run `./scripts/freeze.sh` and **commit** `ws.repos.lock`.

**Run:**
```bash
./scripts/freeze.sh
```

---

### scripts/build.sh
**Use when:** You want to build the workspace.

**What it does:**
- Loads `.env` (e.g., `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`).
- Sources `/opt/ros/${ROS_DISTRO:-kilted}/setup.bash` (container-provided).
- (Optional) You can add an underlay source line in the script if you have one.
- Runs `colcon build --symlink-install` in `ws/`.

**Pass-through args:** Any extra args are forwarded to `colcon build`. Examples:
- Select packages:
  ```bash
  ./scripts/build.sh --packages-select my_pkg another_pkg
  ```
- CMake config:
  ```bash
  ./scripts/build.sh --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
  ```

**Run:**
```bash
./scripts/build.sh
```

---

### scripts/clean.sh
**Use when:** You want to nuke build artifacts and logs.

**What it does:** `rm -rf ws/build ws/install ws/log`

**Run:**
```bash
./scripts/clean.sh
```

---

### scripts/create_pkg.sh
**Use when:** \(single projects\) You want to scaffold a new package under .

**What it does:**
- Ensures ROS env is sourced.
- Invokes  with the given name and dependencies, creating the package under .
- **Language selection:** Defaults to **C++** (). Pass ** as the first argument** to create a **Python** package ().

**Examples:**
```bash
# Minimal (C++, default)
./scripts/create_pkg.sh my_pkg

# Minimal (Python)
./scripts/create_pkg.sh --py my_pkg

# With dependencies (C++)
./scripts/create_pkg.sh my_pkg --dependencies rclcpp std_msgs sensor_msgs

# With dependencies (Python)
./scripts/create_pkg.sh --py my_pkg --dependencies rclpy std_msgs sensor_msgs
```

> Note: Run this **inside the container**.

---

## Lockfiles & Sources

- **`ws.repos`** (meta): Human-maintained list of repos + branches/versions to import into `ws/src`.
- **`ws.repos.lock`** (meta): Generated by `freeze.sh`, pins **exact commits** for reproducibility.
  - Commit this file to share a known-good state.
  - To re-sync to the locked commits, you can:
    ```bash
    vcs import ws/src < ws.repos.lock
    vcs pull ws/src
    ```

---

## Common Tasks

- **Switch RMW or Domain ID**: Edit `.env` (e.g., `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=23`).
- **Rebuild after dependency changes**: Run `./scripts/build.sh` (optionally with `--cmake-args`).
- **Run tests**:
  ```bash
  cd ws
  colcon test && colcon test-result --verbose
  ```
- **Run a node** (after sourcing install in your shell session):
  ```bash
  source ws/install/setup.bash
  ros2 run <package> <executable>
  ```

---

## Troubleshooting

- **Colcon not found / ROS not sourced:** Use the container via the infra repo and make sure you entered the shell through `workon.sh`.
- **Meta repo not pulling updates:** Ensure `ws.repos` lists the branch you expect; run `./scripts/sync.sh` again.
- **Networking between projects:** Set a unique `ROS_DOMAIN_ID` in `.env` to avoid cross-talk with other running workspaces.

---

**Infra link:** This project expects the Docker/ROS environment from **ros-workspace**: https://github.com/Kyaw-Thiha/ros-workspace

