# XLeRobot Integration Report — 2026-04-09

## Objective

Download the `xlerobot` robot driver from the [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) repo, get the `teleoperate_LUNA.py` example running against the current lerobot codebase, and fix cross-platform Xbox controller support.

## What was done

### 1. Downloaded xlerobot folder

Used git sparse checkout to download `software/src/robots/xlerobot` from `https://github.com/Vector-Wangel/XLeRobot` into `src/lerobot/robots/xlerobot/`.

Files added:
- `src/lerobot/robots/xlerobot/__init__.py`
- `src/lerobot/robots/xlerobot/config_xlerobot.py`
- `src/lerobot/robots/xlerobot/xlerobot.py`
- `src/lerobot/robots/xlerobot/xlerobot_client.py`
- `src/lerobot/robots/xlerobot/xlerobot_host.py`

### 2. Fixed import errors in teleoperate_LUNA.py

**Error:** `ImportError: cannot import name 'busy_wait' from 'lerobot.utils.robot_utils'`

**Fix:** Changed line 20 from:
```python
from lerobot.utils.robot_utils import busy_wait
```
to:
```python
from lerobot.utils.robot_utils import precise_sleep
```
**Reason:** `busy_wait` was renamed to `precise_sleep` in the current lerobot codebase.

### 3. Fixed import errors in SO101Robot.py

**Error:** `ModuleNotFoundError: No module named 'lerobot.robots.so101_follower'`

**Fix:** Changed line 6 in `src/lerobot/model/SO101Robot.py` from:
```python
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
```
to:
```python
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
```
Also updated the usage of `SO101FollowerConfig` to `SOFollowerRobotConfig` in the `create_real_robot()` function.

**Reason:** The module `so101_follower` was renamed to `so_follower`, and the config class `SO101FollowerConfig` was renamed to `SOFollowerRobotConfig`.

### 4. Fixed cross-platform Xbox controller mapping (macOS support)

**Problem:** The script used hardcoded pygame `Joystick` button/axis indices (e.g., `buttons[9]`, `axes[4]`) that match the Windows Xbox controller layout but differ on macOS. macOS Xbox controllers have 6 axes and 11 buttons (vs Windows' 5/10), and the same physical button maps to different indices — causing stick presses, triggers, and some commands to silently fail.

**Root cause:** The raw `pygame.joystick.Joystick` API reports platform-specific HID indices.

**Fix:** Replaced raw index-based controller access with the SDL2 GameController API (`pygame._sdl2.controller`), which provides named button/axis access that works identically across platforms and controller models.

Changes made in `examples/xlerobot/teleoperate_LUNA.py`:

1. **Added `ControllerWrapper` class** (~130 lines) — a thin abstraction that:
   - Tries `pygame._sdl2.controller.Controller` first (named access via SDL2's built-in controller database)
   - Falls back to raw `Joystick` with Windows-layout indices if the controller isn't recognized
   - Exposes `get_button(name)`, `get_axis(name)`, `get_hat()` with consistent cross-platform behavior
   - Normalizes trigger axes to 0.0-1.0 and stick axes to -1.0-1.0 regardless of backend
   - Synthesizes hat tuple from D-pad buttons when using SDL2 (since GameController exposes D-pad as buttons, not hats)

2. **Refactored `get_xbox_key_state()`** — replaced all hardcoded indices with named calls:
   - `buttons[9]` → `controller.get_button('left_stick_press')`
   - `buttons[10]` → `controller.get_button('right_stick_press')`
   - `axes[4]` + normalization logic → `controller.get_axis('left_trigger')` (pre-normalized)
   - `axes[0-3]` → `controller.get_axis('left_x'/'left_y'/'right_x'/'right_y')`
   - `buttons[0-6]` → `controller.get_button('a'/'b'/'x'/'y'/'back')`
   - Removed all `if len(axes) > N else False` guards (no longer needed)

3. **Refactored `get_base_action()`** — uses `controller.get_hat()` instead of raw joystick hat access

4. **Refactored `get_base_speed_control()`** — uses `controller.get_button('lb'/'rb')` instead of `buttons[4]/buttons[5]`

5. **Updated `main()`** — initializes `ControllerWrapper(0)` instead of raw `Joystick(0)`, global reset uses `controller.get_button('back')`

### 5. Enabled camera configuration for Jetson host

**Problem:** Camera feeds appeared in rerun.io on Windows but not on macOS. Investigation showed this was not a platform issue — all camera configs in `xlerobot_cameras_config()` were commented out in the freshly downloaded code. The Windows setup had them enabled.

**Fix:** Uncommented the right wrist and head camera configs in `src/lerobot/robots/xlerobot/config_xlerobot.py`:
- **Right wrist**: OpenCV camera at `/dev/video2`, 640x480, 30fps
- **Head**: RealSense camera (serial `125322060037`), 1280x720, 30fps, BGR, depth enabled

**Note:** The robot host runs on a Jetson (not Raspberry Pi). Device paths and camera serial numbers may need adjustment to match the actual Jetson hardware. Verify with `rs-enumerate-devices` on the Jetson.

### 6. Updated project configuration

- Updated `CLAUDE.md`: Changed guidance from `uv run` to `conda run -n lerobot` for executing Python commands.
- Saved memory: This project uses conda env `lerobot`, not `uv`.

## Current status

All import errors are resolved. Cross-platform controller mapping is implemented. Cameras are configured for the Jetson host. The script runs successfully past imports and initialization, and only fails at runtime when expected hardware (Xbox controller, robot) is not connected — which is expected behavior.

## Files modified

| File | Change |
|------|--------|
| `examples/xlerobot/teleoperate_LUNA.py` | `busy_wait` → `precise_sleep` import; added `ControllerWrapper` class; refactored `get_xbox_key_state`, `get_base_action`, `get_base_speed_control`, and `main()` to use named controller access |
| `src/lerobot/model/SO101Robot.py` | `so101_follower` → `so_follower` module, `SO101FollowerConfig` → `SOFollowerRobotConfig` |
| `src/lerobot/robots/xlerobot/config_xlerobot.py` | Enabled right wrist (OpenCV) and head (RealSense) camera configs |
| `CLAUDE.md` | `uv run` → `conda run -n lerobot` |

## Files added

| File | Source |
|------|--------|
| `src/lerobot/robots/xlerobot/*` (5 files) | Downloaded from Vector-Wangel/XLeRobot repo |
