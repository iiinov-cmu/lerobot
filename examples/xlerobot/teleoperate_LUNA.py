# To Run on the host
'''
PYTHONPATH=src python -m lerobot.robots.xlerobot.xlerobot_host --robot.id=my_xlerobot
'''

# To Run the teleop:
'''
python -m examples.xlerobot.teleoperate_LUNA
'''

# raspberry_pi_ip = "172.26.228.175"  # UPDATE THIS WITH YOUR RASPBERRY PI'S IP, dont uncomment this line

import time
import numpy as np
import math
import pygame

from lerobot.robots.xlerobot.config_xlerobot import XLerobotClientConfig
from lerobot.robots.xlerobot.xlerobot_client import XLerobotClient
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from lerobot.model.SO101Robot import SO101Kinematics
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig

# Keymaps (semantic action: controller mapping) - Intuitive human control
LEFT_KEYMAP = {
#     # Left stick controls left arm XY (when not pressed)
#     'x+': 'left_stick_up', 'x-': 'left_stick_down',
#     'y+': 'left_stick_pressed_up', 'y-': 'left_stick_pressed_down',
#     # Left stick pressed controls left arm shoulder_pan
#     'shoulder_pan+': 'left_stick_right', 'shoulder_pan-': 'left_stick_left',
#     # LB pressed controls left arm pitch and wrist_roll
#     'pitch+': 'lb_up', 'pitch-': 'lb_down',
#     'wrist_roll+': 'lb_right', 'wrist_roll-': 'lb_left',
#     # Left trigger controls left gripper
#     'gripper+': 'left_trigger',
    # Head motors
    "head_motor_1+": 'b', "head_motor_1-": 'x',
    "head_motor_2+": 'a', "head_motor_2-": 'y',
     }
RIGHT_KEYMAP = {
    # Right stick controls right arm XY (when not pressed)
    'x+': 'left_stick_up', 'x-': 'left_stick_down',
    'y+': 'right_stick_up', 'y-': 'right_stick_down',
    # Right stick pressed controls right arm shoulder_pan
    'shoulder_pan+': 'left_stick_right', 'shoulder_pan-': 'left_stick_left',
    # RB pressed controls right arm pitch and wrist_roll
    'pitch+': 'lb_up', 'pitch-': 'lb_down',
    'wrist_roll+': 'lb_right', 'wrist_roll-': 'lb_left',
    # Left trigger controls right gripper (toggle)
    'gripper_toggle': 'left_trigger',
}

# Base control keymap - Only forward/backward and rotate left/right
BASE_KEYMAP = {
    'forward': 'dpad_down', 'backward': 'dpad_up',
    'rotate_left': 'dpad_left', 'rotate_right': 'dpad_right',
}

# Global reset key for all components
RESET_KEY = 'back'

RIGHT_JOINT_MAP = {
    "shoulder_pan": "right_arm_shoulder_pan",
    "shoulder_lift": "right_arm_shoulder_lift",
    "elbow_flex": "right_arm_elbow_flex",
    "wrist_flex": "right_arm_wrist_flex",
    "wrist_roll": "right_arm_wrist_roll",
    "gripper": "right_arm_gripper",
}

HEAD_MOTOR_MAP = {
    "head_motor_1": "head_motor_1",
    "head_motor_2": "head_motor_2",
}

class SimpleHeadControl:
    def __init__(self, initial_obs, kp=1):
        self.kp = kp
        self.degree_step = 1
        # Initialize head motor positions
        self.target_positions = {
            "head_motor_1": initial_obs.get("head_motor_1.pos", 0.0),
            "head_motor_2": initial_obs.get("head_motor_2.pos", 0.0),
        }
        self.zero_pos = {"head_motor_1": 0.0, "head_motor_2": 0.0}

    def move_to_zero_position(self, robot):
        self.target_positions = self.zero_pos.copy()
        action = self.p_control_action(robot)
        robot.send_action(action)

    def handle_keys(self, key_state):
        if key_state.get('head_motor_1+'):
            self.target_positions["head_motor_1"] += self.degree_step
            print(f"[HEAD] head_motor_1: {self.target_positions['head_motor_1']}")
        if key_state.get('head_motor_1-'):
            self.target_positions["head_motor_1"] -= self.degree_step
            print(f"[HEAD] head_motor_1: {self.target_positions['head_motor_1']}")
        if key_state.get('head_motor_2+'):
            self.target_positions["head_motor_2"] += self.degree_step
            print(f"[HEAD] head_motor_2: {self.target_positions['head_motor_2']}")
        if key_state.get('head_motor_2-'):
            self.target_positions["head_motor_2"] -= self.degree_step
            print(f"[HEAD] head_motor_2: {self.target_positions['head_motor_2']}")

    def p_control_action(self, robot):
        obs = robot.get_observation()
        action = {}
        for motor in self.target_positions:
            current = obs.get(f"{HEAD_MOTOR_MAP[motor]}.pos", 0.0)
            error = self.target_positions[motor] - current
            control = self.kp * error
            action[f"{HEAD_MOTOR_MAP[motor]}.pos"] = current + control
        return action

class SimpleTeleopArm:
    def __init__(self, kinematics, joint_map, initial_obs, prefix="left", kp=1):
        self.kinematics = kinematics
        self.joint_map = joint_map
        self.prefix = prefix  # To distinguish left and right arm
        self.kp = kp
        # Initial joint positions
        self.joint_positions = {
            "shoulder_pan": initial_obs.get(f"{prefix}_arm_shoulder_pan.pos", 0.0),
            "shoulder_lift": initial_obs.get(f"{prefix}_arm_shoulder_lift.pos", 0.0),
            "elbow_flex": initial_obs.get(f"{prefix}_arm_elbow_flex.pos", 0.0),
            "wrist_flex": initial_obs.get(f"{prefix}_arm_wrist_flex.pos", 0.0),
            "wrist_roll": initial_obs.get(f"{prefix}_arm_wrist_roll.pos", 0.0),
            "gripper": initial_obs.get(f"{prefix}_arm_gripper.pos", 0.0),
        }
        # Set initial x/y to fixed values
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        # Set the degree step and xy step
        self.degree_step = 2
        self.xy_step = 0.005
        # Set target positions to zero for P control
        self.target_positions = {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
        self.zero_pos = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }
        # Gripper toggle state (for left trigger)
        self.gripper_toggle_state = False  # False = open, True = closed
        self.last_toggle_pressed = False  # Track previous state to detect edge

    def move_to_zero_position(self, robot):
        print(f"[{self.prefix}] Moving to Zero Position: {self.zero_pos} ......")
        self.target_positions = self.zero_pos.copy()  # Use copy to avoid reference issues
        
        # Reset kinematic variables to their initial state
        self.current_x = 0.1629
        self.current_y = 0.1131
        self.pitch = 0.0
        
        # Don't let handle_keys recalculate wrist_flex - set it explicitly
        self.target_positions["wrist_flex"] = 0.0
        
        action = self.p_control_action(robot)
        robot.send_action(action)

    def handle_keys(self, key_state):
        # Joint increments
        if key_state.get('shoulder_pan+'):
            self.target_positions["shoulder_pan"] += self.degree_step
            print(f"[{self.prefix}] shoulder_pan: {self.target_positions['shoulder_pan']}")
        if key_state.get('shoulder_pan-'):
            self.target_positions["shoulder_pan"] -= self.degree_step
            print(f"[{self.prefix}] shoulder_pan: {self.target_positions['shoulder_pan']}")
        if key_state.get('wrist_roll+'):
            self.target_positions["wrist_roll"] += self.degree_step
            print(f"[{self.prefix}] wrist_roll: {self.target_positions['wrist_roll']}")
        if key_state.get('wrist_roll-'):
            self.target_positions["wrist_roll"] -= self.degree_step
            print(f"[{self.prefix}] wrist_roll: {self.target_positions['wrist_roll']}")
        
        # Gripper control
        # Left trigger: toggle on/off
        toggle_pressed = key_state.get('gripper_toggle', False)
        if toggle_pressed and not self.last_toggle_pressed:
            # Edge detected: toggle the state
            self.gripper_toggle_state = not self.gripper_toggle_state
            if self.gripper_toggle_state:
                print(f"[{self.prefix}] gripper: TOGGLE CLOSED")
            else:
                print(f"[{self.prefix}] gripper: TOGGLE OPENED")
        self.last_toggle_pressed = toggle_pressed
        
        # Set gripper position based on toggle state
        if self.gripper_toggle_state:
            # Toggle is active (closed)
            self.target_positions["gripper"] = 2
        else:
            # Default: open
            self.target_positions["gripper"] = 90
        
        if key_state.get('pitch+'):
            self.pitch += self.degree_step
            print(f"[{self.prefix}] pitch: {self.pitch}")
        if key_state.get('pitch-'):
            self.pitch -= self.degree_step
            print(f"[{self.prefix}] pitch: {self.pitch}")

        # XY plane (IK)
        moved = False
        if key_state.get('x+'):
            self.current_x += self.xy_step
            moved = True
            print(f"[{self.prefix}] x+: {self.current_x:.4f}, y: {self.current_y:.4f}")
        if key_state.get('x-'):
            self.current_x -= self.xy_step
            moved = True
            print(f"[{self.prefix}] x-: {self.current_x:.4f}, y: {self.current_y:.4f}")
        if key_state.get('y+'):
            self.current_y += self.xy_step
            moved = True
            print(f"[{self.prefix}] x: {self.current_x:.4f}, y+: {self.current_y:.4f}")
        if key_state.get('y-'):
            self.current_y -= self.xy_step
            moved = True
            print(f"[{self.prefix}] x: {self.current_x:.4f}, y-: {self.current_y:.4f}")
        if moved:
            joint2, joint3 = self.kinematics.inverse_kinematics(self.current_x, self.current_y)
            self.target_positions["shoulder_lift"] = joint2
            self.target_positions["elbow_flex"] = joint3
            print(f"[{self.prefix}] shoulder_lift: {joint2}, elbow_flex: {joint3}")

        # Wrist flex is always coupled to pitch and the other two
        self.target_positions["wrist_flex"] = (
            -self.target_positions["shoulder_lift"]
            -self.target_positions["elbow_flex"]
            + self.pitch
        )
        # print(f"[{self.prefix}] wrist_flex: {self.target_positions['wrist_flex']}")

    def p_control_action(self, robot):
        obs = robot.get_observation()
        current = {j: obs[f"{self.prefix}_arm_{j}.pos"] for j in self.joint_map}
        action = {}
        for j in self.target_positions:
            error = self.target_positions[j] - current[j]
            control = self.kp * error
            action[f"{self.joint_map[j]}.pos"] = current[j] + control
        return action
    

# --- Controller Wrapper for cross-platform Xbox support ---
class ControllerWrapper:
    """Wraps SDL2 GameController (preferred) or raw Joystick with a uniform interface.

    The SDL2 GameController API uses named buttons/axes that map correctly across
    platforms (macOS, Windows, Linux) and controller models (Xbox 360, One, Series X).
    Falls back to raw Joystick with Windows-layout indices if the controller isn't
    recognized by SDL2's database.
    """

    _SDL2_BUTTONS = None  # Lazily initialized after pygame import
    _SDL2_AXES = None

    # Raw Joystick (Windows Xbox layout) fallback mappings
    _RAW_BUTTONS = {
        'a': 0, 'b': 1, 'x': 2, 'y': 3,
        'lb': 4, 'rb': 5,
        'back': 6, 'start': 7, 'guide': 8,
        'left_stick_press': 9, 'right_stick_press': 10,
    }
    _RAW_AXES = {
        'left_x': 0, 'left_y': 1,
        'right_x': 2, 'right_y': 3,
        'left_trigger': 4,
    }

    def __init__(self, joystick_index=0):
        self._use_sdl2 = False
        self._controller = None
        self._joystick = None

        # Try SDL2 GameController first
        try:
            import pygame._sdl2.controller as sdl2_ctrl
            sdl2_ctrl.init()
            if sdl2_ctrl.is_controller(joystick_index):
                self._controller = sdl2_ctrl.Controller(joystick_index)
                self._use_sdl2 = True
                self._init_sdl2_maps()
                print(f"[CONTROLLER] Using SDL2 GameController API: {self._controller.name}")
            else:
                self._init_raw_joystick(joystick_index)
        except ImportError:
            self._init_raw_joystick(joystick_index)

    def _init_raw_joystick(self, index):
        self._joystick = pygame.joystick.Joystick(index)
        self._joystick.init()
        print(f"[CONTROLLER] Using raw Joystick API (fallback): {self._joystick.get_name()}")
        print(f"[CONTROLLER] WARNING: Raw Joystick mappings may be incorrect on this platform")

    def _init_sdl2_maps(self):
        if ControllerWrapper._SDL2_BUTTONS is None:
            ControllerWrapper._SDL2_BUTTONS = {
                'a': pygame.CONTROLLER_BUTTON_A,
                'b': pygame.CONTROLLER_BUTTON_B,
                'x': pygame.CONTROLLER_BUTTON_X,
                'y': pygame.CONTROLLER_BUTTON_Y,
                'back': pygame.CONTROLLER_BUTTON_BACK,
                'guide': pygame.CONTROLLER_BUTTON_GUIDE,
                'start': pygame.CONTROLLER_BUTTON_START,
                'left_stick_press': pygame.CONTROLLER_BUTTON_LEFTSTICK,
                'right_stick_press': pygame.CONTROLLER_BUTTON_RIGHTSTICK,
                'lb': pygame.CONTROLLER_BUTTON_LEFTSHOULDER,
                'rb': pygame.CONTROLLER_BUTTON_RIGHTSHOULDER,
                'dpad_up': pygame.CONTROLLER_BUTTON_DPAD_UP,
                'dpad_down': pygame.CONTROLLER_BUTTON_DPAD_DOWN,
                'dpad_left': pygame.CONTROLLER_BUTTON_DPAD_LEFT,
                'dpad_right': pygame.CONTROLLER_BUTTON_DPAD_RIGHT,
            }
            ControllerWrapper._SDL2_AXES = {
                'left_x': pygame.CONTROLLER_AXIS_LEFTX,
                'left_y': pygame.CONTROLLER_AXIS_LEFTY,
                'right_x': pygame.CONTROLLER_AXIS_RIGHTX,
                'right_y': pygame.CONTROLLER_AXIS_RIGHTY,
                'left_trigger': pygame.CONTROLLER_AXIS_TRIGGERLEFT,
                'right_trigger': pygame.CONTROLLER_AXIS_TRIGGERRIGHT,
            }

    @property
    def name(self) -> str:
        if self._use_sdl2:
            return self._controller.name
        return self._joystick.get_name()

    def get_button(self, name: str) -> bool:
        if self._use_sdl2:
            btn_id = self._SDL2_BUTTONS.get(name)
            if btn_id is None:
                return False
            return self._controller.get_button(btn_id)
        else:
            btn_idx = self._RAW_BUTTONS.get(name)
            if btn_idx is None or btn_idx >= self._joystick.get_numbuttons():
                return False
            return bool(self._joystick.get_button(btn_idx))

    def get_axis(self, name: str) -> float:
        """Returns normalized float: -1.0 to 1.0 for sticks, 0.0 to 1.0 for triggers."""
        if self._use_sdl2:
            axis_id = self._SDL2_AXES.get(name)
            if axis_id is None:
                return 0.0
            raw = self._controller.get_axis(axis_id)  # int16: -32768 to 32767
            if name in ('left_trigger', 'right_trigger'):
                return max(0.0, raw / 32767.0)
            return raw / 32767.0
        else:
            axis_idx = self._RAW_AXES.get(name)
            if axis_idx is None or axis_idx >= self._joystick.get_numaxes():
                return 0.0
            raw = self._joystick.get_axis(axis_idx)  # float -1.0 to 1.0
            if name in ('left_trigger', 'right_trigger'):
                if raw < 0:
                    return (raw + 1.0) / 2.0
                return raw
            return raw

    def get_hat(self) -> tuple:
        """Returns (x, y) D-pad state: x in {-1,0,1}, y in {-1,0,1}."""
        if self._use_sdl2:
            x = 0
            y = 0
            if self._controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_RIGHT):
                x = 1
            elif self._controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_LEFT):
                x = -1
            if self._controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_UP):
                y = 1
            elif self._controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_DOWN):
                y = -1
            return (x, y)
        else:
            if self._joystick.get_numhats() > 0:
                return self._joystick.get_hat(0)
            return (0, 0)


# --- XBOX Controller Mapping ---
def get_xbox_key_state(controller, keymap):
    """
    Map XBOX controller state to semantic action booleans using the provided keymap.
    Uses ControllerWrapper for cross-platform named button/axis access.
    """
    hats = controller.get_hat()

    left_stick_pressed = controller.get_button('left_stick_press')
    right_stick_pressed = controller.get_button('right_stick_press')
    lb_pressed = controller.get_button('lb')
    rb_pressed = controller.get_button('rb')

    left_x = controller.get_axis('left_x')
    left_y = controller.get_axis('left_y')
    right_x = controller.get_axis('right_x')
    right_y = controller.get_axis('right_y')

    state = {}
    for action, control in keymap.items():
        if control == 'left_trigger':
            state[action] = controller.get_axis('left_trigger') > 0.5
        elif control == 'a':
            state[action] = controller.get_button('a')
        elif control == 'b':
            state[action] = controller.get_button('b')
        elif control == 'x':
            state[action] = controller.get_button('x')
        elif control == 'y':
            state[action] = controller.get_button('y')
        elif control == 'back':
            state[action] = controller.get_button('back')
        elif control == 'dpad_up':
            state[action] = hats[1] == 1
        elif control == 'dpad_down':
            state[action] = hats[1] == -1
        elif control == 'dpad_left':
            state[action] = hats[0] == -1
        elif control == 'dpad_right':
            state[action] = hats[0] == 1
        # Left stick controls (when not pressed)
        elif control == 'left_stick_up':
            state[action] = (not left_stick_pressed) and (not lb_pressed) and (left_y < -0.5)
        elif control == 'left_stick_down':
            state[action] = (not left_stick_pressed) and (not lb_pressed) and (left_y > 0.5)
        elif control == 'left_stick_left':
            state[action] = (not left_stick_pressed) and (not lb_pressed) and (left_x < -0.5)
        elif control == 'left_stick_right':
            state[action] = (not left_stick_pressed) and (not lb_pressed) and (left_x > 0.5)
        # Right stick controls (when not pressed)
        elif control == 'right_stick_up':
            state[action] = (not right_stick_pressed) and (not rb_pressed) and (right_y < -0.5)
        elif control == 'right_stick_down':
            state[action] = (not right_stick_pressed) and (not rb_pressed) and (right_y > 0.5)
        elif control == 'right_stick_left':
            state[action] = (not right_stick_pressed) and (not rb_pressed) and (right_x < -0.5)
        elif control == 'right_stick_right':
            state[action] = (not right_stick_pressed) and (not rb_pressed) and (right_x > 0.5)
        # Left stick pressed controls
        elif control == 'left_stick_pressed_right':
            state[action] = left_stick_pressed and (not lb_pressed) and (left_x > 0.5)
        elif control == 'left_stick_pressed_left':
            state[action] = left_stick_pressed and (not lb_pressed) and (left_x < -0.5)
        # Right stick pressed controls
        elif control == 'right_stick_pressed_right':
            state[action] = right_stick_pressed and (not rb_pressed) and (right_x > 0.5)
        elif control == 'right_stick_pressed_left':
            state[action] = right_stick_pressed and (not rb_pressed) and (right_x < -0.5)
        # LB pressed controls (only when stick is moved)
        elif control == 'lb_up':
            state[action] = lb_pressed and (left_y < -0.5)
        elif control == 'lb_down':
            state[action] = lb_pressed and (left_y > 0.5)
        elif control == 'lb_right':
            state[action] = lb_pressed and (left_x > 0.5)
        elif control == 'lb_left':
            state[action] = lb_pressed and (left_x < -0.5)
        # RB pressed controls (only when stick is moved)
        elif control == 'rb_up':
            state[action] = rb_pressed and (right_y < -0.5)
        elif control == 'rb_down':
            state[action] = rb_pressed and (right_y > 0.5)
        elif control == 'rb_right':
            state[action] = rb_pressed and (right_x > 0.5)
        elif control == 'rb_left':
            state[action] = rb_pressed and (right_x < -0.5)
        else:
            state[action] = False
    return state

def get_base_action(controller, robot):
    """
    Get base action from XBOX controller input - simplified to only forward/backward and rotate.
    """
    hats = controller.get_hat()

    # Get pressed keys for base control
    pressed_keys = set()

    # Map controller inputs to keyboard-like keys for base control
    if hats[1] == 1:   # D-pad up
        pressed_keys.add('k')  # Forward
    if hats[1] == -1:  # D-pad down
        pressed_keys.add('i')  # Backward
    if hats[0] == -1:  # D-pad left
        pressed_keys.add('u')  # Rotate left
    if hats[0] == 1:   # D-pad right
        pressed_keys.add('o')  # Rotate right

    # Convert to numpy array and get base action
    keyboard_keys = np.array(list(pressed_keys))
    base_action = robot._from_keyboard_to_base_action(keyboard_keys) or {}

    return base_action

def get_base_speed_control(controller):
    """
    Get base speed control from XBOX controller - LB for speed decrease, RB for speed increase.
    Returns speed multiplier (1.0, 2.0, or 3.0) and prints current speed level.
    """
    lb_pressed = controller.get_button('lb')
    rb_pressed = controller.get_button('rb')
    
    # Get current speed level from global variable
    global current_base_speed_level
    if 'current_base_speed_level' not in globals():
        current_base_speed_level = 1  # Default speed level
    
    # Speed control logic
    if lb_pressed and not rb_pressed:
        # LB pressed alone - decrease speed
        if current_base_speed_level > 1:
            current_base_speed_level -= 1
            print(f"[BASE] Speed decreased to level {current_base_speed_level}")
    elif rb_pressed and not lb_pressed:
        # RB pressed alone - increase speed
        if current_base_speed_level < 3:
            current_base_speed_level += 1
            print(f"[BASE] Speed increased to level {current_base_speed_level}")
    
    # Map speed level to multiplier
    speed_multiplier = float(current_base_speed_level)
    
    return speed_multiplier


def main():
    FPS = 30
    # Find it by running 'hostname -I' on the Raspberry Pi
    raspberry_pi_ip = "172.26.228.175"  # UPDATE THIS WITH YOUR RASPBERRY PI'S IP
    
    # RealSense camera configuration
    # You can specify the serial number or name of your RealSense camera
    # To find your camera serial number, you can use: rs-enumerate-devices
    # Or leave as None to auto-detect the first available RealSense camera
    REALSENSE_SERIAL_OR_NAME = None  # Set to your camera's serial number or name, e.g., "146322070293" or None for auto-detect
    REALSENSE_WIDTH = 640
    REALSENSE_HEIGHT = 480
    REALSENSE_FPS = 30
    
    print(f"[MAIN] Attempting to connect to robot at {raspberry_pi_ip}...")
    print(f"[MAIN] Make sure the robot host is running on the Raspberry Pi:")
    print(f"[MAIN]   PYTHONPATH=src python -m lerobot.robots.xlerobot.xlerobot_host --robot.id=my_xlerobot")
    print(f"[MAIN]")
    
    robot_config = XLerobotClientConfig(remote_ip=raspberry_pi_ip)
    robot = XLerobotClient(robot_config)
    try:
        robot.connect()
        print(f"[MAIN] Successfully connected to robot at {raspberry_pi_ip}")
    except Exception as e:
        print(f"[MAIN] Failed to connect to robot: {e}")
        print(f"[MAIN]")
        print(f"[MAIN] Troubleshooting:")
        print(f"[MAIN] 1. Verify robot host is running on Raspberry Pi:")
        print(f"[MAIN]    PYTHONPATH=src python -m lerobot.robots.xlerobot.xlerobot_host --robot.id=my_xlerobot")
        print(f"[MAIN] 2. Check IP address is correct:")
        print(f"[MAIN]    - On Raspberry Pi, run: hostname -I")
        print(f"[MAIN]    - Current IP in script: {raspberry_pi_ip}")
        print(f"[MAIN] 3. Ensure both devices are on the same network")
        print(f"[MAIN] 4. Check firewall on Raspberry Pi:")
        print(f"[MAIN]    sudo ufw allow 5555 && sudo ufw allow 5556")
        print(f"[MAIN] 5. Test connectivity: ping {raspberry_pi_ip}")
        print(f"[MAIN]")
        print(f"[MAIN] Config: {robot_config}")
        return

    init_rerun(session_name="xlerobot_teleop_xbox")

    # Init RealSense camera for head (client-side camera - commented out)
    head_camera = None
    # try:
    #     print("[MAIN] Initializing RealSense camera...")
    #     if REALSENSE_SERIAL_OR_NAME is None:
    #         print("[MAIN] Auto-detecting RealSense camera...")
    #         # Try to find available RealSense cameras
    #         try:
    #             available_cameras = RealSenseCamera.find_cameras()
    #             if not available_cameras:
    #                 raise RuntimeError("No RealSense cameras found. Make sure your camera is connected.")
    #             
    #             # Use the first available camera
    #             first_camera_id = available_cameras[0].get("id", "")
    #             print(f"[MAIN] Found {len(available_cameras)} RealSense camera(s), using: {first_camera_id}")
    #             camera_config = RealSenseCameraConfig(
    #                 serial_number_or_name=first_camera_id,
    #                 fps=REALSENSE_FPS,
    #                 width=REALSENSE_WIDTH,
    #                 height=REALSENSE_HEIGHT
    #             )
    #         except ImportError:
    #             raise RuntimeError("pyrealsense2 library not found. Install it with: pip install pyrealsense2")
    #         except Exception as e:
    #             raise RuntimeError(f"Failed to detect RealSense cameras: {e}")
    #     else:
    #         print(f"[MAIN] Using specified RealSense camera: {REALSENSE_SERIAL_OR_NAME}")
    #         camera_config = RealSenseCameraConfig(
    #             serial_number_or_name=REALSENSE_SERIAL_OR_NAME,
    #             fps=REALSENSE_FPS,
    #             width=REALSENSE_WIDTH,
    #             height=REALSENSE_HEIGHT
    #         )
    #     
    #     head_camera = RealSenseCamera(camera_config)
    #     head_camera.connect()
    #     print("[MAIN] RealSense camera connected successfully!")
    # except Exception as e:
    #     print(f"[MAIN] Warning: Failed to initialize RealSense camera: {e}")
    #     print("[MAIN] Continuing without camera feed...")
    #     print("[MAIN] To fix:")
    #     print("[MAIN]   1. Install pyrealsense2: pip install pyrealsense2")
    #     print("[MAIN]   2. Ensure your RealSense camera is connected and recognized")
    #     print("[MAIN]   3. You can find your camera serial number by running: lerobot-find-cameras realsense")
    #     print("[MAIN]   4. Then set REALSENSE_SERIAL_OR_NAME to your camera's serial number")

    # Init XBOX controller
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No XBOX controller detected!")
        return
    controller = ControllerWrapper(0)
    print(f"[MAIN] Using controller: {controller.name}")

    # Init the arm and head instances
    obs = robot.get_observation()
    kin_right = SO101Kinematics()
    right_arm = SimpleTeleopArm(kin_right, RIGHT_JOINT_MAP, obs, prefix="right")
    head_control = SimpleHeadControl(obs)

    # Move arm and head to zero position at start
    right_arm.move_to_zero_position(robot)

    try:
        while True:
            pygame.event.pump()
            head_key_state = get_xbox_key_state(controller, LEFT_KEYMAP)
            right_key_state = get_xbox_key_state(controller, RIGHT_KEYMAP)

            # Check for global reset (back button)
            global_reset = controller.get_button('back')

            # Handle global reset for all components
            if global_reset:
                print("[MAIN] Global reset triggered!")
                right_arm.move_to_zero_position(robot)
                head_control.move_to_zero_position(robot)
                continue

            # Handle arm and head
            right_arm.handle_keys(right_key_state)
            head_control.handle_keys(head_key_state)

            right_action = right_arm.p_control_action(robot)
            head_action = head_control.p_control_action(robot)

            # Get base action and speed control from controller
            base_action = get_base_action(controller, robot)
            speed_multiplier = get_base_speed_control(controller)
            
            # Apply speed multiplier to base actions if they exist
            if base_action:
                for key in base_action:
                    if 'vel' in key or 'velocity' in key:  # Apply to velocity commands
                        base_action[key] *= speed_multiplier

            # Merge all actions
            action = {**right_action, **head_action, **base_action}
            robot.send_action(action)

            obs = robot.get_observation()
            
            # Debug: Print available observation keys (only once)
            if not hasattr(main, '_obs_keys_printed'):
                print(f"[MAIN] Available observation keys: {list(obs.keys())}")
                # Check for camera keys
                camera_keys = [k for k in obs.keys() if 'camera' in k.lower() or 'wrist' in k.lower() or 'head' in k.lower()]
                if camera_keys:
                    print(f"[MAIN] Camera-related keys found: {camera_keys}")
                else:
                    print("[MAIN] No camera keys found in observations. Check robot host camera configuration.")
                main._obs_keys_printed = True
            
            # Read camera frame if available (client-side camera - commented out)
            # if head_camera is not None:
            #     try:
            #         camera_frame = head_camera.read()
            #         if camera_frame is not None:
            #             # Add camera frame to observation for rerun visualization
            #             # The frame should be in CHW format (channels, height, width) for rerun
            #             # RealSense camera.read() typically returns HWC, so we may need to transpose
            #             if isinstance(camera_frame, np.ndarray):
            #                 # Ensure it's in the right format for rerun (will be auto-converted if needed)
            #                 obs["head_camera"] = camera_frame
            #     except Exception as e:
            #         # Silently continue if camera read fails (camera might be busy)
            #         pass
            
            log_rerun_data(obs, action)
    finally:
        # Cleanup
        # if head_camera is not None:
        #     try:
        #         head_camera.disconnect()
        #         print("[MAIN] RealSense camera disconnected")
        #     except Exception as e:
        #         print(f"[MAIN] Error disconnecting camera: {e}")
        robot.disconnect()
        print("Teleoperation ended.")

if __name__ == "__main__":
    main()