"""
Configuración y constantes del robot PhantomX Pincher.
"""
import numpy as np

# ============================================================
#  CONFIGURACIÓN DE HARDWARE
# ============================================================
USE_XL430 = False

# Longitudes de los eslabones (metros)
L1 = 44.0 / 1000.0
L2 = 107.5 / 1000.0
L3 = 107.5 / 1000.0
L4 = 75.3 / 1000.0

# Límites de alcance
PLANAR_REACH_MAX = L2 + L3 + L4
PLANAR_REACH_MIN = 0.04
Z_MAX = L1 + L2 + L3 + L4
Z_MIN = 0.0

# ============================================================
#  CONFIGURACIÓN DE MOTORES DYNAMIXEL
# ============================================================
if USE_XL430:
    PROTOCOL_VERSION = 2.0
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_MOVING_SPEED = 112
    ADDR_TORQUE_LIMIT = 38
    ADDR_PRESENT_POSITION = 132
    DEFAULT_GOAL = 2048
    MAX_SPEED = 1023
    DXL_MAX_VALUE = 4095
else:
    PROTOCOL_VERSION = 1.0
    ADDR_TORQUE_ENABLE = 24
    ADDR_GOAL_POSITION = 30
    ADDR_MOVING_SPEED = 32
    ADDR_TORQUE_LIMIT = 34
    ADDR_PRESENT_POSITION = 36
    DEFAULT_GOAL = 512
    MAX_SPEED = 1023
    DXL_MAX_VALUE = 1023

# ============================================================
#  CONFIGURACIÓN ROS2
# ============================================================
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 1000000
DEFAULT_DXL_IDS = [1, 2, 3, 4, 5]
DEFAULT_MOVING_SPEED = 100
DEFAULT_TORQUE_LIMIT = 800

# Nombres de articulaciones para ROS2
JOINT_NAMES = [
    'phantomx_pincher_arm_shoulder_pan_joint',
    'phantomx_pincher_arm_shoulder_lift_joint',
    'phantomx_pincher_arm_elbow_flex_joint',
    'phantomx_pincher_arm_wrist_flex_joint',
    'phantomx_pincher_gripper_finger1_joint',
]

# Signos de las articulaciones
JOINT_SIGN = {1: 1, 2: 1, 3: 1, 4: 1, 5: 1}

# ============================================================
#  POSICIONES PREDEFINIDAS (en grados)
# ============================================================
PRESET_POSITIONS = [
    ("Posición 1: HOME", [0, 0, 0, 0, 0], "Todos los motores en 0°"),
    ("Posición 2: Alcance Medio", [25, 25, 20, -20, 0], "Configuración de alcance medio"),
    ("Posición 3: Lateral", [-35, 35, -30, 30, 0], "Movimiento lateral"),
    ("Posición 4: Elevada", [85, -20, 55, 25, 0], "Posición elevada"),
    ("Posición 5: Extendida", [80, -35, 55, -45, 0], "Máxima extensión"),
]

# Posiciones XYZ predefinidas (metros)
XYZ_PRESETS = [
    ("Home", 0.20, 0.00, 0.10),
    ("Alto", 0.15, 0.00, 0.20),
    ("Derecha", 0.18, 0.08, 0.05),
    ("Izquierda", 0.18, -0.08, 0.05),
]
