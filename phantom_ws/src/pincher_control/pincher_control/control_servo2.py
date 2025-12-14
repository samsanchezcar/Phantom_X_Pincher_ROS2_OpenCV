import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QSlider, QLineEdit,
                             QFrame, QMessageBox, QStackedWidget, QScrollArea,
                             QGridLayout, QSpacerItem, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer, QSize, pyqtSignal
from PyQt5.QtGui import QFont, QPixmap, QIcon, QPalette, QColor
import threading
import subprocess
import os
import sys
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

from PyQt5.QtGui import QImage

#Necesario para opencv_detector
import cv2

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

from .opencv_detector import (
    detectar_figuras_y_polares,
    pick_best_detection,
    shape_name_to_code,
    polar_to_cartesian_m,
)


# ============================================================
#  CONFIGURACIÓN
# ============================================================
USE_XL430 = False

# ============================================================
#  CONFIG OPEN-CV (GUI)
# ============================================================
OPENCV_CAMERA_INDEX = 0

# Offset cartesiano (m) - POR AHORA 0, cambia aquí cuando calibres
OPENCV_OFFSET_X_M = 0.0   # <-- CAMBIAR AQUÍ
OPENCV_OFFSET_Y_M = 0.0   # <-- CAMBIAR AQUÍ

# Z fijo para pick (m) - POR AHORA 0, cambia aquí cuando definas altura real
Z_PICK_M = 0.0            # <-- CAMBIAR AQUÍ
Z_APPROACH_M = Z_PICK_M + 0.05   # altura de aproximación (recomendado)

# Puntos destino por figura (m) - CONFIGURA AQUÍ tus trayectorias
DROP_POINTS = {
    "s": (0.18,  0.08, 0.10),  # <-- CAMBIAR AQUÍ
    "r": (0.18, -0.08, 0.10),  # <-- CAMBIAR AQUÍ
    "c": (0.24,  0.00, 0.10),  # <-- CAMBIAR AQUÍ
    "p": (0.14,  0.00, 0.10),  # <-- CAMBIAR AQUÍ
}

# Gripper (en grados) - PLACEHOLDER, calibra según tu garra real
GRIPPER_OPEN_DEG = 0.0     # <-- CAMBIAR AQUÍ
GRIPPER_CLOSE_DEG = 35.0   # <-- CAMBIAR AQUÍ

# Esperas entre movimientos (seg)
MOVE_WAIT_S = 1.0          # <-- si tu robot tarda más, súbelo

######################################################
L1 = 44.0  / 1000.0
L2 = 107.5 / 1000.0
L3 = 107.5 / 1000.0
L4 = 75.3  / 1000.0

PLANAR_REACH_MAX = L2 + L3 + L4
PLANAR_REACH_MIN = 0.04
Z_MAX = L1 + L2 + L3 + L4
Z_MIN = 0.0

if USE_XL430:
    PROTOCOL_VERSION = 2.0
    ADDR_TORQUE_ENABLE    = 64
    ADDR_GOAL_POSITION    = 116
    ADDR_MOVING_SPEED     = 112
    ADDR_TORQUE_LIMIT     = 38
    ADDR_PRESENT_POSITION = 132
    DEFAULT_GOAL = 2048
    MAX_SPEED = 1023
    DXL_MAX_VALUE = 4095
else:
    PROTOCOL_VERSION = 1.0
    ADDR_TORQUE_ENABLE    = 24
    ADDR_GOAL_POSITION    = 30
    ADDR_MOVING_SPEED     = 32
    ADDR_TORQUE_LIMIT     = 34
    ADDR_PRESENT_POSITION = 36
    DEFAULT_GOAL = 512
    MAX_SPEED = 1023
    DXL_MAX_VALUE = 1023

# ============================================================
#  TEMA MODERNO
# ============================================================
MODERN_STYLESHEET = """
* {
    font-family: 'Segoe UI', 'Ubuntu', sans-serif;
}

QMainWindow {
    background-color: #1e1e2e;
}

QWidget#centralWidget {
    background-color: #1e1e2e;
}

QWidget#contentArea {
    background-color: #2a2a3e;
    border-radius: 15px;
}

QFrame#sidebar {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                stop:0 #0f0f1e, stop:1 #1a1a2e);
    border-right: 2px solid #00d9ff;
}

QPushButton#menuButton {
    background-color: transparent;
    color: #b0b0c0;
    text-align: left;
    padding: 18px 20px;
    border: none;
    border-left: 4px solid transparent;
    font-size: 13pt;
    font-weight: 500;
}

QPushButton#menuButton:hover {
    background-color: #2a2a3e;
    color: #00d9ff;
    border-left: 4px solid #00d9ff;
}

QPushButton#menuButton:checked {
    background-color: #0f4c75;
    color: #00d9ff;
    border-left: 4px solid #00d9ff;
    font-weight: bold;
}

QPushButton#homeButtonSidebar {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #6a11cb, stop:1 #2575fc);
    color: white;
    padding: 15px 20px;
    border: none;
    border-radius: 8px;
    font-size: 12pt;
    font-weight: bold;
    margin: 10px;
}

QPushButton#homeButtonSidebar:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #7f1ed6, stop:1 #3d87ff);
}

QPushButton#emergencyButtonSidebar {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #ff416c, stop:1 #ff4b2b);
    color: white;
    padding: 15px 20px;
    border: none;
    border-radius: 8px;
    font-size: 12pt;
    font-weight: bold;
    margin: 10px;
}

QPushButton#emergencyButtonSidebar:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #ff5a7f, stop:1 #ff6747);
}

QLabel#pageTitle {
    color: #00d9ff;
    font-size: 24pt;
    font-weight: bold;
    padding: 10px;
}

QLabel#sectionTitle {
    color: #00d9ff;
    font-size: 16pt;
    font-weight: bold;
    padding: 8px;
}

QLabel#subtitle {
    color: #b0b0c0;
    font-size: 11pt;
    padding: 5px;
}

QFrame#card {
    background-color: #252538;
    border-radius: 12px;
    border: 1px solid #3a3a4e;
    padding: 15px;
}

QFrame#card:hover {
    border: 1px solid #00d9ff;
}

QPushButton {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #00a8cc, stop:1 #00d9ff);
    color: #ffffff;
    border: none;
    padding: 12px 25px;
    border-radius: 8px;
    font-weight: bold;
    font-size: 11pt;
}

QPushButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #00d9ff, stop:1 #00e5ff);
}

QPushButton:pressed {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #008ba3, stop:1 #00a8cc);
}

QPushButton:disabled {
    background-color: #3a3a4e;
    color: #6a6a7e;
}

QPushButton#presetButton {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #667eea, stop:1 #764ba2);
    padding: 20px;
    font-size: 12pt;
    min-height: 80px;
}

QPushButton#presetButton:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #7b92f5, stop:1 #8a5fb5);
}

QLabel {
    color: #e0e0e0;
    font-size: 10pt;
}

QLabel#statusLabel {
    background-color: #252538;
    border: 2px solid #00d9ff;
    border-radius: 8px;
    padding: 10px;
    font-weight: bold;
    font-size: 11pt;
    color: #00d9ff;
}

QLabel#motorLabel {
    color: #00d9ff;
    font-weight: bold;
    font-size: 11pt;
}

QLabel#valueLabel {
    background-color: #1a1a2e;
    border-radius: 5px;
    padding: 8px;
    font-weight: bold;
    color: #00d9ff;
    min-width: 60px;
}

QLineEdit {
    background-color: #1a1a2e;
    border: 2px solid #3a3a4e;
    border-radius: 6px;
    padding: 10px;
    color: #e0e0e0;
    font-size: 11pt;
}

QLineEdit:focus {
    border: 2px solid #00d9ff;
}

QSlider::groove:horizontal {
    border: none;
    height: 10px;
    background: #1a1a2e;
    border-radius: 5px;
}

QSlider::handle:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #00a8cc, stop:1 #00d9ff);
    border: 2px solid #00d9ff;
    width: 22px;
    margin: -6px 0;
    border-radius: 11px;
}

QSlider::handle:horizontal:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #00d9ff, stop:1 #00e5ff);
}

QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #00a8cc, stop:1 #00d9ff);
    border-radius: 5px;
}

QScrollArea {
    border: none;
    background-color: transparent;
}

QScrollBar:vertical {
    background-color: #1a1a2e;
    width: 12px;
    border-radius: 6px;
}

QScrollBar::handle:vertical {
    background-color: #00d9ff;
    border-radius: 6px;
    min-height: 20px;
}

QScrollBar::handle:vertical:hover {
    background-color: #00e5ff;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}
"""

# ============================================================
#  FUNCIONES AUXILIARES
# ============================================================

def write_goal_position(packet, port, dxl_id, position):
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))

def write_moving_speed(packet, port, dxl_id, speed):
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))

def read_present_position(packet, port, dxl_id):
    if USE_XL430:
        return packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    else:
        return packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)

def build_pincher_robot():
    links = [
        rtb.RevoluteDH(d=L1, a=0.0,  alpha=np.pi/2, offset=0.0),
        rtb.RevoluteDH(d=0.0, a=L2, alpha=0.0,     offset=np.pi/2),
        rtb.RevoluteDH(d=0.0, a=L3, alpha=0.0,     offset=0.0),
        rtb.RevoluteDH(d=0.0, a=L4, alpha=0.0,     offset=0.0),
    ]
    robot = rtb.DHRobot(links, name="Pincher")
    T_tool = SE3.Rz(-np.pi/2) * SE3.Rx(-np.pi/2)
    robot.tool = T_tool
    return robot

# ============================================================
#  CONTROLADOR ROS2
# ============================================================

class PincherController(Node):
    # Señal para actualizar GUI
    position_changed = None  # Se asignará desde GUI
    
    def __init__(self):
        super().__init__('pincher_controller')
        
        self.robot_model = build_pincher_robot()
        self.planar_reach_max = PLANAR_REACH_MAX
        self.planar_reach_min = PLANAR_REACH_MIN
        self.z_min = Z_MIN
        self.z_max = Z_MAX

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])
        self.declare_parameter('goal_positions', [DEFAULT_GOAL] * 5)
        self.declare_parameter('moving_speed', 100)
        self.declare_parameter('torque_limit', 800)

        port_name = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.dxl_ids = self.get_parameter('dxl_ids').value
        goal_positions = self.get_parameter('goal_positions').value
        moving_speed = int(self.get_parameter('moving_speed').value)
        torque_limit = int(self.get_parameter('torque_limit').value)

        self.port = PortHandler(port_name)
        if not self.port.openPort():
            self.get_logger().error(f'No se pudo abrir el puerto {port_name}')
            rclpy.shutdown()
            return

        if not self.port.setBaudRate(baudrate):
            self.get_logger().error(f'No se pudo configurar baudrate={baudrate}')
            self.port.closePort()
            rclpy.shutdown()
            return

        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.emergency_stop_activated = False
        
        from sensor_msgs.msg import JointState
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        #######################################################
        # ============================================================
        #  NUEVO: Publishers OpenCV (mismo nodo que joint_states)
        # ============================================================
        self.opencv_shape_pub = self.create_publisher(String, '/opencv/shape', 10)
        self.opencv_point_pub = self.create_publisher(PointStamped, '/opencv/target_point', 10)

        # Última medición OpenCV guardada (para rutina paso a paso)
        self.last_opencv_measurement = None

        ##################################################3
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)
        
        self.current_joint_positions = [0.0] * 5
        
        self.joint_names = [
            'phantomx_pincher_arm_shoulder_pan_joint',
            'phantomx_pincher_arm_shoulder_lift_joint',
            'phantomx_pincher_arm_elbow_flex_joint',
            'phantomx_pincher_arm_wrist_flex_joint',
            'phantomx_pincher_gripper_finger1_joint',
        ]

        # CORREGIDO: Motor 1 normal, motores 2,3,4,5 invertidos
        self.joint_sign = {1: 1, 2: 1, 3: 1, 4: 1, 5: 1}
        
        self.initialize_motors(goal_positions, moving_speed, torque_limit)

    def dxl_to_radians(self, dxl_value):
        if USE_XL430:
            center, scale = 2048.0, 2.618 / 2048.0
        else:
            center, scale = 512.0, 2.618 / 512.0
        return (dxl_value - center) * scale

    def radians_to_dxl(self, radians):
        if USE_XL430:
            center, inv_scale = 2048.0, 2048.0 / 2.618
        else:
            center, inv_scale = 512.0, 512.0 / 2.618
        return int(radians * inv_scale + center)

    def degrees_to_dxl(self, degrees):
        """Convierte grados a valor Dynamixel"""
        radians = np.radians(degrees)
        return self.radians_to_dxl(radians)

    def initialize_motors(self, goal_positions, moving_speed, torque_limit):
        for dxl_id, goal in zip(self.dxl_ids, goal_positions):
            try:
                result, error = self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                if result != 0:
                    self.get_logger().error(f'Error habilitando torque en motor {dxl_id}: {error}')
                    continue
                
                self.update_speed_single_motor(dxl_id, moving_speed)
                write_goal_position(self.packet, self.port, dxl_id, goal)
                
                joint_index = self.dxl_ids.index(dxl_id)
                angle = self.dxl_to_radians(goal)
                angle *= self.joint_sign.get(dxl_id, 1)
                self.current_joint_positions[joint_index] = angle
                
                self.get_logger().info(f'Motor {dxl_id} configurado correctamente')
            except Exception as e:
                self.get_logger().error(f'Error configurando motor {dxl_id}: {str(e)}')

    def publish_joint_states(self):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header
        
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "base_link"
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        self.joint_state_pub.publish(joint_state)

    def move_motor(self, motor_id, position):
        if self.emergency_stop_activated:
            self.get_logger().warning(f'No se puede mover motor {motor_id}: Parada de emergencia activada')
            return False
            
        try:
            result, error = write_goal_position(self.packet, self.port, motor_id, position)
            if result == 0:
                self.get_logger().info(f'[Motor {motor_id}] Moviendo a {position}')
                joint_index = self.dxl_ids.index(motor_id)
                angle = self.dxl_to_radians(position)
                angle *= self.joint_sign.get(motor_id, 1)
                self.current_joint_positions[joint_index] = angle
                
                # Notificar cambio de posición a GUI
                if self.position_changed:
                    self.position_changed.emit(motor_id, position)
                
                return True
            else:
                self.get_logger().error(f'Error moviendo motor {motor_id}: {error}')
                return False
        except Exception as e:
            self.get_logger().error(f'Excepción moviendo motor {motor_id}: {str(e)}')
            return False

    def move_to_angles_degrees(self, angles_deg):
        """Mueve el robot a ángulos especificados en grados"""
        if len(angles_deg) != 5:
            self.get_logger().error(f'Se esperaban 5 ángulos, se recibieron {len(angles_deg)}')
            return False
        
        success = True
        for i, motor_id in enumerate(self.dxl_ids):
            sign = self.joint_sign.get(motor_id, 1)
            angle_deg = angles_deg[i] * sign
            goal_dxl = self.degrees_to_dxl(angle_deg)
            goal_dxl = int(np.clip(goal_dxl, 0, DXL_MAX_VALUE))
            
            if not self.move_motor(motor_id, goal_dxl):
                success = False
        
        return success

    def move_to_joint_angles(self, q_rad):
        if len(q_rad) != 4:
            self.get_logger().error(f'Se esperaban 4 ángulos, se recibieron {len(q_rad)}')
            return False
        
        q_min, q_max = -2.618, 2.618
        success = True
        
        for i, motor_id in enumerate(self.dxl_ids[:4]):
            sign = self.joint_sign.get(motor_id, 1)
            motor_angle = q_rad[i] * sign
            motor_angle_clamped = float(np.clip(motor_angle, q_min, q_max))
            
            if motor_angle != motor_angle_clamped:
                self.get_logger().warning(
                    f'Joint {i+1} saturado: {np.degrees(motor_angle):.1f}° '
                    f'-> {np.degrees(motor_angle_clamped):.1f}°'
                )
            
            goal_dxl = self.radians_to_dxl(motor_angle_clamped)
            if not self.move_motor(motor_id, goal_dxl):
                success = False
        
        return success

    def move_to_xyz(self, x, y, z, orientation='down'):
        try:
            r = np.hypot(x, y)
            
            if r < self.planar_reach_min or r > self.planar_reach_max + 0.01:
                self.get_logger().error(f'Objetivo fuera del alcance radial: r={r:.3f} m')
                return False
            
            if z < self.z_min - 0.01 or z > self.z_max + 0.01:
                self.get_logger().error(f'Objetivo fuera del alcance en Z: z={z:.3f} m')
                return False
            
            if orientation == 'down':
                T_target = SE3(x, y, z) * SE3.Rx(np.pi)
            else:
                T_target = SE3(x, y, z)
            
            q_current = np.array(self.current_joint_positions[:4])
            
            seeds = [
                q_current,
                np.array([0, 0, 0, 0]),
                np.array([0, 0.5, -0.5, 0]),
                np.array([0, 1.0, -1.0, 0]),
            ]
            
            best_sol = None
            best_error = float('inf')
            
            for seed in seeds:
                sol = self.robot_model.ikine_LM(
                    T_target, q0=seed, ilimit=1000, slimit=100,
                    mask=[1, 1, 1, 0, 0, 0]
                )
                
                if sol.success:
                    T_check = self.robot_model.fkine(sol.q)
                    error = np.linalg.norm(T_check.t - T_target.t)
                    
                    if error < best_error:
                        best_error = error
                        best_sol = sol
                    
                    if error < 0.003:
                        break
            
            if best_sol is None:
                self.get_logger().error(f'No se encontró solución IK')
                return False
            
            return self.move_to_joint_angles(best_sol.q)
                
        except Exception as e:
            self.get_logger().error(f'Error en cinemática inversa: {str(e)}')
            return False

    def get_current_xyz(self):
        try:
            q_current = np.array(self.current_joint_positions[:4])
            T = self.robot_model.fkine(q_current)
            return (T.t[0], T.t[1], T.t[2])
        except Exception as e:
            self.get_logger().error(f'Error calculando cinemática directa: {str(e)}')
            return (0, 0, 0)

    def update_speed_single_motor(self, motor_id, speed):
        try:
            result, error = write_moving_speed(self.packet, self.port, motor_id, speed)
            return result == 0
        except Exception as e:
            self.get_logger().error(f'Error actualizando velocidad motor {motor_id}: {str(e)}')
            return False

    def update_speed(self, speed):
        if self.emergency_stop_activated:
            self.get_logger().warning('No se puede actualizar velocidad: Parada de emergencia activada')
            return
        
        success_count = 0
        for motor_id in self.dxl_ids:
            if self.update_speed_single_motor(motor_id, speed):
                success_count += 1

    def home_all_motors(self):
        """Mueve todos los motores a 0 grados"""
        if self.emergency_stop_activated:
            self.reactivate_torque()
        
        # Mover todos a 0 grados
        self.move_to_angles_degrees([0, 0, 0, 0, 0])
        self.get_logger().info('Todos los motores movidos a 0°')

    def emergency_stop(self):
        self.emergency_stop_activated = True
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.get_logger().warning(f'Torque desactivado en motor {dxl_id}')
            except Exception as e:
                self.get_logger().error(f'Error en parada de emergencia motor {dxl_id}: {str(e)}')

    def reactivate_torque(self):
        self.emergency_stop_activated = False
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                self.get_logger().info(f'Torque reactivado en motor {dxl_id}')
            except Exception as e:
                self.get_logger().error(f'Error reactivando torque en motor {dxl_id}: {str(e)}')
#######################################################
    # ============================================================
    #  NUEVO: Publicar dato OpenCV por tópicos
    # ============================================================
    def publish_opencv_measurement(self, shape_code: str, x_m: float, y_m: float, z_m: float):
        msg_shape = String()
        msg_shape.data = shape_code
        self.opencv_shape_pub.publish(msg_shape)

        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "disk_frame"  # <-- si luego defines otro frame, cámbialo
        pt.point.x = float(x_m)
        pt.point.y = float(y_m)
        pt.point.z = float(z_m)
        self.opencv_point_pub.publish(pt)

        self.last_opencv_measurement = (shape_code, x_m, y_m, z_m)

    # ============================================================
    #  NUEVO: Gripper (placeholder, calibrar)
    # ============================================================
    def set_gripper_degrees(self, deg: float):
        """
        Mueve solo el motor 5 (gripper) a un ángulo en grados.
        Ajusta GRIPPER_OPEN_DEG / GRIPPER_CLOSE_DEG arriba.
        """
        if 5 not in self.dxl_ids:
            self.get_logger().warning("No se encontró motor 5 en dxl_ids")
            return False
        motor_id = 5
        sign = self.joint_sign.get(motor_id, 1)
        goal_dxl = self.degrees_to_dxl(deg * sign)
        goal_dxl = int(np.clip(goal_dxl, 0, DXL_MAX_VALUE))
        return self.move_motor(motor_id, goal_dxl)

    def gripper_open(self):
        return self.set_gripper_degrees(GRIPPER_OPEN_DEG)

    def gripper_close(self):
        return self.set_gripper_degrees(GRIPPER_CLOSE_DEG)

    # ============================================================
    #  NUEVO: Rutina pick & place usando dato guardado
    # ============================================================
    def run_pick_place(self, shape_code: str, x_m: float, y_m: float, z_pick_m: float):
        """
        Rutina bloqueante (úsala en un thread desde la GUI).
        - NO vuelve a leer cámara.
        - Usa coordenadas ya guardadas.
        """
        if shape_code not in DROP_POINTS:
            self.get_logger().error(f"Figura '{shape_code}' no tiene DROP_POINT configurado.")
            return False

        drop_x, drop_y, drop_z = DROP_POINTS[shape_code]

        # 1) Aproximar arriba del pick
        if not self.move_to_xyz(x_m, y_m, Z_APPROACH_M):
            return False
        time.sleep(MOVE_WAIT_S)

        # 2) Bajar a Z_PICK (por ahora 0, cambia Z_PICK_M arriba)
        if not self.move_to_xyz(x_m, y_m, z_pick_m):
            return False
        time.sleep(MOVE_WAIT_S)

        # 3) Cerrar gripper (placeholder)
        self.gripper_close()
        time.sleep(MOVE_WAIT_S)

        # 4) Subir
        if not self.move_to_xyz(x_m, y_m, Z_APPROACH_M):
            return False
        time.sleep(MOVE_WAIT_S)

        # 5) Ir arriba del drop
        if not self.move_to_xyz(drop_x, drop_y, max(drop_z, Z_APPROACH_M)):
            return False
        time.sleep(MOVE_WAIT_S)

        # 6) Bajar a drop_z
        if not self.move_to_xyz(drop_x, drop_y, drop_z):
            return False
        time.sleep(MOVE_WAIT_S)

        # 7) Abrir gripper
        self.gripper_open()
        time.sleep(MOVE_WAIT_S)

        # 8) Retirar (subir)
        self.move_to_xyz(drop_x, drop_y, max(drop_z, Z_APPROACH_M))
        time.sleep(MOVE_WAIT_S)

        return True

######################################################

    def close(self):
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            except:
                pass
        self.port.closePort()

# ============================================================
#  INTERFAZ MODERNA CON MENÚ LATERAL
# ============================================================

class PositionSignal(QWidget):
    """Widget para emitir señales de posición"""
    changed = pyqtSignal(int, int)  # motor_id, position

class ModernPincherGUI(QMainWindow):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.rviz_process = None
        self.toolbox_process = None
        self.last_motor_update = {motor_id: 0 for motor_id in controller.dxl_ids}
        self.update_interval = 0.05
        
        # Crear señal para actualización de posición
        self.position_signal = PositionSignal()
        self.position_signal.changed.connect(self.on_position_changed)
        self.controller.position_changed = self.position_signal.changed
        
        self.init_ui()

        ##########################################################
        # =========================
        #  NUEVO: Cámara para pestaña OpenCV
        # =========================
        self.cap = cv2.VideoCapture(OPENCV_CAMERA_INDEX, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            # fallback
            self.cap = cv2.VideoCapture(OPENCV_CAMERA_INDEX)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        self.last_frame = None
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.update_camera_frame)
        # OJO: no la arrancamos aquí. La arrancamos solo cuando entras a la pestaña OpenCV.

        ##########################################################
        
        # Timers
        self.xyz_timer = QTimer()
        self.xyz_timer.timeout.connect(self.update_xyz_display)
        self.xyz_timer.start(200)
        
    def init_ui(self):
        self.setWindowTitle("PhantomX Pincher Control Studio")
        self.setGeometry(100, 50, 1400, 800)
        
        central = QWidget()
        central.setObjectName("centralWidget")
        self.setCentralWidget(central)
        
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # SIDEBAR
        sidebar = self.create_sidebar()
        main_layout.addWidget(sidebar)
        
        # CONTENT AREA
        content_container = QWidget()
        content_container.setObjectName("contentArea")
        content_layout = QVBoxLayout(content_container)
        content_layout.setContentsMargins(25, 25, 25, 25)
        
        self.stack = QStackedWidget()
        content_layout.addWidget(self.stack)
        
        self.create_pages()
        
        # Status bar
        status_frame = QFrame()
        status_layout = QHBoxLayout(status_frame)
        
        status_layout.addStretch()
        
        self.status_label = QLabel("● Sistema Listo")
        self.status_label.setObjectName("statusLabel")
        status_layout.addWidget(self.status_label)
        
        status_layout.addStretch()
        
        content_layout.addWidget(status_frame)
        
        main_layout.addWidget(content_container, 1)
        
    def create_sidebar(self):
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(280)
        
        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(0, 20, 0, 20)
        layout.setSpacing(5)
        
        # Logo/Title
        title = QLabel("PINCHER X100")
        title.setStyleSheet("color: #00d9ff; font-size: 20pt; font-weight: bold; padding: 20px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        subtitle = QLabel("Control Studio")
        subtitle.setStyleSheet("color: #b0b0c0; font-size: 11pt; padding-bottom: 20px;")
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)
        
        # HOME BUTTON en sidebar
        home_btn = QPushButton("🏠 HOME (0°)")
        home_btn.setObjectName("homeButtonSidebar")
        home_btn.clicked.connect(self.home_all)
        layout.addWidget(home_btn)
        
        # EMERGENCY BUTTON en sidebar
        emergency_btn = QPushButton("🛑 EMERGENCIA")
        emergency_btn.setObjectName("emergencyButtonSidebar")
        emergency_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(emergency_btn)
        
        # Separador
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet("background-color: #3a3a4e; margin: 10px;")
        layout.addWidget(separator)
        
        # Menu buttons
        self.menu_buttons = []
        
        menus = [
            ("📊 Panel Principal", 0),
            ("🎚️ Control Manual", 1),
            ("📝 Valores Fijos", 2),
            ("📐 Ángulos Predef.", 3),
            ("🎯 Control XYZ", 4),
            ("📷 OpenCV", 5),            # <-- NUEVO
            ("👁️ Visualización", 6),
            ("ℹ️ Información", 7),
        ]

        
        for text, index in menus:
            btn = QPushButton(text)
            btn.setObjectName("menuButton")
            btn.setCheckable(True)
            btn.setFixedHeight(55)
            btn.clicked.connect(lambda checked, i=index: self.change_page(i))
            layout.addWidget(btn)
            self.menu_buttons.append(btn)
        
        self.menu_buttons[0].setChecked(True)
        
        layout.addStretch()
        
        footer = QLabel("v2.0 • 2024")
        footer.setStyleSheet("color: #6a6a7e; font-size: 9pt; padding: 20px;")
        footer.setAlignment(Qt.AlignCenter)
        layout.addWidget(footer)
        
        return sidebar
    
    def create_pages(self):
        self.stack.addWidget(self.create_dashboard_page())
        self.stack.addWidget(self.create_manual_control_page())
        self.stack.addWidget(self.create_fixed_values_page())
        self.stack.addWidget(self.create_fixed_angles_page())
        self.stack.addWidget(self.create_xyz_page())
        self.stack.addWidget(self.create_opencv_page())            # <-- NUEVO
        self.stack.addWidget(self.create_visualization_page())
        self.stack.addWidget(self.create_info_page())

    
    def change_page(self, index):
        self.stack.setCurrentIndex(index)
        for i, btn in enumerate(self.menu_buttons):
            btn.setChecked(i == index)

        # NUEVO: cámara solo en la pestaña OpenCV (index 5)
        if index == 5:
            if not self.camera_timer.isActive():
                self.camera_timer.start(33)  # ~30 FPS
        else:
            if self.camera_timer.isActive():
                self.camera_timer.stop()

    
    def create_dashboard_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Panel de Control Principal")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        subtitle = QLabel("Vista general del estado del robot")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)
        
        cards_layout = QGridLayout()
        cards_layout.setSpacing(20)
        
        pos_card = self.create_card("Posición Actual", self.create_position_widget())
        cards_layout.addWidget(pos_card, 0, 0)
        
        motors_card = self.create_card("Estado de Motores", self.create_motors_status_widget())
        cards_layout.addWidget(motors_card, 0, 1)
        
        speed_card = self.create_card("Control de Velocidad", self.create_speed_widget())
        cards_layout.addWidget(speed_card, 1, 0, 1, 2)
        
        layout.addLayout(cards_layout)
        layout.addStretch()
        
        return page
    
    def create_manual_control_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Control Manual por Sliders")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        self.sliders = {}
        self.slider_labels = {}
        
        for motor_id in self.controller.dxl_ids:
            motor_card = QFrame()
            motor_card.setObjectName("card")
            motor_layout = QVBoxLayout(motor_card)
            
            motor_title = QLabel(f"Motor {motor_id}")
            motor_title.setObjectName("motorLabel")
            motor_layout.addWidget(motor_title)
            
            slider_layout = QHBoxLayout()
            
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(DXL_MAX_VALUE)
            slider.setValue(DEFAULT_GOAL)
            slider.valueChanged.connect(lambda v, mid=motor_id: self.on_motor_slider_change(mid))
            slider_layout.addWidget(slider, 3)
            
            value_label = QLabel(f"{DEFAULT_GOAL}")
            value_label.setObjectName("valueLabel")
            value_label.setAlignment(Qt.AlignCenter)
            slider_layout.addWidget(value_label)
            
            motor_layout.addLayout(slider_layout)
            
            scroll_layout.addWidget(motor_card)
            
            self.sliders[motor_id] = slider
            self.slider_labels[motor_id] = value_label
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)
        
        return page
    
    def create_fixed_values_page(self):
        """Pestaña para mover a valores fijos ingresados manualmente"""
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Mover a Valores Fijos")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        subtitle = QLabel("Ingresa valores específicos para cada motor")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        self.value_entries = {}
        self.value_entry_labels = {}
        
        for motor_id in self.controller.dxl_ids:
            motor_card = QFrame()
            motor_card.setObjectName("card")
            motor_layout = QHBoxLayout(motor_card)
            
            motor_title = QLabel(f"Motor {motor_id}")
            motor_title.setObjectName("motorLabel")
            motor_title.setMinimumWidth(100)
            motor_layout.addWidget(motor_title)
            
            entry_label = QLabel(f"Valor (0-{DXL_MAX_VALUE}):")
            motor_layout.addWidget(entry_label)
            
            entry = QLineEdit(str(DEFAULT_GOAL))
            entry.setMaximumWidth(120)
            entry.returnPressed.connect(lambda mid=motor_id: self.move_single_motor_from_entry(mid))
            motor_layout.addWidget(entry)
            
            move_btn = QPushButton("Mover")
            move_btn.clicked.connect(lambda checked, mid=motor_id: self.move_single_motor_from_entry(mid))
            motor_layout.addWidget(move_btn)
            
            status_label = QLabel("Listo")
            status_label.setStyleSheet("color: #00d9ff; font-weight: bold;")
            motor_layout.addWidget(status_label)
            
            motor_layout.addStretch()
            
            scroll_layout.addWidget(motor_card)
            
            self.value_entries[motor_id] = entry
            self.value_entry_labels[motor_id] = status_label
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)
        
        # Botón mover todos
        move_all_btn = QPushButton("▶ MOVER TODOS LOS MOTORES")
        move_all_btn.setStyleSheet("font-size: 13pt; padding: 15px;")
        move_all_btn.clicked.connect(self.move_all_motors_from_entries)
        layout.addWidget(move_all_btn)
        
        return page
    
    def create_fixed_angles_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Movimientos a Ángulos Predefinidos")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        subtitle = QLabel("Posiciones predefinidas con un solo clic")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)
        
        presets = [
            ("Posición 1: HOME", [0, 0, 0, 0, 0], "Todos los motores en 0°"),
            ("Posición 2: Alcance Medio", [25, 25, 20, -20, 0], "Configuración de alcance medio"),
            ("Posición 3: Lateral", [-35, 35, -30, 30, 0], "Movimiento lateral"),
            ("Posición 4: Elevada", [85, -20, 55, 25, 0], "Posición elevada"),
            ("Posición 5: Extendida", [80, -35, 55, -45, 0], "Máxima extensión"),
        ]
        
        grid = QGridLayout()
        grid.setSpacing(20)
        
        for i, (name, angles, desc) in enumerate(presets):
            card = QFrame()
            card.setObjectName("card")
            card_layout = QVBoxLayout(card)
            
            title_label = QLabel(name)
            title_label.setObjectName("sectionTitle")
            card_layout.addWidget(title_label)
            
            desc_label = QLabel(desc)
            desc_label.setStyleSheet("color: #b0b0c0; padding: 5px;")
            desc_label.setWordWrap(True)
            card_layout.addWidget(desc_label)
            
            angles_str = f"[{', '.join(f'{a}°' for a in angles)}]"
            angles_label = QLabel(angles_str)
            angles_label.setStyleSheet("color: #00d9ff; font-weight: bold; padding: 10px; font-size: 12pt;")
            card_layout.addWidget(angles_label)
            
            btn = QPushButton("▶ Ejecutar Movimiento")
            btn.setObjectName("presetButton")
            btn.clicked.connect(lambda checked, a=angles: self.move_to_fixed_angles(a))
            card_layout.addWidget(btn)
            
            row = i // 2
            col = i % 2
            grid.addWidget(card, row, col)
        
        layout.addLayout(grid)
        layout.addStretch()
        
        return page
    
    def create_xyz_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Control por Posición XYZ")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        current_card = QFrame()
        current_card.setObjectName("card")
        current_layout = QVBoxLayout(current_card)
        
        current_title = QLabel("Posición Actual del End-Effector")
        current_title.setObjectName("sectionTitle")
        current_layout.addWidget(current_title)
        
        pos_layout = QHBoxLayout()
        self.current_x_label = QLabel("X: 0.000 m")
        self.current_x_label.setStyleSheet("font-size: 14pt; color: #00d9ff; font-weight: bold;")
        pos_layout.addWidget(self.current_x_label)
        
        self.current_y_label = QLabel("Y: 0.000 m")
        self.current_y_label.setStyleSheet("font-size: 14pt; color: #00d9ff; font-weight: bold;")
        pos_layout.addWidget(self.current_y_label)
        
        self.current_z_label = QLabel("Z: 0.000 m")
        self.current_z_label.setStyleSheet("font-size: 14pt; color: #00d9ff; font-weight: bold;")
        pos_layout.addWidget(self.current_z_label)
        
        current_layout.addLayout(pos_layout)
        layout.addWidget(current_card)
        
        target_card = QFrame()
        target_card.setObjectName("card")
        target_layout = QVBoxLayout(target_card)
        
        target_title = QLabel("Posición Objetivo")
        target_title.setObjectName("sectionTitle")
        target_layout.addWidget(target_title)
        
        inputs_layout = QHBoxLayout()
        
        for label_text, default_val, attr_name in [("X (m)", "0.200", "x_entry"),
                                                     ("Y (m)", "0.000", "y_entry"),
                                                     ("Z (m)", "0.100", "z_entry")]:
            v_layout = QVBoxLayout()
            lbl = QLabel(label_text)
            lbl.setStyleSheet("font-weight: bold; color: #00d9ff;")
            v_layout.addWidget(lbl)
            
            entry = QLineEdit(default_val)
            entry.setFixedWidth(120)
            v_layout.addWidget(entry)
            
            inputs_layout.addLayout(v_layout)
            setattr(self, attr_name, entry)
        
        target_layout.addLayout(inputs_layout)
        
        move_btn = QPushButton("🎯 MOVER A POSICIÓN XYZ")
        move_btn.setFixedHeight(50)
        move_btn.setStyleSheet("font-size: 13pt;")
        move_btn.clicked.connect(self.move_to_xyz_target)
        target_layout.addWidget(move_btn)
        
        layout.addWidget(target_card)
        
        presets_card = QFrame()
        presets_card.setObjectName("card")
        presets_layout = QVBoxLayout(presets_card)
        
        presets_title = QLabel("Posiciones Rápidas")
        presets_title.setObjectName("sectionTitle")
        presets_layout.addWidget(presets_title)
        
        presets_grid = QGridLayout()
        presets = [
            ("Home", 0.20, 0.00, 0.10),
            ("Alto", 0.15, 0.00, 0.20),
            ("Derecha", 0.18, 0.08, 0.05),
            ("Izquierda", 0.18, -0.08, 0.05),
        ]
        
        for i, (name, x, y, z) in enumerate(presets):
            btn = QPushButton(f"{name}\n({x:.2f}, {y:.2f}, {z:.2f})")
            btn.setFixedHeight(70)
            btn.clicked.connect(lambda checked, px=x, py=y, pz=z: self.move_to_preset_xyz(px, py, pz))
            presets_grid.addWidget(btn, 0, i)
        
        presets_layout.addLayout(presets_grid)
        layout.addWidget(presets_card)
        
        self.ik_status_label = QLabel("✓ Listo para mover")
        self.ik_status_label.setStyleSheet("color: #00d9ff; font-size: 12pt; padding: 10px;")
        layout.addWidget(self.ik_status_label)
        
        layout.addStretch()
        
        return page
####################
    # ============================================================
    #  NUEVO: Página OpenCV
    # ============================================================
    def create_opencv_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        title = QLabel("OpenCV")
        title.setObjectName("pageTitle")
        layout.addWidget(title)

        subtitle = QLabel("Vista de cámara + captura 1-shot + rutina por figura")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)

        # Video card
        video_card = QFrame()
        video_card.setObjectName("card")
        video_layout = QVBoxLayout(video_card)

        self.camera_label = QLabel("Cámara no iniciada")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumHeight(360)
        self.camera_label.setStyleSheet("background-color: #111; border-radius: 10px;")
        video_layout.addWidget(self.camera_label)

        layout.addWidget(video_card)

        # Info card
        info_card = QFrame()
        info_card.setObjectName("card")
        info_layout = QHBoxLayout(info_card)

        self.opencv_shape_label = QLabel("Figura: -")
        self.opencv_shape_label.setStyleSheet("color: #00d9ff; font-weight: bold; font-size: 14pt;")
        info_layout.addWidget(self.opencv_shape_label)

        self.opencv_xy_label = QLabel("x,y: -")
        self.opencv_xy_label.setStyleSheet("color: #00d9ff; font-weight: bold; font-size: 14pt;")
        info_layout.addWidget(self.opencv_xy_label)

        self.opencv_conf_label = QLabel("conf: -")
        self.opencv_conf_label.setStyleSheet("color: #b0b0c0; font-weight: bold; font-size: 12pt;")
        info_layout.addWidget(self.opencv_conf_label)

        info_layout.addStretch()
        layout.addWidget(info_card)

        # Buttons
        btn_row = QHBoxLayout()

        self.btn_home_opencv = QPushButton("🏠 Home OpenCV")
        self.btn_home_opencv.clicked.connect(self.home_opencv)
        btn_row.addWidget(self.btn_home_opencv)

        self.btn_capture_opencv = QPushButton("📌 Capturar dato OpenCV (1 lectura)")
        self.btn_capture_opencv.clicked.connect(self.capture_opencv_data_once)
        btn_row.addWidget(self.btn_capture_opencv)

        self.btn_run_routine = QPushButton("🤖 Ejecutar rutina (usa dato guardado)")
        self.btn_run_routine.clicked.connect(self.run_saved_routine)
        btn_row.addWidget(self.btn_run_routine)

        layout.addLayout(btn_row)
        layout.addStretch()
        return page

    def update_camera_frame(self):
        try:
            if not self.cap or not self.cap.isOpened():
                self.camera_label.setText("❌ No se pudo abrir la cámara")
                return

            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.camera_label.setText("⚠ No llegó frame (ret=False)")
                return

            self.last_frame = frame.copy()

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pix = QPixmap.fromImage(qimg)

            self.camera_label.setPixmap(
                pix.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        except Exception as e:
            self.camera_label.setText(f"❌ Error cámara: {e}")

    def home_opencv(self):
        """
        Home OpenCV.
        Por ahora usa el home general.
        """
        # =========================
        # CAMBIAR AQUÍ si quieres un HOME específico de OpenCV
        # =========================
        # Ejemplo futuro:
        # self.controller.move_to_angles_degrees(HOME_OPENCV_ANGLES_DEG)
        self.controller.home_all_motors()
        self.status_label.setText("● HOME OpenCV (por ahora: HOME general)")
        QTimer.singleShot(2500, lambda: self.status_label.setText("● Sistema Listo"))

    def capture_opencv_data_once(self):
        """
        1-shot: corre OpenCV UNA VEZ usando el último frame mostrado.
        Guarda shape_code + (x,y) cartesiano con offset y publica por tópicos.
        """
        if self.last_frame is None:
            QMessageBox.warning(self, "OpenCV", "No hay frame disponible todavía.")
            return

        info = detectar_figuras_y_polares(self.last_frame, debug=False)
        best = pick_best_detection(info)

        if best is None:
            self.opencv_shape_label.setText("Figura: u")
            self.opencv_xy_label.setText("x,y: -")
            self.opencv_conf_label.setText("conf: -")
            QMessageBox.warning(self, "OpenCV", "No se detectó ninguna figura.")
            return

        shape_name = best["shape_name"]
        conf = best["confidence"]
        r_cm = best["r_cm"]
        theta_deg = best["theta_deg"]

        shape_code = shape_name_to_code(shape_name)

        # =========================
        # CAMBIAR AQUÍ si quieres otro mapeo/transformación
        # =========================
        x_m, y_m = polar_to_cartesian_m(
            r_cm, theta_deg,
            offset_x_m=OPENCV_OFFSET_X_M,  # <-- offset X
            offset_y_m=OPENCV_OFFSET_Y_M,  # <-- offset Y
        )

        z_m = Z_PICK_M  # <-- Z fijo para pick (por ahora 0)

        # Guardar + publicar por tópicos DESDE el mismo nodo (PincherController)
        self.controller.publish_opencv_measurement(shape_code, x_m, y_m, z_m)

        # UI
        self.opencv_shape_label.setText(f"Figura: {shape_code} ({shape_name})")
        self.opencv_xy_label.setText(f"x,y: {x_m:.3f}, {y_m:.3f} m")
        self.opencv_conf_label.setText(f"conf: {conf:.0%}")

        self.status_label.setText("● Dato OpenCV capturado y publicado")
        QTimer.singleShot(2500, lambda: self.status_label.setText("● Sistema Listo"))

    def run_saved_routine(self):
        """
        Ejecuta rutina usando la última medición guardada en controller.
        NO vuelve a leer la cámara.
        """
        if self.controller.last_opencv_measurement is None:
            QMessageBox.warning(self, "Rutina", "Primero captura el dato con 'Capturar dato OpenCV'.")
            return

        shape_code, x_m, y_m, z_m = self.controller.last_opencv_measurement

        if shape_code not in DROP_POINTS:
            QMessageBox.warning(self, "Rutina", f"No hay punto DROP configurado para '{shape_code}'.")
            return

        # Ejecutar en thread para no congelar GUI
        self.status_label.setText("● Ejecutando rutina...")
        self.btn_run_routine.setEnabled(False)

        def worker():
            ok = self.controller.run_pick_place(shape_code, x_m, y_m, z_m)
            QTimer.singleShot(0, lambda: self.on_routine_finished(ok))

        threading.Thread(target=worker, daemon=True).start()

    def on_routine_finished(self, ok: bool):
        self.btn_run_routine.setEnabled(True)
        if ok:
            self.status_label.setText("● Rutina completada")
        else:
            self.status_label.setText("● Rutina falló")
        QTimer.singleShot(3000, lambda: self.status_label.setText("● Sistema Listo"))

####################
    def create_visualization_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Visualización")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        # RViz
        rviz_card = QFrame()
        rviz_card.setObjectName("card")
        rviz_layout = QVBoxLayout(rviz_card)
        
        rviz_title = QLabel("RViz - Modelo 3D")
        rviz_title.setObjectName("sectionTitle")
        rviz_layout.addWidget(rviz_title)
        
        self.rviz_btn = QPushButton("▶ LANZAR RViz")
        self.rviz_btn.setFixedHeight(60)
        self.rviz_btn.clicked.connect(self.launch_rviz)
        rviz_layout.addWidget(self.rviz_btn)
        
        self.stop_rviz_btn = QPushButton("⏹ DETENER RViz")
        self.stop_rviz_btn.setEnabled(False)
        self.stop_rviz_btn.setFixedHeight(60)
        self.stop_rviz_btn.clicked.connect(self.stop_rviz)
        rviz_layout.addWidget(self.stop_rviz_btn)
        
        self.rviz_status_label = QLabel("● RViz no iniciado")
        self.rviz_status_label.setStyleSheet("color: #ff416c; font-weight: bold; padding: 15px; font-size: 12pt;")
        rviz_layout.addWidget(self.rviz_status_label)
        
        layout.addWidget(rviz_card)
        
        # Toolbox
        toolbox_card = QFrame()
        toolbox_card.setObjectName("card")
        toolbox_layout = QVBoxLayout(toolbox_card)
        
        toolbox_title = QLabel("Robotics Toolbox - Simulación")
        toolbox_title.setObjectName("sectionTitle")
        toolbox_layout.addWidget(toolbox_title)
        
        self.toolbox_btn = QPushButton("▶ LANZAR Toolbox")
        self.toolbox_btn.setFixedHeight(60)
        self.toolbox_btn.clicked.connect(self.launch_toolbox)
        toolbox_layout.addWidget(self.toolbox_btn)
        
        self.stop_toolbox_btn = QPushButton("⏹ DETENER Toolbox")
        self.stop_toolbox_btn.setEnabled(False)
        self.stop_toolbox_btn.setFixedHeight(60)
        self.stop_toolbox_btn.clicked.connect(self.stop_toolbox)
        toolbox_layout.addWidget(self.stop_toolbox_btn)
        
        self.toolbox_status_label = QLabel("● Toolbox no iniciado")
        self.toolbox_status_label.setStyleSheet("color: #ff416c; font-weight: bold; padding: 15px; font-size: 12pt;")
        toolbox_layout.addWidget(self.toolbox_status_label)
        
        layout.addWidget(toolbox_card)
        
        layout.addStretch()
        
        return page
    
    def create_info_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Información del Proyecto")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        card1 = QFrame()
        card1.setObjectName("card")
        layout1 = QVBoxLayout(card1)
        
        name1 = QLabel("Samuel David Sanchez Cardenas")
        name1.setObjectName("sectionTitle")
        layout1.addWidget(name1)
        
        info1 = QLabel("📚 Ingeniería Mecatrónica\n☕ Café Lover\n🤖 Solo Robótica\n⏰ Veremos si acaba primero el semestre o el semestre acaba conmigo\n\n🔗 GitHub: samsanchezcar")
        info1.setStyleSheet("color: #e0e0e0; line-height: 1.6; padding: 10px;")
        layout1.addWidget(info1)
        
        scroll_layout.addWidget(card1)
        
        card2 = QFrame()
        card2.setObjectName("card")
        layout2 = QVBoxLayout(card2)
        
        name2 = QLabel("Santiago Ávila Corredor")
        name2.setObjectName("sectionTitle")
        layout2.addWidget(name2)
        
        info2 = QLabel("😵 El semestre me está matando\n🙏 Pronto todo acabará\n\n🔗 GitHub: Santiago-Avila")
        info2.setStyleSheet("color: #e0e0e0; line-height: 1.6; padding: 10px;")
        layout2.addWidget(info2)
        
        scroll_layout.addWidget(card2)
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)
        
        return page
    
    def create_card(self, title, content_widget):
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        
        title_label = QLabel(title)
        title_label.setObjectName("sectionTitle")
        layout.addWidget(title_label)
        
        layout.addWidget(content_widget)
        
        return card
    
    def create_position_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        self.dash_x_label = QLabel("X: 0.000 m")
        self.dash_x_label.setStyleSheet("font-size: 13pt; color: #00d9ff; font-weight: bold;")
        layout.addWidget(self.dash_x_label)
        
        self.dash_y_label = QLabel("Y: 0.000 m")
        self.dash_y_label.setStyleSheet("font-size: 13pt; color: #00d9ff; font-weight: bold;")
        layout.addWidget(self.dash_y_label)
        
        self.dash_z_label = QLabel("Z: 0.000 m")
        self.dash_z_label.setStyleSheet("font-size: 13pt; color: #00d9ff; font-weight: bold;")
        layout.addWidget(self.dash_z_label)
        
        return widget
    
    def create_motors_status_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        for i, motor_id in enumerate(self.controller.dxl_ids):
            motor_label = QLabel(f"Motor {motor_id}: ● Activo")
            motor_label.setStyleSheet("color: #00d9ff; padding: 5px;")
            layout.addWidget(motor_label)
        
        return widget
    
    def create_speed_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        slider_layout = QHBoxLayout()
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(MAX_SPEED)
        self.speed_slider.setValue(100)
        self.speed_slider.valueChanged.connect(self.on_speed_change)
        slider_layout.addWidget(self.speed_slider, 3)
        
        self.speed_value_label = QLabel("100")
        self.speed_value_label.setObjectName("valueLabel")
        self.speed_value_label.setStyleSheet("font-size: 16pt;")
        slider_layout.addWidget(self.speed_value_label)
        
        layout.addLayout(slider_layout)
        
        return widget
    
    # ============================================================
    #  MÉTODOS DE CONTROL
    # ============================================================
    
    def on_position_changed(self, motor_id, position):
        """Actualiza sliders cuando el motor se mueve desde otras pestañas"""
        if motor_id in self.sliders:
            # Bloquear señal temporalmente para evitar loop
            self.sliders[motor_id].blockSignals(True)
            self.sliders[motor_id].setValue(position)
            self.slider_labels[motor_id].setText(str(position))
            self.sliders[motor_id].blockSignals(False)
    
    def on_motor_slider_change(self, motor_id):
        current_time = time.time()
        if current_time - self.last_motor_update[motor_id] >= self.update_interval:
            position = self.sliders[motor_id].value()
            speed = self.speed_slider.value()
            
            self.slider_labels[motor_id].setText(str(position))
            
            if speed > 0 and not self.controller.emergency_stop_activated:
                self.controller.move_motor(motor_id, position)
                self.status_label.setText(f"● Motor {motor_id} → {position}")
                self.last_motor_update[motor_id] = current_time
                QTimer.singleShot(2000, lambda: self.status_label.setText("● Sistema Listo"))
    
    def on_speed_change(self, value):
        self.controller.update_speed(value)
        self.speed_value_label.setText(str(value))
    
    def move_single_motor_from_entry(self, motor_id):
        try:
            value = self.value_entries[motor_id].text()
            position = int(value)
            
            if 0 <= position <= DXL_MAX_VALUE:
                speed = self.speed_slider.value()
                
                if speed == 0:
                    self.value_entry_labels[motor_id].setText("Velocidad 0")
                    self.value_entry_labels[motor_id].setStyleSheet("color: orange; font-weight: bold;")
                elif self.controller.emergency_stop_activated:
                    self.value_entry_labels[motor_id].setText("EMERGENCIA")
                    self.value_entry_labels[motor_id].setStyleSheet("color: red; font-weight: bold;")
                else:
                    self.controller.move_motor(motor_id, position)
                    self.value_entry_labels[motor_id].setText("Enviado")
                    self.value_entry_labels[motor_id].setStyleSheet("color: blue; font-weight: bold;")
                    self.status_label.setText(f"● Motor {motor_id} → {position}")
                    QTimer.singleShot(2000, lambda: self.value_entry_labels[motor_id].setText("Listo"))
                    QTimer.singleShot(2000, lambda: self.value_entry_labels[motor_id].setStyleSheet("color: #00d9ff; font-weight: bold;"))
            else:
                self.value_entry_labels[motor_id].setText(f"Error: 0-{DXL_MAX_VALUE}")
                self.value_entry_labels[motor_id].setStyleSheet("color: red; font-weight: bold;")
        except ValueError:
            self.value_entry_labels[motor_id].setText("Error: Número")
            self.value_entry_labels[motor_id].setStyleSheet("color: red; font-weight: bold;")
    
    def move_all_motors_from_entries(self):
        speed = self.speed_slider.value()
        
        if speed == 0:
            self.status_label.setText("● Velocidad 0: Los motores no se moverán")
            return
        
        if self.controller.emergency_stop_activated:
            QMessageBox.warning(self, "Advertencia", "Sistema en parada de emergencia")
            return
        
        success_count = 0
        for motor_id in self.controller.dxl_ids:
            try:
                value = self.value_entries[motor_id].text()
                position = int(value)
                
                if 0 <= position <= DXL_MAX_VALUE:
                    self.controller.move_motor(motor_id, position)
                    self.value_entry_labels[motor_id].setText("Enviado")
                    self.value_entry_labels[motor_id].setStyleSheet("color: blue; font-weight: bold;")
                    success_count += 1
            except ValueError:
                self.value_entry_labels[motor_id].setText("Error")
                self.value_entry_labels[motor_id].setStyleSheet("color: red; font-weight: bold;")
        
        self.status_label.setText(f"● {success_count}/{len(self.controller.dxl_ids)} motores movidos")
    
    def move_to_fixed_angles(self, angles):
        if self.controller.emergency_stop_activated:
            QMessageBox.warning(self, "Advertencia", "Sistema en parada de emergencia")
            return
        
        success = self.controller.move_to_angles_degrees(angles)
        
        if success:
            self.status_label.setText(f"● Movido a: {angles}")
            QTimer.singleShot(3000, lambda: self.status_label.setText("● Sistema Listo"))
        else:
            QMessageBox.critical(self, "Error", "No se pudo completar el movimiento")
    
    def move_to_xyz_target(self):
        try:
            x = float(self.x_entry.text())
            y = float(self.y_entry.text())
            z = float(self.z_entry.text())
            
            r = (x**2 + y**2) ** 0.5
            
            if not (self.controller.planar_reach_min <= r <= self.controller.planar_reach_max + 0.01):
                QMessageBox.warning(self, "Error", f"Fuera de alcance radial: {r:.3f} m")
                return
            
            if not (self.controller.z_min - 0.01 <= z <= self.controller.z_max + 0.01):
                QMessageBox.warning(self, "Error", f"Fuera de alcance en Z: {z:.3f} m")
                return
            
            self.ik_status_label.setText(f"⏳ Calculando IK...")
            QApplication.processEvents()
            
            success = self.controller.move_to_xyz(x, y, z)
            
            if success:
                self.ik_status_label.setText(f"✓ Movido a ({x:.3f}, {y:.3f}, {z:.3f})")
                self.status_label.setText("● Movimiento XYZ completado")
            else:
                self.ik_status_label.setText("✗ No se encontró solución IK")
                QMessageBox.critical(self, "Error", "No se pudo alcanzar la posición")
        
        except ValueError:
            QMessageBox.warning(self, "Error", "Valores numéricos inválidos")
    
    def move_to_preset_xyz(self, x, y, z):
        self.x_entry.setText(f"{x:.3f}")
        self.y_entry.setText(f"{y:.3f}")
        self.z_entry.setText(f"{z:.3f}")
        self.move_to_xyz_target()
    
    def update_xyz_display(self):
        try:
            x, y, z = self.controller.get_current_xyz()
            
            if hasattr(self, 'dash_x_label'):
                self.dash_x_label.setText(f"X: {x:.3f} m")
                self.dash_y_label.setText(f"Y: {y:.3f} m")
                self.dash_z_label.setText(f"Z: {z:.3f} m")
            
            if hasattr(self, 'current_x_label'):
                self.current_x_label.setText(f"X: {x:.3f} m")
                self.current_y_label.setText(f"Y: {y:.3f} m")
                self.current_z_label.setText(f"Z: {z:.3f} m")
        except:
            pass
    
    def launch_rviz(self):
        try:
            cmd = ["ros2", "launch", "phantomx_pincher_description", "display.launch.py"]
            
            def run_rviz():
                self.rviz_process = subprocess.Popen(cmd)
                self.rviz_process.wait()
                QTimer.singleShot(0, self.on_rviz_closed)
            
            thread = threading.Thread(target=run_rviz, daemon=True)
            thread.start()
            
            self.rviz_btn.setEnabled(False)
            self.stop_rviz_btn.setEnabled(True)
            self.rviz_status_label.setText("● RViz ejecutándose")
            self.rviz_status_label.setStyleSheet("color: #00d9ff; font-weight: bold; padding: 15px; font-size: 12pt;")
        
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo lanzar RViz: {str(e)}")
    
    def stop_rviz(self):
        if self.rviz_process:
            try:
                self.rviz_process.terminate()
                self.rviz_process = None
            except:
                pass
        self.on_rviz_closed()
    
    def on_rviz_closed(self):
        self.rviz_btn.setEnabled(True)
        self.stop_rviz_btn.setEnabled(False)
        self.rviz_status_label.setText("● RViz no iniciado")
        self.rviz_status_label.setStyleSheet("color: #ff416c; font-weight: bold; padding: 15px; font-size: 12pt;")
    
    def launch_toolbox(self):
        try:
            # Buscar toolbox.py en el directorio actual
            toolbox_path = os.path.join(os.path.dirname(__file__), 'toolbox.py')
            
            if not os.path.exists(toolbox_path):
                QMessageBox.warning(self, "Error", f"No se encontró toolbox.py en:\n{toolbox_path}")
                return
            
            def run_toolbox():
                self.toolbox_process = subprocess.Popen(['python3', toolbox_path])
                self.toolbox_process.wait()
                QTimer.singleShot(0, self.on_toolbox_closed)
            
            thread = threading.Thread(target=run_toolbox, daemon=True)
            thread.start()
            
            self.toolbox_btn.setEnabled(False)
            self.stop_toolbox_btn.setEnabled(True)
            self.toolbox_status_label.setText("● Toolbox ejecutándose")
            self.toolbox_status_label.setStyleSheet("color: #00d9ff; font-weight: bold; padding: 15px; font-size: 12pt;")
        
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo lanzar Toolbox: {str(e)}")
    
    def stop_toolbox(self):
        if self.toolbox_process:
            try:
                self.toolbox_process.terminate()
                self.toolbox_process = None
            except:
                pass
        self.on_toolbox_closed()
    
    def on_toolbox_closed(self):
        self.toolbox_btn.setEnabled(True)
        self.stop_toolbox_btn.setEnabled(False)
        self.toolbox_status_label.setText("● Toolbox no iniciado")
        self.toolbox_status_label.setStyleSheet("color: #ff416c; font-weight: bold; padding: 15px; font-size: 12pt;")
    
    def home_all(self):
        """Mueve todos los motores a 0 grados"""
        if self.controller.emergency_stop_activated:
            reply = QMessageBox.question(self, "Reactivar Sistema",
                "¿Reactivar sistema y mover a HOME (0°)?",
                QMessageBox.Yes | QMessageBox.No)
            
            if reply == QMessageBox.No:
                return
        
        self.controller.home_all_motors()
        self.status_label.setText("● Todos los motores en HOME (0°)")
        QTimer.singleShot(3000, lambda: self.status_label.setText("● Sistema Listo"))
    
    def emergency_stop(self):
        self.controller.emergency_stop()
        self.status_label.setText("⚠ PARADA DE EMERGENCIA ACTIVADA")
        self.status_label.setStyleSheet("background-color: #ff416c; color: white; font-weight: bold; padding: 10px; border-radius: 8px;")
    
    def closeEvent(self, event):
        if self.rviz_process:
            self.stop_rviz()
        if self.toolbox_process:
            self.stop_toolbox()
        
        reply = QMessageBox.question(self, "Salir",
            "¿Cerrar la aplicación?\nSe desactivará el torque.",
            QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.controller.close()
            event.accept()
            rclpy.shutdown()
        else:
            event.ignore()
                # NUEVO: liberar cámara
        try:
            if hasattr(self, "camera_timer") and self.camera_timer.isActive():
                self.camera_timer.stop()
            if hasattr(self, "cap") and self.cap:
                self.cap.release()
        except:
            pass


# ============================================================
#  MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    
    controller = PincherController()
    
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(controller,),
        daemon=True
    )
    spin_thread.start()
    
    app = QApplication(sys.argv)
    app.setStyleSheet(MODERN_STYLESHEET)
    
    try:
        gui = ModernPincherGUI(controller)
        gui.show()
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()