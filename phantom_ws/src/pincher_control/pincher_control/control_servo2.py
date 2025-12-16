# ============================================================
#  IMPORTACIONES DE BIBLIOTECAS
# ============================================================

# ROS2 - Framework de robótica para comunicación entre nodos
import rclpy                          # Biblioteca principal de ROS2 para Python
from rclpy.node import Node           # Clase base para crear nodos ROS2

# Dynamixel SDK - Biblioteca para controlar servomotores Dynamixel
from dynamixel_sdk import PortHandler, PacketHandler  # Manejo de puerto serial y paquetes de comunicación

import time  # Para pausas y temporización

# PyQt5 - Biblioteca para crear interfaces gráficas de usuario (GUI)
from PyQt5.QtWidgets import (
    QApplication,      # Aplicación principal de Qt
    QMainWindow,       # Ventana principal
    QWidget,           # Widget base
    QVBoxLayout,       # Layout vertical (apila widgets de arriba a abajo)
    QHBoxLayout,       # Layout horizontal (apila widgets de izquierda a derecha)
    QLabel,            # Etiqueta de texto
    QPushButton,       # Botón clickeable
    QSlider,           # Control deslizante
    QLineEdit,         # Campo de entrada de texto
    QFrame,            # Marco/contenedor con borde
    QMessageBox,       # Cuadros de diálogo (alertas, confirmaciones)
    QStackedWidget,    # Widget que muestra una página a la vez (como pestañas)
    QScrollArea,       # Área con scroll para contenido largo
    QGridLayout,       # Layout en forma de grilla/tabla
    QSpacerItem,       # Espaciador para layouts
    QSizePolicy        # Políticas de tamaño de widgets
)
from PyQt5.QtCore import Qt, QTimer, QSize, pyqtSignal  # Constantes Qt, temporizadores y señales
from PyQt5.QtGui import QFont, QPixmap, QIcon, QPalette, QColor  # Fuentes, imágenes, iconos, colores

import threading    # Para ejecutar tareas en hilos paralelos (no bloquear la GUI)
import subprocess   # Para ejecutar comandos externos (como lanzar RViz)
import os           # Operaciones del sistema operativo (rutas de archivos)
import sys          # Acceso a argumentos del sistema y salida del programa

import numpy as np  # Biblioteca numérica para cálculos matemáticos y matrices

# Robotics Toolbox - Biblioteca para modelado y cinemática de robots
import roboticstoolbox as rtb  # Herramientas de robótica (cinemática, DH, etc.)
from spatialmath import SE3    # Transformaciones espaciales 3D (matrices homogéneas)

from PyQt5.QtGui import QImage  # Para convertir imágenes de OpenCV a Qt

# OpenCV - Biblioteca de visión por computadora
import cv2  # Procesamiento de imágenes y video

# Mensajes estándar de ROS2
from std_msgs.msg import String              # Mensaje de texto simple
from geometry_msgs.msg import PointStamped   # Punto 3D con timestamp y frame de referencia
from sensor_msgs.msg import Image            # Mensaje de imagen
from cv_bridge import CvBridge               # Convertir entre OpenCV y ROS

import json  # Para parsear información JSON


# ============================================================
#  CONFIGURACIÓN GENERAL DEL ROBOT
# ============================================================

# Selector de tipo de motor Dynamixel
# True = XL430 (protocolo 2.0, 12 bits de resolución)
# False = AX-12/MX (protocolo 1.0, 10 bits de resolución)
USE_XL430 = False

# ============================================================
#  CONFIGURACIÓN DE OPENCV (VISIÓN POR COMPUTADORA)
# ============================================================

# Offset de calibración entre el frame de la cámara y el frame del robot (en metros)
# Estos valores ajustan la diferencia entre donde la cámara "ve" un objeto
# y donde realmente está en el espacio de trabajo del robot
OPENCV_OFFSET_X_M = -0.1   # Offset en X (adelante/atrás del robot)
OPENCV_OFFSET_Y_M = 0.0    # Offset en Y (izquierda/derecha del robot)

# Alturas de trabajo para pick and place (en metros)
Z_PICK_M = -0.025          # Altura Z donde el gripper agarra la pieza
Z_APPROACH_M = Z_PICK_M + 0.05   # Altura de aproximación (5cm arriba del pick)

# Diccionario de puntos de destino (DROP) para cada tipo de figura
# Las claves son códigos de figura: "s"=square, "r"=rectangle, "c"=circle, "p"=pentagon
# Los valores son tuplas (x, y, z) en metros donde soltar cada figura
DROP_POINTS = {
    "s": (0.0,  -0.13, 0.02),   # Cuadrado -> zona Roja
    "r": (0.0, 0.13, 0.02),     # Rectángulo -> zona Amarilla
    "c": (-0.2,  -0.09, 0.02),  # Círculo -> zona Verde
    "p": (-0.2,  0.09, 0.02),   # Pentágono -> zona Azul
}

def restricted_zone_name(x: float, y: float):
    """
    Verifica si una posición (x, y) está en una zona restringida del espacio de trabajo.
    """
    if (x > -0.075):
        return "Cuadrante 1"

    if (x < -0.155) and (-0.025 <= y <= 0.025):
        return "Cuadrante 2"

    return None

# ============================================================
#  CONFIGURACIÓN DE HOME PARA OPENCV
# ============================================================

HOME_OPENCV_MOTOR_VALUE = [512, 254, 798, 783]
HOME_OPENCV_WAIT_S = 1.0

# ============================================================
#  CONFIGURACIÓN DEL GRIPPER (GARRA)
# ============================================================

GRIPPER_OPEN_DEG = 0.0
GRIPPER_CLOSE_DEG = 70.0
MOVE_WAIT_S = 1

# ============================================================
#  DIMENSIONES DEL ROBOT (LONGITUDES DE ESLABONES)
# ============================================================

L1 = 44.0  / 1000.0
L2 = 107.5 / 1000.0
L3 = 107.5 / 1000.0
L4 = 75.3  / 1000.0

PLANAR_REACH_MAX = L2 + L3 + L4
PLANAR_REACH_MIN = 0.04
Z_MAX = L1 + L2 + L3 + L4
Z_MIN = -0.03

# ============================================================
#  CONFIGURACIÓN DE MOTORES DYNAMIXEL
# ============================================================

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
#  HOJA DE ESTILOS CSS PARA LA INTERFAZ GRÁFICA
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
#  FUNCIONES AUXILIARES PARA COMUNICACIÓN CON MOTORES
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
#  CLASE CONTROLADOR ROS2 - NODO PRINCIPAL DEL ROBOT
# ============================================================

class PincherController(Node):
    """
    Nodo ROS2 que controla el robot PhantomX Pincher.
    Ahora se suscribe a los tópicos del detector OpenCV en lugar de capturar directamente.
    """
    
    position_changed = None
    
    def __init__(self):
        super().__init__('pincher_controller')
        
        self.robot_model = build_pincher_robot()
        
        self.planar_reach_max = PLANAR_REACH_MAX
        self.planar_reach_min = PLANAR_REACH_MIN
        self.z_min = Z_MIN
        self.z_max = Z_MAX

        # Parámetros ROS2
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

        # Configuración puerto serial
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
        
        # Publishers (mantenidos para compatibilidad)
        from sensor_msgs.msg import JointState
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # ============================================================
        #  SUSCRIPCIONES A TÓPICOS DE OPENCV
        # ============================================================
        
        # Bridge para convertir imágenes ROS a OpenCV
        self.bridge = CvBridge()
        
        # Suscripción a imagen procesada
        self.image_sub = self.create_subscription(
            Image,
            '/opencv/image',
            self.image_callback,
            10
        )
        
        # Suscripción a código de figura
        self.shape_sub = self.create_subscription(
            String,
            '/opencv/shape',
            self.shape_callback,
            10
        )
        
        # Suscripción a posición 3D
        self.point_sub = self.create_subscription(
            PointStamped,
            '/opencv/target_point',
            self.point_callback,
            10
        )
        
        # Suscripción a información detallada
        self.info_sub = self.create_subscription(
            String,
            '/opencv/detection_info',
            self.info_callback,
            10
        )
        
        # Variables para almacenar última información recibida
        self.last_opencv_image = None
        self.last_opencv_shape = None
        self.last_opencv_point = None
        self.last_opencv_info = None
        self.last_opencv_measurement = None  # (shape_code, x, y, z)
        
        self.get_logger().info('Controlador suscrito a tópicos de OpenCV')

        # Timer para publicar estados
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)
        
        self.current_joint_positions = [0.0] * 5
        
        self.joint_names = [
            'phantomx_pincher_arm_shoulder_pan_joint',
            'phantomx_pincher_arm_shoulder_lift_joint',
            'phantomx_pincher_arm_elbow_flex_joint',
            'phantomx_pincher_arm_wrist_flex_joint',
            'phantomx_pincher_gripper_finger1_joint',
        ]

        self.joint_sign = {1: 1, 2: 1, 3: 1, 4: 1, 5: 1}
        
        self.initialize_motors(goal_positions, moving_speed, torque_limit)

    # ============================================================
    #  CALLBACKS DE SUSCRIPCIONES
    # ============================================================
    
    def image_callback(self, msg):
        """Callback para imagen procesada de OpenCV."""
        try:
            # Convertir imagen ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_opencv_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
    
    def shape_callback(self, msg):
        """Callback para código de figura."""
        self.last_opencv_shape = msg.data
    
    def point_callback(self, msg):
        """Callback para posición 3D."""
        self.last_opencv_point = msg
        # Actualizar medición completa
        if self.last_opencv_shape:
            self.last_opencv_measurement = (
                self.last_opencv_shape,
                msg.point.x,
                msg.point.y,
                msg.point.z
            )
    
    def info_callback(self, msg):
        """Callback para información detallada en JSON."""
        try:
            info_dict = json.loads(msg.data)
            self.last_opencv_info = info_dict
            
            # Actualizar medición completa desde info
            if 'shape_code' in info_dict and 'x_m' in info_dict:
                self.last_opencv_measurement = (
                    info_dict['shape_code'],
                    info_dict['x_m'],
                    info_dict['y_m'],
                    info_dict['z_m']
                )
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parseando JSON: {e}')

    # ============================================================
    #  MÉTODOS DE CONVERSIÓN Y CONTROL (sin cambios)
    # ============================================================

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
        radians = np.radians(degrees)
        return self.radians_to_dxl(radians)

    def initialize_motors(self, goal_positions, moving_speed, torque_limit):
        for dxl_id, goal in zip(self.dxl_ids, goal_positions):
            try:
                result, error = self.packet.write1ByteTxRx(
                    self.port, dxl_id, ADDR_TORQUE_ENABLE, 1
                )
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
            self.get_logger().warning(
                f'No se puede mover motor {motor_id}: Parada de emergencia activada'
            )
            return False
            
        try:
            result, error = write_goal_position(self.packet, self.port, motor_id, position)
            
            if result == 0:
                self.get_logger().info(f'[Motor {motor_id}] Moviendo a {position}')
                
                joint_index = self.dxl_ids.index(motor_id)
                angle = self.dxl_to_radians(position)
                angle *= self.joint_sign.get(motor_id, 1)
                self.current_joint_positions[joint_index] = angle
                
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
    
    def move_to_motor_value(self, values):
        if len(values) != 4:
            self.get_logger().error(f'Se esperaban 4 valores, se recibieron {len(values)}')
            return False

        success = True
        for i, motor_id in enumerate(self.dxl_ids[:4]):
            goal = int(np.clip(values[i], 0, DXL_MAX_VALUE))
            if not self.move_motor(motor_id, goal):
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
                    T_target, 
                    q0=seed,
                    ilimit=1000,
                    slimit=100,
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
        if self.emergency_stop_activated:
            self.reactivate_torque()
        
        self.move_to_angles_degrees([0, 0, 0, 0, 0])
        self.get_logger().info('Todos los motores movidos a 0°')
    
    def go_home_opencv(self):
        if self.emergency_stop_activated:
            self.reactivate_torque()

        ok = self.move_to_motor_value(HOME_OPENCV_MOTOR_VALUE)
        return ok

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

    # ============================================================
    #  MÉTODOS DEL GRIPPER
    # ============================================================
    
    def set_gripper_degrees(self, deg: float):
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
    #  RUTINA DE PICK & PLACE
    # ============================================================
    
    def run_pick_place(self, shape_code: str, x_m: float, y_m: float, z_pick_m: float):
        if shape_code not in DROP_POINTS:
            self.get_logger().error(f"Figura '{shape_code}' no tiene DROP_POINT configurado.")
            return False

        drop_x, drop_y, drop_z = DROP_POINTS[shape_code]

        if not self.move_to_xyz(x_m, y_m, Z_APPROACH_M):
            return False
        time.sleep(MOVE_WAIT_S)

        if not self.move_to_xyz(x_m, y_m, z_pick_m):
            return False
        time.sleep(MOVE_WAIT_S)

        if not self.gripper_close():
            self.get_logger().warning("No se pudo cerrar gripper (revisar calibración)")
        time.sleep(MOVE_WAIT_S)

        if not self.move_to_xyz(x_m, y_m, Z_APPROACH_M):
            return False
        time.sleep(MOVE_WAIT_S)

        if not self.go_home_opencv():
            self.get_logger().warning("No se pudo ir a HOME_OPENCV (continuando igual)")
        time.sleep(MOVE_WAIT_S)

        if shape_code == "s":
            self.move_motor(1, 842)
        elif shape_code == "r":
            self.move_motor(1, 213)
        elif shape_code == "c":
            self.move_motor(1, 598)
        elif shape_code == "p":
            self.move_motor(1, 430)
        time.sleep(MOVE_WAIT_S)
            
        drop_approach_z = max(drop_z, Z_APPROACH_M)
        if not self.move_to_xyz(drop_x, drop_y, drop_approach_z):
            return False
        time.sleep(MOVE_WAIT_S)

        if not self.move_to_xyz(drop_x, drop_y, drop_z):
            return False
        time.sleep(MOVE_WAIT_S)

        if not self.gripper_open():
            self.get_logger().warning("No se pudo abrir gripper (revisar calibración)")
        time.sleep(MOVE_WAIT_S)

        if not self.move_to_xyz(drop_x, drop_y, drop_approach_z):
            return False
        time.sleep(MOVE_WAIT_S)

        if not self.go_home_opencv():
            self.get_logger().warning("No se pudo volver a HOME_OPENCV al final")

        return True

    def close(self):
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            except:
                pass
        self.port.closePort()


# ============================================================
#  CLASE AUXILIAR PARA SEÑALES DE POSICIÓN
# ============================================================

class PositionSignal(QWidget):
    changed = pyqtSignal(int, int)

# ============================================================
#  CLASE PRINCIPAL DE LA INTERFAZ GRÁFICA
# ============================================================

class ModernPincherGUI(QMainWindow):
    """
    Interfaz gráfica moderna para controlar el robot PhantomX Pincher.
    Ahora recibe imágenes y detecciones del nodo opencv_detector vía tópicos ROS2.
    """
    
    routine_finished_sig = pyqtSignal(bool)
    
    def __init__(self, controller):
        super().__init__()
        
        self.controller = controller
        
        self.rviz_process = None
        self.toolbox_process = None
        
        self.last_motor_update = {motor_id: 0 for motor_id in controller.dxl_ids}
        self.update_interval = 0.05
        
        self.position_signal = PositionSignal()
        self.position_signal.changed.connect(self.on_position_changed)
        self.controller.position_changed = self.position_signal.changed

        self.routine_finished_sig.connect(self.on_routine_finished)
        
        self.init_ui()

        # ============================================================
        #  TIMER PARA ACTUALIZAR VISTA DE CÁMARA DESDE ROS
        # ============================================================
        
        # Timer para actualizar imagen desde el tópico ROS
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.update_camera_from_ros)

        # ============================================================
        #  TIMER PARA ACTUALIZAR DISPLAY XYZ
        # ============================================================
        self.xyz_timer = QTimer()
        self.xyz_timer.timeout.connect(self.update_xyz_display)
        self.xyz_timer.start(200)
        
    def update_camera_from_ros(self):
        """
        Actualiza la vista de cámara con la imagen recibida del tópico ROS.
        """
        try:
            # Verificar que hay imagen disponible
            if self.controller.last_opencv_image is None:
                self.camera_label.setText("⏳ Esperando imagen de /opencv/image...")
                return

            frame_vis = self.controller.last_opencv_image.copy()

            # Convertir de BGR (OpenCV) a RGB (Qt)
            rgb = cv2.cvtColor(frame_vis, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            
            qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pix = QPixmap.fromImage(qimg)

            self.camera_label.setPixmap(
                pix.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
            
            # Actualizar labels con información de detección
            if self.controller.last_opencv_info:
                info = self.controller.last_opencv_info
                shape_code = info.get('shape_code', 'u')
                shape_name = info.get('shape_name', 'unknown')
                conf = info.get('confidence', 0.0)
                x_m = info.get('x_m', 0.0)
                y_m = info.get('y_m', 0.0)
                
                self.opencv_shape_label.setText(f"Figura: {shape_code} ({shape_name})")
                self.opencv_xy_label.setText(f"x,y: {x_m:.3f}, {y_m:.3f} m")
                self.opencv_conf_label.setText(f"conf: {conf:.0%}")
            else:
                self.opencv_shape_label.setText("Figura: -")
                self.opencv_xy_label.setText("x,y: -")
                self.opencv_conf_label.setText("conf: -")

        except Exception as e:
            self.camera_label.setText(f"❌ Error actualizando vista: {e}")
    
    def init_ui(self):
        self.setWindowTitle("PhantomX Pincher Control Studio")
        self.setGeometry(100, 50, 1400, 800)
        
        central = QWidget()
        central.setObjectName("centralWidget")
        self.setCentralWidget(central)
        
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        sidebar = self.create_sidebar()
        main_layout.addWidget(sidebar)
        
        content_container = QWidget()
        content_container.setObjectName("contentArea")
        content_layout = QVBoxLayout(content_container)
        content_layout.setContentsMargins(25, 25, 25, 25)
        
        self.stack = QStackedWidget()
        content_layout.addWidget(self.stack)
        
        self.create_pages()
        
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
        
        title = QLabel("PINCHER X100")
        title.setStyleSheet("color: #00d9ff; font-size: 20pt; font-weight: bold; padding: 20px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        subtitle = QLabel("Control Studio")
        subtitle.setStyleSheet("color: #b0b0c0; font-size: 11pt; padding-bottom: 20px;")
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)
        
        home_btn = QPushButton("🏠 HOME (0°)")
        home_btn.setObjectName("homeButtonSidebar")
        home_btn.clicked.connect(self.home_all)
        layout.addWidget(home_btn)
        
        emergency_btn = QPushButton("🛑 EMERGENCIA")
        emergency_btn.setObjectName("emergencyButtonSidebar")
        emergency_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(emergency_btn)
        
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet("background-color: #3a3a4e; margin: 10px;")
        layout.addWidget(separator)
        
        self.menu_buttons = []
        
        menus = [
            ("📊 Panel Principal", 0),
            ("🎚️ Control Manual", 1),
            ("📝 Valores Fijos", 2),
            ("📐 Ángulos Predef.", 3),
            ("🎯 Control XYZ", 4),
            ("📷 OpenCV", 5),
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
        self.stack.addWidget(self.create_opencv_page())
        self.stack.addWidget(self.create_visualization_page())
        self.stack.addWidget(self.create_info_page())

    def change_page(self, index):
        self.stack.setCurrentIndex(index)
        
        for i, btn in enumerate(self.menu_buttons):
            btn.setChecked(i == index)

        # Control de timer de cámara según página
        if index == 5:  # Página OpenCV
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

    def create_opencv_page(self):
        """
        Crea la página de OpenCV.
        Ahora muestra la imagen recibida del nodo detector en lugar de capturar directamente.
        """
        page = QWidget()
        layout = QVBoxLayout(page)

        title = QLabel("OpenCV - Detección de Figuras")
        title.setObjectName("pageTitle")
        layout.addWidget(title)

        subtitle = QLabel("Vista de cámara desde /opencv/image + rutina de pick & place")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)

        # Tarjeta de vista de cámara
        video_card = QFrame()
        video_card.setObjectName("card")
        video_layout = QVBoxLayout(video_card)

        self.camera_label = QLabel("⏳ Esperando conexión con nodo opencv_detector...")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumHeight(360)
        self.camera_label.setStyleSheet("background-color: #111; border-radius: 10px;")
        video_layout.addWidget(self.camera_label)

        layout.addWidget(video_card)

        # Tarjeta de información de detección
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

        # Botones de acción
        btn_row = QHBoxLayout()

        self.btn_home_opencv = QPushButton("🏠 Home OpenCV")
        self.btn_home_opencv.clicked.connect(self.home_opencv)
        self.btn_home_opencv.clicked.connect(self.controller.gripper_open)
        btn_row.addWidget(self.btn_home_opencv)

        self.btn_capture_opencv = QPushButton("📌 Guardar dato actual (usa info de ROS)")
        self.btn_capture_opencv.clicked.connect(self.capture_opencv_data_from_ros)
        btn_row.addWidget(self.btn_capture_opencv)

        self.btn_run_routine = QPushButton("🤖 Ejecutar rutina (usa dato guardado)")
        self.btn_run_routine.clicked.connect(self.run_saved_routine)
        btn_row.addWidget(self.btn_run_routine)

        layout.addLayout(btn_row)
        layout.addStretch()
        return page

    def home_opencv(self):
        """Mueve el robot a HOME OpenCV."""
        self.controller.go_home_opencv()
        self.status_label.setText("● HOME OpenCV")
        QTimer.singleShot(2500, lambda: self.status_label.setText("● Sistema Listo"))

    def capture_opencv_data_from_ros(self):
        """
        Guarda el dato actual recibido de ROS para uso posterior.
        Ya no captura directamente, sino que usa la información del tópico.
        """
        # Verificar que hay información disponible
        if self.controller.last_opencv_measurement is None:
            QMessageBox.warning(
                self, 
                "OpenCV", 
                "No hay detección disponible.\n\nAsegúrate de que el nodo opencv_detector esté corriendo:\n"
                "ros2 run tu_paquete opencv_detector"
            )
            return

        shape_code, x_m, y_m, z_m = self.controller.last_opencv_measurement

        # Mostrar confirmación
        self.status_label.setText(f"● Dato guardado: {shape_code} @ ({x_m:.3f}, {y_m:.3f})")
        QTimer.singleShot(2500, lambda: self.status_label.setText("● Sistema Listo"))

        QMessageBox.information(
            self,
            "Dato Guardado",
            f"Figura: {shape_code}\n"
            f"Posición: ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}) m\n\n"
            f"Usa 'Ejecutar rutina' para hacer pick & place."
        )

    def run_saved_routine(self):
        """Ejecuta la rutina de pick & place con el dato guardado."""
        if self.controller.last_opencv_measurement is None:
            QMessageBox.warning(self, "Rutina", "Primero guarda el dato con 'Guardar dato actual'.")
            return

        shape_code, x_m, y_m, z_m = self.controller.last_opencv_measurement
        
        # Verificar zona restringida
        zone = restricted_zone_name(x_m, y_m)
        if zone is not None:
            msg = (
                f"Posición restringida ({zone}).\n"
                f"x={x_m:.3f} m, y={y_m:.3f} m\n\n"
                "La posición no puede ser alcanzada.\n"
                "No se ejecutará la rutina."
            )
            QMessageBox.warning(self, "Zona restringida", msg)
            self.status_label.setText("● Posición en zona restringida (rutina cancelada)")
            QTimer.singleShot(3000, lambda: self.status_label.setText("● Sistema Listo"))
            return

        self.status_label.setText("● Ejecutando rutina...")
        self.btn_run_routine.setEnabled(False)

        def worker():
            ok = False
            try:
                ok = self.controller.run_pick_place(shape_code, x_m, y_m, z_m)
            except Exception as e:
                print("❌ Excepción en worker rutina:", e)
                ok = False
            finally:
                self.routine_finished_sig.emit(ok)

        threading.Thread(target=worker, daemon=True).start()

    def on_routine_finished(self, ok: bool):
        """Callback cuando termina la rutina."""
        self.btn_run_routine.setEnabled(True)
        if ok:
            self.status_label.setText("● Rutina completada")
        else:
            self.status_label.setText("● Rutina falló")
        QTimer.singleShot(3000, lambda: self.status_label.setText("● Sistema Listo"))

    def create_visualization_page(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Visualización")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
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
        
        developers = [
            ("Samuel David Sanchez Cardenas", 
             "⏰ Veremos si acaba primero el semestre o el semestre acaba conmigo\n🔗 GitHub: samsanchezcar"),
            ("Santiago Ávila Corredor", 
             "🙏 Pronto todo acabará\n🔗 GitHub: Santiago-Avila"),
            ("Santiago Mariño Cortés", 
             "🙊 Don't believe the hype\n🔗 GitHub: mrbrightside8"),
            ("Juan Angel Vargas Rodríguez", 
             "🙏 Agradecido con el de arriba\n🔗 GitHub: juvargasro"),
            ("Juan José Delgado Estrada", 
             "🤖 Mi propósito es tener un robot que me haga el aseo\n🔗 GitHub: Juan-delgado1"),
        ]
        
        for name, info in developers:
            card = QFrame()
            card.setObjectName("card")
            card_layout = QVBoxLayout(card)
            
            name_label = QLabel(name)
            name_label.setObjectName("sectionTitle")
            card_layout.addWidget(name_label)
            
            info_label = QLabel(info)
            info_label.setStyleSheet("color: #e0e0e0; line-height: 1.6; padding: 8px;")
            card_layout.addWidget(info_label)
            
            scroll_layout.addWidget(card)
        
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
    #  MÉTODOS DE CONTROL Y CALLBACKS
    # ============================================================
    
    def on_position_changed(self, motor_id, position):
        if motor_id in self.sliders:
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

            clean_env = os.environ.copy()
            for k in [
                "QT_QPA_PLATFORM_PLUGIN_PATH",
                "QT_PLUGIN_PATH",
                "QT_QPA_PLATFORMTHEME",
                "QT_DEBUG_PLUGINS",
            ]:
                clean_env.pop(k, None)

            def run_rviz():
                self.rviz_process = subprocess.Popen(cmd, env=clean_env)
                self.rviz_process.wait()
                QTimer.singleShot(0, self.on_rviz_closed)

            thread = threading.Thread(target=run_rviz, daemon=True)
            thread.start()

            self.rviz_btn.setEnabled(False)
            self.stop_rviz_btn.setEnabled(True)
            self.rviz_status_label.setText("● RViz ejecutándose")
            self.rviz_status_label.setStyleSheet(
                "color: #00d9ff; font-weight: bold; padding: 15px; font-size: 12pt;"
            )

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
            # Detener timer de cámara
            try:
                if hasattr(self, "camera_timer") and self.camera_timer.isActive():
                    self.camera_timer.stop()
            except:
                pass
            
            self.controller.close()
            event.accept()
            rclpy.shutdown()
        else:
            event.ignore()


# ============================================================
#  FUNCIÓN PRINCIPAL - PUNTO DE ENTRADA
# ============================================================

def main(args=None):
    """
    Función principal que inicializa ROS2, el controlador y la GUI.
    """
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
