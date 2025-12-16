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

# Mensajes estándar de ROS2 para publicar datos
from std_msgs.msg import String              # Mensaje de texto simple
from geometry_msgs.msg import PointStamped   # Punto 3D con timestamp y frame de referencia

# Módulo local con funciones de detección de figuras por visión
from .opencv_detector import (
    detectar_figuras_y_polares,   # Función principal que detecta figuras y calcula coords polares
    pick_best_detection,          # Selecciona la mejor detección de varias candidatas
    shape_name_to_code,           # Convierte nombre de figura a código ("square" -> "s")
    polar_to_cartesian_m,         # Convierte coordenadas polares a cartesianas en metros
)


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

# Índice de la cámara (0 = cámara por defecto del sistema)
OPENCV_CAMERA_INDEX = 0

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
    Las zonas restringidas son áreas donde el robot no puede/debe operar
    (por ejemplo, cerca de la base o fuera del alcance seguro).
    
    Args:
        x: Coordenada X en metros
        y: Coordenada Y en metros
    
    Returns:
        str: Nombre de la zona restringida si está en una, None si es zona válida
    """
    # Cuadrante 1: Zona muy cercana al robot (x > -0.075m)
    if (x > -0.075):
        return "Cuadrante 1"

    # Cuadrante 2: Zona trasera central del robot
    if (x < -0.155) and (-0.025 <= y <= 0.025):
        return "Cuadrante 2"

    # Si no está en ninguna zona restringida, retorna None (posición válida)
    return None

# ============================================================
#  CONFIGURACIÓN DE HOME PARA OPENCV
# ============================================================

# Posición HOME específica para la rutina de OpenCV
# Son valores directos de motor (0-1023 para AX-12) para los 4 primeros motores
# Esta posición es un "waypoint" intermedio seguro para transiciones
HOME_OPENCV_MOTOR_VALUE = [512, 254, 798, 783]   # [motor1, motor2, motor3, motor4]

# Tiempo de espera al llegar al home (en segundos)
HOME_OPENCV_WAIT_S = 1.0

# ============================================================
#  CONFIGURACIÓN DEL GRIPPER (GARRA)
# ============================================================

# Ángulos del gripper en grados - deben calibrarse según la garra física
GRIPPER_OPEN_DEG = 0.0     # Ángulo con gripper completamente abierto
GRIPPER_CLOSE_DEG = 70.0   # Ángulo con gripper cerrado (agarrando objeto)

# Tiempo de espera entre movimientos de la rutina (segundos)
# Aumentar si el robot necesita más tiempo para completar movimientos
MOVE_WAIT_S = 1

# ============================================================
#  DIMENSIONES DEL ROBOT (LONGITUDES DE ESLABONES)
# ============================================================
# Estas medidas definen la geometría del brazo PhantomX Pincher
# Se usan para calcular la cinemática directa e inversa

L1 = 44.0  / 1000.0   # Altura de la base al primer joint (m)
L2 = 107.5 / 1000.0   # Longitud del primer eslabón/hombro (m)
L3 = 107.5 / 1000.0   # Longitud del segundo eslabón/codo (m)
L4 = 75.3  / 1000.0   # Longitud del tercer eslabón hasta el gripper (m)

# Cálculo de los límites del espacio de trabajo
PLANAR_REACH_MAX = L2 + L3 + L4  # Alcance máximo en el plano XY (brazo extendido)
PLANAR_REACH_MIN = 0.04          # Alcance mínimo (evita singularidades cerca de la base)
Z_MAX = L1 + L2 + L3 + L4        # Altura máxima alcanzable
Z_MIN = -0.03                    # Altura mínima (puede ser negativa si trabaja bajo la base)

# ============================================================
#  CONFIGURACIÓN DE MOTORES DYNAMIXEL
# ============================================================
# Los motores Dynamixel tienen diferentes direcciones de memoria
# según el modelo/protocolo usado

if USE_XL430:
    # Configuración para motores XL430 (Protocolo 2.0)
    PROTOCOL_VERSION = 2.0
    ADDR_TORQUE_ENABLE    = 64   # Dirección para habilitar/deshabilitar torque
    ADDR_GOAL_POSITION    = 116  # Dirección para escribir posición objetivo
    ADDR_MOVING_SPEED     = 112  # Dirección para configurar velocidad
    ADDR_TORQUE_LIMIT     = 38   # Dirección para límite de torque
    ADDR_PRESENT_POSITION = 132  # Dirección para leer posición actual
    DEFAULT_GOAL = 2048          # Posición central (12 bits: 0-4095)
    MAX_SPEED = 1023             # Velocidad máxima permitida
    DXL_MAX_VALUE = 4095         # Valor máximo de posición (12 bits)
else:
    # Configuración para motores AX-12/MX (Protocolo 1.0)
    PROTOCOL_VERSION = 1.0
    ADDR_TORQUE_ENABLE    = 24   # Dirección para habilitar/deshabilitar torque
    ADDR_GOAL_POSITION    = 30   # Dirección para escribir posición objetivo
    ADDR_MOVING_SPEED     = 32   # Dirección para configurar velocidad
    ADDR_TORQUE_LIMIT     = 34   # Dirección para límite de torque
    ADDR_PRESENT_POSITION = 36   # Dirección para leer posición actual
    DEFAULT_GOAL = 512           # Posición central (10 bits: 0-1023)
    MAX_SPEED = 1023             # Velocidad máxima permitida
    DXL_MAX_VALUE = 1023         # Valor máximo de posición (10 bits)

# ============================================================
#  HOJA DE ESTILOS CSS PARA LA INTERFAZ GRÁFICA
# ============================================================
# Define el aspecto visual moderno y oscuro de la aplicación
# Usa gradientes, colores neón (#00d9ff) y esquinas redondeadas

MODERN_STYLESHEET = """
/* Fuente global para toda la aplicación */
* {
    font-family: 'Segoe UI', 'Ubuntu', sans-serif;
}

/* Ventana principal - fondo oscuro */
QMainWindow {
    background-color: #1e1e2e;
}

/* Widget central */
QWidget#centralWidget {
    background-color: #1e1e2e;
}

/* Área de contenido principal */
QWidget#contentArea {
    background-color: #2a2a3e;
    border-radius: 15px;
}

/* Barra lateral (sidebar) con gradiente */
QFrame#sidebar {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                stop:0 #0f0f1e, stop:1 #1a1a2e);
    border-right: 2px solid #00d9ff;
}

/* Botones del menú lateral - estado normal */
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

/* Botones del menú - al pasar el mouse */
QPushButton#menuButton:hover {
    background-color: #2a2a3e;
    color: #00d9ff;
    border-left: 4px solid #00d9ff;
}

/* Botones del menú - seleccionado/activo */
QPushButton#menuButton:checked {
    background-color: #0f4c75;
    color: #00d9ff;
    border-left: 4px solid #00d9ff;
    font-weight: bold;
}

/* Botón HOME en la barra lateral */
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

/* Botón de EMERGENCIA en la barra lateral - rojo */
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

/* Título de página */
QLabel#pageTitle {
    color: #00d9ff;
    font-size: 24pt;
    font-weight: bold;
    padding: 10px;
}

/* Título de sección */
QLabel#sectionTitle {
    color: #00d9ff;
    font-size: 16pt;
    font-weight: bold;
    padding: 8px;
}

/* Subtítulo */
QLabel#subtitle {
    color: #b0b0c0;
    font-size: 11pt;
    padding: 5px;
}

/* Tarjetas/cards - contenedores con fondo oscuro */
QFrame#card {
    background-color: #252538;
    border-radius: 12px;
    border: 1px solid #3a3a4e;
    padding: 15px;
}

QFrame#card:hover {
    border: 1px solid #00d9ff;
}

/* Botones generales - gradiente cyan */
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

/* Botones de presets - gradiente púrpura */
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

/* Etiquetas de texto generales */
QLabel {
    color: #e0e0e0;
    font-size: 10pt;
}

/* Etiqueta de estado */
QLabel#statusLabel {
    background-color: #252538;
    border: 2px solid #00d9ff;
    border-radius: 8px;
    padding: 10px;
    font-weight: bold;
    font-size: 11pt;
    color: #00d9ff;
}

/* Etiqueta de motor */
QLabel#motorLabel {
    color: #00d9ff;
    font-weight: bold;
    font-size: 11pt;
}

/* Etiqueta de valor numérico */
QLabel#valueLabel {
    background-color: #1a1a2e;
    border-radius: 5px;
    padding: 8px;
    font-weight: bold;
    color: #00d9ff;
    min-width: 60px;
}

/* Campos de entrada de texto */
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

/* Sliders - pista/groove */
QSlider::groove:horizontal {
    border: none;
    height: 10px;
    background: #1a1a2e;
    border-radius: 5px;
}

/* Sliders - manija/handle */
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

/* Sliders - parte izquierda (valor actual) */
QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                stop:0 #00a8cc, stop:1 #00d9ff);
    border-radius: 5px;
}

/* Área de scroll */
QScrollArea {
    border: none;
    background-color: transparent;
}

/* Barra de scroll vertical */
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
    """
    Escribe la posición objetivo en un motor Dynamixel.
    
    Args:
        packet: PacketHandler de Dynamixel SDK
        port: PortHandler del puerto serial
        dxl_id: ID del motor (1-5)
        position: Posición objetivo (0-1023 o 0-4095 según protocolo)
    
    Returns:
        tuple: (resultado, error) de la operación
    """
    if USE_XL430:
        # Protocolo 2.0 usa 4 bytes para posición
        return packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))
    else:
        # Protocolo 1.0 usa 2 bytes para posición
        return packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))

def write_moving_speed(packet, port, dxl_id, speed):
    """
    Configura la velocidad de movimiento de un motor.
    
    Args:
        packet: PacketHandler de Dynamixel SDK
        port: PortHandler del puerto serial
        dxl_id: ID del motor
        speed: Velocidad (0-1023, donde 0 = máxima)
    
    Returns:
        tuple: (resultado, error) de la operación
    """
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))

def read_present_position(packet, port, dxl_id):
    """
    Lee la posición actual de un motor.
    
    Args:
        packet: PacketHandler de Dynamixel SDK
        port: PortHandler del puerto serial
        dxl_id: ID del motor
    
    Returns:
        tuple: (posición_actual, resultado, error)
    """
    if USE_XL430:
        return packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    else:
        return packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)

def build_pincher_robot():
    """
    Construye el modelo cinemático del robot PhantomX Pincher usando
    parámetros de Denavit-Hartenberg (DH).
    
    El robot tiene 4 grados de libertad (4 joints rotacionales).
    
    Parámetros DH para cada joint:
    - d: offset a lo largo del eje Z anterior
    - a: longitud del eslabón (eje X)
    - alpha: ángulo de torsión entre ejes Z
    - offset: offset angular del joint
    
    Returns:
        DHRobot: Modelo del robot para cálculos cinemáticos
    """
    links = [
        # Joint 1: Base rotacional (pan) - rota alrededor de Z
        rtb.RevoluteDH(d=L1, a=0.0,  alpha=np.pi/2, offset=0.0),
        # Joint 2: Hombro (lift) - levanta el brazo
        rtb.RevoluteDH(d=0.0, a=L2, alpha=0.0,     offset=np.pi/2),
        # Joint 3: Codo (elbow) - flexiona el brazo
        rtb.RevoluteDH(d=0.0, a=L3, alpha=0.0,     offset=0.0),
        # Joint 4: Muñeca (wrist) - orienta el gripper
        rtb.RevoluteDH(d=0.0, a=L4, alpha=0.0,     offset=0.0),
    ]
    
    # Crear el modelo del robot
    robot = rtb.DHRobot(links, name="Pincher")
    
    # Definir la transformación de la herramienta (gripper)
    # Rota el frame del end-effector para que Z apunte hacia abajo
    T_tool = SE3.Rz(-np.pi/2) * SE3.Rx(-np.pi/2)
    robot.tool = T_tool
    
    return robot

# ============================================================
#  CLASE CONTROLADOR ROS2 - NODO PRINCIPAL DEL ROBOT
# ============================================================

class PincherController(Node):
    """
    Nodo ROS2 que controla el robot PhantomX Pincher.
    
    Responsabilidades:
    - Comunicación con motores Dynamixel via serial
    - Publicación de estados de joints a ROS2
    - Cálculos de cinemática directa e inversa
    - Control de movimiento (posición, velocidad)
    - Funciones de seguridad (parada de emergencia)
    - Integración con visión OpenCV
    """
    
    # Señal para actualizar GUI desde otro hilo (se asigna desde la GUI)
    position_changed = None
    
    def __init__(self):
        """
        Inicializa el nodo ROS2 y configura el hardware.
        """
        # Inicializar la clase base Node con el nombre 'pincher_controller'
        super().__init__('pincher_controller')
        
        # Construir el modelo cinemático del robot
        self.robot_model = build_pincher_robot()
        
        # Guardar límites del espacio de trabajo
        self.planar_reach_max = PLANAR_REACH_MAX
        self.planar_reach_min = PLANAR_REACH_MIN
        self.z_min = Z_MIN
        self.z_max = Z_MAX

        # ============================================================
        #  DECLARACIÓN DE PARÁMETROS ROS2
        # ============================================================
        # Los parámetros pueden ser modificados desde la línea de comandos
        # o archivos de configuración YAML
        
        self.declare_parameter('port', '/dev/ttyUSB0')           # Puerto serial
        self.declare_parameter('baudrate', 1000000)              # Velocidad de comunicación
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])       # IDs de los 5 motores
        self.declare_parameter('goal_positions', [DEFAULT_GOAL] * 5)  # Posiciones iniciales
        self.declare_parameter('moving_speed', 100)              # Velocidad de movimiento
        self.declare_parameter('torque_limit', 800)              # Límite de torque

        # Obtener valores de los parámetros
        port_name = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.dxl_ids = self.get_parameter('dxl_ids').value
        goal_positions = self.get_parameter('goal_positions').value
        moving_speed = int(self.get_parameter('moving_speed').value)
        torque_limit = int(self.get_parameter('torque_limit').value)

        # ============================================================
        #  CONFIGURACIÓN DEL PUERTO SERIAL
        # ============================================================
        
        # Crear manejador del puerto
        self.port = PortHandler(port_name)
        
        # Intentar abrir el puerto
        if not self.port.openPort():
            self.get_logger().error(f'No se pudo abrir el puerto {port_name}')
            rclpy.shutdown()
            return

        # Configurar la velocidad de comunicación (baudrate)
        if not self.port.setBaudRate(baudrate):
            self.get_logger().error(f'No se pudo configurar baudrate={baudrate}')
            self.port.closePort()
            rclpy.shutdown()
            return

        # Crear manejador de paquetes según el protocolo
        self.packet = PacketHandler(PROTOCOL_VERSION)
        
        # Flag de parada de emergencia
        self.emergency_stop_activated = False
        
        # ============================================================
        #  PUBLISHERS ROS2
        # ============================================================
        
        # Importar mensaje JointState para publicar estados de los joints
        from sensor_msgs.msg import JointState
        
        # Publisher para estados de joints (usado por RViz para visualización)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Publishers para datos de OpenCV
        self.opencv_shape_pub = self.create_publisher(String, '/opencv/shape', 10)
        self.opencv_point_pub = self.create_publisher(PointStamped, '/opencv/target_point', 10)

        # Variable para guardar la última medición de OpenCV
        self.last_opencv_measurement = None

        # ============================================================
        #  TIMER PARA PUBLICAR ESTADOS
        # ============================================================
        
        # Crear timer que publica estados de joints cada 0.1 segundos (10 Hz)
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Array para almacenar posiciones actuales de los joints (en radianes)
        self.current_joint_positions = [0.0] * 5
        
        # Nombres de los joints (deben coincidir con el URDF del robot)
        self.joint_names = [
            'phantomx_pincher_arm_shoulder_pan_joint',    # Joint 1: Rotación base
            'phantomx_pincher_arm_shoulder_lift_joint',   # Joint 2: Hombro
            'phantomx_pincher_arm_elbow_flex_joint',      # Joint 3: Codo
            'phantomx_pincher_arm_wrist_flex_joint',      # Joint 4: Muñeca
            'phantomx_pincher_gripper_finger1_joint',     # Joint 5: Gripper
        ]

        # Signos de los joints (para corregir dirección de rotación si es necesario)
        # 1 = dirección normal, -1 = invertido
        self.joint_sign = {1: 1, 2: 1, 3: 1, 4: 1, 5: 1}
        
        # Inicializar todos los motores
        self.initialize_motors(goal_positions, moving_speed, torque_limit)

    def dxl_to_radians(self, dxl_value):
        """
        Convierte un valor de posición Dynamixel a radianes.
        
        Los motores Dynamixel reportan posición como un entero:
        - AX-12: 0-1023 representa aproximadamente -150° a +150°
        - XL430: 0-4095 representa el mismo rango con más resolución
        
        Args:
            dxl_value: Valor de posición del motor (entero)
        
        Returns:
            float: Ángulo en radianes
        """
        if USE_XL430:
            center, scale = 2048.0, 2.618 / 2048.0  # 2.618 rad ≈ 150°
        else:
            center, scale = 512.0, 2.618 / 512.0
        return (dxl_value - center) * scale

    def radians_to_dxl(self, radians):
        """
        Convierte un ángulo en radianes a valor de posición Dynamixel.
        
        Args:
            radians: Ángulo en radianes
        
        Returns:
            int: Valor de posición para enviar al motor
        """
        if USE_XL430:
            center, inv_scale = 2048.0, 2048.0 / 2.618
        else:
            center, inv_scale = 512.0, 512.0 / 2.618
        return int(radians * inv_scale + center)

    def degrees_to_dxl(self, degrees):
        """
        Convierte grados a valor Dynamixel (función de conveniencia).
        
        Args:
            degrees: Ángulo en grados
        
        Returns:
            int: Valor de posición para el motor
        """
        radians = np.radians(degrees)
        return self.radians_to_dxl(radians)

    def initialize_motors(self, goal_positions, moving_speed, torque_limit):
        """
        Inicializa todos los motores: habilita torque, configura velocidad
        y mueve a posiciones iniciales.
        
        Args:
            goal_positions: Lista de posiciones iniciales para cada motor
            moving_speed: Velocidad de movimiento
            torque_limit: Límite de torque (no usado actualmente)
        """
        for dxl_id, goal in zip(self.dxl_ids, goal_positions):
            try:
                # Habilitar torque (permite que el motor se mueva y mantenga posición)
                result, error = self.packet.write1ByteTxRx(
                    self.port, dxl_id, ADDR_TORQUE_ENABLE, 1
                )
                if result != 0:
                    self.get_logger().error(f'Error habilitando torque en motor {dxl_id}: {error}')
                    continue
                
                # Configurar velocidad de movimiento
                self.update_speed_single_motor(dxl_id, moving_speed)
                
                # Mover a posición inicial
                write_goal_position(self.packet, self.port, dxl_id, goal)
                
                # Actualizar el array interno de posiciones
                joint_index = self.dxl_ids.index(dxl_id)
                angle = self.dxl_to_radians(goal)
                angle *= self.joint_sign.get(dxl_id, 1)
                self.current_joint_positions[joint_index] = angle
                
                self.get_logger().info(f'Motor {dxl_id} configurado correctamente')
            except Exception as e:
                self.get_logger().error(f'Error configurando motor {dxl_id}: {str(e)}')

    def publish_joint_states(self):
        """
        Publica el estado actual de todos los joints a ROS2.
        
        Este callback se ejecuta periódicamente (10 Hz) y publica
        un mensaje JointState que RViz usa para visualizar el robot.
        """
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header
        
        # Crear mensaje JointState
        joint_state = JointState()
        
        # Configurar header con timestamp actual
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "base_link"
        
        # Asignar nombres y posiciones
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        
        # Publicar el mensaje
        self.joint_state_pub.publish(joint_state)

    def move_motor(self, motor_id, position):
        """
        Mueve un motor individual a una posición específica.
        
        Args:
            motor_id: ID del motor (1-5)
            position: Posición objetivo (valor Dynamixel)
        
        Returns:
            bool: True si el movimiento fue exitoso
        """
        # Verificar parada de emergencia
        if self.emergency_stop_activated:
            self.get_logger().warning(
                f'No se puede mover motor {motor_id}: Parada de emergencia activada'
            )
            return False
            
        try:
            # Enviar comando de posición al motor
            result, error = write_goal_position(self.packet, self.port, motor_id, position)
            
            if result == 0:  # 0 = éxito en Dynamixel SDK
                self.get_logger().info(f'[Motor {motor_id}] Moviendo a {position}')
                
                # Actualizar posición interna
                joint_index = self.dxl_ids.index(motor_id)
                angle = self.dxl_to_radians(position)
                angle *= self.joint_sign.get(motor_id, 1)
                self.current_joint_positions[joint_index] = angle
                
                # Notificar a la GUI del cambio de posición
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
        """
        Mueve todos los motores a ángulos especificados en grados.
        
        Args:
            angles_deg: Lista de 5 ángulos en grados [m1, m2, m3, m4, m5]
        
        Returns:
            bool: True si todos los movimientos fueron exitosos
        """
        if len(angles_deg) != 5:
            self.get_logger().error(f'Se esperaban 5 ángulos, se recibieron {len(angles_deg)}')
            return False
        
        success = True
        for i, motor_id in enumerate(self.dxl_ids):
            # Aplicar signo del joint
            sign = self.joint_sign.get(motor_id, 1)
            angle_deg = angles_deg[i] * sign
            
            # Convertir a valor Dynamixel y limitar al rango válido
            goal_dxl = self.degrees_to_dxl(angle_deg)
            goal_dxl = int(np.clip(goal_dxl, 0, DXL_MAX_VALUE))
            
            if not self.move_motor(motor_id, goal_dxl):
                success = False
        
        return success
    
    def move_to_motor_value(self, values):
        """
        Mueve los primeros 4 motores a valores Dynamixel directos.
        Útil para posiciones predefinidas calibradas.
        
        Args:
            values: Lista de 4 valores Dynamixel [m1, m2, m3, m4]
        
        Returns:
            bool: True si todos los movimientos fueron exitosos
        """
        if len(values) != 4:
            self.get_logger().error(f'Se esperaban 4 valores, se recibieron {len(values)}')
            return False

        success = True
        for i, motor_id in enumerate(self.dxl_ids[:4]):  # Solo los 4 primeros motores
            # Limitar al rango válido
            goal = int(np.clip(values[i], 0, DXL_MAX_VALUE))
            if not self.move_motor(motor_id, goal):
                success = False
        return success

    def move_to_joint_angles(self, q_rad):
        """
        Mueve los 4 primeros joints a ángulos en radianes.
        Usado principalmente por la cinemática inversa.
        
        Args:
            q_rad: Array de 4 ángulos en radianes
        
        Returns:
            bool: True si el movimiento fue exitoso
        """
        if len(q_rad) != 4:
            self.get_logger().error(f'Se esperaban 4 ángulos, se recibieron {len(q_rad)}')
            return False
        
        # Límites angulares de los motores (aproximadamente ±150°)
        q_min, q_max = -2.618, 2.618
        success = True
        
        for i, motor_id in enumerate(self.dxl_ids[:4]):
            sign = self.joint_sign.get(motor_id, 1)
            motor_angle = q_rad[i] * sign
            
            # Saturar ángulo si excede los límites
            motor_angle_clamped = float(np.clip(motor_angle, q_min, q_max))
            
            # Advertir si hubo saturación
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
        """
        Mueve el end-effector (gripper) a una posición cartesiana XYZ.
        Usa cinemática inversa para calcular los ángulos de los joints.
        
        Args:
            x: Coordenada X en metros (adelante/atrás)
            y: Coordenada Y en metros (izquierda/derecha)
            z: Coordenada Z en metros (arriba/abajo)
            orientation: 'down' para gripper apuntando hacia abajo
        
        Returns:
            bool: True si se encontró solución IK y el movimiento fue exitoso
        """
        try:
            # Calcular distancia radial desde la base
            r = np.hypot(x, y)
            
            # Verificar que está dentro del alcance radial
            if r < self.planar_reach_min or r > self.planar_reach_max + 0.01:
                self.get_logger().error(f'Objetivo fuera del alcance radial: r={r:.3f} m')
                return False
            
            # Verificar que está dentro del alcance en Z
            if z < self.z_min - 0.01 or z > self.z_max + 0.01:
                self.get_logger().error(f'Objetivo fuera del alcance en Z: z={z:.3f} m')
                return False
            
            # Construir la matriz de transformación objetivo
            if orientation == 'down':
                # Gripper apuntando hacia abajo (rotación de 180° alrededor de X)
                T_target = SE3(x, y, z) * SE3.Rx(np.pi)
            else:
                # Gripper horizontal
                T_target = SE3(x, y, z)
            
            # Obtener configuración actual como semilla inicial
            q_current = np.array(self.current_joint_positions[:4])
            
            # Múltiples semillas para mejorar probabilidad de encontrar solución
            seeds = [
                q_current,                      # Configuración actual
                np.array([0, 0, 0, 0]),         # Posición cero
                np.array([0, 0.5, -0.5, 0]),    # Configuración típica
                np.array([0, 1.0, -1.0, 0]),    # Más flexionado
            ]
            
            best_sol = None
            best_error = float('inf')
            
            # Intentar IK con cada semilla
            for seed in seeds:
                # Levenberg-Marquardt iterative solver
                sol = self.robot_model.ikine_LM(
                    T_target, 
                    q0=seed,           # Configuración inicial
                    ilimit=1000,       # Máximo de iteraciones
                    slimit=100,        # Máximo de búsquedas
                    mask=[1, 1, 1, 0, 0, 0]  # Solo importa posición, no orientación
                )
                
                if sol.success:
                    # Verificar precisión de la solución con cinemática directa
                    T_check = self.robot_model.fkine(sol.q)
                    error = np.linalg.norm(T_check.t - T_target.t)
                    
                    # Guardar si es la mejor solución hasta ahora
                    if error < best_error:
                        best_error = error
                        best_sol = sol
                    
                    # Si el error es muy pequeño, aceptar inmediatamente
                    if error < 0.003:  # 3mm de tolerancia
                        break
            
            if best_sol is None:
                self.get_logger().error(f'No se encontró solución IK')
                return False
            
            # Ejecutar movimiento con la solución encontrada
            return self.move_to_joint_angles(best_sol.q)
                
        except Exception as e:
            self.get_logger().error(f'Error en cinemática inversa: {str(e)}')
            return False

    def get_current_xyz(self):
        """
        Calcula la posición XYZ actual del end-effector usando cinemática directa.
        
        Returns:
            tuple: (x, y, z) en metros
        """
        try:
            # Obtener ángulos actuales de los 4 primeros joints
            q_current = np.array(self.current_joint_positions[:4])
            
            # Calcular cinemática directa
            T = self.robot_model.fkine(q_current)
            
            # Extraer posición del vector de traslación
            return (T.t[0], T.t[1], T.t[2])
        except Exception as e:
            self.get_logger().error(f'Error calculando cinemática directa: {str(e)}')
            return (0, 0, 0)

    def update_speed_single_motor(self, motor_id, speed):
        """
        Actualiza la velocidad de un motor individual.
        
        Args:
            motor_id: ID del motor
            speed: Velocidad (0-1023)
        
        Returns:
            bool: True si fue exitoso
        """
        try:
            result, error = write_moving_speed(self.packet, self.port, motor_id, speed)
            return result == 0
        except Exception as e:
            self.get_logger().error(f'Error actualizando velocidad motor {motor_id}: {str(e)}')
            return False

    def update_speed(self, speed):
        """
        Actualiza la velocidad de todos los motores.
        
        Args:
            speed: Velocidad (0-1023)
        """
        if self.emergency_stop_activated:
            self.get_logger().warning('No se puede actualizar velocidad: Parada de emergencia activada')
            return
        
        success_count = 0
        for motor_id in self.dxl_ids:
            if self.update_speed_single_motor(motor_id, speed):
                success_count += 1

    def home_all_motors(self):
        """
        Mueve todos los motores a la posición HOME (0 grados).
        Si hay parada de emergencia activa, primero reactiva el torque.
        """
        if self.emergency_stop_activated:
            self.reactivate_torque()
        
        # Mover todos los motores a 0 grados
        self.move_to_angles_degrees([0, 0, 0, 0, 0])
        self.get_logger().info('Todos los motores movidos a 0°')
    
    def go_home_opencv(self):
        """
        Mueve el robot a la posición HOME específica para OpenCV.
        Esta es una posición intermedia segura para las rutinas de pick & place.
        
        Returns:
            bool: True si el movimiento fue exitoso
        """
        if self.emergency_stop_activated:
            self.reactivate_torque()

        # Usar valores de motor predefinidos para HOME OpenCV
        ok = self.move_to_motor_value(HOME_OPENCV_MOTOR_VALUE)
        return ok

    def emergency_stop(self):
        """
        Activa la parada de emergencia.
        Desactiva el torque de todos los motores, permitiendo que
        el robot se mueva libremente (modo "limp").
        """
        self.emergency_stop_activated = True
        for dxl_id in self.dxl_ids:
            try:
                # Deshabilitar torque (motor queda libre)
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.get_logger().warning(f'Torque desactivado en motor {dxl_id}')
            except Exception as e:
                self.get_logger().error(f'Error en parada de emergencia motor {dxl_id}: {str(e)}')

    def reactivate_torque(self):
        """
        Reactiva el torque después de una parada de emergencia.
        Los motores vuelven a mantener su posición.
        """
        self.emergency_stop_activated = False
        for dxl_id in self.dxl_ids:
            try:
                # Habilitar torque nuevamente
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                self.get_logger().info(f'Torque reactivado en motor {dxl_id}')
            except Exception as e:
                self.get_logger().error(f'Error reactivando torque en motor {dxl_id}: {str(e)}')

    # ============================================================
    #  MÉTODOS DE OPENCV - PUBLICACIÓN DE DATOS DE VISIÓN
    # ============================================================
    
    def publish_opencv_measurement(self, shape_code: str, x_m: float, y_m: float, z_m: float):
        """
        Publica la medición de OpenCV a los tópicos ROS2.
        Permite que otros nodos reciban la información de la detección.
        
        Args:
            shape_code: Código de la figura ('s', 'r', 'c', 'p')
            x_m: Coordenada X en metros
            y_m: Coordenada Y en metros
            z_m: Coordenada Z en metros
        """
        # Publicar código de figura como String
        msg_shape = String()
        msg_shape.data = shape_code
        self.opencv_shape_pub.publish(msg_shape)

        # Publicar punto 3D con timestamp
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "disk_frame"  # Frame de referencia (cambiar si es necesario)
        pt.point.x = float(x_m)
        pt.point.y = float(y_m)
        pt.point.z = float(z_m)
        self.opencv_point_pub.publish(pt)

        # Guardar última medición para uso posterior
        self.last_opencv_measurement = (shape_code, x_m, y_m, z_m)

    # ============================================================
    #  MÉTODOS DEL GRIPPER
    # ============================================================
    
    def set_gripper_degrees(self, deg: float):
        """
        Mueve el gripper (motor 5) a un ángulo específico en grados.
        
        Args:
            deg: Ángulo del gripper en grados
        
        Returns:
            bool: True si fue exitoso
        """
        # Verificar que el motor 5 existe
        if 5 not in self.dxl_ids:
            self.get_logger().warning("No se encontró motor 5 en dxl_ids")
            return False
        
        motor_id = 5
        sign = self.joint_sign.get(motor_id, 1)
        
        # Convertir a valor Dynamixel
        goal_dxl = self.degrees_to_dxl(deg * sign)
        goal_dxl = int(np.clip(goal_dxl, 0, DXL_MAX_VALUE))
        
        return self.move_motor(motor_id, goal_dxl)

    def gripper_open(self):
        """Abre el gripper completamente."""
        return self.set_gripper_degrees(GRIPPER_OPEN_DEG)

    def gripper_close(self):
        """Cierra el gripper para agarrar objetos."""
        return self.set_gripper_degrees(GRIPPER_CLOSE_DEG)

    # ============================================================
    #  RUTINA DE PICK & PLACE
    # ============================================================
    
    def run_pick_place(self, shape_code: str, x_m: float, y_m: float, z_pick_m: float):
        """
        Ejecuta la rutina completa de pick and place:
        1. Aproximarse por arriba del objeto
        2. Bajar y agarrar
        3. Subir con el objeto
        4. Ir a HOME intermedio
        5. Ir al punto de destino según la figura
        6. Soltar el objeto
        7. Volver a HOME
        
        Args:
            shape_code: Código de la figura ('s', 'r', 'c', 'p')
            x_m: Coordenada X del objeto en metros
            y_m: Coordenada Y del objeto en metros
            z_pick_m: Altura Z para agarrar el objeto
        
        Returns:
            bool: True si la rutina se completó exitosamente
        """
        # Verificar que la figura tiene punto de destino configurado
        if shape_code not in DROP_POINTS:
            self.get_logger().error(f"Figura '{shape_code}' no tiene DROP_POINT configurado.")
            return False

        # Obtener coordenadas de destino
        drop_x, drop_y, drop_z = DROP_POINTS[shape_code]

        # ============================================================
        #  PASO 1: Aproximarse por arriba del objeto
        # ============================================================
        if not self.move_to_xyz(x_m, y_m, Z_APPROACH_M):
            return False
        time.sleep(MOVE_WAIT_S)  # Esperar a que complete el movimiento

        # ============================================================
        #  PASO 2: Bajar al nivel de pick
        # ============================================================
        if not self.move_to_xyz(x_m, y_m, z_pick_m):
            return False
        time.sleep(MOVE_WAIT_S)

        # ============================================================
        #  PASO 3: Cerrar gripper para agarrar
        # ============================================================
        if not self.gripper_close():
            self.get_logger().warning("No se pudo cerrar gripper (revisar calibración)")
        time.sleep(MOVE_WAIT_S)

        # ============================================================
        #  PASO 4: Subir con la pieza agarrada
        # ============================================================
        if not self.move_to_xyz(x_m, y_m, Z_APPROACH_M):
            return False
        time.sleep(MOVE_WAIT_S)

        # ============================================================
        #  PASO 5: Ir a HOME OpenCV (waypoint intermedio seguro)
        # ============================================================
        if not self.go_home_opencv():
            self.get_logger().warning("No se pudo ir a HOME_OPENCV (continuando igual)")
        time.sleep(MOVE_WAIT_S)

        # ============================================================
        #  PASO 6: Ajuste de motor 1 según la figura
        #  (Rotar base hacia la zona de destino)
        # ============================================================
        if shape_code == "s":
            # Square (cuadrado) -> zona roja
            self.move_motor(1, 842)
        elif shape_code == "r":
            # Rectangle (rectángulo) -> zona amarilla
            self.move_motor(1, 213)
        elif shape_code == "c":
            # Circle (círculo) -> zona verde
            self.move_motor(1, 598)
        elif shape_code == "p":
            # Pentagon (pentágono) -> zona azul
            self.move_motor(1, 430)
        time.sleep(MOVE_WAIT_S)
            
        # ============================================================
        #  PASO 7: Aproximarse al punto de destino
        # ============================================================
        drop_approach_z = max(drop_z, Z_APPROACH_M)  # Usar altura segura
        if not self.move_to_xyz(drop_x, drop_y, drop_approach_z):
            return False
        time.sleep(MOVE_WAIT_S)

        # ============================================================
        #  PASO 8: Bajar a la altura de destino
        # ============================================================
        if not self.move_to_xyz(drop_x, drop_y, drop_z):
            return False
        time.sleep(MOVE_WAIT_S)

        # ============================================================
        #  PASO 9: Abrir gripper para soltar
        # ============================================================
        if not self.gripper_open():
            self.get_logger().warning("No se pudo abrir gripper (revisar calibración)")
        time.sleep(MOVE_WAIT_S)

        # ============================================================
        #  PASO 10: Subir después de soltar
        # ============================================================
        if not self.move_to_xyz(drop_x, drop_y, drop_approach_z):
            return False
        time.sleep(MOVE_WAIT_S)

        # ============================================================
        #  PASO 11: Volver a HOME OpenCV al final
        # ============================================================
        if not self.go_home_opencv():
            self.get_logger().warning("No se pudo volver a HOME_OPENCV al final")
            # No es fallo fatal, la rutina se considera exitosa

        return True

    def close(self):
        """
        Cierra la conexión con los motores.
        Desactiva el torque y cierra el puerto serial.
        Debe llamarse al terminar el programa.
        """
        for dxl_id in self.dxl_ids:
            try:
                # Deshabilitar torque de cada motor
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            except:
                pass
        # Cerrar puerto serial
        self.port.closePort()

# ============================================================
#  CLASE AUXILIAR PARA SEÑALES DE POSICIÓN
# ============================================================

class PositionSignal(QWidget):
    """
    Widget auxiliar que contiene una señal PyQt para comunicar
    cambios de posición entre hilos.
    
    Las señales PyQt son thread-safe y permiten actualizar la GUI
    desde hilos de fondo sin causar problemas de concurrencia.
    """
    changed = pyqtSignal(int, int)  # Parámetros: (motor_id, position)

# ============================================================
#  CLASE PRINCIPAL DE LA INTERFAZ GRÁFICA
# ============================================================

class ModernPincherGUI(QMainWindow):
    """
    Interfaz gráfica moderna para controlar el robot PhantomX Pincher.
    
    Características:
    - Menú lateral con navegación por páginas
    - Panel principal con estado del robot
    - Control manual con sliders
    - Control por valores fijos
    - Posiciones predefinidas
    - Control XYZ con cinemática inversa
    - Integración con OpenCV para pick & place
    - Visualización con RViz y Robotics Toolbox
    - Información del proyecto
    """
    
    # Señal para indicar que la rutina terminó (thread-safe)
    routine_finished_sig = pyqtSignal(bool)
    
    def __init__(self, controller):
        """
        Inicializa la interfaz gráfica.
        
        Args:
            controller: Instancia de PincherController para comunicarse con el robot
        """
        super().__init__()
        
        # Guardar referencia al controlador
        self.controller = controller
        
        # Variables para procesos externos (RViz, Toolbox)
        self.rviz_process = None
        self.toolbox_process = None
        
        # Control de rate-limiting para actualizaciones de sliders
        # Evita saturar la comunicación serial con muchos comandos
        self.last_motor_update = {motor_id: 0 for motor_id in controller.dxl_ids}
        self.update_interval = 0.05  # Mínimo 50ms entre actualizaciones
        
        # Configurar señal de posición para actualizar GUI desde el controlador
        self.position_signal = PositionSignal()
        self.position_signal.changed.connect(self.on_position_changed)
        self.controller.position_changed = self.position_signal.changed

        # Conectar señal de fin de rutina
        self.routine_finished_sig.connect(self.on_routine_finished)
        
        # Construir la interfaz
        self.init_ui()

        # ============================================================
        #  INICIALIZACIÓN DE LA CÁMARA PARA OPENCV
        # ============================================================
        
        # Intentar abrir cámara con backend V4L2 (Linux)
        self.cap = cv2.VideoCapture(OPENCV_CAMERA_INDEX, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            # Fallback: intentar sin especificar backend
            self.cap = cv2.VideoCapture(OPENCV_CAMERA_INDEX)

        # Configurar formato y resolución de la cámara
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))  # Formato MJPG
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # Ancho
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Alto
        self.cap.set(cv2.CAP_PROP_FPS, 30)            # Frames por segundo

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        # Variable para guardar el último frame capturado
        self.last_frame = None
        
        # Timer para actualizar la vista de cámara
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.update_camera_frame)
        # NOTA: No se inicia aquí, se inicia solo cuando se entra a la pestaña OpenCV

        # ============================================================
        #  TIMER PARA ACTUALIZAR DISPLAY XYZ
        # ============================================================
        self.xyz_timer = QTimer()
        self.xyz_timer.timeout.connect(self.update_xyz_display)
        self.xyz_timer.start(200)  # Actualizar cada 200ms (5 Hz)
        
    def init_ui(self):
        """
        Inicializa todos los componentes de la interfaz de usuario.
        """
        # Configurar ventana principal
        self.setWindowTitle("PhantomX Pincher Control Studio")
        self.setGeometry(100, 50, 1400, 800)  # Posición X, Y, Ancho, Alto
        
        # Crear widget central
        central = QWidget()
        central.setObjectName("centralWidget")
        self.setCentralWidget(central)
        
        # Layout principal horizontal (sidebar + contenido)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)  # Sin márgenes
        main_layout.setSpacing(0)  # Sin espacio entre elementos
        
        # Crear y agregar la barra lateral
        sidebar = self.create_sidebar()
        main_layout.addWidget(sidebar)
        
        # Crear área de contenido principal
        content_container = QWidget()
        content_container.setObjectName("contentArea")
        content_layout = QVBoxLayout(content_container)
        content_layout.setContentsMargins(25, 25, 25, 25)
        
        # StackedWidget para cambiar entre páginas
        self.stack = QStackedWidget()
        content_layout.addWidget(self.stack)
        
        # Crear todas las páginas
        self.create_pages()
        
        # Barra de estado en la parte inferior
        status_frame = QFrame()
        status_layout = QHBoxLayout(status_frame)
        
        status_layout.addStretch()  # Espacio flexible a la izquierda
        
        # Label de estado
        self.status_label = QLabel("● Sistema Listo")
        self.status_label.setObjectName("statusLabel")
        status_layout.addWidget(self.status_label)
        
        status_layout.addStretch()  # Espacio flexible a la derecha
        
        content_layout.addWidget(status_frame)
        
        # Agregar contenido al layout principal (con stretch = 1 para que ocupe el espacio restante)
        main_layout.addWidget(content_container, 1)
        
    def create_sidebar(self):
        """
        Crea la barra lateral con navegación y controles principales.
        
        Returns:
            QFrame: Widget de la barra lateral
        """
        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(280)  # Ancho fijo
        
        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(0, 20, 0, 20)
        layout.setSpacing(5)
        
        # ============================================================
        #  TÍTULO Y LOGO
        # ============================================================
        title = QLabel("PINCHER X100")
        title.setStyleSheet("color: #00d9ff; font-size: 20pt; font-weight: bold; padding: 20px;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        subtitle = QLabel("Control Studio")
        subtitle.setStyleSheet("color: #b0b0c0; font-size: 11pt; padding-bottom: 20px;")
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)
        
        # ============================================================
        #  BOTONES DE ACCIÓN RÁPIDA
        # ============================================================
        
        # Botón HOME - mueve todos los motores a 0°
        home_btn = QPushButton("🏠 HOME (0°)")
        home_btn.setObjectName("homeButtonSidebar")
        home_btn.clicked.connect(self.home_all)
        layout.addWidget(home_btn)
        
        # Botón EMERGENCIA - desactiva todos los motores
        emergency_btn = QPushButton("🛑 EMERGENCIA")
        emergency_btn.setObjectName("emergencyButtonSidebar")
        emergency_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(emergency_btn)
        
        # Línea separadora
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet("background-color: #3a3a4e; margin: 10px;")
        layout.addWidget(separator)
        
        # ============================================================
        #  BOTONES DE MENÚ (NAVEGACIÓN)
        # ============================================================
        self.menu_buttons = []
        
        # Lista de menús: (texto, índice de página)
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
        
        # Crear botones de menú
        for text, index in menus:
            btn = QPushButton(text)
            btn.setObjectName("menuButton")
            btn.setCheckable(True)  # Permite estado "checked" (seleccionado)
            btn.setFixedHeight(55)
            # Conectar click al cambio de página
            btn.clicked.connect(lambda checked, i=index: self.change_page(i))
            layout.addWidget(btn)
            self.menu_buttons.append(btn)
        
        # Seleccionar el primer botón por defecto
        self.menu_buttons[0].setChecked(True)
        
        # Espacio flexible para empujar el footer hacia abajo
        layout.addStretch()
        
        # Footer con versión
        footer = QLabel("v2.0 • 2024")
        footer.setStyleSheet("color: #6a6a7e; font-size: 9pt; padding: 20px;")
        footer.setAlignment(Qt.AlignCenter)
        layout.addWidget(footer)
        
        return sidebar
    
    def create_pages(self):
        """
        Crea todas las páginas y las agrega al StackedWidget.
        El orden de agregar determina el índice de cada página.
        """
        self.stack.addWidget(self.create_dashboard_page())      # Índice 0
        self.stack.addWidget(self.create_manual_control_page()) # Índice 1
        self.stack.addWidget(self.create_fixed_values_page())   # Índice 2
        self.stack.addWidget(self.create_fixed_angles_page())   # Índice 3
        self.stack.addWidget(self.create_xyz_page())            # Índice 4
        self.stack.addWidget(self.create_opencv_page())         # Índice 5
        self.stack.addWidget(self.create_visualization_page())  # Índice 6
        self.stack.addWidget(self.create_info_page())           # Índice 7

    
    def change_page(self, index):
        """
        Cambia a la página especificada y actualiza estado de botones.
        
        Args:
            index: Índice de la página a mostrar
        """
        self.stack.setCurrentIndex(index)
        
        # Actualizar estado visual de los botones del menú
        for i, btn in enumerate(self.menu_buttons):
            btn.setChecked(i == index)

        # Control especial para la pestaña OpenCV (índice 5)
        # Solo activar la cámara cuando estamos en esa pestaña
        if index == 5:
            if not self.camera_timer.isActive():
                self.camera_timer.start(33)  # ~30 FPS (1000ms / 30 ≈ 33ms)
        else:
            if self.camera_timer.isActive():
                self.camera_timer.stop()

    
    def create_dashboard_page(self):
        """
        Crea la página del panel principal (dashboard).
        Muestra estado general del robot: posición, motores, velocidad.
        
        Returns:
            QWidget: Página del dashboard
        """
        page = QWidget()
        layout = QVBoxLayout(page)
        
        # Título de la página
        title = QLabel("Panel de Control Principal")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        subtitle = QLabel("Vista general del estado del robot")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)
        
        # Grid de tarjetas con información
        cards_layout = QGridLayout()
        cards_layout.setSpacing(20)
        
        # Tarjeta: Posición actual XYZ
        pos_card = self.create_card("Posición Actual", self.create_position_widget())
        cards_layout.addWidget(pos_card, 0, 0)
        
        # Tarjeta: Estado de los motores
        motors_card = self.create_card("Estado de Motores", self.create_motors_status_widget())
        cards_layout.addWidget(motors_card, 0, 1)
        
        # Tarjeta: Control de velocidad (ocupa 2 columnas)
        speed_card = self.create_card("Control de Velocidad", self.create_speed_widget())
        cards_layout.addWidget(speed_card, 1, 0, 1, 2)
        
        layout.addLayout(cards_layout)
        layout.addStretch()  # Espacio flexible al final
        
        return page
    
    def create_manual_control_page(self):
        """
        Crea la página de control manual con sliders para cada motor.
        
        Returns:
            QWidget: Página de control manual
        """
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Control Manual por Sliders")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        # Área scrolleable para los controles
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        # Diccionarios para almacenar referencias a widgets
        self.sliders = {}        # Sliders de cada motor
        self.slider_labels = {}  # Labels de valor de cada slider
        
        # Crear un control para cada motor
        for motor_id in self.controller.dxl_ids:
            # Tarjeta contenedora
            motor_card = QFrame()
            motor_card.setObjectName("card")
            motor_layout = QVBoxLayout(motor_card)
            
            # Título del motor
            motor_title = QLabel(f"Motor {motor_id}")
            motor_title.setObjectName("motorLabel")
            motor_layout.addWidget(motor_title)
            
            # Layout horizontal para slider y valor
            slider_layout = QHBoxLayout()
            
            # Slider
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(0)
            slider.setMaximum(DXL_MAX_VALUE)
            slider.setValue(DEFAULT_GOAL)
            # Conectar cambio de valor al handler
            slider.valueChanged.connect(lambda v, mid=motor_id: self.on_motor_slider_change(mid))
            slider_layout.addWidget(slider, 3)  # Stretch factor 3
            
            # Label con valor actual
            value_label = QLabel(f"{DEFAULT_GOAL}")
            value_label.setObjectName("valueLabel")
            value_label.setAlignment(Qt.AlignCenter)
            slider_layout.addWidget(value_label)
            
            motor_layout.addLayout(slider_layout)
            scroll_layout.addWidget(motor_card)
            
            # Guardar referencias
            self.sliders[motor_id] = slider
            self.slider_labels[motor_id] = value_label
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)
        
        return page
    
    def create_fixed_values_page(self):
        """
        Crea la página para mover motores a valores específicos ingresados manualmente.
        
        Returns:
            QWidget: Página de valores fijos
        """
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Mover a Valores Fijos")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        subtitle = QLabel("Ingresa valores específicos para cada motor")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)
        
        # Área scrolleable
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        # Diccionarios para widgets
        self.value_entries = {}       # Campos de entrada
        self.value_entry_labels = {}  # Labels de estado
        
        # Crear controles para cada motor
        for motor_id in self.controller.dxl_ids:
            motor_card = QFrame()
            motor_card.setObjectName("card")
            motor_layout = QHBoxLayout(motor_card)
            
            # Nombre del motor
            motor_title = QLabel(f"Motor {motor_id}")
            motor_title.setObjectName("motorLabel")
            motor_title.setMinimumWidth(100)
            motor_layout.addWidget(motor_title)
            
            # Label de rango
            entry_label = QLabel(f"Valor (0-{DXL_MAX_VALUE}):")
            motor_layout.addWidget(entry_label)
            
            # Campo de entrada
            entry = QLineEdit(str(DEFAULT_GOAL))
            entry.setMaximumWidth(120)
            # Enter mueve el motor
            entry.returnPressed.connect(lambda mid=motor_id: self.move_single_motor_from_entry(mid))
            motor_layout.addWidget(entry)
            
            # Botón mover
            move_btn = QPushButton("Mover")
            move_btn.clicked.connect(lambda checked, mid=motor_id: self.move_single_motor_from_entry(mid))
            motor_layout.addWidget(move_btn)
            
            # Label de estado
            status_label = QLabel("Listo")
            status_label.setStyleSheet("color: #00d9ff; font-weight: bold;")
            motor_layout.addWidget(status_label)
            
            motor_layout.addStretch()
            scroll_layout.addWidget(motor_card)
            
            # Guardar referencias
            self.value_entries[motor_id] = entry
            self.value_entry_labels[motor_id] = status_label
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)
        
        # Botón para mover todos los motores a la vez
        move_all_btn = QPushButton("▶ MOVER TODOS LOS MOTORES")
        move_all_btn.setStyleSheet("font-size: 13pt; padding: 15px;")
        move_all_btn.clicked.connect(self.move_all_motors_from_entries)
        layout.addWidget(move_all_btn)
        
        return page
    
    def create_fixed_angles_page(self):
        """
        Crea la página con posiciones predefinidas (presets).
        Permite mover el robot a configuraciones comunes con un click.
        
        Returns:
            QWidget: Página de ángulos predefinidos
        """
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Movimientos a Ángulos Predefinidos")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        subtitle = QLabel("Posiciones predefinidas con un solo clic")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)
        
        # Definición de presets: (nombre, [ángulos], descripción)
        presets = [
            ("Posición 1: HOME", [0, 0, 0, 0, 0], "Todos los motores en 0°"),
            ("Posición 2: Alcance Medio", [25, 25, 20, -20, 0], "Configuración de alcance medio"),
            ("Posición 3: Lateral", [-35, 35, -30, 30, 0], "Movimiento lateral"),
            ("Posición 4: Elevada", [85, -20, 55, 25, 0], "Posición elevada"),
            ("Posición 5: Extendida", [80, -35, 55, -45, 0], "Máxima extensión"),
        ]
        
        # Grid para organizar las tarjetas
        grid = QGridLayout()
        grid.setSpacing(20)
        
        for i, (name, angles, desc) in enumerate(presets):
            # Tarjeta para cada preset
            card = QFrame()
            card.setObjectName("card")
            card_layout = QVBoxLayout(card)
            
            # Título del preset
            title_label = QLabel(name)
            title_label.setObjectName("sectionTitle")
            card_layout.addWidget(title_label)
            
            # Descripción
            desc_label = QLabel(desc)
            desc_label.setStyleSheet("color: #b0b0c0; padding: 5px;")
            desc_label.setWordWrap(True)
            card_layout.addWidget(desc_label)
            
            # Mostrar los ángulos
            angles_str = f"[{', '.join(f'{a}°' for a in angles)}]"
            angles_label = QLabel(angles_str)
            angles_label.setStyleSheet("color: #00d9ff; font-weight: bold; padding: 10px; font-size: 12pt;")
            card_layout.addWidget(angles_label)
            
            # Botón de ejecución
            btn = QPushButton("▶ Ejecutar Movimiento")
            btn.setObjectName("presetButton")
            btn.clicked.connect(lambda checked, a=angles: self.move_to_fixed_angles(a))
            card_layout.addWidget(btn)
            
            # Posicionar en el grid (2 columnas)
            row = i // 2
            col = i % 2
            grid.addWidget(card, row, col)
        
        layout.addLayout(grid)
        layout.addStretch()
        
        return page
    
    def create_xyz_page(self):
        """
        Crea la página de control XYZ (cinemática inversa).
        Permite mover el end-effector a posiciones cartesianas.
        
        Returns:
            QWidget: Página de control XYZ
        """
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Control por Posición XYZ")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        # ============================================================
        #  TARJETA: POSICIÓN ACTUAL
        # ============================================================
        current_card = QFrame()
        current_card.setObjectName("card")
        current_layout = QVBoxLayout(current_card)
        
        current_title = QLabel("Posición Actual del End-Effector")
        current_title.setObjectName("sectionTitle")
        current_layout.addWidget(current_title)
        
        # Labels para X, Y, Z actual
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
        
        # ============================================================
        #  TARJETA: POSICIÓN OBJETIVO
        # ============================================================
        target_card = QFrame()
        target_card.setObjectName("card")
        target_layout = QVBoxLayout(target_card)
        
        target_title = QLabel("Posición Objetivo")
        target_title.setObjectName("sectionTitle")
        target_layout.addWidget(target_title)
        
        # Campos de entrada para X, Y, Z
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
            # Guardar referencia al entry como atributo de la clase
            setattr(self, attr_name, entry)
        
        target_layout.addLayout(inputs_layout)
        
        # Botón para mover a posición XYZ
        move_btn = QPushButton("🎯 MOVER A POSICIÓN XYZ")
        move_btn.setFixedHeight(50)
        move_btn.setStyleSheet("font-size: 13pt;")
        move_btn.clicked.connect(self.move_to_xyz_target)
        target_layout.addWidget(move_btn)
        
        layout.addWidget(target_card)
        
        # ============================================================
        #  TARJETA: POSICIONES RÁPIDAS
        # ============================================================
        presets_card = QFrame()
        presets_card.setObjectName("card")
        presets_layout = QVBoxLayout(presets_card)
        
        presets_title = QLabel("Posiciones Rápidas")
        presets_title.setObjectName("sectionTitle")
        presets_layout.addWidget(presets_title)
        
        # Grid de botones preset
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
        
        # Label de estado de IK
        self.ik_status_label = QLabel("✓ Listo para mover")
        self.ik_status_label.setStyleSheet("color: #00d9ff; font-size: 12pt; padding: 10px;")
        layout.addWidget(self.ik_status_label)
        
        layout.addStretch()
        
        return page

    # ============================================================
    #  PÁGINA DE OPENCV
    # ============================================================
    
    def create_opencv_page(self):
        """
        Crea la página de OpenCV para visión y pick & place.
        Incluye vista de cámara, información de detección y botones de control.
        
        Returns:
            QWidget: Página de OpenCV
        """
        page = QWidget()
        layout = QVBoxLayout(page)

        title = QLabel("OpenCV")
        title.setObjectName("pageTitle")
        layout.addWidget(title)

        subtitle = QLabel("Vista de cámara + captura 1-shot + rutina por figura")
        subtitle.setObjectName("subtitle")
        layout.addWidget(subtitle)

        # ============================================================
        #  TARJETA: VISTA DE CÁMARA
        # ============================================================
        video_card = QFrame()
        video_card.setObjectName("card")
        video_layout = QVBoxLayout(video_card)

        # Label donde se mostrará el video
        self.camera_label = QLabel("Cámara no iniciada")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumHeight(360)
        self.camera_label.setStyleSheet("background-color: #111; border-radius: 10px;")
        video_layout.addWidget(self.camera_label)

        layout.addWidget(video_card)

        # ============================================================
        #  TARJETA: INFORMACIÓN DE DETECCIÓN
        # ============================================================
        info_card = QFrame()
        info_card.setObjectName("card")
        info_layout = QHBoxLayout(info_card)

        # Label: Figura detectada
        self.opencv_shape_label = QLabel("Figura: -")
        self.opencv_shape_label.setStyleSheet("color: #00d9ff; font-weight: bold; font-size: 14pt;")
        info_layout.addWidget(self.opencv_shape_label)

        # Label: Coordenadas X, Y
        self.opencv_xy_label = QLabel("x,y: -")
        self.opencv_xy_label.setStyleSheet("color: #00d9ff; font-weight: bold; font-size: 14pt;")
        info_layout.addWidget(self.opencv_xy_label)

        # Label: Confianza de la detección
        self.opencv_conf_label = QLabel("conf: -")
        self.opencv_conf_label.setStyleSheet("color: #b0b0c0; font-weight: bold; font-size: 12pt;")
        info_layout.addWidget(self.opencv_conf_label)

        info_layout.addStretch()
        layout.addWidget(info_card)

        # ============================================================
        #  BOTONES DE ACCIÓN
        # ============================================================
        btn_row = QHBoxLayout()

        # Botón: Ir a HOME de OpenCV
        self.btn_home_opencv = QPushButton("🏠 Home OpenCV")
        self.btn_home_opencv.clicked.connect(self.home_opencv)
        self.btn_home_opencv.clicked.connect(self.controller.gripper_open)  # También abre gripper
        btn_row.addWidget(self.btn_home_opencv)

        # Botón: Capturar dato (una sola lectura)
        self.btn_capture_opencv = QPushButton("📌 Capturar dato OpenCV (1 lectura)")
        self.btn_capture_opencv.clicked.connect(self.capture_opencv_data_once)
        btn_row.addWidget(self.btn_capture_opencv)

        # Botón: Ejecutar rutina de pick & place
        self.btn_run_routine = QPushButton("🤖 Ejecutar rutina (usa dato guardado)")
        self.btn_run_routine.clicked.connect(self.run_saved_routine)
        btn_row.addWidget(self.btn_run_routine)

        layout.addLayout(btn_row)
        layout.addStretch()
        return page

    def update_camera_frame(self):
        """
        Callback del timer de cámara. Captura un frame, ejecuta detección
        de figuras, dibuja overlay visual y muestra en la GUI.
        """
        try:
            # Verificar que la cámara está abierta
            if not self.cap or not self.cap.isOpened():
                self.camera_label.setText("❌ No se pudo abrir la cámara")
                return

            # Capturar frame
            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.camera_label.setText("⚠ No llegó frame (ret=False)")
                return

            # Guardar frame para uso posterior (captura 1-shot)
            self.last_frame = frame.copy()

            # ============================================================
            #  PROCESAR Y DIBUJAR OVERLAY DE DETECCIÓN
            # ============================================================
            frame_vis = frame.copy()

            # Ejecutar detección de figuras
            info = detectar_figuras_y_polares(frame_vis, debug=False)
            best = pick_best_detection(info)

            if best is not None:
                # Extraer información de la mejor detección
                shape_name = best["shape_name"]
                conf = best["confidence"]
                r_cm = best["r_cm"]
                theta_deg = best["theta_deg"]

                # Dibujar contorno del disco (referencia)
                if "disk_cnt" in best:
                    cv2.drawContours(frame_vis, [best["disk_cnt"]], -1, (0, 255, 255), 2)

                # Dibujar contorno de la figura detectada
                if "cnt" in best:
                    cv2.drawContours(frame_vis, [best["cnt"]], -1, (0, 255, 0), 2)

                # Dibujar centro del disco
                if "disk_center_px" in best:
                    dcx, dcy = best["disk_center_px"]
                    cv2.circle(frame_vis, (int(dcx), int(dcy)), 4, (0, 255, 255), -1)

                # Dibujar centro de la figura y línea al disco
                if "center_px" in best:
                    cx, cy = best["center_px"]
                    cv2.circle(frame_vis, (int(cx), int(cy)), 4, (0, 0, 255), -1)

                    if "disk_center_px" in best:
                        cv2.line(frame_vis, (int(dcx), int(dcy)), (int(cx), int(cy)), (255, 0, 0), 2)

                # Calcular coordenadas en el frame del robot
                x_cv, y_cv = polar_to_cartesian_m(r_cm, theta_deg, offset_x_m=0.0, offset_y_m=0.0)
                x_robot, y_robot = y_cv, x_cv  # Swap de ejes
                y_robot = -y_robot              # Invertir Y
                x_m = x_robot + OPENCV_OFFSET_X_M
                y_m = y_robot + OPENCV_OFFSET_Y_M

                # Mostrar texto con información
                txt1 = f"{shape_name} ({shape_name_to_code(shape_name)})  conf={conf:.0%}"
                txt2 = f"r={r_cm:.1f}cm  th={theta_deg:.1f}deg"
                txt3 = f"x={x_m:.3f}m  y={y_m:.3f}m"

                cv2.putText(frame_vis, txt1, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                cv2.putText(frame_vis, txt2, (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                cv2.putText(frame_vis, txt3, (15, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            # ============================================================
            #  CONVERTIR A FORMATO QT Y MOSTRAR
            # ============================================================
            # Convertir de BGR (OpenCV) a RGB (Qt)
            rgb = cv2.cvtColor(frame_vis, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            
            # Crear QImage desde los datos
            qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pix = QPixmap.fromImage(qimg)

            # Escalar y mostrar en el label
            self.camera_label.setPixmap(
                pix.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )

        except Exception as e:
            self.camera_label.setText(f"❌ Error cámara: {e}")

    def home_opencv(self):
        """
        Mueve el robot a la posición HOME específica de OpenCV.
        """
        self.controller.go_home_opencv()
        self.status_label.setText("● HOME OpenCV (por ahora: HOME general)")
        # Restaurar mensaje después de 2.5 segundos
        QTimer.singleShot(2500, lambda: self.status_label.setText("● Sistema Listo"))

    def capture_opencv_data_once(self):
        """
        Captura una sola lectura de OpenCV usando el último frame disponible.
        Detecta la figura, calcula coordenadas y guarda para uso posterior.
        """
        # Verificar que hay un frame disponible
        if self.last_frame is None:
            QMessageBox.warning(self, "OpenCV", "No hay frame disponible todavía.")
            return

        # Ejecutar detección
        info = detectar_figuras_y_polares(self.last_frame, debug=False)
        best = pick_best_detection(info)

        # Si no se detectó nada
        if best is None:
            self.opencv_shape_label.setText("Figura: u")
            self.opencv_xy_label.setText("x,y: -")
            self.opencv_conf_label.setText("conf: -")
            QMessageBox.warning(self, "OpenCV", "No se detectó ninguna figura.")
            return

        # Extraer información
        shape_name = best["shape_name"]
        conf = best["confidence"]
        r_cm = best["r_cm"]
        theta_deg = best["theta_deg"]

        # Convertir nombre a código
        shape_code = shape_name_to_code(shape_name)

        # ============================================================
        #  TRANSFORMACIÓN DE COORDENADAS
        # ============================================================
        
        # 1) Convertir de polar a cartesiano (sin offset aún)
        x_cv, y_cv = polar_to_cartesian_m(
            r_cm, theta_deg,
            offset_x_m=0.0,
            offset_y_m=0.0,
        )

        # 2) Intercambiar ejes (corrección de orientación cámara-robot)
        x_robot, y_robot = y_cv, x_cv

        # 3) Invertir eje Y del robot
        y_robot = -y_robot

        # 4) Aplicar offsets de calibración
        x_m = x_robot + OPENCV_OFFSET_X_M
        y_m = y_robot + OPENCV_OFFSET_Y_M

        # Altura fija para pick
        z_m = Z_PICK_M

        # Publicar datos a ROS2 y guardar internamente
        self.controller.publish_opencv_measurement(shape_code, x_m, y_m, z_m)

        # Actualizar UI
        self.opencv_shape_label.setText(f"Figura: {shape_code} ({shape_name})")
        self.opencv_xy_label.setText(f"x,y: {x_m:.3f}, {y_m:.3f} m")
        self.opencv_conf_label.setText(f"conf: {conf:.0%}")

        self.status_label.setText("● Dato OpenCV capturado y publicado")
        QTimer.singleShot(2500, lambda: self.status_label.setText("● Sistema Listo"))

    def run_saved_routine(self):
        """
        Ejecuta la rutina de pick & place usando el dato guardado previamente.
        La rutina se ejecuta en un hilo separado para no bloquear la GUI.
        """
        # Verificar que hay un dato guardado
        if self.controller.last_opencv_measurement is None:
            QMessageBox.warning(self, "Rutina", "Primero captura el dato con 'Capturar dato OpenCV'.")
            return

        shape_code, x_m, y_m, z_m = self.controller.last_opencv_measurement
        
        # Verificar que no está en zona restringida
        zone = restricted_zone_name(x_m, y_m)
        if zone is not None:
            msg = (
                f"Posición restringida ({zone}).\n"
                f"x={x_m:.3f} m, y={y_m:.3f} m\n\n"
                "La posición no puede ser alcanzada.\n"
                "No se ejecutará la rutina de recogida."
            )
            QMessageBox.warning(self, "Zona restringida", msg)
            self.status_label.setText("● Posición en zona restringida (rutina cancelada)")
            QTimer.singleShot(3000, lambda: self.status_label.setText("● Sistema Listo"))
            return

        # Preparar UI para ejecución
        self.status_label.setText("● Ejecutando rutina...")
        self.btn_run_routine.setEnabled(False)  # Deshabilitar botón durante ejecución

        def worker():
            """Función que se ejecuta en hilo separado."""
            ok = False
            try:
                ok = self.controller.run_pick_place(shape_code, x_m, y_m, z_m)
            except Exception as e:
                print("❌ Excepción en worker rutina:", e)
                ok = False
            finally:
                # Emitir señal para actualizar GUI de forma thread-safe
                self.routine_finished_sig.emit(ok)

        # Iniciar hilo
        threading.Thread(target=worker, daemon=True).start()

    def on_routine_finished(self, ok: bool):
        """
        Callback que se ejecuta cuando la rutina termina.
        Actualiza la UI según el resultado.
        
        Args:
            ok: True si la rutina fue exitosa
        """
        self.btn_run_routine.setEnabled(True)  # Re-habilitar botón
        if ok:
            self.status_label.setText("● Rutina completada")
        else:
            self.status_label.setText("● Rutina falló")
        QTimer.singleShot(3000, lambda: self.status_label.setText("● Sistema Listo"))

    def create_visualization_page(self):
        """
        Crea la página de visualización.
        Permite lanzar RViz y Robotics Toolbox para visualizar el robot.
        
        Returns:
            QWidget: Página de visualización
        """
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Visualización")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        # ============================================================
        #  TARJETA: RVIZ
        # ============================================================
        rviz_card = QFrame()
        rviz_card.setObjectName("card")
        rviz_layout = QVBoxLayout(rviz_card)
        
        rviz_title = QLabel("RViz - Modelo 3D")
        rviz_title.setObjectName("sectionTitle")
        rviz_layout.addWidget(rviz_title)
        
        # Botón lanzar RViz
        self.rviz_btn = QPushButton("▶ LANZAR RViz")
        self.rviz_btn.setFixedHeight(60)
        self.rviz_btn.clicked.connect(self.launch_rviz)
        rviz_layout.addWidget(self.rviz_btn)
        
        # Botón detener RViz
        self.stop_rviz_btn = QPushButton("⏹ DETENER RViz")
        self.stop_rviz_btn.setEnabled(False)  # Deshabilitado inicialmente
        self.stop_rviz_btn.setFixedHeight(60)
        self.stop_rviz_btn.clicked.connect(self.stop_rviz)
        rviz_layout.addWidget(self.stop_rviz_btn)
        
        # Label de estado
        self.rviz_status_label = QLabel("● RViz no iniciado")
        self.rviz_status_label.setStyleSheet("color: #ff416c; font-weight: bold; padding: 15px; font-size: 12pt;")
        rviz_layout.addWidget(self.rviz_status_label)
        
        layout.addWidget(rviz_card)
        
        # ============================================================
        #  TARJETA: ROBOTICS TOOLBOX
        # ============================================================
        toolbox_card = QFrame()
        toolbox_card.setObjectName("card")
        toolbox_layout = QVBoxLayout(toolbox_card)
        
        toolbox_title = QLabel("Robotics Toolbox - Simulación")
        toolbox_title.setObjectName("sectionTitle")
        toolbox_layout.addWidget(toolbox_title)
        
        # Botón lanzar Toolbox
        self.toolbox_btn = QPushButton("▶ LANZAR Toolbox")
        self.toolbox_btn.setFixedHeight(60)
        self.toolbox_btn.clicked.connect(self.launch_toolbox)
        toolbox_layout.addWidget(self.toolbox_btn)
        
        # Botón detener Toolbox
        self.stop_toolbox_btn = QPushButton("⏹ DETENER Toolbox")
        self.stop_toolbox_btn.setEnabled(False)
        self.stop_toolbox_btn.setFixedHeight(60)
        self.stop_toolbox_btn.clicked.connect(self.stop_toolbox)
        toolbox_layout.addWidget(self.stop_toolbox_btn)
        
        # Label de estado
        self.toolbox_status_label = QLabel("● Toolbox no iniciado")
        self.toolbox_status_label.setStyleSheet("color: #ff416c; font-weight: bold; padding: 15px; font-size: 12pt;")
        toolbox_layout.addWidget(self.toolbox_status_label)
        
        layout.addWidget(toolbox_card)
        layout.addStretch()
        
        return page
    
    def create_info_page(self):
        """
        Crea la página de información del proyecto.
        Muestra créditos del equipo de desarrollo.
        
        Returns:
            QWidget: Página de información
        """
        page = QWidget()
        layout = QVBoxLayout(page)
        
        title = QLabel("Información del Proyecto")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        
        # Área scrolleable para la lista de desarrolladores
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        # Tarjeta: Desarrollador 1
        card1 = QFrame()
        card1.setObjectName("card")
        layout1 = QVBoxLayout(card1)
        
        name1 = QLabel("Samuel David Sanchez Cardenas")
        name1.setObjectName("sectionTitle")
        layout1.addWidget(name1)
        
        info1 = QLabel("⏰ Veremos si acaba primero el semestre o el semestre acaba conmigo\n🔗 GitHub: samsanchezcar")
        info1.setStyleSheet("color: #e0e0e0; line-height: 1; padding: 8px;")
        layout1.addWidget(info1)
        
        scroll_layout.addWidget(card1)
        
        # Tarjeta: Desarrollador 2
        card2 = QFrame()
        card2.setObjectName("card")
        layout2 = QVBoxLayout(card2)
        
        name2 = QLabel("Santiago Ávila Corredor")
        name2.setObjectName("sectionTitle")
        layout2.addWidget(name2)
        
        info2 = QLabel("🙏 Pronto todo acabará\n🔗 GitHub: Santiago-Avila")
        info2.setStyleSheet("color: #e0e0e0; line-height: 1; padding: 8px;")
        layout2.addWidget(info2)
        
        scroll_layout.addWidget(card2)

        # Tarjeta: Desarrollador 3
        card3 = QFrame()
        card3.setObjectName("card")
        layout3 = QVBoxLayout(card3)
        
        name3 = QLabel("Santiago Mariño Cortés")
        name3.setObjectName("sectionTitle")
        layout3.addWidget(name3)
        
        info3 = QLabel("🙊 Don't believe the hype\n🔗 GitHub: mrbrightside8")
        info3.setStyleSheet("color: #e0e0e0; line-height: 1; padding: 8px;")
        layout3.addWidget(info3)
        
        scroll_layout.addWidget(card3)

        # Tarjeta: Desarrollador 4
        card4 = QFrame()
        card4.setObjectName("card")
        layout4 = QVBoxLayout(card4)
        
        name4 = QLabel("Juan Angel Vargas Rodríguez")
        name4.setObjectName("sectionTitle")
        layout4.addWidget(name4)
        
        info4 = QLabel("🙏 Agradecido con el de arriba\n🔗 GitHub: juvargasro")
        info4.setStyleSheet("color: #e0e0e0; line-height: 1; padding: 8px;")
        layout4.addWidget(info4)
        
        scroll_layout.addWidget(card4)

        # Tarjeta: Desarrollador 5
        card5 = QFrame()
        card5.setObjectName("card")
        layout5 = QVBoxLayout(card5)
        
        name5 = QLabel("Juan José Delgado Estrada")
        name5.setObjectName("sectionTitle")
        layout5.addWidget(name5)
        
        info5 = QLabel("🤖 Mi propósito es tener un robot que me hagas el aseo\n🔗 GitHub: Juan-delgado1")
        info5.setStyleSheet("color: #e0e0e0; line-height: 1; padding: 8px;")
        layout5.addWidget(info5)
        
        scroll_layout.addWidget(card5)
        
        scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)
        
        return page
    
    def create_card(self, title, content_widget):
        """
        Crea una tarjeta (card) con título y contenido.
        Helper para crear secciones consistentes en la UI.
        
        Args:
            title: Título de la tarjeta
            content_widget: Widget con el contenido
        
        Returns:
            QFrame: Widget de la tarjeta
        """
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        
        title_label = QLabel(title)
        title_label.setObjectName("sectionTitle")
        layout.addWidget(title_label)
        
        layout.addWidget(content_widget)
        
        return card
    
    def create_position_widget(self):
        """
        Crea el widget que muestra la posición XYZ actual en el dashboard.
        
        Returns:
            QWidget: Widget con labels de posición
        """
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Labels para X, Y, Z
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
        """
        Crea el widget que muestra el estado de los motores en el dashboard.
        
        Returns:
            QWidget: Widget con estado de motores
        """
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Un label por cada motor
        for i, motor_id in enumerate(self.controller.dxl_ids):
            motor_label = QLabel(f"Motor {motor_id}: ● Activo")
            motor_label.setStyleSheet("color: #00d9ff; padding: 5px;")
            layout.addWidget(motor_label)
        
        return widget
    
    def create_speed_widget(self):
        """
        Crea el widget de control de velocidad para el dashboard.
        
        Returns:
            QWidget: Widget con slider de velocidad
        """
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        slider_layout = QHBoxLayout()
        
        # Slider de velocidad global
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(MAX_SPEED)
        self.speed_slider.setValue(100)  # Velocidad inicial
        self.speed_slider.valueChanged.connect(self.on_speed_change)
        slider_layout.addWidget(self.speed_slider, 3)
        
        # Label con valor actual
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
        """
        Callback cuando la posición de un motor cambia (desde otra pestaña o controlador).
        Actualiza los sliders sin disparar eventos de cambio.
        
        Args:
            motor_id: ID del motor que cambió
            position: Nueva posición
        """
        if motor_id in self.sliders:
            # Bloquear señal para evitar loop infinito
            self.sliders[motor_id].blockSignals(True)
            self.sliders[motor_id].setValue(position)
            self.slider_labels[motor_id].setText(str(position))
            self.sliders[motor_id].blockSignals(False)
    
    def on_motor_slider_change(self, motor_id):
        """
        Callback cuando se mueve un slider de motor.
        Implementa rate-limiting para no saturar la comunicación.
        
        Args:
            motor_id: ID del motor cuyo slider cambió
        """
        current_time = time.time()
        
        # Verificar rate-limiting
        if current_time - self.last_motor_update[motor_id] >= self.update_interval:
            position = self.sliders[motor_id].value()
            speed = self.speed_slider.value()
            
            # Actualizar label de valor
            self.slider_labels[motor_id].setText(str(position))
            
            # Solo mover si velocidad > 0 y no hay emergencia
            if speed > 0 and not self.controller.emergency_stop_activated:
                self.controller.move_motor(motor_id, position)
                self.status_label.setText(f"● Motor {motor_id} → {position}")
                self.last_motor_update[motor_id] = current_time
                QTimer.singleShot(2000, lambda: self.status_label.setText("● Sistema Listo"))
    
    def on_speed_change(self, value):
        """
        Callback cuando cambia el slider de velocidad.
        Actualiza la velocidad de todos los motores.
        
        Args:
            value: Nueva velocidad
        """
        self.controller.update_speed(value)
        self.speed_value_label.setText(str(value))
    
    def move_single_motor_from_entry(self, motor_id):
        """
        Mueve un motor individual usando el valor del campo de entrada.
        
        Args:
            motor_id: ID del motor a mover
        """
        try:
            # Obtener valor del campo
            value = self.value_entries[motor_id].text()
            position = int(value)
            
            # Validar rango
            if 0 <= position <= DXL_MAX_VALUE:
                speed = self.speed_slider.value()
                
                if speed == 0:
                    self.value_entry_labels[motor_id].setText("Velocidad 0")
                    self.value_entry_labels[motor_id].setStyleSheet("color: orange; font-weight: bold;")
                elif self.controller.emergency_stop_activated:
                    self.value_entry_labels[motor_id].setText("EMERGENCIA")
                    self.value_entry_labels[motor_id].setStyleSheet("color: red; font-weight: bold;")
                else:
                    # Mover motor
                    self.controller.move_motor(motor_id, position)
                    self.value_entry_labels[motor_id].setText("Enviado")
                    self.value_entry_labels[motor_id].setStyleSheet("color: blue; font-weight: bold;")
                    self.status_label.setText(f"● Motor {motor_id} → {position}")
                    
                    # Restaurar después de 2 segundos
                    QTimer.singleShot(2000, lambda: self.value_entry_labels[motor_id].setText("Listo"))
                    QTimer.singleShot(2000, lambda: self.value_entry_labels[motor_id].setStyleSheet("color: #00d9ff; font-weight: bold;"))
            else:
                self.value_entry_labels[motor_id].setText(f"Error: 0-{DXL_MAX_VALUE}")
                self.value_entry_labels[motor_id].setStyleSheet("color: red; font-weight: bold;")
        except ValueError:
            self.value_entry_labels[motor_id].setText("Error: Número")
            self.value_entry_labels[motor_id].setStyleSheet("color: red; font-weight: bold;")
    
    def move_all_motors_from_entries(self):
        """
        Mueve todos los motores a los valores especificados en los campos de entrada.
        """
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
        """
        Mueve el robot a una configuración predefinida de ángulos.
        
        Args:
            angles: Lista de 5 ángulos en grados
        """
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
        """
        Mueve el end-effector a la posición XYZ especificada en los campos de entrada.
        """
        try:
            # Obtener coordenadas de los campos
            x = float(self.x_entry.text())
            y = float(self.y_entry.text())
            z = float(self.z_entry.text())
            
            # Validar alcance
            r = (x**2 + y**2) ** 0.5
            
            if not (self.controller.planar_reach_min <= r <= self.controller.planar_reach_max + 0.01):
                QMessageBox.warning(self, "Error", f"Fuera de alcance radial: {r:.3f} m")
                return
            
            if not (self.controller.z_min - 0.01 <= z <= self.controller.z_max + 0.01):
                QMessageBox.warning(self, "Error", f"Fuera de alcance en Z: {z:.3f} m")
                return
            
            # Mostrar estado "calculando"
            self.ik_status_label.setText(f"⏳ Calculando IK...")
            QApplication.processEvents()  # Forzar actualización de UI
            
            # Ejecutar movimiento
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
        """
        Mueve a una posición XYZ predefinida (botones de preset).
        
        Args:
            x, y, z: Coordenadas del preset
        """
        # Llenar campos con valores del preset
        self.x_entry.setText(f"{x:.3f}")
        self.y_entry.setText(f"{y:.3f}")
        self.z_entry.setText(f"{z:.3f}")
        # Ejecutar movimiento
        self.move_to_xyz_target()
    
    def update_xyz_display(self):
        """
        Callback del timer para actualizar el display de posición XYZ.
        Se ejecuta periódicamente para mostrar la posición actual del end-effector.
        """
        try:
            # Obtener posición actual
            x, y, z = self.controller.get_current_xyz()
            
            # Actualizar labels del dashboard (si existen)
            if hasattr(self, 'dash_x_label'):
                self.dash_x_label.setText(f"X: {x:.3f} m")
                self.dash_y_label.setText(f"Y: {y:.3f} m")
                self.dash_z_label.setText(f"Z: {z:.3f} m")
            
            # Actualizar labels de la página XYZ (si existen)
            if hasattr(self, 'current_x_label'):
                self.current_x_label.setText(f"X: {x:.3f} m")
                self.current_y_label.setText(f"Y: {y:.3f} m")
                self.current_z_label.setText(f"Z: {z:.3f} m")
        except:
            pass  # Ignorar errores si los widgets no están disponibles
    
    def launch_rviz(self):
        """
        Lanza RViz2 en un proceso separado para visualizar el modelo del robot.
        """
        try:
            # Comando para lanzar RViz con el archivo de configuración del robot
            cmd = ["ros2", "launch", "phantomx_pincher_description", "display.launch.py"]

            # Limpiar variables de entorno de Qt que pueden causar conflictos
            # (OpenCV de pip a veces inyecta paths de plugins Qt incompatibles)
            clean_env = os.environ.copy()
            for k in [
                "QT_QPA_PLATFORM_PLUGIN_PATH",
                "QT_PLUGIN_PATH",
                "QT_QPA_PLATFORMTHEME",
                "QT_DEBUG_PLUGINS",
            ]:
                clean_env.pop(k, None)

            def run_rviz():
                """Función que se ejecuta en hilo separado."""
                self.rviz_process = subprocess.Popen(cmd, env=clean_env)
                self.rviz_process.wait()  # Esperar a que termine
                QTimer.singleShot(0, self.on_rviz_closed)  # Notificar cierre

            # Iniciar hilo
            thread = threading.Thread(target=run_rviz, daemon=True)
            thread.start()

            # Actualizar UI
            self.rviz_btn.setEnabled(False)
            self.stop_rviz_btn.setEnabled(True)
            self.rviz_status_label.setText("● RViz ejecutándose")
            self.rviz_status_label.setStyleSheet(
                "color: #00d9ff; font-weight: bold; padding: 15px; font-size: 12pt;"
            )

        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo lanzar RViz: {str(e)}")

    
    def stop_rviz(self):
        """
        Detiene el proceso de RViz.
        """
        if self.rviz_process:
            try:
                self.rviz_process.terminate()
                self.rviz_process = None
            except:
                pass
        self.on_rviz_closed()
    
    def on_rviz_closed(self):
        """
        Callback cuando RViz se cierra (manual o automáticamente).
        Restaura el estado de la UI.
        """
        self.rviz_btn.setEnabled(True)
        self.stop_rviz_btn.setEnabled(False)
        self.rviz_status_label.setText("● RViz no iniciado")
        self.rviz_status_label.setStyleSheet("color: #ff416c; font-weight: bold; padding: 15px; font-size: 12pt;")
    
    def launch_toolbox(self):
        """
        Lanza el visualizador de Robotics Toolbox en un proceso separado.
        """
        try:
            # Buscar el script toolbox.py en el mismo directorio
            toolbox_path = os.path.join(os.path.dirname(__file__), 'toolbox.py')
            
            if not os.path.exists(toolbox_path):
                QMessageBox.warning(self, "Error", f"No se encontró toolbox.py en:\n{toolbox_path}")
                return
            
            def run_toolbox():
                """Función que se ejecuta en hilo separado."""
                self.toolbox_process = subprocess.Popen(['python3', toolbox_path])
                self.toolbox_process.wait()
                QTimer.singleShot(0, self.on_toolbox_closed)
            
            # Iniciar hilo
            thread = threading.Thread(target=run_toolbox, daemon=True)
            thread.start()
            
            # Actualizar UI
            self.toolbox_btn.setEnabled(False)
            self.stop_toolbox_btn.setEnabled(True)
            self.toolbox_status_label.setText("● Toolbox ejecutándose")
            self.toolbox_status_label.setStyleSheet("color: #00d9ff; font-weight: bold; padding: 15px; font-size: 12pt;")
        
        except Exception as e:
            QMessageBox.critical(self, "Error", f"No se pudo lanzar Toolbox: {str(e)}")
    
    def stop_toolbox(self):
        """
        Detiene el proceso de Robotics Toolbox.
        """
        if self.toolbox_process:
            try:
                self.toolbox_process.terminate()
                self.toolbox_process = None
            except:
                pass
        self.on_toolbox_closed()
    
    def on_toolbox_closed(self):
        """
        Callback cuando Toolbox se cierra.
        Restaura el estado de la UI.
        """
        self.toolbox_btn.setEnabled(True)
        self.stop_toolbox_btn.setEnabled(False)
        self.toolbox_status_label.setText("● Toolbox no iniciado")
        self.toolbox_status_label.setStyleSheet("color: #ff416c; font-weight: bold; padding: 15px; font-size: 12pt;")
    
    def home_all(self):
        """
        Mueve todos los motores a la posición HOME (0 grados).
        Si hay parada de emergencia, pregunta si reactivar.
        """
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
        """
        Activa la parada de emergencia.
        Desactiva torque de todos los motores.
        """
        self.controller.emergency_stop()
        self.status_label.setText("⚠ PARADA DE EMERGENCIA ACTIVADA")
        self.status_label.setStyleSheet("background-color: #ff416c; color: white; font-weight: bold; padding: 10px; border-radius: 8px;")
    
    def closeEvent(self, event):
        """
        Evento de cierre de la ventana.
        Limpia recursos y pregunta confirmación al usuario.
        
        Args:
            event: Evento de cierre de Qt
        """
        # Detener procesos externos si están corriendo
        if self.rviz_process:
            self.stop_rviz()
        if self.toolbox_process:
            self.stop_toolbox()
        
        # Pedir confirmación
        reply = QMessageBox.question(self, "Salir",
            "¿Cerrar la aplicación?\nSe desactivará el torque.",
            QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # Liberar cámara
            try:
                if hasattr(self, "camera_timer") and self.camera_timer.isActive():
                    self.camera_timer.stop()
                if hasattr(self, "cap") and self.cap:
                    self.cap.release()
            except:
                pass
            
            # Cerrar controlador
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
    # Inicializar ROS2
    rclpy.init(args=args)
    
    # Crear el controlador del robot
    controller = PincherController()
    
    # Crear hilo para el spin de ROS2
    # (procesa callbacks, mensajes, etc.)
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(controller,),
        daemon=True  # Se termina automáticamente cuando el programa principal termina
    )
    spin_thread.start()
    
    # Crear aplicación Qt
    app = QApplication(sys.argv)
    app.setStyleSheet(MODERN_STYLESHEET)  # Aplicar estilos globales
    
    try:
        # Crear y mostrar la GUI
        gui = ModernPincherGUI(controller)
        gui.show()
        
        # Ejecutar loop de eventos de Qt (bloquea hasta que se cierre la ventana)
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass  # Ctrl+C
    finally:
        # Limpieza final
        controller.close()
        controller.destroy_node()
        rclpy.shutdown()

# Punto de entrada cuando se ejecuta el script directamente
if __name__ == '__main__':
    main()