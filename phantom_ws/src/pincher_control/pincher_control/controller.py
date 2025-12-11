"""
Nodo ROS2 para control del robot PhantomX Pincher.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from .config import (
    PROTOCOL_VERSION, ADDR_TORQUE_ENABLE, DEFAULT_GOAL, DXL_MAX_VALUE,
    DEFAULT_PORT, DEFAULT_BAUDRATE, DEFAULT_DXL_IDS,
    DEFAULT_MOVING_SPEED, DEFAULT_TORQUE_LIMIT,
    JOINT_NAMES, JOINT_SIGN,
    PLANAR_REACH_MAX, PLANAR_REACH_MIN, Z_MAX, Z_MIN
)
from .utils import (
    write_goal_position, write_moving_speed,
    dxl_to_radians, radians_to_dxl, degrees_to_dxl
)
from .kinematics import (
    build_pincher_robot, forward_kinematics, inverse_kinematics,
    check_workspace_limits
)


class PincherController(Node):
    """Nodo ROS2 para controlar el robot PhantomX Pincher."""
    
    # Señal para actualizar GUI (se asigna desde la GUI)
    position_changed = None
    
    def __init__(self):
        super().__init__('pincher_controller')
        
        # Modelo cinemático
        self.robot_model = build_pincher_robot()
        self.planar_reach_max = PLANAR_REACH_MAX
        self.planar_reach_min = PLANAR_REACH_MIN
        self.z_min = Z_MIN
        self.z_max = Z_MAX
        
        # Declarar parámetros ROS2
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('dxl_ids', DEFAULT_DXL_IDS)
        self.declare_parameter('goal_positions', [DEFAULT_GOAL] * 5)
        self.declare_parameter('moving_speed', DEFAULT_MOVING_SPEED)
        self.declare_parameter('torque_limit', DEFAULT_TORQUE_LIMIT)
        
        # Obtener parámetros
        port_name = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.dxl_ids = self.get_parameter('dxl_ids').value
        goal_positions = self.get_parameter('goal_positions').value
        moving_speed = int(self.get_parameter('moving_speed').value)
        torque_limit = int(self.get_parameter('torque_limit').value)
        
        # Configurar puerto
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
        
        # Configurar publicador de estados articulares
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Estado actual
        self.current_joint_positions = [0.0] * 5
        self.joint_names = JOINT_NAMES
        self.joint_sign = JOINT_SIGN
        
        # Inicializar motores
        self.initialize_motors(goal_positions, moving_speed, torque_limit)
    
    def initialize_motors(self, goal_positions, moving_speed, torque_limit):
        """Inicializa todos los motores con las posiciones y velocidades dadas."""
        for dxl_id, goal in zip(self.dxl_ids, goal_positions):
            try:
                # Habilitar torque
                result, error = self.packet.write1ByteTxRx(
                    self.port, dxl_id, ADDR_TORQUE_ENABLE, 1
                )
                if result != 0:
                    self.get_logger().error(
                        f'Error habilitando torque en motor {dxl_id}: {error}'
                    )
                    continue
                
                # Configurar velocidad y posición
                self.update_speed_single_motor(dxl_id, moving_speed)
                write_goal_position(self.packet, self.port, dxl_id, goal)
                
                # Actualizar estado interno
                joint_index = self.dxl_ids.index(dxl_id)
                angle = dxl_to_radians(goal)
                angle *= self.joint_sign.get(dxl_id, 1)
                self.current_joint_positions[joint_index] = angle
                
                self.get_logger().info(f'Motor {dxl_id} configurado correctamente')
            except Exception as e:
                self.get_logger().error(f'Error configurando motor {dxl_id}: {str(e)}')
    
    def publish_joint_states(self):
        """Publica el estado actual de las articulaciones."""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "base_link"
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        self.joint_state_pub.publish(joint_state)
    
    def move_motor(self, motor_id, position):
        """Mueve un motor individual a la posición especificada."""
        if self.emergency_stop_activated:
            self.get_logger().warning(
                f'No se puede mover motor {motor_id}: Parada de emergencia activada'
            )
            return False
        
        try:
            result, error = write_goal_position(
                self.packet, self.port, motor_id, position
            )
            if result == 0:
                self.get_logger().info(f'[Motor {motor_id}] Moviendo a {position}')
                
                # Actualizar estado interno
                joint_index = self.dxl_ids.index(motor_id)
                angle = dxl_to_radians(position)
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
        """Mueve el robot a ángulos especificados en grados."""
        if len(angles_deg) != 5:
            self.get_logger().error(
                f'Se esperaban 5 ángulos, se recibieron {len(angles_deg)}'
            )
            return False
        
        success = True
        for i, motor_id in enumerate(self.dxl_ids):
            sign = self.joint_sign.get(motor_id, 1)
            angle_deg = angles_deg[i] * sign
            goal_dxl = degrees_to_dxl(angle_deg)
            goal_dxl = int(np.clip(goal_dxl, 0, DXL_MAX_VALUE))
            
            if not self.move_motor(motor_id, goal_dxl):
                success = False
        
        return success
    
    def move_to_joint_angles(self, q_rad):
        """Mueve el robot a ángulos articulares en radianes (primeros 4 motores)."""
        if len(q_rad) != 4:
            self.get_logger().error(
                f'Se esperaban 4 ángulos, se recibieron {len(q_rad)}'
            )
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
            
            goal_dxl = radians_to_dxl(motor_angle_clamped)
            if not self.move_motor(motor_id, goal_dxl):
                success = False
        
        return success
    
    def move_to_xyz(self, x, y, z, orientation='down'):
        """Mueve el efector final a la posición XYZ especificada."""
        try:
            # Verificar límites del espacio de trabajo
            is_valid, error_msg = check_workspace_limits(
                x, y, z,
                self.planar_reach_min, self.planar_reach_max,
                self.z_min, self.z_max
            )
            
            if not is_valid:
                self.get_logger().error(error_msg)
                return False
            
            # Calcular cinemática inversa
            q_current = np.array(self.current_joint_positions[:4])
            success, q_solution = inverse_kinematics(
                self.robot_model, x, y, z, q_current, orientation
            )
            
            if not success:
                self.get_logger().error('No se encontró solución IK')
                return False
            
            return self.move_to_joint_angles(q_solution)
            
        except Exception as e:
            self.get_logger().error(f'Error en cinemática inversa: {str(e)}')
            return False
    
    def get_current_xyz(self):
        """Obtiene la posición XYZ actual del efector final."""
        try:
            q_current = np.array(self.current_joint_positions[:4])
            return forward_kinematics(self.robot_model, q_current)
        except Exception as e:
            self.get_logger().error(f'Error calculando cinemática directa: {str(e)}')
            return (0, 0, 0)
    
    def update_speed_single_motor(self, motor_id, speed):
        """Actualiza la velocidad de un motor individual."""
        try:
            result, error = write_moving_speed(
                self.packet, self.port, motor_id, speed
            )
            return result == 0
        except Exception as e:
            self.get_logger().error(
                f'Error actualizando velocidad motor {motor_id}: {str(e)}'
            )
            return False
    
    def update_speed(self, speed):
        """Actualiza la velocidad de todos los motores."""
        if self.emergency_stop_activated:
            self.get_logger().warning(
                'No se puede actualizar velocidad: Parada de emergencia activada'
            )
            return
        
        for motor_id in self.dxl_ids:
            self.update_speed_single_motor(motor_id, speed)
    
    def home_all_motors(self):
        """Mueve todos los motores a la posición home (0 grados)."""
        if self.emergency_stop_activated:
            self.reactivate_torque()
        
        self.move_to_angles_degrees([0, 0, 0, 0, 0])
        self.get_logger().info('Todos los motores movidos a 0°')
    
    def emergency_stop(self):
        """Activa la parada de emergencia deshabilitando el torque."""
        self.emergency_stop_activated = True
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(
                    self.port, dxl_id, ADDR_TORQUE_ENABLE, 0
                )
                self.get_logger().warning(f'Torque desactivado en motor {dxl_id}')
            except Exception as e:
                self.get_logger().error(
                    f'Error en parada de emergencia motor {dxl_id}: {str(e)}'
                )
    
    def reactivate_torque(self):
        """Reactiva el torque en todos los motores."""
        self.emergency_stop_activated = False
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(
                    self.port, dxl_id, ADDR_TORQUE_ENABLE, 1
                )
                self.get_logger().info(f'Torque reactivado en motor {dxl_id}')
            except Exception as e:
                self.get_logger().error(
                    f'Error reactivando torque en motor {dxl_id}: {str(e)}'
                )
    
    def close(self):
        """Cierra la conexión con los motores."""
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(
                    self.port, dxl_id, ADDR_TORQUE_ENABLE, 0
                )
            except:
                pass
        self.port.closePort()
