#!/usr/bin/env python3
"""
toolbox_live.py - Visualización en tiempo real del Pincher usando ROS2 + Robotics Toolbox
Se suscribe a /joint_states para actualizar la visualización mientras controlas desde terminal_control.py
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import roboticstoolbox as rtb
from spatialmath import SE3
import matplotlib.pyplot as plt


def build_pincher_robot():
    """
    Modelo DH del Pincher, equivalente al de MATLAB
    """
    # Pasar de mm a m
    L1 = 44.0 / 1000.0
    L2 = 107.5 / 1000.0
    L3 = 107.5 / 1000.0
    L4 = 75.3 / 1000.0

    links = [
        rtb.RevoluteDH(d=L1, a=0.0,  alpha=np.pi/2, offset=0.0),
        rtb.RevoluteDH(d=0.0, a=L2, alpha=0.0, offset=np.pi/2),
        rtb.RevoluteDH(d=0.0, a=L3, alpha=0.0, offset=0.0),
        rtb.RevoluteDH(d=0.0, a=L4, alpha=0.0, offset=0.0),
    ]

    robot = rtb.DHRobot(links, name="Pincher")

    # Herramienta: T_tool = trotz(-pi/2)*trotx(-pi/2)
    T_tool = SE3.Rz(-np.pi/2) * SE3.Rx(-np.pi/2)
    robot.tool = T_tool

    return robot


class PincherVisualizer(Node):
    def __init__(self, robot):
        super().__init__('pincher_visualizer')
        
        self.robot = robot
        self.current_q = np.zeros(4)  # [waist, shoulder, elbow, wrist]
        
        # Suscribirse a /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Configurar matplotlib para modo interactivo
        plt.ion()
        self.fig = None
        self.ax = None
        
        # Timer para actualizar la visualización
        self.timer = self.create_timer(0.1, self.update_visualization)  # 10 Hz
        
        self.get_logger().info('Visualizador iniciado - esperando datos de /joint_states...')
    
    def joint_state_callback(self, msg):
        """
        Callback que recibe los estados de las articulaciones desde /joint_states
        """
        # Asumiendo que los nombres son: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'gripper']
        # Tomamos solo las primeras 4 articulaciones
        if len(msg.position) >= 4:
            self.current_q = np.array(msg.position[:4])
            
    def update_visualization(self):
        """
        Actualiza la visualización 3D del robot
        """
        try:
            if self.fig is None:
                # Primera vez: crear la figura
                self.fig = self.robot.plot(
                    self.current_q,
                    block=False,
                    limits=[-0.3, 0.3, -0.3, 0.3, 0, 0.4]
                )
            else:
                # Actualizar la configuración del robot
                self.robot.q = self.current_q
                self.fig.step()
                
            # Forzar actualización de la ventana
            plt.pause(0.001)
            
        except Exception as e:
            self.get_logger().error(f'Error en visualización: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    # Crear el modelo del robot
    robot = build_pincher_robot()
    
    # Crear el nodo visualizador
    visualizer = PincherVisualizer(robot)
    
    print("\n=== Visualizador en Vivo - Pincher X100 ===")
    print("Este nodo se suscribe a /joint_states y muestra el robot en 3D")
    print("Ejecuta terminal_control.py en otra terminal para mover el robot")
    print("Presiona Ctrl+C para salir\n")
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        print("\nCerrando visualizador...")
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == "__main__":
    main()