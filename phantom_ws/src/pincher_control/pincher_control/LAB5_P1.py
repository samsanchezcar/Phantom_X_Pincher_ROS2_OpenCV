# pincher_control/Lab5_P1.py

import time
import threading

import rclpy
from rclpy.node import Node

from pincher_control.control_servo import PincherController


class SecuenciaLab5(Node):
    def __init__(self):
        # OJO: este nodo va a reusar el controlador de motores
        super().__init__('Lab5_P1')

        # Creamos el controlador de motores (también es un Node)
        self.controller = PincherController()
        self.get_logger().info('Nodo SecuenciaLab5 inicializado.')

        # Lanzamos el spin del controlador en un hilo aparte
        self.spin_thread = threading.Thread(
            target=rclpy.spin,
            args=(self.controller,),
            daemon=True
        )
        self.spin_thread.start()




    def ejecutar_secuencia(self):
        # IDs típicos: 1=waist, 2=shoulder, 3=elbow, 4=wrist
        waist_id = 1
        shoulder_id = 2
        elbow_id = 3
        wrist_id = 4

        # --- 1) Ir a HOME usando el DEFAULT_GOAL definido en control_servo.py ---
        self.get_logger().info('Llevando todos los motores a HOME...')
        self.controller.home_all_motors_sec()
        time.sleep(3.0)

        # --- 2) Definir posiciones objetivo en radianes (ejemplo) ---
        # Aquí tú pones los ángulos que quieras (en radianes) para la pose objetivo
        q_waist   = 0.5   # ~28.6°
        q_shoulder = -0.3 # ~-17°
        q_elbow    = 0.6  # ~34°
        q_wrist    = 0.6  # ~34°

        # Convertir radianes a valor Dynamixel (0-4095)
        waist_goal   = self.controller.radians_to_dxl(q_waist)
        shoulder_goal = self.controller.radians_to_dxl(q_shoulder)
        elbow_goal    = self.controller.radians_to_dxl(q_elbow)
        wrist_goal    = self.controller.radians_to_dxl(q_wrist)

        # --- 3) Mover de forma secuencial con pausas ---

        self.get_logger().info('Moviendo WAIST...')
        self.controller.move_motor(waist_id, waist_goal ) #Cambiar numero por waist_goal e igual con los demas
        time.sleep(2.0)

        self.get_logger().info('Moviendo SHOULDER...')
        self.controller.move_motor(shoulder_id, shoulder_goal)
        time.sleep(2.0)

        self.get_logger().info('Moviendo ELBOW...')
        self.controller.move_motor(elbow_id, elbow_goal )
        time.sleep(2.0)

        self.get_logger().info('Moviendo WRIST...')
        self.controller.move_motor(wrist_id, wrist_goal)
        time.sleep(2.0)

        self.get_logger().info('Secuencia terminada.')




def main(args=None):
    rclpy.init(args=args)

    node = SecuenciaLab5()
    try:
        while(True):
            node.ejecutar_secuencia()
    finally:
        # Cerrar correctamente
        node.controller.close()
        node.controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()