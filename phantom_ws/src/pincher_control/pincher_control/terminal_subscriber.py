import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePrinter(Node):
    def __init__(self):
        super().__init__('joint_state_printer')

        # Nos suscribimos al tópico /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Nombres de las articulaciones que esperamos (los mismos que usa tu controlador)
        self.joint_names = ['waist', 'shoulder', 'elbow', 'wrist', 'gripper']

        # Últimos valores en grados
        self.angles_deg = {name: 0.0 for name in self.joint_names}

        self.get_logger().info("JointStatePrinter iniciado. Esperando mensajes en /joint_states ...")

    def joint_state_callback(self, msg: JointState):
        """
        Callback que se ejecuta cada vez que llega un mensaje JointState.
        Convierte las posiciones de radianes a grados y las imprime.
        """
        # Actualizamos los valores que vengan en el mensaje
        for name, pos_rad in zip(msg.name, msg.position):
            if name in self.angles_deg:
                self.angles_deg[name] = math.degrees(pos_rad)

        # Construimos una línea bonita con los 5 motores
        text = (
            f"waist: {self.angles_deg['waist']:.1f}°  | "
            f"shoulder: {self.angles_deg['shoulder']:.1f}°  | "
            f"elbow: {self.angles_deg['elbow']:.1f}°  | "
            f"wrist: {self.angles_deg['wrist']:.1f}°  | "
            f"gripper: {self.angles_deg['gripper']:.1f}°"
        )

        # Imprimimos en consola
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()