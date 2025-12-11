#!/usr/bin/env python3
"""
PhantomX Pincher Control Studio - Punto de entrada principal.

Uso:
    python -m pincher_control.main
    
    o bien:
    
    ros2 run pincher_control main
"""
import sys
import threading

import rclpy
from PyQt5.QtWidgets import QApplication

from .controller import PincherController
from .gui import ModernPincherGUI
from .styles import MODERN_STYLESHEET


def main(args=None):
    """Función principal de la aplicación."""
    # Inicializar ROS2
    rclpy.init(args=args)
    
    # Crear el controlador
    controller = PincherController()
    
    # Iniciar el spin de ROS2 en un hilo separado
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(controller,),
        daemon=True
    )
    spin_thread.start()
    
    # Crear y ejecutar la aplicación Qt
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
