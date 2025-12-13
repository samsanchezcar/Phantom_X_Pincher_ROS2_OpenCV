# pincher_control/terminal_control.py

import math
import threading

import rclpy
from pincher_control.control_servo import PincherController


def get_preset_configurations():
    """
    Retorna las configuraciones predefinidas
    Formato: [waist, shoulder, elbow, wrist, gripper] en grados
    """
    return {
        'c1': [0, 0, 0, 0, 0],
        'c2': [25, 25, 20, -20, 0],
        'c3': [-35, 35, -30, 30, 0],
        'c4': [85, -20, 55, 25, 0],
        'c5': [80, -35, 55, -45, 0],
    }


def send_configuration(controller, config_deg):
    """
    Envía una configuración completa al robot
    
    Args:
        controller: PincherController instance
        config_deg: Lista de 5 ángulos en grados [waist, shoulder, elbow, wrist, gripper]
    """
    print(f"\n📤 Enviando configuración: {config_deg}")
    
    # IDs de los motores [1, 2, 3, 4, 5]
    ids = controller.dxl_ids[:5]
    
    for i, motor_id in enumerate(ids):
        angle_deg = config_deg[i]
        angle_rad = math.radians(angle_deg)
        
        # Ajuste de signo según el mapeo motor ↔ articulación
        sign = controller.joint_sign.get(motor_id, 1)
        angle_rad_motor = angle_rad * sign
        
        # Convertir a valor Dynamixel
        goal_dxl = controller.radians_to_dxl(angle_rad_motor)
        
        # Enviar comando
        controller.move_motor(motor_id, goal_dxl)
        
        print(f"  Motor {motor_id}: {angle_deg:6.1f}° (dxl={goal_dxl})")
    
    print("✓ Configuración enviada")


def print_help():
    """Imprime la ayuda con las opciones disponibles"""
    configs = get_preset_configurations()
    
    print("\n" + "="*60)
    print("COMANDOS DISPONIBLES:")
    print("="*60)
    print("\n1️⃣  Control Individual:")
    print("   Formato: ID angulo_grados")
    print("   Ejemplo: 2 45  → Mueve motor 2 a +45°")
    
    print("\n2️⃣  Configuraciones Predefinidas:")
    for key, config in configs.items():
        print(f"   {key}: {config}  (waist, shoulder, elbow, wrist, gripper)")
    
    print("\n3️⃣  Otros:")
    print("   help  → Muestra esta ayuda")
    print("   q     → Salir")
    print("="*60 + "\n")


def main(args=None):
    rclpy.init(args=args)

    # Creamos el controlador de motores (mismo que la GUI)
    controller = PincherController()

    if not rclpy.ok():
        # Si algo falló en el init del controlador (puerto, baudrate, etc.)
        return

    # Lanzamos el spin en un hilo aparte para que se sigan publicando /joint_states
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(controller,),
        daemon=True
    )
    spin_thread.start()

    dxl_ids = controller.dxl_ids  # típicamente [1, 2, 3, 4, 5]
    configs = get_preset_configurations()

    print("\n╔═══════════════════════════════════════════════════════╗")
    print("║   Control por Terminal - PhantomX Pincher X100       ║")
    print("╚═══════════════════════════════════════════════════════╝")
    print(f"\nMotores disponibles: {dxl_ids}")
    print("Convención: 1=waist, 2=shoulder, 3=elbow, 4=wrist, 5=gripper")
    
    # Mostrar configuraciones disponibles
    print("\n📋 Configuraciones predefinidas:")
    for key, config in configs.items():
        print(f"   {key}: {config}")
    
    print("\n💡 Escribe 'help' para ver todos los comandos")
    print("   Escribe 'q' para salir\n")

    try:
        while True:
            line = input(">> ").strip().lower()

            if line in ("q", "quit", "exit"):
                print("👋 Saliendo...")
                break

            if not line:
                continue

            # Comando de ayuda
            if line == "help":
                print_help()
                continue

            # Verificar si es una configuración predefinida
            if line in configs:
                config_deg = configs[line]
                send_configuration(controller, config_deg)
                continue

            # Control individual de motor
            parts = line.split()
            if len(parts) != 2:
                print("❌ Formato inválido.")
                print("   Usa: ID angulo_grados  (ej: 2 45)")
                print("   O usa: c1, c2, c3, c4, c5 para configuraciones predefinidas")
                print("   Escribe 'help' para más información")
                continue

            try:
                motor_id = int(parts[0])
                angle_deg_joint = float(parts[1])
            except ValueError:
                print("❌ Error: ID debe ser entero y el ángulo un número. Ej: 2 -45")
                continue

            # Validar que el ID exista en la lista de motores
            if motor_id not in dxl_ids:
                print(f"❌ ID {motor_id} no está en la lista de motores: {dxl_ids}")
                continue

            # Validar límites articulares (en grados)
            if (angle_deg_joint < -150.0 or angle_deg_joint > 150.0) and (motor_id == 1):
                print("❌ Ángulo fuera de límites [-150°, 150°]. No se envía comando.")
                continue
            elif (angle_deg_joint < -100.0 or angle_deg_joint > 100.0) and (motor_id == 2):
                print("❌ Ángulo fuera de límites [-100°, 100°]. No se envía comando.")
                continue
            elif (angle_deg_joint < -120.0 or angle_deg_joint > 120.0) and (motor_id == 3):
                print("❌ Ángulo fuera de límites [-120°, 120°]. No se envía comando.")
                continue
            elif (angle_deg_joint < -120.0 or angle_deg_joint > 120.0) and (motor_id == 4):
                print("❌ Ángulo fuera de límites [-120°, 120°]. No se envía comando.")
                continue
            elif (angle_deg_joint < 0 or angle_deg_joint > 120.0) and (motor_id == 5):
                print("❌ Ángulo fuera de límites [0, 120°]. No se envía comando.")
                continue

            # Convertir a radianes (ángulo de la articulación en el modelo)
            angle_rad_joint = math.radians(angle_deg_joint)

            # Ajuste de signo según el mapeo motor ↔ articulación
            sign = controller.joint_sign.get(motor_id, 1)
            # Este es el ángulo que debe ver el motor (antes del signo en RViz)
            angle_rad_motor = angle_rad_joint * sign

            # Convertir radianes a valor Dynamixel (0–4095)
            goal_dxl = controller.radians_to_dxl(angle_rad_motor)

            # Mandar comando al motor
            controller.move_motor(motor_id, goal_dxl)

            print(
                f"✓ Motor {motor_id}: {angle_deg_joint:.1f}° "
                f"(motor_rad={angle_rad_motor:.3f}, dxl={goal_dxl})"
            )

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrumpido por el usuario.")

    finally:
        # Apagar torque y cerrar puerto
        print("\n🔧 Cerrando recursos...")
        controller.close()
        controller.destroy_node()
        rclpy.shutdown()
        print("✓ Finalizado\n")


if __name__ == "__main__":
    main()