"""
Funciones auxiliares para comunicación con motores Dynamixel.
"""
from .config import (
    USE_XL430, 
    ADDR_GOAL_POSITION, 
    ADDR_MOVING_SPEED, 
    ADDR_PRESENT_POSITION
)


def write_goal_position(packet, port, dxl_id, position):
    """Escribe la posición objetivo en un motor Dynamixel."""
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))


def write_moving_speed(packet, port, dxl_id, speed):
    """Escribe la velocidad de movimiento en un motor Dynamixel."""
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))


def read_present_position(packet, port, dxl_id):
    """Lee la posición actual de un motor Dynamixel."""
    if USE_XL430:
        return packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    else:
        return packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)


def dxl_to_radians(dxl_value):
    """Convierte valor Dynamixel a radianes."""
    if USE_XL430:
        center, scale = 2048.0, 2.618 / 2048.0
    else:
        center, scale = 512.0, 2.618 / 512.0
    return (dxl_value - center) * scale


def radians_to_dxl(radians):
    """Convierte radianes a valor Dynamixel."""
    if USE_XL430:
        center, inv_scale = 2048.0, 2048.0 / 2.618
    else:
        center, inv_scale = 512.0, 512.0 / 2.618
    return int(radians * inv_scale + center)


def degrees_to_dxl(degrees):
    """Convierte grados a valor Dynamixel."""
    import numpy as np
    radians = np.radians(degrees)
    return radians_to_dxl(radians)
