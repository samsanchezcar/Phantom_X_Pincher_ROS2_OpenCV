"""
Modelo cinemático del robot PhantomX Pincher usando Robotics Toolbox.
"""
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

from .config import L1, L2, L3, L4


def build_pincher_robot():
    """
    Construye el modelo DH del robot PhantomX Pincher.
    
    Returns:
        rtb.DHRobot: Modelo del robot con 4 GDL
    """
    links = [
        rtb.RevoluteDH(d=L1, a=0.0, alpha=np.pi/2, offset=0.0),
        rtb.RevoluteDH(d=0.0, a=L2, alpha=0.0, offset=np.pi/2),
        rtb.RevoluteDH(d=0.0, a=L3, alpha=0.0, offset=0.0),
        rtb.RevoluteDH(d=0.0, a=L4, alpha=0.0, offset=0.0),
    ]
    robot = rtb.DHRobot(links, name="Pincher")
    T_tool = SE3.Rz(-np.pi/2) * SE3.Rx(-np.pi/2)
    robot.tool = T_tool
    return robot


def forward_kinematics(robot, q):
    """
    Calcula la cinemática directa.
    
    Args:
        robot: Modelo del robot
        q: Vector de ángulos articulares (4 elementos)
    
    Returns:
        tuple: Posición (x, y, z) del efector final
    """
    T = robot.fkine(q)
    return (T.t[0], T.t[1], T.t[2])


def inverse_kinematics(robot, x, y, z, q_current, orientation='down'):
    """
    Calcula la cinemática inversa usando el método Levenberg-Marquardt.
    
    Args:
        robot: Modelo del robot
        x, y, z: Posición objetivo
        q_current: Configuración articular actual
        orientation: 'down' para orientación hacia abajo, otro para arriba
    
    Returns:
        tuple: (success, q_solution) donde q_solution son los ángulos articulares
    """
    if orientation == 'down':
        T_target = SE3(x, y, z) * SE3.Rx(np.pi)
    else:
        T_target = SE3(x, y, z)
    
    seeds = [
        np.array(q_current),
        np.array([0, 0, 0, 0]),
        np.array([0, 0.5, -0.5, 0]),
        np.array([0, 1.0, -1.0, 0]),
    ]
    
    best_sol = None
    best_error = float('inf')
    
    for seed in seeds:
        sol = robot.ikine_LM(
            T_target, q0=seed, ilimit=1000, slimit=100,
            mask=[1, 1, 1, 0, 0, 0]
        )
        
        if sol.success:
            T_check = robot.fkine(sol.q)
            error = np.linalg.norm(T_check.t - T_target.t)
            
            if error < best_error:
                best_error = error
                best_sol = sol
            
            if error < 0.003:
                break
    
    if best_sol is None:
        return (False, None)
    
    return (True, best_sol.q)


def check_workspace_limits(x, y, z, planar_min, planar_max, z_min, z_max):
    """
    Verifica si una posición está dentro del espacio de trabajo.
    
    Args:
        x, y, z: Coordenadas del punto
        planar_min, planar_max: Límites del alcance planar
        z_min, z_max: Límites en Z
    
    Returns:
        tuple: (is_valid, error_message)
    """
    r = np.hypot(x, y)
    
    if r < planar_min or r > planar_max + 0.01:
        return (False, f'Fuera del alcance radial: r={r:.3f} m')
    
    if z < z_min - 0.01 or z > z_max + 0.01:
        return (False, f'Fuera del alcance en Z: z={z:.3f} m')
    
    return (True, None)
