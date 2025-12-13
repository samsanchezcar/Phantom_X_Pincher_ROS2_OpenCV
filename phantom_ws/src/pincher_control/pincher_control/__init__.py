"""
PhantomX Pincher Control - Paquete modular para control del robot.
"""
from .config import *
from .controller import PincherController
from .kinematics import build_pincher_robot, forward_kinematics, inverse_kinematics
from .styles import MODERN_STYLESHEET

__version__ = "2.0.0"
__author__ = "Samuel Sanchez & Santiago Ávila"
