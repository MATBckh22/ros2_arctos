"""
Arctos Hardware Package.

This package provides hardware control for the Arctos robot arm,
including CAN bus communication with MKS SERVO motor drivers,
differential wrist kinematics, and homing procedures.

Modules:
    can_interface: Low-level CAN bus communication
    mks_servo: MKS SERVO motor driver protocol
    differential_wrist: J5/J6 coupled axis kinematics
    homing: Motor homing and calibration procedures
    arctos_controller: High-level robot controller

Usage:
    from arctos_hardware import ArctosController, ArctosConfig
    
    config = ArctosConfig(can_device='can0')
    controller = ArctosController(config)
    
    controller.connect()
    controller.enable_motors()
    controller.home_all()
    controller.move_to_positions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    controller.disconnect()
"""

from arctos_hardware.can_interface import CanInterface, CanError
from arctos_hardware.mks_servo import MksServo, MotorStatus
from arctos_hardware.differential_wrist import DifferentialWrist
from arctos_hardware.homing import ArctosHoming, HomingState
from arctos_hardware.arctos_controller import ArctosController, ArctosConfig

__all__ = [
    'CanInterface',
    'CanError',
    'MksServo',
    'MotorStatus',
    'DifferentialWrist',
    'ArctosHoming',
    'HomingState',
    'ArctosController',
    'ArctosConfig',
]

__version__ = '1.0.0'
