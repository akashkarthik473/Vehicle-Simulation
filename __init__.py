"""
Vehicle Simulation Package

A physics-based vehicle simulation with PID control and power limiting.
"""

from .vehicle import Vehicle
from .pid import PID
from .PowerLimiter import PowerLimiter

__version__ = "1.0.0"
__author__ = "Akash Karthik"

__all__ = ['Vehicle', 'PID', 'PowerLimiter'] 