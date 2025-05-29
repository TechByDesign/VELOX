"""
Force Analysis Package

A package for structural analysis of truss structures in Blender.
"""

from .models import Support, SupportType
from .settings import (
    force_components, force_magnitude, base_radius, text_scale, input_force_color,
    supports, add_support, remove_support, clear_supports, get_supports, get_support, has_support
)
from .calculations import get_mesh_data, calculate_forces
from .visualization import visualize_forces, clear_force_collection

# Make these available at package level
__all__ = [
    # Models
    'Support',
    'SupportType',
    
    # Settings
    'force_components',
    'force_magnitude',
    'base_radius',
    'text_scale',
    'input_force_color',
    'supports',
    'add_support',
    'remove_support',
    'clear_supports',
    'get_supports',
    'get_support',
    'has_support',
    
    # Calculations
    'get_mesh_data',
    'calculate_forces',
    
    # Visualization
    'visualize_forces',
    'clear_force_collection'
]
