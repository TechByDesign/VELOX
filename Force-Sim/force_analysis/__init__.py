"""
Force Analysis Package

This package provides functionality for analyzing forces in truss structures in Blender.
"""

# Import core components to make them available at package level
from .support import Support, SupportType, supports, add_support, remove_support, clear_supports, get_supports, get_support, has_support
from .settings import force_components, force_magnitude, base_radius, text_scale, input_force_color
from .calculations import get_mesh_data, calculate_forces
from .visualization import visualize_forces, clear_force_collection, get_force_collection
from .ui import select_vertices, detect_support_vertices, get_selected_vertex_index, run_analysis

__all__ = [
    'Support', 'SupportType', 'supports', 'add_support', 'remove_support', 
    'clear_supports', 'get_supports', 'get_support', 'has_support',
    'force_components', 'force_magnitude', 'base_radius', 'text_scale', 'input_force_color',
    'get_mesh_data', 'calculate_forces',
    'visualize_forces', 'clear_force_collection', 'get_force_collection',
    'select_vertices', 'detect_support_vertices', 'get_selected_vertex_index', 'run_analysis'
]
