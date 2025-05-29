"""
Configuration and constants for the force analysis package.
"""
from typing import Dict, Optional, List
from mathutils import Vector
from .models import Support, SupportType

# Force configuration
force_components = {
    'x': 0,      # X component (horizontal)
    'y': 0,      # Y component (horizontal)
    'z': 1       # Z component (vertical, default downward)
}

# Base magnitude multiplier
force_magnitude = 1000

# Visualization settings
base_radius = 0.05  # Base radius for all elements
text_scale = 0.08
input_force_color = (1, 1, 1, 1)  # White for input force

# Support storage
supports: Dict[int, Support] = {}

def add_support(vertex_index: int, support_type: SupportType = SupportType.FIXED, direction=None) -> bool:
    """Add or update a support at the given vertex."""
    if vertex_index < 0:
        return False
    supports[vertex_index] = Support(vertex_index, support_type, direction)
    return True

def remove_support(vertex_index: int) -> bool:
    """Remove support from the given vertex."""
    if vertex_index in supports:
        del supports[vertex_index]
        return True
    return False

def clear_supports() -> None:
    """Remove all supports."""
    supports.clear()

def get_supports() -> list:
    """Get a list of all supports."""
    return list(supports.values())

def get_support(vertex_index: int):
    """Get the support at the given vertex, or None if none exists."""
    return supports.get(vertex_index)

def has_support(vertex_index: int) -> bool:
    """Check if a support exists at the given vertex."""
    return vertex_index in supports
