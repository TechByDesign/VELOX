"""
Settings and configuration for force analysis.

This module contains all the configuration parameters used throughout the force analysis.
"""

# Force configuration
force_components = {
    'x': 0,      # X component (horizontal)
    'y': 0,      # Y component (horizontal)
    'z': 1       # Z component (vertical, default downward)
}

# Base magnitude multiplier
force_magnitude = 5000


# Visualization settings
base_radius = 0.05  # Base radius for all elements
text_scale = 0.08
input_force_color = (1, 1, 1, 1)  # White for input force