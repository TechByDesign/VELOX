"""
Main entry point for the Force Analysis script.

This script provides a simple interface to run the force analysis on a selected mesh.
"""

import bpy
import os
import sys
import importlib
from typing import Set, Callable, List, Optional

# Add the parent directory to Python path so we can import force_analysis
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

def reload_modules():
    """Reload all force analysis modules to reflect code changes without restarting Blender."""
    modules_to_reload = [
        'force_analysis.visualization',
        'force_analysis.settings',
        'force_analysis.calculations',
        'force_analysis.ui',
        'force_analysis.support',
        'force_analysis'
    ]
    
    print("\n=== Reloading Modules ===")
    for module_name in modules_to_reload:
        if module_name in sys.modules:
            try:
                module = sys.modules[module_name]
                importlib.reload(module)
                print(f"Reloaded: {module_name}")
            except Exception as e:
                print(f"Error reloading {module_name}: {str(e)}")
    print("=== Reload Complete ===\n")

# Now import from the package
reload_modules()  # Initial load of all modules
from force_analysis import run_analysis
from force_analysis.visualization import clear_force_collection

# Global set to track cleanup functions
_cleanup_functions: Set[Callable[[], None]] = set()

def register():
    """Register all necessary Blender properties and handlers."""
    # Register the selected_vertex_index property if it doesn't exist
    if not hasattr(bpy.types.Scene, 'selected_vertex_index'):
        bpy.types.Scene.selected_vertex_index = bpy.props.IntProperty(
            name="Selected Vertex Index",
            description="Index of the selected vertex for force application",
            default=-1,
            min=-1
        )
    
    # Add cleanup handler if not already added
    if bpy.app.background:
        return
    
    def cleanup():
        """Cleanup function to run when Blender exits."""
        try:
            clear_force_collection()
            for func in _cleanup_functions:
                try:
                    func()
                except Exception as e:
                    print(f"Error during cleanup: {e}")
        except Exception as e:
            print(f"Error during cleanup: {e}")
    
    # Store the cleanup function
    _cleanup_functions.add(cleanup)
    
    # Register the cleanup function
    if not hasattr(bpy.app.handlers, 'persistent'):
        bpy.app.handlers.persistent = lambda f: f
    
    if hasattr(bpy.app.handlers, 'save_pre'):
        bpy.app.handlers.save_pre.append(cleanup)
    if hasattr(bpy.app.handlers, 'load_post'):
        bpy.app.handlers.load_post.append(cleanup)

def unregister():
    """Unregister all handlers and cleanup."""
    if hasattr(bpy.types.Scene, 'selected_vertex_index'):
        del bpy.types.Scene.selected_vertex_index
    
    # Run cleanup
    for func in _cleanup_functions:
        try:
            func()
        except Exception as e:
            print(f"Error during cleanup: {e}")
    _cleanup_functions.clear()

if __name__ == "__main__":
    try:
        # Ensure we're in object mode
        if bpy.context.object and bpy.context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        
        # Clear any existing visualizations
        clear_force_collection()
        
        # Run the analysis
        run_analysis()
    except Exception as e:
        print(f"Error in main: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Ensure we don't leave the scene in a bad state
        if bpy.context.object and bpy.context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

# Handle addon registration if running as an addon
if hasattr(bpy.utils, 'register_class') and not bpy.app.background:
    # Store the original register/unregister functions
    _original_register = register
    _original_unregister = unregister
    
    # Define new register/unregister functions for addon
    def register():
        """Register the addon."""
        _original_register()
    
    def unregister():
        """Unregister the addon."""
        _original_unregister()
    
    # Register the addon if not already registered
    if not bpy.app.background:
        register()
else:
    # Running as a script, just call register
    if not bpy.app.background:
        register()
