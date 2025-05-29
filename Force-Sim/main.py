"""
Main script for running force analysis on a mesh in Blender.

To use:
1. Select a mesh object in Blender
2. Run this script from the Text Editor or Scripting workspace
"""

import bpy
import bmesh
import os
import sys
import traceback
from mathutils import Vector
from typing import List, Dict, Optional, Tuple, Any

# Get the directory where the current script is located
if hasattr(bpy.data, 'texts') and bpy.context.space_data and bpy.context.space_data.text:
    # If running from Blender's text editor
    script_path = bpy.context.space_data.text.filepath
    if not script_path:
        # If the script hasn't been saved, use the current working directory
        script_dir = os.getcwd()
    else:
        script_dir = os.path.dirname(os.path.abspath(script_path))
else:
    # If running as a standalone script
    script_dir = os.path.dirname(os.path.abspath(__file__))

print(f"Script directory: {script_dir}")

# Add the script's directory to Python path
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

# Try to import the force_analysis package
try:
    # Try absolute import first
    from force_analysis import (
        SupportType, Support,
        force_components, force_magnitude,
        base_radius, text_scale, input_force_color,
        supports, add_support, clear_supports, get_supports, get_support, has_support,
        get_mesh_data, calculate_forces,
        visualize_forces, clear_force_collection
    )
    print("Successfully imported force_analysis package using absolute import")
except ImportError as e:
    print(f"Absolute import failed: {e}")
    try:
        # Try relative import
        from .force_analysis import (
            SupportType, Support,
            force_components, force_magnitude,
            base_radius, text_scale, input_force_color,
            supports, add_support, clear_supports, get_supports, get_support, has_support,
            get_mesh_data, calculate_forces,
            visualize_forces, clear_force_collection
        )
        print("Successfully imported force_analysis package using relative import")
    except ImportError as e2:
        print(f"Relative import failed: {e2}")
        print("\nERROR: Could not import force_analysis package")
        print("Please ensure the force_analysis directory is in the same directory as this script.")
        print(f"Current directory: {os.listdir(script_dir)}")
        raise

# Global variables
selected_vertex_index = None

# Copy of functions from the original script that need to be in the main namespace
def get_selected_vertex_index() -> Optional[int]:
    """Get the selected vertex index in Edit Mode"""
    if bpy.context.mode != 'EDIT_MESH':
        print("Not in EDIT SELECTION mode")
        return None
        
    obj = bpy.context.edit_object
    if not obj or obj.type != 'MESH':
        print("Select a valid MESH")
        return None
        
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    
    selected_verts = [v for v in bm.verts if v.select]
    
    if len(selected_verts) == 1:
        return selected_verts[0].index
    print("Select a valid amount of verticies")
    return None

def detect_support_vertices(vertices: List[Vector], 
                           support_type: SupportType = SupportType.FIXED, 
                           max_supports: int = 4) -> List[Support]:
    """Automatically detect support vertices based on lowest Z coordinates."""
    if not vertices:
        return []
    
    # Find the lowest Z coordinate
    min_z = min(v.z for v in vertices)
    
    # Find all vertices at or near the lowest Z
    tolerance = 0.001
    bottom_vertices = [
        (i, v) for i, v in enumerate(vertices) 
        if abs(v.z - min_z) <= tolerance
    ]
    
    # Sort by X coordinate to get left-to-right order
    bottom_vertices.sort(key=lambda x: x[1].x)
    
    # Take up to max_supports vertices, evenly distributed
    if len(bottom_vertices) > max_supports:
        step = max(1, len(bottom_vertices) // max_supports)
        selected_indices = [i for i in range(0, len(bottom_vertices), step)][:max_supports]
        bottom_vertices = [bottom_vertices[i] for i in selected_indices]
    
    # Create supports
    support_list = []
    for idx, _ in bottom_vertices:
        support = Support(vertex_index=idx, support_type=support_type)
        support_list.append(support)
        
    return support_list

def select_vertices() -> None:
    """Vertex selection system with automatic support detection"""
    global selected_vertex_index
    
    obj = bpy.context.active_object
    if not obj or obj.type != 'MESH':
        print("No mesh object selected")
        return
    
    # Get the mesh data
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    
    # Get selected vertices
    selected_verts = [v for v in bm.verts if v.select]
    
    if not selected_verts:
        print("No vertices selected")
        return
    
    # Get vertex indices
    selected_indices = [v.index for v in selected_verts]
    
    # Toggle selection for force application
    if len(selected_indices) == 1:
        idx = selected_indices[0]
        if selected_vertex_index == idx:
            # Deselect if clicking the same vertex
            selected_vertex_index = None
            print(f"Deselected vertex {idx}")
        else:
            selected_vertex_index = idx
            print(f"Selected vertex {idx} for force application")
    
    # Auto-detect supports if exactly 4 vertices are selected for wheel positions
    if len(selected_indices) == 4:
        clear_supports()
        for idx in selected_indices:
            add_support(idx, SupportType.FIXED)
        print(f"Added supports at vertices: {selected_indices}")
    
    # Update the display
    bmesh.update_edit_mesh(obj.data)

def run_analysis() -> None:
    """Main analysis function"""
    selected_vertex_index = get_selected_vertex_index()
    
    obj = bpy.context.active_object
    if not obj or obj.type != 'MESH':
        print("No mesh object selected")
        return
    
    # Get mesh data
    vertices, edges = get_mesh_data(obj)
    
    # Auto-detect supports if none exist
    if not supports:
        support_list = detect_support_vertices(vertices)
        for support in support_list:
            add_support(support.vertex_index, support.support_type)
    
    # Calculate forces
    edge_forces, max_force = calculate_forces(
        vertices=vertices,
        edges=edges,
        force_components=force_components,
        force_magnitude=force_magnitude,
        selected_vertex_index=selected_vertex_index
    )
    
    # Visualize forces
    visualize_forces(
        vertices=vertices,
        edges=edges,
        edge_forces=edge_forces,
        max_force=max_force,
        selected_vertex_index=selected_vertex_index
    )
    
    print("Analysis complete")

if __name__ == "__main__":
    # Clear any existing visualizations
    clear_force_collection()
    clear_supports()
    
    # Run the analysis
    try:
        run_analysis()
    except Exception as e:
        import traceback
        error_msg = f"Error in force analysis: {str(e)}\n\n{traceback.format_exc()}"
        print("\nERROR:", error_msg)
        # If we can't show a popup, at least print to console
        print("\nFull error traceback:")
        traceback.print_exc()
