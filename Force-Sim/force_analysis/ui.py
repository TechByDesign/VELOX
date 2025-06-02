"""
UI and interaction functions for force analysis.

This module handles user interaction and the main analysis workflow.
"""

import bpy
import bmesh
from mathutils import Vector
from typing import List, Optional, Tuple, Dict, Any

from .support import Support, SupportType, supports, add_support, clear_supports
from .settings import force_components, force_magnitude, base_radius, text_scale, input_force_color
from .calculations import get_mesh_data, calculate_forces
from .visualization import visualize_forces, clear_force_collection

def get_selected_vertex_index() -> Optional[int]:
    """Get the selected vertex index in Edit Mode"""
    switch_to_edit_mode = False
    try:
        obj = bpy.context.active_object
        if not obj or obj.type != 'MESH' or obj.mode != 'EDIT':
            print("Not in Edit Mode or no mesh object selected: Reverting to Edit mode")
            bpy.ops.object.mode_set(mode='EDIT')
            switch_to_edit_mode = True
            # return None
        
        # Get the mesh data
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        
        # Find selected vertices
        selected_verts = [v for v in bm.verts if v.select]
        
        if len(selected_verts) == 1:
            print("Selected vertex index:", selected_verts[0].index)
            return selected_verts[0].index
        
        print("No single vertex selected")
        if switch_to_edit_mode:
            bpy.ops.object.mode_set(mode='OBJECT')
        return None
    except Exception as e:
        print(f"Error in get_selected_vertex_index: {e}")
        return None
    finally:
        # Ensure we don't leak BMesh objects
        if 'bm' in locals():
            bm.free()

def detect_support_vertices(vertices: List[Vector], support_type=None, 
                          max_supports: int = 4) -> List[Support]:
    """Automatically detect support vertices based on lowest Z coordinates.
    
    Args:
        vertices: List of vertex coordinates
        support_type: Type of support to create (default: FIXED for chassis analysis)
        max_supports: Maximum number of supports to create (default: 4 for wheel positions)
        
    Returns:
        list: List of Support objects for the detected vertices
    """
    if not vertices:
        return []
    
    # Ensure support_type is a valid SupportType
    from .support import SupportType as ST
    if support_type is None:
        support_type = ST.FIXED
    elif not isinstance(support_type, ST):
        raise TypeError(f"support_type must be a SupportType enum, got {type(support_type).__name__}")
    
    # Sort vertices by Z coordinate (lowest first)
    sorted_verts = sorted([(i, v) for i, v in enumerate(vertices)], 
                         key=lambda x: x[1].z)
    
    # Take the lowest max_supports vertices
    support_verts = sorted_verts[:max_supports]
    
    # Create supports
    support_objs = []
    for idx, _ in support_verts:
        support = Support(vertex_index=idx, support_type=support_type)
        supports[idx] = support
        support_objs.append(support)
    
    return support_objs

def select_vertices() -> None:
    """Vertex selection system with automatic support detection"""
    try:
        obj = bpy.context.active_object
        if not obj or obj.type != 'MESH':
            print("No mesh object selected")
            return
        
        # Store current mode
        current_mode = obj.mode
        if current_mode != 'EDIT':
            bpy.ops.object.mode_set(mode='EDIT')
        
        # Get mesh data
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
            if hasattr(bpy.context.scene, 'selected_vertex_index') and \
               bpy.context.scene.selected_vertex_index == idx:
                # Deselect if clicking the same vertex
                bpy.context.scene.selected_vertex_index = -1
                print(f"Deselected vertex {idx}")
            else:
                bpy.context.scene.selected_vertex_index = idx
                print(f"Selected vertex {idx} for force application")
        
        # Auto-detect supports if exactly 4 vertices are selected for wheel positions
        if len(selected_indices) == 4:
            from .support import SupportType as ST
            clear_supports()
            for idx in selected_indices:
                add_support(idx, ST.FIXED)
            print(f"Added supports at vertices: {selected_indices}")
            
    except Exception as e:
        print(f"Error in select_vertices: {e}")
    finally:
        # Clean up BMesh
        if 'bm' in locals():
            bm.free()
        # Restore mode if it was changed
        if 'current_mode' in locals() and current_mode != 'EDIT':
            bpy.ops.object.mode_set(mode=current_mode)

def run_analysis() -> None:
    """Main coordinator"""
    try:
        # Ensure we're in object mode
        if bpy.context.object and bpy.context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        
        # Get active object
        obj = bpy.context.active_object
        if not obj or obj.type != 'MESH':
            print("Please select a mesh object")
            return
        
        # Get mesh data
        vertices, edges = get_mesh_data(obj)
        if not vertices or not edges:
            print("No valid mesh data found")
            return
        
        # Auto-detect supports if none are defined
        if not supports:
            from .support import SupportType as ST
            detect_support_vertices(vertices, support_type=ST.FIXED)
        
        # Get selected vertex for force application
        selected_vertex_index = get_selected_vertex_index()
        print("Selected vertex index:", selected_vertex_index)
        if selected_vertex_index == -1 or selected_vertex_index >= len(vertices):
            selected_vertex_index = None
        
        # Calculate forces
        edge_forces, max_force = calculate_forces(
            vertices, 
            edges, 
            force_components, 
            force_magnitude,
            selected_vertex_index
        )
        
        if not edge_forces:
            print("No forces to visualize")
            return
        
        # Set selected_vertex_index in visualization module
        from . import visualization
        visualization.selected_vertex_index = selected_vertex_index if selected_vertex_index is not None else -1
        
        # Visualize forces
        visualization.visualize_forces(vertices, edges, edge_forces, max_force)
        
        print("Analysis complete")
        
    except Exception as e:
        print(f"Error in run_analysis: {e}")
        import traceback
        traceback.print_exc()
