"""
User interface components for the force analysis tool.
"""
import bpy
import bmesh
from mathutils import Vector
from typing import List, Optional, Dict, Any, Tuple

from .models import Support, SupportType
from .settings import supports, force_components, force_magnitude, base_radius, text_scale, input_force_color
from .calculations import get_mesh_data, calculate_forces
from .visualization import visualize_forces, clear_force_collection

# Global variables
selected_vertex_index = None


def get_selected_vertex_index() -> Optional[int]:
    """Get the selected vertex index in Edit Mode"""
    if bpy.context.mode != 'EDIT_MESH':
        return None
        
    obj = bpy.context.edit_object
    if not obj or obj.type != 'MESH':
        return None
        
    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    
    selected_verts = [v for v in bm.verts if v.select]
    
    if len(selected_verts) == 1:
        return selected_verts[0].index
    
    return None


def detect_support_vertices(vertices: List[Vector], 
                           support_type: SupportType = SupportType.FIXED, 
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
        from .settings import clear_supports, add_support
        clear_supports()
        for idx in selected_indices:
            add_support(idx, SupportType.FIXED)
        print(f"Added supports at vertices: {selected_indices}")
    
    # Update the display
    bmesh.update_edit_mesh(obj.data)


def run_analysis() -> None:
    """Main analysis function to be called from Blender's scripting panel"""
    obj = bpy.context.active_object
    if not obj or obj.type != 'MESH':
        print("No mesh object selected")
        return
    
    # Get mesh data
    vertices, edges = get_mesh_data(obj)
    
    # Auto-detect supports if none exist
    from .settings import supports, add_support
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
