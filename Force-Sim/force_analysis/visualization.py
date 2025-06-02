"""
Visualization functions for force analysis.

This module handles the creation and management of force visualizations in Blender.
"""

import bpy
import bmesh
import math
from mathutils import Vector, Quaternion
from typing import List, Dict, Tuple, Optional

# Import settings
try:
    from .settings import text_scale, input_force_color
except ImportError:
    # Fallback values in case import fails
    text_scale = 0.08
    input_force_color = (1, 1, 1, 1)  # White

# Global settings
base_radius = 0.05

# Global state
selected_vertex_index = -1  # Will be set by the main analysis function

def get_force_collection() -> bpy.types.Collection:
    """Get or create the Forces collection"""
    collection_name = "Forces"
    
    # Remove existing collection if it exists
    if collection_name in bpy.data.collections:
        return bpy.data.collections[collection_name]
    
    # Create new collection
    collection = bpy.data.collections.new(collection_name)
    print("Created new Collection: ", collection_name)
    
    # Link collection to the current scene
    # if bpy.context.scene:
    #     bpy.context.scene.collection.children.link(collection)
    #     print("Linked collection to scene: ", collection_name)
    
    return collection

def clear_force_collection() -> None:
    """Remove all objects from Forces collection"""
    collection = get_force_collection()
    # Make a list of objects to remove to avoid modifying the collection during iteration
    objects_to_remove = list(collection.objects)
    
    # Remove objects from the collection and from the main database
    for idx, obj in enumerate(objects_to_remove):
        collection.objects.unlink(obj)
        if obj.users:  # Only remove if no other users
            print(f"removed {obj.name}", end = " ... ")
            bpy.data.objects.remove(obj, do_unlink=True)
       
    print("Removed all objects from collection: ", collection.name)

def get_force_color(force: float, max_force: float) -> Tuple[float, float, float, float]:
    """Returns perfect green (0N) -> red/blue gradient"""
    if max_force == 0:
        return (0, 1, 0, 1)  # Pure green if no forces
    
    normalized = force / max_force  # Range [-1, 1]
    
    if normalized < 0:  # Compression (green -> red)
        print("Compression: ", normalized)
        return (min(1, -normalized), 1 - min(1, -normalized), 0, 1)
    else:  # Tension (green -> blue)
        print("Tension: ", normalized)
        return (0, 1 - min(1, normalized), min(1, normalized), 1)

def create_force_material(edge_key: Tuple[int, int], force: float, 
                         max_force: float, is_input_force: bool = False) -> bpy.types.Material:
    """Generates material with exact gradient specs"""
    mat_name = f"ForceMaterial_{edge_key[0]}_{edge_key[1]}"
    mat = bpy.data.materials.new(name=mat_name)
    mat.use_nodes = True
    
    # Get the Principled BSDF node
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    
    if is_input_force:
        bsdf.inputs["Base Color"].default_value = input_force_color
    else:
        bsdf.inputs["Base Color"].default_value = get_force_color(force, max_force)
    
    return mat

def create_force_visual(collection: bpy.types.Collection, 
                       edge: Tuple[int, int], 
                       force: float, 
                       midpoint: Vector, 
                       direction: Vector, 
                       max_force: float) -> bpy.types.Object:
    """Creates force visualization in specified collection"""
    # Check if this is an input force (applied to a vertex)
    is_input = edge[0] == selected_vertex_index or edge[1] == selected_vertex_index
    
    # Store the current active object to restore later
    prev_active = bpy.context.view_layer.objects.active
    
    try:
        # Create object based on force magnitude
        if abs(force) < 0.001:  # Zero force - create sphere
            bpy.ops.mesh.primitive_uv_sphere_add(
                radius=base_radius * 1.5,
                location=midpoint
            )
        else:  # Force present - create cylinder
            bpy.ops.mesh.primitive_cylinder_add(
                vertices=16,
                radius=base_radius * (0.5 + 0.5 * abs(force)/max_force) if max_force > 0 else base_radius,
                depth=direction.length * 0.8,
                location=midpoint
            )
            
            # Get the created object
            vis_obj = bpy.context.view_layer.objects.active
            if vis_obj is None:
                print("ERROR: Failed to create visualization object!")
                return None
                
            # Rotate cylinder to align with edge
            rot_quat = direction.to_track_quat('Z', 'Y')
            if force < 0:  # Reverse for compression
                rot_quat = rot_quat @ Quaternion((0, 0, 1), math.pi)
            vis_obj.rotation_euler = rot_quat.to_euler()
            
            # Set name and material
            vis_obj.name = f"ForceVis_{edge[0]}_{edge[1]}"
            mat = create_force_material(edge, force, max_force, is_input)
            vis_obj.data.materials.append(mat)
            collection.objects.link(vis_obj)
            
            return vis_obj
        
        # For sphere case (zero force)
        vis_obj = bpy.context.view_layer.objects.active
        if vis_obj is None:
            print("ERROR: Failed to create visualization object!")
            return None
            
        vis_obj.name = f"ForceVis_{edge[0]}_{edge[1]}"
        mat = create_force_material(edge, force, max_force, is_input)
        vis_obj.data.materials.append(mat)
        collection.objects.link(vis_obj)
        
        return vis_obj
        
    finally:
        # Restore the previous active object
        if prev_active and prev_active.name in bpy.data.objects:
            bpy.context.view_layer.objects.active = prev_active

def visualize_forces(vertices: List[Vector], 
                    edges: List[Tuple[int, int]], 
                    edge_forces: Dict[Tuple[int, int], float], 
                    max_force: float) -> None:
    """Creates visual elements for forces"""
    # Store current mode and selection
    current_mode = bpy.context.object.mode if bpy.context.object else 'OBJECT'
    if current_mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    
    force_collection = get_force_collection()
    
    try:
        # Clear existing visualizations
        clear_force_collection()
        
        valid_edge_count = 0
        # Create visualizations for each edge
        for edge in edges:
            force = edge_forces.get(edge, 0)
            v1, v2 = vertices[edge[0]], vertices[edge[1]]
            midpoint = (v1 + v2) / 2
            direction = v2 - v1
            
            # Skip if direction length is zero
            if direction.length < 0.0001:
                continue
                
            # Create force visualization
            print("Creating force visualization for edge: ", edge, " with force: ", force)
            create_force_visual(force_collection, edge, force, midpoint, direction, max_force)
            valid_edge_count += 1
            # Create force label
            bpy.ops.object.text_add(location=midpoint + direction.normalized() * base_radius * 3)
            text = bpy.context.view_layer.objects.active
            if text is None:
                print(f"ERROR: Failed to create text label for edge {edge}")
                continue
                
            # Set text properties
            text.name = f"ForceLabel_{edge[0]}_{edge[1]}"
            text.data.body = f"{force:.1f} N"
            text.data.align_x = 'CENTER'
            text.data.align_y = 'CENTER'
            text.scale = (text_scale, text_scale, text_scale)
            
            # Get the active object for orientation
            active_obj = bpy.context.view_layer.objects.active
            if active_obj is not None and hasattr(active_obj.data, 'materials') and active_obj.data.materials:
                text.rotation_euler = active_obj.rotation_euler
                text.rotation_euler.x += math.pi/2
                text.active_material = active_obj.data.materials[0]
            
            # Add to collection
            force_collection.objects.link(text)
        print("Generated force visualizations for ", valid_edge_count, " edges")
    except Exception as e:
        print(f"Error in visualize_forces: {str(e)}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Restore previous mode
        if current_mode != 'OBJECT':
            try:
                bpy.ops.object.mode_set(mode=current_mode)
            except Exception as e:
                print(f"Error restoring mode: {e}")
