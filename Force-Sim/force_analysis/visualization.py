"""
Visualization functions for force analysis.
"""
import bpy
from mathutils import Vector
from typing import Dict, Tuple, List, Any


def get_force_collection():
    """Get or create the Forces collection"""
    if "Forces" not in bpy.data.collections:
        collection = bpy.data.collections.new("Forces")
        bpy.context.scene.collection.children.link(collection)
    return bpy.data.collections["Forces"]


def clear_force_collection():
    """Remove all objects from Forces collection"""
    collection = get_force_collection()
    for obj in list(collection.objects):
        bpy.data.objects.remove(obj, do_unlink=True)


def get_force_color(force: float, max_force: float) -> Tuple[float, float, float, float]:
    """Returns perfect green (0N) -> red/blue gradient"""
    if max_force == 0:
        return (0, 1, 0, 1)  # Green for zero force
    
    # Normalize force to 0-1 range
    normalized = abs(force) / max_force
    
    if force > 0:  # Tension (red)
        return (normalized, 1 - normalized, 0, 1)
    else:  # Compression (blue)
        return (0, 1 - normalized, normalized, 1)


def create_force_material(edge_key: Tuple[int, int], force: float, 
                         max_force: float, is_input_force: bool = False) -> bpy.types.Material:
    """Generates material with exact gradient specs"""
    mat_name = f"Force_{edge_key[0]}_{edge_key[1]}"
    
    # Check if material already exists
    if mat_name in bpy.data.materials:
        return bpy.data.materials[mat_name]
    
    mat = bpy.data.materials.new(name=mat_name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    
    # Clear default nodes
    for node in nodes:
        nodes.remove(node)
    
    # Create shader nodes
    output = nodes.new('ShaderNodeOutputMaterial')
    emission = nodes.new('ShaderNodeEmission')
    
    if is_input_force:
        # White for input force
        emission.inputs[0].default_value = (1, 1, 1, 1)
    else:
        # Color based on force type and magnitude
        color = get_force_color(force, max_force)
        emission.inputs[0].default_value = color
    
    emission.inputs[1].default_value = 2.0  # Strength
    
    # Link nodes
    mat.node_tree.links.new(emission.outputs[0], output.inputs[0])
    
    return mat

def create_force_visual(collection, edge: Tuple[int, int], force: float, 
                       midpoint: Vector, direction: Vector, max_force: float):
    """Creates force visualization in specified collection"""
    # Create cylinder for force vector
    bpy.ops.mesh.primitive_cylinder_add(
        radius=0.05,
        depth=force/1000,
        location=midpoint,
        rotation=direction.to_track_quat('Z', 'Y').to_euler()
    )
    
    cylinder = bpy.context.active_object
    cylinder.scale.z = 0.5  # Adjust length
    
    # Create cone for arrowhead
    bpy.ops.mesh.primitive_cone_add(
        radius1=0.1,
        radius2=0,
        depth=0.2,
        location=midpoint + direction.normalized() * (force/2000 + 0.1),
        rotation=direction.to_track_quat('Z', 'Y').to_euler()
    )
    
    cone = bpy.context.active_object
    
    # Create empty for grouping
    bpy.ops.object.empty_add(location=midpoint)
    empty = bpy.context.active_object
    empty.name = f"Force_{edge[0]}_{edge[1]}"
    
    # Parent objects to empty
    cylinder.parent = empty
    cone.parent = empty
    
    # Set materials
    is_input_force = (edge[0] == edge[1])  # Input force is a self-edge
    mat = create_force_material(edge, force, max_force, is_input_force)
    
    for obj in [cylinder, cone]:
        if obj.data.materials:
            obj.data.materials[0] = mat
        else:
            obj.data.materials.append(mat)
    
    # Add to collection
    for obj in [empty, cylinder, cone]:
        collection.objects.link(obj)
        bpy.context.collection.objects.unlink(obj)
    
    return empty

def visualize_forces(vertices: List[Vector], edges: List[Tuple[int, int]], 
                    edge_forces: Dict[Tuple[int, int], float], 
                    max_force: float, selected_vertex_index: int) -> None:
    """Creates visual elements for forces"""
    # Clear existing visualizations
    clear_force_collection()
    collection = get_force_collection()
    
    # Visualize edge forces
    for (v1, v2), force in edge_forces.items():
        if abs(force) < 0.1:  # Skip very small forces
            continue
            
        # Calculate midpoint and direction
        point1 = vertices[v1]
        point2 = vertices[v2]
        midpoint = (point1 + point2) / 2
        direction = (point2 - point1).normalized()
        
        # Create force visualization
        create_force_visual(collection, (v1, v2), force, midpoint, direction, max_force)
    
    # Visualize input force if applicable
    if selected_vertex_index is not None and selected_vertex_index < len(vertices):
        force_vector = Vector((
            force_components['x'],
            force_components['y'],
            force_components['z']
        )) * force_magnitude
        
        if force_vector.length > 0.1:  # Only show if significant force
            point = vertices[selected_vertex_index]
            direction = force_vector.normalized()
            create_force_visual(
                collection,
                (selected_vertex_index, selected_vertex_index),  # Self-edge for input force
                force_vector.length,
                point + direction * 0.1,  # Slight offset from vertex
                direction,
                max_force
            )
