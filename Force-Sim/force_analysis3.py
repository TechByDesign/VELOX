import bpy
import bmesh
from mathutils import Vector, Quaternion
import math

# ===== SETTINGS =====
# Force configuration
force_components = {
    'x': -100,      # X component (horizontal)
    'y': 100.0,      # Y component (horizontal)
    'z': 101.0      # Z component (vertical, default downward)
}
force_magnitude = 1.0  # Base magnitude multiplier
selected_vertex_index = 15
base_radius = 0.05  # Base radius for all elements
text_scale = 0.08
input_force_color = (1, 1, 1, 1)  # White for input force
# ====================

# --------------------------
# COLLECTION MANAGEMENT
# --------------------------

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

# --------------------------
# DATA CALCULATION FUNCTIONS 
# --------------------------

def get_mesh_data(obj):
    """Extracts raw mesh topology and coordinates"""
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    
    vertices = [vert.co.copy() for vert in bm.verts]
    edges = [(edge.verts[0].index, edge.verts[1].index) for edge in bm.edges]
    
    bm.free()
    return vertices, edges

def calculate_forces(vertices, edges, force_components, force_magnitude):
    """Calculate forces in the truss structure based on the selected vertex.
    
    Args:
        vertices: List of vertex coordinates
        edges: List of edge tuples (v1_idx, v2_idx)
        force_components: Dictionary of force components (x, y, z)
        force_magnitude: Base magnitude multiplier
        
    Returns:
        tuple: (edge_forces_dict, max_force)
    """
    # Calculate the actual force vector from components
    force_vector = Vector((
        force_components['x'],
        force_components['y'],
        force_components['z']
    ))
    
    # Scale by magnitude
    force_vector *= force_magnitude
    
    print(f"Applying force {force_vector} at vertex {selected_vertex_index}")
    
    # Create normalized edge representation
    normalized_edges = [tuple(sorted(edge)) for edge in edges]
    edge_forces = {edge: 0.0 for edge in normalized_edges}
    
    # If no vertex is selected, return zero forces
    if selected_vertex_index is None or selected_vertex_index >= len(vertices):
        print("No valid vertex selected for force application")
        return edge_forces, 1.0
    
    # Create adjacency list
    adjacency = {i: [] for i in range(len(vertices))}
    for v1, v2 in edges:
        adjacency[v1].append(v2)
        adjacency[v2].append(v1)
    
    # Initialize BFS
    visited = set([selected_vertex_index])
    queue = [(selected_vertex_index, force_vector)]
    
    while queue:
        current_vertex, current_force = queue.pop(0)
        neighbors = adjacency[current_vertex]
        
        if not neighbors:
            continue
            
        # Calculate total weight for force distribution
        total_weight = 0
        weights = []
        for neighbor in neighbors:
            # Calculate weight based on angle from force vector
            v1 = vertices[current_vertex]
            v2 = vertices[neighbor]
            direction = (v2 - v1).normalized()
            
            # Calculate dot product to get projection
            projection = abs(direction.dot(current_force.normalized()))
            weight = projection + 0.1  # Add small base weight
            weights.append((neighbor, weight))
            total_weight += weight
        
        # Distribute force to neighbors
        for neighbor, weight in weights:
            edge = tuple(sorted((current_vertex, neighbor)))
            
            if edge not in edge_forces:
                print(f"Warning: Edge {edge} not found in original edges")
                continue
                
            # Calculate force magnitude and direction
            edge_direction = (v2 - v1).normalized()
            force_magnitude = current_force.length * (weight / total_weight)
            
            # Determine force direction based on dot product
            force_direction = edge_direction.dot(current_force.normalized())
            force = force_magnitude * force_direction
            
            # Add to edge force
            edge_forces[edge] += force
            
            # Continue BFS if not visited
            if neighbor not in visited:
                visited.add(neighbor)
                # Reduce force as we move away from source
                queue.append((neighbor, current_force * 0.8))  # Damping factor
    
    # Find the maximum force magnitude for normalization
    max_force = max(abs(f) for f in edge_forces.values()) if edge_forces else 1.0
    print(f"Max force calculated: {max_force}")
    
    return edge_forces, max_force

# --------------------------
# VISUALIZATION FUNCTIONS
# --------------------------

def get_force_color(force, max_force):
    """Returns perfect green (0N) -> red/blue gradient"""
    if max_force == 0:
        return (0, 1, 0, 1)  # Pure green if no forces
    
    normalized = force / max_force  # Range [-1, 1]
    
    if normalized < 0:  # Compression (green -> red)
        return (min(1, -normalized), 1 - min(1, -normalized), 0, 1)
    else:  # Tension (green -> blue)
        return (0, 1 - min(1, normalized), min(1, normalized), 1)

def create_force_material(edge_key, force, max_force, is_input_force):
    """Generates material with exact gradient specs"""
    mat = bpy.data.materials.new(name=f"ForceMaterial_{edge_key[0]}_{edge_key[1]}")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    
    if is_input_force:
        bsdf.inputs["Base Color"].default_value = input_force_color
    else:
        bsdf.inputs["Base Color"].default_value = get_force_color(force, max_force)
    return mat

def create_force_visual(collection, edge, force, midpoint, direction, max_force):
    """Creates either sphere (0N) or tapered cylinder (force) in specified collection"""
    is_input = edge[0] == selected_vertex_index or edge[1] == selected_vertex_index
    
    if abs(force) < 0.001:  # Zero force - create sphere
        bpy.ops.mesh.primitive_uv_sphere_add(
            radius=base_radius * 1.5,
            location=midpoint
        )
        vis_obj = bpy.context.object
        vis_obj.name = f"ForceVis_{edge[0]}_{edge[1]}"
    else:  # Force present - create tapered cylinder
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=16,
            radius=base_radius * (0.5 + 0.5 * abs(force)/max_force),
            depth=direction.length * 0.8,  # 80% of edge length
            location=midpoint
        )
        vis_obj = bpy.context.object
        vis_obj.name = f"ForceVis_{edge[0]}_{edge[1]}"
        
        # Rotate cylinder to align with edge
        rot_quat = direction.to_track_quat('Z', 'Y')
        if force < 0:  # Reverse for compression
            rot_quat = rot_quat @ Quaternion((0, 0, 1), math.pi)
        vis_obj.rotation_euler = rot_quat.to_euler()
    
    # Assign material and add to collection
    mat = create_force_material(edge, force, max_force, is_input)
    vis_obj.data.materials.append(mat)
    collection.objects.link(vis_obj)
    bpy.context.collection.objects.unlink(vis_obj)  # Remove from main collection
    
    return vis_obj

def visualize_forces(vertices, edges, edge_forces, max_force):
    """Creates all visual elements in Forces collection"""
    force_collection = get_force_collection()
    
    for edge in edges:
        force = edge_forces.get(edge, 0)
        v1, v2 = vertices[edge[0]], vertices[edge[1]]
        midpoint = (v1 + v2) / 2
        direction = v2 - v1
        
        # Create visual element
        vis_obj = create_force_visual(force_collection, edge, force, midpoint, direction, max_force)
        
        # Create and position label
        bpy.ops.object.text_add(
            location=midpoint + direction.normalized() * base_radius * 3
        )
        text = bpy.context.object
        text.name = f"ForceLabel_{edge[0]}_{edge[1]}"
        text.data.body = f"{force:.1f} N"
        text.data.align_x = 'CENTER'
        text.data.align_y = 'CENTER'
        text.scale = (text_scale, text_scale, text_scale)
        
        # Make text face camera
        text.rotation_euler = vis_obj.rotation_euler
        text.rotation_euler.x += math.pi/2  # Stand text upright
        text.active_material = vis_obj.data.materials[0]
        
        # Add text to collection
        force_collection.objects.link(text)
        bpy.context.collection.objects.unlink(text)

# --------------------------
# EXECUTION CONTROL
# --------------------------

def run_analysis():
    """Main coordinator"""
    obj = bpy.context.active_object
    if not obj or obj.type != 'MESH':
        print("ERROR: Select a mesh object!")
        return

    if bpy.context.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')

    clear_force_collection()
    vertices, edges = get_mesh_data(obj)
    edge_forces, max_force = calculate_forces(vertices, edges, force_components, force_magnitude)
    visualize_forces(vertices, edges, edge_forces, max_force)

if __name__ == "__main__":
    run_analysis()