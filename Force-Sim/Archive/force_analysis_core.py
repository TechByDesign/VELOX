import bpy
import bmesh
from mathutils import Vector, Quaternion
import math

# ===== SETTINGS =====
force_magnitude = 10000.0
selected_vertex_index = 0
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

def calculate_forces(vertices, edges):
    """Pure force calculation (returns edge keys with forces)"""
    edge_forces = {}
    for v1_idx, v2_idx in edges:
        v1 = vertices[v1_idx]
        v2 = vertices[v2_idx]
        # Simplified physics - replace with actual solver
        force = (v1.z - v2.z) * force_magnitude * 0.01
        edge_forces[(v1_idx, v2_idx)] = force
    
    max_force = max(abs(f) for f in edge_forces.values()) if edge_forces else 1.0
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
    edge_forces, max_force = calculate_forces(vertices, edges)
    visualize_forces(vertices, edges, edge_forces, max_force)

if __name__ == "__main__":
    run_analysis()