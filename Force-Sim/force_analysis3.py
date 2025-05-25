import bpy
import bmesh
from mathutils import Vector, Quaternion
import math
from enum import Enum
from typing import List, Dict, Optional, Tuple

# ===== SUPPORT SYSTEM =====
class SupportType(Enum):
    FIXED = 'fixed'     # All DOFs constrained (X,Y,Z translations)
    PINNED = 'pinned'   # X,Y translations constrained, Z rotation free
    ROLLER = 'roller'   # Single axis constraint (specified by direction)

class Support:
    """Represents a support constraint in the structural analysis.
    
    Attributes:
        vertex_index: Index of the vertex where support is applied
        support_type: Type of support (FIXED, PINNED, ROLLER)
        direction: For ROLLER supports, the normal vector of the rolling plane
        reaction_forces: Calculated reaction forces at this support
    """
    def __init__(self, vertex_index: int, support_type: SupportType = SupportType.FIXED, 
                 direction: Optional[Vector] = None):
        self.vertex_index = vertex_index
        self.support_type = support_type
        self.direction = direction.normalized() if direction else Vector((0, 0, 1))
        self.reaction_forces = None
    
    def get_constraints(self) -> List[bool]:
        """Returns a list of boolean constraints [x, y, z] where True means constrained."""
        if self.support_type == SupportType.FIXED:
            return [True, True, True]  # All translations constrained
        elif self.support_type == SupportType.PINNED:
            return [True, True, False]  # X,Y constrained, Z free
        elif self.support_type == SupportType.ROLLER:
            # Only constrain movement in the direction normal to the rolling plane
            normal = self.direction
            # This is simplified - in practice, would need to project onto global axes
            return [bool(abs(normal.x) > 0.9), bool(abs(normal.y) > 0.9), bool(abs(normal.z) > 0.9)]
        return [False, False, False]

# Global support storage
supports: Dict[int, Support] = {}

def add_support(vertex_index: int, support_type: SupportType, direction: Optional[Vector] = None) -> bool:
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

def get_supports() -> List[Support]:
    """Get a list of all supports."""
    return list(supports.values())

def get_support(vertex_index: int) -> Optional[Support]:
    """Get the support at the given vertex, or None if none exists."""
    return supports.get(vertex_index)

def has_support(vertex_index: int) -> bool:
    """Check if a support exists at the given vertex."""
    return vertex_index in supports
# =========================

# ===== SETTINGS =====
# Force configuration
force_components = {
    'x': -100,      # X component (horizontal)
    'y': 100.0,      # Y component (horizontal)
    'z': 101.0      # Z component (vertical, default downward)
}
force_magnitude = 1.0  # Base magnitude multiplier
selected_vertex_index = None  # Force application vertex
support_vertex_indices = []   # Support vertices
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
    """Creates force visualization in specified collection"""
    is_input = edge[0] == selected_vertex_index or edge[1] == selected_vertex_index
    
    # Create object based on force magnitude
    if abs(force) < 0.001:  # Zero force - create sphere
        bpy.ops.mesh.primitive_uv_sphere_add(
            radius=base_radius * 1.5,
            location=midpoint
        )
    else:  # Force present - create cylinder
        bpy.ops.mesh.primitive_cylinder_add(
            vertices=16,
            radius=base_radius * (0.5 + 0.5 * abs(force)/max_force),
            depth=direction.length * 0.8,
            location=midpoint
        )
        # Store the active object before modifying it
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
    
    # For sphere case
    vis_obj = bpy.context.view_layer.objects.active
    if vis_obj is None:
        print("ERROR: Failed to create visualization object!")
        return None
        
    vis_obj.name = f"ForceVis_{edge[0]}_{edge[1]}"
    mat = create_force_material(edge, force, max_force, is_input)
    vis_obj.data.materials.append(mat)
    collection.objects.link(vis_obj)
    
    return vis_obj

def visualize_forces(vertices, edges, edge_forces, max_force):
    """Creates visual elements for forces"""
    force_collection = get_force_collection()
    
    for edge in edges:
        force = edge_forces.get(edge, 0)
        v1, v2 = vertices[edge[0]], vertices[edge[1]]
        midpoint = (v1 + v2) / 2
        direction = v2 - v1
        
        # Create force visualization
        create_force_visual(force_collection, edge, force, midpoint, direction, max_force)
        
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
        
        # Store the active object before modifying it
        active_obj = bpy.context.view_layer.objects.active
        if active_obj is None:
            print(f"ERROR: No active object found for edge {edge}")
            continue
            
        # Orient text
        text.rotation_euler = active_obj.rotation_euler
        text.rotation_euler.x += math.pi/2
        
        # Only set material if the active object has materials
        if hasattr(active_obj.data, 'materials') and active_obj.data.materials:
            text.active_material = active_obj.data.materials[0]
        
        # Add to collection
        force_collection.objects.link(text)

# --------------------------
# EXECUTION CONTROL
# --------------------------

def get_selected_vertex_index():
    """Get the selected vertex index in Edit Mode"""
    obj = bpy.context.active_object
    if not obj or obj.type != 'MESH':
        print("ERROR: Select a mesh object!")
        return None
    
    # Get the selected vertex in Edit Mode
    bpy.ops.object.mode_set(mode='EDIT')
    
    try:
        bm = bmesh.from_edit_mesh(obj.data)
        if bm is None:
            print("ERROR: Could not get BMesh data!")
            return None
            
        bm.verts.ensure_lookup_table()
        selected_verts = [v for v in bm.verts if v.select]
        
        if len(selected_verts) == 1:
            return selected_verts[0].index
        elif len(selected_verts) > 1:
            print("WARNING: Multiple vertices selected. Using first selected vertex.")
            return selected_verts[0].index
        else:
            print("WARNING: No vertex selected. Using vertex 0 as default.")
            return 0
            
    finally:
        # Always return to OBJECT mode
        bpy.ops.object.mode_set(mode='OBJECT')
        
        # Free BMesh if it exists
        if 'bm' in locals():
            bm.free()

def detect_support_vertices(vertices):
    """Automatically detect support vertices based on lowest Z coordinates"""
    # Sort vertices by Z coordinate (lowest to highest)
    sorted_verts = sorted(enumerate(vertices), key=lambda x: x[1].z)
    
    # Take the lowest 3 vertices as supports
    support_indices = [idx for idx, _ in sorted_verts[:3]]
    
    print(f"Automatically detected support vertices: {support_indices}")
    return support_indices

def select_vertices():
    """Vertex selection system with automatic support detection"""
    obj = bpy.context.active_object
    if not obj or obj.type != 'MESH':
        print("ERROR: Select a mesh object!")
        return False

    # Get vertices data
    vertices, _ = get_mesh_data(obj)
    
    # Get force application vertex
    global selected_vertex_index
    selected_vertex_index = get_selected_vertex_index()
    if selected_vertex_index is None:
        return False
    
    print(f"\n=== Vertex Selection ===")
    print(f"Force application vertex: {selected_vertex_index}")

    # Get support vertices
    global support_vertex_indices
    support_vertex_indices = []
    
    while True:
        try:
            support_input = input("\nSupport vertices (comma-separated, or 'auto' for automatic detection): ").lower()
            
            if support_input == 'auto' or support_input == '':
                support_vertex_indices = detect_support_vertices(vertices)
                print(f"Using automatic support detection: {support_vertex_indices}")
                break
                
            indices = [int(idx.strip()) for idx in support_input.split(',')]
            support_vertex_indices = indices
            print(f"Support vertices: {support_vertex_indices}")
            break
            
        except ValueError:
            print("ERROR: Please enter valid vertex indices or 'auto'!")
            continue
        except Exception as e:
            print(f"ERROR: {str(e)}")
            continue

    # Create visual feedback
    collection = get_force_collection()
    
    # Force application vertex marker
    v = obj.data.vertices[selected_vertex_index]
    marker = bpy.data.objects.new(f"ForceMarker", bpy.data.meshes.new("ForceMarker"))
    marker.location = v.co
    marker.scale = (0.1, 0.1, 0.1)
    marker.show_name = True
    marker.name = "Force Application Point"
    collection.objects.link(marker)
    
    # Support vertex markers
    for idx in support_vertex_indices:
        try:
            v = obj.data.vertices[idx]
            marker = bpy.data.objects.new(f"SupportMarker_{idx}", bpy.data.meshes.new("SupportMarker"))
            marker.location = v.co
            marker.scale = (0.08, 0.08, 0.08)
            marker.show_name = True
            marker.name = f"Support Point {idx}"
            collection.objects.link(marker)
        except IndexError:
            print(f"WARNING: Vertex index {idx} is out of range!")
            continue

    return True

def run_analysis():
    """Main coordinator"""
    obj = bpy.context.active_object
    if not obj or obj.type != 'MESH':
        print("ERROR: Select a mesh object!")
        return

    # First, let user select vertices
    if not select_vertices():
        return

    if bpy.context.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')

    clear_force_collection()
    vertices, edges = get_mesh_data(obj)
    
    # Check if we have a force application point
    if selected_vertex_index is None:
        print("ERROR: No force application vertex selected!")
        return
    
    # Check if we have enough supports
    if len(support_vertex_indices) < 3:
        print("WARNING: Less than 3 support points selected. Structure may be unstable.")
    
    edge_forces, max_force = calculate_forces(vertices, edges, force_components, force_magnitude)
    visualize_forces(vertices, edges, edge_forces, max_force)

if __name__ == "__main__":
    run_analysis()