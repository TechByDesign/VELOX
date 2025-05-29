"""
Force calculation functions for structural analysis.
"""
from mathutils import Vector
from typing import List, Dict, Tuple, Any
import bmesh
from .models import Support
from .settings import supports


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


def calculate_forces(vertices: List[Vector], edges: List[Tuple[int, int]], 
                    force_components: Dict[str, float], force_magnitude: float,
                    selected_vertex_index: int) -> Tuple[Dict, float]:
    """Calculate forces in the truss structure based on the selected vertex.
    
    Args:
        vertices: List of vertex coordinates
        edges: List of edge tuples (v1_idx, v2_idx)
        force_components: Dictionary of force components (x, y, z)
        force_magnitude: Base magnitude multiplier
        selected_vertex_index: Index of vertex where force is applied
        
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
            edge_direction = (vertices[neighbor] - vertices[current_vertex]).normalized()
            force_magnitude = weight / total_weight * current_force.length
            edge_forces[edge] = force_magnitude
            
            # Add neighbor to queue with remaining force
            if neighbor not in visited:
                visited.add(neighbor)
                remaining_force = edge_direction * force_magnitude
                queue.append((neighbor, remaining_force))
    
    # Find maximum force for normalization
    max_force = max(abs(f) for f in edge_forces.values()) if edge_forces else 1.0
    
    return edge_forces, max_force
