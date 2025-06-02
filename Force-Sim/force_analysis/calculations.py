"""
Force calculation functions.

This module contains functions for calculating forces in the truss structure.
"""

import bmesh
from mathutils import Vector
from typing import List, Dict, Tuple, Any, Optional

def get_mesh_data(obj) -> tuple[list[Vector], list[tuple[int, int]]]:
    """Extracts raw mesh topology and coordinates
    
    Args:
        obj: Blender mesh object
        
    Returns:
        tuple: (vertices, edges) where vertices is a list of Vector objects
               and edges is a list of tuples (v1_idx, v2_idx)
    """
    if not obj or not hasattr(obj, 'data') or not hasattr(obj.data, 'vertices'):
        print("Invalid mesh object")
        return [], []
    
    bm = bmesh.new()
    try:
        bm.from_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        
        vertices = [vert.co.copy() for vert in bm.verts]
        edges = [(edge.verts[0].index, edge.verts[1].index) for edge in bm.edges]
        
        return vertices, edges
    except Exception as e:
        print(f"Error extracting mesh data: {e}")
        return [], []
    finally:
        bm.free()

def calculate_forces(vertices: List[Vector], 
                    edges: List[Tuple[int, int]], 
                    force_components: Dict[str, float], 
                    force_magnitude: float,
                    selected_vertex_index: Optional[int] = None) -> Tuple[Dict, float]:
    """Calculate forces in the truss structure based on the selected vertex.
    
    This implementation uses a simplified method of joints approach to distribute
    forces through the truss structure.
    
    Args:
        vertices: List of vertex coordinates
        edges: List of edge tuples (v1_idx, v2_idx)
        force_components: Dictionary of force components (x, y, z)
        force_magnitude: Base magnitude multiplier
        selected_vertex_index: Index of the vertex where force is applied
        
    Returns:
        tuple: (edge_forces_dict, max_force)
    """
    # Input validation
    if not vertices or not edges:
        print("No vertices or edges provided")
        return {}, 0.0
        
    if not force_components or not all(k in force_components for k in ['x', 'y', 'z']):
        print("Invalid force components")
        return {}, 0.0
    
    # Initialize return values
    edge_forces = {}
    max_force = 0.0
    
    try:
        # Create normalized edge representation
        normalized_edges = [tuple(sorted(edge)) for edge in edges]
        edge_forces = {edge: 0.0 for edge in normalized_edges}
        
        if selected_vertex_index is not None and 0 <= selected_vertex_index < len(vertices):
            # Calculate the actual force vector from components
            try:
                force_vector = Vector((
                    float(force_components['x']),
                    float(force_components['y']),
                    float(force_components['z'])
                )).normalized() * abs(float(force_magnitude))
                
                print(f"Applying force {force_vector} at vertex {selected_vertex_index}")
                
                # Find all edges connected to the selected vertex
                connected_edges = [
                    edge for edge in edges 
                    if selected_vertex_index in edge and 
                       edge[0] < len(vertices) and 
                       edge[1] < len(vertices)
                ]
                
                if connected_edges:
                    # Calculate the total force magnitude
                    total_force = force_vector.length
                    
                    # Distribute force to connected edges based on angle
                    total_weight = 0.0
                    edge_weights = {}
                    
                    # First pass: calculate weights based on angle to force direction
                    for edge in connected_edges:
                        other_vertex = edge[1] if edge[0] == selected_vertex_index else edge[0]
                        edge_vector = vertices[other_vertex] - vertices[selected_vertex_index]
                        
                        if edge_vector.length_squared < 0.0001:  # Skip zero-length edges
                            continue
                            
                        edge_dir = edge_vector.normalized()
                        # Weight is higher for edges more aligned with the force
                        weight = max(0.1, abs(force_vector.normalized().dot(edge_dir)))
                        edge_weights[edge] = weight
                        total_weight += weight
                    
                    if total_weight > 0:
                        # Second pass: distribute force based on weights
                        for edge, weight in edge_weights.items():
                            normalized_edge = tuple(sorted(edge))
                            force_fraction = weight / total_weight
                            edge_vector = vertices[edge[1]] - vertices[edge[0]]
                            edge_dir = edge_vector.normalized()
                            
                            # Project force onto edge direction
                            force_mag = force_vector.dot(edge_dir) * force_fraction
                            
                            # Store the force (positive for tension, negative for compression)
                            edge_forces[normalized_edge] = force_mag
                            
                            # Update max force for visualization scaling
                            if abs(force_mag) > abs(max_force):
                                max_force = force_mag
                                
                        # Distribute remaining force to other edges in the structure
                        # This is a simplified approach - a full truss analysis would be better
                        for edge in normalized_edges:
                            if edge not in edge_forces or edge_forces[edge] == 0:
                                # Apply a small fraction of the force to other edges
                                # based on their length and orientation
                                v1, v2 = vertices[edge[0]], vertices[edge[1]]
                                edge_vector = v2 - v1
                                if edge_vector.length_squared < 0.0001:
                                    continue
                                    
                                edge_dir = edge_vector.normalized()
                                force_mag = force_vector.dot(edge_dir) * 0.1  # Dampen the effect
                                edge_forces[edge] = force_mag
                                
                                if abs(force_mag) > abs(max_force):
                                    max_force = force_mag
            except Exception as e:
                print(f"Error calculating force vector: {e}")
                import traceback
                traceback.print_exc()
                return {}, 0.0
        
        return edge_forces, abs(max_force) if max_force != 0.0 else 1.0
        
    except Exception as e:
        print(f"Error in calculate_forces: {e}")
        import traceback
        traceback.print_exc()
        return {}, 0.0
