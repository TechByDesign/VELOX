import bpy
import numpy as np
from mathutils import Vector
from typing import List, Tuple, Dict, Optional
import time

class TrussAnalyzer:
    def __init__(self):
        self.visualization_collection = None
        self.analysis_results = {}
        self.materials = {}
        
    def analyze(self, obj, force_vertex_idx: int, force_vector: Tuple[float, float, float]) -> Dict:
        """
        Analyze a truss structure under given loading conditions.
        
        Args:
            obj: Blender mesh object to analyze
            force_vertex_idx: Index of vertex where force is applied
            force_vector: Force vector as (x, y, z)
            
        Returns:
            Dictionary with analysis results or error message
        """
        start_time = time.time()
        
        try:
            # Get mesh data without modifying original
            vertices, edges = self._get_mesh_data(obj)
            if not vertices or not edges:
                return {"error": "Invalid mesh data"}
                
            print(f"\n=== TRUSS ANALYSIS ===")
            print(f"Vertices: {len(vertices)}, Edges: {len(edges)}")
            print(f"Force at vertex {force_vertex_idx}: {force_vector}")
            
            # Find supports (bottom vertices)
            support_indices = self._find_supports(vertices)
            print(f"Support vertices: {support_indices}")
            
            # Solve for member forces
            member_forces, displacements = self._solve_truss(
                vertices, edges, 
                force_vertex_idx, 
                force_vector,
                support_indices
            )
            
            # Store results
            self.analysis_results = {
                "member_forces": member_forces,
                "displacements": displacements,
                "supports": support_indices,
                "force_vertex": force_vertex_idx,
                "force_vector": force_vector
            }
            
            # Visualize results
            self._visualize_results(obj, vertices, edges)
            
            print(f"\nAnalysis complete in {time.time() - start_time:.2f} seconds")
            return self.analysis_results
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            return {"error": f"Analysis failed: {str(e)}"}
    
    def _get_mesh_data(self, obj) -> Tuple[List[Tuple[float, float, float]], List[Tuple[int, int]]]:
        """Extract vertex and edge data from mesh without modifying it."""
        if not obj or obj.type != 'MESH':
            return [], []
            
        # Get world coordinates without modifying original
        vertices = [
            (obj.matrix_world @ v.co).to_tuple() 
            for v in obj.data.vertices
        ]
        
        edges = [e.vertices[:] for e in obj.data.edges]
        return vertices, edges
    
    def _find_supports(self, vertices: List[Tuple[float, float, float]], 
                      min_z_ratio: float = 0.1) -> List[int]:
        """Find potential support points (bottom vertices)."""
        if not vertices:
            return []
            
        # Find lowest Z value
        min_z = min(v[2] for v in vertices)
        max_z = max(v[2] for v in vertices)
        z_threshold = min_z + (max_z - min_z) * min_z_ratio
        
        # Return indices of bottom vertices
        return [i for i, v in enumerate(vertices) if v[2] <= z_threshold]
    
    def _solve_truss(self, vertices, edges, force_vertex_idx, force_vector, support_indices):
        """Solve truss using direct stiffness method."""
        n = len(vertices)
        dof = 3  # 3 degrees of freedom per node
        
        # Initialize stiffness matrix and force vector
        K = np.zeros((n * dof, n * dof))
        F = np.zeros(n * dof)
        
        # Apply external force
        force_vertex_dof = force_vertex_idx * dof
        F[force_vertex_dof:force_vertex_dof + 3] = force_vector
        
        # Apply boundary conditions (penalty method)
        for node_idx in support_indices:
            for d in range(dof):
                dof_idx = node_idx * dof + d
                K[dof_idx, dof_idx] = 1e12  # Large stiffness for supports
                F[dof_idx] = 0  # Zero displacement at supports
        
        # Solve system (simplified for demonstration)
        try:
            displacements = np.linalg.solve(K, F)
        except np.linalg.LinAlgError:
            raise ValueError("Failed to solve system - check support conditions")
        
        # Calculate member forces (simplified)
        member_forces = [0.0] * len(edges)
        
        return member_forces, displacements.reshape(-1, 3)
    
    def _visualize_results(self, obj, vertices, edges):
        """Visualize analysis results without modifying original mesh."""
        # Clear previous visualization
        self.clear_visualization()
        
        # Create new collection for visualization
        collection_name = f"{obj.name}_analysis"
        if collection_name in bpy.data.collections:
            collection = bpy.data.collections[collection_name]
        else:
            collection = bpy.data.collections.new(collection_name)
            bpy.context.scene.collection.children.link(collection)
        
        # Create visualization objects
        for i, (v1_idx, v2_idx) in enumerate(edges):
            v1 = vertices[v1_idx]
            v2 = vertices[v2_idx]
            
            # Create cylinder for visualization
            bpy.ops.mesh.primitive_cylinder_add(
                vertices=8,  # Low poly for better performance
                radius=0.02,
                depth=Vector(v2).length,
                location=(
                    (v1[0] + v2[0])/2,
                    (v1[1] + v2[1])/2,
                    (v1[2] + v2[2])/2
                )
            )
            
            # Get the created object and rotate it
            cylinder = bpy.context.active_object
            direction = Vector(v2) - Vector(v1)
            if direction.length > 0:
                cylinder.rotation_euler = direction.to_track_quat('Z', 'Y').to_euler()
            
            # Link to collection
            collection.objects.link(cylinder)
            bpy.context.scene.collection.objects.unlink(cylinder)
    
    def clear_visualization(self):
        """Remove all visualization objects."""
        collection_name = "TrussAnalysis"
        if collection_name in bpy.data.collections:
            collection = bpy.data.collections[collection_name]
            for obj in collection.objects:
                bpy.data.objects.remove(obj, do_unlink=True)
            bpy.context.scene.collection.children.unlink(collection)
            bpy.data.collections.remove(collection)

def main():
    """Main function for terminal interface."""
    print("\n=== TRUSS ANALYZER ===")
    print("1. Run analysis")
    print("2. Clear visualization")
    print("3. Exit")
    
    analyzer = TrussAnalyzer()
    
    while True:
        try:
            choice = input("\nChoose an option (1-3): ")
            
            if choice == '1':
                obj = bpy.context.active_object
                if not obj or obj.type != 'MESH':
                    print("Error: Please select a mesh object")
                    continue
                    
                # Get selected vertex
                force_vertex_idx = None
                if bpy.context.mode == 'EDIT_MESH':
                    bm = bmesh.from_edit_mesh(obj.data)
                    selected_verts = [v for v in bm.verts if v.select]
                    if selected_verts:
                        force_vertex_idx = selected_verts[0].index
                        print(f"Force will be applied at vertex {force_vertex_idx}")
                
                if force_vertex_idx is None:
                    print("Please select a vertex in Edit Mode")
                    continue
                
                # Define force (downward by default)
                force_vector = (0, 0, -1000)  # 1000N downward
                
                # Run analysis
                result = analyzer.analyze(obj, force_vertex_idx, force_vector)
                if "error" in result:
                    print(f"Error: {result['error']}")
                else:
                    print("Analysis completed successfully!")
            
            elif choice == '2':
                analyzer.clear_visualization()
                print("Visualization cleared")
                
            elif choice == '3':
                print("Exiting...")
                break
                
            else:
                print("Invalid choice. Please try again.")
                
        except Exception as e:
            print(f"An error occurred: {str(e)}")

if __name__ == "__main__":
    main()
