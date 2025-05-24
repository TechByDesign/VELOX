import bpy
import bmesh
import numpy as np
from mathutils import Vector, Matrix
import math
from collections import defaultdict

# ===== CONFIGURATION =====
FORCE_MAGNITUDE = 5000.0  # Newtons
FORCE_DIRECTION = (0, 0, -1)  # Downward force
BASE_RADIUS = 0.05
TEXT_SCALE = 0.08
DISPLAY_TEXT = True

# ===== SUPPORT DETECTION STRATEGIES =====
class SupportDetector:
    """Multiple strategies for finding valid support vertices"""
    
    @staticmethod
    def get_ground_vertices(vertices, tolerance=0.1):
        """Find vertices at ground level (lowest Z values)"""
        if not vertices:
            return []
        
        z_coords = [v[2] for v in vertices]
        min_z = min(z_coords)
        ground_verts = [i for i, v in enumerate(vertices) if abs(v[2] - min_z) <= tolerance]
        return ground_verts
    
    @staticmethod
    def get_corner_vertices(vertices):
        """Find vertices at bounding box corners"""
        if len(vertices) < 4:
            return list(range(len(vertices)))
        
        # Calculate bounding box
        x_coords = [v[0] for v in vertices]
        y_coords = [v[1] for v in vertices]
        z_coords = [v[2] for v in vertices]
        
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)
        z_min, z_max = min(z_coords), max(z_coords)
        
        # Find vertices closest to each corner
        corners = [
            (x_min, y_min, z_min), (x_max, y_min, z_min),
            (x_min, y_max, z_min), (x_max, y_max, z_min)
        ]
        
        corner_verts = []
        for corner in corners:
            distances = [(i, sum((v[j] - corner[j])**2 for j in range(3))) 
                        for i, v in enumerate(vertices)]
            closest = min(distances, key=lambda x: x[1])[0]
            if closest not in corner_verts:
                corner_verts.append(closest)
        
        return corner_verts[:4]  # Maximum 4 corner supports
    
    @staticmethod
    def get_distributed_vertices(vertices, num_supports=4):
        """Get evenly distributed vertices"""
        if len(vertices) <= num_supports:
            return list(range(len(vertices)))
        
        # Simple distribution - take every nth vertex
        step = len(vertices) // num_supports
        return [i * step for i in range(num_supports)]
    
    @staticmethod
    def validate_supports(vertices, support_indices):
        """Check if supports are non-collinear and adequate"""
        if len(support_indices) < 3:
            return False, "Need at least 3 support vertices"
        
        # Check for collinearity (2D case)
        if len(support_indices) >= 3:
            p1, p2, p3 = [vertices[i] for i in support_indices[:3]]
            
            # Calculate cross product to check collinearity
            v1 = Vector(p2) - Vector(p1)
            v2 = Vector(p3) - Vector(p1)
            cross = v1.cross(v2)
            
            if cross.length < 1e-6:
                return False, "Support vertices are collinear"
        
        # Check 3D support adequacy
        if len(support_indices) >= 3:
            support_points = [vertices[i] for i in support_indices]
            z_coords = [p[2] for p in support_points]
            
            # All supports at same Z level is problematic for 3D analysis
            if max(z_coords) - min(z_coords) < 1e-6:
                # Check if we have good XY distribution
                x_coords = [p[0] for p in support_points]
                y_coords = [p[1] for p in support_points]
                
                x_span = max(x_coords) - min(x_coords)
                y_span = max(y_coords) - min(y_coords)
                
                if x_span < 1e-6 or y_span < 1e-6:
                    return False, "Supports need better spatial distribution"
        
        return True, "Supports validated"

# ===== MATH VERIFICATION =====
class MathValidator:
    """Triple-check all mathematical operations"""
    
    @staticmethod
    def verify_stiffness_matrix(K, vertices, edges):
        """Verify stiffness matrix construction"""
        n = len(vertices)
        expected_size = 3 * n
        
        if K.shape != (expected_size, expected_size):
            raise ValueError(f"Stiffness matrix wrong size: {K.shape}, expected {(expected_size, expected_size)}")
        
        # Check symmetry
        if not np.allclose(K, K.T, rtol=1e-10):
            raise ValueError("Stiffness matrix is not symmetric")
        
        # Check that diagonal entries are non-negative (physical requirement)
        diag_entries = np.diag(K)
        if np.any(diag_entries < -1e-10):
            raise ValueError("Stiffness matrix has negative diagonal entries")
        
        return True
    
    @staticmethod
    def verify_boundary_conditions(K, F, fixed_indices):
        """Verify boundary condition application"""
        n_dof = K.shape[0]
        
        # Check that fixed DOFs have unit diagonal and zero off-diagonal
        for node_idx in fixed_indices:
            for dof in range(3):
                global_dof = 3 * node_idx + dof
                if global_dof >= n_dof:
                    continue
                
                # Check diagonal is 1 (or large penalty value)
                if K[global_dof, global_dof] < 1e-6:
                    raise ValueError(f"Fixed DOF {global_dof} not properly constrained")
                
                # Check force is zero for fixed DOF
                if abs(F[global_dof]) > 1e-10:
                    raise ValueError(f"Non-zero force at fixed DOF {global_dof}")
        
        return True

# ===== CORE ANALYSIS ENGINE =====
class EVChassisAnalyzer:
    """Robust structural analysis for EV chassis"""
    
    def __init__(self):
        self.detector = SupportDetector()
        self.validator = MathValidator()
    
    def analyze_structure(self, obj, force_vertex_idx=None, force_vector=None):
        """Main analysis function with stiffness matrix method and visualization"""
        
        # Get mesh data
        vertices, edges = self.extract_mesh_data(obj)
        if not vertices or not edges:
            return {"error": "Invalid mesh data"}
        
        print(f"=== EV CHASSIS ANALYSIS ===")
        print(f"Structure: {len(vertices)} vertices, {len(edges)} edges")
        
        # Determine force application point
        if force_vertex_idx is None:
            force_vertex_idx = self.find_load_point(vertices, edges)
        
        if force_vector is None:
            force_vector = Vector(FORCE_DIRECTION) * FORCE_MAGNITUDE
        
        print(f"🎯 Force application: Vertex {force_vertex_idx}")
        print(f"⚡ Force vector: {force_vector}")
        
        # Find supports using multiple strategies
        supports = self.find_robust_supports(vertices, force_vertex_idx)
        if "error" in supports:
            return supports
        
        support_indices = supports["indices"]
        print(f"🔒 Support vertices: {support_indices}")
        print(f"📍 Support strategy: {supports['strategy']}")
        
        # Initialize stiffness matrix and force vector
        n = len(vertices)
        K = np.zeros((3*n, 3*n))
        F = np.zeros(3*n)
        
        # Assemble stiffness matrix
        for edge in edges:
            i, j = edge
            v1, v2 = vertices[i], vertices[j]
            
            # Calculate element length and direction
            L = (Vector(v2) - Vector(v1)).length
            if L < 1e-6:
                print(f"Warning: Very short edge between vertices {i} and {j}")
                continue
                
            # Element stiffness matrix
            dx = (v2[0] - v1[0]) / L
            dy = (v2[1] - v1[1]) / L
            dz = (v2[2] - v1[2]) / L
            
            k = np.array([
                [dx*dx, dx*dy, dx*dz],
                [dy*dx, dy*dy, dy*dz], 
                [dz*dx, dz*dy, dz*dz]
            ])
            
            # Assemble into global matrix
            K[3*i:3*i+3, 3*i:3*i+3] += k
            K[3*j:3*j+3, 3*j:3*j+3] += k
            K[3*i:3*i+3, 3*j:3*j+3] -= k
            K[3*j:3*j+3, 3*i:3*i+3] -= k
        
        # Apply boundary conditions (penalty method)
        for node_idx in support_indices:
            for dof in range(3):
                global_dof = 3 * node_idx + dof
                K[global_dof, global_dof] = 1e6  # Large stiffness
                F[global_dof] = 0
        
        # Apply force
        global_dof = 3 * force_vertex_idx
        # Convert Vector to NumPy array for assignment
        force_array = np.array(force_vector)
        F[global_dof:global_dof+3] = force_array
        
        # Solve K * U = F
        try:
            U = np.linalg.solve(K, F)
        except np.linalg.LinAlgError:
            return {"error": "Matrix is singular - check support configuration"}
        
        # Calculate member forces
        member_forces = []
        for edge in edges:
            i, j = edge
            v1, v2 = vertices[i], vertices[j]
            
            # Calculate displacement difference
            u1 = U[3*i:3*i+3]
            u2 = U[3*j:3*j+3]
            delta = u2 - u1
            
            # Calculate force magnitude
            L = (Vector(v2) - Vector(v1)).length
            force = (delta[0]**2 + delta[1]**2 + delta[2]**2)**0.5 / L
            member_forces.append(force)
        
        # Visualize results
        self.visualize_results(vertices, edges, member_forces, force_vertex_idx, support_indices)
        
        results = {
            "success": True,
            "member_forces": member_forces,
            "displacements": U.reshape(-1, 3)
        }
        
        try:
            return {
                "success": True,
                "forces": results["member_forces"],
                "max_force": max(results["member_forces"]),
                "supports": support_indices,
                "displacement": max(np.linalg.norm(results["displacements"], axis=1))
            }
        except Exception as e:
            return {"error": f"Analysis failed: {str(e)}"}
    
    def extract_mesh_data(self, obj):
        """Extract vertices and edges from Blender object"""
        if obj.type != 'MESH':
            return [], []
        
        # Get mesh in world coordinates
        mesh = obj.data
        world_matrix = obj.matrix_world
        
        vertices = []
        for vert in mesh.vertices:
            world_coord = world_matrix @ vert.co
            vertices.append((world_coord.x, world_coord.y, world_coord.z))
        
        edges = []
        for edge in mesh.edges:
            edges.append((edge.vertices[0], edge.vertices[1]))
        
        return vertices, edges
    
    def find_load_point(self, vertices, edges):
        """Find appropriate load application point"""
        # Strategy: Find vertex with good connectivity that's not at ground level
        vertex_connections = defaultdict(int)
        for i, j in edges:
            vertex_connections[i] += 1
            vertex_connections[j] += 1
        
        # Find vertices above ground level
        z_coords = [v[2] for v in vertices]
        min_z = min(z_coords)
        elevated_verts = [i for i, v in enumerate(vertices) if v[2] > min_z + 0.1]
        
        if not elevated_verts:
            # Fallback to most connected vertex
            return max(vertex_connections.items(), key=lambda x: x[1])[0]
        
        # Choose elevated vertex with good connectivity
        best_vert = max(elevated_verts, key=lambda i: vertex_connections[i])
        return best_vert
    
    def find_robust_supports(self, vertices, force_vertex_idx):
        """Find supports using multiple strategies with validation"""
        
        strategies = [
            ("ground_vertices", self.detector.get_ground_vertices),
            ("corner_vertices", self.detector.get_corner_vertices),
            ("distributed_vertices", self.detector.get_distributed_vertices)
        ]
        
        for strategy_name, strategy_func in strategies:
            try:
                if strategy_name == "distributed_vertices":
                    candidates = strategy_func(vertices, 4)
                else:
                    candidates = strategy_func(vertices)
                
                # Remove force vertex from supports
                candidates = [i for i in candidates if i != force_vertex_idx]
                
                # Ensure we have enough supports
                if len(candidates) < 3:
                    continue
                
                # Take first 4 supports (3 minimum + 1 extra for stability)
                support_indices = candidates[:4]
                
                # Validate supports
                is_valid, message = self.detector.validate_supports(vertices, support_indices)
                
                if is_valid:
                    return {
                        "indices": support_indices,
                        "strategy": strategy_name,
                        "validation": message
                    }
                else:
                    print(f"⚠️  {strategy_name} failed: {message}")
                    
            except Exception as e:
                print(f"⚠️  {strategy_name} error: {str(e)}")
                continue
        
        # Ultimate fallback - use first few vertices
        fallback_supports = [i for i in range(min(4, len(vertices))) if i != force_vertex_idx]
        if len(fallback_supports) >= 3:
            print("⚠️  Using fallback support selection")
            return {
                "indices": fallback_supports,
                "strategy": "fallback",
                "validation": "minimal constraints"
            }
        
        return {"error": "Could not find adequate support configuration"}
    
    def solve_forces(self, vertices, edges, force_idx, force_vector, support_indices):
        """Solve structural forces with verified math"""
        
        n = len(vertices)
        K = np.zeros((3*n, 3*n))
        F = np.zeros(3*n)
        
        # Apply external force
        F[3*force_idx:3*force_idx+3] = [force_vector.x, force_vector.y, force_vector.z]
        
        print(f"📐 Building stiffness matrix...")
        
        # Build stiffness matrix
        valid_edges = 0
        for i, j in edges:
            if i >= n or j >= n:
                continue
                
            # Calculate member properties
            pi = Vector(vertices[i])
            pj = Vector(vertices[j])
            L = (pj - pi).length
            
            if L < 1e-10:
                print(f"⚠️  Skipping zero-length edge {i}-{j}")
                continue
            
            # Direction cosines
            dx, dy, dz = (pj - pi) / L
            
            # Local stiffness matrix
            k_local = np.array([
                [dx*dx, dx*dy, dx*dz],
                [dy*dx, dy*dy, dy*dz],
                [dz*dx, dz*dy, dz*dz]
            ])
            
            # Assemble into global matrix
            K[3*i:3*i+3, 3*i:3*i+3] += k_local
            K[3*j:3*j+3, 3*j:3*j+3] += k_local
            K[3*i:3*i+3, 3*j:3*j+3] -= k_local
            K[3*j:3*j+3, 3*i:3*i+3] -= k_local
            
            valid_edges += 1
        
        print(f"📐 Assembled {valid_edges} valid edges")
        
        # Verify stiffness matrix
        try:
            self.validator.verify_stiffness_matrix(K, vertices, edges)
        except ValueError as e:
            return {"error": f"Stiffness matrix validation failed: {e}"}
        
        # Apply boundary conditions
        print(f"🔒 Applying boundary conditions...")
        
        penalty_value = 1e6
        fixed_dofs = 0
        
        for node_idx in support_indices:
            if node_idx >= n:
                continue
            
            for dof in range(3):  # Fix all 3 DOFs for each support
                global_dof = 3 * node_idx + dof
                
                # Zero out row and column
                K[global_dof, :] = 0
                K[:, global_dof] = 0
                
                # Set diagonal to penalty value
                K[global_dof, global_dof] = penalty_value
                
                # Zero the force
                F[global_dof] = 0
                
                fixed_dofs += 1
        
        print(f"🔒 Applied {fixed_dofs} boundary conditions")
        
        # Verify boundary conditions
        try:
            self.validator.verify_boundary_conditions(K, F, support_indices)
        except ValueError as e:
            return {"error": f"Boundary condition validation failed: {e}"}
        
        # Check matrix condition
        try:
            cond_num = np.linalg.cond(K)
            print(f"📊 Matrix condition number: {cond_num:.2e}")
            
            if cond_num > 1e12:
                return {"error": f"Matrix poorly conditioned (cond={cond_num:.2e})"}
            
        except np.linalg.LinAlgError:
            return {"error": "Could not compute matrix condition number"}
        
        # Solve system
        print(f"🔢 Solving displacement system...")
        
        try:
            U = np.linalg.solve(K, F)
            max_displacement = np.max(np.abs(U))
            print(f"✅ Solution converged! Max displacement: {max_displacement:.6f}")
            
        except np.linalg.LinAlgError as e:
            return {"error": f"Failed to solve system: {e}"}
        
        # Calculate member forces
        print(f"⚡ Calculating member forces...")
        
        edge_forces = {}
        for i, j in edges:
            if i >= n or j >= n:
                continue
                
            pi = Vector(vertices[i])
            pj = Vector(vertices[j])
            L = (pj - pi).length
            
            if L < 1e-10:
                edge_forces[(i, j)] = 0.0
                continue
            
            # Direction cosines
            dx, dy, dz = (pj - pi) / L
            
            # Axial force calculation
            u_i = U[3*i:3*i+3]
            u_j = U[3*j:3*j+3]
            
            delta_u = u_j - u_i
            axial_force = dx * delta_u[0] + dy * delta_u[1] + dz * delta_u[2]
            
            edge_forces[(i, j)] = axial_force
        
        max_force = max(abs(f) for f in edge_forces.values()) if edge_forces else 1.0
        
        print(f"📊 Maximum force magnitude: {max_force:.1f} N")
        
        # Show top forces
        sorted_forces = sorted(edge_forces.items(), key=lambda x: abs(x[1]), reverse=True)
        print(f"🔝 Top 5 member forces:")
        for k, (edge, force) in enumerate(sorted_forces[:5]):
            force_type = "compression" if force < 0 else "tension"
            print(f"  {k+1}. Edge {edge[0]}-{edge[1]}: {force:.1f} N ({force_type})")
        
        return {
            "forces": edge_forces,
            "max_force": max_force,
            "max_displacement": max_displacement
        }
    
    def clear_analysis(self):
        """Clear previous analysis results"""
        if "Forces" in bpy.data.collections:
            forces_col = bpy.data.collections["Forces"]
            for obj in forces_col.objects:
                bpy.data.objects.remove(obj, do_unlink=True)
            bpy.context.scene.collection.children.unlink(forces_col)
            bpy.data.collections.remove(forces_col)
        
        # Clear materials
        if "ForceMaterial" in bpy.data.materials:
            bpy.data.materials.remove(bpy.data.materials["ForceMaterial"])
        if "HighlightMaterial" in bpy.data.materials:
            bpy.data.materials.remove(bpy.data.materials["HighlightMaterial"])
        
        print("Analysis results cleared!")

    def visualize_results(self, vertices, edges, member_forces, force_vertex_idx, support_indices):
        """Visualize analysis results with professional visualization system"""
        
        # Create visualization collection
        if "Forces" not in bpy.data.collections:
            forces_col = bpy.data.collections.new(name="Forces")
            bpy.context.scene.collection.children.link(forces_col)
        else:
            forces_col = bpy.data.collections["Forces"]
            
        # Clear existing visualization
        for obj in forces_col.objects:
            bpy.data.objects.remove(obj, do_unlink=True)
            
        # Ensure collection is visible in viewport
        forces_col.hide_viewport = False
        forces_col.hide_render = False
        
        # Create visualization objects
        max_force = max(abs(f) for f in member_forces)
        for i, (v1_idx, v2_idx) in enumerate(edges):
            v1 = vertices[v1_idx]
            v2 = vertices[v2_idx]
            force = member_forces[i]
            force_magnitude = abs(force) / max_force if max_force > 0 else 0
            
            # Calculate midpoint
            midpoint = (
                (v1[0] + v2[0]) / 2,
                (v1[1] + v2[1]) / 2,
                (v1[2] + v2[2]) / 2
            )
            
            # Calculate direction vector
            direction = Vector(v2) - Vector(v1)
            length = direction.length
            
            # Create tapered cylinder
            bpy.ops.mesh.primitive_cylinder_add(
                radius=0.01 * force_magnitude,
                depth=length,
                location=midpoint
            )
            
            # Get the created object
            obj = bpy.context.view_layer.objects.active
            
            # Scale and rotate cylinder (using local coordinates)
            obj.scale = (0.01 * force_magnitude, 0.01 * force_magnitude, length / 2)
            
            # Calculate rotation to align with edge
            direction = Vector(v2) - Vector(v1)
            if direction.length > 0:
                quat = direction.to_track_quat('Z', 'Y')
                obj.rotation_euler = quat.to_euler()
            
            # Move to edge center (already done in primitive_cylinder_add)
            
            # Apply material with force magnitude
            if "ForceMaterial" in bpy.data.materials:
                base_mat = bpy.data.materials["ForceMaterial"]
                new_mat = base_mat.copy()
                new_mat.name = f"ForceMaterial_{i}"
                obj.data.materials.append(new_mat)
                
                # Update material node values
                if "Value" in new_mat.node_tree.nodes:
                    value_node = new_mat.node_tree.nodes["Value"]
                    if hasattr(value_node, 'outputs') and len(value_node.outputs) > 0:
                        value_node.outputs[0].default_value = force_magnitude
            
            # Add force label
            if DISPLAY_TEXT:
                # Calculate text position (midpoint of edge)
                text_pos = (
                    (v1[0] + v2[0]) / 2,
                    (v1[1] + v2[1]) / 2,
                    (v1[2] + v2[2]) / 2
                )
                bpy.ops.object.text_add(
                    location=text_pos,
                    rotation=obj.rotation_euler
                )
                text_obj = bpy.context.view_layer.objects.active
                text_obj.scale *= TEXT_SCALE
                text_obj.data.body = f"{abs(force):.1f}N"
                forces_col.objects.link(text_obj)
                
                # Make text face camera
                camera = bpy.context.scene.camera
                if camera:
                    direction = camera.location - text_obj.location
                    rot_quat = direction.to_track_quat('Z', 'Y')
                    text_obj.rotation_euler = rot_quat.to_euler()
                    
                    # Ensure text is in the correct collection
                    if text_obj.name not in forces_col.objects:
                        forces_col.objects.link(text_obj)
                        bpy.context.scene.collection.objects.unlink(text_obj)
        
        # Highlight force application edge
        if force_vertex_idx is not None:
            for edge in edges:
                if force_vertex_idx in edge:
                    v1_idx, v2_idx = edge
                    v1 = vertices[v1_idx]
                    v2 = vertices[v2_idx]
                    midpoint = (
                        (v1[0] + v2[0]) / 2,
                        (v1[1] + v2[1]) / 2,
                        (v1[2] + v2[2]) / 2
                    )
                    
                    bpy.ops.mesh.primitive_cylinder_add(
                        radius=0.02,
                        depth=1,
                        location=midpoint
                    )
                    obj = bpy.context.object
                    direction = Vector(v2) - Vector(v1)
                    obj.scale[2] = direction.length
                    obj.rotation_euler = direction.to_track_quat('Z', 'Y').to_euler()
                    
                    # Create white material for highlight
                    if "ForceMaterial" not in bpy.data.materials:
                        mat = bpy.data.materials.new("ForceMaterial")
                        mat.use_nodes = True
                        nodes = mat.node_tree.nodes
                        links = mat.node_tree.links
                        
                        # Remove default nodes
                        for node in nodes:
                            nodes.remove(node)
                        
                        # Add Principled BSDF
                        bsdf = nodes.new("ShaderNodeBsdfPrincipled")
                        
                        # Create gradient color ramp
                        ramp = nodes.new("ShaderNodeValToRGB")
                        ramp.color_ramp.elements[0].color = (0, 1, 0, 1)  # Green for 0N
                        ramp.color_ramp.elements[1].color = (1, 0, 0, 1)  # Red for compression
                        
                        # Add tension color
                        tension_color = nodes.new("ShaderNodeRGB")
                        tension_color.outputs[0].default_value = (0, 0, 1, 1)  # Blue for tension
                        
                        # Mix shader for tension/compression
                        mix_shader = nodes.new("ShaderNodeMixRGB")
                        mix_shader.blend_type = 'MIX'
                        
                        # Connect nodes
                        links.new(ramp.outputs[0], mix_shader.inputs[1])
                        links.new(tension_color.outputs[0], mix_shader.inputs[2])
                        links.new(mix_shader.outputs[0], bsdf.inputs[0])  # Base Color
                        
                        # Add value node for force magnitude
                        force_value = nodes.new("ShaderNodeValue")
                        links.new(force_value.outputs[0], ramp.inputs[0])
                        links.new(force_value.outputs[0], mix_shader.inputs[0])
                        
                        # Add transparency control
                        transparency = nodes.new("ShaderNodeValue")
                        transparency.outputs[0].default_value = 0.8  # Slightly transparent
                        links.new(transparency.outputs[0], bsdf.inputs[17])  # Alpha input
                        
                        # Add metallic control
                        metallic = nodes.new("ShaderNodeValue")
                        metallic.outputs[0].default_value = 0.2  # Slightly metallic
                        links.new(metallic.outputs[0], bsdf.inputs[4])  # Metallic input
                        
                        # Add roughness control
                        roughness = nodes.new("ShaderNodeValue")
                        roughness.outputs[0].default_value = 0.3  # Medium roughness
                        links.new(roughness.outputs[0], bsdf.inputs[7])  # Roughness input
                        
                        # Add Material Output
                        output = nodes.new("ShaderNodeOutputMaterial")
                        links.new(bsdf.outputs[0], output.inputs[0])
                    obj.active_material = bpy.data.materials["ForceMaterial"]
                    
                    forces_col.objects.link(obj)
                    break
        for support_idx in support_indices:
            if support_idx >= len(vertices):
                continue
            
            pos = Vector(vertices[support_idx])
            bpy.ops.mesh.primitive_cube_add(
                size=BASE_RADIUS * 2,
                location=pos
            )
            obj = bpy.context.active_object
            obj.data.materials.append(materials["support"])
            collection.objects.link(obj)
            bpy.context.scene.collection.objects.unlink(obj)
        
        print(f"✅ Visualization complete!")

    def create_materials(self):
        """Create materials for different force types"""
        materials = {}
        
        # Zero force (green)
        mat = bpy.data.materials.new("Zero_Force")
        mat.use_nodes = True
        mat.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0, 1, 0, 1)
        materials["zero"] = mat
        
        # Compression (red)
        mat = bpy.data.materials.new("Compression")
        mat.use_nodes = True
        mat.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (1, 0, 0, 1)
        materials["compression"] = mat
        
        # Tension (blue)
        mat = bpy.data.materials.new("Tension")
        mat.use_nodes = True
        mat.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0, 0, 1, 1)
        materials["tension"] = mat
        
        # Input force (white)
        mat = bpy.data.materials.new("Input_Force")
        mat.use_nodes = True
        mat.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (1, 1, 1, 1)
        materials["input"] = mat
        
        # Support points (yellow)
        mat = bpy.data.materials.new("Support")
        mat.use_nodes = True
        mat.node_tree.nodes["Principled BSDF"].inputs[0].default_value = (1, 1, 0, 1)
        materials["support"] = mat
        
        return materials
    
    def create_force_label(self, position, text, collection):
        """Create text label for force value"""
        bpy.ops.object.text_add(location=position)
        text_obj = bpy.context.active_object
        text_obj.data.body = text
        text_obj.data.size = TEXT_SCALE
        
        # Offset text to avoid collision
        text_obj.location += Vector((0, 0, BASE_RADIUS * 2))
        
        collection.objects.link(text_obj)
        bpy.context.scene.collection.objects.unlink(text_obj)
    
    def clear_previous_results(self):
        """Clear previous analysis results"""
        if "EV_Force_Analysis" in bpy.data.collections:
            collection = bpy.data.collections["EV_Force_Analysis"]
            
            # Delete all objects in collection
            for obj in collection.objects:
                bpy.data.objects.remove(obj, do_unlink=True)
            
            # Remove collection
            bpy.data.collections.remove(collection)

# ===== USER INTERFACE =====
def run_ev_analysis():
    """Main function to run EV chassis analysis"""
    
    # Get active object
    obj = bpy.context.active_object
    if not obj or obj.type != 'MESH':
        print("❌ Please select a mesh object")
        return
    
    # Check for selected vertex in edit mode
    force_vertex = None
    if obj.mode == 'EDIT':
        bm = bmesh.from_edit_mesh(obj.data)
        selected_verts = [v.index for v in bm.verts if v.select]
        if len(selected_verts) == 1:
            force_vertex = selected_verts[0]
            print(f"🎯 Using selected vertex {force_vertex} for force application")
        elif len(selected_verts) > 1:
            print("⚠️  Multiple vertices selected, using first one")
            force_vertex = selected_verts[0]
    
    # Switch to object mode for analysis
    if obj.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')
    
    # Run analysis
    analyzer = EVChassisAnalyzer()
    results = analyzer.analyze_structure(obj, force_vertex)
    
    if "error" in results:
        print(f"❌ Analysis failed: {results['error']}")
    else:
        print(f"🎉 Analysis successful!")
        print(f"   Max force: {results['max_force']:.1f} N")
        print(f"   Supports: {len(results['supports'])} vertices")
        print(f"   Max displacement: {results['displacement']:.6f}")

# ===== TEST STRUCTURE GENERATOR =====
def create_ev_test_chassis():
    """Create a test EV chassis structure"""
    
    # Clear scene
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    
    # Create chassis geometry (simplified EV platform)
    vertices = [
        # Ground level frame (4 corners)
        (0, 0, 0), (3, 0, 0), (3, 2, 0), (0, 2, 0),
        
        # Battery platform level
        (0.5, 0.3, 0.3), (2.5, 0.3, 0.3), (2.5, 1.7, 0.3), (0.5, 1.7, 0.3),
        
        # Upper structure (cabin area)
        (0.8, 0.5, 1.2), (2.2, 0.5, 1.2), (2.2, 1.5, 1.2), (0.8, 1.5, 1.2),
        
        # Suspension mounting points
        (0.2, 0.2, 0.4), (2.8, 0.2, 0.4), (2.8, 1.8, 0.4), (0.2, 1.8, 0.4)
    ]
    
    edges = [
        # Ground frame
        (0, 1), (1, 2), (2, 3), (3, 0),
        
        # Battery platform
        (4, 5), (5, 6), (6, 7), (7, 4),
        
        # Upper structure
        (8, 9), (9, 10), (10, 11), (11, 8),
        
        # Vertical supports
        (0, 4), (1, 5), (2, 6), (3, 7),  # Ground to battery
        (4, 8), (5, 9), (6, 10), (7, 11),  # Battery to upper
        
        # Cross bracing
        (0, 5), (1, 4), (2, 7), (3, 6),  # Ground to battery diagonal
        (4, 9), (5, 8), (6, 11), (7, 10),  # Battery to upper diagonal
        
        # Suspension mounts
        (0, 12), (1, 13), (2, 14), (3, 15),
        (12, 4), (13, 5), (14, 6), (15, 7)
    ]
    
    # Create mesh
    mesh = bpy.data.meshes.new("EV_Chassis")
    mesh.from_pydata(vertices, edges, [])
    mesh.update()
    
    # Create object
    obj = bpy.data.objects.new("EV_Chassis", mesh)
    bpy.context.collection.objects.link(obj)
    
    # Select the object
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    
    print("🏗️  EV test chassis created!")
    print("   16 vertices, 32 edges")
    print("   Recommended force vertex: 8-11 (upper structure)")
    print("   Ground supports will be auto-detected")

# ===== MAIN EXECUTION =====
if __name__ == "__main__":
    print("=" * 50)
    print("EV CHASSIS FORCE ANALYZER")
    print("=" * 50)
    print()
    print("Welcome to the EV Chassis Force Analyzer!")
    print("This tool performs structural analysis on EV chassis designs.")
    print("""\nUsage:
1. Create or import your chassis mesh in Blender
2. Select a vertex for force application (optional)
3. Run the analysis using the provided commands

Commands:
- create_ev_test_chassis(): Creates a test chassis structure
- run_ev_analysis(): Analyzes the selected mesh
- clear_analysis(): Removes previous analysis results
- exit(): Exits the program
""")
    print()
    
    analyzer = EVChassisAnalyzer()
    
    while True:
        print("\nAvailable actions:")
        print("1. Create test chassis")
        print("2. Run analysis")
        print("3. Clear analysis")
        print("4. Exit")
        
        try:
            choice = input("\nChoose an action (1-4): ")
            
            if choice == '1':
                print("\nCreating test chassis...")
                create_ev_test_chassis()
                print("Test chassis created!")
                print("Recommended force vertex: 8-11 (upper structure)")
                
            elif choice == '2':
                print("\nRunning analysis...")
                
                # Get selected object
                obj = bpy.context.active_object
                if not obj:
                    print("Error: No object selected!")
                    print("Please select a chassis mesh in Blender first.")
                    continue
                
                # Get selected vertex for force application
                force_vertex_idx = None
                if obj.mode == 'EDIT':
                    bm = bmesh.from_edit_mesh(obj.data)
                    selected_verts = [v for v in bm.verts if v.select]
                    if selected_verts:
                        force_vertex_idx = selected_verts[0].index
                        print(f"Force will be applied at vertex {force_vertex_idx}")
                
                # Create force vector
                force_vector = Vector((0, 0, -FORCE_MAGNITUDE))  # Downward force
                
                # Run analysis
                result = analyzer.analyze_structure(obj, force_vertex_idx, force_vector)
                
                if "error" in result:
                    print(f"Error: {result['error']}")
                else:
                    print("Analysis complete!")
                    print(f"Maximum force: {max(abs(f) for f in result['member_forces']):.1f}N")
                    print(f"Maximum displacement: {max(np.linalg.norm(result['displacements'], axis=1)):.3f}m")
                    
            elif choice == '3':
                print("\nClearing analysis results...")
                analyzer.clear_analysis()
                print("Analysis cleared!")
                
            elif choice == '4':
                print("\nExiting program...")
                break
                
            else:
                print("Invalid choice! Please enter a number between 1-4.")
                
        except Exception as e:
            print(f"Error: {str(e)}")
            print("Please try again.")
    
    print("Thank you for using the EV Chassis Force Analyzer!")
    # create_ev_test_chassis()