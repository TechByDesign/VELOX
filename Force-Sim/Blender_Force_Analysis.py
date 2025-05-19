bl_info = {
    "name": "Pipe Force Analysis",
    "author": "Claude",
    "version": (1, 0),
    "blender": (3, 3, 0),
    "location": "View3D > Sidebar > Force Analysis",
    "description": "Analyze forces on pipe structures",
    "warning": "",
    "doc_url": "",
    "category": "3D View",
}

import bpy
import bmesh
import numpy as np
from mathutils import Vector, Matrix
import math
from bpy.props import FloatProperty, FloatVectorProperty, BoolProperty, EnumProperty, StringProperty, PointerProperty

# Constants for steel pipe properties (default values)
DEFAULT_ELASTIC_MODULUS = 200e9  # Steel elastic modulus in Pa
DEFAULT_PIPE_DIAMETER = 0.05     # 5cm diameter
DEFAULT_PIPE_THICKNESS = 0.005   # 5mm thickness
DEFAULT_MAX_STRESS_COLOR = (1.0, 0.0, 0.0, 1.0)  # Red
DEFAULT_MIN_STRESS_COLOR = (0.0, 0.0, 1.0, 1.0)  # Blue

# Helper functions for structural analysis
def calculate_element_stiffness(v1, v2, EA):
    """Calculate element stiffness matrix for a truss element"""
    dx = v2.x - v1.x
    dy = v2.y - v1.y
    dz = v2.z - v1.z
    L = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    if L < 0.0001:  # Prevent division by near-zero length
        return np.zeros((6, 6))
    
    # Direction cosines
    cx = dx / L
    cy = dy / L
    cz = dz / L
    
    # Local stiffness in global coordinates
    k = EA / L
    
    # Transformation matrix
    T = np.array([
        [cx, cy, cz, 0, 0, 0],
        [0, 0, 0, cx, cy, cz]
    ])
    
    # Local stiffness matrix
    k_local = np.array([
        [1, -1],
        [-1, 1]
    ]) * k
    
    # Transform to global coordinates
    k_global = T.T @ k_local @ T
    
    return k_global

def assemble_global_stiffness(mesh, pipe_props):
    """Assemble global stiffness matrix for the entire structure"""
    # Ensure lookup tables are updated
    mesh.verts.ensure_lookup_table()
    mesh.edges.ensure_lookup_table()
    
    # Count vertices for matrix size
    n_vertices = len(mesh.verts)
    n_dof = n_vertices * 3  # 3 DOFs per vertex (x, y, z)
    
    # Initialize global stiffness matrix
    K = np.zeros((n_dof, n_dof))
    
    # Calculate cross-sectional area
    r_outer = pipe_props.pipe_diameter / 2
    r_inner = r_outer - pipe_props.pipe_thickness
    area = math.pi * (r_outer**2 - r_inner**2)
    EA = pipe_props.elastic_modulus * area
    
    # Map from vertex index to DOF indices
    vertex_to_dof = {v.index: (v.index * 3, v.index * 3 + 1, v.index * 3 + 2) for v in mesh.verts}
    
    # Assemble matrix from element contributions
    for edge in mesh.edges:
        v1 = mesh.verts[edge.verts[0].index]
        v2 = mesh.verts[edge.verts[1].index]
        
        # Calculate element stiffness matrix
        k_elem = calculate_element_stiffness(v1.co, v2.co, EA)
        
        # Get DOF indices
        dofs_1 = vertex_to_dof[v1.index]
        dofs_2 = vertex_to_dof[v2.index]
        all_dofs = dofs_1 + dofs_2
        
        # Add to global stiffness matrix
        for i, dof_i in enumerate(all_dofs):
            for j, dof_j in enumerate(all_dofs):
                K[dof_i, dof_j] += k_elem[i, j]
    
    return K

def solve_displacements(K, forces, fixed_vertices):
    """Solve for displacements given forces and boundary conditions"""
    n_dof = K.shape[0]
    
    # Apply boundary conditions (remove fixed DOFs)
    free_dofs = []
    for i in range(n_dof // 3):
        if i not in fixed_vertices:
            free_dofs.extend([i * 3, i * 3 + 1, i * 3 + 2])
    
    # Also include DOFs where a force is applied
    for i, force in enumerate(forces):
        if any(abs(f) > 0.0001 for f in force):
            if i not in fixed_vertices and i * 3 not in free_dofs:
                free_dofs.extend([i * 3, i * 3 + 1, i * 3 + 2])
    
    free_dofs = sorted(list(set(free_dofs)))
    
    # Extract submatrices
    K_ff = K[np.ix_(free_dofs, free_dofs)]
    F_f = np.array([forces[i // 3][i % 3] if i // 3 < len(forces) else 0 for i in free_dofs])
    
    # Solve for free displacements
    try:
        U_f = np.linalg.solve(K_ff, F_f)
    except np.linalg.LinAlgError:
        # If matrix is singular, use pseudo-inverse
        U_f = np.linalg.pinv(K_ff) @ F_f
    
    # Reconstruct full displacement vector
    U = np.zeros(n_dof)
    for i, dof in enumerate(free_dofs):
        U[dof] = U_f[i]
    
    return U

def calculate_element_forces(mesh, displacements, pipe_props):
    """Calculate internal forces in each element"""
    # Ensure lookup tables are updated
    mesh.verts.ensure_lookup_table()
    mesh.edges.ensure_lookup_table()
    
    forces = {}
    stresses = {}
    
    # Calculate cross-sectional area
    r_outer = pipe_props.pipe_diameter / 2
    r_inner = r_outer - pipe_props.pipe_thickness
    area = math.pi * (r_outer**2 - r_inner**2)
    E = pipe_props.elastic_modulus
    
    # Calculate force for each edge
    for edge in mesh.edges:
        v1 = mesh.verts[edge.verts[0].index]
        v2 = mesh.verts[edge.verts[1].index]
        
        # Original length
        dx = v2.co.x - v1.co.x
        dy = v2.co.y - v1.co.y
        dz = v2.co.z - v1.co.z
        L = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if L < 0.0001:  # Skip very short edges
            forces[edge.index] = 0
            stresses[edge.index] = 0
            continue
        
        # Direction cosines
        cx = dx / L
        cy = dy / L
        cz = dz / L
        
        # Displacements at endpoints
        u1 = displacements[v1.index * 3]
        v1_disp = displacements[v1.index * 3 + 1]
        w1 = displacements[v1.index * 3 + 2]
        
        u2 = displacements[v2.index * 3]
        v2_disp = displacements[v2.index * 3 + 1]
        w2 = displacements[v2.index * 3 + 2]
        
        # Axial deformation
        delta_L = cx * (u2 - u1) + cy * (v2_disp - v1_disp) + cz * (w2 - w1)
        
        # Axial force (positive = tension, negative = compression)
        force = E * area * delta_L / L
        forces[edge.index] = force
        
        # Stress
        stress = force / area
        stresses[edge.index] = stress
    
    return forces, stresses

def apply_color_map(obj, stresses):
    """Apply color map to edges based on stress values"""
    mesh = obj.data
    
    if not mesh.vertex_colors:
        mesh.vertex_colors.new(name="Stress")
    
    color_layer = mesh.vertex_colors["Stress"]
    
    # Find min and max stress for normalization
    stress_values = list(stresses.values())
    if not stress_values:
        return
    
    max_stress = max(abs(min(stress_values)), abs(max(stress_values)))
    if max_stress < 0.0001:
        max_stress = 0.0001  # Prevent division by zero
    
    # Create material if it doesn't exist
    mat_name = "StressMap"
    if mat_name not in bpy.data.materials:
        mat = bpy.data.materials.new(name=mat_name)
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        links = mat.node_tree.links
        
        # Clear default nodes
        for node in nodes:
            nodes.remove(node)
        
        # Create nodes
        output = nodes.new(type='ShaderNodeOutputMaterial')
        principled = nodes.new(type='ShaderNodeBsdfPrincipled')
        vertex_color = nodes.new(type='ShaderNodeVertexColor')
        vertex_color.layer_name = "Stress"
        
        # Connect nodes
        links.new(vertex_color.outputs["Color"], principled.inputs["Base Color"])
        links.new(principled.outputs["BSDF"], output.inputs["Surface"])
    else:
        mat = bpy.data.materials[mat_name]
    
    # Assign material to object
    if obj.data.materials:
        obj.data.materials[0] = mat
    else:
        obj.data.materials.append(mat)
    
    # Create BMesh for better access to topology
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bm.edges.ensure_lookup_table()
    
    # Default to blue for low stress
    default_color = (0, 0, 1, 1)
    
    # Set all vertices to default color first
    for i in range(len(color_layer.data)):
        color_layer.data[i].color = default_color
    
    # Create edge to loop map
    edge_to_loops = {}
    for face in bm.faces:
        for loop in face.loops:
            edge = loop.edge
            if edge.index not in edge_to_loops:
                edge_to_loops[edge.index] = []
            # Map to color layer index
            loop_idx = loop.index
            if loop_idx < len(color_layer.data):
                edge_to_loops[edge.index].append(loop_idx)
    
    # Apply color based on stress
    for edge_idx, stress in stresses.items():
        if edge_idx in edge_to_loops:
            # Normalize and calculate color
            normalized_stress = abs(stress) / max_stress
            
            # Blue to red color mapping
            if stress > 0:  # Tension: blue to red
                color = (normalized_stress, 0, 1 - normalized_stress, 1)
            else:  # Compression: blue to green
                color = (0, normalized_stress, 1 - normalized_stress, 1)
            
            # Apply to all loops connected to this edge
            for loop_idx in edge_to_loops[edge_idx]:
                if loop_idx < len(color_layer.data):
                    color_layer.data[loop_idx].color = color
    
    bm.free()

# Property groups
class PipeProperties(bpy.types.PropertyGroup):
    elastic_modulus: FloatProperty(
        name="Elastic Modulus (Pa)",
        description="Young's modulus for the pipe material",
        default=DEFAULT_ELASTIC_MODULUS,
        min=1e6,
        max=1e12
    )
    pipe_diameter: FloatProperty(
        name="Pipe Diameter (m)",
        description="Outer diameter of pipe",
        default=DEFAULT_PIPE_DIAMETER,
        min=0.001,
        max=1.0
    )
    pipe_thickness: FloatProperty(
        name="Pipe Thickness (m)",
        description="Thickness of pipe wall",
        default=DEFAULT_PIPE_THICKNESS,
        min=0.0001,
        max=0.1
    )

class ForceProperties(bpy.types.PropertyGroup):
    force_vector: FloatVectorProperty(
        name="Force Vector (N)",
        description="Force vector to apply",
        default=(0.0, 0.0, -1000.0),
        subtype='XYZ'
    )

# UI Panel
class FORCE_PT_panel(bpy.types.Panel):
    bl_label = "Pipe Force Analysis"
    bl_idname = "FORCE_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Force Analysis"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        pipe_props = scene.pipe_properties
        force_props = scene.force_properties
        
        box = layout.box()
        box.label(text="Pipe Properties:")
        box.prop(pipe_props, "elastic_modulus")
        box.prop(pipe_props, "pipe_diameter")
        box.prop(pipe_props, "pipe_thickness")
        
        layout.separator()
        
        box = layout.box()
        box.label(text="Applied Force:")
        box.prop(force_props, "force_vector")
        
        layout.separator()
        
        layout.label(text="Operations:")
        layout.operator("force.apply_vertices")
        layout.operator("force.clear_forces")
        layout.operator("force.analyze")
        layout.operator("force.clear_analysis")

# Operators
class FORCE_OT_apply_vertices(bpy.types.Operator):
    bl_idname = "force.apply_vertices"
    bl_label = "Apply Force to Selected Vertices"
    bl_description = "Apply the defined force to all selected vertices"
    
    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Please select a mesh object")
            return {'CANCELLED'}
        
        # Check if in edit mode and there are selected vertices
        if context.mode != 'EDIT_MESH':
            self.report({'ERROR'}, "Please enter Edit Mode and select vertices")
            return {'CANCELLED'}
        
        # Get selected vertices
        bm = bmesh.from_edit_mesh(obj.data)
        # Ensure lookup tables are updated
        bm.verts.ensure_lookup_table()
        selected_verts = [v.index for v in bm.verts if v.select]
        
        if not selected_verts:
            self.report({'ERROR'}, "No vertices selected")
            return {'CANCELLED'}
        
        # Store vertex forces in object custom properties
        if "vertex_forces" not in obj:
            obj["vertex_forces"] = {}
        
        force_vec = tuple(context.scene.force_properties.force_vector)
        
        # Add force to selected vertices
        for v_idx in selected_verts:
            obj["vertex_forces"][str(v_idx)] = force_vec
        
        self.report({'INFO'}, f"Applied force to {len(selected_verts)} vertices")
        return {'FINISHED'}

class FORCE_OT_clear_forces(bpy.types.Operator):
    bl_idname = "force.clear_forces"
    bl_label = "Clear All Forces"
    bl_description = "Clear all applied forces"
    
    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Please select a mesh object")
            return {'CANCELLED'}
        
        if "vertex_forces" in obj:
            del obj["vertex_forces"]
        
        self.report({'INFO'}, "Cleared all forces")
        return {'FINISHED'}

class FORCE_OT_analyze(bpy.types.Operator):
    bl_idname = "force.analyze"
    bl_label = "Analyze Forces"
    bl_description = "Calculate and visualize forces in the structure"
    
    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Please select a mesh object")
            return {'CANCELLED'}
        
        if "vertex_forces" not in obj or not obj["vertex_forces"]:
            self.report({'ERROR'}, "No forces applied. Use 'Apply Force to Selected Vertices' first")
            return {'CANCELLED'}
        
        # Get mesh data
        bm = bmesh.new()
        bm.from_mesh(obj.data)
        
        # Ensure lookup tables are updated
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        
        # Convert vertex_forces to array format
        forces = []
        for i in range(len(bm.verts)):
            if str(i) in obj["vertex_forces"]:
                forces.append(list(obj["vertex_forces"][str(i)]))
            else:
                forces.append([0, 0, 0])
        
        # For simplicity, use vertices with forces of (0,0,0) as fixed
        fixed_vertices = [i for i, f in enumerate(forces) if sum(abs(x) for x in f) < 0.0001]
        
        if not fixed_vertices:
            # If no fixed vertices, fix the first vertex
            fixed_vertices = [0]
            self.report({'WARNING'}, "No fixed points found. Fixing the first vertex.")
        
        # Perform structural analysis
        pipe_props = context.scene.pipe_properties
        
        try:
            # Assemble stiffness matrix
            K = assemble_global_stiffness(bm, pipe_props)
            
            # Solve for displacements
            displacements = solve_displacements(K, forces, fixed_vertices)
            
            # Calculate internal forces
            element_forces, element_stresses = calculate_element_forces(bm, displacements, pipe_props)
            
            # Store results on object for later use
            obj["element_forces"] = {str(k): v for k, v in element_forces.items()}
            obj["element_stresses"] = {str(k): v for k, v in element_stresses.items()}
            
            # Apply color map visualization
            apply_color_map(obj, element_stresses)
            
            # Determine max stress for reporting
            max_stress = max([abs(s) for s in element_stresses.values()]) if element_stresses else 0
            
            self.report({'INFO'}, f"Analysis complete. Max stress: {max_stress:.2e} Pa")
            
        except Exception as e:
            self.report({'ERROR'}, f"Analysis failed: {str(e)}")
            return {'CANCELLED'}
        
        finally:
            bm.free()
        
        return {'FINISHED'}

class FORCE_OT_clear_analysis(bpy.types.Operator):
    bl_idname = "force.clear_analysis"
    bl_label = "Clear Analysis Results"
    bl_description = "Clear all analysis results and visualization"
    
    def execute(self, context):
        obj = context.active_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Please select a mesh object")
            return {'CANCELLED'}
        
        # Clear stored results
        for prop in ["element_forces", "element_stresses"]:
            if prop in obj:
                del obj[prop]
        
        # Clear vertex colors
        if obj.data.vertex_colors.get("Stress"):
            obj.data.vertex_colors.remove(obj.data.vertex_colors["Stress"])
        
        self.report({'INFO'}, "Cleared analysis results")
        return {'FINISHED'}

# Registration
classes = (
    PipeProperties,
    ForceProperties,
    FORCE_PT_panel,
    FORCE_OT_apply_vertices,
    FORCE_OT_clear_forces,
    FORCE_OT_analyze,
    FORCE_OT_clear_analysis,
)

def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    
    bpy.types.Scene.pipe_properties = bpy.props.PointerProperty(type=PipeProperties)
    bpy.types.Scene.force_properties = bpy.props.PointerProperty(type=ForceProperties)

def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    
    del bpy.types.Scene.pipe_properties
    del bpy.types.Scene.force_properties

if __name__ == "__main__":
    register()