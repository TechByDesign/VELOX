import bpy
import bmesh
import numpy as np
from mathutils import Vector
from bpy.props import FloatProperty, IntProperty

class PipeForceAnalyzer(bpy.types.Panel):
    bl_label = "Pipe Force Analysis"
    bl_idname = "PT_PipeForceAnalyzer"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Pipe Force Analysis"

    def draw(self, context):
        layout = self.layout
        scene = context.scene

        layout.prop(scene, "force_magnitude")
        layout.prop(scene, "selected_vertex_index")
        layout.operator("object.calculate_pipe_forces")

class CalculatePipeForces(bpy.types.Operator):
    bl_idname = "object.calculate_pipe_forces"
    bl_label = "Calculate Forces"
    bl_description = "Compute forces on each pipe (edge) from applied load"

    def execute(self, context):
        obj = bpy.context.active_object
        if not obj or obj.type != 'MESH':
            self.report({'ERROR'}, "Select a mesh object!")
            return {'CANCELLED'}

        force = context.scene.force_magnitude
        selected_vertex = context.scene.selected_vertex_index

        # Get mesh data
        mesh = obj.data
        bm = bmesh.new()
        bm.from_mesh(mesh)
        bm.verts.ensure_lookup_table()

        if selected_vertex >= len(bm.verts):
            self.report({'ERROR'}, "Invalid vertex index!")
            return {'CANCELLED'}

        # --- Force Calculation (Simplified Truss Analysis) ---
        # Assumptions:
        # 1. All joints are PINNED (no bending).
        # 2. Forces are axial (tension/compression only).
        # 3. Pipes are of uniform stiffness (EA=1 for simplicity).

        # Build adjacency and stiffness matrix (simplified)
        n = len(bm.verts)
        K = np.zeros((n * 3, n * 3))  # 3D stiffness matrix
        force_vector = np.zeros(n * 3)

        # Apply force to selected vertex (in Z-direction)
        force_vector[selected_vertex * 3 + 2] = force  # Applying force in Z-axis

        # --- Very Simplified Stiffness Matrix (Placeholder) ---
        # In reality, you'd compute stiffness for each beam element.
        # Here, we just assign random forces for visualization.
        edge_forces = {}
        for edge in bm.edges:
            v1, v2 = edge.verts
            edge_key = (v1.index, v2.index)
            edge_forces[edge_key] = (v1.index - v2.index) * force * 0.1  # Fake force for demo

        # --- Color Edges Based on Force ---
        max_force = max(abs(f) for f in edge_forces.values()) if edge_forces else 1
        color_map = bpy.data.node_groups.new("ForceColorMap", 'ShaderNodeTree')

        # Create a color ramp
        ramp = color_map.nodes.new('ShaderNodeValToRGB')
        ramp.color_ramp.elements[0].color = (0, 0, 1, 1)  # Blue (compression)
        ramp.color_ramp.elements[1].color = (1, 0, 0, 1)  # Red (tension)

        # Assign colors to edges
        for edge in bm.edges:
            v1, v2 = edge.verts
            edge_key = (v1.index, v2.index)
            force_val = edge_forces.get(edge_key, 0)
            normalized_force = force_val / max_force if max_force != 0 else 0

            # Create a material per edge (for visualization)
            mat = bpy.data.materials.new(f"Force_{edge_key}")
            mat.use_nodes = True
            bsdf = mat.node_tree.nodes.get('Principled BSDF')
            bsdf.inputs[0].default_value = (max(0, normalized_force), 0, max(0, -normalized_force), 1  # R/G/B

            # Assign material to edge (via faces, hack for visibility)
            if not mesh.materials:
                mesh.materials.append(mat)
            else:
                mesh.materials.append(mat)

        bm.to_mesh(mesh)
        bm.free()

        self.report({'INFO'}, f"Force calculation done (simplified demo)!")
        return {'FINISHED'}

def register():
    bpy.utils.register_class(PipeForceAnalyzer)
    bpy.utils.register_class(CalculatePipeForces)
    bpy.types.Scene.force_magnitude = FloatProperty(
        name="Force Magnitude",
        default=100.0,
        description="Force applied at selected vertex (Z-axis)"
    )
    bpy.types.Scene.selected_vertex_index = IntProperty(
        name="Vertex Index",
        default=0,
        description="Index of vertex where force is applied"
    )

def unregister():
    bpy.utils.unregister_class(PipeForceAnalyzer)
    bpy.utils.unregister_class(CalculatePipeForces)
    del bpy.types.Scene.force_magnitude
    del bpy.types.Scene.selected_vertex_index

if __name__ == "__main__":
    register()