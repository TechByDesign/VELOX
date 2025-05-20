import bpy
import os
import sys
from importlib import import_module

# === WORKAROUND: Fix missing extensions directories ===
ext_path = bpy.utils.user_resource('SCRIPTS', path="extensions")
os.makedirs(os.path.join(ext_path, "blender_org"), exist_ok=True)
os.makedirs(os.path.join(ext_path, "user_default"), exist_ok=True)
# =====================================================

# === Import your core analysis script ===
try:
    # Ensure the directory containing your core script is in Python path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if script_dir not in sys.path:
        sys.path.append(script_dir)
    
    # Import your core analysis module
    core = import_module("force_analysis_core")  # Assuming your core script is named this
except ImportError as e:
    print(f"ERROR: Could not import core analysis module: {e}")
    # Create dummy core module to prevent crashes
    class DummyCore:
        force_magnitude = 10000.0
        selected_vertex_index = 0
        base_radius = 0.05
        text_scale = 0.08
        @staticmethod
        def run_analysis(): print("Core module not loaded!")
        @staticmethod
        def clear_force_collection(): print("Core module not loaded!")
    core = DummyCore()
# =======================================

bl_info = {
    "name": "Pipe Force Analysis",
    "author": "Your Name",
    "version": (1, 0),
    "blender": (4, 2, 0),
    "location": "View3D > Sidebar > Analysis",
    "description": "Force visualization for pipe structures",
    "category": "Analysis",
}

class ForceAnalysisPanel(bpy.types.Panel):
    """Creates a Panel in the 3D View sidebar"""
    bl_label = "Pipe Force Analysis"
    bl_idname = "VIEW3D_PT_pipe_force_analysis"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Analysis'

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        # Input controls
        layout.prop(scene, "fa_force_magnitude")
        layout.prop(scene, "fa_selected_vertex_index")
        
        # Visualization settings
        box = layout.box()
        box.label(text="Visual Settings")
        box.prop(scene, "fa_base_radius")
        box.prop(scene, "display_text", text="Display Text")
        box.prop(scene, "fa_text_scale")
        
        # Action buttons
        row = layout.row()
        row.operator("object.fa_run_analysis")
        row.operator("object.fa_clear_visuals")

class RunForceAnalysis(bpy.types.Operator):
    bl_label = "Analyze Forces"
    bl_idname = "object.fa_run_analysis"
    
    def execute(self, context):
        # Pass UI settings to your original code's global variables
        core.force_magnitude = context.scene.fa_force_magnitude
        core.selected_vertex_index = context.scene.fa_selected_vertex_index
        core.base_radius = context.scene.fa_base_radius
        core.text_scale = context.scene.fa_text_scale
        
        # Call your original function
        core.run_analysis()
        return {'FINISHED'}

class ClearForceVisuals(bpy.types.Operator):
    bl_label = "Clear Visuals"
    bl_idname = "object.fa_clear_visuals"
    
    def execute(self, context):
        core.clear_force_collection()
        return {'FINISHED'}

def register():
    # Register UI properties
    bpy.types.Scene.fa_force_magnitude = bpy.props.FloatProperty(
        name="Force Magnitude",
        default=10000.0,
        min=0.0
    )
    bpy.types.Scene.fa_selected_vertex_index = bpy.props.IntProperty(
        name="Vertex Index",
        default=0,
        min=0
    )
    bpy.types.Scene.fa_display_text = bpy.props.BoolProperty(
        name="Display Text",
        default=True,
        description="Show/hide force value labels"
    )
    bpy.types.Scene.fa_base_radius = bpy.props.FloatProperty(
        name="Base radius",
        default=0.05,
        min=0.001,
    )
    
    bpy.types.Scene.fa_text_scale = bpy.props.FloatProperty(
        name="Text Scale",
        default=0.08,
        min=0.01
    )
    
    # Register UI classes
    bpy.utils.register_class(ForceAnalysisPanel)
    bpy.utils.register_class(RunForceAnalysis)
    bpy.utils.register_class(ClearForceVisuals)

def unregister():
    # Cleanup
    del bpy.types.Scene.fa_force_magnitude
    del bpy.types.Scene.fa_selected_vertex_index
    del bpy.types.Scene.fa_base_radius
    del bpy.types.Scene.fa_text_scale
    
    bpy.utils.unregister_class(ForceAnalysisPanel)
    bpy.utils.unregister_class(RunForceAnalysis)
    bpy.utils.unregister_class(ClearForceVisuals)

if __name__ == "__main__":
    register()