"""
Force Analysis Panel for Blender

This script provides a simple UI panel in Blender's 3D View for running force analysis
on selected mesh objects. To use:

1. Open Blender's Scripting workspace
2. Open this script
3. Run the script (Alt+P or click the play button)
4. Find the "Force Analysis" panel in the 3D View's sidebar (N key)
"""

import bpy
from bpy.types import Panel, Operator, PropertyGroup
from bpy.props import (FloatVectorProperty,
                      FloatProperty,
                      IntProperty,
                      EnumProperty,
                      PointerProperty)
from mathutils import Vector

# Import the force analysis package
from force_analysis import (
    SupportType,
    force_components,
    force_magnitude,
    run_analysis
)

# Global variable to store the selected vertex index
selected_vertex_index = None

class ForceAnalysisProperties(PropertyGroup):
    """Properties for the force analysis panel"""
    force_x: FloatProperty(
        name="X",
        description="X component of the force",
        default=force_components['x'],
        update=lambda self, context: self.update_force_components()
    )
    
    force_y: FloatProperty(
        name="Y",
        description="Y component of the force",
        default=force_components['y'],
        update=lambda self, context: self.update_force_components()
    )
    
    force_z: FloatProperty(
        name="Z",
        description="Z component of the force",
        default=force_components['z'],
        update=lambda self, context: self.update_force_components()
    )
    
    magnitude: FloatProperty(
        name="Magnitude",
        description="Force magnitude multiplier",
        default=force_magnitude,
        min=0.1,
        max=10000.0,
        update=lambda self, context: self.update_force_magnitude()
    )
    
    support_type: EnumProperty(
        name="Support Type",
        description="Type of support to add",
        items=[
            ('FIXED', 'Fixed', 'All translations constrained'),
            ('PINNED', 'Pinned', 'X,Y constrained, Z rotation free'),
            ('ROLLER', 'Roller', 'Single axis constraint')
        ],
        default='FIXED'
    )
    
    def update_force_components(self):
        """Update the global force components when UI changes"""
        from force_analysis import force_components
        force_components['x'] = self.force_x
        force_components['y'] = self.force_y
        force_components['z'] = self.force_z
    
    def update_force_magnitude(self):
        """Update the global force magnitude when UI changes"""
        from force_analysis import force_magnitude as fm
        global force_magnitude
        force_magnitude = self.magnitude
        fm = self.magnitude


class FORCE_ANALYSIS_PT_main_panel(Panel):
    """Creates a Panel in the 3D Viewport"""
    bl_label = "Force Analysis"
    bl_idname = "FORCE_ANALYSIS_PT_main_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Force Analysis'
    
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        force_props = scene.force_analysis_props
        
        # Force settings
        box = layout.box()
        box.label(text="Force Settings")
        
        # Force components
        row = box.row()
        row.label(text="Force Direction:")
        
        col = box.column(align=True)
        col.prop(force_props, "force_x", text="X")
        col.prop(force_props, "force_y", text="Y")
        col.prop(force_props, "force_z", text="Z")
        
        # Force magnitude
        box.prop(force_props, "magnitude")
        
        # Support settings
        box = layout.box()
        box.label(text="Support Settings")
        box.prop(force_props, "support_type")
        
        # Buttons
        box = layout.box()
        box.operator("force_analysis.run_analysis", text="Run Analysis")
        box.operator("force_analysis.select_vertex", text="Select Force Vertex")
        box.operator("force_analysis.select_supports", text="Select Support Vertices")
        box.operator("force_analysis.clear_all", text="Clear All")


class FORCE_ANALYSIS_OT_run_analysis(Operator):
    """Run the force analysis on the selected object"""
    bl_idname = "force_analysis.run_analysis"
    bl_label = "Run Force Analysis"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        from force_analysis import run_analysis
        
        if not context.active_object or context.active_object.type != 'MESH':
            self.report({'ERROR'}, "Please select a mesh object")
            return {'CANCELLED'}
        
        try:
            run_analysis()
            self.report({'INFO'}, "Analysis complete")
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Error during analysis: {str(e)}")
            return {'CANCELLED'}


class FORCE_ANALYSIS_OT_select_vertex(Operator):
    """Select a vertex for force application"""
    bl_idname = "force_analysis.select_vertex"
    bl_label = "Select Force Vertex"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        global selected_vertex_index
        
        if context.mode != 'EDIT_MESH':
            self.report({'ERROR'}, "Please enter edit mode first")
            return {'CANCELLED'}
        
        # Import here to avoid circular imports
        from force_analysis.ui import select_vertices
        
        try:
            select_vertices()
            return {'FINISHED'}
        except Exception as e:
            self.report({'ERROR'}, f"Error selecting vertex: {str(e)}")
            return {'CANCELLED'}


class FORCE_ANALYSIS_OT_select_supports(Operator):
    """Select vertices for supports"""
    bl_idname = "force_analysis.select_supports"
    bl_label = "Select Support Vertices"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        if context.mode != 'EDIT_MESH':
            self.report({'ERROR'}, "Please enter edit mode first")
            return {'CANCELLED'}
        
        # In this case, we'll just print a message since select_vertices()
        # already handles support selection when 4 vertices are selected
        self.report({'INFO'}, "Select 4 vertices for supports and click 'Select Support Vertices' again")
        return {'FINISHED'}


class FORCE_ANALYSIS_OT_clear_all(Operator):
    """Clear all force visualizations and selections"""
    bl_idname = "force_analysis.clear_all"
    bl_label = "Clear All"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        global selected_vertex_index
        
        # Clear force visualizations
        from force_analysis.visualization import clear_force_collection
        clear_force_collection()
        
        # Clear selections
        selected_vertex_index = None
        
        # Clear supports
        from force_analysis.settings import clear_supports
        clear_supports()
        
        self.report({'INFO'}, "Cleared all force visualizations and selections")
        return {'FINISHED'}


# Registration
def register():
    bpy.utils.register_class(ForceAnalysisProperties)
    bpy.utils.register_class(FORCE_ANALYSIS_PT_main_panel)
    bpy.utils.register_class(FORCE_ANALYSIS_OT_run_analysis)
    bpy.utils.register_class(FORCE_ANALYSIS_OT_select_vertex)
    bpy.utils.register_class(FORCE_ANALYSIS_OT_select_supports)
    bpy.utils.register_class(FORCE_ANALYSIS_OT_clear_all)
    
    # Add properties to the scene
    bpy.types.Scene.force_analysis_props = PointerProperty(type=ForceAnalysisProperties)


def unregister():
    # Remove properties from the scene
    del bpy.types.Scene.force_analysis_props
    
    # Unregister classes
    bpy.utils.unregister_class(FORCE_ANALYSIS_OT_clear_all)
    bpy.utils.unregister_class(FORCE_ANALYSIS_OT_select_supports)
    bpy.utils.unregister_class(FORCE_ANALYSIS_OT_select_vertex)
    bpy.utils.unregister_class(FORCE_ANALYSIS_OT_run_analysis)
    bpy.utils.unregister_class(FORCE_ANALYSIS_PT_main_panel)
    bpy.utils.unregister_class(ForceAnalysisProperties)


if __name__ == "__main__":
    register()
