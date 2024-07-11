from bpy.types import (
    Operator,
    OperatorFileListElement,
    Panel
)
from bpy.props import (
    StringProperty,
    CollectionProperty
)
from bpy_extras.io_utils import ImportHelper
import bpy
import pickle
from os.path import splitext, join, basename

bl_info = {
    "name": "PyBulletSimImporter",
    "author": "Huy Ha <hqh2101@columbia.edu>",
    "version": (0, 0, 1),
    "blender": (2, 92, 0),
    "location": "3D View > Toolbox > Animation tab > PyBullet Simulation Importer",
    "description": "Imports PyBullet Simulation Results",
    "warning": "",
    "category": "Animation",
}


class ANIM_OT_import_pybullet_sim(Operator, ImportHelper):
    bl_label = "Import simulation"
    bl_idname = "pybulletsim.import"
    bl_description = "Imports a PyBullet Simulation"
    bl_options = {'REGISTER', 'UNDO'}
    files: CollectionProperty(
        name="Simulation files",
        type=OperatorFileListElement,
    )
    directory: StringProperty(subtype='DIR_PATH')
    filename_ext = ".pkl"
    filter_glob: StringProperty(
        default='*.pkl',
        options={'HIDDEN'})
    skip_frames:  bpy.props.IntProperty(
        name="Skip Frames", default=4, min=1, max=100)
    max_frames:  bpy.props.IntProperty(
        name="Max Frames", default=-1, min=-1, max=10000)

    def execute(self, context):
        for file in self.files:
            filepath = join(self.directory, file.name)
            print(f'Processing {filepath}')
            with open(filepath, 'rb') as pickle_file:
                data = pickle.load(pickle_file)
                collection_name = splitext(basename(filepath))[0]
                collection = bpy.data.collections.new(collection_name)
                bpy.context.scene.collection.children.link(collection)
                context.view_layer.active_layer_collection = \
                    context.view_layer.layer_collection.children[-1]
                
                for obj_key in data:
                    pybullet_obj = data[obj_key]
                    if pybullet_obj['type'] == 'mesh':
                        # Load mesh of each link
                        # assert pybullet_obj['type'] == 'mesh'
                        extension = pybullet_obj['mesh_path'].split(
                            ".")[-1].lower()
                        # Handle different mesh formats
                        if 'obj' in extension:
                            bpy.ops.import_scene.obj(
                                filepath=pybullet_obj['mesh_path'],
                                axis_forward='Y', axis_up='Z')
                        elif 'dae' in extension:
                            bpy.ops.wm.collada_import(
                                filepath=pybullet_obj['mesh_path'])
                        elif 'stl' in extension:
                            bpy.ops.import_mesh.stl(
                                filepath=pybullet_obj['mesh_path'])
                        else:
                            print("Unsupported File Format:{}".format(extension))
                            pass

                        # Delete lights and camera
                        parts = 0
                        final_objs = []
                        for import_obj in context.selected_objects:
                            bpy.ops.object.select_all(action='DESELECT')
                            import_obj.select_set(True)
                            if 'Camera' in import_obj.name \
                                    or 'Light' in import_obj.name\
                                    or 'Lamp' in import_obj.name:
                                bpy.ops.object.delete(use_global=True)
                            else:
                                scale = pybullet_obj['mesh_scale']
                                # if scale is not None and 'dae' not in extension:
                                if scale is not None:
                                    import_obj.scale.x = scale[0]
                                    import_obj.scale.y = scale[1]
                                    import_obj.scale.z = scale[2]
                                final_objs.append(import_obj)
                                parts += 1
                        bpy.ops.object.select_all(action='DESELECT')
                        for obj in final_objs:
                            if obj.type == 'MESH':
                                obj.select_set(True)
                        if len(context.selected_objects):
                            context.view_layer.objects.active =\
                                context.selected_objects[0]
                            # join them
                            bpy.ops.object.join()
                        blender_obj = context.view_layer.objects.active
                        blender_obj.name = obj_key
                        
                    elif pybullet_obj['type'] == 'box':
                        size = pybullet_obj['mesh_scale']  # Assuming mesh_scale contains box dimensions
                        bpy.ops.mesh.primitive_cube_add(
                            size=1.0,
                            enter_editmode=False,
                            align='WORLD',
                            location=(0, 0, 0),
                            scale=(size[0], size[1], size[2])
                        )
                        blender_obj = context.view_layer.objects.active
                        blender_obj.name = obj_key
                        
                    elif pybullet_obj['type'] == 'cylinder':
                        length = pybullet_obj['mesh_scale'][0]  # Assuming mesh_scale contains cylinder dimensions: [length, radius, radius]
                        radius = pybullet_obj['mesh_scale'][1]
                        bpy.ops.mesh.primitive_cylinder_add(
                            radius=radius,
                            depth=length,
                            enter_editmode=False,
                            align='WORLD',
                            location=(0, 0, 0)
                        )
                        blender_obj = context.view_layer.objects.active
                        blender_obj.name = obj_key

                        # 删除灯光和摄像机（在生成圆柱体后）
                        bpy.ops.object.select_all(action='DESELECT')
                        blender_obj.select_set(True)
                        for import_obj in context.selected_objects:
                            if 'Camera' in import_obj.name or 'Light' in import_obj.name or 'Lamp' in import_obj.name:
                                bpy.ops.object.delete(use_global=True)


                    # Keyframe motion of imported object
                    for _, frame_data in enumerate(pybullet_obj['frames']):
                        frame_count = frame_data['frame']
                        if frame_count % self.skip_frames != 0:
                            continue
                        if self.max_frames > 1 and frame_count > self.max_frames:
                            print('Exceed max frame count')
                            break
                        # percentage_done = frame_count / \
                        #     len(pybullet_obj['frames'])
                        # print(f'\r[{percentage_done*100:.01f}% | {obj_key}]',
                        #       '#' * int(60*percentage_done), end='')
                        pos = frame_data['position']
                        orn = frame_data['orientation']
                        context.scene.frame_set(frame_count // self.skip_frames)
                        
                        # Apply position and rotation
                        blender_obj.location.x = pos[0]
                        blender_obj.location.y = pos[1]
                        blender_obj.location.z = pos[2]
                        blender_obj.rotation_mode = 'QUATERNION'
                        blender_obj.rotation_quaternion.x = orn[0]
                        blender_obj.rotation_quaternion.y = orn[1]
                        blender_obj.rotation_quaternion.z = orn[2]
                        blender_obj.rotation_quaternion.w = orn[3]
                        
                        bpy.ops.anim.keyframe_insert_menu(
                            type='Rotation')
                        bpy.ops.anim.keyframe_insert_menu(
                            type='Location')
        return {'FINISHED'}


class VIEW3D_PT_pybullet_recorder(Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Animation"
    bl_label = 'PyBulletSimImporter'

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.operator("pybulletsim.import")


def register():
    bpy.utils.register_class(VIEW3D_PT_pybullet_recorder)
    bpy.utils.register_class(ANIM_OT_import_pybullet_sim)


def unregister():
    bpy.utils.unregister_class(VIEW3D_PT_pybullet_recorder)
    bpy.utils.unregister_class(ANIM_OT_import_pybullet_sim)


if __name__ == "__main__":
    register()
