import numpy as np
import open3d as o3d
from PIL import ImageDraw

def sample_points(points, sampling_rate=1):
    
    N = len(points)
    num_samples = int(N*sampling_rate)
    indices = np.random.choice(N, num_samples, replace=False)
    sampled_points = points[indices]
    return sampled_points, indices


def draw_rectangle(image, bbox, width=5):
    
    img_drw = ImageDraw.Draw(image)
    x1, y1, x2, y2 = bbox[0], bbox[1], bbox[2], bbox[3]

    width_increase = 5
    for _ in range(width_increase):
        img_drw.rectangle([(x1, y1), (x2, y2)], outline="green")

        x1 -= 1
        y1 -= 1
        x2 += 1
        y2 += 1
    
    return img_drw


def visualize_cloud_geometries(cloud, geometries, translation = None, rotation = None, visualize = True, save_file = None):
    """
        cloud       : Point cloud of points
        grippers    : list of grippers of form graspnetAPI grasps
        visualise   : To show windows
        save_file   : Visualisation file name
    """

    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
    if translation is not None:
        coordinate_frame1 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
        translation[2] = -translation[2]
        coordinate_frame1.translate(translation)
        coordinate_frame1.rotate(rotation)

    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(visible=visualize)
    for geometry in geometries:
        visualizer.add_geometry(geometry)
    visualizer.add_geometry(cloud)
    if translation is not None:
        visualizer.add_geometry(coordinate_frame1)
    visualizer.poll_events()
    visualizer.update_renderer()

    if save_file is not None:
        ## Controlling the zoom
        view_control = visualizer.get_view_control()
        zoom_scale_factor = 1.4  
        view_control.scale(zoom_scale_factor)

        visualizer.capture_screen_image(save_file, do_render = True)
        print(f"Saved screen shot visualization at {save_file}")

    if visualize:
        visualizer.add_geometry(coordinate_frame)
        visualizer.run()
    else:
        visualizer.destroy_window()    