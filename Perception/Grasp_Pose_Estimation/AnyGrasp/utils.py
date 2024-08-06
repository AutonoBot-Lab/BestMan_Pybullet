from typing import Tuple

import numpy as np
import open3d as o3d
from PIL import ImageDraw

Bbox = Tuple[int, int, int, int]


def sample_points(points, sampling_rate=1):
    """Randomly sample some points from a given set of points at a specified sampling rate.

    Args:
        points (np.ndarray): Array of points to sample from.
        sampling_rate (float, optional): Rate at which to sample points. Defaults to 1.

    Returns:
        Tuple[np.ndarray, np.ndarray]: Sampled points and their indices.
    """
    N = len(points)
    num_samples = int(N * sampling_rate)
    indices = np.random.choice(N, num_samples, replace=False)
    sampled_points = points[indices]
    return sampled_points, indices


def draw_rectangle(image, bbox, width=5):
    """Draw a green rectangle on the given image.

    Args:
        image (ImageDraw): Image on which to draw the rectangle.
        bbox (Bbox): Bounding box coordinates as a tuple (x1, y1, x2, y2).
        width (int, optional): Width of the rectangle outline. Defaults to 5.

    Returns:
        ImageDraw: Image with the rectangle drawn.
    """
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


def visualize_cloud_geometries(
    cloud, geometries, translation=None, rotation=None, visualize=True, save_file=None
):
    """Visualize point cloud with additional geometries.

    Args:
        cloud (o3d.geometry.PointCloud): Point cloud of points.
        geometries (list): List of geometries to visualize with the point cloud.
        translation (np.ndarray, optional): Translation vector. Defaults to None.
        rotation (np.ndarray, optional): Rotation matrix. Defaults to None.
        visualize (bool, optional): Flag to show the visualization window. Defaults to True.
        save_file (str, optional): File name to save the visualization. Defaults to None.
    """
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.2, origin=[0, 0, 0]
    )
    if translation is not None:
        coordinate_frame1 = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.2, origin=[0, 0, 0]
        )
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

        visualizer.capture_screen_image(save_file, do_render=True)
        print(f"[AnyGrasp] Saved screen shot visualization at {save_file}")

    if visualize:
        # visualizer.add_geometry(coordinate_frame)
        visualizer.run()
    else:
        visualizer.destroy_window()
