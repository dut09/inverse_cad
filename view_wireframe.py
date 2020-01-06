import numpy as np
import matplotlib.pyplot as plt
from scene import *
from common import *

def view_matrix(camera_location, camera_lookat, camera_up):
    camera_z = camera_location - camera_lookat
    camera_z /= np.linalg.norm(camera_z)
    camera_x = np.cross(camera_up, camera_z)
    camera_x /= np.linalg.norm(camera_x)
    camera_y = np.cross(camera_z, camera_x)
    Rt = np.array([camera_x, camera_y, camera_z])
    view_matrix = np.eye(4)
    view_matrix[:3, :3] = Rt
    view_matrix[:3, -1] = -Rt @ camera_location
    return view_matrix

def projection_matrix(fov, aspect_ratio, z_min, z_max):
    tan_half_fov = np.tan(np.deg2rad(fov) / 2)
    proj_matrix = np.zeros((4, 4))
    proj_matrix[0, 0] = 1 / (aspect_ratio * tan_half_fov)
    proj_matrix[1, 1] = 1 / tan_half_fov
    proj_matrix[2, 2] = -(z_max + z_min) / (z_max - z_min)
    proj_matrix[3, 2] = -1
    proj_matrix[2, 3] = -(2 * z_min * z_max) / (z_max - z_min)
    return proj_matrix

# Each row is a point.
def rasterize(pts_in_world, V, P):
    if pts_in_world.size == 0:
        return np.zeros((0, 2))
    new_col = np.ones((pts_in_world.shape[0], 1))
    pts = np.append(pts_in_world, new_col, 1)
    pts = pts @ V.T @ P.T
    pts[:, :3] /= pts[:, -1][:, None]
    # Remap x and y from (-1, 1) to (0, 1)
    pts[:, :2] = pts[:, :2] * 0.5 + 0.5
    return pts[:, :2]

# Draw wireframes.
def draw_scene(scene, V, P, fn, canvas_or_target="canvas", title="", show=False, colors=None):
    # Loop over all vertices and rasterize:
    vertices = []
    if canvas_or_target == 'canvas':
        for vid in range(s.GetSceneVertexNumber()):
            v = s.GetSceneVertex(vid)
            name = v.name
            x, y, z = str2float(v.x), str2float(v.y), str2float(v.z)
            vertices.append((x, y, z))
    elif canvas_or_target == 'target':
        for vid in range(s.GetTargetVertexNumber()):
            v = s.GetTargetVertex(vid)
            name = v.name
            x, y, z = str2float(v.x), str2float(v.y), str2float(v.z)
            vertices.append((x, y, z))
    else:
        raise NotImplementedError    
    vertices = np.asarray(vertices)
    vertices = rasterize(vertices, V, P)
    # Now vertices is an N x 2 matrix and each row represents the projected coordinates of one vertex.

    # Loop over all edges and draw them.
    fig, ax = plt.subplots(1, 1)
    if canvas_or_target == 'canvas':
        for eid in range(s.GetSceneHalfEdgeNumber()):
            e = s.GetSceneHalfEdge(eid)
            source, target = e.source, e.target
            v_source_and_target = vertices[[source, target]]
            ax.plot(v_source_and_target[:, 0], v_source_and_target[:, 1], 'y')
    elif canvas_or_target == 'target':
        for eid in range(s.GetTargetHalfEdgeNumber()):
            e = s.GetTargetHalfEdge(eid)
            source, target = e.source, e.target
            v_source_and_target = vertices[[source, target]]
            ax.plot(v_source_and_target[:, 0], v_source_and_target[:, 1], 'y')
    else:
        raise NotImplementedError
    # Draw vertices -- you can assign different colors here. I generated the color randomly.
    for v in vertices:
        ax.plot(v[0], v[1], marker='o', MarkerSize=5, color=np.random.rand(3))
    plt.title(title)
    plt.savefig(fn)
    if show:
        plt.show()
    
if __name__ == '__main__':
    s = Scene()
    # Create the sofa scene.
    s.LoadTarget("example/sofa.nef3")

    # Modify your camera parameters here to generate +x, -x, +y, -y, +z, and -z images as you wish.
    camera_location = np.array([0.0, 1.2, -2.0])
    camera_lookat = np.array([0.0, 1.0, 0.0])
    camera_up = np.array([0.0, 1.0, 0.0])
    fov = 60    # This is in degrees.
    aspect_ratio = 1.0
    z_min = 0.0
    z_max = 2.0

    V = view_matrix(camera_location, camera_lookat, camera_up)
    P = projection_matrix(fov, aspect_ratio, z_min, z_max)

    draw_scene(s, V, P, "/tmp/wireframe.png", 'target', show=True)
