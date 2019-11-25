import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import sys
import numpy as np

from scene import *

####################### Helper functions. You can skip this section ###################
# Fancy 3d arrow draing.
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)

# Coloful print.
def print_error(*message):
    print('\033[91m', 'ERROR ', *message, '\033[0m')

def print_ok(*message):
    print('\033[92m', *message, '\033[0m')

def print_info(*message):
    print('\033[93m', *message, '\033[0m')
####################### Helper functions. You can skip this section ###################

# Plot.
def plot_outward_facets(scene, title):
    vertices = []
    for vid in range(scene.GetSceneVertexNumber()):
        v = scene.GetSceneVertex(vid)
        x, y, z = v.x, v.y, v.z
        vertices.append((x, y, z))
    vertices = np.asarray(vertices)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    cnt = 0
    for fid in range(scene.GetSceneHalfFacetNumber()):
        f = scene.GetSceneHalfFacet(fid)
        # f.outward is a boolean flag that indicates whether this half facet is facing outward or not.
        # The if statement below is merely for visualization and you can skip it.
        if f.outward:
            # Plot this half facet.
            cnt += 1
            color = np.random.rand(3)
            for fc in f.cycles:
                vcs = vertices[list(fc)]
                vc_center = np.mean(vcs, 0)
                vcs = (vcs - vc_center) * 0.95 + vc_center
                vc_cnt = len(fc)
                for i in range(vc_cnt):
                    j = (i + 1) % vc_cnt
                    vi, vj = vcs[i], vcs[j]
                    # Plot edge vi -> vj.
                    ax.plot([vi[0]], [vi[1]], [vi[2]], 'k')
                    a = Arrow3D([vi[0], vj[0]], [vi[1], vj[1]], [vi[2], vj[2]], mutation_scale=15,
                                 lw=3, arrowstyle="-|>", color=color)
                    ax.add_artist(a)
    assert(cnt * 2 == scene.GetSceneHalfFacetNumber())

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title(title)
    plt.draw()
    plt.show()

# Case 1: sofa.
s = Scene()
print_info('Creating the sofa scene...')
s.ExtrudeFromString("extrude 1 0 0 1 1 0 -1 1 0 -1 0.5 0 0 0.5 0 0 0 0 0 0.1 1 +")
s.ExtrudeFromString("extrude -0.8 0.2 1 0.8 0.2 1 0.8 0.8 1 -0.8 0.8 1 0 0 -0.5 -")
plot_outward_facets(s, 'sofa')

# Case 2: disjoint cubes.
s2 = Scene()
print_info('Creating disjoint cubes...')
s2.ExtrudeFromString("extrude 1 0 0 1 1 0 0 1 0 0 0 0 0 0.1 1 +")
s2.ExtrudeFromString("extrude 3 5 0.5 3 6 0.5 2 6 0.5 2 5 0.5 0 0.1 1 +")
plot_outward_facets(s2, 'disjoint cubes')

# Case 3: cube with a hole.
s3 = Scene()
print_info('Creating cube with a hole...')
s3.ExtrudeFromString("extrude 1 0 0 1 1 0 0 1 0 0 0 0 0 0.1 1 +")
s3.ExtrudeFromString("extrude 0.25 0.25 0 0.25 0.75 0 0.75 0.75 0 0.75 0.25 0 0 0.1 1 -")
plot_outward_facets(s3, 'cube with a hole')