import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np

from scene import *

# Coloful print.
def print_error(*message):
    print('\033[91m', 'ERROR ', *message, '\033[0m')

def print_ok(*message):
    print('\033[92m', *message, '\033[0m')

def print_info(*message):
    print('\033[93m', *message, '\033[0m')

s = Scene()

# Create the sofa scene.
s.LoadTarget("example/sofa.nef3")
s.ExtrudeFromString("extrude 1 0 0 1 1 0 -1 1 0 -1 0.5 0 0 0.5 0 0 0 0 0 0.1 1 +")
fid = 0 # Pick facet 0 from the target.
use_target = False

f = s.GetTargetHalfFacet(fid) if use_target else s.GetSceneHalfFacet(fid)

def plot_poly(ax, vc, color, label):
    vc = np.asarray(vc)
    vc = np.vstack([vc, vc[0]])
    ax.plot(vc[:, 0], vc[:, 1], vc[:, 2], color=color, label=label)

# Plot this facet.
fig = plt.figure()
ax = p3.Axes3D(fig)
vcs = []
for fc in f.cycles:
    vc = []
    for vid in f.cycles[0]:
        v = s.GetTargetVertex(vid) if use_target else s.GetSceneVertex(vid)
        vc.append([v.x, v.y, v.z])
    plot_poly(ax, vc, 'b', 'facet')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    vcs.append(vc)

# Pick a facet.
# 0 is the facet id and True/False means the facet comes from the target/canvas-so-far.
# Plot the polygon.
for i in range(100):
    ax.cla()
    for vc in vcs:
        plot_poly(ax, vc, 'b', 'facet')
    polygon = s.GenerateRandomPolygon(0, use_target)
    rand_vc = np.asarray([float(v) for v in polygon.strip().split()]).reshape((-1, 3))
    plot_poly(ax, rand_vc, 'r', 'random_{}'.format(i))
    ax.legend()
    plt.pause(1)
plt.show()