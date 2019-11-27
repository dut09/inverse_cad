import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import sys
import os
import numpy as np
import shutil
try:
    export_gif_ok = True
    import export_gif
except:
    export_gif_ok = False

from scene import *

s = Scene()

# Create the sofa scene.
s.LoadTarget("example/sofa.nef3")
s.ExtrudeFromString("extrude 1 0 0 1 1 0 -1 1 0 -1 0.5 0 0 0.5 0 0 0 0 0 0.1 1 +")
############## Beginning of parameters ######################
fid = 0
use_target = False
arguments = sys.argv
arguments = dict(zip(range(len(arguments)), arguments))
skip_prob = float(arguments.get(1,0.5))         # Larger number -> simpler polygon.
collapse_prob = float(arguments.get(2,0.5))     # Larger number -> simpler polygon.
############## End of parameters ############################

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
max_iter = 20
folder = 'rand_poly_gif'
if os.path.isdir(folder):
    shutil.rmtree(folder)
os.makedirs(folder, exist_ok=True)
for i in range(max_iter):
    ax.cla()
    for vc in vcs:
        plot_poly(ax, vc, 'b', 'facet')
    polygon = s.GenerateRandomPolygon(0, skip_prob, collapse_prob, use_target)
    rand_vc = np.asarray([float(v) for v in polygon.strip().split()]).reshape((-1, 3))
    plot_poly(ax, rand_vc, 'r', 'random_{}'.format(i))
    ax.legend()
    plt.pause(1)
    plt.savefig(os.path.join(folder, '{:04d}.png'.format(i)))
plt.show()
if export_gif_ok:
    export_gif.run(folder, 'rand_poly.gif', 5)
shutil.rmtree(folder)