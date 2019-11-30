from scene import Scene
import numpy as np

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

import sys
if len(sys.argv) != 2 or sys.argv[1] not in {'0','1'}:
    print("Usage: python test.py [0|1]")
    sys.exit(0)

# create a 3-D figure to illustrate the faces
fig = plt.figure()
ax = p3.Axes3D(fig)

COLORS = ['r','b','g','orange']
def showExtrusion(vertices, displacement):
    global COLORS
    
    vertices = vertices.split(" ")
    # pack up into an array
    vertices = [ [float(vertices[3*i]),float(vertices[3*i+1]),float(vertices[3*i+2])]
                 for i in range(len(vertices)//3) ]
    vertices.append(vertices[0])

    vertices = np.array(vertices)
    ax.plot(vertices[:,0],vertices[:,1],vertices[:,2],
            color=COLORS[0])

    # also illustrate what would happen if you extruded the face along this vector
    d = np.array(list(map(float,displacement.split())))
    vertices = vertices + d
    ax.plot(vertices[:,0],vertices[:,1],vertices[:,2],
            color=COLORS[0])

    COLORS = COLORS[1:]

commands = {'0': ["5 0 0 5 0 2.68747 5 1.74934 3 5 5 3 3.6042687739429273 0.0 0.0",
                  "5 0 0 5 4.47911 2.68747 6.26102 5 3 8.60427 5 3 -0.0 1.669884959000657 -2.7831364504696787"],
              '1': ["0 5 3 5 5 3 5 5 0.895277 3.59781 5 0 0 5 0 0.0 -3.534620585254398 0.0",
              "5 1.46538 3 5 5 3 1.49213 5 3 0 4.00876 3 0 1.46538 3 0.0 0.0 3.8627544028619845"]}[sys.argv[1]]

print("showing extrusions, color ordering is: ",COLORS)
for k in commands:
    print(k)
    polygon = " ".join(k.split()[:-3])
    vector = " ".join(k.split(" ")[-3:])
    showExtrusion(polygon, vector)

plt.show()

# now try to build the thing using the Python APIs
c = Scene()

for k in commands:
    print("about to execute",k)
    c.ExtrudeFromString(f"extrude {k} +")

c.SaveScene("/tmp/BUG.off")
