from scene import Scene
import numpy as np

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

# create a 3-D figure to illustrate the faces
fig = plt.figure()
ax = p3.Axes3D(fig)

def showExtrusion(vertices, displacement):
    vertices = vertices.split(" ")
    # pack up into an array
    vertices = [ [float(vertices[3*i]),float(vertices[3*i+1]),float(vertices[3*i+2])]
                 for i in range(len(vertices)//3) ]
    vertices.append(vertices[0])

    vertices = np.array(vertices)
    ax.plot(vertices[:,0],vertices[:,1],vertices[:,2])

    # also illustrate what would happen if you extruded the face along this vector
    d = np.array(displacement)
    vertices = vertices + d
    ax.plot(vertices[:,0],vertices[:,1],vertices[:,2])



showExtrusion("1.36496 1.36496 3.10004 1.36496 1.36496 1.03412 1.44766 0.992338 0.846285 1.66791 0.0 3.10004",
              [1.4520674377395917, 0.3222833125243299, -0.0])

showExtrusion("5.0 0.0 0.0 1.66791 0.0 0.0 1.36496 1.36496 0.0 5.0 5.0 0.0",
              [-0.0, -0.0, 3.1000430377011106])
plt.show()

# now try to build the thing using the Python APIs
c = Scene()


c.ExtrudeFromString("extrude 5.0 0.0 0.0 1.66791 0.0 0.0 1.36496 1.36496 0.0 5.0 5.0 0.0 -0.0 -0.0 3.1000430377011106 +")
c.ExtrudeFromString("extrude 1.36496 1.36496 3.10004 1.36496 1.36496 1.03412 1.44766 0.992338 0.846285 1.66791 0.0 3.10004 1.4520674377395917 0.3222833125243299 -0.0 +")

c.SaveScene("/tmp/BUG.off")
