from scene import Scene

k = "extrude 0 0 0 0 5 0 5 5 0 5 0 0 0 0 3 +" # make a box
c = Scene()
c.ExtrudeFromString(k)

c.SaveScene("/tmp/BUG.off")
faceID = 0
f = c.GetSceneHalfFacet(faceID)

print("I have selected a face!")
print("here is everything on that face:")
for e in f.cycles[0]:
    e = c.GetSceneHalfEdge(e)
    source = c.GetSceneVertex(e.source)
    target = c.GetSceneVertex(e.target)
    print("an edge going from",
          (source.x,source.y,source.z),
          (target.x,target.y,target.z))
