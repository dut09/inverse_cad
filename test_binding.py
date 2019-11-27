import os
from common import print_info, print_error, print_ok
from scene import *

############## Sample code begins ################
s = Scene()
s_copy = s.Clone()

# Create the sofa scene.
print_info('Creating the sofa scene...')
s.ExtrudeFromString("extrude 1 0 0 1 1 0 -1 1 0 -1 0.5 0 0 0.5 0 0 0 0 0 0.1 1 +")
s.ExtrudeFromString("extrude -0.8 0.2 1 0.8 0.2 1 0.8 0.8 1 -0.8 0.8 1 0 0 -0.5 -")
s.SaveScene("sofa.nef3")
s.SaveScene("sofa.off")

# Verify deep copy.
print_info('Verifing if deep copy works...')
print_info('Output 1')
s.ListSceneVertices()
print_info('Output 2')
s_copy.ListSceneVertices()
print_info('You should see two *different* outputs above. Press enter to continue...')
input()

# Reconstruct the sofa scene.
print_info('Reconstructing sofa...')

# Use the following two lines to verify SetTargetFromOtherScene works.
s.LoadTarget('sofa.nef3')
s_copy.SetTargetFromOtherScene(s)
# Alternativley, you can do s_copy.LoadTarget('sofa.nef3') instead.

s_copy.ExtrudeFromTargetRef(2, 0, 0, 6, '+')
s_copy.ListSceneFaces()
print_info('You should see some of the lines in the output above become green. Press enter to continue...')
input()

s_copy.ExtrudeFromTargetRef(16, 0, 14, 11, '-')
s_copy.ListSceneFaces()
print_info('You should see all output above become green. Press enter to continue...')
input()

print_info('Exploring vertices/edges/facets...')
print_info('All vertices in the canvas:')
vertices = []
for vid in range(s.GetSceneVertexNumber()):
    v = s.GetSceneVertex(vid)
    name = v.name
    x, y, z = v.x, v.y, v.z
    vertices.append((name, x, y, z))
print(vertices)
print_info('Press enter to continue.')
input()

print_info('All half edges in the target:')
edges = []
for eid in range(s.GetTargetHalfEdgeNumber()):
    e = s.GetTargetHalfEdge(eid)
    name = e.name
    source, target = e.source, e.target
    twin = e.twin
    edges.append((name, source, target, twin))
print(edges)
print_info('Press enter to continue.')
input()

print_info('All half facets in the canvas:')
facets = []
for fid in range(s.GetSceneHalfFacetNumber()):
    f = s.GetSceneHalfFacet(fid)
    name = f.name
    cycles = [fc for fc in f.cycles]
    twin = f.twin
    outward = f.outward
    facets.append((name, cycles, twin, outward))
print(facets)
print_info('Press enter to continue.')
input()

s_copy.SaveScene('sofa_reconstructed.nef3')
s_copy.SaveScene('sofa_reconstructed.off')
print_info('You can use meshlab to check sofa.off and sofa_reconstructed.off. They should be identical.')
input()
############## Sample code ends ################