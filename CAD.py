from scene import Scene

import numpy as np

import scipy.spatial
import random
import math

class Feature():
    def __repr__(self): return str(self)
    def __eq__(self,o): return str(self) == str(o)
    def __ne__(self,o): return not (self == o)
    def __hash__(self): return hash(str(self))

class Vertex(Feature):
    def __init__(self,x,y=None,z=None):
        if y is None:
            x,y,z = x.x,x.y,x.z
        self.p = np.array([float(x),float(y),float(z)])
        # for bizarre reasons we actually need to represent these as strings
        self.x = str(x)
        self.y = str(y)
        self.z = str(z)
    def child(self,o): return False
    def close(self,o,e=0.01):
        return ((self.p - o.p)*(self.p - o.p)).sum()**0.5 < e
    def __hash__(self): return hash(tuple(list(self.p)))
    def __eq__(self,o):
        return isinstance(o,Vertex) and (self.p == o.p).all()
    def __ne__(self,o):
        return not (self == o)
    def __str__(self):
        return f"Vertex({self.x},{self.y},{self.z})"
    def __repr__(self): return str(self)
    def __gt__(self,o):
        return tuple(list(self.p)) > tuple(list(o.p))
    def __lt__(self,o):
        return tuple(list(self.p)) < tuple(list(o.p))
    def __add__(self,o):
        if isinstance(o,Vertex):
            return Vertex(*[A + B for A,B in zip(self.p,o.p) ])
        assert o == 0
        return self

    def __radd__(self,o): return self + o
    def __mul__(self,w):
        assert isinstance(w,(float,int))
        return Vertex(*[w*c for c in self.p ])

class Edge(Feature):
    def __init__(self,u,v):
        if u > v:
            self.e = (u,v)
        else:
            self.e = (v,u)

    def child(self,o):
        return isinstance(o,Vertex) and o in self.e
    
    def __hash__(self): return hash(self.e)
    def __eq__(self,o):
        return isinstance(o,Edge) and self.e == o.e
    def __ne__(self,o):
        return not (self == o)
    def __str__(self):
        return f"Edge{self.e}"
    def __repr__(self): return str(self)

    def numpy(self):
        return self.e[0].p - self.e[1].p

class Face(Feature):
    def __init__(self, cycles):
        assert len(cycles) == 1
        assert all(isinstance(v,Vertex)
                   for c in cycles
                   for v in c )
        assert all( isinstance(c,frozenset) for c in cycles )
        self.cycles = tuple(cycles)
    def child(self,o):
        if isinstance(o,Edge): return any( o.child(v)
                                           for c in self.cycles
                                           for v in c )
        if isinstance(o,Vertex):
            return any( e == o
                        for c in self.cycles
                        for e in c )
        return False
    def __str__(self):
        return f"Face{self.cycles}"
    def __hash__(self):
        return hash(self.cycles)
    def __eq__(self,o):
        return isinstance(o,Face) and self.cycles == o.cycles

class CAD():
    """Purely functional wrapper over Scene"""
    def __init__(self, child=None):
        self.child = child or Scene()

    @property
    def empty(self):
        return self.child.GetSceneVertexNumber() == 0

    def extrude(self, face, direction, union=True):
        s = self.child.Clone()
        command = []
        for cs in face + [direction]:
            for c in cs:
                command.append(str(c))
        if union:
            command.append('+')
        else:
            command.append('-')
        command = " ".join(["extrude"] + command)
        print("about to execute command")
        print(command)
        s.ExtrudeFromString(command)
        print("successfully executed command")
        return CAD(s)

    def export(self, fn):
        self.child.SaveScene(fn)

    def findClose(self, v):
        candidates = [vp for vp in self.getVertices()
                      if v.close(vp)]
        if candidates: return candidates[0]
        return None


    def getVertices(self):
        N = self.child.GetSceneVertexNumber()
        return [Vertex(v.x,v.y,v.z)
                for n in range(N)
                for v in [self.child.GetSceneVertex(n)] ]

    def getEdges(self):
        return list({Edge(Vertex(self.child.GetSceneVertex(e.source)),
                          Vertex(self.child.GetSceneVertex(e.target)))
                     for n in range(self.child.GetSceneHalfEdgeNumber())
                     for e in [self.child.GetSceneHalfEdge(n)] })

    def getFaces(self):
        fs = set()
        for n in range(self.child.GetSceneHalfFacetNumber()):
            f = self.child.GetSceneHalfFacet(n)
            cycles = []
            for cycle in f.cycles:
                thisCycle = []
                for v in cycle:                
                    v = Vertex(self.child.GetSceneVertex(v))
                    thisCycle.append(v)
                    
                cycles.append(frozenset(thisCycle))
            fs.add(Face(cycles))
        return fs
    
    def getFeatures(self):
        assert False, "not yet implemented correctly"
        # mapping from vertex to index and vice versa
        v2n = {}
        n2v = []
        for n in range(self.child.GetSceneVertexNumber()):
            v = self.child.GetSceneVertex(n)
            v = Vertex(v.x,v.y,v.z)
            v2n[v] = n
            n2v.append(v)


        e2n = {}
        n2e = []
        for n in range(self.child.GetSceneHalfEdgeNumber()):
            e = self.child.GetSceneHalfEdge(n)
            source = n2v[e.source]
            target = n2v[e.target]

            e2n[e] = n
            n2e.append(e)

    def setTarget(self, c):
        self.child.SetTargetFromOtherScene(c.child)
        
            

class Extrusion():
    def __init__(self, displacement, union, vertices):
        self.vertices, self.displacement, self.union = vertices, displacement, union

    def __str__(self):
        return f"Extrusion({self.vertices}, D={self.displacement}, U={self.union})"

    def __repr__(self):
        return str(self)

    def compile(self, target):
        from state import NextVertex, Extrude
        vs = list(self.vertices)
        if not self.union:
            print("compiling a subtraction!")
            print("here the vertices inside of the subtraction command")
            print(vs)
            print("here are the vertices inside of the target")
            print(target.getVertices())
        # want to pick a random rotation of the vertices
        if random.random() > 0.5:
            vs.reverse()
        N = random.choice(range(len(vs)))
        vs = vs[N:] + vs[:N]
        compilation = [ NextVertex(v) for v in vs ]
        base = vs[-1]
        connection = Vertex(base.p[0] + self.displacement[0],
                            base.p[1] + self.displacement[1],
                            base.p[2] + self.displacement[2])
        connection = target.findClose(connection)
        if connection is None: return None
        compilation.append(Extrude(connection, self.union))
        return compilation

    def execute(self,c):
        return c.extrude([(v.x,v.y,v.z) for v in self.vertices],
                         self.displacement,
                         self.union)

    @staticmethod
    def sample(c, union=True):
        if c.empty:
            # initial box
            c = c.extrude([(0,0,0),
                           (0,5,0),
                           (5,5,0),
                           (5,0,0)],
                          (0,0,3))
        faceID = random.choice(range(c.child.GetSceneHalfFacetNumber()))
        f = c.child.GetSceneHalfFacet(faceID)
        polygon = c.child.GenerateRandomPolygon(faceID, 0.5, 0.5, False)
        print("created random polygon",polygon)
        vertices = polygon.strip().split()
        vertices = [Vertex(vertices[3*i],vertices[3*i + 1],vertices[3*i + 2])
                    for i in range(len(vertices)//3)]
        
        normal = np.asarray([float(v) for v in f.normal.split()])
        if not f.outward: normal = -normal
        L = ((normal*normal).sum()**0.5)
        if L < 0.001:
            print("for some reason the normal is nothing")
            return Extrusion.sample(c, union=union)
        normal = normal/L

        magnitude = random.random()*3 + 1
        direction = list(normal*magnitude)
        command = Extrusion(direction, union, vertices)
        return command
            

class Program():
    def __init__(self, commands):
        self.commands = commands

    def execute(self,c):
        for k in self.commands: c = k.execute(c)
        return c

    def compile(self, target):
        actions = []
        for k in self.commands:
            compilation = k.compile(target)
            if compilation is None: return None
            actions.extend(compilation)
        return actions

    def __str__(self):
        return f"Program{self.commands}"

    def __repr__(self):
        return str(self)

    @staticmethod
    def sample(s):
        commands = []
        for _ in range(2):#random.choice(range(1,5))):
            k = Extrusion.sample(s,
                                 union=True)#random.random() > 0.3)
            s = k.execute(s)
            commands.append(k)
        return Program(commands)
        return Program([Extrusion((0, 0.1, 1), True,
                                  [Vertex(1, 0, 0),
                                   Vertex(1, 1, 0),
                                   Vertex(-1, 1, 0),
                                   Vertex(-1, 0.5, 0),
                                   Vertex(0, 0.5, 0),
                                   Vertex(0, 0, 0)]),
                        Extrusion((0, 0, -0.5),False,
                        [
                            Vertex(-0.8, 0.6, 1),
                            Vertex(0.0, 0.6, 1),
                            Vertex(0.0, 0.2, 1),
                            Vertex(0.8, 0.2, 1),
                            Vertex(0.8, 0.8, 1),
                            Vertex(-0.8, 0.8, 1)
                        ])
        ])
                            
        angle = 0
        N = random.choice(range(3,8))
        r = random.random() + 1
        face = [Vertex(r,r,0)]
        for _ in range(N - 1):            
            d_angle = 2*math.pi/N#6.28/N

            x,y,_ = face[-1].p
            x = x + r*math.cos(angle + d_angle)
            y = y + r*math.sin(angle + d_angle)

            angle += d_angle

            face.append(Vertex(x,y,0))
        command1 = Extrusion((0,0,1),True,face)

        subtraction = [Vertex(v.p[0] + 1.,v.p[1] + 1.,v.p[2] + 1.)
                       for v in face[:3]]
        command2 = Extrusion((0,0,-0.5),False,subtraction)

        return Program([command1, command2])
