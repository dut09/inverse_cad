from CAD import *

class Death(Exception): pass

ALL_TAGS = [
    "EXTRUDE_LATEST",
    "EXTRUDE_FACE",
    "EXTRUDE_START"
    ]

class State():
    def __init__(self, canvas, target, tags=None, extrude_vertices=[]):
        self.canvas, self.target = canvas, target

        self.canvas_features = set()
        self.target_features = set()
        self.vertices = []

        for v in target.getVertices():
            self.registerVertex(v,True,False)
        for v in canvas.getVertices():
            self.registerVertex(v,False,True)

        self.edges = []
        for e in target.getEdges():
            self.registerEdge(e,True,False)
        for e in canvas.getEdges():
            self.registerEdge(e,False,True)

        self.faces = []
        for f in target.getFaces():
            self.registerFace(f,True,False)
        for f in canvas.getFaces():
            self.registerFace(f,False,True)

        self.tags = tags or {feature: set()
                             for feature in self.canvas_features|self.target_features}
        self.extrude_vertices = extrude_vertices

    def visualize(self,fn,title=None):
        # Modify your camera parameters here to generate +x, -x, +y, -y, +z, and -z images as you wish.
        from view_wireframe import view_matrix,projection_matrix,rasterize
        import matplotlib.pyplot as plot
        plt = plot # lol
        fig, ax = plt.subplots(1, 1)
        camera_location = np.array([0.5, 1.2, 2.0])
        camera_lookat = np.array([0.0, 1.0, 0])
        #camera_lookat = camera_lookat/((camera_lookat*camera_lookat).sum()**0.5)
        camera_up = np.array([0.0, 1.0, 0.0])
        fov = 60    # This is in degrees.
        aspect_ratio = 1.0
        z_min = min([0.0] + [v.p[-1] for v in self.vertices ])
        z_max = max([2.0] + [v.p[-1] for v in self.vertices ])
        

        V = view_matrix(camera_location, camera_lookat, camera_up)
        P = projection_matrix(fov, aspect_ratio, z_min, z_max)

        legend_entries = {}

        # draw all of the edges
        for e in self.edges:
            u = e.e[0].p
            v = e.e[1].p
            uv = rasterize(np.array([u,v]),V,P)
            u = uv[0]
            v = uv[1]
            if e in self.canvas_features and e in self.target_features:
                color = 'k'
            elif e in self.canvas_features:
                color = 'r'
            elif e in self.target_features:
                color = 'b'
            else:
                assert False
            
            ax.plot(uv[:,0], uv[:, 1], color)

        for v in self.vertices:
            p = rasterize(np.array([v.p]),V,P)[0]
            if v in self.canvas_features and v in self.target_features:
                color = 'k'
            elif v in self.canvas_features:
                color = 'r'
            elif v in self.target_features:
                color = 'b'
            else:
                assert False
            if color not in legend_entries:
                label = {'k': 'explained',
                         'r': 'extraneous',
                         'b': 'to explain'}[color]
                legend_entries[color] = label
            else: label = None
            ax.plot(p[0], p[1], marker='o', MarkerSize=4, color=color, label=label)

            for tag in self.tags[v]:
                i = ALL_TAGS.index(tag)
                c = ['c','y','m'][i]
                r = 0.01
                angle = i*6.14/len(ALL_TAGS)
                if c not in legend_entries:
                    label = {'c': 'extrude',
                             'y': 'face'}[c]
                    legend_entries[c] = label
                else: label = None
                ax.plot(r*math.cos(angle) + p[0], r*math.sin(angle) + p[1], marker='o', MarkerSize=7, color=c, label=label)
                

        plot.legend(bbox_to_anchor=(0,-0.04), loc='upper left', ncol=len(legend_entries))

            

        if title is not None:
            plt.title(title)
        plt.savefig(fn)

    @staticmethod
    def exportTrace(states, actions, prefix):
        for n,s,a in zip(range(9999), states, actions + ["DONE"]):
            s.visualize(f"{prefix}_{n}.png",title=str(a))

    def iou(self):
        return len(self.canvas_features&self.target_features)/len(self.canvas_features|self.target_features)

    def addTag(self, feature, tag):
        feature_ = self.findClose(feature)
        if feature_ is None:
            print("couldn't add tag to",feature,"because there is nothing close")
            print("here are the available vertices")
            for v in self.vertices:
                print(v)
            assert 0
        feature = feature_
        tags = {k: (v if k != feature else v|{tag})
            for k,v in self.tags.items() }
        return State(self.canvas,
                     self.target,
                     tags,
                     self.extrude_vertices)

    def showTags(self):
        for k,v in self.tags.items():
            if len(v) == 0: continue
            print(k,"\t",v)

    def removeTag(self, feature, tag):
        feature_ = self.findClose(feature)
        if feature_ is None: assert 0
        feature = feature_
        tags = {k: (v if k != feature else v-{tag})
            for k,v in self.tags.items() }
        return State(self.canvas,
                     self.target,
                     tags,
                     self.extrude_vertices)

    def getTagged(self, tag):
        return [f for f,ts in self.tags.items() if tag in ts]
            

    def registerVertex(self,v, spec, canvas):
        vp = self.findClose(v)
        if vp is not None: v = vp
        
        if spec: self.target_features.add(v)
        if canvas: self.canvas_features.add(v)
        if vp is None:
            self.vertices.append(v)
        return v

    def findClose(self,v):
        if isinstance(v,Vertex):
            candidates = [vp for vp in self.vertices if v.close(vp) ]
            if candidates: return candidates[0]
            return None
        elif isinstance(v,Edge):
            u,v = self.findClose(v.e[0]),self.findClose(v.e[1])
            if u is None or v is None: return None            
            v = Edge(u,v)
            if v not in self.tags: return None
            return v
        elif isinstance(v,Face):
            cycles = []
            for cycle in v.cycles:
                cycle = frozenset(self.findClose(e) for e in cycle)
                if any( e is None for e in cycle ): return None
                cycles.append(cycle)
            f = Face(cycles)
            if f not in self.tags: return None
            return f
        else:
            assert False, "attempt to find something close which was not a feature"

    def registerEdge(self,e, spec, canvas):
        e = Edge(self.findClose(e.e[0]),self.findClose(e.e[1]))
        if spec: self.target_features.add(e)
        if canvas: self.canvas_features.add(e)
        if e not in self.edges:
            self.edges.append(e)
        return e

    def registerFace(self,f, spec, canvas):
        f = Face([ frozenset(self.registerVertex(v, spec, canvas) for v in cycle)
                   for cycle in f.cycles ])
        if spec: self.target_features.add(f)
        if canvas: self.canvas_features.add(f)
        if f not in self.faces:
            self.faces.append(f)

class Action():
    def __repr__(self):
        return str(self)

    def __hash__(self):
        return hash(str(self))

    def __eq__(self,o):
        return str(self) == str(o)

    def __ne__(self,o):
        return not (self == o)

class NextVertex(Action):
    def __init__(self, v):
        self.v = v

    def code(self): return (0,self.v)

    def __str__(self):
        return f"NextVertex({self.v})"

    def __call__(self, s):
        old_latest = s.getTagged("EXTRUDE_LATEST")
        s = s.addTag(self.v, "EXTRUDE_LATEST")
        
        for latest in old_latest:
            s = s.addTag(latest,"EXTRUDE_FACE")
            s = s.removeTag(latest,"EXTRUDE_LATEST")
        
        s.extrude_vertices = s.extrude_vertices + [self.v]
        return s

class Extrude(Action):
    def __init__(self, v, union):
        self.v = v
        self.union = union

    def __str__(self):
        return f"Extrude({self.v}, union={self.union})"

    def code(self): return (1 + int(self.union), self.v)

    def __call__(self, s):
        el = s.getTagged("EXTRUDE_LATEST")
        if len(el) != 1: raise Death()

        el = el[0]
        displacement = (self.v.p[0] - el.p[0],
                        self.v.p[1] - el.p[1],
                        self.v.p[2] - el.p[2])

        face = [(v.x,v.y,v.z)
                for v in s.extrude_vertices]
        newCanvas = s.canvas.extrude(face, displacement, self.union)

        return State(newCanvas, s.target)

    
if __name__ == "__main__":
    from CAD import *
    while True:
        while True:
            try:
                p = Program.sample(CAD(),1)
                t = p.execute(CAD())
                states = [State(CAD(),t)]
                actions = p.compile(t)
            except RuntimeError: continue
            if actions is None: continue
            for k in p.commands:
                print(k)
            print()

            for a in actions:
                print(a)
                states.append(a(states[-1]))
            t.export("/tmp/targeting.off")
            states[-1].canvas.export("/tmp/reconstruction.off")
            break


