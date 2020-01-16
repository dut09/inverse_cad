from geometric import BoundaryEncoder
from utilities import *
from CAD import *
from state import ALL_TAGS
from state import *

import os
import time

import torch.nn.functional as F
import torch
import torch.nn as nn
#from torch.nn.modules.transformer import TransformerEncoder, TransformerEncoderLayer, LayerNorm
from transformer import TransformerEncoder

class Agent(Module):
    backends = ["gnn","transformer","alternate","torch_transformer"]
    def __init__(self, backend="gnn"):
        super(Agent, self).__init__()
        self.d_model = 128
        heads = 4
        d_ff = self.d_model*2
        layers = 4

        if backend in {"transformer","alternate"}:
            self.encoder = TransformerEncoder(layers, heads, self.d_model,
                                              hidden_dimensionality=d_ff,
                                              alternate=(backend == "alternate"))
            backend = "transformer"
        elif backend == "torch_transformer":
            layer = TransformerEncoderLayer(self.d_model,
                                            heads,
                                            d_ff,
                                            dropout=0.0,
                                            activation="relu")
            self.encoder = TransformerEncoder(layer, layers, LayerNorm(self.d_model))
        elif backend == "gnn":
            self.encoder = BoundaryEncoder(layers=layers, H=self.d_model)

        self.backend = backend

        number_of_actions = 3 # you can go to next vertex, subtract, or add
        self.predict = nn.Linear(self.d_model, number_of_actions)

        self.finalize()

    def forward(self, state):

        def vectorize(f):
            if isinstance(f,Vertex):
                return [1,0,0,int(f in state.canvas_features),int(f in state.target_features)] + \
                    list(f.p)
            if isinstance(f,Edge):
                return [0,1,0,int(f in state.canvas_features),int(f in state.target_features)]
            if isinstance(f,Face):
                return [0,0,1,int(f in state.canvas_features),int(f in state.target_features)]

        fs = list(sorted(list(state.canvas_features|state.target_features),
                         key=str))
        mask = np.zeros((len(fs),len(fs)))
        relationship = [["disconnected" for _ in range(len(fs)) ]
                        for _ in range(len(fs)) ]

        X = []

        for i,f in enumerate(fs):
            v = vectorize(f)
            v.extend([float(int(t in state.tags[f])) for t in ALL_TAGS ])
            v = v + [0.]*(self.d_model - len(v))
            X.append(v)

            for j,fp in enumerate(fs):
                if i == j or f.child(fp) or fp.child(f):
                    mask[i,j] = 1.
                    mask[j,i] = 1.
                if f.child(fp) or fp.child(f):
                    source = {Face: "f",Edge: "e",Vertex: "v"}[f.__class__]
                    destination = {Face: "f",Edge: "e",Vertex: "v"}[fp.__class__]
                    relationship[i][j] = source + "2" + destination
                    relationship[j][i] = destination + "2" + source

        X = self.tensor(X).unsqueeze(0)
        mask = self.tensor(mask).unsqueeze(0)

        if self.backend == "transformer":
            encodings = self.encoder(X, [X.size(1)], mask=mask).squeeze(0)
        elif self.backend == "gnn":
            encodings = self.encoder(X.squeeze(0), relationship)
        else:
            assert False, "pytorch transformer deprecated/nonsupported"
        yh = self.predict(encodings)
        number_objects, number_predictions = yh.shape
        yh = yh.contiguous().view(-1)
        yh = F.log_softmax(yh,dim=-1).contiguous().view(number_objects,number_predictions)
        return yh

    def loss(self, state, action):
        yh = self(state)
        action_index, feature = action.code()
        feature = state.findClose(feature)
        fs = list(sorted(list(state.canvas_features|state.target_features),
                         key=str))
        feature_index = fs.index(feature)

        return -yh[feature_index,action_index]

    def sample(self, state):
        yh = self(state)
        axis1, axis2 = yh.size()
        i = self.to_numpy(torch.distributions.categorical.Categorical(probs=torch.exp(yh).view(-1)).sample())
        fs = list(sorted(list(state.canvas_features|state.target_features),
                         key=str))
        # axis2: number of actions
        # axis1: number of objects
        code = (i%axis2, fs[i//axis2])
        codeToAction = {
            0: lambda v: NextVertex(v),
            1: lambda v: Extrude(v, False),
            2: lambda v: Extrude(v, True)
            }
        return codeToAction[code[0]](code[1])

    def rollout(self, spec, maximumLength):
        states = [State(CAD(), spec)]
        actions = []
        for _ in range(maximumLength):
            a = self.sample(states[-1])
            try:
                actions.append(a)
                states.append(a(states[-1]))
            except: break

        return states, actions

    def rollouts(self, spec, maximumLength, N):
        best = None
        for _ in range(N):
            states, actions = self.rollout(spec, maximumLength)
            iou = states[-1].iou()
            if best is None or iou > best[0]:
                best = (iou,states,actions)
        return best[1:]            
                
        
        
def makeExample(referenceProgram=None, N=2):        
    while True:
        try:
            p = referenceProgram or Program.sample(CAD(),N)
            t = p.execute(CAD())
        except RuntimeError: continue
        
        for _ in range(10):
            actions = p.compile()
            if actions is not None: break
        if actions is None: continue

        try:
            states = [State(CAD(),t)]
        except FaceFailure: continue

        try:
            for a in actions:
                states.append(a(states[-1]))
        except (FaceFailure,RuntimeError): continue
        break
    return states, actions, t, p


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description = "")
    parser.add_argument("--numberExtrusions","-n",default=2,type=int)
    parser.add_argument("--load","-l",default=None)
    parser.add_argument("--backend","-b",default="gnn",choices=Agent.backends)
    parser.add_argument("--save","-s",default=None)
    parser.add_argument("--test","-t",default=False,action='store_true')
    parser.add_argument("--memorize","-m",default=False,action='store_true')
    
    arguments = parser.parse_args()

    if arguments.load:
        m = torch.load(arguments.load)
    else:
        assert not arguments.test, "you need to tell me what checkpoint to load if you are testing"
        m = Agent()

    if not arguments.test:
        O = torch.optim.Adam(m.parameters(), lr=0.0001)
        iteration = 0
        timeMakingExamples = 0
        modelTime = 0
        losses = []
        if arguments.memorize:
            states, actions, t, p = makeExample(Program.couch())
        while True:
            startTime = time.time()
            # make a training set of actions/states
            if not arguments.memorize:
                states, actions, t, p = makeExample(N=arguments.numberExtrusions)
            timeMakingExamples += (time.time() - startTime)

            if False:
                print("Training on the following program:")
                print(p)
                print("which gives the following target:")
                print(t)
                print("and has the following actions:")
                for a in actions:
                    print(a)

            startTime = time.time()
            m.zero_grad()
            L = 0
            for a,s in zip(actions,states):
                L += m.loss(s, a)
            L = L/len(actions)
            L.backward()
            O.step()
            modelTime += (time.time() - startTime)
            losses.append(L.cpu().tolist())
            iteration += 1
            if iteration%50 == 1:
                print(f"LOSS {iteration}\t",sum(losses)/len(losses))
                losses = []
                print(f"Total graphics time: {timeMakingExamples}")
                print(f"Total model time: {modelTime}")

                if arguments.save:
                    torch.save(m,arguments.save)
    else:
        if arguments.memorize:
            p = Program.couch()
            t = p.execute(CAD())
            states, actions = m.rollouts(t,len(p.compile()),1)
            State.exportTrace(states, actions,"couch")
            states[-1].canvas.export("couch_reconstruction.off")
            t.export("couch_target.off")
        else:
            os.system("mkdir  -p data")
            for n in range(100):
                os.system(f"rm data/{n}_*")
                states, actions, t, p = makeExample(N=arguments.numberExtrusions)
                states[-1].canvas.export(f"data/{n}_target.off")
                State.exportTrace(states, actions, f"data/{n}_gt_")
                CAD.instrument = True
                print("about to do a rollout")
                states, actions = m.rollout(t,len(actions))
                print("successfully rolled")
                CAD.instrument = False                
                State.exportTrace(states, actions, f"data/{n}_")
                print("successfully exported trace")
                
        
        

            






