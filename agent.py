import torch.nn.functional as F

from utilities import *
from CAD import *
from state import ALL_TAGS
from state import *

import torch
import torch.nn as nn
from torch.nn.modules.transformer import TransformerEncoder, TransformerEncoderLayer, LayerNorm

class Agent(Module):
    def __init__(self):
        super(Agent, self).__init__()
        self.d_model = 256
        heads = 4
        d_ff = self.d_model*2
        layers = 4


        layer = TransformerEncoderLayer(self.d_model,
                                        heads,
                                        d_ff,
                                        dropout=0.0,
                                        activation="relu")
        self.encoder = TransformerEncoder(layer, layers, LayerNorm(self.d_model))

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

        X = self.tensor(X).unsqueeze(1)
        mask = self.tensor(mask).log()
        
        encodings = self.encoder(X, mask=mask)
        yh = self.predict(encodings.squeeze(1))
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
        for _ in range(maximumLength):
            a = self.sample(states[-1])
            try:
                actions.append(a)
                states.append(a(states[-1]))
            except: break

        return states, actions
            
                
        
        
        



if __name__ == "__main__":
    m = Agent()
    O = torch.optim.Adam(m.parameters(), lr=0.0001)
    iteration = 0
    while True:
        # make a training set of actions/states
        while True:
            p = Program.sample(CAD())
            t = p.execute(CAD())
            for _ in range(10):
                actions = p.compile()
                if actions is not None: break
            if actions is None: continue

            try:
                states = [State(CAD(),t)]
            except FaceFailure: continue
            
            for a in actions:
                states.append(a(states[-1]))
            break

        print("Training on the following program:")
        print(p)
        print("which gives the following target:")
        print(t)
        print("and has the following actions:")
        for a in actions:
            print(a)
            
        m.zero_grad()
        L = 0
        for a,s in zip(actions,states):
            L += m.loss(s, a)
        L = L/len(actions)
        L.backward()
        O.step()
        print(f"LOSS {iteration}\t",L)
        iteration += 1
        if iteration%100 == 1:
            print("going to try a rollout both on some random data and on the couch")
            for target,maximumLength in [(t,len(actions)),
                                         (Program.couch().execute(CAD()),2)]:
                states, actions = m.rollout(target,maximumLength)
                name = "random" if target is t else "couch"
                target.export(f"/tmp/{name}_target.off")
                states[-1].canvas.export(f"/tmp/{name}_reconstruction.off")

    states = [states[0]]
    for _ in range(len(actions)):
        a = m.sample(states[-1])
        print(a)
        states.append(a(states[-1]))
        
    t.export("/tmp/targeting.off")
    states[-1].canvas.export("/tmp/reconstruction.off")
    

    
        

        
