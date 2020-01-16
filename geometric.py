from utilities import *

import numpy as np

import torch
import torch.nn as nn

# from torch_geometric.nn import MessagePassing
# from torch_geometric.utils import add_self_loops, degree



class GraphLayer(Module):
    def __init__(self, H):
        super(GraphLayer, self).__init__()

        
        C = len(BoundaryEncoder.edgeCode)
        self.C = C
        self.H = H
        
        self.message = nn.Sequential(nn.Linear(H*2 + C, H),
                                     nn.ReLU(),
                                     nn.Linear(H,H))
        self.update = nn.Sequential(nn.Linear(H*2, H),
                                    nn.ReLU(),
                                    nn.Linear(H,H))

        self.finalize()

    def forward(self, e, c):
        """e: embedding of CAD features, NxH
        c: one-hot characterization of edge, NxNxC"""
        N = e.size(0)
        x = e.unsqueeze(1).expand(-1,N,-1)
        y = e.unsqueeze(0).expand(N,-1,-1)
        z = torch.cat([x,y,c],2)
        z = self.message(z).sum(1)
        prediction = self.update(torch.cat([z,e],1))

        return prediction

class BoundaryEncoder(Module):
    edgeCode = ["e2v","e2f","f2e","v2e","v2f","f2v","disconnected"]
    def __init__(self, layers, H=128):
        super(BoundaryEncoder, self).__init__()
        self.layers = nn.ModuleList([GraphLayer(H) for _ in range(layers) ])
        self.finalize()

    def forward(self, e, c):
        """c: should be a matrix-like thing whose entries are strings inside of edgeCode"""
        N = e.size(0)
        a = np.zeros((N,N,len(BoundaryEncoder.edgeCode)))
        for i in range(N):
            for j in range(N):
                theCode = BoundaryEncoder.edgeCode.index(c[i][j])
                a[i,j,theCode] = 1
        a = self.tensor(a)

        for l in self.layers:
            e = l(e,a)
        return e
        




# class GraphEncoder(MessagePassing):
#     def __init__(self, features, H):
#         super(GraphEncoder, self).__init__(aggr='add')
#         self.ff = nn.Linear(features, H)
        
#     def forward(self, x, edge_index):
#         # x has shape [N, in_channels]
#         # edge_index has shape [2, E]

#         # Step 1: Add self-loops to the adjacency matrix.
#         if False:
#             edge_index, _ = add_self_loops(edge_index, num_nodes=x.size(0))

#         # Step 2: Linearly transform node feature matrix.
#         x = self.ff(x)

#         # Step 3-5: Start propagating messages.
#         return self.propagate(edge_index, size=(x.size(0), x.size(0)), x=x)

#     def message(self, x_j, edge_index, size):
#         # x_j has shape [E, out_channels]

#         # Step 3: Normalize node features.
#         row, col = edge_index
#         deg = degree(row, size[0], dtype=x_j.dtype)
#         deg_inv_sqrt = deg.pow(-0.5)
#         norm = deg_inv_sqrt[row] * deg_inv_sqrt[col]

#         return norm.view(-1, 1) * x_j

#     def update(self, aggr_out):
#         # aggr_out has shape [N, out_channels]

#         # Step 5: Return new node embeddings.
#         return aggr_out


# N = 5
# H = 32
# C = 3
# c = torch.randn(N,N,C)
# e = torch.randn(N,H)

# ff = nn.Sequential(nn.Linear(H*2 + C, H),
#                    nn.ReLU(),
#                    nn.Linear(H,H))
# update = nn.Sequential(nn.Linear(H*2, H),
#                        nn.ReLU(),
#                        nn.Linear(H,H))

# for n in range(N):
#     for m in range(N):
#         assert (z[n,m] - torch.cat([e[n],e[m]])).abs().sum() < 0.0001

# z = torch.cat([z,c],2)
# z = ff(z).sum(1)
# prediction = update(torch.cat([z,e],1))
# print(prediction.shape)

# # now time to do it manually
# for n in range(N):
#     neighbors = []
#     for m in range(N):
#         neighbors.append(ff(torch.cat([e[n],e[m],c[n,m]])))
#     allMessages = torch.stack(neighbors,0).sum(0)
#     aggregate = update(torch.cat([allMessages,e[n]]))
#     print(aggregate)
#     print(prediction[n])
#     assert (aggregate - prediction[n]).abs().sum() < 0.0001


