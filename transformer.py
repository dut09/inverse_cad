from utilities import *
import torch.nn.functional as F

import random

import math

class Pointer:
    def __init__(self, index):
        self.index = index
    def __str__(self): return f"P({self.index})"
    def __repr__(self): return str(self)
    def __eq__(self,o): return isinstance(o,Pointer) and o.index == self.index
    def __ne__(self,o): return not (self == o)
    def __hash__(self): return hash(self.index)

class PositionalEncoder(Module):
    def __init__(self, d_model, max_seq_len = 80):
        super(PositionalEncoder, self).__init__()
        self.d_model = d_model
        
        # create constant 'pe' matrix with values dependant on 
        # pos and i
        pe = np.zeros((max_seq_len, d_model))
        for pos in range(max_seq_len):
            for i in range(0, d_model, 2):
                pe[pos, i] = math.sin(pos / (10000 ** ((2 * i)/d_model)))
                pe[pos, i + 1] = math.cos(pos / (10000 ** ((2 * (i + 1))/d_model)))

        self.pe = pe

        self.finalize()
 
    
    def forward(self, x):
        """x: BxLxE"""
        # make embeddings relatively larger
        x = x * math.sqrt(self.d_model)
        #add constant to embedding
        L = x.size(1)
        pe = self.tensor(self.pe).unsqueeze(0)[:,:L,:]
        return x + pe
    
class MultiHeadAttention(Module):
    def __init__(self, heads, entity_dimensionality):
        super().__init__()
        self.entity_dimensionality = entity_dimensionality
        self.heads = heads

        assert entity_dimensionality%heads == 0,\
        "dimensionality of entities must be divisible by number of heads"

        # Dimensionality of each head
        self.d = entity_dimensionality//heads

        self.Q = nn.Linear(entity_dimensionality, entity_dimensionality)
        self.V = nn.Linear(entity_dimensionality, entity_dimensionality)
        self.K = nn.Linear(entity_dimensionality, entity_dimensionality)
        self.output = nn.Linear(entity_dimensionality, entity_dimensionality)

        self.finalize()

    @staticmethod
    def makeAttentionMask(n_entities, n_attended=None, N=None, M=None):
        if n_attended is None:
            n_attended = n_entities
            M = N or max(n_entities)
        
        N = N or max(n_entities)
        M = M or max(n_attended)
        B = len(n_entities)
        assert len(n_attended) == B
        
        mask = np.ones((B,N,M))
        for b,n in enumerate(n_entities):
            mask[b,n:,:] = 0
        for b,m in enumerate(n_attended):
            mask[b,:,m:] = 0
        return mask

    def _forward(self, entities, n_entities, mask=None):
        """
        entities: Bx(# entities)x(entity_dimensionality)
        n_entities: list of length B, each entry should be in range 0 .. #entities-1
        mask: if this is provided it should be of size BxNxN, with each entry in {0.,1.}. Overrides n_entries.
        returns: (# entities)x(entity_dimensionality)
        """
        B = entities.size(0) # batch size
        N = entities.size(1) # maximum number of entities

        # query, values, and keys should all be of size BxHxExD
        q = self.Q(entities).view(B, N, self.heads, self.d).transpose(1,2)
        v = self.V(entities).view(B, N, self.heads, self.d).transpose(1,2)
        k = self.K(entities).view(B, N, self.heads, self.d).transpose(1,2)

        # attention[i,j] = q_i . k_j
        # i.e., amount that object I is attending to object J
        attention = q @ (k.transpose(-2,-1)) / (self.d**0.5)
        # attention has shape [B,H,N,N]

        # apply the mask, calculating it from n_entities if needed
        if mask is None:
            mask = self.tensor(MultiHeadAttention.makeAttentionMask(n_entities, N=N))
        attention = attention.masked_fill(mask.unsqueeze(1) == 0, NEGATIVEINFINITY)
        attention = F.softmax(attention, dim=-1)
        attention = attention.masked_fill(mask.unsqueeze(1) == 0, 0.)

        # Mix together values
        o = (attention@v).transpose(1,2).contiguous().view(B, N, self.entity_dimensionality)

        # Apply output transformation
        return self.output(o)

    def forward(self, entities, n_entities, attend_over=None, n_attended=None, mask=None):
        """
        entities: Bx(# entities)x(entity_dimensionality)
        n_entities: list of length B, each entry should be in range 0 .. #entities-1
        attend_over: BxMxE
        n_attended: list of length B, each entry should be in range 0 .. M
        mask: if this is provided it should be of size BxNxM, with each entry in {0.,1.}. Overrides n_entries/n_attended
        returns: (# entities)x(entity_dimensionality)
        """
        if attend_over is None:
            attend_over = entities
            assert n_attended is None
            n_attended = n_entities
        
        B = entities.size(0) # batch size
        N = entities.size(1) # maximum number of entities
        M = attend_over.size(1) # maximum number of things we can attend over

        # query, values, and keys should all be of size BxHxExD
        q = self.Q(entities).view(B, N, self.heads, self.d).transpose(1,2)
        v = self.V(attend_over).view(B, M, self.heads, self.d).transpose(1,2)
        k = self.K(attend_over).view(B, M, self.heads, self.d).transpose(1,2)

        # attention[i,j] = q_i . k_j
        # i.e., amount that object I is attending to object J
        attention = q @ (k.transpose(-2,-1)) / (self.d**0.5)
        # attention has shape [B,H,N,M]

        # apply the mask, calculating it from n_entities if needed
        if mask is None:
            mask = self.tensor(MultiHeadAttention.makeAttentionMask(n_entities, n_attended,
                                                                    N=N, M=M))
        attention = attention.masked_fill(mask.unsqueeze(1) == 0, NEGATIVEINFINITY)
        attention = F.softmax(attention, dim=-1)
        attention = attention.masked_fill(mask.unsqueeze(1) == 0, 0.)

        # Mix together values
        o = (attention@v).transpose(1,2).contiguous().view(B, N, self.entity_dimensionality)

        # Apply output transformation
        return self.output(o)

    
    
class TransformerLayer(Module):
    def __init__(self, heads, entity_dimensionality, hidden_dimensionality=None):
        super(TransformerLayer, self).__init__()
        hidden_dimensionality = hidden_dimensionality or entity_dimensionality
        
        self.attention = MultiHeadAttention(heads=heads, entity_dimensionality=entity_dimensionality)
        self.norm_1 = LayerNorm(entity_dimensionality)
        self.norm_2 = LayerNorm(entity_dimensionality)
        self.ff = nn.Sequential(nn.Linear(entity_dimensionality, hidden_dimensionality),
                                nn.ReLU(),
                                nn.Linear(hidden_dimensionality, entity_dimensionality),
                                nn.ReLU())
        self.finalize()

    def forward(self, x, n_entities, mask=None):
        x = x + self.attention(self.norm_1(x), n_entities, mask=mask)
        x = x + self.ff(self.norm_2(x))
        return x

class TransformerEncoder(Module):
    def __init__(self, layers, heads, entity_dimensionality, hidden_dimensionality=None):
        super(TransformerEncoder, self).__init__()
        self.layers = \
         nn.ModuleList([TransformerLayer(heads=heads, entity_dimensionality=entity_dimensionality,
                                         hidden_dimensionality=hidden_dimensionality)])
        self.finalize()

    def forward(self, x, n_entities, mask=None):
        if mask is None:
            N = x.size(1)
            mask = self.tensor(MultiHeadAttention.makeAttentionMask(n_entities, N=N))
        for l in self.layers:
            x = l(x, n_entities, mask=mask)
        return x

class DecoderBlock(Module):
    def __init__(self, heads, entity_dimensionality, hidden_dimensionality=None):
        super(DecoderBlock, self).__init__()
        hidden_dimensionality = hidden_dimensionality or entity_dimensionality
        
        self.output_attention = MultiHeadAttention(heads=heads, entity_dimensionality=entity_dimensionality)
        self.input_attention = MultiHeadAttention(heads=heads, entity_dimensionality=entity_dimensionality)
        self.norm_1 = LayerNorm(entity_dimensionality)
        self.norm_2 = LayerNorm(entity_dimensionality)
        self.norm_3 = LayerNorm(entity_dimensionality)
        self.ff = nn.Sequential(nn.Linear(entity_dimensionality, hidden_dimensionality),
                                nn.ReLU(),
                                nn.Linear(hidden_dimensionality, entity_dimensionality),
                                nn.ReLU())
                
        
        self.finalize()

    def forward(self, encoder_objects, decoder_objects, n_encoder, n_decoder,
                encoder_mask=None, decoder_mask=None):
        # all of the decoder outputs talk to each other
        a = self.output_attention(decoder_objects, n_decoder, mask=decoder_mask)
        decoder_objects = decoder_objects + self.norm_1(a)
        # all of the decoder and encoder talk to each other
        a = self.input_attention(decoder_objects, n_decoder,
                                 attend_over=encoder_objects,
                                 n_attended=n_encoder)
        decoder_objects = decoder_objects + self.norm_2(a)
        decoder_objects = decoder_objects + self.norm_3(self.ff(decoder_objects))
        return decoder_objects

class TransformerDecoder(Module):
    """Transformer pointer network!"""
    def __init__(self, layers, heads, hidden_dimensionality=None, embedding_size=64):
        super(TransformerDecoder, self).__init__()
        self.layers = \
         nn.ModuleList([DecoderBlock(heads=heads, entity_dimensionality=embedding_size,
                                     hidden_dimensionality=hidden_dimensionality)])
        self.finalize()

    def forward(self, encodedInputs, inputLengths, outputsSoFar, outputLengths):
        """encodedInputs: BxNxE
        outputsSoFar: BxMxL
        outputLengths: B-dimensional tensor whose entries are in [0,N-1]
        inputLengths: B-dimensional tensor whose entries are in [0,M-1]"""
        for l in self.layers:
            outputsSoFar = l(encodedInputs, outputsSoFar,
                             inputLengths, outputLengths)
        return outputsSoFar

class Transformer(Module):
    def __init__(self, lexicon, layers, heads, hidden_dimensionality=None, embedding_size=64,
                 positional_input=False, positional_output=True, vectorizeInput=None):
        super(Transformer, self).__init__()

        if positional_input:
            self.positional_input = PositionalEncoder(embedding_size)
        else:
            self.positional_input = IdentityLayer()
        if positional_output:
            self.positional_output = PositionalEncoder(embedding_size)
        else:
            self.positional_output = IdentityLayer()
            
        self.lexicon = ["START","FINISHED","POINTER"] + list(lexicon)
        self.lexicon2index = {w: n
                              for n,w in enumerate(self.lexicon) }
        self.embedding = LexicalEmbedding(self.lexicon, embedding_size)
        self.vectorizeInput = vectorizeInput or self.embedding
        #self.embedding = nn.Embedding(len(self.lexicon), embedding_size)
        self.encoder = TransformerEncoder(heads=heads, layers=layers,
                                          entity_dimensionality=embedding_size,
                                          hidden_dimensionality=hidden_dimensionality)
        self.decoder = TransformerDecoder(heads=heads, layers=layers,
                                          embedding_size=embedding_size,
                                          hidden_dimensionality=hidden_dimensionality)
        self.predictToken = nn.Sequential(nn.Linear(embedding_size,len(self.lexicon)),
                                          nn.LogSoftmax(dim=-1))

        # pointer attention
        self.inputKey = nn.Linear(embedding_size,embedding_size)
        self.outputQueries = nn.Linear(embedding_size,embedding_size)

        self.embedding_size = embedding_size

        self.finalize()

    def forward(self, inputs, outputs,
                inputEncodings=None, inputSizes=None):
        """inputs/outputs: list of list of symbols in lexicon. outputs can also contain `Pointer`s
        inputEncodings/inputSizes can override inputs, and should be the output of self.encoder.
        returns: log distribution over lexicon, log distribution over input objects (pointer attention)"""
        if inputEncodings is not None:
            assert inputs is None
            B = inputEncodings.size(0)
            N = inputEncodings.size(1)
        else:
            B = len(inputs)

            inputSizes = [len(ws) for ws in inputs]
            N = max(inputSizes)
            
            X = self.vectorizeInput(inputs)
            X = self.positional_input(X)
            inputEncodings = self.encoder(X, inputSizes)

        assert len(outputs) == B
        outputSizes = [len(ws) for ws in outputs]
        M = max(outputSizes)
        Y = [ ["POINTER" if isinstance(w,Pointer) else w for w in ws]
              for ws in outputs ]
        Y = self.embedding(Y)
        Y = self.positional_output(Y)
        # Add in pointer values
        # Y: BxMxE
        # Unbatched version
        # for b in range(B):
        #     for m,o in enumerate(outputs[b]):
        #         if isinstance(o, Pointer):
        #             Y[b,m] += inputEncodings[b,o.index]
        pointerIndices = [ [ o.index if isinstance(o,Pointer) else N for o in outputs[b] ] + [N]*(M - len(outputs[b]))
                           for b in range(B) ]
        inputEncodings_padded = torch.cat([inputEncodings,self.device(torch.zeros(B,1,self.embedding_size))], 1)
        #inputEncodings_padded: Bx(N+1)xE
        #pointerIndices: BxM, elements living in 0..N
        #pointerInfo: BxMxE. pointerInfo[b,m,e] = inputEncodings_padded[b,pointerIndices[b,m],e]
        pointerInfo = inputEncodings_padded[torch.arange(B).unsqueeze(-1),torch.tensor(pointerIndices)]
        if False: # check
            for b in range(B):
                for m in range(M):
                    print(pointerInfo[b,m] == inputEncodings_padded[b,pointerIndices[b][m]])
        
        Y = Y + pointerInfo
        
        outputEncodings = self.decoder(inputEncodings, inputSizes, Y, outputSizes)

        # FIXME: Which of the output encodings participates in the calculation of prediction?
        relevantOutputs = torch.stack([ outputEncodings[b,outputSizes[b] - 1]
                                        for b in range(B) ])
        tokenPrediction = self.predictToken(relevantOutputs)

        
        attentionQueries = self.outputQueries(relevantOutputs)
        attentionKeys = self.inputKey(inputEncodings)
        attentionMatrix = (attentionKeys@(attentionQueries.unsqueeze(2))).squeeze(2)
        attentionMatrix = attentionMatrix/(self.embedding_size**0.5)
        # Mask
        pointerMask = np.zeros((B,N))
        for b,sz in enumerate(inputSizes): pointerMask[b,sz:] = NEGATIVEINFINITY
        attentionMatrix = attentionMatrix + self.tensor(pointerMask)
        attention = F.log_softmax(attentionMatrix, dim=-1)
            
        return tokenPrediction, attention

    def logLikelihood(self, x, y):
        # calculate embeddings of the input and then duplicate for the whole batch
        inputs = [x]
        X = self.vectorizeInput(inputs)
#        print("input",        print("vectorize",X)
        X = self.positional_input(X)
#        print("position lysed",X)
        inputEncodings = self.encoder(X, [len(x)])
#        print("encoded",X)


        padded_y = ["START"] + y + ["FINISHED"]
        outputs = [padded_y[:n] for n in range(1,len(padded_y)) ]
        prediction_targets = padded_y[1:]
        B = len(outputs)

        inputEncodings = inputEncodings.repeat(B,1,1)
        outputSizes = [len(ws) for ws in outputs]

        predictions, attention = self(inputs=None, outputs=outputs,
                                      inputEncodings=inputEncodings, inputSizes=[len(x)]*B)

        yh = self.tensor([self.lexicon2index["POINTER" if isinstance(pt,Pointer) else pt]
                          for pt in prediction_targets])
        tokenLikelihood = predictions.gather(1,yh.unsqueeze(1)).sum()

        if True:
            pointerMask = np.array([int(isinstance(pt, Pointer))*1.
                                    for pt in prediction_targets ])
            pointerTargets = np.array([pt.index if isinstance(pt, Pointer) else 0
                                       for pt in prediction_targets ])
            attentionLikelihood = attention.gather(1,self.tensor(pointerTargets).unsqueeze(1))*self.tensor(pointerMask)
            attentionLikelihood = attentionLikelihood.sum()
            return attentionLikelihood + tokenLikelihood
        else:
            return tokenLikelihood

    def batched_logLikelihood(self, xs, ys):
        assert False, "not working"
        # calculate embeddings of the input and then duplicate for the whole batch
        X = self.vectorizeInput(xs)
#        print("input",        print("vectorize",X)
        X = self.positional_input(X)
#        print("position lysed",X)
        inputSizes = [len(x) for x in xs]
        inputEncodings = self.encoder(X, inputSizes)
#        print("encoded",X)

        padded_ys = [ ["START"] + y + ["FINISHED"]
                      for y in ys ]
        outputs = [padded_y[:n]
                   for padded_y in padded_ys
                   for n in range(1,len(padded_y)) ]
        prediction_targets = [ pt
                               for padded_y in padded_ys
                               for pt in padded_y[1:] ]
        B = len(outputs)

        inputEncodings = torch.cat([ inputEncodings[n].repeat(len(y) + 1,1,1)
                                       for n,y in enumerate(ys) ],
                                   0)
        assert inputEncodings.size(0) == B
        inputSizes = [isize
                      for n,y in enumerate(ys)
                      for isize in [inputSizes[n]]*(len(y) + 1) ]
                                     
        outputSizes = [len(ws) for ws in outputs]

        predictions, attention = self(inputs=None, outputs=outputs,
                                      inputEncodings=inputEncodings, inputSizes=inputSizes)

        yh = self.tensor([self.lexicon2index["POINTER" if isinstance(pt,Pointer) else pt]
                          for pt in prediction_targets])
        tokenLikelihood = predictions.gather(1,yh.unsqueeze(1)).sum()

        if True:
            pointerMask = np.array([int(isinstance(pt, Pointer))*1.
                                    for pt in prediction_targets ])
            pointerTargets = np.array([pt.index if isinstance(pt, Pointer) else 0
                                       for pt in prediction_targets ])
            attentionLikelihood = attention.gather(1,self.tensor(pointerTargets).unsqueeze(1))*self.tensor(pointerMask)
            attentionLikelihood = attentionLikelihood.sum()
            return attentionLikelihood + tokenLikelihood
        else:
            return tokenLikelihood

    def sample(self, x, substitutePointers=True):
        """x: list of symbols in lexicon
        substitutePointers: when the model indexes into the input using pointer attention, should we return Pointer objects or the original objects in the input"""
        # calculate embeddings of the input
        inputs = [x]
        X = self.vectorizeInput(inputs)
        X = self.positional_input(X)
        inputEncodings = self.encoder(X, [len(x)])

        y = ["START"]
        to_return = []
        

        while len(y) < 20:
            prediction, attention = self(inputs=None, outputs=[y],
                                         inputEncodings=inputEncodings, inputSizes=[len(x)])
            prediction = prediction[0]
            attention = attention[0]
            
            next_symbol = self.lexicon[torch.multinomial(prediction.exp(), 1)]
            if next_symbol == "FINISHED": break
            
            if next_symbol == "POINTER":
                pi = torch.multinomial(attention.exp(), 1).tolist()[0]
                if substitutePointers:
                    to_return.append(x[pi])
                else:
                    to_return.append(Pointer(pi))

                y.append(Pointer(pi))
            else:
                y.append(next_symbol)
                to_return.append(next_symbol)            
            
        return to_return
            
        

        
        
        
if __name__ == "__main__":
    m = Transformer([" < "," > "," == "] + [str(n) for n in range(9) ],
                    positional_input=True, positional_output=True,
                    layers=3,heads=4,
                    embedding_size=32)

    # examples = [("this vocabulary","this is the vocabulary"),
    #             ("the","the vocabulary"),
    #             ("this","this vocabulary"),
    #             ("vocabulary vocabulary","this this")]
    # examples = [(x.split(" "), y.split(" "))
    #             for x,y in examples ]

    optimizer = torch.optim.Adam(m.parameters(), lr=0.001, eps=1e-3, amsgrad=True)
    iteration = 0
    frequency = 300
    correct = 0
    totalLosses = 0.

    bs = 10
    
    while True:
        xs = []
        ys = []
        for _ in range(bs) :
            a = random.choice(range(9))
            b = random.choice(range(9))

            if random.choice([True,False]):
                if a > b: y = [Pointer(0)," > ",Pointer(1)]
                if a < b: y = [Pointer(0)," < ",Pointer(1)]
                if a == b: y = [Pointer(0)," == ",Pointer(1)]
                y = ['0'] + y
            else:
                if a < b: y = [Pointer(1)," > ",Pointer(0)]
                if a > b: y = [Pointer(1)," < ",Pointer(0)]
                if a == b: y = [Pointer(1)," == ",Pointer(0)]
                y = ['1'] + y
            x = [str(a),str(b)]

            if False:
                x = [str(a),
                     str(b),
                     " == "," == "," == "," == "," == "," == "," == "," == "]
                y = [str(a),
                     " > " if a > b else (" == " if a == b else " < "),
                     str(b)]
                y = [str(b),' == ',str(a)]
                y = [Pointer(0)]
                y = [Pointer(1)]
                y = [Pointer(0),Pointer(1)]
                y = ['1'] if random.choice([True,False]) else ['2']
                y = ['0',Pointer(0)] if random.choice([True,False]) else ['1',Pointer(1)]
                y = [Pointer(0)] if random.choice([True,False]) else [Pointer(1)]
            """Working: Can memorize that it should always output a constant sequence"""
            """Working: Copying input without pointer attention mechanism"""
            """Working: Copying the first/second input using pointer attention"""
            """Working: Copying the first and then the second input using pointer attention"""
            """Working: Randomly choosing to either print a one or two"""
            """Working: Randomly choosing to attend to either one or two, AFTER printing '0'/'1'"""
            """Working: randomly choosing between copying either the first/second"""

            xs.append(x)
            ys.append(y)
        
        m.zero_grad()
        L = 0.
        for x,y in zip(xs,ys) :
            L =- m.logLikelihood(x,y)
        Lp = -m.batched_logLikelihood(xs,ys)
        print(L,Lp)
        L.backward()
        totalLosses += L.data.cpu().numpy()
        optimizer.step()
        iteration += 1

        # yh = m.sample(x, substitutePointers=True)
        # if str(a) in yh and str(b) in yh:
        #     try:
        #         command = " ".join(yh)
        #         print(command)
        #         if eval(command): correct += 1
        #     except: pass
        if iteration%frequency == 0:
            print(f"{correct/frequency} accuracy")
            correct = 0
            print("<L> = ", totalLosses/frequency)
            totalLosses = 0
            print(x,m.sample(x),y)
