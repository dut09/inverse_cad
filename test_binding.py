import os
from scene import *

# Coloful print.
def print_error(*message):
    print('\033[91m', 'ERROR ', *message, '\033[0m')

def print_ok(*message):
    print('\033[92m', *message, '\033[0m')

def print_info(*message):
    print('\033[93m', *message, '\033[0m')

############## Sample code begins ################

s = Scene()
s_copy = s.Clone()

# Create the sofa scene.
print_info('Creating the sofa scene...')
s.ExtrudeFromString("extrude 1 0 0 1 1 0 -1 1 0 -1 0.5 0 0 0.5 0 0 0 0 0 0.1 1 +")
s.ExtrudeFromString("extrude -0.8 0.2 1 0.8 0.2 1 0.8 0.8 1 -0.8 0.8 1 0 0 -0.5 -")
s.SaveScene("sofa.nef3")
s.SaveScene("sofa.off")

# Verify the clone.
print_info('Verifing if deep copy works...')
print_info('Output 1')
s.ListSceneVertices()
print_info('Output 2')
s_copy.ListSceneVertices()
print_info('You should see two *different* outputs above. Press enter to continue...')
input()

# Reconstruct the sofa scene.
print_info('Reconstructing sofa...')
s_copy.LoadTarget('sofa.nef3')
s_copy.ExtrudeFromTargetRef(2, 0, 0, 6, '+')
s_copy.ListSceneFaces()
print_info('You should see some of the lines in the output above become green. Press enter to continue...')
input()

s_copy.ExtrudeFromTargetRef(16, 0, 14, 11, '-')
s_copy.ListSceneFaces()
print_info('You should see all output above become green. Press enter to continue...')
input()

s_copy.SaveScene('sofa_reconstructed.nef3')
s_copy.SaveScene('sofa_reconstructed.off')
print_info('You can use meshlab to check sofa.off and sofa_reconstructed.off. They should be identical.')

############## Sample code ends ################