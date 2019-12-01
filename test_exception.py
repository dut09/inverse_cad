import sys
import os
import numpy as np

from scene import *

s = Scene()

# Create the sofa scene.
try:
    s.ExtrudeFromString("extrude 1 0 0 +")
except Exception as e:
    print('I failed gracefully:', e)
print('I can continue...')