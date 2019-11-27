import imageio
import os
import sys
from common import *

def run(folder_name, gif_name, fps):
    frame_names = [os.path.join(folder_name, f) for f in os.listdir(folder_name)
        if os.path.isfile(os.path.join(folder_name, f)) and f.endswith('.png')]
    frame_names = sorted(frame_names)

    # Read images.
    images = [imageio.imread(f) for f in frame_names]
    if fps > 0:
        imageio.mimsave(gif_name, images, fps=fps)
    else:
        imageio.mimsave(gif_name, images)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print_error('Usage: python export_gif <image folder> <gif name> [fps]')
        sys.exit(0)
    folder_name = sys.argv[1]
    gif_name = sys.argv[2]
    if len(sys.argv) >= 4:
        fps = int(sys.argv[3])
    else:
        fps = 0
    run(folder_name, gif_name, fps)