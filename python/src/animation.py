import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from src.file_reader import leer_header, leer_frames
import os
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

def animate(filename, save_as=None):
    N, L = leer_header(filename)
    
    square_size = 0.09
    rect_width = 0.09
    rect_height = L 

    # Necessary offsets for line art :)
    total_width = square_size + rect_width
    total_height = max(square_size, rect_height)
    y_offset = (square_size - rect_height) / 2
    
    # Quality
    fig, ax = plt.subplots(dpi=150)

    # Axis and legend
    ax.set_xlim(0, total_width)
    ax.set_ylim(0, total_height)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")

    # Remove unnecessary vb lines
    ax.set_aspect('equal')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(True)
    ax.spines['left'].set_visible(True)

    # Left Square
    ax.plot([0, square_size], [0, 0], color='black', linewidth=1)
    ax.plot([0, 0], [0, square_size], color='black', linewidth=1)
    ax.plot([0, square_size], [square_size, square_size], color='black', linewidth=1)
    
    # Right Square
    ax.plot([square_size, total_width], [y_offset, y_offset], color='black', linewidth=1)
    ax.plot([total_width, total_width], [y_offset, y_offset + rect_height], color='black', linewidth=1)
    ax.plot([square_size, total_width], [y_offset + rect_height, y_offset + rect_height], color='black', linewidth=1)

    # Opening/ Connector
    ax.plot([square_size, square_size], 
            [y_offset + L, total_height], 
            color='black', linewidth=1)
    ax.plot([square_size, square_size], 
            [0, y_offset], 
            color='black', linewidth=1)

    # Particle scale
    scat = ax.scatter([], [], s=10)


    frames = list(leer_frames(filename))

    def init():
        scat.set_offsets(np.empty((0, 2)))
        return scat,

    def update(frame):
        time, _, _, particles = frame
        coords = [(float(p.x), float(p.y)) for p in particles]
        scat.set_offsets(coords)
        return scat,

    ani = FuncAnimation(fig, update, frames=frames, init_func=init,
                        blit=True, interval=50)

    if save_as:
        ani.save(save_as, fps=30)
    else:
        plt.show()

if __name__ == "__main__":
    animate("../test-data/initial_state.csv", save_as="animacion.mp4")
