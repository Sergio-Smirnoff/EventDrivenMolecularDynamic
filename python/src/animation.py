import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from src.file_reader import leer_header, leer_frames
import numpy as np
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

def _resolve_path(p: str | Path) -> Path:
    p = Path(p)
    return p if p.is_absolute() else (BASE_DIR / p).resolve()

def animate(filename, save_as=None, print_data=False, print_first_k=5):
    file_path = _resolve_path(filename)
    N, L = leer_header(str(file_path))

    square_size = 0.09
    rect_width = 0.09
    rect_height = L
    ball_radius = 0.0015

    total_width = square_size + rect_width
    total_height = max(square_size, rect_height)
    y_offset = (square_size - rect_height) / 2

    fig, ax = plt.subplots(dpi=150)
    ax.set_xlim(0, total_width)
    ax.set_ylim(0, total_height)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect('equal')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    ax.plot([0, 0], [0, square_size], color='black', linewidth=1)
    ax.plot([0, square_size], [0, 0], color='black', linewidth=1)
    ax.plot([0, square_size], [square_size, square_size], color='black', linewidth=1)
    ax.plot([square_size, total_width], [y_offset, y_offset], color='black', linewidth=1)
    ax.plot([total_width, total_width], [y_offset, y_offset + rect_height], color='black', linewidth=1)
    ax.plot([square_size, total_width], [y_offset + rect_height, y_offset + rect_height], color='black', linewidth=1)
    ax.plot([square_size, square_size], [y_offset + L, total_height], color='black', linewidth=1)
    ax.plot([square_size, square_size], [0, y_offset], color='black', linewidth=1)

    frames = list(leer_frames(str(file_path)))

    first_time, _, first_particles = frames[0]
    circles = [
        Circle((p.x, p.y), radius=ball_radius, fc="blue", ec="none", alpha=0.7)
        for p in first_particles
    ]
    for c in circles:
        ax.add_patch(c)

    def init():
        for c in circles:
            c.set_center((-10, -10))
        return circles

    def update(frame):
        time, _, particles = frame

        for c, p in zip(circles, particles):
            c.set_center((p.x, p.y))

        if print_data:
            print(f"t={time:.5f} | N={len(particles)}")
            for p in particles[:print_first_k]:
                print(f"  id={p.id} x={p.x} y={p.y} vx={p.vx} vy={p.vy}")
            if len(particles) > print_first_k:
                print(f"  ... ({len(particles) - print_first_k} más)")

        return circles

    ani = FuncAnimation(fig, update, frames=frames, init_func=init,
                        blit=True, interval=20)

    if save_as:
        ani.save(save_as, fps=30)
    else:
        plt.show()

if __name__ == "__main__":
    animate("../test-data/initial_state.csv", save_as="animacion.mp4", print_data=True, print_first_k=3)
