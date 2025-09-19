import cv2
import numpy as np
from pathlib import Path
from .file_reader import leer_frames, leer_header

def animate(filename, save_as=None, print_data=False, print_first_k=5):
    file_path = Path(filename)
    try:
        N, L = leer_header(str(file_path))
        frames_data = list(leer_frames(str(file_path)))
        if not frames_data:
            print("Error: No frames were read from the data file.")
            return
    except Exception as e:
        print(f"An error occurred while reading the file: {e}")
        return

    square_size = 0.09
    rect_width = 0.09
    rect_height = L
    world_width = square_size + rect_width
    world_height = max(square_size, rect_height)
    ball_radius_world = 0.0015
    y_offset = (square_size - rect_height) / 2

    PADDING = 80
    scaled_sim_height = 720
    scale = scaled_sim_height / world_height
    IMG_HEIGHT = scaled_sim_height + (2 * PADDING)
    IMG_WIDTH = int(world_width * scale) + (2 * PADDING)
    ball_radius_pixels = max(1, int(ball_radius_world * scale))

    BG_COLOR = (245, 245, 245)
    PARTICLE_COLOR = (220, 120, 0)
    EVENT_COLOR = (40, 100, 255)
    BOUNDARY_COLOR = (50, 50, 50)
    TEXT_COLOR = (80, 80, 80)
    PARTICLE_BORDER = (50, 50, 50)
    AXIS_COLOR = (100, 100, 100)

    def world_to_pixel_vectorized(coords: np.ndarray) -> np.ndarray:
        pixel_coords = np.empty_like(coords, dtype=int)
        pixel_coords[:, 0] = (coords[:, 0] * scale + PADDING).astype(int)
        pixel_coords[:, 1] = (IMG_HEIGHT - (coords[:, 1] * scale + PADDING)).astype(int)
        return pixel_coords

    boundary_lines_world = [
        ((0, 0), (0, square_size)), ((0, 0), (square_size, 0)),
        ((0, square_size), (square_size, square_size)),
        ((square_size, y_offset), (world_width, y_offset)),
        ((world_width, y_offset), (world_width, y_offset + rect_height)),
        ((square_size, y_offset + rect_height), (world_width, y_offset + rect_height)),
        ((square_size, y_offset + rect_height), (square_size, world_height)),
        ((square_size, 0), (square_size, y_offset)),
    ]
    boundary_lines_pixels = [
        (world_to_pixel_vectorized(np.array([p1]))[0], world_to_pixel_vectorized(np.array([p2]))[0])
        for p1, p2 in boundary_lines_world
    ]

    writer = None
    if save_as:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(save_as, fourcc, 30, (IMG_WIDTH, IMG_HEIGHT))
        if not writer.isOpened():
            print(f"Error: Could not open video writer for '{save_as}'.")
            return
        
    total_frames = len(frames_data)
    print(f"Processing {total_frames} frames...")
    
    for i, (time, event_pid, particles) in enumerate(frames_data):
        if i % 10 != 0:
            continue

        frame = np.full((IMG_HEIGHT, IMG_WIDTH, 3), BG_COLOR, dtype=np.uint8)

        x_ticks = np.arange(0, 0.18 + 0.001, 0.02)
        for x_val in x_ticks:
            p_world = np.array([[x_val, 0]])
            p_pixel = world_to_pixel_vectorized(p_world)[0]
            cv2.line(frame, (p_pixel[0], p_pixel[1]), (p_pixel[0], p_pixel[1] + 5), AXIS_COLOR, 1, cv2.LINE_AA)
            cv2.putText(frame, f"{x_val:.2f}", (p_pixel[0] - 10, p_pixel[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1, cv2.LINE_AA)

        y_ticks = np.arange(0, 0.09 + 0.001, 0.01)
        for y_val in y_ticks:
            p_world = np.array([[0, y_val]])
            p_pixel = world_to_pixel_vectorized(p_world)[0]
            cv2.line(frame, (p_pixel[0], p_pixel[1]), (p_pixel[0] - 5, p_pixel[1]), AXIS_COLOR, 1, cv2.LINE_AA)
            cv2.putText(frame, f"{y_val:.2f}", (p_pixel[0] - 40, p_pixel[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1, cv2.LINE_AA)

        x_label_pos = world_to_pixel_vectorized(np.array([[world_width / 2, 0]]))[0]
        y_label_pos = world_to_pixel_vectorized(np.array([[0, world_height / 2]]))[0]
        cv2.putText(frame, "X [m]", (x_label_pos[0] - 20, IMG_HEIGHT - (PADDING // 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 2, cv2.LINE_AA)
        cv2.putText(frame, "Y [m]", (PADDING // 4, y_label_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 2, cv2.LINE_AA)
        
        
        for p1, p2 in boundary_lines_pixels:
            cv2.line(frame, tuple(p1), tuple(p2), BOUNDARY_COLOR, 1, lineType=cv2.LINE_AA)

        if particles:
            positions = np.array([[p.x, p.y] for p in particles])
            particle_ids = np.array([p.id for p in particles])
            pixel_coords = world_to_pixel_vectorized(positions)
            is_event = np.isin(particle_ids, event_pid if event_pid else [])
            colors = np.where(is_event[:, np.newaxis], PARTICLE_COLOR, PARTICLE_COLOR)
            for center_px, color in zip(pixel_coords, colors):
                cv2.circle(frame, tuple(center_px), ball_radius_pixels, PARTICLE_BORDER, -1, lineType=cv2.LINE_AA)
                cv2.circle(frame, tuple(center_px), ball_radius_pixels - 1, tuple(color.tolist()), -1, lineType=cv2.LINE_AA)
        
        time_text = f"Time: {time:.3f} s"
        cv2.putText(frame, time_text, (PADDING, PADDING - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 2, cv2.LINE_AA)

        if writer:
            writer.write(frame)
            if not print_data:
                print(f"\r  Frame {i + 1}/{total_frames}", end="")
        else:
            cv2.imshow('Animation', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    if writer:
        writer.release()
        print(f"Animation saved successfully to '{save_as}'!")
    else:
        cv2.destroyAllWindows()