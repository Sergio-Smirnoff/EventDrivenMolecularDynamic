import numpy as np
import matplotlib.pyplot as plt
from src.file_reader import leer_header, leer_frames
from scipy.optimize import curve_fit

# Presiones vs Tiempo
# The interval is the time window to compute pressure for each box
def presiones_vs_t(filename, interval=0.8):
    N, L = leer_header(filename)
    frames = list(leer_frames(filename))

    box_A_size = 0.09
    box_B_width = 0.09
    box_B_height = L
    particle_radius = 0.0015

    bottomB = (box_A_size - L) / 2
    topB = (box_A_size + L) / 2

    wall_lengths = {
        "A_left": box_A_size,
        "A_bottom": box_A_size,
        "A_top": box_A_size,
        "A_right_bottom_segment": bottomB,
        "A_right_top_segment": (box_A_size - topB),
        "B_bottom": box_B_width,
        "B_top": box_B_width,
        "B_right": box_B_height
    }

    total_area_A = wall_lengths["A_left"] + wall_lengths["A_bottom"] + wall_lengths["A_top"] + wall_lengths["A_right_bottom_segment"] + wall_lengths["A_right_top_segment"]
    total_area_B = sum(wall_lengths[w] for w in ["B_right", "B_top", "B_bottom"])

    tiempos = [0.0]
    P_A_list = [0.0]
    P_B_list = [0.0]

    current_interval_start = frames[0][0]
    walls = {k: 0 for k in wall_lengths.keys()}

    # Main loop
    for t, p_id, particles in frames:
        while t >= current_interval_start + interval:
        # Calculate pressure for the interval
            total_impulse_A = sum(walls[w] for w in ["A_left", "A_bottom", "A_top", "A_right_bottom_segment", "A_right_top_segment"])
            total_impulse_B = sum(walls[w] for w in ["B_right", "B_top", "B_bottom"])

            P_A = total_impulse_A / (total_area_A * interval)
            P_B = total_impulse_B / (total_area_B * interval)

            tiempos.append(current_interval_start + interval/2)
            P_A_list.append(P_A)
            P_B_list.append(P_B)

            walls = {k: 0 for k in wall_lengths.keys()}
            current_interval_start += interval

        # Decide which wall was collided with
        if p_id is not None:
            p = particles[p_id]
            if p.x <= 0 + particle_radius:
                walls["A_left"] += 2 * abs(p.vx)
            if p.y <= 0 + particle_radius:
                walls["A_bottom"] += 2 * abs(p.vy)
            if p.y >= box_A_size - particle_radius:
                walls["A_top"] += 2 * abs(p.vy)
            if p.x >= box_A_size - particle_radius:
                if p.y <= bottomB + particle_radius:
                    walls["A_right_bottom_segment"] += 2 * abs(p.vx)
                elif p.y >= topB - particle_radius:
                    walls["A_right_top_segment"] += 2 * abs(p.vx)

            if p.x >= box_A_size + box_B_width - particle_radius:
                walls["B_right"] += 2 * abs(p.vx)
            if p.y <= bottomB + particle_radius:
                walls["B_bottom"] += 2 * abs(p.vy)
            if p.y >= topB - particle_radius:
                walls["B_top"] += 2 * abs(p.vy)

    # Clean up loop
    if any(v != 0 for v in walls.values()):
        total_impulse_A = sum(walls[w] for w in ["A_left", "A_bottom", "A_top", "A_right_bottom_segment", "A_right_top_segment"])
        total_impulse_B = sum(walls[w] for w in ["B_right", "B_top", "B_bottom"])

        P_A = total_impulse_A / (total_area_A * interval)
        P_B = total_impulse_B / (total_area_B * interval)

        tiempos.append(current_interval_start + interval/2)
        P_A_list.append(P_A)
        P_B_list.append(P_B)

    return tiempos, P_A_list, P_B_list

def plot_presiones_vs_t(filename, interval=0.8):
    tiempos, P_A_list, P_B_list = presiones_vs_t(filename, interval)

    plt.plot(tiempos, P_A_list, label="Caja A")
    plt.plot(tiempos, P_B_list, label="Caja B")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("Presión [Pa]")
    plt.legend()
    plt.grid()
    plt.show()

    return tiempos, P_A_list, P_B_list

def presion_promedio(P_A_list, P_B_list):
    pA_mean = np.mean(P_A_list)
    pB_mean = np.mean(P_B_list)

    return pA_mean, pB_mean

def plot_presion_vs_L(L_values, P_means_A, P_means_B):
    plt.scatter(L_values, P_means_A, label="Caja A")
    plt.scatter(L_values, P_means_B, label="Caja B")
    plt.xlabel("Longitud L [m]")
    plt.ylabel("Presión promedio [Pa]")
    plt.grid(True)
    plt.legend()
    plt.show()

def presion_vs_area(files):
    P_means, A_inv = [], []

    for fname in files:
        N, L = leer_header(fname)
        
        tiempos, P_A_list, P_B_list = presiones_vs_t(fname)

        pA_mean, pB_mean = presion_promedio(P_A_list, P_B_list)
        
        area_A = 0.09 * 0.09
        area_B = 0.09 * L
        A_total = area_A + area_B
        
        P_mean = 0.5 * (pA_mean + pB_mean)
        P_means.append(P_mean)
        A_inv.append(1.0 / A_total)

    P_means, A_inv = np.array(P_means), np.array(A_inv)

    plt.scatter(A_inv, P_means, label="Datos simulación")
    plt.xlabel("1/Área [1/m²]")
    plt.ylabel("Presión promedio [Pa]")
    plt.grid()
    plt.legend()
    plt.show()

    return A_inv, P_means

def modelo(A_inv, c):
    return c * A_inv

def ajuste_presion_vs_area(A_inv, P_means):
    popt, pcov = curve_fit(modelo, A_inv, P_means)
    c = popt[0]

    A_inv_fit = np.linspace(min(A_inv), max(A_inv), 100)
    P_fit = modelo(A_inv_fit, c)

    plt.scatter(A_inv, P_means, label="Datos simulación")
    plt.plot(A_inv_fit, P_fit, "--", label=f"Ajuste P = c/A, c={c:.3e}")
    plt.xlabel("1/Área [1/m²]")
    plt.ylabel("Presión promedio [Pa]")
    plt.grid()
    plt.legend()
    plt.show()

# Difusión (MSD)
def difusion(filename):
    frames = list(leer_frames(filename))
    pos0 = np.array([[float(p.x), float(p.y)] for p in frames[0][2]])

    msd, tiempos = [], []
    for t, _, particles in frames:
        pos = np.array([[float(p.x), float(p.y)] for p in particles])
        disp = pos - pos0
        sq_disp = np.sum(disp**2, axis=1)
        msd.append(np.mean(sq_disp))
        tiempos.append(float(t))

    tiempos = np.array(tiempos)
    msd = np.array(msd)

    def modelo_diff(t, D):
        return 4 * D * t

    popt, _ = curve_fit(modelo_diff, tiempos, msd)
    D = popt[0]
    print(f"Coeficiente de difusión D ≈ {D:.6e} m²/s")

    plt.plot(tiempos, msd, label="MSD")
    plt.plot(tiempos, modelo_diff(tiempos, D),
             "--", label=f"Ajuste lineal (D={D:.4e})")
    plt.xlabel("Tiempo [s]")
    plt.ylabel("MSD [m²]")
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    
    files = ["./test-data/initial_state_0.03.csv",
    "./test-data/initial_state_0.05.csv",
    "./test-data/initial_state_0.06.csv",
    "./test-data/initial_state_0.07.csv",
    "./test-data/initial_state_0.08.csv",
    "./test-data/initial_state_0.09.csv"]

    for file in files:
        plot_presiones_vs_t(file)

    A_inv, P_means = presion_vs_area(files)
    ajuste_presion_vs_area(A_inv, P_means)

    difusion("./test-data/initial_state_0.05.csv")
    

