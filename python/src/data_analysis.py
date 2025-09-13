import numpy as np
import matplotlib.pyplot as plt
from src.file_reader import leer_header, leer_frames

def manual_linear_fit(x_data, y_data):
    sum_xy = np.sum(x_data * y_data)
    sum_x_squared = np.sum(x_data**2)
    
    if sum_x_squared == 0:
        return 0, 0
        
    c = sum_xy / sum_x_squared
    
    y_predicted = c * x_data
    residuals = y_data - y_predicted
    n = len(x_data)
    if n > 1:
        error = np.sqrt(np.sum(residuals**2) / (n - 1))
    else:
        error = 0

    return c, error

def modelo_diff(t, D):
    return 4 * D * t

def manual_diffusion_fit(t, msd):
    if len(t) > 0 and t[0] == 0:
        t = t[1:]
        msd = msd[1:]
    
    sum_t_msd = np.sum(t * msd)
    sum_t_squared = np.sum(t**2)
    
    if sum_t_squared == 0:
        return 0, 0

    slope_m = sum_t_msd / sum_t_squared
    D = slope_m / 4.0

    msd_predicted = 4 * D * t
    residuals = msd - msd_predicted
    n = len(t)
    if n > 1:
        rmse = np.sqrt(np.sum(residuals**2) / (n - 1))
    else:
        rmse = 0
        
    return D, rmse

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
    tiempos_A = [0.0]
    tiempos_B = [0.0]
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
            p = particles[p_id[0]]
            if p.x <= box_A_size - particle_radius:
                if p.x <= 0 + particle_radius:
                    walls["A_left"] += 2 * abs(p.vx)
                    tiempos_A.append(t)
                elif p.y <= 0 + particle_radius:
                    walls["A_bottom"] += 2 * abs(p.vy)
                    tiempos_A.append(t)
                elif p.y >= box_A_size - particle_radius:
                    walls["A_top"] += 2 * abs(p.vy)
                    tiempos_A.append(t)
                elif p.x <= box_A_size - particle_radius:
                    if p.y <= bottomB + particle_radius:
                        walls["A_right_bottom_segment"] += 2 * abs(p.vx)
                        tiempos_A.append(t)
                    elif p.y >= topB - particle_radius:
                        walls["A_right_top_segment"] += 2 * abs(p.vx)
                        tiempos_A.append(t)
            elif p.x >= box_A_size + particle_radius:
                if p.x >= box_A_size + box_B_width - particle_radius:
                    walls["B_right"] += 2 * abs(p.vx)
                    tiempos_B.append(t)
                elif p.y <= bottomB + particle_radius:
                    walls["B_bottom"] += 2 * abs(p.vy)
                    tiempos_B.append(t)
                elif p.y >= topB - particle_radius:
                    walls["B_top"] += 2 * abs(p.vy)
                    tiempos_B.append(t)

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
    plt.tight_layout()
    plt.legend()
    plt.show()

    return tiempos, P_A_list, P_B_list

def presion_promedio(P_A_list, P_B_list):
    pA_mean = np.mean(P_A_list)
    pB_mean = np.mean(P_B_list)

    return pA_mean, pB_mean

def plot_presion_vs_L(files):
    L_values, P_means, P_errors = [], [], []

    for fname in files:
        N, L = leer_header(fname)
        tiempos, P_A_list, P_B_list = presiones_vs_t(fname)

        pA_mean, pB_mean = presion_promedio(P_A_list, P_B_list)

        P_total_list = P_A_list + P_B_list
        P_std_dev = np.std(P_total_list)

        L_values.append(L)
        P_mean_ = 0.5 * (pA_mean + pB_mean)
        P_means.append(P_mean_)
        P_errors.append(P_std_dev) 
    plt.figure(figsize=(10, 6))
    plt.errorbar(L_values, P_means, yerr=P_errors, fmt='o', color='blue',
                 ecolor='lightblue', capsize=5, label="Presión Promedio")
    
    plt.xlabel("Longitud L [m]")
    plt.ylabel("Presión promedio [Pa]")
    plt.legend()
    plt.tight_layout()
    plt.show()

    return L_values, P_means, P_errors


def presion_vs_area(files):
    P_means, A_inv, P_errors = [], [], []

    for fname in files:
        N, L = leer_header(fname)
        
        tiempos, P_A_list, P_B_list = presiones_vs_t(fname)

        pA_mean, pB_mean = presion_promedio(P_A_list, P_B_list)
        
        P_total_list = P_A_list + P_B_list
        P_std_dev = np.std(P_total_list)
        
        area_A = 0.09 * 0.09
        area_B = 0.09 * L
        A_total = area_A + area_B
        
        P_mean = 0.5 * (pA_mean + pB_mean)
        
        P_means.append(P_mean)
        A_inv.append(1.0 / A_total)
        P_errors.append(P_std_dev) 

    P_means = np.array(P_means)
    A_inv = np.array(A_inv)
    P_errors = np.array(P_errors)

    plt.figure(figsize=(10, 6))
    plt.errorbar(A_inv, P_means, yerr=P_errors, fmt='o', color='blue', 
                 ecolor='lightblue', capsize=5, label="Datos de simulación")
    
    plt.xlabel("1/Área [1/m²]")
    plt.ylabel("Presión promedio [Pa]")
    plt.tight_layout()
    plt.legend()
    plt.show()

    return A_inv, P_means, P_errors

def modelo(A_inv, c):
    return c * A_inv

def ajuste_presion_vs_area(A_inv, P_means, P_errors):
    c_manual, rmse = manual_linear_fit(A_inv, P_means)

    A_inv_fit = np.linspace(min(A_inv), max(A_inv), 100)
    P_fit_manual = modelo(A_inv_fit, c_manual)

    plt.errorbar(A_inv, P_means, yerr=P_errors, fmt='o', color='blue', 
                 ecolor='lightblue', capsize=5, label="Datos de simulación")
    
    plt.plot(A_inv_fit, P_fit_manual, "--", color='red', 
             label=f"Ajuste Manual: P = c/A\n$c={c_manual:.3e}$ (RMSE={rmse:.3e})")
    
    plt.xlabel("1/Área [1/m²]")
    plt.ylabel("Presión promedio [Pa]")
    plt.tight_layout()
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

    D_manual, rmse = manual_diffusion_fit(tiempos, msd)

    plt.figure(figsize=(10, 6))
    
    plt.scatter(tiempos, msd, s=10, alpha=0.7, label="Datos MSD (Simulación)")
    plt.plot(tiempos, modelo_diff(tiempos, D_manual),
             color="red", linestyle="--", label=f"Ajuste Manual (D={D_manual:.4e})")
    
    plt.xscale('log')
    plt.yscale('log')
    plt.xlabel("Tiempo (s)")
    plt.ylabel("MSD ($m^2$)")
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    files = ["./test-data/initial_state_0.03.csv",
    "./test-data/initial_state_0.05.csv",
    "./test-data/initial_state_0.06.csv",
    "./test-data/initial_state.csv",
    "./test-data/initial_state_0.08.csv",
    "./test-data/initial_state_0.09.csv"]

    for file in files:
        plot_presiones_vs_t(file)

    A_inv, P_means = presion_vs_area(files)
    ajuste_presion_vs_area(A_inv, P_means)

    difusion("./test-data/initial_state_0.05.csv")
    

