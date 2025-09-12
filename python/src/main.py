import tkinter as tk
from tkinter import filedialog, messagebox
from src import data_analysis
from src.animation import animate

class PressureDiffusionApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Presión y Difusión")
        self.geometry("500x350")

        self.files = []

        self.file_label = tk.Label(self, text="No files selected")
        self.file_label.pack(pady=10)

        self.select_btn = tk.Button(self, text="Seleccionar archivos CSV", command=self.select_files)
        self.select_btn.pack(pady=5)

        interval_frame = tk.Frame(self)
        interval_frame.pack(pady=10)
        tk.Label(interval_frame, text="Intervalo (s): ").pack(side=tk.LEFT)
        self.interval_entry = tk.Entry(interval_frame)
        self.interval_entry.insert(0, "0.8")
        self.interval_entry.pack(side=tk.LEFT)

        self.pressure_btn = tk.Button(self, text="Graficar presión vs tiempo", command=self.run_pressure)
        self.pressure_btn.pack(pady=5)

        self.area_btn = tk.Button(self, text="Presión promedio vs área", command=self.run_pressure_vs_area)
        self.area_btn.pack(pady=5)

        self.area_btn = tk.Button(self, text="Presión promedio vs L", command=self.run_pressure_vs_L)
        self.area_btn.pack(pady=5)

        self.diffusion_btn = tk.Button(self, text="Calcular difusión (MSD)", command=self.run_diffusion)
        self.diffusion_btn.pack(pady=5)

        self.animation_btn = tk.Button(self, text="Ejecutar animación", command=self.run_animation)
        self.animation_btn.pack(pady=10)

    def select_files(self):
        self.files = filedialog.askopenfilenames(filetypes=[("CSV Files", "*.csv")])
        if self.files:
            self.file_label.config(text=f"{len(self.files)} archivos seleccionados")
        else:
            self.file_label.config(text="No files selected")

    def run_pressure(self):
        if not self.files:
            messagebox.showwarning("Error", "Selecciona primero los archivos CSV")
            return

        try:
            interval = float(self.interval_entry.get())
        except ValueError:
            messagebox.showwarning("Error", "Intervalo inválido")
            return

        for file in self.files:
            data_analysis.plot_presiones_vs_t(file, interval=interval)

    def run_pressure_vs_area(self):
        if not self.files:
            messagebox.showwarning("Error", "Selecciona primero los archivos CSV")
            return

        A_inv, P_means, P_errors = data_analysis.presion_vs_area(self.files)
        data_analysis.ajuste_presion_vs_area(A_inv, P_means, P_errors)

    def run_pressure_vs_L(self):
        if not self.files:
            messagebox.showwarning("Error", "Selecciona primero los archivos CSV")
            return
        data_analysis.plot_presion_vs_L(self.files)


    def run_diffusion(self):
        if not self.files:
            messagebox.showwarning("Error", "Selecciona primero los archivos CSV")
            return

        for file in self.files:
            data_analysis.difusion(file)

    def run_animation(self):
        if not self.files:
            messagebox.showwarning("Error", "Selecciona primero un archivo CSV")
            return

        for file in self.files:
            animate(file, save_as="animation.mp4")

if __name__ == "__main__":
    app = PressureDiffusionApp()
    app.mainloop()