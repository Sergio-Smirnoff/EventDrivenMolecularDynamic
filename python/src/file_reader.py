# src/file_reader.py
import re
from dataclasses import dataclass

hdr_N = re.compile(r"^\s*N\s*:\s*(\d+)\s*$", re.IGNORECASE)
hdr_L = re.compile(r"^\s*L\s*:\s*([+-]?\d+(?:\.\d+)?)\s*$", re.IGNORECASE)
hdr_cols = re.compile(r"^\s*positionX\s*;\s*positionY\s*;\s*velocityX\s*;\s*velocityY\s*$", re.IGNORECASE)
hdr_time = re.compile(r"^\s*([^;]+)\s*;\s*$")

@dataclass
class Particle:
    id: int
    x: float
    y: float
    vx: float
    vy: float

def leer_header(path: str):
    N, L = None, None
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            m = hdr_N.match(s)
            if m:
                N = int(m.group(1))
                continue
            m = hdr_L.match(s)
            if m:
                L = float(m.group(1))
                continue
            if N is not None and L is not None:
                break
    if N is None or L is None:
        raise ValueError("No se pudieron leer correctamente los headers N y L.")
    return N, L

def leer_frames(path: str):
    N, L = leer_header(path)

    with open(path, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f if ln.strip()]

    i = 0
    # saltar hasta el primer "tm;"
    while i < len(lines) and not hdr_time.match(lines[i]):
        i += 1

    while i < len(lines):
        m_time = hdr_time.match(lines[i])
        if not m_time:
            i += 1
            continue

        t_str = m_time.group(1).strip()
        try:
            t = float(t_str)
        except ValueError:
            t = t_str
        i += 1

        # saltar encabezado de columnas si aparece
        if i < len(lines) and hdr_cols.match(lines[i]):
            i += 1

        particles = []
        for pid in range(N):
            if i >= len(lines):
                raise ValueError("Fin inesperado del archivo al leer partículas.")
            parts = lines[i].split(";")
            if len(parts) < 4:
                raise ValueError(f"Línea inválida de partícula: '{lines[i]}'")
            px, py, vx, vy = map(float, parts[:4])
            particles.append(Particle(pid, px, py, vx, vy))
            i += 1

        yield t, particles
