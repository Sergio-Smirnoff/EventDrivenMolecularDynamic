import re
from dataclasses import dataclass

hdr_N = re.compile(r"^\s*N\s*:\s*(\d+)\s*$", re.IGNORECASE)
hdr_L = re.compile(r"^\s*L\s*:\s*([+-]?\d+(?:\.\d+)?)\s*$", re.IGNORECASE)
hdr_cols = re.compile(
    r"^\s*positionX\s*;\s*positionY\s*;\s*velocityX\s*;\s*velocityY\s*$",
    re.IGNORECASE,
)


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
                L = float(m.group(1))/100000.0
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
    while i < len(lines) and ";" not in lines[i]:
        i += 1

    while i < len(lines):
        time_line = lines[i]
        parts = [p for p in time_line.split(";") if p != ""]
        if not parts:
            i += 1
            continue

        try:
            t = float(parts[0])
        except ValueError:
            i += 1
            continue

        event_pid = int(parts[1]) if len(parts) > 1 else None
        i += 1

        if i < len(lines) and hdr_cols.match(lines[i]):
            i += 1

        particles = []
        for pid in range(N):
            if i >= len(lines):
                raise ValueError("Fin inesperado del archivo al leer partículas.")
            parts = lines[i].split(";")
            if len(parts) < 4:
                raise ValueError(f"Línea inválida de partícula: '{lines[i]}'")
            px, py, vx, vy = [float(x)/100000 for x in parts[:4]]
            particles.append(Particle(pid, px, py, vx, vy))
            i += 1

        yield t, event_pid, particles