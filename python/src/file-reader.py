import re
import os
from dataclasses import dataclass

"""  
     * Headers --> N: particlesCount
     *         --> L: heightSecondBox
     * time;pressureA;pressureB;  :: These are not headers, they are values
     * positionX;positionY;velocityX;velocityY  :: These are actual headers
     * .....
     * tm;pAm;PBm
     * positionX;positionY;velocityX;velocityY
     * px0;py0;vx0;vy0
     * ...
     * pxn;pyn;vxn;vyn
     * tm+1;pAm+1;PBm+1
"""

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

hdr_N = re.compile(r"^\s*N\s*:\s*(\d+)")
hdr_L = re.compile(r"^\s*L\s*:\s*(\d+)")

@dataclass
class Particle:
    id: int
    x: float
    y: float
    vx: float
    vy: float

def leer_header(filename):
    N = None
    L = None
    with open(os.path.join(BASE_DIR, filename)) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            m = hdr_N.match(line)
            if m:
                N = int(m.group(1))
            m = hdr_L.match(line)
            if m:
                L = int(m.group(1))
            if N is not None and L is not None:
                break
    return N, L

def leer_frames(filename):
    N, L = leer_header(os.path.join(BASE_DIR, filename))

    with open(os.path.join(BASE_DIR, filename)) as f:
        lines = [l.strip() for l in f if l.strip()]

    i = 0
    while i < len(lines):
        line = lines[i]

        # gotta skip headers :D
        if hdr_N.match(line) or hdr_L.match(line):
            i += 1
            continue

        # Frame start: tm; pAm; pBm
        if line.count(";") >= 2 and not line.startswith("positionX"):
            time, pA, pB = [v for v in line.split(";")[:3]]
            i += 1

            # gotta skip the "positionX;positionY;velocityX;velocityY" row
            if lines[i].startswith("positionX"):
                i += 1

            particles = []
            for pid in range(N):  # read exactly N particles
                px, py, vx, vy = [v for v in lines[i].split(";")]
                particles.append(Particle(pid, px, py, vx, vy))
                i += 1

            yield time, pA, pB, particles
        else:
            i += 1

# This main is meant to test file-reader.py
# I ran the tests, hand checked, we are reading properly!
if "__main__":
    N, L = leer_header("./test-data/initial_state.csv")
    print(f"N={N}, L={L}")

    for t, pA, pB, particles in leer_frames("./test-data/initial_state.csv"):
        print(f"Frame at t={t}, pA={pA}, pB={pB}")
        for p in particles[:5]:
            print("   ", p)
