import numpy as np
import matplotlib.pyplot as plt

# Başlangıç pozisyonu ve yön (yukarı)
pos = np.array([0, 0])
direction = np.array([0, 1])  # Başta yukarı (pozitif y)

# Grid boyutu ve hücre boyutu
cell_size = 0.6  # metre
positions = [pos.copy()]

# Gelen komutlar (örnek mesaj listesi)
commands = [
    {"command": 1, "move": 1, "turn": 0},
    {"command": 2, "move": 0, "turn": 2},
    {"command": 1, "move": 2, "turn": 0},
    {"command": 2, "move": 0, "turn": 1},
    {"command": 1, "move": 1, "turn": 0},
    {"command": 2, "move": 0, "turn": 2},
    {"command": 1, "move": 2, "turn": 0},
    {"command": 2, "move": 0, "turn": 1},
    {"command": 1, "move": 1, "turn": 0},
]

# Yön dönüş fonksiyonu
def rotate(dir_vector, turn):
    if turn == 1:  # sağa dönüş
        rot_matrix = np.array([[0, 1], [-1, 0]])
    elif turn == 2:  # sola dönüş
        rot_matrix = np.array([[0, -1], [1, 0]])
    else:
        return dir_vector
    return rot_matrix @ dir_vector

# Komutları işleyerek pozisyonları listele
for cmd in commands:
    if cmd["command"] == 1:
        for _ in range(cmd["move"]):
            pos += direction
            positions.append(pos.copy())
    elif cmd["command"] == 2:
        direction = rotate(direction, cmd["turn"])

# Çizim
positions = np.array(positions)
plt.figure(figsize=(8, 8))
plt.plot(positions[:, 0] * cell_size, positions[:, 1] * cell_size, marker='o', linestyle='-')
plt.grid(True)
plt.xticks(np.arange(0, 5 * cell_size, cell_size))
plt.yticks(np.arange(0, 5 * cell_size, cell_size))
plt.title("Robotun İzlediği Yol")
plt.xlabel("X (metre)")
plt.ylabel("Y (metre)")
plt.axis('equal')
plt.show()
