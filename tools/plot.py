import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

data = pd.read_csv("../data/traj.csv")

x = data["x"]
y = data["y"]
z = data["z"]
x_th = data["x_theory"]
y_th = data["y_theory"]
z_th = data["z_theory"]

fig = plt.figure(figsize=(14, 5))

# Trajectoire 3D
ax1 = fig.add_subplot(121, projection='3d')
ax1.plot(x, y, z, label="réelle", color='blue')
ax1.plot(x_th, y_th, z_th, label="théorique", color='orange', linestyle='--')
ax1.scatter(300, 25, 100, color='red', s=50, label="cible")
ax1.set_xlabel("X")
ax1.set_ylabel("Y")
ax1.set_zlabel("Z")
ax1.set_title("Trajectoire 3D")
ax1.legend()

# Poussée au fil du temps
ax2 = fig.add_subplot(122)
ax2.plot(data["t"], data["a_thrust_x"], label="thrust X")
ax2.plot(data["t"], data["a_thrust_y"], label="thrust Y")
ax2.plot(data["t"], data["a_thrust_z"], label="thrust Z")
ax2.set_xlabel("Temps (s)")
ax2.set_ylabel("Accélération (m/s²)")
ax2.set_title("Poussée au fil du temps")
ax2.legend()

plt.tight_layout()
plt.show()