import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

data = pd.read_csv("../data/traj.csv")

fig = plt.figure(figsize=(18, 12))
fig.suptitle("Simulation M31 LRU", fontsize=11, fontweight='bold')

# 1. Trajectoire 3D interactive
ax1 = fig.add_subplot(231, projection='3d')
ax1.plot(data["x"], data["y"], data["z"], label="réelle", color='blue')
ax1.plot(data["x_theory"], data["y_theory"], data["z_theory"], 
         label="théorique", linestyle='--', color='orange')
ax1.scatter(*data[["x","y","z"]].iloc[-1], color='red', s=50, label="impact")
ax1.set_title("Trajectoire 3D")
ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)"); ax1.set_zlabel("Z (m)")
ax1.legend()

# 3. Drag
ax3 = fig.add_subplot(233)
drag_magnitude = np.sqrt(data["drag_x"]**2 + data["drag_y"]**2 + data["drag_z"]**2)
ax3.plot(data["t"], drag_magnitude, color='brown', label="|drag|")
ax3.plot(data["t"], data["drag_x"], linestyle='--', alpha=0.5, label="drag X")
ax3.plot(data["t"], data["drag_y"], linestyle='--', alpha=0.5, label="drag Y")
ax3.plot(data["t"], data["drag_z"], linestyle='--', alpha=0.5, label="drag Z")
ax3.set_title("Drag (avec vent)")
ax3.set_xlabel("Temps (s)")
ax3.set_ylabel("Accélération (m/s²)")
ax3.legend(); ax3.grid(True)

# 4. Énergie cinétique
ax4 = fig.add_subplot(234)
ax4.plot(data["t"], data["kinetic_energy"] / 1e6, color='red')
ax4.set_title("Énergie cinétique")
ax4.set_xlabel("Temps (s)")
ax4.set_ylabel("Énergie (MJ)")
ax4.grid(True)

# 5. Erreur pos_theory vs réelle
ax5 = fig.add_subplot(235)
ax5.plot(data["t"], data["pos_error"], color='green')
ax5.set_title("Erreur trajectoire")
ax5.set_xlabel("Temps (s)")
ax5.set_ylabel("Écart (m)")
ax5.grid(True)

# 6. Précision finale
ax6 = fig.add_subplot(236)
final = data.iloc[-1]
ecart_x = final["x"] - final["x_theory"]
ecart_y = final["y"] - final["y_theory"]
ax6.add_patch(plt.Circle((0,0), 5, color='red', alpha=0.3, label="5m"))
ax6.add_patch(plt.Circle((0,0), 10, color='orange', alpha=0.2, label="10m"))
ax6.add_patch(plt.Circle((0,0), 50, color='yellow', alpha=0.1, label="50m"))
ax6.scatter(ecart_x, ecart_y, color='blue', s=100, zorder=5, label=f"impact ({ecart_x:.1f}m, {ecart_y:.1f}m)")
ax6.set_xlim(-60, 60); ax6.set_ylim(-60, 60)
ax6.set_aspect('equal')
ax6.set_title("Précision finale")
ax6.set_xlabel("Écart X (m)"); ax6.set_ylabel("Écart Y (m)")
ax6.legend(); ax6.grid(True)

ax7 = fig.add_subplot(232)
ax7.plot(data["t"], data["mach"], color='red')
ax7.set_title("Vitesse du Missile")
ax7.set_xlabel("Temps (s)")
ax7.set_ylabel("Vitesse (Mach)")
ax7.grid(True)

plt.tight_layout()
plt.show()