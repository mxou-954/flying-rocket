import pandas as pd
import matplotlib.pyplot as plt

# Charger le fichier Excel
data = pd.read_excel("data.xlsx")

# Colonnes
x = data["x"]
y = data["y"]
z = data["z"]

# Création du graphique 3D
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# Trajectoire
ax.plot(x, y, z)

# Labels
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

plt.show()