import numpy as np

# Parametri intrinseci della telecamera
fx =  554.254691191187  # Sostituisci con il tuo valore
fy =  554.254691191187  # Sostituisci con il tuo valore
cx =  279        # Sostituisci con il tuo valore
cy =  226        # Sostituisci con il tuo valore

# Matrice di trasformazione intrinseca
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0, 0, 1]])

# Coordinate pixel 2D e profondità (z)
pixel_x = 108  # Sostituisci con le tue coordinate pixel
pixel_y = 216
depth =   2.49   # Sostituisci con il tuo valore di profondità

# Aggiungi una colonna di 1 per ottenere le coordinate omogenee
pixel_coordinates = np.array([pixel_x, pixel_y, 1])

# Calcola le coordinate del mondo (omogenee)
world_coordinates_homogeneous = np.linalg.inv(K).dot(pixel_coordinates) * depth

# Normalizza le coordinate omogenee dividendo per l'ultima componente
world_coordinates = world_coordinates_homogeneous

# Stampa le coordinate del mondo
print("Coordinate del mondo:", world_coordinates[:3])
