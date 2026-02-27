from PIL import Image
import numpy as np
from scipy.ndimage import binary_dilation

# Charger l'image
img = Image.open('image_inversee.png').convert('L')
arr = np.array(img)

# Binariser : True pour le blanc (corridor), False pour le noir (mur)
binary = arr > 128

# SOLUTION : Utiliser une structure plus grande ou plus d'itérations
# Ici, on crée un disque (ou carré) de 5x5 pour dilater plus vite
struct = np.ones((5, 5)) 
# Augmente iterations pour un effet "gros corridors"
dilated = binary_dilation(binary, structure=struct, iterations=20)

# Convertir en image
# (dilated * 255) remet les True à 255 (blanc) et False à 0 (noir)
corridor_img = Image.fromarray((dilated * 255).astype(np.uint8))
corridor_img.save('image_corridor_grossi.png')

print('Expansion terminée : les corridors blancs ont "mangé" les murs noirs.')