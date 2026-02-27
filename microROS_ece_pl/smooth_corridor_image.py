from PIL import Image
import numpy as np
from scipy.ndimage import binary_erosion, binary_dilation

# Charge image existante
img = Image.open('image.png').convert('L')
arr = np.array(img)

# Binarise (noir corridor, blanc ailleurs)
corridor = arr < 128

# Applique une dilation puis une érosion pour arrondir les bords
corridor_smoothed = binary_dilation(corridor, structure=np.ones((7,7)))
corridor_smoothed = binary_erosion(corridor_smoothed, structure=np.ones((7,7)))

# Recrée image
arr_out = np.where(corridor_smoothed, 0, 255).astype(np.uint8)
img_out = Image.fromarray(arr_out, mode='L')
img_out.save('image.png')
print('image.png corridor arrondi et lissé.')
