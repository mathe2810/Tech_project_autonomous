from PIL import Image
import numpy as np

# Charge l'image existante
img = Image.open('image.png').convert('RGB')
arr = np.array(img)

# Seuils pour noir/blanc
black_thresh = 60
white_thresh = 200

# Remplacement des pixels
for y in range(arr.shape[0]):
	for x in range(arr.shape[1]):
		r, g, b = arr[y, x]
		# Si proche de noir
		if r < black_thresh and g < black_thresh and b < black_thresh:
			arr[y, x] = [0, 0, 0]
		# Si proche de blanc
		elif r > white_thresh and g > white_thresh and b > white_thresh:
			arr[y, x] = [255, 255, 255]
		# Sinon, choix le plus proche
		else:
			if (r+g+b)/3 < 128:
				arr[y, x] = [0, 0, 0]
			else:
				arr[y, x] = [255, 255, 255]

# Sauvegarde
img_out = Image.fromarray(arr)
img_out.save('image.png')
print('image.png convertie en noir/blanc pur.')
