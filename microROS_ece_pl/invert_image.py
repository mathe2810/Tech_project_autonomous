from PIL import Image

# Charger l'image
img = Image.open('image.png').convert('L')  # 'L' pour niveau de gris

# Inverser les pixels
img_inv = Image.eval(img, lambda x: 255 - x)

# Sauvegarder l'image inversée
img_inv.save('image_inversee.png')
print('Image inversée sauvegardée sous image_inversee.png')
