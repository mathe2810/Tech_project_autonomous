from PIL import Image

# Paramètres
input_path = 'image.png'  # Chemin de l'image à agrandir
output_path = 'image.png'  # Chemin de sortie
scale_factor = 3  # Facteur d'agrandissement (ex: 1.5 = +50%)

# Chargement de l'image
img = Image.open(input_path)

# Calcul des nouvelles dimensions
new_size = (int(img.width * scale_factor), int(img.height * scale_factor))

# Redimensionnement
img_resized = img.resize(new_size, resample=Image.NEAREST)

# Sauvegarde
img_resized.save(output_path)
print(f"Image redimensionnée enregistrée sous {output_path} (facteur {scale_factor})")
