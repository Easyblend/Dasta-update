from PIL import Image, ImageDraw, ImageFilter
import random
import math
import numpy as np
import os


def generate_image(width, height, blob_number=5, noise_amp=0.1,path = "soft\ground station\generated/", img_index = 0):
    # Créer une image en niveau de gris
    image = Image.new("L", (width, height), color=0)
    
    blob = {}

    # Générer 5 ellipses
    for i in range(blob_number):
        # Définir la taille de l'ellipse de manière aléatoire
        ra = random.randint(6, width // 20)  # demi-largeur de l'ellipse
        rb = random.randint(6, height // 20)  # demi-hauteur de l'ellipse
        theta = random.randint(0, 360)/180*math.pi  # angle de rotation de l'ellipse
        rx = ra * rb / ((ra * math.sin(theta)) ** 2 + (rb * math.cos(theta)) ** 2) ** 0.5
        ry = ra * rb / ((ra * math.cos(theta)) ** 2 + (rb * math.sin(theta)) ** 2) ** 0.5
        # Définir la position de l'ellipse de manière aléatoire de manière à ce qu'au moins une partie de l'ellipse soit visible
        rx_int = int(round(rx))
        ry_int = int(round(ry))
        x = random.randint(rx_int, width - rx_int)
        y = random.randint(ry_int, height - ry_int)
        
        brightness = random.randint(50, 255)
        blur = random.randint(0, min(rx, ry) // 3)
        
        # Dessiner l'ellipse avec l'angle de rotation aléatoire dans une nouvelle image 
        ellipse_image = Image.new("L", (2 * rx_int, 2 * ry_int), color=0)
        ellipse_draw = ImageDraw.Draw(ellipse_image)
        ellipse_draw.ellipse((0, 0, 2 * rx_int, 2 * ry_int), fill=brightness)
        ellipse_image = ellipse_image.rotate(theta, expand=True)
        
        # Appliquer le flou à l'ellipse
        ellipse_image = ellipse_image.filter(ImageFilter.GaussianBlur(blur))
        
        

        # ajouter l'ellipse à l'image 
        image.paste(ellipse_image, (x - rx_int, y - ry_int), ellipse_image)
        blob[i] = [x,y, ra, rb, theta, brightness, blur]
        
    #ajouter du bruit blanc
    
    noise = np.random.randint(0, 255, (width, height))
    noise = Image.fromarray(noise)
    noise = noise.convert('L')
    image = Image.blend(image, noise, noise_amp)

    # enregistrer l'image et les paramètres des ellipses en csv
    #dans le dossier "path" avec le nom "img"+numero de l'image

    image.save(path+"/img" + str(img_index) + ".png")
    f = open(path+"/img" + str(img_index) + ".csv", "w")
    
    f.write("x,y,ra,rb,theta,brightness,blur\n")
    for i in range(blob_number):
        f.write(str(blob[i][0]) + "," + str(blob[i][1]) + "," + str(blob[i][2]) + "," + str(blob[i][3]) + "," + str(blob[i][4]) + "," + str(blob[i][5]) + "," + str(blob[i][6]) + "\n")
    f.close()
    

# Spécifier la taille de l'image
width = 480
height = 480

# Générer les images
min_blob = 1
max_blob = 5
same_blob_number = 5 #number of images with the same number of blobs
min_noise = 0
max_noise = 0.2
noise_step_number = 5 #number of different noise amplitudes

blobs_number = np.linspace(min_blob, max_blob, max_blob - min_blob + 1)
noise = np.linspace(min_noise, max_noise, noise_step_number)
#print actual path
print(os.path.abspath(os.getcwd()))
path = "soft/ground station/generated/"
#clear the folder or create it if it doesn't exist
if not os.path.exists(path):
    os.makedirs(path)
for filename in os.listdir(path):
    file_path = os.path.join(path, filename)
    try:
        if os.path.isfile(file_path) or os.path.islink(file_path):
            os.unlink(file_path)
    except Exception as e:
        print('Failed to delete %s. Reason: %s' % (file_path, e))
        

i = 0
for blob_number in blobs_number:
    for _ in range(same_blob_number):
        for noise_amp in noise:
            i += 1
            generate_image(width, height, int(blob_number), noise_amp, img_index = i, path = path)
