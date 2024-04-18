import pygame
from pygame.locals import *
import numpy as np
from pyquaternion import Quaternion

# Paramètres de la fenêtre
width, height = 800, 600

def init():
    # Initialisation de Pygame
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Cube en orientation quaternion")
    return screen

# Couleurs
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

dist2cam = 0.5


side = 0.1

cube_vertices = np.array([[-1, -1, -1],
                          [1, -1, -1],
                          [1, 1, -1],
                          [-1, 1, -1],
                          [-1, -1, 1],
                          [1, -1, 1],
                          [1, 1, 1],
                          [-1, 1, 1]]) * side

def project(point_pos,cube_orien,cube_pos,cam_pos,cam_orien,cam_k):
        '''
        project a point from world frame to camera frame
        '''
        # qdcm = quat2dcm(drone_orien[0],-drone_orien[1:4])
        # res = qdcm@led_pos
        # res1 = res + drone_pos - cam_pos
        # qdcm2 = quat2dcm(cam_orien[0],-cam_orien[1:4])
        # res2 = qdcm2@res1
        # res3 = res2[0:2]*cam_k/res2[2]
        # return res3
        #using quaternion
        rotated_point = cube_orien.rotate(point_pos)
        rotated_point += cube_pos - cam_pos
        rotated_point = cam_orien.rotate(rotated_point)
        return rotated_point[0:2]*cam_k/rotated_point[2]
        

def draw_cube(screen,postion, orientation, leds_location_in_img=None, camPos=None, camOri=None, cam_k=300):
    
    if camPos is None:
        camPos = np.array([0,0,-dist2cam])
    if camOri is None:
        camOri = Quaternion(1,0,0,0)
        
    camPos = np.array(camPos)
    camOri = Quaternion(camOri)
        
    # On efface l'écran
    screen.fill(WHITE)
    #on projette les points du cube sur l'écran
    cube_vertices_rotated = [project(v,orientation,postion,camPos,camOri,cam_k) for v in cube_vertices]
    # On dessine les arêtes du cube
    for i, j in [(0, 1), (1, 2), (2, 3), (3, 0),
                    (4, 5), (5, 6), (6, 7), (7, 4),
                    (0, 4), (1, 5), (2, 6), (3, 7)]:
            pygame.draw.line(screen, BLACK,
                            (width / 2 + cube_vertices_rotated[i][0],
                            height / 2 + cube_vertices_rotated[i][1]),
                            (width / 2 + cube_vertices_rotated[j][0],
                            height / 2 + cube_vertices_rotated[j][1])) 
    

    # cube_vertices_rotated = np.array([q.rotate(v) for v in cube_vertices])
    # # On décale le cube pour qu'il soit visible
    # cube_vertices_rotated[:, 2] += dist2cam
    # #on ajoute la position du cube
    # cube_vertices_rotated[:,:] += p
    # On projette les points sur le plan de l'écran
    # cube_vertices_projected = np.array([(x / z, y / z) for x, y, z in cube_vertices_rotated])
    # # On dessine les arêtes du cube
    # for i, j in [(0, 1), (1, 2), (2, 3), (3, 0),
    #              (4, 5), (5, 6), (6, 7), (7, 4),
    #              (0, 4), (1, 5), (2, 6), (3, 7)]:
    #     pygame.draw.line(screen, BLACK,
    #                      (width / 2 + cube_vertices_projected[i][0] * width / 3,
    #                       height / 2 + cube_vertices_projected[i][1] * width / 3),
    #                      (width / 2 + cube_vertices_projected[j][0] * width / 3,
    #                       height / 2 + cube_vertices_projected[j][1] * width / 3))
        
    #on ajoute l'affichage des leds
    # print(leds_location_in_img)
    if leds_location_in_img is not None:
        for shape in leds_location_in_img:
            for led in shape:
                pygame.draw.circle(screen, BLACK, (led[0]+width/2, led[1]+height/2), 5)
        
    #on ajoute l'affichage des matrices de calibration
    # txt = "acc_ortho_correction:"
    # font = pygame.font.Font('freesansbold.ttf', 12)
    # text = font.render(txt, True, BLACK, WHITE)
    # textRect = text.get_rect()
    # textRect.center = (width // 2, height // 2 + 120)
    # screen.blit(text, textRect)
    # for i in range(3):
    #     txt  = np.array2string(X[13+i*3:13+i*3+3].reshape(3), precision=3, separator=',')
    #     text = font.render(txt, True, BLACK, WHITE)
    #     textRect = text.get_rect()
    #     textRect.center = (width // 2, height // 2 + 140 + i*20)
    #     screen.blit(text, textRect)
    
    
    
    
    
        
        
 
def draw_state(screen,state):
    # afficher l'état sur l'écran (avec du texte) state est un np.array de taille 47

    screen.fill(WHITE)
    font = pygame.font.Font('freesansbold.ttf', 12)
    # afficher la position
    text = font.render('position: ' + str(state[0:3]), True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2)
    screen.blit(text, textRect)
    # afficher la vitesse
    text = font.render('vitesse: ' + str(state[3:6]), True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2 + 20)
    screen.blit(text, textRect)
    # afficher l'orientation
    text = font.render('orientation: ' + str(state[6:10]), True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2 + 40)
    screen.blit(text, textRect)
    # afficher le biais d'accélération
    text = font.render('biais acc: ' + str(state[10:13]), True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2 + 60)
    screen.blit(text, textRect)
    # afficher la les gains d'accélération
    text = font.render('gain acc: ' + str(state[[13, 17, 21]]), True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2 + 80)
    screen.blit(text, textRect)
    # afficher la les gains de gyroscope
    text = font.render('gain gyr: ' + str(state[[22, 26, 30]]), True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2 + 100)
    screen.blit(text, textRect)
    
    
    
    
    

    
def draw_Cov(screen,cov_matrix):
    # afficher la matrice de covariance sur l'écran2
    try :
        screen.fill(WHITE)
        # cov_matrix est un np.array carré. calculer la racine carrée de chaque élément pour avoir la matrice d'écart-type
        # cov_matrix = np.sqrt(cov_matrix)
        # afficher la matrice d'écart-type sur l'écran2 avec une echelle de couleur
        # pour chaque élément de la matrice
        m = np.max(cov_matrix)
        for i in range(cov_matrix.shape[0]):
            for j in range(cov_matrix.shape[1]):
                # calculer la couleur de l'élément
                color = int(cov_matrix[i, j] / m * 100)
                # afficher le pixel correspondant à l'élément
                pygame.draw.rect(screen, (color, color, color), (i*10, j*10, 10, 10))
        
    except Exception as e:
        pass
    
if __name__ == '__main__':
    screen = init()
    # On crée un quaternion unitaire
    q = Quaternion()
    # On crée une horloge pour contrôler la vitesse de rotation
    clock = pygame.time.Clock()
    # Boucle de contrôle
    while True:
        # On récupère les entrées utilisateur
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                exit()
        # On calcule le quaternion de rotation
        q = Quaternion(axis=[1, 0, 0], angle=np.pi / 100) * q
        # On dessine le cube
        draw_cube(screen,np.array([0,0,0]),q)
        # On affiche le résultat
        pygame.display.flip()
        # On attend 10 ms avant de recommencer
        clock.tick(100)