import cv2
import numpy as np
from skimage.morphology import skeletonize
import time

#structure "entity" pour stocker des informations sur un segment de l'image
class Entity:
    def __init__(self, centoid, endPoints, distances, skeletonLength):
        self.centoid = centoid # centre de l'entité
        self.endPoints = endPoints #liste des extrémités du squelette de l'entité
        self.distances = distances #liste des distances des extrémités au bord le plus proche
        self.skeletonLength = skeletonLength #longueur du squelette de l'entité
        
    def chosePoints(self, distanceThreshold=4):
        #si le squelette n'a pas d'extrémité (donc pas de endPoints), on retourne le centre de l'entité, cela arrive pour les objets parfaitement ronds
        if len(self.endPoints)==0:
            return [[self.centoid[0],self.centoid[1]]]
                
        #si la longueur du squelette est plus grande que distanceThreshold fois la distance maximale à chaque extrémité, ça veut dire que l'entité est longue, donc on retourne les extrémités
        if (self.skeletonLength>distanceThreshold*np.max(self.distances)):
            return self.endPoints
        else:
            return [[self.centoid[0],self.centoid[1]]]#sinon on retourne le centre de l'entité car c'est probablement un objet plus ou moins rond
        
    def __str__(self):
        return f"Centoid: {self.centoid}, EndPoints: {self.endPoints}, Distances: {self.distances}, SkeletonLength: {self.skeletonLength}"
        
def scanEntities(labels):
    """
    Cette fonction prend en entrée une image de labels et retourne une liste d'entités, chaque entité contient des informations sur un segment de l'image
    
    labels: np.array, image de labels -> chaque pixel de l'image est un entier qui correspond à un label, les pixels qui ont le même label font partie du même segment
    
    on retourne une liste d'entités, chaque entité contient:
    - le centoid du segment
    - les endPoints du squelette du segment
    - les distances transformées aux endPoints
    - la longueur du squelette
    """
    n_entities = np.max(labels) #nombre de labels
    entities = [] #liste d'entités à retourner à la fin
    kernel = np.array([[1,1,1],[1,10,1],[1,1,1]]) #kernel pour trouver les endPoints du squelette
    
    for i in range(1,n_entities+1):
        
        #séparer le label i du reste de l'image
        mask = np.zeros(labels.shape,dtype=np.uint8)
        mask[labels==i] = 1
        
        #calculer le centoid du label
        M = cv2.moments(mask)
        if M["m00"] == 0:
            continue
        centoid = (M["m01"] / M["m00"],M["m10"] / M["m00"])
        
        #squelettiser le label, le resultat est une image binaire
        skeleton = np.uint8(skeletonize(mask))
        
        #convolution du squelette avec le kernel, les extrémités (une seule connection) sont les pixels qui ont exactement une valeur de 11 après la convolution
        endPoints = np.uint16(np.argwhere(cv2.filter2D(skeleton,-1,kernel)==11))
        
        #calculer la longueur du squelette
        skeletonLength = np.sum(skeleton)
        
        #calculer la transformée de distance pour ce label -> la distance transformée est la distance de chaque pixel au bord le plus proche
        distTransorm = cv2.distanceTransform(mask,cv2.DIST_L2,5)
        
        #on ne garde que les distances aux endPoints -> si seulement on pouvait ne calculer la transformée de distance que pour les endPoints, ça serait plus rapide
        distances = distTransorm[endPoints[:,0],endPoints[:,1]]
        
        #on crée une nouvelle entité et on l'ajoute à la liste
        entities.append(Entity(centoid, endPoints, distances, skeletonLength))
        
    return entities
        
def preProcess(grayScale, expectedEntities, threshold=80):
    #normaliser l'image permet d'aussi bien traiter les images dans lesquelles il y a beaucoup de lumière que celles dans lesquelles il y en a peu -> attention au bruit et à l'environnement qui doit rester moins lumineux que les leds
    normalized = np.uint8(np.round(255.0/np.float64(np.max(grayScale))*np.float32(grayScale)))
    
    # on seuilise l'image pour ne garder que les objets qui sont les plus lumineux
    thresholded = cv2.threshold(normalized,threshold,255,cv2.THRESH_BINARY)[1] #si besoin pour de meilleurs résultats, il faudra utiliser une seuil à hystérésis plutôt qu'un seuil simple, mais pour l'instant ça marche bien avec un seuil de 110

    #on sépare les objets qui ne se touchent pas
    ret, labels = cv2.connectedComponents(np.uint8(thresholded))
    
    n_labels = np.max(labels)
    
    #si il y a trop de labels, c'est peut-être parce qu'une led apparait en plusieurs morceaux, on peut essayer de dilater l'image pour refermer les petits trous
    if np.max(labels)>expectedEntities:
        kernel = np.ones((5,5),np.uint8)
        dilated = cv2.dilate(thresholded,kernel,iterations = 1)
        ret, labels = cv2.connectedComponents(np.uint8(dilated))
        
    return labels    

def process(grayScale,expectedEntities):
    #on prétraite l'image pour en extraire les différents objets
    labels = preProcess(grayScale,expectedEntities)
    
    #on compte le nombre d'objets
    n_labels = np.max(labels)
    
    # s'il y a trop d'objet, c'est probablement du bruit
    if n_labels>expectedEntities:
        return []
        
    #mais sinon on cherche à décrire les entités, on cherche à carractériser leur forme pour proposer des positions des leds
    entities = scanEntities(labels)
    
    #on retourne les points choisi des entités dans une liste -> c'est ce qu'on va envoyer en bluetooth
    return [entity.chosePoints() for entity in entities]
    
   
ledNumber = 2 #nombre de leds attendues


def init(camera_index=0):
    print("opening camera at index",camera_index)
    cap = cv2.VideoCapture(camera_index)
    # set to 480p
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # cap.set(cv2.CAP_PROP_EXPOSURE, -4)

    
    return cap

lastFrame = None
def read(cap):
    ret,originalFrame = cap.read()
    if ret is False:
        return None
    if originalFrame is None:
        return None
    global lastFrame
    if lastFrame is None:
        lastFrame = originalFrame
        return originalFrame
    diff = cv2.absdiff(originalFrame, lastFrame)
    if np.sum(diff)==0:
        return None
    
    # #normalisation de l'image
    lastFrame = originalFrame
    return originalFrame


def main(cap):
    # ret,originalFrame = cap.read()
    originalFrame = read(cap)
    
    if originalFrame is None:
        return None, None
    
    #tout le traitement se fait en niveau de gris
    gray = cv2.cvtColor(originalFrame, cv2.COLOR_BGR2GRAY)
    
    #on traite l'image
    entity_points = process(gray, expectedEntities=ledNumber)
    
    
    return entity_points, originalFrame
    
    
if __name__ == "__main__":
    cap = init()
    
    while(cap.isOpened()):
        
        entity_points, originalFrame = main(cap)
        if originalFrame is None:
            continue
                
        #pour l'affichage, on veut une image plus grande et en couleur (originalFrame l'est déjà)
        # frame2show = cv2.resize(originalFrame,(originalFrame.shape[1]*2,originalFrame.shape[0]*2))   
        frame2show=originalFrame.copy()
        #on ajoute le nombre de d'objets trouvés sur l'image
        cv2.putText(frame2show,str(len(entity_points))+ "/"+str(ledNumber),(50,50), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
        #et on les ajoute sur l'image
        for segment in entity_points:
            for point in segment:
                cv2.circle(frame2show,(round(point[1]),round(point[0])), 5, (0,0,255), -1)
                
        #puis on affiche l'image
        cv2.imshow('frame',frame2show)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break