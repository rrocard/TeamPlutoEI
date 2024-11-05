import cv2
import numpy as np

def from_lsd(lines_lsd):
    """
    lines_lsd : the weird output of cv line segment detector.
    returns : [[x1, y1, x2, y2, a, b, c, L],
               [...                       ],
               ...
              ]

              where 
                - [(x1, y1), (x2, y2)] is the segment (in image coordinates).
                - ax + by + c = 0 is the line equation
                - (a, b) is the normal vector of the line
                - |(a, b)| = 1, b >= 0
                - L is the length of the segment.
    """
    if lines_lsd is None :
        return np.array([]).reshape((0,8))

    nlines = lines_lsd.shape[0]
    segments = None

    if nlines == 0 :
        return np.array([]).reshape((0,8))

    if nlines == 1 :
        segments = lines_lsd.squeeze().reshape((1,4))
    else :
        segments = lines_lsd.squeeze()

    x1 = segments[:, 0]
    y1 = segments[:, 1]
    x2 = segments[:, 2]
    y2 = segments[:, 3]
    b = x1 - x2
    a = y2 - y1
    c = -(a * x1 + b * y1)

    # Ensure that the normal vectors are always
    # pointing toward positive y
    where_b_is_neg = b < 0
    c[where_b_is_neg] *= -1.0
    b[where_b_is_neg] *= -1.0
    a[where_b_is_neg] *= -1.0

    # The length of the segment
    norm = np.sqrt(a**2 + b**2)
    return np.stack([x1, y1, x2, y2, a/norm, b/norm, c/norm, norm], axis=1)
def draw_segments(img, segments, color, thickness):
    '''
        img : cv2 image
        segments : np.array(num_lines, 8)
        color  : 3 element tuple
        thickness : int
    '''
    for s in segments:
        cv2.line(img, (s[0], s[1]), (s[2], s[3]), color, thickness)


def draw_lines(img, segments, color, thickness):
    '''
        img : cv2 image
        segments : np.array(num_lines, 8)
        color  : 3 element tuple
        thickness : int
    '''

    for s in segments:
        a, b, c = s[4:7]
        # If the line is horizontal
        if a == 0:
            cv2.line(img, (0, -int(c/b)), (img.shape[1]-1, -int((c+a*(img.shape[1]-1))/b)), color, thickness)
        # If the line is vertical
        elif b == 0:
            cv2.line(img, (-int(c/a), 0), (-int(c/a), img.shape[0] - 1), color, thickness)
        else:
            # We look for the intersection point with the horizontal axis y=0 and y=height-1
            y = 0
            x = -c/a   #-(c - b*y)/a
            if x < 0:
                x = 0
                y = -(c + a * x) / b
            elif x >= img.shape[1]:
                x = img.shape[1] - 1
                y = -(c + a * x) / b
            first_point = (int(x), int(y))

            y = img.shape[0] - 1
            x = -(c+b*y)/a
            if x < 0:
                x = 0
                y = -(c + a * x) / b
            elif x >= img.shape[1]:
                x = img.shape[1] - 1
                y = -(c + a * x) / b
            second_point = (int(x), int(y))

            cv2.line(img, first_point, second_point, color, thickness)


def intersections(segments):
    intersections = []
    for i, si in enumerate(segments):
        for sj in segments[i+1:]:
            cross_product = np.cross(si[4:6], sj[4:6]) # [a1,b1] ^ [a2, b2]
            if cross_product != 0:
                coeff = 1.0 / cross_product

                intersections.append([coeff * np.cross(si[5:7]   , sj[5:7]), # [b1, c1] ^ [b2, c2]
                                      coeff * np.cross(sj[[4, 6]], si[[4, 6]])]) # -[a1, c1] ^ [a2, c2]
    return np.array(intersections)

def vanishing_point(img,segments,xlim=40,ylim=120):

    width=img.shape[1]
    height=img.shape[0]

    intersect=intersections(segments)
    
    #Pour le calcul du vanishing point : on prend la moyenne des intersections si il y en a plusieurs
    #On applique aussi un filtre sur les intersections prises en compte

    #Les intersections filtrées doivent se situer dans une rectangle défini par (xlim, width-xlim) et (ylim, height-ylim)
    #xlim et ylim étant réglable

    #On a aussi envisagé : de prendre qu'en compte seulement des intersections proches l'un de l'autre 
    #afin de d'éliminer des intersections parasites. 
    #On aurait alors utilisé des méthodes de machine learning pour répérer les clusters de points

    #On a aussi filtrer les courbes verticales et les courbes similaires.

    #Pour les courbes similaires nous avons repéré les clusters (avec l'algo de DBSCAN)
    #Puis remplacé un cluster de droites par la moyenne du cluster
    #On ne pouvait pas simplement appliquer un mask comme pour les autres filtres malheureusement
    #Car il fallait remplacer par une droite les clusters non les supprimés,
    #Car on travail sur la norm entre chacune des droites

    #Nous aurions pu aussi essayer de garder en mémoire l'historique du vanishing point
    #et imposer une forme de continuité. On n'accepterait que des petits déplacements.
    #le problème serait de définir ce "petit" et de parer au cas où dès le début le vanishing point est décalé

    # print(f"intersect1: {intersect}")

    if len(intersect)!=0 :

   
        # print(np.logical_and(np.logical_and(intersect[...,0]>xlim,intersect[...,0]<width-xlim),
        #                 np.logical_and(intersect[...,1]>ylim,intersect[...,1]<height-ylim)))

        # print(np.logical_and(intersect[...,1]>ylim,intersect[...,1]<height-ylim))
     
        # print(np.logical_and(intersect[...,0]>xlim,intersect[...,0]<width-xlim))

        # print(xlim,ylim,width,height)

        # print(intersect[...,1], intersect[...,0])

        intersect=intersect[np.logical_and(np.logical_and(intersect[...,0]>xlim,intersect[...,0]<width-xlim),
                        np.logical_and(intersect[...,1]>ylim,intersect[...,1]<height-ylim))]
        
        # print(f"intersect2: {intersect}")

        vanishingpoint=np.mean(intersect,axis=0)

        if len(intersect)!=0 :
            #cv2.circle(img, (int(vanishingpoint[0]),int(vanishingpoint[1])), 10, (0,0,255),-1)
            #pour montrer le vanishing point (la fonction a été modif pour ne plus avoir en argument l'image)
            return vanishingpoint

    return [width//2,height//2] #valeur par défaut du vanishing point

from sklearn.cluster import DBSCAN

def cluster_filtering(img,segments,eps=0.1):

    #print("segments base",segments)

    #If there is not much lines, no need for the algorithm
    if len(segments)<=2:
        return segments

    copy=np.copy(segments)
    lines=copy[...,4:7]
    lines[...,2]=lines[...,2]/10000 #Utiliser ça modif directement copy, d'ou le np.copy
    #on réduit l'impact de l'ordonnée à l'origine pour l'ajuster à a et b
    n,m=np.shape(segments)

    #We get the norm between each row in a matrix (I sadly could not find a way to vectorize this)
    #We might not need this
    # norm=np.zeros((n,n))
    # for i in range(n):
    #     for j in range(i,n):
    #         norm[i,j]=np.linalg.norm(lines[i]-lines[j])
    #         norm[j,i]=norm[i,j]
    # print("norm", norm)

    dbscan = DBSCAN(eps, min_samples=2)
    cluster = dbscan.fit_predict(lines)

    # print("cluster", cluster)

    new_segments=[]

    #On définit un dictionnaire avec pour clé un label de cluster 
    #et en value les indices des cores du cluster correspondants

    #Je voyais pas comment réduire le nombre de if ici
    d={}
    #Dictionnaire avec les indices des droites de même cluster
    for i in range (n):
        key=cluster[i]
        if key != -1:
            if key in dbscan.core_sample_indices_:
                if key in d:
                    d[key].append(i)
                else :
                    d[key]=[i]
        else :
            #Lines that aren't in a cluster are taken into account
            new_segments.append(segments[i])
    
    # print("intermediate seg", new_segments, "dict", d)


    for label in d :

        #On crée un array avec toutes les droites core d'un cluster
        cluster_lines=np.array([lines[i] for i in d[label]])
        #print("clusterseg",cluster_lines)
        #On calcule la moyenne
        new_line=np.mean(cluster_lines,axis=0)

        #On a les coefficients de la droite ainsi
        a,b,c=new_line[0],new_line[1],new_line[2]*10000

        #print("abc",a,b,c)

        #On trouve un segment de la droite qui rentre sur la fenêtre
        x1,y1,x2,y2=segment_on_line(img,a,b,c)

        L=((x1-x2)**2+(y2-y1)**2)**(1/2)
        
        #On définit alors un segment et on l'ajoute à new_segment
        new_segment=[x1,x2,y1,y2,a,b,c,L]

        #print("new_seg",new_segment)
        new_segments.append(new_segment) 

    #l=len(new_segments)
    #Le reshape est nécessaire car sinon on a un vecteur renvoyé non une matrice à cause des appends
    #segments=new_segments.reshape(l//m,m)

    new_segments = np.array(new_segments, dtype=np.float32)

    return new_segments
    #Ce filtre fait qu'on a moins souvent le vanishing point, mais il est plus précis et stable

import random as rd

def segment_on_line(img,a,b,c):

    #crée un segment appartenant à une droite de paramètres a,b,c

    assert b!=0
    width=img.shape[1]
    height=img.shape[0]

    segment=[]

    while len(segment)<4:
        x=rd.uniform(0,width)
        y=-(a*x+c)/b
        if 0<=y<=height:
            segment.append(x)
            segment.append(y)

    return segment[0],segment[1],segment[2],segment[3]

def length_filtering(segments,min_length=90):

    segments = segments[segments[...,7] > min_length]
    return segments

def angle_filtering(segments,upper_angle=80,lower_angle=10):

    ab = segments[..., 4:6] * np.array([1, -1]) 
    scal = np.matmul(ab, np.array([[0],[1]])).reshape(-1)
    filter = np.abs(np.arccos(np.abs(scal)))*180/np.pi
    boolfilter = (filter > lower_angle) & (filter < upper_angle)
    segments = segments[boolfilter]
    return segments

def ceiling_filtering(segments,ceiling=160):

    segments = segments[np.logical_and(segments[..., 1] > ceiling, segments[..., 3] > ceiling)]
    return segments


