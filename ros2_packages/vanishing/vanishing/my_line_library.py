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

def draw_function(img,Y,hmin,hmax,ymin,ymax,color,thickness):

    width=img.shape[1]
    cv2.line(img,(0,ymin),(width,ymin),(255,0,255),2)    
    cv2.line(img,(0,ymax),(width,ymax),(255,0,255),2)
    
    n=len(Y)
    for i in range (n-1):
        cv2.line(img,(i,Y[i]),(i+1,Y[i+1]),color,thickness)
    

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
    #On applique aussi au préalable un filtre sur les intersections prises en compte

    #Les intersections filtrées doivent se situer dans une rectangle défini par (xlim, width-xlim) et (ylim, height-ylim)
    #xlim et ylim étant réglable

    #Nous avons aussi repéré les clusters de lines (avec l'algo de DBSCAN)
    #Puis remplacé un cluster de droites par la moyenne du cluster (voir fonction suivante)

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
            cv2.circle(img, (int(vanishingpoint[0]),int(vanishingpoint[1])), 8, (0,0,255),-1)
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
    lines[...,2]=lines[...,2]/10000 
    #Utiliser ça modif directement copy, d'ou le np.copy
    #on réduit l'impact de l'ordonnée à l'origine pour l'ajuster à a et b d'où le facteur 10000

    n,m=np.shape(segments)

    #On obtient les différents cluster de droites
    #L'avantage de cette algo par rapport au k-means par exemple est de ne pas avoir à préciser le nombre
    #de cluster, seulement une valeur epsilon et le min_samples :
    #nombre minimal de lignes par cluster 
    dbscan = DBSCAN(eps, min_samples=2)
    cluster = dbscan.fit_predict(lines)

    # print("cluster", cluster)

    #On va stocker les segments filtrés ici
    new_segments=[]

    #On définit un dictionnaire avec pour clé un label de cluster 
    #et en value les indices des cores du cluster correspondants
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

        #On trouve un segment qui se trouve sur la droite avec les paramètres de la droite
        x1,y1,x2,y2=segment_on_line(img,a,b,c)

        L=((x1-x2)**2+(y2-y1)**2)**(1/2)
        
        #On définit alors un segment et on l'ajoute à new_segment
        new_segment=[x1,x2,y1,y2,a,b,c,L]

        #print("new_seg",new_segment)
        new_segments.append(new_segment) 

    new_segments = np.array(new_segments, dtype=np.float32)

    return new_segments
    #Ce filtre fait qu'on a moins souvent le vanishing point, mais il est plus précis et stable.

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

def intensity_mesure(img,h):

    #This function returns a mesure of intensity along a given line h on the image
    #It returns a numpy array of dimension width=img.shape[1]

    if len(img.shape)<3:
        return img[h] #Grayscaled
    

    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return gray_image[h]


def local_shift(i1,i2,sigma=50,p=50):

    #We assume i1 and i2 are intensity curves

    n=len(i1)
    shift=np.zeros(n)


    # i1_expanded = i1.reshape(1, -1)  
    # i2_expanded = i2.reshape(1, -1)

    # h_range = np.arange(-p, p)
    # s_range = np.arange(sigma) 


    # diffs = np.array([[np.sum((i1_expanded[0, x + h_range] - i2_expanded[0, x + h_range - s])**2) for s in s_range] for x in range(n)])

    # shift = np.argmin(squared_diff, axis=1)
    
    shiftbis=np.zeros(n)

    #Déplacement vers la droite
    for x in range (sigma+p,n-sigma-p):
        shiftbis[x]=np.argmin([sum([(i1[x+h]-i2[x+h-s])**2 for h in range (-p,p)]) for s in range (0,sigma)])

    # print("shiftbis",shiftbis)
    # print(len(shiftbis),n)
    shiftbis=shiftbis.astype(int)

    return shiftbis
