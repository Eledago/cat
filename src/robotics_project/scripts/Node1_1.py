#########-----------NODO CHE SALVA SU FILE TXT I PRIMI 8 VALORI (Vertici)-------------########

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Header

global R, camera_coord, u0, v0, f, epsilon, flag
#epsilon= 0.006

R = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
camera_coord = np.array([0.547619, -0.064144, 2])
u0 = 320
v0 = 320
f = 554
z = 0.982
flag= False
# Aggiunta di questa variabile globale per tenere traccia del numero di punti salvati
num_points_saved = 0

def camera_callback(image_msg, contour_pub):
    global num_points_saved, flag
    
    rospy.sleep(1)
    
    if flag == False:
        flag = True
    else: 
        rospy.signal_shutdown("Termina il nodo")
        return
    
    
    

    try:
        # Converti il messaggio di immagine ROS in un'immagine OpenCV
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr("Errore nella conversione dell'immagine: %s", str(e))
        return

    # Metti il tuo codice per l'elaborazione dell'immagine qui
    immagine_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Definisci il range di colori per il giallo
    colore_basso_giallo = np.array([20, 100, 100])
    colore_alto_giallo = np.array([30, 255, 255])
    
    # Crea una maschera binaria per il giallo
    maschera_giallo = cv2.inRange(immagine_hsv, colore_basso_giallo, colore_alto_giallo)
    
    # Filtraggio immagine
    kernel = np.ones((5,5), np.uint8)
    maschera_giallo = cv2.morphologyEx(maschera_giallo, cv2.MORPH_OPEN, kernel)
    
    # Algoritmo Shi-Tomasi
    corners = cv2.goodFeaturesToTrack(maschera_giallo, 8, 0.01, 10)
    
    # Converto i punti in interi
    corners = np.intp(corners)
    
    # Estrai i punti di contorno e pubblicali come Point
    contour_points = []  # Inizializza la lista una sola volta
    print(corners)
    for point in corners:
            x, y = point.ravel()
            
            # cv2.imwrite("/home/eleonora/frame_salvato.jpg", frame)
            
            # cv2.circle(frame, (x,y), 3, 255, -1)

            # TRASFORMAZIONE PIXEL e COORDINATE MONDO
            x_c = ((x - u0) / f) * z
            y_c = ((y - v0) / f) * z
            X_c = np.array([x_c, y_c, z])
            X_r = np.dot(R, X_c)
            X_w = X_r + camera_coord
            #X_w[1]= X_w[1]-epsilon

            # Pubblica ogni punto come messaggio di tipo Point
            contour_point = Point(x=X_w[0], y=X_w[1], z=X_w[2])
            contour_points.append(contour_point)

    # Pubblica i punti di contorno
    for point in contour_points:
        # Incrementa il contatore e salva solo i primi 8 punti
        num_points_saved += 1
        if num_points_saved <= 8:
            contour_pub.publish(point)

            # Salva le coordinate su un file di testo
            with open('coordinate_punti.txt', 'a') as file:
                file.write(f"{point.x} {point.y} {point.z}\n")

    # Visualizza l'immagine con i contorni
    # cv2.imshow("Immagine con vertici", frame)
    # cv2.waitKey(1)  # Aggiorna la finestra per visualizzare l'immagine

def main():
    rospy.init_node('image_processing_node', anonymous=True)

    # Crea una nuova topic per pubblicare i punti di contorno
    contour_pub = rospy.Publisher('/camera/contour_points', Point, queue_size=1)
    
    # Sottoscriviti alla topic dell'immagine proveniente dalla telecamera Gazebo
    camera_topic = "/camera/color/image_raw"
    rospy.Subscriber(camera_topic, Image, camera_callback, contour_pub)

    rospy.spin()

if __name__ == '__main__':
    main()
