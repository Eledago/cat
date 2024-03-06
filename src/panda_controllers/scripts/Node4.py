#######-----PATH PLANNING PER ESPLORAZIONE PROVINO-----#######

#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped, Quaternion
from gazebo_msgs.msg import LinkStates, ContactsState
from std_msgs.msg import Header
import rospy
import random
import math
import pygame

#Posizione iniziale da cui parte l'end effector 
init_pos = [0.672918, -0.025246, 0.01, 0.9238892418800541, -0.3826599852741115, -5.329145561324561e-05,
            3.96022311888679e-05]

global shutdown, initial_pos_publisher
shutdown = False
#Pubblicazione della posa iniziale sulla topic del controllore
initial_pos_publisher = rospy.Publisher('/cartesian_impedance_controller_softbots/equilibrium_pose', PoseStamped, queue_size=10)

# Inizializzazione di Pygame per la riproduzione audio e per la finestra grafica
pygame.init()

# Definzione delle dimensioni della finestra grafica
window_width = 500
window_height = 300

# Caricamento del suono desiderato
sound_file_path = "/home/eleonora/catkin_ws/src/errore.wav"
pygame_sound = pygame.mixer.Sound(sound_file_path)


# Funzione per mostrare una finestra grafica di avviso
def show_warning_window():
    global screen, pygame_sound

    # Riproduzione suono con Pygame
    pygame_sound.play()
    pygame.time.wait(int(pygame_sound.get_length() * 1000))

    # Inizializzazione della finestra di avviso con scritta e disegno
    screen = pygame.display.set_mode((window_width, window_height))
    pygame.display.set_caption('Avviso')
    font = pygame.font.Font(None, 36)
    text = font.render('RICHIESTO INTERVENTO!', True, (255, 0, 0))
    screen.blit(text, (window_width // 2 - text.get_width() // 2, window_height // 2 - text.get_height() // 2))

    #Disegno di triangolo di pericolo
    image_path = "/home/eleonora/catkin_ws/src/pericolo.jpg"
    image = pygame.image.load(image_path)
    image_rect = image.get_rect(center=(window_width // 2, 3 * window_height // 4))
    screen.blit(image, image_rect)
    
    pygame.display.flip()

#Generazione di una traiettoria random ad ogni accensione del nodo per l'esplorazione del provino
def generate_random_trajectory(center_point, amplitude, num_s_waypoints):
    trajectory = []
    for i in range(num_s_waypoints):
        t = i / float(num_s_waypoints - 1)

        random_sin = random.uniform(-1, 1)
        random_cos = random.uniform(-1, 1)

        x = center_point[0] + amplitude * (random_sin + 0.5 * random_sin * random_sin)
        y = center_point[1] + amplitude * (random_cos - 0.5 * random_cos * random_cos)
        z = center_point[2]

        orientation = init_pos[3:]

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation = Quaternion(*orientation)

        trajectory.append(pose_stamped)

    return trajectory
pass

#Funzione che richiama la posizione del link 7 del Panda
def link_states_callback(msg):
    try:
        link_index = msg.name.index("panda::panda_link7")
    except ValueError:
        rospy.logwarn("Link 'panda::panda_link7' non trovato in link_states.")
        return

    center_point = init_pos[:3]
    amplitude = 0.02
    num_s_waypoints = 20

    s_waypoints = generate_random_trajectory(center_point, amplitude, num_s_waypoints)
    rate = rospy.Rate(0.5)
    publish_waypoints(s_waypoints, rate)

    rospy.loginfo("Traiettoria random completata. Spegnimento del nodo.")
    rospy.signal_shutdown("Traiettoria random completata. Spegnimento del nodo.")
    pass

#Pubblicazione dei waypoints sulla topic del controllore
def publish_waypoints(waypoints, rate):
    pub = rospy.Publisher('/cartesian_impedance_controller_softbots/equilibrium_pose', PoseStamped, queue_size=10)
    rospy.loginfo("Publishing random trajectory...")

    for waypoint in waypoints:
        if rospy.is_shutdown():
            break
        pub.publish(waypoint)
        rate.sleep()
    pass

def contact_callback(data):
    global shutdown
    # Logica per la gestione del contatto con i muri virtuali
    for contact_state in data.states:
        #Il contatto viene rilevato quando la forza letta dalla topic risulta essere in modulo tra 0.5 e 10 N
        force_value = contact_state.total_wrench.force
        force = math.sqrt(force_value.x**2 + force_value.y**2 + force_value.z**2)
        if 0.5 < force < 10:
            rospy.logwarn("AVVISO: Rilevato contatto con il muro!")
            rospy.loginfo("Forza:  {}".format(force))
            # Comparsa della finestra di avviso se le condizioni imposte sono verificate
            show_warning_window()
            rospy.sleep(1)
            
            # Se si verifica la violazione ritorna alla posa iniziale
            return_to_init_pos()
            
            #Spegnimento del nodo se si verifica la violazione
            rospy.signal_shutdown('Spegnimento nodo')
            shutdown = False
    pass


def return_to_init_pos():
    global initial_pos_publisher

    initial_pos = PoseStamped()
    initial_pos.header = Header()
    initial_pos.header.stamp = rospy.Time.now()
    initial_pos.header.frame_id = 'world'

    initial_pos.pose.position.x = init_pos[0]
    initial_pos.pose.position.y = init_pos[1]
    initial_pos.pose.position.z = init_pos[2]
    initial_pos.pose.orientation.x = init_pos[3]
    initial_pos.pose.orientation.y = init_pos[4]
    initial_pos.pose.orientation.z = init_pos[5]
    initial_pos.pose.orientation.w = init_pos[6]

    initial_pos_publisher.publish(initial_pos)
    pass


def main():
    rospy.sleep(28)
    try:
        rospy.init_node('random_trajectory_publisher', anonymous=True)
        #Sottoscrizione alle topic /gazebo/link_states & /panda_leftfinger_contactsensor_state
        rospy.Subscriber('/gazebo/link_states', LinkStates, link_states_callback)
        rospy.Subscriber('/panda_leftfinger_contactsensor_state', ContactsState, contact_callback)

        #Pubblicazione sulla topic del controllore
        pub = rospy.Publisher('/cartesian_impedance_controller_softbots/equilibrium_pose', PoseStamped, queue_size=10)
        rate = rospy.Rate(0.5)  

        while not rospy.is_shutdown():
            center_point = init_pos[:3]
            amplitude = 0.02
            num_s_waypoints = 20

            s_waypoints = generate_random_trajectory(center_point, amplitude, num_s_waypoints)

            for waypoint in s_waypoints:
                if rospy.is_shutdown():
                    break
                pub.publish(waypoint)
                rate.sleep()

            rospy.loginfo("Traiettoria random completata. Spegnimento del nodo.")
            return_to_init_pos()
            rospy.signal_shutdown("Traiettoria random completata. Spegnimento del nodo.")

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()