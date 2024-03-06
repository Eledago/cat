#######--------NODO CHE FA COMPIERE UNA TRAIETTORIA AL ROBOT PER RAGGIUNGERE IL PROVINO-------#######

#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, Quaternion
import math

flag = False  # Dichiarazione del flag come variabile globale
end_point = [] 

#Funzione che richiama la posizione del link 7 del Panda
def link_states_callback(msg):
    global flag, end_point  # Usa la variabile globale
    try:
        link_index = msg.name.index("panda::panda_link7")
    except ValueError:
        rospy.logwarn("Link 'panda::panda_link7' non trovato in link_states.")
        return

    link_pose = msg.pose[link_index]
    #Punto in cui di trova il link 7 inzialmente
    start_point = [link_pose.position.x, link_pose.position.y, link_pose.position.z,
                   link_pose.orientation.x, link_pose.orientation.y, link_pose.orientation.z, link_pose.orientation.w]

    global end_point

    if not flag:
        #Punto in cui deve arrivare il link 7
        end_point = [0.672918, -0.025246, 0.01, 0.9238892418800541, -0.3826599852741115, -5.329145561324561e-05,
                     3.96022311888679e-05]  # x, y, z, qx, qy, qz, qw
        end_orientation = [0.9238892418800541, -0.3826599852741115, -5.329145561324561e-05,
                           3.96022311888679e-05]  # qx, qy, qz, qw
        num_waypoints = 20
        
        
        waypoints = generate_linear_trajectory(start_point, end_point, end_orientation, num_waypoints)
        rate = rospy.Rate(3)
        publish_waypoints(waypoints, rate)
        
        flag = True  # Aggiornamento del flag per indicare che la traiettoria lineare è stata completata
        rospy.loginfo("Traiettoria lineare completata. Spegnimento del nodo.")
        rospy.signal_shutdown("Traiettoria lineare completata. Spegnimento del nodo.")

#Generazione della traiettoria lineare per portare il robot dal punto iniziale a quello finale
def generate_linear_trajectory(start_point, end_point, end_orientation, num_waypoints):
    trajectory = []
    for i in range(num_waypoints + 1):
        alpha = i / float(num_waypoints)
        x = (1 - alpha) * start_point[0] + alpha * end_point[0]
        y = (1 - alpha) * start_point[1] + alpha * end_point[1]
        z = (1 - alpha) * start_point[2] + alpha * end_point[2]

        orientation = quaternion_slerp(start_point[3:], end_orientation, alpha)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "panda_link7"
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation = Quaternion(*orientation)

        trajectory.append(pose_stamped)

    return trajectory

#Trasformazione da roll-pitch-yaw a quaternioni
def quaternion_slerp(start, end, alpha):
    q_start = Quaternion(*start)
    q_end = Quaternion(*end)
    q_result = Quaternion()

    q_result.x = (1.0 - alpha) * q_start.x + alpha * q_end.x
    q_result.y = (1.0 - alpha) * q_start.y + alpha * q_end.y
    q_result.z = (1.0 - alpha) * q_start.z + alpha * q_end.z
    q_result.w = (1.0 - alpha) * q_start.w + alpha * q_end.w

    norm = math.sqrt(q_result.x**2 + q_result.y**2 + q_result.z**2 + q_result.w**2)
    q_result.x /= norm
    q_result.y /= norm
    q_result.z /= norm
    q_result.w /= norm

    return q_result.x, q_result.y, q_result.z, q_result.w

def publish_waypoints(waypoints, rate):
    #Pubblicazione dei waypoints sulla topic del controllore
    pub = rospy.Publisher('/cartesian_impedance_controller_softbots/equilibrium_pose', PoseStamped, queue_size=10)
    rospy.loginfo("Publishing linear trajectory...")

    for waypoint in waypoints:
        if rospy.is_shutdown():
            break
        pub.publish(waypoint)
        rate.sleep()

def main():
    
    global flag  
    try:
        rospy.init_node('linear_trajectory_publisher', anonymous=True)
        # Sottoscrizione alla topic '/gazebo/link_states'
        rospy.Subscriber('/gazebo/link_states', LinkStates, link_states_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
