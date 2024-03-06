#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from std_msgs.msg import Header
from gazebo_msgs.msg import LinkStates
#from sensor_msgs.msg import JointState


# Variabili globali per memorizzare la posizione iniziale del link 7 del Panda
initial_panda_link_pose = PoseStamped()
flag = False

def link_states_callback(msg):
    global flag, initial_panda_link_pose

    #try:
        #panda_link_index = msg.name.index("panda::panda_link7")
    #except ValueError:
       # rospy.logwarn("Link 'panda::panda_link7' non trovato in link_states.")
        #return

    #panda_link_pose = msg.pose[panda_link_index]

    if not flag:
        initial_panda_link_pose.header = Header()
        #initial_panda_link_pose.header.frame_id = "panda_link0"
        initial_panda_link_pose.header.stamp = rospy.Time.now()
        initial_panda_link_pose.pose.position.x = msg.pose.position.x
        initial_panda_link_pose.pose.position.y = msg.pose.position.y
        initial_panda_link_pose.pose.position.z = msg.pose.position.z
        initial_panda_link_pose.pose.orientation = msg.pose.orientation
        flag = True



def phantom_pose_callback(msg):
    global initial_panda_link_pose

    #if initial_panda_link_pose.pose.position.x == 0 and initial_panda_link_pose.pose.position.y == 0 and initial_panda_link_pose.pose.position.z == 0:
     #   rospy.logwarn("Posizione iniziale del link 7 non ancora ricevuta.")
       # return

    panda_pose = PoseStamped()
    panda_pose.header = Header()
    panda_pose.header.stamp = rospy.Time.now()
    panda_pose.header.frame_id = "panda_link7"

    # Calcoliamo la posizione del Panda rispetto alla posizione iniziale del link 7
    panda_pose.pose.position.x = initial_panda_link_pose.pose.position.x + (msg.transform.translation.x)
    panda_pose.pose.position.y = initial_panda_link_pose.pose.position.y + (msg.transform.translation.y)
    panda_pose.pose.position.z = initial_panda_link_pose.pose.position.z + (msg.transform.translation.z)
    panda_pose.pose.orientation = Quaternion(1, 0, 0, 0)  # Orientazione fissa

    #panda_pose.pose.position.x= (panda_pose.pose.position.x)/100
    #panda_pose.pose.position.y= (panda_pose.pose.position.y)/100
    #panda_pose.pose.position.z= (panda_pose.pose.position.z)/100

    panda_equilibrium_pose_pub.publish(panda_pose)
    rospy.loginfo("Publishing")

def main():
    try:
        rospy.init_node('phantom_to_panda_pose', anonymous=True)

        # Subscribe to link states topic to get Panda's link 7 pose
        rospy.Subscriber('/cartesian_impedance_controller_softbots/franka_ee_pose', PoseStamped, link_states_callback)

        # Subscribe to Phantom end effector pose topic
        rospy.Subscriber('/base/ele', TransformStamped, phantom_pose_callback)

        # Publisher for Panda controller equilibrium pose topic
        global panda_equilibrium_pose_pub 
        panda_equilibrium_pose_pub = rospy.Publisher('/cartesian_impedance_controller_softbots/equilibrium_pose', PoseStamped, queue_size=10)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
