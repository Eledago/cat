#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped

def generate_linear_trajectory(start_point, end_point, num_waypoints):
    trajectory = []

    # Interpolate linearly between start and end points
    for i in range(num_waypoints + 1):
        alpha = i / float(num_waypoints)
        x = (1 - alpha) * start_point[0] + alpha * end_point[0]
        y = (1 - alpha) * start_point[1] + alpha * end_point[1]
        z = (1 - alpha) * start_point[2] + alpha * end_point[2]

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "panda_link7"
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.w = 1.0

        trajectory.append(pose_stamped)

    return trajectory

def publish_waypoints(waypoints, rate):
    pub = rospy.Publisher('/cartesian_impedance_controller_softbots/equilibrium_pose', PoseStamped, queue_size=10)

    rospy.loginfo("Publishing linear trajectory...")

    for waypoint in waypoints:
        if rospy.is_shutdown():  # Verifica se è stato richiesto l'arresto del nodo
            break
        pub.publish(waypoint)
        rate.sleep()

def main():
    try:
        rospy.init_node('linear_trajectory_publisher', anonymous=True)

        start_point = [0.63, 0.0, 1.05]  # Adjust the start point as needed
        end_point = [0.63, 1.0, 1.05]    # Adjust the end point as needed
        num_waypoints = 20
        publish_rate = 0.5  # Hz

        waypoints = generate_linear_trajectory(start_point, end_point, num_waypoints)

        print(waypoints, flush=True)  # Stampa la lista dei waypoints

        # Calcola il tempo di attesa tra i waypoint per ottenere la frequenza desiderata
        rate = rospy.Rate(0.5 / publish_rate)

        publish_waypoints(waypoints, rate)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
