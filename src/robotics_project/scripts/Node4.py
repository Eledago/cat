#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray

def pose_array_callback(pose_array, publisher):
    stamped_poses = pose_array_to_stamped(pose_array)
    
    # Print or use the converted PoseStamped messages as needed
    for stamped_pose in stamped_poses:
        #print(stamped_pose)
        # Publish each PoseStamped message to the new topic
        publisher.publish(stamped_pose)

def pose_array_to_stamped(pose_array):
    stamped_poses = []
    
    for pose in pose_array.poses:
        # Assuming the frame_id for PoseStamped is the same as in PoseArray
        stamped_pose = PoseStamped()
        stamped_pose.header = pose_array.header
        stamped_pose.pose = pose
        stamped_poses.append(stamped_pose)

    return stamped_poses

def main():
    rospy.init_node('pose_array_to_stamped_node', anonymous=True)

    # Create a publisher for the new topic
    publisher = rospy.Publisher('/cartesian_impedance_controller_softbots/equilibrium_pose', PoseStamped, queue_size=10)

    # Subscribe to the /waypoints topic and pass the publisher to the callback
    rospy.Subscriber('/waypoints', PoseArray, pose_array_callback, callback_args=publisher)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
