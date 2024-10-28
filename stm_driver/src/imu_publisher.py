#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import tf
from geometry_msgs.msg import TransformStamped

class IMUTransformPublisher:
    def __init__(self):
        # Initialize the node
        rospy.init_node('imu_transform_publisher_node', anonymous=True)

        # Create a subscriber for IMU data
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)

        # Create a TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

    def imu_callback(self, imu_msg):
        rospy.loginfo("Received IMU data: %s", imu_msg)

        # Create and send the transform
        transform = TransformStamped()
        transform.header.stamp = imu_msg.header.stamp
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "imu_frame"

        # Set the translation (assuming IMU is at the origin)
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0

        # Use the IMU orientation as the rotation
        orientation = imu_msg.orientation

        # Check if the quaternion is valid
        if (orientation.x == 0.0 and orientation.y == 0.0 and
            orientation.z == 0.0 and orientation.w == 0.0):
            rospy.logwarn("Received zero quaternion. Defaulting to identity quaternion.")
            orientation.x = 0.0
            orientation.y = 0.0
            orientation.z = 0.0
            orientation.w = 1.0  # Identity quaternion

        # Publish the transform
        self.tf_broadcaster.sendTransform(
            (transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z),
            (orientation.x,
            orientation.y,
            orientation.z,
            orientation.w),
            imu_msg.header.stamp,
            "imu_frame",
            "base_link"
        )	

if __name__ == '__main__':
    try:
        imu_transform_publisher = IMUTransformPublisher()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
