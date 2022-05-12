#!/usr/bin/env python3

import dynamic_reconfigure.client
import rospy
import pcl
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf 
from geometry_msgs.msg import TransformStamped, PoseStamped
import tf2_geometry_msgs
import tf_conversions # Because of transformations
from math import asin, acos



class arm_dynamic_crop():
	def __init__(self):
	    self.tf_buffer = tf2_ros.Buffer()
	    self.listener = tf2_ros.TransformListener(self.tf_buffer)
	    self.pcl_sub = rospy.Subscriber("front_lslidar_point_cloud", pc2.PointCloud2, self.pcl_callback, queue_size=1)
	    self.window_size = [1.45, 1.2, 1.34]
	    self.input_frame = rospy.get_param('input_frame', 'bobcat_base')
	    self.output_frame = rospy.get_param('output_frame', "arms_link")
	    self.crop_node = '/end_effector_crop_box'
	    self.client = dynamic_reconfigure.client.Client(self.crop_node)
	    self.params = {'min_x' : -1.0,
	    			   'max_x' :  1.0,
	    			   'min_y' : -1.0,
	    			   'max_y' :  1.0,
	    			   'min_z' : -1.0,
	    			   'max_z' :  1.0,
	    			   'output_frame' : self.input_frame,
	    			   'input_frame'  : self.output_frame,
	    			   'negative' : 0,
	    			   'keep_organized' : 1

	    }
	    self._transform_broadcaster = tf2_ros.TransformBroadcaster()



	def pcl_callback(self, ros_msg):

		self.params = self.broadcast_tf2()

		print(self.params)
		self.client.update_configuration(self.params)



	def change_frame(self, data, new_frame):

	    return  self.tf_buffer.transform(data, new_frame, rospy.Duration(2.0))

	def publish_tf(self, x, y, z, i, trans, frame_id = 'arms_link'):
	    ## Publish tf on Rviz

	    transform = TransformStamped()
	    transform.header.stamp = rospy.Time.now() # Works at the moment
	    transform.child_frame_id = "pos_"  + str(i)
	    transform.header.frame_id = frame_id
	    transform.transform.translation.x = x
	    transform.transform.translation.y = y
	    transform.transform.translation.z = z
	    # transform.transform.rotation.x = 0
	    # transform.transform.rotation.y = 0
	    # transform.transform.rotation.z = 0
	    # transform.transform.rotation.w  = 1
	    transform.transform.rotation.x = trans.transform.rotation.x
	    transform.transform.rotation.y = trans.transform.rotation.y
	    transform.transform.rotation.z = trans.transform.rotation.z
	    transform.transform.rotation.w  = trans.transform.rotation.w
	    self._transform_broadcaster.sendTransform(transform)
	    print(transform)
	    return transform


	def broadcast_tf2(self):

			br = tf2_ros.TransformBroadcaster()
			t = TransformStamped()
			tf_buffer = tf2_ros.Buffer()
			listener = tf2_ros.TransformListener(tf_buffer)
			t.header.stamp = rospy.Time.now()
			t.header.frame_id = "bobcat_base"
			t.child_frame_id = "arms_link"
			t.transform.translation.x = -1.174615
			t.transform.translation.y = 0.0
			t.transform.translation.z = 1.413525
			try:
				end_eff = tf_buffer.lookup_transform(self.output_frame, 'end_effector_link', rospy.Time(), rospy.Duration(3.0))
				arms = tf_buffer.lookup_transform(self.input_frame, 'end_effector_link', rospy.Time(), rospy.Duration(3.0))
				# print(arms, arms2)
				if arms.transform.translation.z > 2.73:
				    arms.transform.translation.z = 2.73
				value = (arms.transform.translation.z-1.413525)/2.5
				q = tf_conversions.transformations.quaternion_from_euler(0,
				                                                         -atan(value)-0.413525,
				                                                         0)
				t.header.stamp = rospy.Time.now()
				t.header.frame_id = "bobcat_base"
				t.child_frame_id = "arms_link"
				t.transform.translation.x = -1.174615
				t.transform.translation.y = 0.0
				t.transform.translation.z = 1.413525
				t.transform.rotation.x = q[0]
				t.transform.rotation.y = q[1]
				t.transform.rotation.z = q[2]
				t.transform.rotation.w = q[3]




				min_x = arms.transform.translation.x
				max_x = arms.transform.translation.x*1.2 + 1.2

				max_values = self.publish_tf(arms.transform.translation.x*1.2 + 1.2, 
																		 t.transform.translation.y + 0.9,
																		 (asin((arms.transform.translation.z-1.413525)/2.5) + 0.9),
																		 'max_values', t)
				min_values = self.publish_tf(arms.transform.translation.x - 0.6, 
																		 t.transform.translation.y - 0.9,
																		 (asin((arms.transform.translation.z-1.413525)/2.5) + 0.2),
																		 'min_values', t)

				max_frame = PoseStamped()
				max_frame.header = t.header
				max_frame.pose.position.x = max_values.transform.translation.x
				max_frame.pose.position.y =  max_values.transform.translation.y
				max_frame.pose.position.z =  max_values.transform.translation.z

				min_frame = PoseStamped()
				min_frame.header = t.header
				min_frame.pose.position.x = min_values.transform.translation.x
				min_frame.pose.position.y =  min_values.transform.translation.y
				min_frame.pose.position.z =  min_values.transform.translation.z


				self.params['min_x'] = min_frame.pose.position.x 
				self.params['max_x'] = max_frame.pose.position.x 
				self.params['min_y'] = min_frame.pose.position.y 
				self.params['max_y'] = max_frame.pose.position.y
				self.params['min_z'] = min_frame.pose.position.z 
				self.params['max_z'] = max_frame.pose.position.z


			except (tf2_ros.LookupException, 
			        tf2_ros.ConnectivityException, 
			        tf2_ros.ExtrapolationException) as e:
				print (e)
			return self.params

def main():
  rospy.init_node('cluster_pointcloud', anonymous=True)
  cp = arm_dynamic_crop()
  
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass

if __name__ == '__main__':
    main()
