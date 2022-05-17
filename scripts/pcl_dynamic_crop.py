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
from math import asin, acos, atan



class arm_dynamic_crop():
  def __init__(self):
      self.tf_buffer = tf2_ros.Buffer()
      self.listener = tf2_ros.TransformListener(self.tf_buffer)
      self._transform_broadcaster = tf2_ros.TransformBroadcaster()
      self.arms_link_tf = TransformStamped()

      self.pcl_sub = rospy.Subscriber("front_lslidar_point_cloud", pc2.PointCloud2, self.pcl_callback, queue_size=1)
      self.output_frame = rospy.get_param('output_frame', 'bobcat_base')
      self.input_frame = rospy.get_param('input_frame', "arms_link")
      self.tool = rospy.get_param('~tool_crop')
      print(self.tool)



      self.params = {'min_x' : -1.0,
               'max_x' :  1.0,
               'min_y' : -1.0,
               'max_y' :  1.0,
               'min_z' : -1.0,
               'max_z' :  1.0,
               'output_frame' : self.output_frame,
               'input_frame'  : self.input_frame,
               'negative' : 1,
               'keep_organized' : 1
      }


      if (self.tool == 'camera_tool'):
        print('at camera tool init')
        self.cam_tool_crop_node = '/camera_tool_crop_box'
        self.cam_tool_client = dynamic_reconfigure.client.Client(self.cam_tool_crop_node)
        self.cam_params = self.params
      else: 
        print("at end eff")
        self.end_eff_crop_node = '/end_effector_crop_box'
        self.end_eff_client = dynamic_reconfigure.client.Client(self.end_eff_crop_node)
        self.end_eff_params = self.params






  def pcl_callback(self, ros_msg):

    self.broadcast_tf2()
    if (self.tool == 'camera_tool'):
      # print(self.cam_params)
      self.cam_tool_client.update_configuration(self.cam_params)
    else:
      # print(self.end_eff_params)

      self.end_eff_client.update_configuration(self.end_eff_params)


  def change_frame(self, data, new_frame):

      return  self.tf_buffer.transform(data, new_frame, rospy.Duration(2.0))

  def publish_tf(self, x, y, z, i, frame_id = 'arms_link'):
      ## Publish tf on Rviz

      transform = TransformStamped()
      transform.header.stamp = rospy.Time.now() # Works at the moment
      transform.child_frame_id = "pos_"  + str(i)
      transform.header.frame_id = frame_id
      transform.transform.translation.x = x
      transform.transform.translation.y = y
      transform.transform.translation.z = z
      # transform.transform.rotation.x = self.arms_link_tf.transform.rotation.x
      # transform.transform.rotation.y = self.arms_link_tf.transform.rotation.y
      # transform.transform.rotation.z = self.arms_link_tf.transform.rotation.z
      # transform.transform.rotation.w  = self.arms_link_tf.transform.rotation.w
      transform.transform.rotation.x = 0
      transform.transform.rotation.y = 0
      transform.transform.rotation.z = 0
      transform.transform.rotation.w  = 1
      self._transform_broadcaster.sendTransform(transform)
      return transform


  def broadcast_tf2(self):

      try:

        end_eff = self.tf_buffer.lookup_transform(self.output_frame, 'end_effector_link', rospy.Time(), rospy.Duration(3.0))
        camera = self.tf_buffer.lookup_transform(self.output_frame, 'camera_tool_link', rospy.Time(), rospy.Duration(3.0))


        if end_eff.transform.translation.z > 2.73:
            end_eff.transform.translation.z = 2.73
        value = (end_eff.transform.translation.z-1.413525)/2.5
        q = tf_conversions.transformations.quaternion_from_euler(0,
                                                                 -atan(value)-0.463525,
                                                                 0)
        self.arms_link_tf.header.stamp = rospy.Time.now()
        self.arms_link_tf.header.frame_id = self.output_frame
        self.arms_link_tf.child_frame_id = self.input_frame
        self.arms_link_tf.transform.translation.x = -1.174615
        self.arms_link_tf.transform.translation.y = 0.0
        self.arms_link_tf.transform.translation.z = 1.413525
        self.arms_link_tf.transform.rotation.x = q[0]
        self.arms_link_tf.transform.rotation.y = q[1]
        self.arms_link_tf.transform.rotation.z = q[2]
        self.arms_link_tf.transform.rotation.w = q[3]
        self._transform_broadcaster.sendTransform(self.arms_link_tf)

        if (self.tool == 'camera_tool'):
          # CAMERA TOOL CROPBOX
          frame = self.output_frame
          z_min_value = end_eff.transform.translation.z - 0.45
          if z_min_value < 0.3:
            z_min_value = 0.3
          x_max_value = camera.transform.translation.x + 0.7
          if x_max_value < end_eff.transform.translation.x + 0.3:
            x_max_value = end_eff.transform.translation.x + 0.3

          # print(z_min_value - end_eff.transform.translation.z)
          cam_max_values = self.publish_tf(x_max_value, 
                                       camera.transform.translation.y + 1.2,
                                       camera.transform.translation.z + 0.3,
                                       'cam_max_values', frame)
          cam_min_values = self.publish_tf(end_eff.transform.translation.x - 0.35, 
                                       camera.transform.translation.y - 1.2,
                                       z_min_value,
                                       'cam_min_values', frame)
          cam_max_frame = PoseStamped()
          cam_max_frame.header = cam_max_values.header
          cam_max_frame.pose.position.x = cam_max_values.transform.translation.x
          cam_max_frame.pose.position.y =  cam_max_values.transform.translation.y
          cam_max_frame.pose.position.z =  cam_max_values.transform.translation.z
          # cam_max_frame = self.change_frame(cam_max_frame, self.input_frame)

          cam_min_frame = PoseStamped()
          cam_min_frame.header.stamp = rospy.Time.now()
          cam_min_frame.header.frame_id = frame
          cam_min_frame.pose.position.x = cam_min_values.transform.translation.x
          cam_min_frame.pose.position.y =  cam_min_values.transform.translation.y
          cam_min_frame.pose.position.z =  cam_min_values.transform.translation.z
          # cam_min_frame = self.change_frame(cam_min_frame, self.input_frame)

          self.cam_params['min_x'] = cam_min_frame.pose.position.x 
          self.cam_params['min_y'] = cam_min_frame.pose.position.y 
          self.cam_params['min_z'] = cam_min_frame.pose.position.z
          self.cam_params['max_x'] = cam_max_frame.pose.position.x 
          self.cam_params['max_y'] = cam_max_frame.pose.position.y 
          self.cam_params['max_z'] = cam_max_frame.pose.position.z
          self.cam_params['input_frame'] = frame
          # print(self.cam_params, cam_min_frame, cam_max_frame)
        else:
          z_value_max = end_eff.transform.translation.z - self.arms_link_tf.transform.translation.z + 0.2
          if z_value_max > 0.25:
            z_value_max = 0.25
          if z_value_max < -0.9:
            z_value_max = -0.9
          # END EFFECTOR CROPBOX
          end_eff_max_values = self.publish_tf(acos(end_eff.transform.translation.x/1.3) + 2.35, 
                                       self.arms_link_tf.transform.translation.y  + 0.9,
                                       z_value_max ,
                                       'max_values')
          end_eff_min_values = self.publish_tf(acos(-(end_eff.transform.translation.x/1.3)) - 2.5, 
                                       self.arms_link_tf.transform.translation.y - 1.0,
                                       -asin((end_eff.transform.translation.z/2.73)),
                                       'min_values')

          

          end_eff_max_frame = PoseStamped()
          end_eff_max_frame.header = self.arms_link_tf.header
          end_eff_max_frame.pose.position.x = end_eff_max_values.transform.translation.x
          end_eff_max_frame.pose.position.y =  end_eff_max_values.transform.translation.y
          end_eff_max_frame.pose.position.z =  end_eff_max_values.transform.translation.z

          end_eff_min_frame = PoseStamped()
          end_eff_min_frame.header = self.arms_link_tf.header
          end_eff_min_frame.pose.position.x = end_eff_min_values.transform.translation.x
          end_eff_min_frame.pose.position.y =  end_eff_min_values.transform.translation.y
          end_eff_min_frame.pose.position.z =  end_eff_min_values.transform.translation.z

          self.end_eff_params['min_x'] = end_eff_min_frame.pose.position.x 
          self.end_eff_params['min_y'] = end_eff_min_frame.pose.position.y 
          self.end_eff_params['min_z'] = end_eff_min_frame.pose.position.z
          self.end_eff_params['max_x'] = end_eff_max_frame.pose.position.x 
          self.end_eff_params['max_y'] = end_eff_max_frame.pose.position.y
          self.end_eff_params['max_z'] = end_eff_max_frame.pose.position.z
          self.end_eff_params['input_frame'] = self.input_frame
      except (tf2_ros.LookupException, 
              tf2_ros.ConnectivityException, 
              tf2_ros.ExtrapolationException) as e:
        print (e)

def main():
  rospy.init_node('cluster_pointcloud', anonymous=True)
  cp = arm_dynamic_crop()
  
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass

if __name__ == '__main__':
    main()
