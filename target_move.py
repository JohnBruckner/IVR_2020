#!/usr/bin/env python


import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64

# Calculate the forward kinematics
def forward_kinematics(self, image): 
  joints - self.detect_joint_angles(image) # or other method
  end_effector = np.array([-2*np.cos(t_1)*np.cos(t_2)*np.cos(t_3)*np.cos(t_4) - 2*np.cos(t_4)*np.sin(t_1)*np.sin(t_3) + 2*np.cos(t_1)*np.sin(t_2)*np.sin(t_4)-3*np.cos(t_1)*np.cos(t_2)*np.cos(t_3)- 3*np.sin(t_1)*np.sin(t_3),
                  -2*np.cos(t_2)*np.cos(t_3)*np.cos(t_4)*np.sin(t_1)+2*np.cos(t_1)*np.cos(t_4)*np.sin(t_3)+2*np.sin(t_1)*np.sin(t_4)-3*np.sin(t_1)*np.cos(t_2)*np.cos(t_3)+3*np.cos(t_1)*np.sin(t_3),
                  -2*np.cos(t_3)*np.cos(t_4)*np.sin(t_2)-2*np.cos(t_2)*np.sin(t_4)-3*np.cos(t_3)*np.sin(t_2)+2])
  return end_effector 

# Calculate Jacobian
def calculate_jacobian(self, image):
    joints - self.detect_joint_angles(image) # or other method
    [t_1, t_2, t_3, t_4] = joints
    jacobian = np.array([[2*np.sin(t_1)*np.cos(t_2)*np.cos(t_3)*np.cos(t_4) - 2*np.cos(t_4)*np.cos(t_1)*np.sin(t_3) - 2*np.sin(t_1)*np.sin(t_2)*np.sin(t_4) + 3*np.sin(t_1)*np.cos(t_2)*np.cos(t_3) - 3*np.cos(t_1)*np.sin(t_3),
                  2*np.cos(t_1)*np.sin(t_2)*np.cos(t_3)*np.cos(t_4) + 2*np.cos(t_1)*np.cos(t_2)*np.sin(t_4) + 3*np.cos(t_1)*np.sin(t_2)*np.cos(t_3),
                  2*np.cos(t_1)*np.cos(t_2)*np.sin(t_3)*np.cos(t_4) - 2*np.cos(t_4)*np.sin(t_1)*np.cos(t_3) + 3*np.cos(t_1)*np.cos(t_2)*np.sin(t_3) - 3*np.sin(t_1)*np.cos(t_3)],
                  2*np.cos(t_1)*np.cos(t_2)*np.cos(t_3)*np.sin(t_4) + 2*np.sin(t_4)*np.sin(t_1)*np.sin(t_3) - 2*np.sin(t_1)*np.sin(t_2)*np.cos(t_4),
                  [-2*np.cos(t_2)*np.cos(t_3)*np.cos(t_4)*np.cos(t_1) - 2*np.sin(t_1)*np.cos(t_4)*np.sin(t_3) +2*np.cos(t_1)*np.sin(t_4) - 3*np.cos(t_1)*np.cos(t_2)*np.cos(t_3) - 3*np.sin(t_1)*np.sin(t_3),
                  2*np.sin(t_2)*np.cos(t_3)*np.cos(t_4)*np.sin(t_1) + 3*np.sin(t_1)*np.sin(t_2)*np.cos(t_3),
                  2*np.cos(t_2)*np.sin(t_3)*np.cos(t_4)*np.sin(t_1) + 2*np.cos(t_1)*np.cos(t_4)*np.cos(t_3) +3*np.sin(t_1)*np.cos(t_2)*np.sin(t_3)+3*np.cos(t_1)*np.cos(t_3),
                  2*np.cos(t_2)*np.cos(t_3)*np.sin(t_4)*np.sin(t_1) - 2*np.cos(t_1)*np.sin(t_4)*np.sin(t_3) +2*np.sin(t_1)*np.cos(t_4)],
                  [0,
                  -2*np.cos(t_3)*np.cos(t_4)*np.cos(t_2) + 2*np.sin(t_2)*np.sin(t_4) -3*np.cos(t_3)*np.cos(t_2),
                  2*np.sin(t_3)*np.cos(t_4)*np.sin(t_2)+3*np.sin(t_3)*np.sin(t_2),
                  2*np.cos(t_3)*np.sin(t_4)*np.sin(t_2)-2*np.cos(t_2)*np.cos(t_4)]])
  return jacobian

# Publish data
def move():
  rospy.init_node('target_pos_cmd', anonymous=True)
  rate = rospy.Rate(30) # 30hz
  # initialize a publisher to send joints' angular position to the robot
  robot_joint1_pub = rospy.Publisher("/target/x_position_controller/command", Float64, queue_size=10)
  robot_joint2_pub = rospy.Publisher("/target/y_position_controller/command", Float64, queue_size=10)
  robot_joint3_pub = rospy.Publisher("/target/z_position_controller/command", Float64, queue_size=10)
  robot_joint4_pub = rospy.Publisher("/target2/x2_position_controller/command", Float64, queue_size=10)
  robot_joint5_pub = rospy.Publisher("/target2/y2_position_controller/command", Float64, queue_size=10)
  robot_joint6_pub = rospy.Publisher("/target2/z2_position_controller/command", Float64, queue_size=10)
  t0 = rospy.get_time()
  while not rospy.is_shutdown():
    cur_time = np.array([rospy.get_time()])-t0
    #y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
    x_d = 1.5* np.cos(cur_time * np.pi/5)
    y_d = 1.5* np.sin(cur_time * np.pi/5)
    z_d = 1* np.sin(cur_time * np.pi/5)
    joint1=Float64()
    joint1.data= 2 + x_d
    joint2=Float64()
    joint2.data= 2.5 + y_d
    joint3=Float64()
    joint3.data= 5.5 + z_d
    robot_joint1_pub.publish(joint1)
    robot_joint2_pub.publish(joint2)
    robot_joint3_pub.publish(joint3)
    x_d = 2* np.cos(cur_time * np.pi/5)
    y_d = 2* np.sin(cur_time * np.pi/5)
    joint4=Float64()
    joint4.data=  x_d
    joint5=Float64()
    joint5.data=  y_d
    joint6=Float64()
    joint6.data= 5
    robot_joint4_pub.publish(joint4)
    robot_joint5_pub.publish(joint5)
    robot_joint6_pub.publish(joint6)
    rate.sleep()



# run the code if the node is called
if __name__ == '__main__':
  try:
    move()
  except rospy.ROSInterruptException:
    pass


