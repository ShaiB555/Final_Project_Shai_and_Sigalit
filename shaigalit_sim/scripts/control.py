#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from math import atan2, sin, cos, sqrt
import time
import angle_diff

def control_callback(K_pos,K_ori):

    rospy.init_node('jackal_controller')
    
    #Initial values
    x_d_init = [[-3.0], [2.0], [0.0]]
    error_init = [[0.0], [0.0], [0.0]]
    x_est_init = [[0.0],[0.0],[0.0]]
    error_int_init = [[0.0], [0.0], [0.0]]
    x_d=np.array(rospy.get_param("x_d",x_d_init))
    x_est=np.array(rospy.get_param("x_est",x_est_init))
    Phase_init=True

    #Calculating the time increment from the last iteration to the current time, dt:
    current_time = time.time()
    last_time=rospy.get_param("cont_last_time",0)
    dt = current_time - last_time
    rospy.set_param("cont_last_time",current_time)

    # Errors calculation
    error_prev = rospy.get_param("error",error_init)
    error_int = rospy.get_param("error_int",error_int_init)
    error_x=x_d-x_est
    error_dist=np.sqrt(error_x[0,0]**2+error_x[1,0]**2)
    theta=np.arctan(error_x[1,0]/error_x[0,0])
    phi_est=x_est[2,0]
    phi_d=x_d[2,0]
    error_ori_path=angle_diff.angle_diff(theta,phi_est)
    # error_ori_path=angle_diff.angle_diff(np.pi,error_ori_path)
    error_ori_end=angle_diff.angle_diff(phi_d,phi_est)
    error=[[float(error_dist)],[float(error_ori_path)],[float(error_ori_end)]]
    rospy.set_param("error",error)
    error_der=(np.array(error)-np.array(error_prev))/dt

    # PID controller
    error=np.array(error)
    print("ERROR")
    print(error)
    error_int=np.array(error_int)
    error_prev=np.array(error_prev)
    error=np.array(error)
    error_der=np.array(error_der)
    # print("ERROR INT")
    # print(error_int)
    Phase1=rospy.get_param("Phase1",Phase_init)
    Phase2=rospy.get_param("Phase2",Phase_init)
    Phase3=rospy.get_param("Phase3",Phase_init)
    print("PHASE 1")
    print(Phase1)
    print("PHASE 2")
    print(Phase2)

                                
    if abs(error[1,0])>0.05 and Phase1==True:
        #Initial orientation to goal position:
        error_int[0,0]=error[0,0]+error_prev[0,0]
        v=0
        w=K_ori[0,0]*error[1,0]+K_ori[1,0]*error_int[1,0]+K_ori[2,0]*error_der[1,0]
        error_int=error_int.tolist()
        rospy.set_param("error_int",error_int)
    elif abs(error[0,0])>0.1 and Phase2==True:
        #Phase 2: distance control:
        Phase1=False
        rospy.set_param("Phase1",Phase1)
        #Closing the distance to goal position:
        error_int[1,0]=error[1,0]+error_prev[1,0]
        v=K_pos[0,0]*error[0,0]+K_pos[1,0]*error_int[0,0]+K_pos[2,0]*error_der[0,0]
        w=K_ori[0,0]*error[1,0]+K_ori[1,0]*error_int[1,0]+K_ori[2,0]*error_der[1,0]
        error_int=error_int.tolist()
        rospy.set_param("error_int",error_int)
    elif abs(error[2,0])>0.1 and Phase3==True:
        Phase2=False
        rospy.set_param("Phase2",Phase2)
        #Correcting to desired orientation:
        error_int[2,0]=error[2,0]+error_prev[2,0]
        v=0
        w=K_ori[0,0]*error[2,0]+K_ori[1,0]*error_int[2,0]+K_ori[2,0]*error_der[2,0]
        error_int=error_int.tolist()
        rospy.set_param("error_int",error_int)
    else:
        Phase3=False
        rospy.set_param("Phase3",Phase3)
        #Destination reached within desired error envelope
        v=0
        w=0

    u=[[float(v)],[float(w)]]
    rospy.set_param("u",u)
    u=np.array(u)
    print("U")
    print(u)

    # Publish Twist message with control inputs
    cmd_vel_pub = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    twist_msg = Twist()
    twist_msg.linear.x = u[0,0]
    twist_msg.angular.z = u[1,0]
    cmd_vel_pub.publish(twist_msg)

if __name__ == '__main__':
    try:
        #PID Values - Position
        Kp = 1.0  # Proportional gain
        Ki = 0.0  # Integral gain
        Kd = 10.0  # Derivative gain
        K_pos=np.array([[Kp],[Ki],[Kd]])
        #PID Values - Orientation
        Kp = 0.1  # Proportional gain
        Ki = 0.0  # Integral gain
        Kd = 10.0  # Derivative gain
        K_ori=np.array([[Kp],[Ki],[Kd]])
        rospy.init_node('jackal_controller')
        rate = rospy.Rate(10)  # Adjust the rate as per your requirement
        while not rospy.is_shutdown():
            controller = control_callback(K_pos,K_ori)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
