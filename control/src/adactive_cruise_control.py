#!/usr/bin/env python

import rospy  ,std_msgs.msg as std # std is object
from sensors.msg import z_vector
from sensors.msg import x_state

vx=0 
v_host =0
def fnc_callback(msg):
    global vx

    vx=msg.vx
	
def feedback_velocity(msg):
	global v_host
	v_host=msg.data
def the_main():
    
    rospy.init_node("adactive_cruise_control", anonymous=True) # the name of node  is Adactive cruise control
    pub_x= rospy.Publisher("cruise_velocity", std.Float32, queue_size=10) # make object of publisher with name of TOPIC plus data type of it plus length of wait list
    sub=rospy.Subscriber('object_position', x_state, fnc_callback)
    velocity_sub=rospy.Subscriber('feedback_velocity',std.Float32,queue_size=10)
    rate=rospy.Rate(10) #determine the rate of publishing
    #-----------------------------------
    x_min =3 #minimum sefy distance
    t=2.5    # time constant 1.6 for fast ,2.5 meduim, 4 slow
    kp=.1    # propertional gain
   # px=12    # come from lidar sensor
    v_host =40
    v_target=vx
    while not rospy.is_shutdown(): # to avoid working while the the master is shutdown
       # try: # if any failure in this scope get out from the scope
       
            v=v_host - v_target
            x_s=v*t+x_min
            if vx<x_s :
                while v>0 :
                    v_p=kp*v
                    v=v-v_p
                    # msg ="test message"
                    pub_x.publish(v)
                    rate.sleep() # force the loop to match the frequancy of node
                    rospy.loginfo("done%d",v) # as debugger
        	    rate.sleep()
#except:    
         #   break
if __name__ =='__main__':
    the_main()
