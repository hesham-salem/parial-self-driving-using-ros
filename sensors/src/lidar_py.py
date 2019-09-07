#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import numpy as np
import time 
from sensors.msg import x_state
x_state1 =x_state()

global x_axis
x_axis=[]
global mean_list
mean_list=[]
global last_mini_distance
last_mini_distance= 0
global last_time 
last_time = 0
def mycallBack_fun(mesg):
	global x_axis
	x_axis=list(dict.fromkeys(mesg.ranges))
	print 'here in callback_fun'
	#print x_axis

def the_main_fun():
	
	print 'test 1'	
	sub_y=rospy.Subscriber('scan',LaserScan,mycallBack_fun)
   	pub=rospy.Publisher('object_position',x_state, queue_size=10)
        rospy.init_node('lidar_driver')
	
        rate=rospy.Rate(1)
	#global x_axis1
	#i=0
	#x_axis1=x_axis
	#x_axis[len(lis)]=None
	#x_axis=[1,2,.5,3,0,1,2]
	#x_axis1 = [x_axis[i]*np.sin(np.deg2rad(360-(360/len(x_axis))*i)) for i in range(len(x_axis))] 
	#while i < len(x_axis) :

		#print x_axis[i]
	#	x_axis1.append(x_axis[i]*np.sin(360-(360/len(x_axis))*i))
		#print 'after'
		#print x_axis[i]
		#print ''
	#	i+=1
	#x_axis1=list(dict.fromkeys(x_axis1))
	print'test 1.5'

	while not rospy.is_shutdown() :

		global last_mini_distance
		global last_time 	
		print 'test 2'
		x_axis.remove(0.0)
		print x_axis

		mini_distance = min(x_axis)
		print 'mini_distance'
		print mini_distance
		if mini_distance <3.5   :
			x_state1.px=mini_distance
		#mean_list.append(45+x_axis.index(mini_distance)*(360/len(x_axis)))
	#	if len(mean_list)>30:
	#		mean=max(mean_list,key=mean_list.count)
	#		print ''
	#		print mini_distance 
	#		print mean
	#		print mini_distance*np.sin(np.deg2rad(mean))
	#		print ''
			#seconds_elapsed = time() - last_time
			#last=time()
	#		del mean_list[:]
			delta_x=mini_distance-last_mini_distance
			last_mini_distance=mini_distance
			
			delta_time=time.time()-last_time
			last_time=time.time()
		
			velocity_x = delta_x/delta_time
			x_state1.vx=velocity_x
		#	print mini_distance	
			print delta_x
			print delta_time
			print velocity_x
			pub.publish(x_state1)
	#		print (x_axis.index(mini_distance)*0.5)
			

	        rate.sleep()
	


if __name__ == '__main__':
	the_main_fun()
