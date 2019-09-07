#!/usr/bin/env python

#
# Implement a P controller by running 100 iterations
# of robot motion. The steering angle should be set
# by the parameter tau so that:
#
# steering = -kp * CTE - kd * diff_CTE - ki * int_CTE
#
# where the integrated crosstrack error (int_CTE) is
# the sum of all the previous crosstrack errors.
# This term works to cancel out steering drift.

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
#from sensors.msg import x_state
global cte
cte=0
print "cte"
print type(cte)
py=None # from lidar
#def fnc_callback(msg):
  #  global py , cte

  #  py=msg.py



# from lane detection
def callback(data):
   
    
    # Get the departure direction:
    global cte
   # cte = abs(data)
    cte = float(data.data)
    # Play a sound that notifies the driver 
    # if the departure is greater than 50 cm
    # debug
    #cte=.52
    #print cte

def pid_function(cte ,prev_cte,int_cte):
	kp=.2
	kd=3
	ki=.004
	diff_cte = -cte + prev_cte
	prev_cte = cte
	int_cte += cte
	steering =kp * cte + kd * diff_cte + ki * int_cte
 
        print steering  
	return steering

# ------------------------------------------------------------------------
def the_main():
        global cte ,prev_cte
	

	
	
        rospy.init_node("steering_control",anonymous=True)
        pub=rospy.Publisher("steering_angle",Float32,queue_size=10)
    	#sub1=rospy.Subscriber('object_position', x_state, fnc_callback)
        rospy.Subscriber("image_lane_departure", String, callback)
	rate=rospy.Rate(1)
        #cte=.5 # from cte deperture form the center of lane
	
	max_steering_angle=.52
	lane_wide=3.7
	py=1


	#rospy.loginfo("cte %f",cte)
        while not rospy.is_shutdown() :
	    
#	    if py>0 :
#	    	if py<.5*lane_wide : # the object close to me from left side  case 1
#			y_offset=-.5*lane_wide+py #shift to right
#	    elif py<0 :
#	    	if py>-.5*lane_wide : # the object close to me from right side
#			y_offset=.5*lane_wide+py #shift to left
			
#	    else : 
#		y_offset = 0
		
				



	  # to shift tempory until the object pass
	    prev_cte = cte
	    int_cte = 0
#
	    if cte>0 :#or y_offset<0 : # case 1 

		    while cte >  0 : #move to right
			
			#cte=cte-.1 # (fake feedback) it will replace by the value came form lane detection node
	
			steering = pid_function(cte,prev_cte,int_cte)
		        if steering > max_steering_angle:
		        	steering = max_steering_angle


			rospy.loginfo("first case rotate to left ") 
			rospy.loginfo("steering %f ",steering) 

			#rospy.loginfo("cte %f",cte)
			pub.publish(steering)
			rate.sleep()
	   # if py>0 :
	    #	while py>.5*lane_wide : # to avoid go to next step and return y=0
		#	i=1

#	    if cte>0  :#or y_offset>0 :	
#		    while cte < 0  :
			
			#cte=cte+.1 #fake
#			steering = pid_function(cte,prev_cte,int_cte)

#		       	if steering < 0:
#		        	steering = .01
#		        if steering < -max_steering_angle:
#		        	steering = max_steering_angle
#			rospy.loginfo("second case")
#			rospy.loginfo(steering) 
#			rospy.loginfo(y)
#			pub.publish(steering)
#			rate.sleep()
#	    if py<0 :
#	    	while py>-.5*lane_wide : # to avoid go to next step and return y=0
#			i=1       
			
            if cte==0 :
                    print "at the center of lane"

        			
            if cte < 0 :
				#move to left
			prev_cte=cte
                        print " left"                   
                        while cte < 0 : #shift to y_offset as a referance 
			
			#cte=cte-.1 # (fake feedback) it will replace by the value came form lane detection node
	                       
			     steering = pid_function(cte,prev_cte,int_cte)

		      
		             if steering < -max_steering_angle:
		        	       steering = -max_steering_angle


			     rospy.loginfo("first case shift to right ") 
			     rospy.loginfo("steering %f ",steering) 

			     rospy.loginfo("cte %f",cte)
			     pub.publish(steering)
			     rate.sleep()
	     
if __name__=='__main__' :
    the_main()

