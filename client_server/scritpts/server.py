#!/usr/bin/env python3

import rospy
from std_srvs.srv import SetBool, SetBoolResponse # 阿SetBoolResponse 是從哪裡來的我還要再研究一下

def callback(req): 
	# Create the response message
	response = SetBoolResponse()
	
	# Execute the task
	if req.data == True:
		response.success = True
		response.message = "This device was enabled"
	else:
		response.success = False
		response.message = "This deive was disabled"
		
	# Return Response
	return response

# Node setup
rospy.init_node("server")
rospy.Service("test_service", SetBool, callback)

rospy.spin()
