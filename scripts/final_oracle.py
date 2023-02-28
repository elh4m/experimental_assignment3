#! /usr/bin/env python2

## @package exp_assignment3
# \file_oracle.py
# \brief This file contains code for oracle node.
# \author Elham Mohammadi
# \version 1.0
# \date 25.01.2023
#
# \details
#
# Service : <BR>
# ° /armor_interface_srv
# ° /oracle_service
#  
# This node waits for '/oracle_service' service request from the 'motion_controller' node. Based on the type of recieved request it loads the hint in the ARMOR reasonser, start the reasoner to deduced a hypotheses based on previously loaded hints and request ARMOR reasoner for the list of 'COMPLETE' hypotheses. If the recently deduced hypotheses is 'CONSISTENT' it respond with 'True' otherwise it respond with 'False'. Similarly if the hypotheses is 'CONSISTENT' then the user request this node to check if the hypotheses is also 'CORRECT'. A hypotheses correct if it is 'CONSISTENT' and belong to a predefined list of correct hypotheses in oracle node. If hypotheses is also correct then the node respond back with 'True' otherewise 'False'
#

import rospy
import time
from exporobot_assignment1.srv import Oracle,OracleResponse
from armor_msgs.srv import ArmorDirective,ArmorDirectiveRequest


##  Initializing global variable 'armor_client_' with 'None' for '/armor_interface_srv' service client.
armor_client_ = None
##  Initializing global variable 'armor_req_' with 'None' for '/armor_interface_srv' service request.
armor_req_ = None

##  Initializing global variable 'count_' with '0'.
count_ = 0
##  Initializing global variable 'prev_comp_hypo_' with '0'.
prev_comp_hypo_ = 0


correct_hypotheses_ = [['Prof. Plum','Revolver','Ballroom'],['Rev. Green','Rope','Hall'],['Col. Mustard','Dagger','Lounge'],['Mrs. White','Spanner','Library'],['Mrs. Peacock','Lead Pipe','Study'],['Miss. Scarlett','Candlestick','Kitchen']]


who_ = "none"
where_ = "none"
what_ = "none"



##
# \brief This is a 'clbk_oracle_service' function of oracle node. 
# 
# \return Bool
#
# This function is a callback function of '/oracle_service' node. It waits for '/oracle_service' service request from the 'motion_controller' node. Based on the type of recieved request, it loads the hint in the ARMOR reasonser, start the reasoner to deduced a hypotheses based on previously loaded hints and request ARMOR reasoner for the list of 'COMPLETE' hypotheses. If the hypotheses is 'CONSISTENT' it respond with 'True' otherwise it respond with 'False'. Similarly if the hypotheses is 'CONSISTENT' then the user request this node to check if the hypotheses is also 'CORRECT'. A hypotheses correct if it is 'CONSISTENT' and belong to a predefined list of correct hypotheses in this node. If hypotheses is also correct then the node respond back with 'True' otherewise 'False'
#
#

def clbk_oracle_service(msg):
	global count_
	global armor_req_
	global armor_res_
	global prev_comp_hypo_
	global who_ 
	global where_
	global what_ 
	
	if(count_ == 0):
		
		# Making sure that this section run only once. 
		count_ += 1	
		
		# Loading the ontology file in the reasoner.
		armor_req_ = ArmorDirectiveRequest()
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'LOAD'
		armor_req_.armor_request.primary_command_spec = 'FILE'
		armor_req_.armor_request.secondary_command_spec = ''
		armor_req_.armor_request.args = ['/root/Desktop/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology',
																							 'true', 'PELLET', 'true']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
	if(msg.command == "who" and armor_res_.armor_response.success == True):
		
		# loading the hint in the reasoner.
		print("msg.command :", msg.command)
		
		who_ = msg.args[2]
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'OBJECTPROP'
		armor_req_.armor_request.secondary_command_spec = 'IND'
		armor_req_.armor_request.args = msg.args
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		time.sleep(1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = [msg.args[2],'PERSON']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			return OracleResponse(True)
		
	elif(msg.command == "what" and armor_res_.armor_response.success == True):
		
		# loading the hint in the reasoner.
		print("msg.command :", msg.command)
		
		what_ = msg.args[2]
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'OBJECTPROP'
		armor_req_.armor_request.secondary_command_spec = 'IND'
		armor_req_.armor_request.args = msg.args
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		time.sleep(1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = [msg.args[2],'WEAPON']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			return OracleResponse(True)
	
	elif(msg.command == "where" and armor_res_.armor_response.success == True):
		
		# loading the hint in the reasoner.
		print("msg.command :", msg.command)
	
		where_ = msg.args[2]
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'OBJECTPROP'
		armor_req_.armor_request.secondary_command_spec = 'IND'
		armor_req_.armor_request.args = msg.args
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		time.sleep(1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = [msg.args[2],'PLACE']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			return OracleResponse(True)
		
	elif(msg.command == "REASON" and armor_res_.armor_response.success == True):

		# Starting the reasoner
		print("msg.command :", msg.command)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'REASON'
		armor_req_.armor_request.primary_command_spec = ''
		armor_req_.armor_request.secondary_command_spec = ''
		armor_req_.armor_request.args = []
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			return OracleResponse(True)
		
	elif(msg.command == "COMPLETED" and armor_res_.armor_response.success == True):
		
		# Starting the reasoner
		print("msg.command :", msg.command)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'QUERY'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = ['COMPLETED']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
					
			new_comp_hypo =  len(armor_res_.armor_response.queried_objects)
			
			print("NEW COMPLETE HYPOTHESIS :", new_comp_hypo)
			print("PREVIOUS COMPLETE HYPOTHESIS :", prev_comp_hypo_)
			
			if(new_comp_hypo > prev_comp_hypo_):
				prev_comp_hypo_ = 	new_comp_hypo		
				return OracleResponse(True)
			else:
				return OracleResponse(False)
	
	elif(msg.command == "CORRECTNESS" and armor_res_.armor_response.success == True):	
		
		print("msg.command :", msg.command)
		t = 0
		for i in range(len(correct_hypotheses_)):
			if(who_ == correct_hypotheses_[i][0] and what_ == correct_hypotheses_[i][1] and where_ == correct_hypotheses_[i][2]):
				return OracleResponse(True)

		return OracleResponse(False)

##
# \brief This is a 'main' function of oracle node. 
# 
# \return [none].
#
# This is a 'main' function of 'oracle' node. It initializes client for '/armor_interface_srv' service hosted by 'armor' package and server for '/oracle_service' service.
# 

def main():
	global armor_client_
	rospy.init_node('oracle')
	oracle_server = rospy.Service('/oracle_service', Oracle, clbk_oracle_service)
	armor_client_ = rospy.ServiceProxy('/armor_interface_srv', ArmorDirective)
	rospy.spin()

if __name__ == '__main__':
    main()

