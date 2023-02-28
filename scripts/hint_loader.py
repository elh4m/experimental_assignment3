#! /usr/bin/env python2

## @package exp_assignment3
# \file hint_loader.py
# \brief This file contains code for 'hind_loader' node.
# \author Elham Mohammadi
# \version 1.0
# \date 25.01.2023
#
# \details
#
# Service: <BR>
# /hint_loader_service
# /armor_interface_srv
# /oracle_solution
#  
# Subscribes to: <BR>
#	[None]
#
# Publishes to: <BR>
#	[None]
#
# This node waits for 'hint_loader_service' service requests from the 'hintcollector' node. Based on the request recieved, it 
# loads the hint in the ARMOR reasonser, start the reasoner to deduced a hypotheses based on the previously loaded hints and 
# request ARMOR reasoner for the list of 'COMPLETE' hypotheses. If the recently deduced hypothesis is 'COMPLETE' then it checks
# its 'CORRECTNESS' by using the service '/oracle_solution'. 
#

import rospy
import time
from erl2.srv import Oracle, OracleRequest, OracleResponse
from erl2.msg import ErlOracle
from exp_assignment3.srv import HintLoader, HintLoaderResponse, HintLoaderRequest
from armor_msgs.srv import ArmorDirective,ArmorDirectiveRequest


##  initializing global variable 'oracle_client_' with 'None' for '/oracle_solution' service client.
oracle_client_ = None

##  initializing global variable 'armor_client_' with 'None' for '/armor_interface_srv' service client.
armor_client_ = None
##  initializing global variable 'armor_req_' with 'None' for '/armor_interface_srv' service request.
armor_req_ = None

##  initializing global variable 'count_' with '0'.
count_ = 0
##  initializing global variable 'prev_comp_hypo_' with '0'.
prev_comp_hypo_ = 0

##  initializing global variable 'ID_' with '-1'.
ID_ = -1

##  initializing global variable  dictionary 'Hints_' in which all hints will be stored.
Hints_ = {}
##  initializing global variable 'complete_hypo_id_' with '-1' which is use for storing id of most recent complete hypothesis.
complete_hypo_id_ = -1
##  initializing global variable 'complete_hypo_id_list_ which is use for storing ids of all complete hypothesises.
complete_hypo_id_list_ = []


##
# \brief The function checks if the provided id alreay exist or not in the 'complete_hypo_id_list_' list. 
# 
# \return Bool
#
# The function checks if the provided id alreay exist or not in the 'complete_hypo_id_list_' list. 
#
def isExist_hypo_id_list(id):
    global complete_hypo_id_list_
    for element in complete_hypo_id_list_:
        if element == id:
            return True
    return False

##
# \brief This is a callback function for the 'hint_loader_service' service. 
# 
# \return Bool
#
# This function is a callback function of '/hint_loader_service' service. It waits for service to recieve request from the 'hint_collector' node. 
# Based on the type of the recieved request, it loads the hint in the ARMOR knowlegde base, start the reasoner to deduced a hypotheses based on previously 
# loaded hints and request ARMOR reasoner for the list of 'COMPLETE' hypotheses. If the hypotheses is 'CONSISTENT' it respond with 'True' otherwise
# it respond with 'False'. Similarly if the hypotheses is 'CONSISTENT' then the user request this node to check if the hypotheses is also 'CORRECT'. 
# If hypotheses is also correct then the node respond back with 'True' and hint statement otherewise it returns 'False'.
#
def clbk_oracle_service(msg):
	global count_
	global armor_req_
	global armor_res_
	global prev_comp_hypo_
	global ID_
	global Hints_
	global complete_hypo_id_
	global complete_hypo_id_list_

	if(count_ == 0):
				
		# Loading the ontology file in the reasoner.
		armor_req_ = ArmorDirectiveRequest()
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'LOAD'
		armor_req_.armor_request.primary_command_spec = 'FILE'
		armor_req_.armor_request.secondary_command_spec = ''
		armor_req_.armor_request.args = ['/root/Desktop/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology','true', 'PELLET', 'true']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		time.sleep(1)

	if(msg.req.key == "who" and armor_res_.armor_response.success == True):
		
		print("count_ :", count_)
		count_ += 1
		hintname = "Hint" + str(count_)
		Hint = { "id":msg.req.ID, "key":msg.req.key, "value":msg.req.value }	
		Hints_.update({hintname:Hint})


		# loading the hint in the reasoner.
		print("msg->key :", msg.req.key)

		array1 = [msg.req.key,str(msg.req.ID),msg.req.value]
		print(array1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'OBJECTPROP'
		armor_req_.armor_request.secondary_command_spec = 'IND'
		armor_req_.armor_request.args = array1
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		time.sleep(1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = [msg.req.value,'PERSON']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			response = HintLoaderResponse()
			response.res = True
			response.hintStatement = ""
			response.hintLoaded = False
			response.completeHypo = False
			return response
		
	elif(msg.req.key == "what" and armor_res_.armor_response.success == True):
		
		print("count_ :", count_)
		count_ += 1
		hintname = "Hint" + str(count_)
		Hint = { "id":msg.req.ID, "key":msg.req.key, "value":msg.req.value }	
		Hints_.update({hintname:Hint})

		# loading the hint in the reasoner.
		print("msg.command :", msg.req.key)		
		array2 = [msg.req.key,str(msg.req.ID),msg.req.value]
		print(array2)

		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'OBJECTPROP'
		armor_req_.armor_request.secondary_command_spec = 'IND'
		armor_req_.armor_request.args = array2
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		time.sleep(1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = [msg.req.value,'WEAPON']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			response = HintLoaderResponse()
			response.res = True
			response.hintStatement = ""
			response.hintLoaded = True
			response.completeHypo = False
			return response
	
	elif(msg.req.key == "where" and armor_res_.armor_response.success == True):
		
		print("count_ :", count_)
		count_ += 1
		hintname = "Hint" + str(count_)
		Hint = { "id":msg.req.ID, "key":msg.req.key, "value":msg.req.value }	
		Hints_.update({hintname:Hint})

		# loading the hint in the reasoner.
		print("msg.command :", msg.req.key)
	
		array3 = [msg.req.key,str(msg.req.ID),msg.req.value]
		print(array3)

		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'OBJECTPROP'
		armor_req_.armor_request.secondary_command_spec = 'IND'
		armor_req_.armor_request.args = array3
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		time.sleep(1)
		
		armor_req_.armor_request.client_name = 'tutorial'
		armor_req_.armor_request.reference_name = 'ontoTest'
		armor_req_.armor_request.command = 'ADD'
		armor_req_.armor_request.primary_command_spec = 'IND'
		armor_req_.armor_request.secondary_command_spec = 'CLASS'
		armor_req_.armor_request.args = [msg.req.value,'PLACE']
		print(armor_req_)
		armor_res_ = armor_client_(armor_req_)
		print(armor_res_)
		
		if(armor_res_.armor_response.success == True):
			response = HintLoaderResponse()
			response.res = True
			response.hintStatement = ""
			response.hintLoaded = True
			response.completeHypo = False
			return response
		
	elif(msg.req.key == "REASON" and armor_res_.armor_response.success == True):

		# Starting the reasoner
		print("msg.key :", msg.req.key)
		
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
			response = HintLoaderResponse()
			response.res = True
			response.hintStatement = ""
			response.hintLoaded = True
			response.completeHypo = False
			return response
		
	elif(msg.req.key == "COMPLETED" and armor_res_.armor_response.success == True):	
		# Starting the reasoner
		print("msg.key :", msg.req.key)
		
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

			for element in armor_res_.armor_response.queried_objects:
			    for char in element:
			        if char.isdigit():
			            if not isExist_hypo_id_list(int(char)):
			                complete_hypo_id_list_.insert(0,int(char))
			                complete_hypo_id_ = int(char)
			                print("complete hypothesis id is :",char)


			print("NEW COMPLETE HYPOTHESIS :", new_comp_hypo)
			print("PREVIOUS COMPLETE HYPOTHESIS :", prev_comp_hypo_)
			print("COMPLETE HYPOTHESIS LIST :", armor_res_.armor_response.queried_objects)
			
			if(new_comp_hypo > prev_comp_hypo_):
				prev_comp_hypo_ = 	new_comp_hypo		
				response = HintLoaderResponse()
				response.res = True
				response.hintStatement = ""
				response.hintLoaded = True
				response.completeHypo = True
				return response
			else:
				response = HintLoaderResponse()
				response.res = False
				response.hintStatement = ""
				response.hintLoaded = True
				response.completeHypo = False
				return response
	
	elif(msg.req.key == "CORRECTNESS" and armor_res_.armor_response.success == True):	
		
		print("msg.key :", msg.req.key)
		
		#calling the 'oracle_solution' service for correct solution. 
		correct_solution_id = oracle_client_()
		print("correct_solution_id = ", correct_solution_id.ID)
		print("ID_ =", complete_hypo_id_)
		

		for element in Hints_:
		    if Hints_.get(element).get("id") == complete_hypo_id_:
		        if Hints_.get(element).get("key") == "who":
		            who = Hints_.get(element).get("value")
		            #print("who_ : ", who_)
		        elif Hints_.get(element).get("key") == "where":
		            where = Hints_.get(element).get("value")
		            #print("where_ : ", where_)
		        elif Hints_.get(element).get("key") == "what":
		            what = Hints_.get(element).get("value")
		            #print("what_ : ", what_)

		hintStatement = who + " with the " + what + " in the " + where

		if(correct_solution_id.ID == complete_hypo_id_):
				print(hintStatement)
				response = HintLoaderResponse()
				response.res = True
				response.hintStatement = hintStatement
				response.hintLoaded = True
				response.completeHypo = True
				return response
		else:
				response = HintLoaderResponse()
				response.res = False
				response.hintStatement = ""
				response.hintLoaded = True
				response.completeHypo = True
				return response


##
# \brief This is a 'main' function of oracle node. 
# 
# \return [none].
#
# This is a 'main' function of 'hind_loader' node. It initializes service clients for '/armor_interface_srv' and '/oracle_solution' services. It also initializes '/hint_loader_service'
# service server.
# 

def main():
	global armor_client_
	global oracle_client_	
	rospy.init_node('oracle')
	print("'hint_loader_service' is live...")
	hint_loader_server = rospy.Service('/hint_loader_service', HintLoader, clbk_oracle_service)
	armor_client_ = rospy.ServiceProxy('/armor_interface_srv', ArmorDirective)
	oracle_client_ = rospy.ServiceProxy('/oracle_solution', Oracle)
	rospy.spin()

if __name__ == '__main__':
    main()
