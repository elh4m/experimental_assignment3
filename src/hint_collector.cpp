/**
* \file hint_collector.cpp
* \brief This files contains code for the 'hint_collector' node.
* \author Elham Mohammadi
* \version 1.0
* \date 25.01.2023
*
* \details
* 
* Services: <BR>
* /hint_loader_service
* /oracle_hint
* 
* Advertised Services: <BR>
* /request_hint_collector
*  
* Description :
* This node provides the '/request_hint_collector' service which collects and store the hints one by one from the '/oracle_hint'
* service. Once the hint is collected, it checks if it is consistent and load them in the 'ARMOR' ontology knowlegde base (KB). Then 
* it starts the ontology reasoner, and check if there is a complete and correct hypothesis. If the hints are inconsistent (malformed),
* or the hypothesis is incomplete or incorrect then the node return 'false' otherwise it return 'true'.
*  
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <sstream>
#include <iostream>
#include <string>
#include "erl2/ErlOracle.h"
#include "exp_assignment3/HintLoader.h"
#include <std_srvs/Empty.h>
#include <exp_assignment3/HintCollector.h>
#include <cstdlib>
#include <unistd.h>
#include <exp_assignment3/Marker.h>


///< global variable used for collecting hint value.
erl2::ErlOracle hint;

//< pointer for a ROS client of '\oracle_hint' service
ros::ServiceClient *clientPtr_; 

///< initializing ROS Service '\hint_loader_service' client variable. 
ros::ServiceClient hint_loader_client;

///< global variable used to keep the count of total hints collected.
std::int64_t count = 0;

/**
* \brief The function use for checking the consistency of the collected hint.
* \param none
* \return returns a bool value based on the outcome of the consistency test.
*
* This function checks if the provided hint is properly filled with 'key' and 'value' fields. If these conditions are not met then
* then the hint is consider inconsistent.
* 
*/
bool check_consistency()
{
    ROS_INFO("Checking Consistency  of hint.");
    if(hint.key != "" && hint.key !="-1" && hint.value != "" && hint.value !="-1" && hint.ID >= 0)
        return true;
    else
        return false;
}

/**
* \brief The function use for loading the hints in ontology knowlegde base (KB).  
* \param none
* \return returns a bool variable; 'True' if the hints are successfully loaded in the knowlegde base. 
*
* This function load hints in the ontology knowlegde base (KB) using the 'hint_loader_service' service.
* 
*/
bool load_hint()
{
    exp_assignment3::HintLoader hint_loader_srv;
    hint_loader_srv.request.req.ID = hint.ID;
    hint_loader_srv.request.req.key = hint.key;
    hint_loader_srv.request.req.value = hint.value;
    ROS_INFO("loading hint.");
    if (hint_loader_client.call(hint_loader_srv))
    {
        if(hint_loader_srv.response.res)
        {
            ROS_INFO("Successfully loaded hint.");
            return true;
        }
    }
    else
    {
      ROS_ERROR("Failed to call service request_hint_collector for hint");
      return false;
    } 
}

/**
* \brief The function use for starting the ARMOR reasoner. 
* \param none
* \return returns a bool variable; 'True' if the reasoner is successfully started. 
*
* The function request to start the ARMOR reasoner using the 'hint_loader_service' service.
* 
*/
bool start_reasoner()
{
    exp_assignment3::HintLoader check_correctness_srv;
    check_correctness_srv.request.req.ID = -2;
    check_correctness_srv.request.req.key = "REASON";
    check_correctness_srv.request.req.value = "";
    if (hint_loader_client.call(check_correctness_srv))
    {
        if(check_correctness_srv.response.res)
        {
            ROS_INFO("Successfully start the reasoner.");
            return true;
        }
    }
    else
    {
      ROS_ERROR("Failed to call service request_hint_collector for starting the reasoner");
      return false;
    }
    
}

/**
* \brief The function is use to check if there is/are complete hypothesis(es) in knowlegde base.
* \param none
* \return returns a bool variable; 'True' if there is a deduced hypothesis which is complete. 
*
* This function checks if the deduced hypothesis based on the previously load hints is complete or not.
* 
*/
bool check_completeness()
{
    exp_assignment3::HintLoader check_completeness_srv;
    check_completeness_srv.request.req.ID = -11;
    check_completeness_srv.request.req.key ="COMPLETED";
    check_completeness_srv.request.req.value = "";
    if (hint_loader_client.call(check_completeness_srv))
    {
        if(check_completeness_srv.response.res)
        {
            ROS_INFO("Complete Hypothesis Found");
            return true;
        }
        else
        {
            ROS_INFO("Complete Hypothesis Not Found");
            return false;
        }
    }
    else
    {
      ROS_ERROR("Failed to call service request_hint_collector for starting the reasoner");
      return false;
    }
    
}

/**
* \brief A callback function for the 'request_hint_collector' service client.
* \param req request arguement of the service 'request_hint_collector' with data type exp_assignment3::HintCollector::Request 
* \param res response arguement of the service 'request_hint_collector' with data type exp_assignment3::HintCollector::Response
* \return returns a bool variable.
*
* This function respond to the client's request by collecting and storing the hint. Upon successful collection of hint, the node
* check if hint is consistent and then load it in the armor knowlegde base. Onc the hint is loaded then it checks if there is any
* complete hypothesis deduced by the reasoner. If there is a complete hypothesis then it check if it is also correct.
* 
*/
bool collect_hint(exp_assignment3::HintCollector::Request  &req,
                  exp_assignment3::HintCollector::Response &res)
{
    //ROS_INFO("Got the request %s", req.req.c_str());
    
    if(req.req == "collect")
    {   
        ROS_INFO("recieved aruco_marker_id :%d\n",req.markerId);
        exp_assignment3::Marker srv_get_hint;
        srv_get_hint.request.markerId = req.markerId;
        //ROS_INFO("Sending markerId as request msg : %d",srv_get_hint.request.markerId);
        ros::ServiceClient oracle_hint_client = (ros::ServiceClient)*clientPtr_;
        
        ROS_INFO("sending request to collect hint");
        if(oracle_hint_client.call(srv_get_hint))
        {
            hint = srv_get_hint.response.oracle_hint;
            ROS_INFO("Assigning values to Hint1 -> (ID: %d, key: %s, value: %s)", hint.ID, 
                                                    hint.key.c_str(), hint.value.c_str());

            if(check_consistency())
            {
                ROS_INFO("Hint is ready to be loaded.");
                if(load_hint())
                {
                    if(start_reasoner())
                    {
                        if(check_completeness())
                        {
                            ROS_INFO("Complete hypothesis is found. Now checking correctness.");
                            res.hintStatement = "";
                            res.hintLoaded = true;
                            res.completeHypo = true;
                        }
                        else{ ROS_INFO("Hints are not complete. Lets go again."); } 
                    }
                    else{ ROS_INFO("There was an error in starting the reasoner"); }   
                }
                else{ ROS_INFO("There was an error in loading the hint"); }
            }
            else { ROS_INFO("Hint is not consistent"); }
        }
        res.hintStatement = "";
        res.hintLoaded = true;
        res.completeHypo = false;
    }
    else if(req.req == "check_correctness")
    {
        ROS_INFO("checking correctness.");
        exp_assignment3::HintLoader check_correctness_srv;
        check_correctness_srv.request.req.ID = -2;
        check_correctness_srv.request.req.key = "CORRECTNESS";
        check_correctness_srv.request.req.value = "";
        if (hint_loader_client.call(check_correctness_srv))
        {
            if(check_correctness_srv.response.res)
            {
                res.hintStatement = check_correctness_srv.response.hintStatement;
                res.hintLoaded = true;
                res.completeHypo = true;
            }
            else
            {
                res.hintStatement = "";
                res.hintLoaded = true;
                res.completeHypo = true;
            }
        }
        else
        {
          ROS_ERROR("Failed to call service request_hint_collector for starting the reasoner");
        }
    }
}

/**
* \brief The main function of the node 'hint_collector'
* \param argc an integer arguement  
* \param argv a string double pointer arguement.
*
* \return always return 0 as this function cannot fail.
*
* This function initialize the ros node, initializes the 'request_hint_collector' service and clients
* for 'hint_loader_service' and 'oracle_hint' services. 
* 
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "hint_collector");
  ros::NodeHandle n;
  ros::ServiceClient oracle_hint_client = n.serviceClient<exp_assignment3::Marker>("/oracle_hint");
  clientPtr_ = &oracle_hint_client;
  hint_loader_client = n.serviceClient<exp_assignment3::HintLoader>("hint_loader_service");
  ros::ServiceServer service = n.advertiseService("request_hint_collector", collect_hint);
  std::cout << "'request_hint_collector' service is live.." << std::endl;
  ros::spin();
  return 0;
}
