/**
* \file simple_action.cpp
* \brief This files contains code for the 'simple_action' node.
* \author Elham Mohammadi
* \version 1.0
* \date 25.01.2023
*
* \details
* 
* Action Clients / Services: <BR>
* /request_hint_collector
* /request_follow_marker
* /MoveBaseAction
* 
* Subscribes to:<BR>
* /odom
* 
*  
* Description :
* 
* This node implements a state machine which curates the robot behavior. The node recieves the odometry 
* data of the robot from the topic '/odom' and keeps the track of robot's position. Based on robot's
* position the 'state machine' move in the following states, 1) Reach center of the room[x]. 2) 
* start exploration in room[x] and collect hint. 3) If hint is detected then call '/hint_collector' 
* service to collect and load the hint in armor knowlegde base (KB).
*  
*/


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/Odometry.h"
#include <erl2/SetOrien.h>
#include <exp_assignment3/HintCollector.h>
#include <exp_assignment3/FollowMarker.h>
#include <cmath>

///<initializing the action client handle for ROS navigation stack 'MoveBase' service.  
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
///<generating message instance of MoveBaseGoal.
move_base_msgs::MoveBaseGoal goal;
///<global variable used to keep the value of robot's current position x-cordinates.
double robot_curr_pos_x_ = 0.0;
///<global variable used to keep the value of robot's current position y-cordinates.
double robot_curr_pos_y_ = 0.0;

///<pointer for a hint collector service client
ros::ServiceClient *hint_collector_clientPtr_;
///<pointer for a follow marker server client
ros::ServiceClient *follow_marker_clientPtr_;


///< Global variable used to keep the state count of state machine.
int counter_ = 0;
///< Global variable used to keep the count of total hints collected in each room.
int colllected_hint_ = 0;
///< Global variable used to keep the value of distance between goal and robot's current position.
double dist = 0;
double dist2  = 0;


bool go_to_centre()
{

  dist2 = sqrt(pow((0.0 - robot_curr_pos_x_), 2) + 
                                    pow((-1.0 - robot_curr_pos_x_), 2));

  MoveBaseClient ac("move_base", true);
  // assigning the new goal to move_base action service
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = -1.0;
  goal.target_pose.pose.orientation.w = -1.0;
        
  ROS_INFO("goal(x2,y2) = (%f,%f), current_pos(x1,y1) = (%f,%f)",0.0,-1.0,
                                          robot_curr_pos_x_,robot_curr_pos_y_);
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  while (dist2 > 1.0)
  {
    dist2 = sqrt(pow((0.0 - robot_curr_pos_x_), 2) + pow((-1.0 - robot_curr_pos_y_), 2));
    ROS_INFO("goal(x2,y2) = (%f,%f), current_pos(x1,y1) = (%f,%f)",0.0,-1.0,
                                          robot_curr_pos_x_,robot_curr_pos_y_);
    ROS_INFO("dist2 = %f",dist2);      
  }

  //cancelling the previous goal
  ROS_INFO("Goal is completed.");
  ac.cancelAllGoals();
  ros::Duration(1.0).sleep();
  return true;   
}

/**
* \brief The function is used as a callback for '\odom' topic subscriber.
* \param none
* \return none
*
* This function recieves the odometry data of the robot and based on that it implements the state machine. 
* The statement machine is consist of following states, 1) Reach center of the room[x]. 2) start exploration
* in room[x] and collect hint. 3) If hint is detected then call '/hint_collector' service to collect and load
* the hint in armor knowlegde base (KB).
* 
*/
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_curr_pos_x_ = msg->pose.pose.position.x;
  robot_curr_pos_y_ = msg->pose.pose.position.y;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true); 

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  int target_list_[6][2] = {{-4,-3}, {-4,2}, {-4,7}, {5,-7}, {5,-3}, {5,1}};
    
  if(counter_ < 2)
  {
    if(counter_ == 0)
    {
      // creating a request msg for action server
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = target_list_[counter_][0];
      goal.target_pose.pose.position.y = target_list_[counter_][1];
      goal.target_pose.pose.orientation.w = 1.0;

      // sending goal
      ac.sendGoal(goal);
      counter_++;    
    }

    dist = sqrt(pow((goal.target_pose.pose.position.x - robot_curr_pos_x_), 2) + 
                                    pow((goal.target_pose.pose.position.y - robot_curr_pos_y_), 2));

    ROS_INFO("goal(x2,y2) = (%f,%f), current_pos(x1,y1) = (%f,%f)",goal.target_pose.pose.position.x,
                                goal.target_pose.pose.position.y ,robot_curr_pos_x_,robot_curr_pos_y_);
    ROS_INFO("dist = %f",dist);

    if(dist < 1.0)
    {

      //cancelling the previous goal
      ROS_INFO("Cancelling the goal..");
      ac.cancelAllGoals();
      ros::Duration(1.0).sleep();     

      //generating srv msg for the follow_marker service
      exp_assignment3::FollowMarker srv_follow_marker;
      srv_follow_marker.request.req = "follow";
      ros::ServiceClient follow_marker_client = (ros::ServiceClient)*follow_marker_clientPtr_;

      while(colllected_hint_ < 5)  
      {
        ROS_INFO("Requesting follow_marker service..");
        
        // call service for video survoying.
        if(follow_marker_client.call(srv_follow_marker))
        {
          ROS_INFO("Follow_marker service is done. marker_id :%d",srv_follow_marker.response.markerid);
          
          //generating srv msg for the hint_collector service to collect the hint.
          exp_assignment3::HintCollector srv_hint_collector;
          srv_hint_collector.request.req = "collect";
          srv_hint_collector.request.markerId = srv_follow_marker.response.markerid;
          ros::ServiceClient hint_collector_client = (ros::ServiceClient)*hint_collector_clientPtr_;

          // calling the service.
          if(hint_collector_client.call(srv_hint_collector))
          {
            // checking if the hint is loaded successfully.
            if(srv_hint_collector.response.hintLoaded)
            {
              ROS_INFO("hint has been laoded successfully.");
              colllected_hint_++;
              
              // checking if there is any complete hypothesis.
              if(srv_hint_collector.response.completeHypo)
              {
                ROS_INFO("Complete Hypothesis has been found now going to check its correctness.");
                
                //Making the robot to go at the centre (0.0,-1.0)
                go_to_centre();

                // //generating srv msg for the hint_collector service.
                exp_assignment3::HintCollector srv_hint_collector;
                srv_hint_collector.request.req = "check_correctness";
                srv_hint_collector.request.markerId = -1;

                // calling the service again
                if(hint_collector_client.call(srv_hint_collector))
                {
 
                  // checking if the hypothesis in correct or not.
                  if(srv_hint_collector.response.hintStatement != "")
                  {
                    ROS_INFO("Hint Statement : %s",srv_hint_collector.response.hintStatement);
                  }
                  else
                  {
                    ROS_INFO("The Hypothesis is not correct. Let's go on..");
                  }
                }
              }
            }
          } 
        } 
        else
        {
          ROS_INFO("Problem in calling the 'follow_marker' service."); 
        }        
      }
      // assigning the new goal to move_base action service
      std::cout << "Reached target goal" << std::endl;
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = target_list_[counter_][0];
      goal.target_pose.pose.position.y = target_list_[counter_][1];
      goal.target_pose.pose.orientation.w = -1.0;
            
      ROS_INFO("Current target goal (x,y) = (%f,%f)",goal.target_pose.pose.position.x,
                                                         goal.target_pose.pose.position.y);
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);
      colllected_hint_ = 0;
      counter_++;
    }
  }
  else
  {
    ROS_INFO("All the goals are achieved.");
    ros::shutdown();
  }   
}

/**
* \brief The main function of the node 'simple_action'
* \param argc an integer arguement  
* \param argv a string double pointer arguement.
*
* \return always return 0 as this function cannot fail.
*
* This function initialize the ros node, server for 'request_hint_collector' service, client for
* 'request_follow_marker' service and subscriber for the topic 'odom'. 
* 
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_nav_action");
  ros::NodeHandle n;

  ros::ServiceClient hint_collector_client = n.serviceClient<exp_assignment3::HintCollector>("request_hint_collector");
  hint_collector_clientPtr_ = &hint_collector_client;

  ros::ServiceClient follow_marker_client = n.serviceClient<exp_assignment3::FollowMarker>("request_follow_marker");
  follow_marker_clientPtr_ = &follow_marker_client;

  ros::Subscriber sub = n.subscribe("odom", 10, odom_callback);

  ros::spin();

  return 0;
}
