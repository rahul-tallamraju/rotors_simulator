/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <uav_msgs/uav_pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>


double waypoint[3], poi[3], currentPosition[3];

void wayPointCallback(const uav_msgs::uav_pose::ConstPtr& msg)
{
    waypoint[0] = msg->position.x;
    waypoint[1] = -msg->position.y;
    waypoint[2] = -msg->position.z;
    
    poi[0] = msg->POI.x;
    poi[1] = -msg->POI.y;
    poi[2] = -msg->POI.z;
    
}

void selfPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    currentPosition[0] = msg->pose.pose.position.x;
    currentPosition[1] = msg->pose.pose.position.y;
    currentPosition[2] = msg->pose.pose.position.z;

    
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh("");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Started waypoint_publisher.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);
  
  ros::Subscriber subNMPCwaypoint_ = nh.subscribe("/waypoint_firefly_"+args.at(1), 1000, wayPointCallback); 
  
  ros::Subscriber subSelfPose_ = nh.subscribe("/firefly_"+args.at(1)+"/ground_truth/pose_with_covariance", 1000, selfPoseCallback);   
  
  double delay;

  if (args.size() == 5) {
    delay = 1.0;
  }
  else if (args.size() == 6) {
    delay = std::stof(args.at(5));
  }
  else{
    ROS_ERROR("Usage: waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>]\n");
    return -1;
  }

  const float DEG_2_RAD = M_PI / 180.0;
  const float takeOffHeight = 2.5; //in meter

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();



  // Wait for some time to create the ros publisher.
  ros::Duration(delay).sleep();

  while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
    ROS_INFO("There is no subscriber available, trying again in 1 second.");
    ros::Duration(1.0).sleep();
  }

  ros::Time takeOffStartTime = ros::Time::now();
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    
    if((ros::Time::now() - takeOffStartTime).toSec() < 10.0)  //perform takeoff
    {
        Eigen::Vector3d desired_position(currentPosition[0],currentPosition[1],takeOffHeight);
        double desired_yaw = atan2(poi[1]-currentPosition[1],poi[0]-currentPosition[0]);
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
            desired_yaw, &trajectory_msg);      
        //ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",nh.getNamespace().c_str(),desired_position.x(),desired_position.y(),desired_position.z());
        trajectory_pub.publish(trajectory_msg);    
    }
    else
    {
        
        Eigen::Vector3d desired_position(waypoint[0],waypoint[1],waypoint[2]);

        double desired_yaw;

        if(fabs(0-currentPosition[0])>0.01)
            desired_yaw = atan2(0-currentPosition[1],0-currentPosition[0]);
        else
            desired_yaw = atan2(0-currentPosition[1],0.01);
        
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
            desired_yaw, &trajectory_msg);      
        
        //ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",nh.getNamespace().c_str(),desired_position.x(),desired_position.y(),desired_position.z());
        
        trajectory_pub.publish(trajectory_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }  
  

  return 0;
}
