/*
 * Copyright (c) 2012, Daniel Perea Strom
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(fullSource)%
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseStamped.h>

using namespace visualization_msgs;

ros::Publisher pub;
ros::Publisher pub2;

void processFeedback2(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_INFO_STREAM(
      feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", "
          << feedback->pose.position.z);
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = feedback->pose.position.x;
  msg.pose.position.y = feedback->pose.position.y;
  msg.pose.position.z = feedback->pose.position.z;

  msg.pose.orientation.x = feedback->pose.orientation.x;
  msg.pose.orientation.y = feedback->pose.orientation.y;
  msg.pose.orientation.z = feedback->pose.orientation.z;
  msg.pose.orientation.w = feedback->pose.orientation.w;

  pub2.publish(msg);
}


void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  ROS_INFO_STREAM(
      feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", "
          << feedback->pose.position.z);
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.pose.position.x = feedback->pose.position.x;
  msg.pose.position.y = feedback->pose.position.y;
  msg.pose.position.z = feedback->pose.position.z;

  msg.pose.orientation.x = feedback->pose.orientation.x;
  msg.pose.orientation.y = feedback->pose.orientation.y;
  msg.pose.orientation.z = feedback->pose.orientation.z;
  msg.pose.orientation.w = feedback->pose.orientation.w;

  pub.publish(msg);
}


int main(int argc, char** argv)
{
ros::init(argc, argv, "goal_interactive_marker");

ros::NodeHandle nh("~");
double goal_init_pose_x_;
double goal_init_pose_y_;
double goal_init_pose_z_;
nh.param("goal_init_pose_x", goal_init_pose_x_, 0.0);
nh.param("goal_init_pose_y", goal_init_pose_y_, 0.0);
nh.param("goal_init_pose_z", goal_init_pose_z_, 0.0);

double start_init_pose_x_;
double start_init_pose_y_;
double start_init_pose_z_;
nh.param("start_init_pose_x", start_init_pose_x_, 0.0);
nh.param("start_init_pose_y", start_init_pose_y_, 0.0);
nh.param("start_init_pose_z", start_init_pose_z_, 0.0);

pub = nh.advertise<geometry_msgs::PoseStamped>("/goal_pose", 5);
pub2 = nh.advertise<geometry_msgs::PoseStamped>("/currentPos", 5, true);

// create an interactive marker server on the topic namespace goal_interactive_marker
interactive_markers::InteractiveMarkerServer server("goal_interactive_marker");

// create an interactive marker for our server
InteractiveMarker int_marker;
int_marker.header.frame_id = "/map";
int_marker.name = "goal";
int_marker.description = "Set goal pose";
int_marker.pose.position.x = goal_init_pose_x_;
int_marker.pose.position.y = goal_init_pose_y_;
int_marker.pose.position.z = goal_init_pose_z_;

// create a grey box marker
Marker box_marker;
box_marker.type = visualization_msgs::Marker::CUBE;
box_marker.scale.x = 0.05;
box_marker.scale.y = 0.05;
box_marker.scale.z = 0.05;
box_marker.color.r = 0.5;
box_marker.color.g = 0.5;
box_marker.color.b = 0.5;
box_marker.color.a = 1.0;

InteractiveMarkerControl box_control;
box_control.always_visible = true;
box_control.markers.push_back(box_marker);
int_marker.controls.push_back(box_control);

InteractiveMarkerControl control;

control.orientation.w = 1;
control.orientation.x = 0;
control.orientation.y = 1;
control.orientation.z = 0;
control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
int_marker.controls.push_back(control);
control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
int_marker.controls.push_back(control);

server.insert(int_marker);
server.setCallback(int_marker.name, &processFeedback);

// 'commit' changes and send to all clients
server.applyChanges();

// Current postion marker

// create an interactive marker for our server
// start is now at 5.66191, -1.07912, 0.468205

InteractiveMarker int_marker2;
int_marker2.header.frame_id = "/map";
int_marker2.name = "start";
int_marker2.description = "Set current start pose";
int_marker2.pose.position.x = start_init_pose_x_;
int_marker2.pose.position.y = start_init_pose_y_;
int_marker2.pose.position.z = start_init_pose_z_;

// create a grey box marker
Marker box_marker2;
box_marker2.type = visualization_msgs::Marker::CUBE;
box_marker2.scale.x = 0.05;
box_marker2.scale.y = 0.05;
box_marker2.scale.z = 0.05;
box_marker2.color.r = 0.5;
box_marker2.color.g = 0.5;
box_marker2.color.b = 0.5;
box_marker2.color.a = 1.0;

InteractiveMarkerControl box_control2;
box_control2.always_visible = true;
box_control2.markers.push_back(box_marker2);
int_marker2.controls.push_back(box_control2);

InteractiveMarkerControl control2;

control2.orientation.w = 1;
control2.orientation.x = 0;
control2.orientation.y = 1;
control2.orientation.z = 0;
control2.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
int_marker2.controls.push_back(control2);
control2.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
int_marker2.controls.push_back(control2);

server.insert(int_marker2);
server.setCallback(int_marker2.name, &processFeedback2);

// 'commit' changes and send to all clients
server.applyChanges();

// Publishing ONLY inital start pose (so it is available for other nodes to use)
geometry_msgs::PoseStamped msg;
msg.header.stamp = ros::Time::now();
msg.pose.position.x = start_init_pose_x_;
msg.pose.position.y = start_init_pose_y_;
msg.pose.position.z = start_init_pose_z_;

pub2.publish(msg);



// start the ROS main loop
ros::spin();
}
// %Tag(fullSource)%
