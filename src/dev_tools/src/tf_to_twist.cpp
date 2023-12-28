#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>



int main(int argc, char** argv){

  ros::init(argc, argv, "tf_to_twist");

  ros::NodeHandle node;

  std::string source_frame, target_frame;
  double frequency;

  // Get parameters
  node.param<std::string>("source_frame", source_frame, "base_link");
  node.param<std::string>("target_frame", target_frame, "odom");
  node.param<double>("frequency", frequency, 5.0);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Publisher twist_pub = node.advertise<geometry_msgs::TwistStamped>("/dev_tools/twist_from_tf", 10);

  ros::Rate rate(frequency);

  geometry_msgs::TransformStamped prev_transformStamped;

  ROS_INFO("Started publishing twist messages from %s to %s at %f Hz", source_frame.c_str(), target_frame.c_str(), frequency);

  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform(target_frame, source_frame,
                               ros::Time(0));
    ROS_INFO_ONCE("Succesfully getting transforms from %s to %s...", source_frame.c_str(), target_frame.c_str());
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      rate.sleep();
      continue;
    }

    // Check if the transform has changed
    if (transformStamped.header.stamp == prev_transformStamped.header.stamp) {
      rate.sleep();
      continue;
    }

    geometry_msgs::TwistStamped twist_msg;
    
    // Populate the TwistStamped message
    twist_msg.header.stamp = transformStamped.header.stamp;
    twist_msg.header.frame_id = target_frame;

    // Compute linear and angular velocities if previous pose is available
    if (!prev_transformStamped.header.frame_id.empty()) {
      double dt = (transformStamped.header.stamp - prev_transformStamped.header.stamp).toSec();

      tf2::Vector3 curr_pos, prev_pos;
      tf2::fromMsg(transformStamped.transform.translation, curr_pos);
      tf2::fromMsg(prev_transformStamped.transform.translation, prev_pos);
      tf2::Vector3 vel = (curr_pos - prev_pos) / dt;

      // Transform linear velocity to vehicle's frame
      tf2::Quaternion curr_rot;
      tf2::fromMsg(transformStamped.transform.rotation, curr_rot);
      tf2::Vector3 vel_vehicle_frame = tf2::quatRotate(curr_rot.inverse(), vel);
      tf2::convert(vel_vehicle_frame, twist_msg.twist.linear);

      tf2::Quaternion prev_rot;
      tf2::fromMsg(prev_transformStamped.transform.rotation, prev_rot);
      tf2::Quaternion d_rot = curr_rot * prev_rot.inverse();
      tf2::Matrix3x3 m(d_rot);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      tf2::Vector3 ang_vel(roll, pitch, yaw);
      ang_vel /= dt;
      tf2::convert(ang_vel, twist_msg.twist.angular);
    }

    twist_pub.publish(twist_msg);

    // Store current transform for next iteration
    prev_transformStamped = transformStamped;

    rate.sleep();
  }
  return 0;
};