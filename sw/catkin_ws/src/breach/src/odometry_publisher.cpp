#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "geometry_msgs/Twist.h"
#include <std_msgs/Float64.h>

#define COUNT_PER_METER (1000)
#define WHEEL_DISTANCE  (0.485)

// distance change
double delta_x;
double delta_y;
double delta_th;

// flag for incomming message
int newValue;

// last values
double last_encoder_0;
double last_encoder_1;
double diff_enc_0, diff_enc_1;
double pos_x, pos_y, pos_fi;

// physical model parameters
double countPerMeter;
double w;
double R;

// session duration
double time_begin;
double time_now;
double time_last_publish;
std_msgs::Float64 runtime;

// session distance
std_msgs::Float64 distance;

void dist_vel_callback(const geometry_msgs::Twist& dist_cmd)
{

    double v1, v2, fi, R;

    time_now = ros::Time::now().toSec();

    //~ // cut off bad values
    double dist = sqrt((dist_cmd.linear.x/countPerMeter)*(dist_cmd.linear.x/countPerMeter) + (dist_cmd.linear.y/countPerMeter)*(dist_cmd.linear.y/countPerMeter));
    if( ((fabs(dist_cmd.linear.x - last_encoder_0)) < (1.0*countPerMeter)) &&
        ((fabs(dist_cmd.linear.y - last_encoder_1)) < (1.0*countPerMeter)) ) {
        diff_enc_0 = (dist_cmd.linear.x - last_encoder_0);
        diff_enc_1 = (dist_cmd.linear.y - last_encoder_1);
        distance.data = dist;

        // actual speed from distance diferential
        v1 =  diff_enc_0/countPerMeter;     // left wheel
        v2 =  diff_enc_1/countPerMeter;     // right wheel

        // zero divide protection
        if( fabs(v1) < 0.00001 && fabs(v2) < 0.00001 ) {
            delta_x = 0.0;
            delta_y = 0.0;
            delta_th = 0.0;
        } else {
            if( fabs(v2-v1) < 0.00000001 ) {
                R = 9999.9;
            } else {
                R = (w/2) * ((v1+v2)/(v2-v1));
            }

            fi = (v2-v1)/w;

            // count x, y, z speed
            delta_x = ((v1+v2)/2) * cos(fi);
            delta_y = ((v1+v2)/2) * sin(fi);
            delta_th = fi;
        }

        // newValue flag
        newValue = 1;

    }
    newValue = 1;

    pos_x += delta_x;
    pos_y += delta_y;
    pos_fi += delta_th;

    ROS_INFO("v1: %f, v2: %f",v1, v2);
    ROS_INFO("x: %f, y: %f, z: %f",delta_x, delta_y, delta_th);
    ROS_INFO("x: %f, y: %f, z: %f\n",pos_x, pos_y, pos_fi);

    // save last values
    last_encoder_0 = dist_cmd.linear.x;
    last_encoder_1 = dist_cmd.linear.y;

}



int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  // init nodes
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Subscriber sub = n.subscribe("/breach/distance", 5, dist_vel_callback);
  tf::TransformBroadcaster odom_broadcaster;

  // init values
  countPerMeter = COUNT_PER_METER;
  w             = WHEEL_DISTANCE;
  time_begin = ros::Time::now().toSec();

  double dx     = 0.0;
  double dy     = 0.0;
  double dth    = 0.0;

  double x  = 0.0;
  double y  = 0.0;
  double th     = 0.0;

  last_encoder_0= 0.0;
  last_encoder_1= 0.0;

  double dt     = 0.1;

  newValue      = 0;

  // init time
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  // rate
  ros::Rate r(50.0);//Hz

  while(n.ok())
  {
    ros::spinOnce();              // check for incoming messages

    if( newValue ) {
        // reset flag newValue
        newValue = 0;

        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();

        time_last_publish = ros::Time::now().toSec();

        dx = delta_x * cos(th) - delta_y * sin(th);
        dy = delta_x * sin(th) + delta_y * cos(th);
        dth = delta_th;

        x += dx;
        y += dy;
        th += dth;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;

        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position [odometry]
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = delta_x/dt;
        odom.twist.twist.linear.y = delta_y/dt;
        odom.twist.twist.angular.z = delta_th/dt;

        //publish the message
        odom_pub.publish(odom);
    }
    last_time = current_time;
    r.sleep();
  }
}
