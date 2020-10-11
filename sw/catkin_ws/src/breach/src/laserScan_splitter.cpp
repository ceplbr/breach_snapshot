#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

bool flag_callback = false;

sensor_msgs::LaserScan laserScan_data = sensor_msgs::LaserScan();


void scanCallback (const sensor_msgs::LaserScanConstPtr &scan_msg)
{
    laserScan_data.header = scan_msg->header;
    laserScan_data.range_min = scan_msg->range_min;
    laserScan_data.range_max = scan_msg->range_max;
    laserScan_data.angle_increment = scan_msg->angle_increment;
    laserScan_data.time_increment = scan_msg->time_increment;
    laserScan_data.scan_time = scan_msg->scan_time;
    laserScan_data.header.frame_id = "laser";
    laserScan_data.angle_min =scan_msg->angle_min;
    laserScan_data.angle_max = scan_msg->angle_max;

    // TODO - also copy intensity values

    laserScan_data.ranges.resize(360);
    memcpy(&laserScan_data.ranges[0], &scan_msg->ranges[0], 360*sizeof(int));

    flag_callback = true;
}

int main(int argc, char **argv)
{
    ROS_INFO ("Starting ScanSplitter");

    ros::init(argc, argv, "ScanSplitter");
    ros::NodeHandle nh;

    ros::Subscriber scan_subscriber_ = nh.subscribe ("laserscan", 10, &scanCallback);
    ros::Publisher  scan_publisher = nh.advertise<sensor_msgs::LaserScan>("scan", 10);

    ros::Rate loop_rate(100);
    while(ros::ok()) {
        if (flag_callback) {
            flag_callback = false;

            laserScan_data.angle_min += 1.5708;
            laserScan_data.angle_max += 1.5708;

            for(int i = 225; i < 315; i++) {
                if(laserScan_data.ranges[i] < 0.7) {
                    laserScan_data.ranges[i] = std::numeric_limits<float>::infinity();
                }
            }

            scan_publisher.publish(laserScan_data);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }


    ROS_INFO ("Destroying ScanSplitter");
    return 0;
}

