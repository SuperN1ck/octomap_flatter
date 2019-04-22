/**
 * Copyright?
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <octomap_flatter.h>

// #include <.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "octomap_flatter");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    octflat::OctomapFlatter octomap_flatter(nh, nh_private);

    ROS_INFO("started octomap_flatter node ...");

    // ros::Spinner spinner(1);
    // spinner.spin();
    ros::spin();

    return 0;
}
