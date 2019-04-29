#ifndef OCTOMAP_FLATTER_H_
#define OCTOMAP_FLATTER_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace octflat
{
typedef message_filters::sync_policies::ApproximateTime<octomap_msgs::Octomap,
                                                        nav_msgs::OccupancyGrid>
    SyncPolicy;

class OctomapFlatter
{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    message_filters::Subscriber<octomap_msgs::Octomap> octomap_sub_;
    message_filters::Subscriber<nav_msgs::OccupancyGrid> projected_map_sub_;
    message_filters::Synchronizer<SyncPolicy> synchronizer_;
    message_filters::Connection connection;

    ros::Publisher octomap_pub_;

public:
    OctomapFlatter(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &octomap_msg,
                         const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid_msg);
};
} // namespace octflat

#endif