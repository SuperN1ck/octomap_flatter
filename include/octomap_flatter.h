#ifndef OCTOMAP_FLATTER_H_
#define OCTOMAP_FLATTER_H_

#include <ros/ros.h>
#include <ros/console.h>

namespace octflat
{
    class OctomapFlatter
    {
        public:
            OctomapFlatter(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    };
}

#endif