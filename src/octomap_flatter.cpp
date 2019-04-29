#include <octomap_flatter.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>

namespace octflat
{
OctomapFlatter::OctomapFlatter(ros::NodeHandle &nh, ros::NodeHandle &nh_private) : 
    nh_(nh),
    nh_private_(nh_private),
    octomap_sub_(nh, "/octomap_full", 1),
    projected_map_sub_(nh, "/projected_map", 1),
    synchronizer_(SyncPolicy(10), octomap_sub_, projected_map_sub_),
    config_()
{
    /* Start Tracking */
    /* In */
    connection_ = synchronizer_.registerCallback(boost::bind(&OctomapFlatter::octomapCallback, this, _1, _2));
    /* Out */
    octomap_pub_ = nh.advertise<octomap_msgs::Octomap>("/octomap_flattened", 1);
    /* Parameters */
    dynamic_reconfigure::Server<octomap_flatter::OctoFlatterConfig>::CallbackType f;
    f = boost::bind(&OctomapFlatter::dynamicParameterCallback, this, _1, _2);
    param_server_.setCallback(f);

    ROS_INFO("started octomap_flatter ...");
}

void OctomapFlatter::octomapCallback(const octomap_msgs::Octomap::ConstPtr &octomap_msg,
                                     const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid_msg)
{
    ROS_INFO("Flattening Octomap");
    // http://docs.ros.org/kinetic/api/octomap_msgs/html/msg/Octomap.html
    octomap::AbstractOcTree *m_abstract_octomap = octomap_msgs::fullMsgToMap(*octomap_msg);
    octomap::ColorOcTree *m_octomap = static_cast<octomap::ColorOcTree *>(m_abstract_octomap);
    ROS_INFO_STREAM("Received tree with " << m_octomap->calcNumNodes() << " nodes");

    double min_Z = std::numeric_limits<double>::max();

    for (octomap::ColorOcTree::leaf_iterator it = m_octomap->begin_leafs(), end = m_octomap->end_leafs(); it != end; ++it)
    {
        if (it.getZ() >= min_Z)
            continue;

        min_Z = it.getZ();
    }

    ROS_INFO_STREAM("min_Z: " << min_Z);

    for (octomap::ColorOcTree::leaf_iterator it = m_octomap->begin_leafs(), end = m_octomap->end_leafs(); it != end; ++it)
    {
        // Access various node proprertie e.g.:
        // std::cout << "Node center: " << it.getCoordinate() << std::endl;
        // std::cout << "Node size: " << it.getSize() << std::endl;
        // std::cout << "Node value: " << it->getValue() << std::endl;
        octomap::point3d coord = it.getCoordinate();

        if (coord.z() > min_Z + config_.height_threshold)
            continue;

        octomap::OcTreeKey key = it.getKey();
        // ROS_INFO_STREAM("Key: " << key[0] << " " << key[1] << " " << key[2]);
        /* Delete current node */
        m_octomap->deleteNode(key);

        // ROS_INFO_STREAM("Old coordinate: " << coord);
        // m_octomap->updateNode(key, false, true); // This didn't worked
        /* Push Node to ground */
        coord.z() = min_Z;
        /* Set it filled */
        m_octomap->updateNode(coord, true, true);
    }

    m_octomap->updateInnerOccupancy();
    m_octomap->toMaxLikelihood();
    m_octomap->prune();
    /* Create Message */
    octomap_msgs::Octomap out_msg;
    octomap_msgs::fullMapToMsg(*m_octomap, out_msg);
    out_msg.header = octomap_msg->header;
    // out_msg.header.stamp = ros::Time::now(); // This creates the new octomap at the new timestamp

    /* Publish & Clean up */
    ROS_INFO_STREAM("Publishing tree with " << m_octomap->calcNumNodes() << " nodes");
    octomap_pub_.publish(out_msg);
    delete m_octomap;
}

void OctomapFlatter::dynamicParameterCallback(octomap_flatter::OctoFlatterConfig &config, uint32_t level)
{
    config_ = config;
}


} // namespace octflat