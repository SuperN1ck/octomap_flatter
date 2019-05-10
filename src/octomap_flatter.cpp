#include <octomap_flatter.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <cstdint>

#include <visualization_msgs/Marker.h>

namespace octflat
{
OctomapFlatter::OctomapFlatter(ros::NodeHandle &nh, ros::NodeHandle &nh_private) : nh_(nh),
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
    bounding_box_pub_ = nh.advertise<visualization_msgs::Marker>("/octomap_flatter_bounding_box", 1);
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
    octomap::OcTree *m_octomap = static_cast<octomap::OcTree *>(m_abstract_octomap);
    ROS_INFO_STREAM("Received tree with " << m_octomap->calcNumNodes() << " nodes");

    double min_Z = std::numeric_limits<double>::max();

    for (octomap::OcTree::leaf_iterator it = m_octomap->begin_leafs(), end = m_octomap->end_leafs(); it != end; ++it)
    {
        if (it.getZ() >= min_Z)
            continue;

        min_Z = it.getZ();
    }

    ROS_INFO_STREAM("min_Z: " << min_Z);

    tf::StampedTransform transform;
    try
    {
        transform_listener_.lookupTransform("/world", "/base_link_estimate",
                                            octomap_msg->header.stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    /* Construct Height Image */
    // - Translation: [0.267921, 2.43532, 2.16394]
    // - Rotation: in+ Quaternion [-0.286553, 0.576745, -0.163858, 0.747263]

    ROS_INFO_STREAM("Time Stamp: " << octomap_msg->header.stamp);

    // std::cout << transform << std::endl;
    tf::Vector3 v = transform.getOrigin();
    tf::Quaternion q = transform.getRotation();
    std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
              << q.getZ() << ", " << q.getW() << "]" << std::endl;

    // Roll: Around X-Axis
    // Pitch: Around Y-Axis
    // Yaw: Around Z-Axis
    double roll, pitch, yaw;
    transform.getBasis().getRPY(roll, pitch, yaw);
    std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", "
              << q.getZ() << ", " << q.getW() << "]" << std::endl
              << "            in RPY (radian) [" << roll << ", " << pitch << ", " << yaw << "]" << std::endl
              << "            in RPY (degree) [" << roll * 180.0 / M_PI << ", " << pitch * 180.0 / M_PI << ", " << yaw * 180.0 / M_PI << "]" << std::endl;

    // TODO: Get from config
    double flattening_width = 1.0;
    double flattening_height = 2.0;

    /* Calculate box that captures desired view in world frame because we got the previous transform in world frame */
    /* Wrong Method? */
    // if (yaw < 0)
    // {
    //     yaw = 90 + yaw; // --> 90 + (-20) = 70
    // }

    // ROS_INFO_STREAM("Yaw: " << yaw);

    // double a_1 = cos(yaw) * flattening_width;
    // double a_2 = sin(yaw) * flattening_height;
    // double b_1 = cos(yaw) * flattening_height;
    // double b_2 = sin(yaw) * flattening_width;

    // ROS_INFO_STREAM("a_1: " << a_1 << " a_2: " << a_2 << " b_1: " << b_1 << " b_2: " << b_2);

    // double x_length = a_1 + a_2;
    // double y_length = b_1 + b_2;

    // double x_origin = v.getX() + a_2;
    // double y_origin = v.getY() - b_2 / 2;

    // ROS_INFO_STREAM("x_origin: " << x_origin << " y_origin: " << y_origin);

    // octomap::point3d start_box(x_origin, y_origin, min_Z);
    // octomap::point3d end_box(x_origin - x_length, y_origin + y_length, v.getZ());

    /* Better Method */
    std::vector<tf::Vector3> box_corners{tf::Vector3(v.getX(), v.getY() + flattening_width / 2, 0),
                                          tf::Vector3(v.getX(), v.getY() - flattening_width / 2, 0),
                                          tf::Vector3(v.getX() + flattening_height, v.getY() + flattening_width / 2, 0),
                                          tf::Vector3(v.getX() + flattening_height, v.getY() - flattening_width / 2, 0)};
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::min();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::min();
    
    for (tf::Vector3 point : box_corners)
    {
        ROS_INFO_STREAM("Before rotating: " << point);
        point.rotate(tf::Vector3(0., 0., 1.), yaw);
        ROS_INFO_STREAM("After rotating: " << point);
        x_min = std::min(point.getX(), x_min);
        x_max = std::max(point.getX(), x_max);
        y_min = std::min(point.getY(), y_min);
        y_max = std::max(point.getY(), y_max);
    }

    octomap::point3d start_box(x_min, y_min, min_Z);
    octomap::point3d end_box(x_max, y_max, v.getZ());

    ROS_INFO_STREAM("Bounding box: (" << x_min << ", " << y_min << ") --> (" << x_max << ", " << y_max << ")");

    publish_bounding_box(start_box, end_box);

    /* Create data array */
    // TODO Check for correct frame direction
    // double resolution = m_octomap->getResolution();
    // uint32_t image_width = end_box.x() - start_box.x() / resolution;
    // uint32_t image_height = end_box.y() - start_box.y() / resolution;

    // ROS_INFO_STREAM("Image Width: " << image_width << " Image Height: " << image_height);

    // /* Taken from sensor_msgs/Images documentation: uint8[] data # actual matrix data, size is (step * rows) */
    // uint8_t data[image_width * image_height];
    // /* Parallelize this loop for more speed */
    // for (uint y = 0; y < image_height; ++y)
    // {
    //     for (uint x = 0; x < image_width; ++x)
    //     {
    //         array_idx = y * image_width + x;
    //         octomap::KeyRay key_ray;
    //         m_octomap->computeRayKeys(
    //             octomap::point3d(x_min + x * resolution, y_min + y * resolution, v.getZ()),
    //             octomap::point3d(0, 0, -1),
    //             key_ray);
    //         ROS_INFO("------------------------------");
    //         ROS_INFO_STREAM("Key Ray Size: " << key_ray.size());
    //         for (auto key : key_ray)
    //         {
    //             octomap::OcTreeNode* node = m_octomap->search(key);
    //             if (node && node->getOccupancy() > m_octomap->getOccupancyThres())
    //                 ROS_INFO_STREAM("Node: " << node->getOccupancy());
    //         }
    //     }
    // }

    ROS_INFO_STREAM("Occupancy Threshold: " << m_octomap->getOccupancyThres());

    for (octomap::OcTree::leaf_iterator it = m_octomap->begin_leafs(), end = m_octomap->end_leafs(); it != end; ++it)
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

void OctomapFlatter::publish_bounding_box(octomap::point3d start_box, octomap::point3d end_box)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now(); // TODO get from image

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "octomap_flatter vizualization";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (start_box.x() + end_box.x()) / 2;
    marker.pose.position.y = (start_box.y() + end_box.y()) / 2;
    marker.pose.position.z = (start_box.z() + end_box.z()) / 2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = end_box.x() - start_box.x();
    marker.scale.y = end_box.y() - start_box.y();
    marker.scale.z = end_box.z() - start_box.z();

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.2;

    marker.lifetime = ros::Duration();

    bounding_box_pub_.publish(marker);
}

} // namespace octflat