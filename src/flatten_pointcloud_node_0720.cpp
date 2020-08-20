#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <iostream>
#include <math.h>
#include <cmath>
#include <string>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <floor_octomap/StampedString.h>

using namespace Eigen;
using namespace std;

ros::Publisher pub_ground, pub_steps, pub_steps_world_flat;

ros::Publisher pub_steps_space_filtered;

string in_points, in_request, out_ground, out_steps, out_steps_world_flat;
string steps_frame, base_link_frame;

// RANSAC
float downsample_size;
float ransac_dt;

// Octomap
ros::Publisher pub_octomap;
ros::Publisher pub_octomap_flat;
string out_steps_octomap;
string out_steps_octomap_flat;
string octomap_frame;
float octomap_resolution;
ros::Publisher pub_bounding_box;
string out_bounding_box;
vector<double> bounding_box;
vector<double> pre_crop_box;

octomap::OcTree *octree;
octomap::OcTree *octree_flat;
int counter = 0;
int octomap_threshold;

// Camera to World transform
bool tf_ready = false;
tf::TransformListener *tf_listener; 

void change_pointcloud_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<float> rgb)
{
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); it++)
    {
        it->r = rgb[0]; it->g = rgb[1]; it->b = rgb[2];
    }
}

void ransac_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane)
{
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setInputCloud (cloud);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (ransac_dt);
    seg.segment (*inliers, *coefficients);
    pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, *inliers, *plane);
}

void downsample_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new, float voxel_size)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> voxl_grid_filter;
    voxl_grid_filter.setInputCloud(cloud_orig);
    voxl_grid_filter.setLeafSize (voxel_size, voxel_size, voxel_size);
    voxl_grid_filter.filter (*cloud_new);
}

void cropbox_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new, std::vector<double> box_size)
{
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(box_size[0], box_size[2], box_size[4], 1.0));
    boxFilter.setMax(Eigen::Vector4f(box_size[1], box_size[3], box_size[5], 1.0));
    boxFilter.setInputCloud(cloud_orig);
    boxFilter.filter(*cloud_new);
}

void publish_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, ros::Publisher* pub, ros::Time stamp, vector<float> color = {-1})
{
    if (color[0] >= 0)
        change_pointcloud_color(cloud, color);
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = stamp;
    pcl::toROSMsg (*cloud, cloud_msg);
    pub->publish(cloud_msg);
}

void publish_octomap(octomap::OcTree *octree, ros::Publisher* pub, ros::Time stamp)
{
    octree->updateInnerOccupancy();
    octree->toMaxLikelihood();
    octree->prune();
    
    octomap_msgs::Octomap octomap_msg;
    //octomap_msgs::fullMapToMsg(*octree, octomap_msg);
    octomap_msgs::binaryMapToMsg(*octree, octomap_msg);
    octomap_msg.header.frame_id = octomap_frame;
    octomap_msg.header.stamp = stamp;
    pub->publish(octomap_msg);
    delete octree;

    counter = 0;
}

void publish_bounding_box(ros::Time stamp, std::vector<double> bbox)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = base_link_frame;
    marker.header.stamp = stamp;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "floor_octomap vizualization";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (bbox[0] + bbox[1]) / 2;
    marker.pose.position.y = (bbox[2] + bbox[3]) / 2;
    marker.pose.position.z = (bbox[4] + bbox[5]) / 2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = bbox[1] - bbox[0];
    marker.scale.y = bbox[3] - bbox[2];
    marker.scale.z = bbox[5] - bbox[4];

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.2;
    marker.lifetime = ros::Duration();

    pub_bounding_box.publish(marker);
}

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const floor_octomap::StampedString::ConstPtr& req_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        plane_1 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        outliers (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        plane_2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg (*cloud_msg, *whole_cloud);

    /* Crop pointcloud before calculations */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud_crop (new pcl::PointCloud<pcl::PointXYZRGB>);
    cropbox_pointcloud(whole_cloud, whole_cloud_crop, pre_crop_box);

    /* Downsample pointcloud */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud_down (new pcl::PointCloud<pcl::PointXYZRGB>);
    downsample_pointcloud(whole_cloud_crop, whole_cloud_down, downsample_size);
    publish_pointcloud(whole_cloud_down, &pub_steps_space_filtered, cloud_msg->header.stamp, {255,0,0});


    /* Segment major plane */
    pcl::PointIndices::Ptr inliers_1 (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients_1 (new pcl::ModelCoefficients ());
    ransac_pointcloud(whole_cloud_down, inliers_1, coefficients_1, plane_1);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (whole_cloud_down);
    extract.setIndices (inliers_1);
    extract.setNegative (true);//false
    extract.filter (*outliers);


    /* Segment minor plane */
    pcl::PointIndices::Ptr inliers_2 (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients_2 (new pcl::ModelCoefficients ());
    ransac_pointcloud(outliers, inliers_2, coefficients_2, plane_2);


    /* Assign ground and steps plane */
    float c_1 = coefficients_1->values[2]/coefficients_1->values[3];
    float c_2 = coefficients_2->values[2]/coefficients_2->values[3];

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground = c_1 > c_2 ? plane_1 : plane_2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr steps = c_1 > c_2 ? plane_2 : plane_1;
    pcl::ModelCoefficients::Ptr step_coefficients = c_1 > c_2 ? coefficients_2 : coefficients_1;

    float a = step_coefficients->values[0];
    float b = step_coefficients->values[1];
    float c = step_coefficients->values[2];
    float d = step_coefficients->values[3];


    /* Publish both pointclouds */
    publish_pointcloud(ground, &pub_ground, cloud_msg->header.stamp, {255,0,0});
    publish_pointcloud(steps, &pub_steps, cloud_msg->header.stamp, {0,0,255});


    /* Create octomap of steps pointcloud */
    
    /* Transform to world frame */
    // tf_listener->waitForTransform(octomap_frame, steps_frame, cloud_msg->header.stamp, ros::Duration(0.2));
    tf::StampedTransform pointcloud_transform;
    tf::StampedTransform map_to_baselink_transform;
    try
    {
        tf_listener->lookupTransform(octomap_frame, steps_frame, cloud_msg->header.stamp, pointcloud_transform);
        tf_listener->lookupTransform(octomap_frame, base_link_frame, cloud_msg->header.stamp, map_to_baselink_transform);
    }
    catch (tf::TransformException &ex)
    {
        std::cout << "No TF!" << std::endl;
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr steps_world_full (new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 steps_cloud_msg, steps_cloud_world_msg;
    pcl::toROSMsg (*steps, steps_cloud_msg);
    // pcl_ros::transformPointCloud(octomap_frame, steps_cloud_msg, steps_cloud_world_msg, *tf_listener);
    pcl_ros::transformPointCloud(octomap_frame, pointcloud_transform, steps_cloud_msg, steps_cloud_world_msg);
    pcl::fromROSMsg (steps_cloud_world_msg, *steps_world_full);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr steps_world (new pcl::PointCloud<pcl::PointXYZRGB>);
    tf::Vector3 min_point(bounding_box[0], bounding_box[2], bounding_box[4]);
    tf::Vector3 max_point(bounding_box[1], bounding_box[3], bounding_box[5]);
    min_point = map_to_baselink_transform * min_point;
    max_point = map_to_baselink_transform * max_point;
    std::vector<double> transformed_bounding_box = {min_point[0], max_point[0], min_point[1], max_point[1], min_point[2], max_point[2]};
    cropbox_pointcloud(steps_world_full, steps_world, transformed_bounding_box);


    /* Without forced flattening */
    if (counter == 0)
        octree = new octomap::OcTree(octomap_resolution);

    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = steps_world->begin(); it != steps_world->end(); it++)
    {
        octomap::point3d coord(it->x, it->y, it->z);
        octree->updateNode(coord, true, true);
    }


    /* With forced flattening */
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = steps_world->begin(); it != steps_world->end(); it++)
        it->z = 0;

    publish_pointcloud(steps_world, &pub_steps_world_flat, cloud_msg->header.stamp, {100,0,200});

    if (counter == 0)
        octree_flat = new octomap::OcTree(octomap_resolution);

    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = steps_world->begin(); it != steps_world->end(); it++)
    {
        octomap::point3d coord(it->x, it->y, it->z);
        octree_flat->updateNode(coord, true, true);
    }

    if (++counter == octomap_threshold)
    {
        publish_octomap(octree, &pub_octomap, cloud_msg->header.stamp);
        publish_octomap(octree_flat, &pub_octomap_flat, cloud_msg->header.stamp);
        counter = 0;
    }
    publish_bounding_box(cloud_msg->header.stamp, bounding_box);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "flatten_pointcloud_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.getParam("in_points", in_points);
    nh_private.getParam("in_request", in_request);

    nh_private.getParam("out_ground", out_ground);
    nh_private.getParam("out_steps", out_steps);
    nh_private.getParam("out_steps_world_flat", out_steps_world_flat);

    nh_private.getParam("steps_frame", steps_frame);
    nh_private.getParam("base_link_frame", base_link_frame);

    nh_private.getParam("downsample_size", downsample_size);
    nh_private.getParam("ransac_dt", ransac_dt);

    nh_private.getParam("out_steps_octomap", out_steps_octomap);
    nh_private.getParam("out_steps_octomap_flat", out_steps_octomap_flat);
    nh_private.getParam("octomap_frame", octomap_frame);
    nh_private.getParam("octomap_resolution", octomap_resolution);
    nh_private.getParam("out_bounding_box", out_bounding_box);
    nh_private.getParam("bounding_box", bounding_box);
    nh_private.getParam("pre_crop_box", pre_crop_box);
    nh_private.getParam("octomap_threshold", octomap_threshold);

    // ros::Subscriber sub_points = nh.subscribe(in_points, 1, pointcloud_callback);
    // ros::Subscriber sub_request = nh.subscribe(in_request, 1, pointcloud_callback);

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_points (nh, in_points, 10);
    message_filters::Subscriber<floor_octomap::StampedString> sub_request (nh, in_request, 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, floor_octomap::StampedString> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    sync_.reset(new Sync(MySyncPolicy(10), sub_points, sub_request));
    sync_->registerCallback(boost::bind(pointcloud_callback, _1, _2));

    // message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, floor_octomap::StampedString> sync(sub_points, sub_request, 10);
    // sync.registerCallback(boost::bind(pointcloud_callback, _1, _2));

    pub_ground = nh.advertise<sensor_msgs::PointCloud2>(out_ground, 1);
    pub_steps = nh.advertise<sensor_msgs::PointCloud2>(out_steps, 1);
    pub_steps_world_flat = nh.advertise<sensor_msgs::PointCloud2>(out_steps_world_flat, 1);
    
    //crop test
    pub_steps_space_filtered = nh.advertise<sensor_msgs::PointCloud2>("space_filtered", 1);

    pub_octomap = nh.advertise<octomap_msgs::Octomap>(out_steps_octomap, 1);
    pub_octomap_flat = nh.advertise<octomap_msgs::Octomap>(out_steps_octomap_flat, 1);
    pub_bounding_box = nh.advertise<visualization_msgs::Marker>(out_bounding_box, 1);

    tf_listener = new tf::TransformListener();

    ros::spin();
    return 0;
}
