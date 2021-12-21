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
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
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
#include <stdlib.h>
#include <math.h> 

using namespace Eigen;
using namespace std;

/*
TODO:
RANSAC detecting stairs in diagonal planes
remove small "chunk" planes withing single plane consisting of mulitiple "chunks"
*/

ros::Publisher pub_ground, pub_steps, pub_steps_world_flat;

ros::Publisher pub_steps_space_filtered;

string in_points, in_request, out_ground, out_steps, out_steps_world_flat;
string steps_frame, base_link_frame;

// RANSAC
float downsample_size;
float ransac_dt;

// Octomap
ros::Publisher pub_octomap;
ros::Publisher pub_steppable;
ros::Publisher pub_obstacle;
string out_steppable;
string out_obstacle;
string octomap_frame;
float octomap_resolution;
ros::Publisher pub_bounding_box;
string out_bounding_box;
vector<double> bounding_box;
vector<double> pre_crop_box;
vector<double> legs_crop_box;
float obstacle_plane_height;

// Camera to World transform
bool tf_ready = false;
tf::TransformListener *tf_listener;

// Traversable parameters
float MIN_PLANE_ANGLE;
float MAX_PLANE_HEIGHT;
int MIN_PLANE_SIZE;
float MAX_VERTICAL_ANGLE;
int MIN_CLUSTER_SIZE;
float MIN_PLANE_DENSITY;
float MAX_PLANE_ANGLE;

// Run mode
bool FLATTEN;
int SOR_MK;
float SOR_STDDEV;
float ROR_RAD;
int ROR_MINNEIGH;

bool DEMO;
int DEMO_COUNT;

void change_pointcloud_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<float> rgb)
{
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); it++)
    {
        it->r = rgb[0]; it->g = rgb[1]; it->b = rgb[2];
    }
}

void ransac_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane)
{
    // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // seg.setOptimizeCoefficients (true);
    // seg.setInputCloud (cloud);
    // // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE); // only want points perpendicular to a given axis
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations (1000);
    // seg.setDistanceThreshold (ransac_dt);

    // Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0); // because we want a specific plane (X-Y Plane) (the ground plane is perpendicular to the z axis)
    // seg.setAxis (axis);
    // seg.setEpsAngle (MAX_PLANE_ANGLE * (M_PI/180.0f) ); // plane can be within 10 degrees of X-Y plane

    // seg.segment (*inliers, *coefficients);
    // pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, *inliers, *plane);

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

void crophull_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new, std::vector<Eigen::Vector3d> box_points)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::Vertices> hull_polygon;
    for (int i = 0 ; i < box_points.size() ; i ++)
    {
        pcl::PointXYZRGB point;
        point.x = box_points[i](0);
        point.y = box_points[i](1);
        point.z = box_points[i](2);
        box_cloud->push_back(point);
    }

    pcl::ConvexHull<pcl::PointXYZRGB> cHull;
    cHull.setInputCloud(box_cloud);
    cHull.reconstruct(*hull_cloud, hull_polygon);
    pcl::CropHull<pcl::PointXYZRGB> cropHullFilter;
    cropHullFilter.setHullIndices(hull_polygon);
    cropHullFilter.setHullCloud(hull_cloud);
    cropHullFilter.setDim(3);
    cropHullFilter.setCropOutside(true);

    cropHullFilter.setInputCloud(cloud_orig);
    cropHullFilter.filter(*cloud_new);
}

void cluster_pointcloud_multiple(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> *all_clusters)
{
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_orig);
    std::vector<pcl::PointIndices> cluster_indices;

    ec.setClusterTolerance (1);
    ec.setMinClusterSize (20);
    ec.setMaxClusterSize (5000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_orig);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const int &index : it->indices)
            cloud_cluster->points.push_back(cloud_orig->points[index]);
            
        cloud_cluster->width = static_cast<uint32_t> (cloud_cluster->points.size());
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        all_clusters->push_back(cloud_cluster);
    }
}

void crop_remove_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new, pcl::PointCloud<pcl::PointXYZRGB>::Ptr under_feet, std::vector<Eigen::Vector3d> box_points)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::Vertices> hull_polygon;
    for (int i = 0 ; i < box_points.size() ; i ++)
    {
        pcl::PointXYZRGB point;
        point.x = box_points[i](0);
        point.y = box_points[i](1);
        point.z = box_points[i](2);
        box_cloud->push_back(point);
    }

    pcl::ConvexHull<pcl::PointXYZRGB> cHull;
    cHull.setInputCloud(box_cloud);
    cHull.reconstruct(*hull_cloud, hull_polygon);
    pcl::CropHull<pcl::PointXYZRGB> cropHullFilter;
    cropHullFilter.setHullIndices(hull_polygon);
    cropHullFilter.setHullCloud(hull_cloud);
    cropHullFilter.setDim(3);
    cropHullFilter.setCropOutside(false);

    cropHullFilter.setInputCloud(cloud_orig);
    cropHullFilter.filter(*cloud_new);

    cropHullFilter.setCropOutside(true);
    cropHullFilter.filter(*under_feet);
}

void publish_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, ros::Publisher* pub, ros::Time stamp, string frame, vector<float> color = {-1})
{
    if (color[0] >= 0)
        change_pointcloud_color(cloud, color);
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg (*cloud, cloud_msg);
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = frame;
    pub->publish(cloud_msg);
}

void publish_octomap(octomap::OcTree *octree, ros::Publisher* pub, ros::Time stamp)
{
    octree->updateInnerOccupancy();
    octree->toMaxLikelihood();
    octree->prune();
    
    octomap_msgs::Octomap octomap_msg;
    octomap_msgs::binaryMapToMsg(*octree, octomap_msg);
    octomap_msg.header.frame_id = octomap_frame;
    octomap_msg.header.stamp = stamp;
    pub->publish(octomap_msg);
    delete octree;
}

void publish_bounding_box(ros::Time stamp, std::vector<Eigen::Vector3d> box_points)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = octomap_frame;
    marker.header.stamp = stamp;
    marker.ns = "base_link bbox";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.color.a = 0.3;
    std::vector <int> loop {0, 1, 1, 2, 2, 3, 3, 0, 0, 6, 1, 7, 3, 5, 2, 4, 4, 5, 5, 6, 6, 7, 7, 4};
    for (int i = 0 ; i < loop.size() ; i ++)
    {
        geometry_msgs::Point p;
        p.x = box_points[loop[i]](0);
        p.y = box_points[loop[i]](1);
        p.z = box_points[loop[i]](2);
        marker.points.push_back(p);
    }
    pub_bounding_box.publish(marker);
}

int is_traversable(pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coeff, double tz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr removed_cloud)
{
    float a = coeff->values[0];
    float b = coeff->values[1];
    float c = coeff->values[2];
    float d = coeff->values[3];

    Eigen::Vector3f n (a, b, c);
    n.normalize();
    float plane_angle = n(2);

    Eigen::Vector4f plane_centroid;
    pcl::compute3DCentroid (*cloud, plane_centroid);
    float plane_height = plane_centroid[2];

    int plane_size = cloud->size();

    // std::cout << plane_angle << ", " << plane_size << ", " << plane_height << std::endl;
    
    if (plane_angle < MIN_PLANE_ANGLE)
        return -1;
    if (plane_size < MIN_PLANE_SIZE)
        return -1;
    if (plane_height-tz > MAX_PLANE_HEIGHT)
        return -1;
    
    // Remove Outliers
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror (true);
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(ROR_RAD);
    ror.setMinNeighborsInRadius(ROR_MINNEIGH);
    ror.filter(*out_cloud);

    pcl::PointIndicesPtr inliers(new pcl::PointIndices);
    ror.getRemovedIndices(*inliers);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.filter (*removed_cloud);

    if (out_cloud->size() < 10)
        return -2;

    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    std::vector<pcl::Vertices> hull_polygon;
    chull.setInputCloud(out_cloud);
    chull.setDimension(3);
    chull.setComputeAreaVolume(true);
    chull.reconstruct(*hull_cloud, hull_polygon);

    float plane_density = out_cloud->size() / chull.getTotalArea();
    float min_density = pow((1 / downsample_size),2) * MIN_PLANE_DENSITY;
    
    if (plane_density < min_density)
        return -4;

    // std::cout << plane_angle << ", " << plane_height << ", " << chull.getTotalArea() << ", " << cloud->size() << ", " << out_cloud->size() << ", " << plane_density << ", " << min_density << std::endl;
    if ((plane_height-tz) < 0.05 && plane_size > 1000 && plane_angle > 0.99)
    {
        change_pointcloud_color(out_cloud, {255,255,255});
    }
    else if (plane_density < min_density*SOR_STDDEV)
    {
        pcl::CropHull<pcl::PointXYZRGB> cropHullFilter;
        cropHullFilter.setHullIndices(hull_polygon);
        cropHullFilter.setHullCloud(hull_cloud);
        cropHullFilter.setDim(3);
        cropHullFilter.setCropOutside(true);
        cropHullFilter.setInputCloud(full_cloud);
        cropHullFilter.filter(*out_cloud);

        pcl::CropHull<pcl::PointXYZRGB> cropHullFilter2;
        cropHullFilter2.setHullIndices(hull_polygon);
        cropHullFilter2.setHullCloud(hull_cloud);
        cropHullFilter2.setDim(3);
        cropHullFilter2.setCropOutside(false);
        cropHullFilter2.setInputCloud(cloud);
        cropHullFilter2.filter(*removed_cloud);

        change_pointcloud_color(out_cloud, {80,160,240});
        DEMO_COUNT += out_cloud->size();
    }
    else
    {
        change_pointcloud_color(out_cloud, {80,160,240});
        DEMO_COUNT += out_cloud->size();
    }

    return 1;
}

bool is_obstacle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coeff)
{
    float a = coeff->values[0];
    float b = coeff->values[1];
    float c = coeff->values[2];
    float d = coeff->values[3];

    Eigen::Vector3f n (a, b, c);
    n.normalize();
    float plane_angle = n(2);

    int plane_size = cloud->size();
    
    if (plane_angle < MAX_VERTICAL_ANGLE)
        return true;
    if (plane_size < MIN_PLANE_SIZE)
        return true;
    return false;
}

void plane_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg (*cloud_msg, *whole_cloud);

    /* Crop pointcloud before calculations */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud_crop (new pcl::PointCloud<pcl::PointXYZRGB>);
    cropbox_pointcloud(whole_cloud, whole_cloud_crop, pre_crop_box);


    /* Downsample pointcloud */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud_down (new pcl::PointCloud<pcl::PointXYZRGB>);
    downsample_pointcloud(whole_cloud_crop, whole_cloud_down, downsample_size);
    publish_pointcloud(whole_cloud_down, &pub_steps_space_filtered, cloud_msg->header.stamp, steps_frame, {255,0,0});


    /* Get tf */
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
    double tx = map_to_baselink_transform.getOrigin().getX();
    double ty = map_to_baselink_transform.getOrigin().getY();
    double tz = map_to_baselink_transform.getOrigin().getZ();


    /* Transform pointcloud */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr steps_world_full (new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 steps_cloud_msg, steps_cloud_world_msg;
    pcl::toROSMsg (*whole_cloud_down, steps_cloud_msg);
    pcl_ros::transformPointCloud(octomap_frame, pointcloud_transform, steps_cloud_msg, steps_cloud_world_msg);
    pcl::fromROSMsg (steps_cloud_world_msg, *steps_world_full);


    /* Extract all planes */
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining_cloud = steps_world_full;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr remaining_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    *remaining_cloud += *steps_world_full;
    int original_size = remaining_cloud->size();
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr traversable_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacle_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr leftover_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    int run_count = 0;
    DEMO_COUNT = 0;
    while (remaining_cloud->size() > original_size*0.05 && run_count < 20)
    {
        /* Segment a single plane */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        ransac_pointcloud(remaining_cloud, inliers, coefficients, plane_cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr removed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        int flag = is_traversable(steps_world_full, plane_cloud, coefficients, tz, processed_cloud, removed_cloud);
        if (flag == 1)
        {
            *traversable_cloud += *processed_cloud;

            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud (remaining_cloud);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*remaining_cloud);
            *remaining_cloud += *removed_cloud;
            *remaining_cloud += *leftover_cloud;
            leftover_cloud->clear();
        }
        // else if (is_obstacle(plane_cloud, coefficients))
        else if (flag == -1)
        {
            *obstacle_cloud += *plane_cloud;

            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud (remaining_cloud);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*remaining_cloud);
        }
        else if (flag == -2)
        {
            *leftover_cloud += *plane_cloud;

            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud (remaining_cloud);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*remaining_cloud);
        }
        // else if (processed_cloud->size() > 0)
        else if (processed_cloud->size() > 0 && flag == -4)
        {
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud (remaining_cloud);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*remaining_cloud);
            *remaining_cloud += *processed_cloud;
            *leftover_cloud += *removed_cloud;
        }
        // else if (flag == -3)
        // {
        //     *obstacle_cloud += *plane_cloud;
        // }
        run_count ++;
    }
    // std::cout << remaining_cloud->size() << std::endl;

    if (DEMO)
    {
        std::cout << "T: " << DEMO_COUNT << ", O: " << original_size*0.1 << std::endl;
        if (DEMO_COUNT > original_size*0.1)
        {
            /* Publish both pointclouds */
            change_pointcloud_color(obstacle_cloud, {255, 80, 80});
            publish_pointcloud(traversable_cloud, &pub_ground, cloud_msg->header.stamp, octomap_frame);
            publish_pointcloud(obstacle_cloud, &pub_steps, cloud_msg->header.stamp, octomap_frame);

            change_pointcloud_color(remaining_cloud, {0, 0, 0});
            publish_pointcloud(remaining_cloud, &pub_steps_world_flat, cloud_msg->header.stamp, octomap_frame);
        }
        else
        {
            std::cout << "Missed" << std::endl;
        }
    }
    else
    {
        /* Publish both pointclouds */
        change_pointcloud_color(obstacle_cloud, {255, 80, 80});
        publish_pointcloud(traversable_cloud, &pub_ground, cloud_msg->header.stamp, octomap_frame);
        publish_pointcloud(obstacle_cloud, &pub_steps, cloud_msg->header.stamp, octomap_frame);


        change_pointcloud_color(remaining_cloud, {0, 0, 0});
        publish_pointcloud(remaining_cloud, &pub_steps_world_flat, cloud_msg->header.stamp, octomap_frame);
    }
}

void callback_demo_branch(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    const clock_t begin_time = clock();
    plane_callback(cloud_msg);
}

void callback_branch(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const floor_octomap::StampedString::ConstPtr& req_msg)
{
    const clock_t begin_time = clock();
    plane_callback(cloud_msg);
    std::cout << "Elapsed time: " << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
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

    nh_private.getParam("out_steppable", out_steppable);
    nh_private.getParam("out_obstacle", out_obstacle);
    nh_private.getParam("octomap_frame", octomap_frame);
    nh_private.getParam("octomap_resolution", octomap_resolution);
    nh_private.getParam("out_bounding_box", out_bounding_box);
    nh_private.getParam("bounding_box", bounding_box);
    nh_private.getParam("pre_crop_box", pre_crop_box);
    nh_private.getParam("legs_crop_box", legs_crop_box);
    nh_private.getParam("obstacle_plane_height", obstacle_plane_height);

    nh_private.getParam("MIN_PLANE_ANGLE", MIN_PLANE_ANGLE);
    nh_private.getParam("MAX_PLANE_HEIGHT", MAX_PLANE_HEIGHT);
    nh_private.getParam("MIN_PLANE_SIZE", MIN_PLANE_SIZE);
    nh_private.getParam("MAX_VERTICAL_ANGLE", MAX_VERTICAL_ANGLE);
    nh_private.getParam("MIN_CLUSTER_SIZE", MIN_CLUSTER_SIZE);
    nh_private.getParam("MIN_PLANE_DENSITY", MIN_PLANE_DENSITY);
    nh_private.getParam("MAX_PLANE_ANGLE", MAX_PLANE_ANGLE);

    nh_private.getParam("FLATTEN", FLATTEN);
    nh_private.getParam("SOR_MK", SOR_MK);
    nh_private.getParam("SOR_STDDEV", SOR_STDDEV);
    nh_private.getParam("ROR_RAD", ROR_RAD);
    nh_private.getParam("ROR_MINNEIGH", ROR_MINNEIGH);

    nh_private.getParam("DEMO", DEMO);

    if (DEMO)
    {
        ros::Subscriber sub = nh.subscribe(in_points, 10, callback_demo_branch);

        pub_ground = nh.advertise<sensor_msgs::PointCloud2>(out_ground, 1);
        pub_steps = nh.advertise<sensor_msgs::PointCloud2>(out_steps, 1);
        pub_steps_world_flat = nh.advertise<sensor_msgs::PointCloud2>(out_steps_world_flat, 1);
        
        //crop test
        pub_steps_space_filtered = nh.advertise<sensor_msgs::PointCloud2>("space_filtered", 1);

        pub_steppable = nh.advertise<octomap_msgs::Octomap>(out_steppable, 1);
        pub_obstacle = nh.advertise<octomap_msgs::Octomap>(out_obstacle, 1);
        pub_bounding_box = nh.advertise<visualization_msgs::Marker>(out_bounding_box, 1);

        tf_listener = new tf::TransformListener();

        ros::spin();
        return 0;
    }
    else
    {
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_points (nh, in_points, 10);
        message_filters::Subscriber<floor_octomap::StampedString> sub_request (nh, in_request, 10);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, floor_octomap::StampedString> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        sync_.reset(new Sync(MySyncPolicy(10), sub_points, sub_request));
        sync_->registerCallback(boost::bind(callback_branch, _1, _2));

        pub_ground = nh.advertise<sensor_msgs::PointCloud2>(out_ground, 1);
        pub_steps = nh.advertise<sensor_msgs::PointCloud2>(out_steps, 1);
        pub_steps_world_flat = nh.advertise<sensor_msgs::PointCloud2>(out_steps_world_flat, 1);
        
        //crop test
        pub_steps_space_filtered = nh.advertise<sensor_msgs::PointCloud2>("space_filtered", 1);

        pub_steppable = nh.advertise<octomap_msgs::Octomap>(out_steppable, 1);
        pub_obstacle = nh.advertise<octomap_msgs::Octomap>(out_obstacle, 1);
        pub_bounding_box = nh.advertise<visualization_msgs::Marker>(out_bounding_box, 1);

        tf_listener = new tf::TransformListener();

        ros::spin();
        return 0;
    }
}
