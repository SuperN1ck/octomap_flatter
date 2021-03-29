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

// MODE
int RUN_MODE;

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

void bridge_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
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
    publish_pointcloud(whole_cloud_down, &pub_steps_space_filtered, cloud_msg->header.stamp, steps_frame, {255,0,0});


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


    /* Publish both pointclouds */
    publish_pointcloud(ground, &pub_ground, cloud_msg->header.stamp, steps_frame, {255,0,0});
    publish_pointcloud(steps, &pub_steps, cloud_msg->header.stamp, steps_frame, {0,0,255});


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


    /* Transform pointcloud */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr steps_world_full (new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 steps_cloud_msg, steps_cloud_world_msg;
    pcl::toROSMsg (*steps, steps_cloud_msg);
    pcl_ros::transformPointCloud(octomap_frame, pointcloud_transform, steps_cloud_msg, steps_cloud_world_msg);
    pcl::fromROSMsg (steps_cloud_world_msg, *steps_world_full);


    /* Transform bounding box */
    double tx = map_to_baselink_transform.getOrigin().getX();
    double ty = map_to_baselink_transform.getOrigin().getY();
    tf::Matrix3x3 m(map_to_baselink_transform.getRotation());
    double rr, rp, ry;
    m.getRPY(rr, rp, ry);
    tf::Transform bbox_transform;
    bbox_transform.setOrigin(tf::Vector3(tx, ty, 0));
    bbox_transform.setRotation(tf::createQuaternionFromRPY(rr, 0, ry));
    
    tf::Vector3 min_point(bounding_box[0], bounding_box[2], bounding_box[4]);
    tf::Vector3 max_point(bounding_box[1], bounding_box[3], bounding_box[5]);
    tf::Vector3 xyz_size = max_point - min_point;
    std::vector<tf::Vector3> box_points_tf;
    box_points_tf.push_back(min_point);
    min_point[0] += xyz_size[0];
    box_points_tf.push_back(min_point);
    min_point[1] += xyz_size[1];
    box_points_tf.push_back(min_point);
    min_point[0] -= xyz_size[0];
    box_points_tf.push_back(min_point);
    box_points_tf.push_back(max_point);
    max_point[0] -= xyz_size[0];
    box_points_tf.push_back(max_point);
    max_point[1] -= xyz_size[1];
    box_points_tf.push_back(max_point);
    max_point[0] += xyz_size[0];
    box_points_tf.push_back(max_point);

    std::vector<Eigen::Vector3d> transformed_bbox_points;
    for (int i=0; i<box_points_tf.size(); i++)
    {
        tf::Vector3 point_tf = bbox_transform*box_points_tf[i];
        Eigen::Vector3d point (point_tf[0], point_tf[1], point_tf[2]);
        transformed_bbox_points.push_back(point);
    }
    publish_bounding_box(cloud_msg->header.stamp, transformed_bbox_points);


    /* Crop pointcloud by bounding box */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr steps_world (new pcl::PointCloud<pcl::PointXYZRGB>);
    crophull_pointcloud(steps_world_full, steps_world, transformed_bbox_points);


    /* Flatten and publish octomap */
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = steps_world->begin(); it != steps_world->end(); it++)
        it->z = 0;
    publish_pointcloud(steps_world, &pub_steps_world_flat, cloud_msg->header.stamp, octomap_frame, {100,0,200});

    octomap::OcTree *octree_steppable = new octomap::OcTree(octomap_resolution);
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = steps_world->begin(); it != steps_world->end(); it++)
    {
        octomap::point3d coord(it->x, it->y, it->z);
        octree_steppable->updateNode(coord, true, true);
    }
    publish_octomap(octree_steppable, &pub_steppable, cloud_msg->header.stamp);
}

void obstacle_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        steppable (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        obstacles (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        temp_plane (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        temp_other (new pcl::PointCloud<pcl::PointXYZRGB>);

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


    /* Transform steppable pointcloud */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud_world (new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 before_cloud_msg, after_cloud_msg;
    pcl::toROSMsg (*whole_cloud_down, before_cloud_msg);
    pcl_ros::transformPointCloud(octomap_frame, pointcloud_transform, before_cloud_msg, after_cloud_msg);
    pcl::fromROSMsg (after_cloud_msg, *whole_cloud_world);


    /* Segment major plane */
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    ransac_pointcloud(whole_cloud_world, inliers, coefficients, temp_plane);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (whole_cloud_world);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*temp_other);

    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    Eigen::Vector3f n (a, b, c);
    n.normalize();

    while (n(2) < 0.9)
    {
        *obstacles += *temp_plane;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_temp_other (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointIndices::Ptr new_inliers (new pcl::PointIndices ());
        pcl::ModelCoefficients::Ptr new_coefficients (new pcl::ModelCoefficients ());
        ransac_pointcloud(temp_other, new_inliers, new_coefficients, temp_plane);

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (temp_other);
        extract.setIndices (new_inliers);
        extract.setNegative (true);
        extract.filter (*new_temp_other);

        temp_other = new_temp_other;
        a = new_coefficients->values[0];
        b = new_coefficients->values[1];
        c = new_coefficients->values[2];
        d = new_coefficients->values[3];
        n << a, b, c;
        n.normalize();
    }

    steppable = temp_plane;
    *obstacles += *temp_other;


    /* Points below the plane is set also as ground */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr under_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices());
    pcl::PointIndices::Ptr obstacle_inliers(new pcl::PointIndices());
    for (int i = 0; i < (*obstacles).size(); i++)
    {
        Eigen::Vector3f pt (obstacles->points[i].x, obstacles->points[i].y, obstacles->points[i].z);
        float pd = pt.dot(n);
        if (pd < obstacle_plane_height)
            ground_inliers->indices.push_back(i);
        else
            obstacle_inliers->indices.push_back(i);
    }
    pcl::ExtractIndices<pcl::PointXYZRGB> extract1;
    extract1.setInputCloud(obstacles);
    extract1.setIndices(obstacle_inliers);
    extract1.setNegative(true);
    extract1.filter(*under_points);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract2;
    extract2.setInputCloud(obstacles);
    extract2.setIndices(ground_inliers);
    extract2.setNegative(true);
    extract2.filter(*obstacles);

    *steppable += *under_points;


    /* Publish both pointclouds */
    publish_pointcloud(obstacles, &pub_ground, cloud_msg->header.stamp, octomap_frame, {255,0,0});
    publish_pointcloud(steppable, &pub_steps, cloud_msg->header.stamp, octomap_frame, {0,0,255});


    /* Transform bounding box */
    double tx = map_to_baselink_transform.getOrigin().getX();
    double ty = map_to_baselink_transform.getOrigin().getY();
    tf::Matrix3x3 m(map_to_baselink_transform.getRotation());
    double rr, rp, ry;
    m.getRPY(rr, rp, ry);
    tf::Transform bbox_transform;
    bbox_transform.setOrigin(tf::Vector3(tx, ty, 0));
    bbox_transform.setRotation(tf::createQuaternionFromRPY(rr, 0, ry));
    
    tf::Vector3 min_point(bounding_box[0], bounding_box[2], bounding_box[4]);
    tf::Vector3 max_point(bounding_box[1], bounding_box[3], bounding_box[5]);
    tf::Vector3 xyz_size = max_point - min_point;
    std::vector<tf::Vector3> box_points_tf;
    box_points_tf.push_back(min_point);
    min_point[0] += xyz_size[0];
    box_points_tf.push_back(min_point);
    min_point[1] += xyz_size[1];
    box_points_tf.push_back(min_point);
    min_point[0] -= xyz_size[0];
    box_points_tf.push_back(min_point);
    box_points_tf.push_back(max_point);
    max_point[0] -= xyz_size[0];
    box_points_tf.push_back(max_point);
    max_point[1] -= xyz_size[1];
    box_points_tf.push_back(max_point);
    max_point[0] += xyz_size[0];
    box_points_tf.push_back(max_point);

    std::vector<Eigen::Vector3d> transformed_bbox_points;
    for (int i=0; i<box_points_tf.size(); i++)
    {
        tf::Vector3 point_tf = bbox_transform*box_points_tf[i];
        Eigen::Vector3d point (point_tf[0], point_tf[1], point_tf[2]);
        transformed_bbox_points.push_back(point);
    }
    publish_bounding_box(cloud_msg->header.stamp, transformed_bbox_points);


    /* Crop obstacle pointcloud by bounding box */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacle_world (new pcl::PointCloud<pcl::PointXYZRGB>);
    crophull_pointcloud(obstacles, obstacle_world, transformed_bbox_points);


    /* Transform bounding box */
    double leg_tx = map_to_baselink_transform.getOrigin().getX();
    double leg_ty = map_to_baselink_transform.getOrigin().getY();
    tf::Matrix3x3 leg_m(map_to_baselink_transform.getRotation());
    double leg_rr, leg_rp, leg_ry;
    leg_m.getRPY(leg_rr, leg_rp, leg_ry);
    tf::Transform leg_box_transform;
    leg_box_transform.setOrigin(tf::Vector3(leg_tx, leg_ty, 0));
    leg_box_transform.setRotation(tf::createQuaternionFromRPY(leg_rr, 0, leg_ry));

    tf::Vector3 min_leg_point(legs_crop_box[0], legs_crop_box[2], legs_crop_box[4]);
    tf::Vector3 max_leg_point(legs_crop_box[1], legs_crop_box[3], legs_crop_box[5]);
    tf::Vector3 leg_xyz_size = max_leg_point - min_leg_point;
    std::vector<tf::Vector3> leg_box_points_tf;
    leg_box_points_tf.push_back(min_leg_point);
    min_leg_point[0] += leg_xyz_size[0];
    leg_box_points_tf.push_back(min_leg_point);
    min_leg_point[1] += leg_xyz_size[1];
    leg_box_points_tf.push_back(min_leg_point);
    min_leg_point[0] -= leg_xyz_size[0];
    leg_box_points_tf.push_back(min_leg_point);
    leg_box_points_tf.push_back(max_leg_point);
    max_leg_point[0] -= leg_xyz_size[0];
    leg_box_points_tf.push_back(max_leg_point);
    max_leg_point[1] -= leg_xyz_size[1];
    leg_box_points_tf.push_back(max_leg_point);
    max_leg_point[0] += leg_xyz_size[0];
    leg_box_points_tf.push_back(max_leg_point);

    std::vector<Eigen::Vector3d> transformed_leg_points;
    for (int i=0; i<leg_box_points_tf.size(); i++)
    {
        tf::Vector3 point_tf = leg_box_transform*leg_box_points_tf[i];
        Eigen::Vector3d point (point_tf[0], point_tf[1], point_tf[2]);
        transformed_leg_points.push_back(point);
    }
    // publish_bounding_box(cloud_msg->header.stamp, transformed_leg_points);


    /* Crop obstacle pointcloud to remove legs */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacle_world_new (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr under_feet (new pcl::PointCloud<pcl::PointXYZRGB>);
    crop_remove_pointcloud(obstacle_world, obstacle_world_new, under_feet, transformed_leg_points);


    /* Create and publish octomap */
    octomap::OcTree *octree_obstacle = new octomap::OcTree(octomap_resolution);
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = obstacle_world_new->begin(); it != obstacle_world_new->end(); it++)
    {
        if (it->z <= 0)
            continue;
        octomap::point3d coord(it->x, it->y, it->z);
        octree_obstacle->updateNode(coord, true, true);
    }
    publish_octomap(octree_obstacle, &pub_obstacle, cloud_msg->header.stamp);


    /* Crop pointcloud by bounding box */
    *steppable += *under_feet;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr steps_world (new pcl::PointCloud<pcl::PointXYZRGB>);
    crophull_pointcloud(steppable, steps_world, transformed_bbox_points);


    /* Flatten and publish octomap */
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = steps_world->begin(); it != steps_world->end(); it++)
        it->z = 0;
    publish_pointcloud(steps_world, &pub_steps_world_flat, cloud_msg->header.stamp, octomap_frame, {100,0,200});

    octomap::OcTree *octree_steppable = new octomap::OcTree(octomap_resolution);
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = steps_world->begin(); it != steps_world->end(); it++)
    {
        octomap::point3d coord(it->x, it->y, it->z);
        octree_steppable->updateNode(coord, true, true);
    }
    publish_octomap(octree_steppable, &pub_steppable, cloud_msg->header.stamp);
}

void callback_branch(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const floor_octomap::StampedString::ConstPtr& req_msg)
{
    if (RUN_MODE == 1)
        bridge_callback(cloud_msg);
    else if (RUN_MODE == 2)
        obstacle_callback(cloud_msg);
    else
        std::cout << "ERROR: RUN_MODE must be either 1 or 2. Currently set to " << RUN_MODE << std::endl;
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

    nh_private.getParam("RUN_MODE", RUN_MODE);

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
