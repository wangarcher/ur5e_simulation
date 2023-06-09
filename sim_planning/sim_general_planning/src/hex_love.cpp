// a simple planning demo, initialized in 210813
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include "ros/ros.h"

#include <iostream>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

struct PoseInfo
{
    double general_dist;
    double start_dist;
    double end_dist;
    geometry_msgs::Pose pose;
};

class GeneralPickPlace
{
public:
    Eigen::Vector3d obj_cam_translation_;
    Eigen::Quaterniond obj_cam_orientation_;

    bool first_reco_flag;    // outwards flag, when the ee reached the desired pre-picking place
    bool second_reco_flag;   // outwards flag, when the ee reached the desired pre-placeing place
    bool pick_flag = false;  // inwards flag, when the flag set as true, the robot is ready for the pick motion sequence.
    bool place_flag = false; // inwards flag, whent the flag set as true, the robot is ready for the place motion sequence.

public:
    void obj_visual_callback(const geometry_msgs::PoseStamped &odom);

    void pseudo_obj_squence_callback(geometry_msgs::PoseArray &obj_poses);

    std::vector<PoseInfo> getKNearest(const geometry_msgs::PoseArray &obj_poses,
                                      const geometry_msgs::PoseStamped &start_pose,
                                      const geometry_msgs::PoseStamped &end_pose,
                                      int k);

    std::vector<PoseInfo> getSequence(std::vector<PoseInfo> &k_nearest_poses_info,
                                      int m, int n, int k);

    std::vector<PoseInfo> getSequence_1(std::vector<PoseInfo> &k_nearest_poses_info,
                                        int k,
                                        const geometry_msgs::PoseStamped &start_pose);

    bool dbscan(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, std::vector<std::vector<int>> &clusters_index, const double &r, const int &size);
};

void GeneralPickPlace::obj_visual_callback(const geometry_msgs::PoseStamped &odom)
{
    obj_cam_translation_ << odom.pose.position.x,
        odom.pose.position.y,
        odom.pose.position.z;
    obj_cam_orientation_.coeffs() << odom.pose.orientation.x,
        odom.pose.orientation.y,
        odom.pose.orientation.z,
        odom.pose.orientation.w;
    if (odom.header.frame_id == "1st")
    {
        pick_flag = true;
        place_flag = false;
    }
    if (odom.header.frame_id == "2nd")
    {
        place_flag = true;
        pick_flag = false;
    }
}

void GeneralPickPlace::pseudo_obj_squence_callback(geometry_msgs::PoseArray &obj_poses)
{
    srand(static_cast<unsigned>(time(0)));
    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3f rand_position;
        geometry_msgs::Pose set_pose;
        double lower = 0;
        double upper = 2;
        for (int j = 0; j < 3; j++)
        {
            rand_position(j) = lower + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (upper - lower)));
        }
        set_pose.position.x = rand_position(0);
        set_pose.position.y = rand_position(1);
        set_pose.position.z = 0.0;
        obj_poses.poses.push_back(set_pose);
    }
}

std::vector<PoseInfo> GeneralPickPlace::getKNearest(const geometry_msgs::PoseArray &obj_poses,
                                                    const geometry_msgs::PoseStamped &start_pose,
                                                    const geometry_msgs::PoseStamped &end_pose,
                                                    int k)
{
    int obj_poses_size = obj_poses.poses.size();
    std::vector<PoseInfo> poses_info;
    PoseInfo new_pose_info;
    for (int i = 0; i < obj_poses_size; i++)
    {
        double start_dx = obj_poses.poses[i].position.x - start_pose.pose.position.x;
        double start_dy = obj_poses.poses[i].position.y - start_pose.pose.position.y;
        double start_dz = obj_poses.poses[i].position.z - start_pose.pose.position.z;
        double end_dx = obj_poses.poses[i].position.x - end_pose.pose.position.x;
        double end_dy = obj_poses.poses[i].position.y - end_pose.pose.position.y;
        double end_dz = obj_poses.poses[i].position.z - end_pose.pose.position.z;
        new_pose_info.start_dist = std::sqrt(start_dx * start_dx + start_dy * start_dy + start_dz * start_dz);
        new_pose_info.end_dist = std::sqrt(end_dx * end_dx + end_dy * end_dy + end_dz * end_dz);
        new_pose_info.general_dist = new_pose_info.start_dist + new_pose_info.end_dist;
        new_pose_info.pose = obj_poses.poses[i];
        poses_info.push_back(new_pose_info);
    }
    sort(poses_info.begin(), poses_info.end(), [](PoseInfo a, PoseInfo b)
         { return a.general_dist <= b.general_dist; });

    std::vector<PoseInfo> k_nearest_poses_info(poses_info.begin(), poses_info.begin() + k);
    return k_nearest_poses_info;
}

std::vector<PoseInfo> GeneralPickPlace::getSequence(std::vector<PoseInfo> &k_nearest_poses_info,
                                                    int m, int n, int k)
{
    std::vector<PoseInfo> sequence;
    sort(k_nearest_poses_info.begin(), k_nearest_poses_info.end(), [](PoseInfo a, PoseInfo b)
         { return a.start_dist <= b.start_dist; });
    std::vector<PoseInfo> start_sequence(k_nearest_poses_info.begin(), k_nearest_poses_info.begin() + m);
    std::vector<PoseInfo> end_sequence(k_nearest_poses_info.begin() + m, k_nearest_poses_info.end());

    sort(end_sequence.begin(), end_sequence.begin() + n, [](PoseInfo a, PoseInfo b)
         { return a.end_dist >= b.end_dist; });

    sequence.insert(sequence.end(), start_sequence.begin(), start_sequence.end());
    sequence.insert(sequence.end(), end_sequence.begin(), end_sequence.end());
    return sequence;
}

std::vector<PoseInfo> GeneralPickPlace::getSequence_1(std::vector<PoseInfo> &k_nearest_poses_info,
                                                      int k,
                                                      const geometry_msgs::PoseStamped &start_pose)
{
    std::vector<PoseInfo> sequence;

    for (int i = 0; i < k; i++)
    {
        sort(k_nearest_poses_info.begin(), k_nearest_poses_info.end(), [](PoseInfo a, PoseInfo b)
             { return a.start_dist <= b.start_dist; });

        sequence.push_back(k_nearest_poses_info[0]);
        for (int j = 1; j < k_nearest_poses_info.size(); j++)
        {
            double start_dx = k_nearest_poses_info[j].pose.position.x - k_nearest_poses_info[0].pose.position.x;
            double start_dy = k_nearest_poses_info[j].pose.position.y - k_nearest_poses_info[0].pose.position.y;
            double start_dz = k_nearest_poses_info[j].pose.position.z - k_nearest_poses_info[0].pose.position.z;
            k_nearest_poses_info[j].start_dist = std::sqrt(start_dx * start_dx + start_dy * start_dy + start_dz * start_dz);
        }
        std::vector<PoseInfo>::iterator erase = k_nearest_poses_info.begin();
        k_nearest_poses_info.erase(erase);
    }

    return sequence;
}

bool dbscan(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, std::vector<std::vector<int>> &clusters_index, const double &r, const int &size)
{
    if (!cloud_in->size())
        return false;
    // LOG()
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud_in);
    std::vector<bool> cloud_processed(cloud_in->size(), false);

    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        if (cloud_processed[i])
        {
            continue;
        }

        std::vector<int> seed_queue;
        std::vector<int> indices_cloud;
        std::vector<float> dists_cloud;
        if (tree.radiusSearch(cloud_in->points[i], r, indices_cloud, dists_cloud) >= size)
        {
            seed_queue.push_back(i);
            cloud_processed[i] = true;
        }
        else
        {
            // cloud_processed[i] = true;
            continue;
        }

        int seed_index = 0;
        while (seed_index < seed_queue.size())
        {
            std::vector<int> indices;
            std::vector<float> dists;
            if (tree.radiusSearch(cloud_in->points[seed_queue[seed_index]], r, indices, dists) < size)
            {
                ++seed_index;
                continue;
            }
            for (size_t j = 0; j < indices.size(); ++j)
            {
                if (cloud_processed[indices[j]])
                {
                    continue;
                }
                seed_queue.push_back(indices[j]);
                cloud_processed[indices[j]] = true;
            }
            ++seed_index;
        }
        clusters_index.push_back(seed_queue);
    }

    // std::cout << clusters_index.size() << std::endl;
    if (clusters_index.size())
        return true;
    else
        return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twelve_monkeys");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(7);
    spinner.start();

    GeneralPickPlace general_pick_place;

    ros::Publisher sequence_pub_ = n.advertise<nav_msgs::Path>("sequence", 1);
    ros::Publisher marker_pub_ = n.advertise<visualization_msgs::Marker>("markers", 1000);

    ros::Rate loop_rate(10);
    geometry_msgs::PoseArray obj_poses;
    std::vector<PoseInfo> k_nearest_poses_info;
    std::vector<PoseInfo> sequence;
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped end_pose;
    start_pose.pose.position.x = 0.0;
    start_pose.pose.position.y = 0.0;
    start_pose.pose.position.z = 0.0;
    end_pose.pose.position.x = 0.0;
    end_pose.pose.position.y = 1.0;
    end_pose.pose.position.z = 0.0;

    general_pick_place.pseudo_obj_squence_callback(obj_poses);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point;

    for (int i = 0; i < obj_poses.poses.size(); i++)
    {
        point.x = obj_poses.poses[i].position.x;
        point.y = obj_poses.poses[i].position.y;
        point.z = obj_poses.poses[i].position.z;
        cloud_in->points.push_back(point);
    }

    std::cout << "the cloud_in size: " << cloud_in->points.size() << " ." << std::endl;

    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    typename pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.20); // 2cm
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(6);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_in);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {

        typename pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin();
             pit != it->indices.end(); ++pit)
        {
            cluster->points.push_back(cloud_in->points[*pit]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    // std::vector<std::vector<int>> clusters_index;
    // general_pick_place.dbscan(cloud_in, clusters_index, 0.1, 10);
    std::cout << "now we have: " << clusters.size() << " clusters." << std::endl;

    while (n.ok())
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "Point";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(2.0);

        for (int i = 0; i < clusters.size(); ++i)
        {
            for (size_t j = 0; j < clusters[i]->points.size(); ++j)
            {
                geometry_msgs::Point point;
                point.x = clusters[i]->points[j].x;
                point.y = clusters[i]->points[j].y;
                point.z = clusters[i]->points[j].z;

                marker.points.push_back(point);
            }
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(2.0);
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 0.03 + 0.60 * i;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker_pub_.publish(marker);
        }

        // nav_msgs::Path gui_path;
        // gui_path.poses.resize(sequence.size());
        // gui_path.header.frame_id = "map";
        // for (int i = 0; i < 12; i++)
        // {
        //     std::cout << "sequence num. " << i << ": "
        //               << sequence[i].pose.position.x << ", "
        //               << sequence[i].pose.position.y << ", "
        //               << sequence[i].pose.position.z << std::endl;
        //     gui_path.poses[i].pose = sequence[i].pose;
        // }
        // gui_path.poses.insert(gui_path.poses.begin(), start_pose);
        // gui_path.poses.insert(gui_path.poses.end(), end_pose);

        // sequence_pub_.publish(gui_path);

        loop_rate.sleep();
    }

    ros::waitForShutdown();

    return 0;
}