//
// Created by tao on 7/25/21.
//
#include <unistd.h>
#include "velodyne_converter.h"

#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace kaist2bag
{
    VelodyneConverter::VelodyneConverter(const std::string &dataset_dir, const std::string &save_dir,
                                         const std::string &topic, const std::string &right_topic)
        : Converter(dataset_dir, save_dir), left_topic_(topic), right_topic_(right_topic)
    {
        left_bag_name_ = FilterSlash(left_topic_) + ".bag";
        right_bag_name_ = FilterSlash(right_topic_) + ".bag";
    }

    int VelodyneConverter::Convert()
    {
        CheckAndCreateSaveDir();

        boost::filesystem::path left_bag_file = boost::filesystem::path(save_dir_) / left_bag_name_;
        boost::filesystem::path right_bag_file = boost::filesystem::path(save_dir_) / right_bag_name_;

        const std::string left_stamp_file = dataset_dir_ + "/" + default_left_stamp_file;
        const std::string left_data_dir = dataset_dir_ + "/" + default_left_data_dir;
        const std::string right_stamp_file = dataset_dir_ + "/" + default_right_stamp_file;
        const std::string right_data_dir = dataset_dir_ + "/" + default_right_data_dir;

        ROS_INFO("saving velodyne");
        Convert(left_stamp_file, left_data_dir, left_bag_file.string() /*, left_topic_*/, right_stamp_file, right_data_dir, right_bag_file.string(), right_topic_, "velodyne");
        // Convert(left_stamp_file, left_data_dir, left_bag_file.string(), left_topic_, right_stamp_file, right_data_dir, right_bag_file.string(), right_topic_,"velodyne");
        ROS_INFO("done saving velodyne");

        return 0;
    }

    // void VelodyneConverter::Convert(const std::string& left_stamp_file, const std::string& left_data_dir,
    //                  const std::string& left_bag_file, const std::string& left_topic,
    //                  const std::string& right_stamp_file, const std::string& right_data_dir,
    //                  const std::string& right_bag_file, const std::string& right_topic,
    //                  const std::string& frame_id) {
    //     Eigen::Matrix3d R_v_L_lidar;
    //     Eigen::Vector3d t_v_L_lidar;

    //     Eigen::Matrix3d R_v_R_lidar;
    //     Eigen::Vector3d t_v_R_lidar;

    //     R_v_L_lidar << -0.514169,-0.702457,-0.492122,0.48979,-0.711497,0.503862,-0.704085,0.0180335,0.709886;
    //     t_v_L_lidar << -0.31189,0.394734,1.94661;

    //     R_v_R_lidar << -0.507842, 0.704544, -0.495695, -0.49974, -0.709646, -0.496651, -0.701681, -0.00450156,0.712477;
    //     t_v_R_lidar << -0.306052, -0.417145, 1.95223;

    //     rosbag::Bag bag(left_bag_file, rosbag::bagmode::Write);
    //     bag.setChunkThreshold(768*1024);
    //     bag.setCompression(rosbag::compression::BZ2);

    //     FILE* left_fp = fopen(left_stamp_file.c_str(), "r");
    //     int64_t left_stamp;
    //     std::vector<int64_t> left_all_stamps;
    //     while (fscanf(left_fp, "%ld\n", &left_stamp) == 1) {
    //         left_all_stamps.push_back(left_stamp);
    //     }
    //     fclose(left_fp);

    //     size_t left_total = left_all_stamps.size();
    //     for (size_t i = 0; i < left_all_stamps.size(); ++i) {
    //         std::string left_st = std::to_string(left_all_stamps[i]);
    //         std::string left_frame_file = left_data_dir + "/" + left_st + ".bin";
    //         ROS_INFO("converting %s\n", left_frame_file.c_str());
    //         if (!boost::filesystem::exists(left_frame_file)) {
    //             ROS_WARN("%s not exist\n", left_frame_file.c_str());
    //             continue;
    //         }
    //         std::ifstream left_file;
    //         left_file.open(left_frame_file, std::ios::in | std::ios::binary);
    //         pcl::PointCloud<pcl::PointXYZI> left_pcl_cloud;
    //         while (!left_file.eof()) {
    //             pcl::PointXYZI left_point;
    //             left_file.read(reinterpret_cast<char *>(&left_point.x), sizeof(float));
    //             left_file.read(reinterpret_cast<char *>(&left_point.y), sizeof(float));
    //             left_file.read(reinterpret_cast<char *>(&left_point.z), sizeof(float));
    //             left_file.read(reinterpret_cast<char *>(&left_point.intensity), sizeof(float));
    //             left_pcl_cloud.points.push_back(left_point);
    //         }
    //         left_file.close();
    //         sensor_msgs::PointCloud2 left_cloud;
    //         pcl::toROSMsg(left_pcl_cloud, left_cloud);
    //         left_cloud.header.stamp.fromNSec(left_all_stamps[i]);
    //         left_cloud.header.frame_id = frame_id;
    //         bag.write(left_topic, left_cloud.header.stamp, left_cloud);
    //         ROS_INFO("bag write %s, %u points\n", left_topic.c_str(), left_cloud.height * left_cloud.width);
    //         ROS_INFO("done converting %s\n",  left_frame_file.c_str());
    //         ROS_INFO("left_total %lu, already convert %lu, remain %lu\n", left_total, i + 1, left_total - i - 1);
    //     }

    //     FILE* right_fp = fopen(right_stamp_file.c_str(), "r");
    //     int64_t right_stamp;
    //     std::vector<int64_t> right_all_stamps;
    //     while (fscanf(right_fp, "%ld\n", &right_stamp) == 1) {
    //         right_all_stamps.push_back(right_stamp);
    //     }
    //     fclose(right_fp);

    //     size_t right_total = right_all_stamps.size();
    //     for (size_t i = 0; i < right_all_stamps.size(); ++i) {
    //         std::string right_st = std::to_string(right_all_stamps[j]);
    //         std::string right_frame_file = right_data_dir + "/" + right_st + ".bin";
    //         ROS_INFO("converting %s\n", right_frame_file.c_str());
    //         if (!boost::filesystem::exists(right_frame_file)) {
    //             ROS_WARN("%s not exist\n", right_frame_file.c_str());
    //             continue;
    //         }
    //         std::ifstream right_file;
    //         right_file.open(right_frame_file, std::ios::in | std::ios::binary);
    //         pcl::PointCloud<pcl::PointXYZI> right_pcl_cloud;
    //         //pcl::PointCloud<velodyne_ros::Point> right_velodyne_cloud
    //         while (!right_file.eof()) {
    //             pcl::PointXYZI right_point;
    //             right_file.read(reinterpret_cast<char *>(&right_point.x), sizeof(float));
    //             right_file.read(reinterpret_cast<char *>(&right_point.y), sizeof(float));
    //             right_file.read(reinterpret_cast<char *>(&right_point.z), sizeof(float));
    //             right_file.read(reinterpret_cast<char *>(&right_point.intensity), sizeof(float));
    //             Eigen::Vector3d raw_point = Eigen::Vector3d(right_point.x, right_point.y, right_point.z);
    //             Eigen::Vector3d L_point = R_v_L_lidar.inverse()*(R_v_R_lidar * raw_point + t_v_R_lidar - t_v_L_lidar);
    //             right_point.x = L_point.x();
    //             right_point.y = L_point.y();
    //             right_point.z = L_point.z();
    //             right_pcl_cloud.points.push_back(right_point);
    //         }
    //         right_file.close();
    //         sensor_msgs::PointCloud2 right_cloud;
    //         pcl::toROSMsg(right_pcl_cloud, right_cloud);
    //         right_cloud.header.stamp.fromNSec(right_all_stamps[j]);
    //         right_cloud.header.frame_id = frame_id;
    //         bag.write(right_topic, right_cloud.header.stamp, right_cloud);
    //         ROS_INFO("bag write %s, %u points\n", right_topic.c_str(), right_cloud.height * right_cloud.width);
    //         ROS_INFO("done converting %s\n", right_frame_file.c_str());
    //         ROS_INFO("right_total %lu, already convert %lu, remain %lu\n", right_total, i + 1, right_total - i - 1);
    //     }

    //     bag.close();
    // }
    // }

    // void VelodyneConverter::Convert(const std::string& left_stamp_file, const std::string& left_data_dir,
    //                  const std::string& left_bag_file/*, const std::string& left_topic*/,
    //                  const std::string& right_stamp_file, const std::string& right_data_dir,
    //                  const std::string& right_bag_file, const std::string& topic,
    //                  const std::string& frame_id)
    // {
    //     Eigen::Matrix3d R_v_L_lidar;
    //     Eigen::Vector3d t_v_L_lidar;

    //     Eigen::Matrix3d R_v_R_lidar;
    //     Eigen::Vector3d t_v_R_lidar;

    //     R_v_L_lidar << -0.514169,-0.702457,-0.492122,0.48979,-0.711497,0.503862,-0.704085,0.0180335,0.709886;
    //     t_v_L_lidar << -0.31189,0.394734,1.94661;

    //     R_v_R_lidar << -0.507842, 0.704544, -0.495695, -0.49974, -0.709646, -0.496651, -0.701681, -0.00450156,0.712477;
    //     t_v_R_lidar << -0.306052, -0.417145, 1.95223;

    //     rosbag::Bag bag(left_bag_file, rosbag::bagmode::Write);
    //     bag.setChunkThreshold(768*1024);
    //     bag.setCompression(rosbag::compression::BZ2);

    //     FILE* left_fp = fopen(left_stamp_file.c_str(), "r");
    //     int64_t left_stamp;
    //     std::vector<int64_t> left_all_stamps;
    //     while (fscanf(left_fp, "%ld\n", &left_stamp) == 1) {
    //         left_all_stamps.push_back(left_stamp);
    //     }
    //     fclose(left_fp);

    //     FILE* right_fp = fopen(right_stamp_file.c_str(), "r");
    //     int64_t right_stamp;
    //     std::vector<int64_t> right_all_stamps;
    //     while (fscanf(right_fp, "%ld\n", &right_stamp) == 1) {
    //         right_all_stamps.push_back(right_stamp);
    //     }
    //     fclose(right_fp);

    //     size_t left_total = left_all_stamps.size();
    //     size_t right_total = right_all_stamps.size();
    //     size_t i = 0, j = 0;

    //     while(i < left_all_stamps.size() && j < right_all_stamps.size()){
    //         if(left_all_stamps[i] < right_all_stamps[j]){
    //             std::string st = std::to_string(left_all_stamps[i]);
    //             std::string frame_file = left_data_dir + "/" + st + ".bin";
    //             ROS_INFO("converting %s\n", frame_file.c_str());
    //             if (!boost::filesystem::exists(frame_file)) {
    //                 ROS_WARN("%s not exist\n", frame_file.c_str());
    //                 continue;
    //             }
    //             std::ifstream file;
    //             file.open(frame_file, std::ios::in | std::ios::binary);
    //             // pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    //             pcl::PointCloud< velodyne_ros::Point> pcl_cloud;
    //             while (!file.eof()) {

    //                 velodyne_ros::Point point;
    //                 // pcl::PointXYZI point;

    //                 file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    //                 file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    //                 file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    //                 file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    //                 pcl_cloud.points.push_back(point);
    //             }
    //             file.close();
    //             calculateTime(pcl_cloud,left_all_stamps[i]);

    //             sensor_msgs::PointCloud2 cloud;
    //             pcl::toROSMsg(pcl_cloud, cloud);
    //             cloud.header.stamp.fromNSec( left_all_stamps[i]);
    //             cloud.header.frame_id = frame_id;
    //             bag.write(topic, cloud.header.stamp, cloud);
    //             ROS_INFO("bag write %s, %u points\n", topic.c_str(), cloud.height * cloud.width);
    //             ROS_INFO("done converting %s\n", frame_file.c_str());
    //             ROS_INFO("left_total %lu, already convert %lu, remain %lu\n", left_total, i + 1, left_total - i - 1);
    //             i++;
    //             }
    //         if(left_all_stamps[i] > right_all_stamps[j]){
    //             std::string st = std::to_string(right_all_stamps[j]);
    //             std::string frame_file = right_data_dir + "/" + st + ".bin";
    //             ROS_INFO("converting %s\n", frame_file.c_str());
    //             if (!boost::filesystem::exists(frame_file)) {
    //                 ROS_WARN("%s not exist\n", frame_file.c_str());
    //                 continue;
    //             }
    //             std::ifstream file;
    //             file.open(frame_file, std::ios::in | std::ios::binary);
    //             pcl::PointCloud< velodyne_ros::Point> pcl_cloud;
    //             while (!file.eof()) {
    //                 velodyne_ros::Point point;
    //                 file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    //                 file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    //                 file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    //                 file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    //                 Eigen::Vector3d raw_point = Eigen::Vector3d(point.x, point.y, point.z);
    //                 Eigen::Vector3d L_point = R_v_L_lidar.inverse()*(R_v_R_lidar * raw_point + t_v_R_lidar - t_v_L_lidar);
    //                 point.x = L_point.x();
    //                 point.y = L_point.y();
    //                 point.z = L_point.z();
    //                 pcl_cloud.points.push_back(point);
    //             }
    //             file.close();
    //             calculateTime(pcl_cloud,left_all_stamps[i]);

    //             sensor_msgs::PointCloud2 cloud;
    //             pcl::toROSMsg(pcl_cloud, cloud);
    //             cloud.header.stamp.fromNSec( right_all_stamps[j]);
    //             cloud.header.frame_id = frame_id;
    //             bag.write(topic, cloud.header.stamp, cloud);
    //             ROS_INFO("bag write %s, %u points\n", topic.c_str(), cloud.height * cloud.width);
    //             ROS_INFO("done converting %s\n", frame_file.c_str());
    //             ROS_INFO("right_total %lu, already convert %lu, remain %lu\n", right_total, i + 1, right_total - i - 1);
    //             j++;
    //         }
    //     }
    //     if(!(i < left_all_stamps.size()) && j < right_all_stamps.size()){
    //         while(j < right_all_stamps.size()){
    //             std::string st = std::to_string(right_all_stamps[j]);
    //             std::string frame_file = right_data_dir + "/" + st + ".bin";
    //             ROS_INFO("converting %s\n", frame_file.c_str());
    //             if (!boost::filesystem::exists(frame_file)) {
    //                 ROS_WARN("%s not exist\n", frame_file.c_str());
    //                 continue;
    //             }
    //             std::ifstream file;
    //             file.open(frame_file, std::ios::in | std::ios::binary);
    //             pcl::PointCloud< velodyne_ros::Point> pcl_cloud;
    //             while (!file.eof()) {
    //                 velodyne_ros::Point point;
    //                 file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    //                 file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    //                 file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    //                 file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    //                 Eigen::Vector3d raw_point = Eigen::Vector3d(point.x, point.y, point.z);
    //                 Eigen::Vector3d L_point = R_v_L_lidar.inverse()*(R_v_R_lidar * raw_point + t_v_R_lidar - t_v_L_lidar);
    //                 point.x = L_point.x();
    //                 point.y = L_point.y();
    //                 point.z = L_point.z();
    //                 pcl_cloud.points.push_back(point);
    //             }
    //             file.close();
    //             calculateTime(pcl_cloud,left_all_stamps[i]);

    //             sensor_msgs::PointCloud2 cloud;
    //             pcl::toROSMsg(pcl_cloud, cloud);
    //             cloud.header.stamp.fromNSec( right_all_stamps[j]);
    //             cloud.header.frame_id = frame_id;
    //             bag.write(topic, cloud.header.stamp, cloud);
    //             ROS_INFO("bag write %s, %u points\n", topic.c_str(), cloud.height * cloud.width);
    //             ROS_INFO("done converting %s\n", frame_file.c_str());
    //             ROS_INFO("right_total %lu, already convert %lu, remain %lu\n", right_total, i + 1, right_total - i - 1);
    //             j++;
    //         }
    //     }

    //     if(i < left_all_stamps.size() && !(j < right_all_stamps.size())){
    //         while(i < left_all_stamps.size()){
    //             std::string st = std::to_string(left_all_stamps[i]);
    //             std::string frame_file = left_data_dir + "/" + st + ".bin";
    //             ROS_INFO("converting %s\n", frame_file.c_str());
    //             if (!boost::filesystem::exists(frame_file)) {
    //                 ROS_WARN("%s not exist\n", frame_file.c_str());
    //                 continue;
    //             }
    //             std::ifstream file;
    //             file.open(frame_file, std::ios::in | std::ios::binary);
    //             pcl::PointCloud< velodyne_ros::Point> pcl_cloud;
    //             while (!file.eof()) {
    //                 velodyne_ros::Point point;
    //                 file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
    //                 file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
    //                 file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
    //                 file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
    //                 pcl_cloud.points.push_back(point);
    //             }
    //             file.close();
    //             calculateTime(pcl_cloud,left_all_stamps[i]);

    //             sensor_msgs::PointCloud2 cloud;
    //             pcl::toROSMsg(pcl_cloud, cloud);
    //             cloud.header.stamp.fromNSec( left_all_stamps[i]);
    //             cloud.header.frame_id = frame_id;
    //             bag.write(topic, cloud.header.stamp, cloud);
    //             ROS_INFO("bag write %s, %u points\n", topic.c_str(), cloud.height * cloud.width);
    //             ROS_INFO("done converting %s\n", frame_file.c_str());
    //             ROS_INFO("left_total %lu, already convert %lu, remain %lu\n", left_total, i + 1, left_total - i - 1);
    //             i++;
    //         }
    //     }
    //     bag.close();
    // }
    void VelodyneConverter::Convert(const std::string &left_stamp_file, const std::string &left_data_dir,
                                    const std::string &left_bag_file /*, const std::string& left_topic*/,
                                    const std::string &right_stamp_file, const std::string &right_data_dir,
                                    const std::string &right_bag_file, const std::string &topic,
                                    const std::string &frame_id)
    {
        rosbag::Bag bag(left_bag_file, rosbag::bagmode::Write);
        bag.setChunkThreshold(768 * 1024);
        bag.setCompression(rosbag::compression::BZ2);

        FILE *left_fp = fopen(left_stamp_file.c_str(), "r");
        int64_t left_stamp;
        std::vector<int64_t> left_all_stamps, left_all_stamps_0;
        while (fscanf(left_fp, "%ld\n", &left_stamp) == 1)
        {
            left_all_stamps.push_back(left_stamp);
        }
        fclose(left_fp);

        FILE *right_fp = fopen(right_stamp_file.c_str(), "r");
        int64_t right_stamp;
        std::vector<int64_t> right_all_stamps, right_all_stamps_0;
        while (fscanf(right_fp, "%ld\n", &right_stamp) == 1)
        {
            right_all_stamps.push_back(right_stamp);
        }
        fclose(right_fp);

        size_t left_total = left_all_stamps.size();
        size_t right_total = right_all_stamps.size();
        size_t i = 0, j = 0, begin = 0, end = 0;
        if (left_all_stamps[0] <= right_all_stamps[0])
        {
            begin = 0;
        }
        else
        {
            while (!(left_all_stamps[0] <= right_all_stamps[begin]))
                begin++;
        }
        if (left_all_stamps[left_all_stamps.size() - 1] <= right_all_stamps[right_all_stamps.size() - 1])
        {
            while (left_all_stamps[left_all_stamps.size() - 1] <= right_all_stamps[right_all_stamps.size() - 1 - end])
                end++;
        }
        else
        {
            end = 0;
        }

        std::vector<pcl::PointCloud<velodyne_ros::Point>> left_point_cloud;
        std::vector<pcl::PointCloud<velodyne_ros::Point>> right_point_cloud;

        std::vector<bool> is_first_left, is_first_right;
        is_first_left.resize(N_SCAN);
        fill(is_first_left.begin(), is_first_left.end(), true);
        is_first_right.resize(N_SCAN);
        fill(is_first_right.begin(), is_first_right.end(), true);

        std::vector<double> yaw_first_point_left, yaw_first_point_right;
        yaw_first_point_left.resize(N_SCAN);
        fill(yaw_first_point_left.begin(), yaw_first_point_left.end(), 0.0);
        yaw_first_point_right.resize(N_SCAN);
        fill(yaw_first_point_right.begin(), yaw_first_point_right.end(), 0.0);

        while (i < left_all_stamps.size())
        {
            std::string st = std::to_string(left_all_stamps[i]);
            std::string frame_file = left_data_dir + "/" + st + ".bin";
            ROS_INFO("converting %s\n", frame_file.c_str());
            if (!boost::filesystem::exists(frame_file))
            {
                ROS_WARN("%s not exist\n", frame_file.c_str());
                continue;
            }
            std::ifstream file;
            file.open(frame_file, std::ios::in | std::ios::binary);
            pcl::PointCloud<velodyne_ros::Point> pcl_cloud, pcl_cloud_left;
            while (!file.eof())
            {
                velodyne_ros::Point point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                // // 计算竖直方向上的角度（雷达的第几线）
                // verticalAngle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
                // // rowIdn计算出该点激光雷达是竖直方向上第几线的
                // // 从下往上计数，-15度记为初始线，第0线，一共16线(N_SCAN=16)
                // rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
                // // std::cout << "ring : " << rowIdn << std::endl;
                // if (rowIdn < 0 || rowIdn >= N_SCAN)
                // {
                //     ROS_INFO("%d point : rowIdn < 0 or rowIdn >= 16\n", i);
                //     i++;
                //     continue;
                // }
                // point.ring = rowIdn;

                // double omega = 0.361 * SCAN_RATE;
                // double yaw_angle = atan2(point.y, point.x) * 57.2957;
                // int layer = point.ring;
                // if (is_first_left[layer])
                // {
                //     yaw_first_point_left[layer] = yaw_angle;
                //     is_first_left[layer] = false;
                //     relative_time = 0.0;
                //     i++;
                //     continue;
                // }
                // // compute offset time
                // if (yaw_angle <= yaw_first_point_left[layer])
                // {
                //     relative_time = (yaw_first_point_left[layer] - yaw_angle) / omega;
                // }
                // else
                // {
                //     relative_time = (yaw_first_point_left[layer] - yaw_angle + 360.0) / omega;
                // }
                // if (relative_time / double(1000) >= 0.1)
                // {
                //     ROS_INFO("%d relative_time  >= 0.1\n", i);
                //     i++;
                //     continue;
                // }
                // point.time = relative_time / double(1000);
                // i++;

                pcl_cloud.points.push_back(point);
            }
            file.close();
            calculateTime(pcl_cloud, left_all_stamps[i]/*, pcl_cloud_left, left_all_stamps_0*/);
            left_point_cloud.push_back(pcl_cloud_left);

            // sensor_msgs::PointCloud2 cloud;
            // pcl::toROSMsg(pcl_cloud, cloud);
            // cloud.header.stamp.fromNSec( left_all_stamps[i]);
            // cloud.header.frame_id = frame_id;
            // bag_left.write(topic, cloud.header.stamp, cloud);
            // ROS_INFO("bag_left write %s, %u points\n", topic.c_str(), cloud.height * cloud.width);
            ROS_INFO("done converting %s\n", frame_file.c_str());
            ROS_INFO("left_total %lu, already convert %lu, remain %lu\n", left_total, i + 1, left_total - i - 1);
            i++;
        }
        // bag_left.close();

        while (j < right_all_stamps.size())
        {
            std::string st = std::to_string(right_all_stamps[j]);
            std::string frame_file = right_data_dir + "/" + st + ".bin";
            ROS_INFO("converting %s\n", frame_file.c_str());
            if (!boost::filesystem::exists(frame_file))
            {
                ROS_WARN("%s not exist\n", frame_file.c_str());
                continue;
            }
            std::ifstream file;
            file.open(frame_file, std::ios::in | std::ios::binary);
            pcl::PointCloud<velodyne_ros::Point> pcl_cloud,pcl_cloud_right;

            while (!file.eof())
            {
                velodyne_ros::Point point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                // // 计算竖直方向上的角度（雷达的第几线）
                // verticalAngle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
                // // rowIdn计算出该点激光雷达是竖直方向上第几线的
                // // 从下往上计数，-15度记为初始线，第0线，一共16线(N_SCAN=16)
                // rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
                // // std::cout << "ring : " << rowIdn << std::endl;
                // if (rowIdn < 0 || rowIdn >= N_SCAN)
                // {
                //     ROS_INFO("%d point : rowIdn < 0 or rowIdn >= 16\n", i);
                //     j++;
                //     continue;
                // }
                // point.ring = rowIdn;

                // double omega = 0.361 * SCAN_RATE;
                // double yaw_angle = atan2(point.y, point.x) * 57.2957;
                // int layer = point.ring;
                // if (is_first_right[layer])
                // {
                //     yaw_first_point_right[layer] = yaw_angle;
                //     is_first_right[layer] = false;
                //     relative_time = 0.0;
                //     j++;
                //     continue;
                // }
                // // compute offset time
                // if (yaw_angle <= yaw_first_point_right[layer])
                // {
                //     relative_time = (yaw_first_point_right[layer] - yaw_angle) / omega;
                // }
                // else
                // {
                //     relative_time = (yaw_first_point_right[layer] - yaw_angle + 360.0) / omega;
                // }
                // if (relative_time / double(1000) >= 0.1)
                // {
                //     ROS_INFO("%d relative_time  >= 0.1\n", i);
                //     j++;
                //     continue;
                // }
                // point.time = relative_time / double(1000);
                // j++;

                pcl_cloud.points.push_back(point);
            }
            file.close();
            calculateTime(pcl_cloud, right_all_stamps[j]/*, pcl_cloud_right, right_all_stamps_0*/);
            R2Lconvert(pcl_cloud_right);
            right_point_cloud.push_back(pcl_cloud_right);

            // sensor_msgs::PointCloud2 cloud;
            // pcl::toROSMsg(pcl_cloud, cloud);
            // cloud.header.stamp.fromNSec( right_all_stamps[j]);
            // cloud.header.frame_id = frame_id;
            // bag_right.write(topic, cloud.header.stamp, cloud);
            // ROS_INFO("bag_right write %s, %u points\n", topic.c_str(), cloud.height * cloud.width);
            ROS_INFO("done converting %s\n", frame_file.c_str());
            ROS_INFO("right_total %lu, already convert %lu, remain %lu\n", right_total, j + 1, right_total - j - 1);
            j++;
        }
        // bag_right.close();

        int size_left = left_all_stamps.size(), size_right = right_all_stamps.size();

        if (begin)
        {
            right2left(right_point_cloud[begin], right_all_stamps[begin], left_point_cloud[0], left_point_cloud[1], left_all_stamps[0], left_all_stamps[1]);
        }
        if (end)
        {
            right2left(right_point_cloud[size_right - end], right_all_stamps[size_right - end], left_point_cloud[size_left - 1], left_point_cloud[size_left - 1], left_all_stamps[size_left - 1], left_all_stamps[size_left - 1]);
        }

        int m = 0, n = 0;

        while (m < size_right - begin - end)
        {
            if (m < size_left)
                right2left(right_point_cloud[m + begin], right_all_stamps[m + begin], left_point_cloud[m], left_point_cloud[m + 1], left_all_stamps[m], left_all_stamps[m + 1]);
            ROS_INFO("%d  is transformed ...\n", m + begin + end);
            m++;
        }

        size_left = left_all_stamps.size();
        while (n < size_left)
        {
            sensor_msgs::PointCloud2 cloud;
            pcl::toROSMsg(left_point_cloud[n], cloud);
            cloud.header.stamp.fromNSec(left_all_stamps[n]);
            cloud.header.frame_id = frame_id;
            bag.write(topic, cloud.header.stamp, cloud);
            n++;
            ROS_INFO("%d sweep is saved     remain %d\n", n, size_left - n);
            ROS_INFO("bag write %s, %u points\n", topic.c_str(), cloud.height * cloud.width);
        }
        std::cout << "begin : " << begin << "     end : " << end << std::endl;
        bag.close();
    }
    void VelodyneConverter::calculateTime(pcl::PointCloud<velodyne_ros::Point> &pcl_cloud, int64_t timestamp/*, pcl::PointCloud<velodyne_ros::Point> &pcl_cloud_, std::vector<int64_t> &timestamps*/)
    {
        std::vector<bool> is_first;
        is_first.resize(N_SCAN);
        fill(is_first.begin(), is_first.end(), true);

        std::vector<double> yaw_first_point;
        yaw_first_point.resize(N_SCAN);
        fill(yaw_first_point.begin(), yaw_first_point.end(), 0.0);

        int i = 0;

        // std::ofstream foutC2(std::string("/home/lang/code/LIW/src/lio_opt/debug/dt.txt"), std::ios::app);

        // foutC2.setf(std::ios::scientific, std::ios::floatfield);
        // foutC2.precision(6);

        while (i < pcl_cloud.points.size())
        {
            // 计算竖直方向上的角度（雷达的第几线）
            verticalAngle = atan2(pcl_cloud.points[i].z, sqrt(pcl_cloud.points[i].x * pcl_cloud.points[i].x + pcl_cloud.points[i].y * pcl_cloud.points[i].y)) * 180 / M_PI;

            // rowIdn计算出该点激光雷达是竖直方向上第几线的
            // 从下往上计数，-15度记为初始线，第0线，一共16线(N_SCAN=16)
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            // std::cout << "ring : " << rowIdn << std::endl;

            if (rowIdn < 0 || rowIdn >= N_SCAN)
            {
                // pcl_cloud.points.erase(pcl_cloud.points.begin() + i);
                pcl_cloud.points[i].x=0;
                pcl_cloud.points[i].y=0;
                pcl_cloud.points[i].z=0;
                ROS_INFO("%d point : rowIdn < 0 or rowIdn >= 16\n", i);
                i++;
                continue;
            }

            pcl_cloud.points[i].ring = rowIdn;

            double omega = 0.361 * SCAN_RATE;
            double yaw_angle = atan2(pcl_cloud.points[i].y, pcl_cloud.points[i].x) * 57.2957;
            int layer = pcl_cloud.points[i].ring;

            if (is_first[layer])
            {
                yaw_first_point[layer] = yaw_angle;
                is_first[layer] = false;
                relative_time = 0.0;
                i++;
                continue;
            }
            // compute offset time
            if (yaw_angle <= yaw_first_point[layer])
            {
                relative_time = (yaw_first_point[layer] - yaw_angle) / omega;
            }
            else
            {
                relative_time = (yaw_first_point[layer] - yaw_angle + 360.0) / omega;
            }
            if (relative_time / double(1000) >= 0.1)
            {
                // pcl_cloud.points.erase(pcl_cloud.points.begin() + i);
                pcl_cloud.points[i].x=0;
                pcl_cloud.points[i].y=0;
                pcl_cloud.points[i].z=0;
                ROS_INFO("%d relative_time  >= 0.1\n", i);
                i++;
                continue;
            }
            pcl_cloud.points[i].time = relative_time / double(1000);
            // foutC2 << std::fixed << pcl_cloud.points[i].time << std::endl;
            // pcl_cloud_.points.push_back(pcl_cloud.points[i]);
            // timestamps.push_back(timestamp);
            i++;
        }
        // foutC2.close();
    }

    void VelodyneConverter::right2left(pcl::PointCloud<velodyne_ros::Point> &right_cloud, int64_t timestamp, pcl::PointCloud<velodyne_ros::Point> &left_cloud_0, pcl::PointCloud<velodyne_ros::Point> &left_cloud_1, int64_t timestamp_0, int64_t timestamp_1)
    {
        int i = 0;
        // std::ofstream foutC2(std::string("/home/lang/code/LIW/src/lio_opt/debug/dt.txt"), std::ios::app);
        // foutC2.setf(std::ios::scientific, std::ios::floatfield);
        // foutC2.precision(6);

        while (i < right_cloud.points.size())
        {
            if (right_cloud.points[i].time + timestamp < timestamp_0)
            {
                i++;
                continue;
            }
            if (right_cloud.points[i].time + timestamp >= timestamp_0 && right_cloud.points[i].time + timestamp <= timestamp_1)
            {
                // foutC2 << std::fixed << right_cloud.points[i].time << std::endl;
                left_cloud_0.points.push_back(right_cloud.points[i]);
                i++;
            }
            if (right_cloud.points[i].time + timestamp > timestamp_1)
            {
                // foutC2 << std::fixed << right_cloud.points[i].time << std::endl;
                left_cloud_1.points.push_back(right_cloud.points[i]);
                i++;
            }
        }
        // foutC2.close();
    }

    void VelodyneConverter::R2Lconvert(pcl::PointCloud<velodyne_ros::Point> &right_cloud)
    {
        int i = 0;
        Eigen::Matrix3d R_v_L_lidar;
        Eigen::Vector3d t_v_L_lidar;

        Eigen::Matrix3d R_v_R_lidar;
        Eigen::Vector3d t_v_R_lidar;

        R_v_L_lidar << -0.514169, -0.702457, -0.492122, 0.48979, -0.711497, 0.503862, -0.704085, 0.0180335, 0.709886;
        t_v_L_lidar << -0.31189, 0.394734, 1.94661;

        R_v_R_lidar << -0.507842, 0.704544, -0.495695, -0.49974, -0.709646, -0.496651, -0.701681, -0.00450156, 0.712477;
        t_v_R_lidar << -0.306052, -0.417145, 1.95223;
        // std::ofstream foutC2(std::string("/home/lang/code/LIW/src/lio_opt/debug/dt.txt"), std::ios::app);
        // foutC2.setf(std::ios::scientific, std::ios::floatfield);
        // foutC2.precision(6);
        while (i < right_cloud.size())
        {
            Eigen::Vector3d raw_point = Eigen::Vector3d(right_cloud.points[i].x, right_cloud.points[i].y, right_cloud.points[i].z);
            Eigen::Vector3d L_point = R_v_L_lidar.inverse() * (R_v_R_lidar * raw_point + t_v_R_lidar - t_v_L_lidar);
            right_cloud.points[i].x = L_point.x();
            right_cloud.points[i].y = L_point.y();
            right_cloud.points[i].z = L_point.z();
            i++;

            // foutC2 << std::fixed << right_cloud.points[i].time << std::endl;
            // foutC2.close();
        }
    }
}
// namespace kaist2bag