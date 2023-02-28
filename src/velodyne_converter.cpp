//
// Created by tao on 7/25/21.
//
#include <unistd.h>
#include <cmath>
#include "velodyne_converter.h"

#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Dense>

int temp;
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
        std::vector<int64_t> left_all_stamps;
        while (fscanf(left_fp, "%ld\n", &left_stamp) == 1)
        {
            left_all_stamps.push_back(left_stamp);
        }
        fclose(left_fp);

        FILE *right_fp = fopen(right_stamp_file.c_str(), "r");
        int64_t right_stamp;
        std::vector<int64_t> right_all_stamps;
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
        if (left_all_stamps[left_total - 1] <= right_all_stamps[right_total - 1])
        {
            while (left_all_stamps[left_total - 1] <= right_all_stamps[right_total - 1 - end])
                end++;
        }
        else
        {
            end = 0;
        }

        std::vector<pcl::PointCloud<velodyne_ros::Point>> left_point_cloud;
        left_point_cloud.resize(left_total);
        while (i < left_total)
        {
            std::string st = std::to_string(left_all_stamps[i]);
            std::string frame_file = left_data_dir + "/" + st + ".bin";
            pcl::PointCloud<velodyne_ros::Point> pcl_cloud;
            ROS_INFO("converting %s\n", frame_file.c_str());
            if (!boost::filesystem::exists(frame_file))
            {
                ROS_WARN("%s not exist\n", frame_file.c_str());
                i++;
                continue;
            }
            std::ifstream file;
            file.open(frame_file, std::ios::in | std::ios::binary);
            while (!file.eof())
            {
                velodyne_ros::Point point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                pcl_cloud.points.push_back(point);
            }
            file.close();
            calculateTime(pcl_cloud, left_all_stamps[i]);
            left_point_cloud[i] = pcl_cloud;
            ROS_INFO("bag_left write %s, %u points\n", topic.c_str(), pcl_cloud.points.size());
            ROS_INFO("left_total %lu, already convert %lu, remain %lu\n", left_total, i + 1, left_total - i - 1);
            i++;
        }
        while (j < right_total)
        {
            std::string st = std::to_string(right_all_stamps[j]);
            std::string frame_file = right_data_dir + "/" + st + ".bin";
            ROS_INFO("converting %s\n", frame_file.c_str());
            if (!boost::filesystem::exists(frame_file))
            {
                ROS_WARN("%s not exist\n", frame_file.c_str());
                j++;
                continue;
            }
            std::ifstream file;
            file.open(frame_file, std::ios::in | std::ios::binary);
            pcl::PointCloud<velodyne_ros::Point> pcl_cloud;
            while (!file.eof())
            {
                velodyne_ros::Point point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
                pcl_cloud.points.push_back(point);
            }
            file.close();
            calculateTime(pcl_cloud, right_all_stamps[j]);
            R2Lconvert(pcl_cloud);
            if ((begin < j) && (j - begin + 1 < left_total))
            {
                right2left(pcl_cloud, right_all_stamps[j], left_point_cloud[j - begin], left_point_cloud[j - begin + 1], left_all_stamps[j - begin], left_all_stamps[j - begin + 1]);
                ROS_INFO("done %d  is transformed ...\n", j - begin);

                sensor_msgs::PointCloud2 cloud;
                pcl::toROSMsg(left_point_cloud[j - begin], cloud);
                cloud.header.stamp.fromNSec(left_all_stamps[j - begin]);
                cloud.header.frame_id = frame_id;
                bag.write(topic, cloud.header.stamp, cloud);
                ROS_INFO("%d sweep is saved     remain %d\n", j - begin + 1, left_total - j + begin - 1);
            }
            ROS_INFO("bag_right write %s, %u points\n", topic.c_str(), pcl_cloud.points.size());
            ROS_INFO("right_total %lu, already convert %lu, remain %lu\n", right_total, j + 1, right_total - j - 1);
            j++;
            // std::vector<velodyne_ros::Point>().swap(left_point_cloud[j - begin]);
            // ROS_INFO("left_points_cloud size :  %d",left_point_cloud[j - begin].points.size());

            // if (( j >= begin) && (j - begin + 1 < left_total))
            // {
            //     right2left(pcl_cloud, right_all_stamps[j], left_point_cloud[0], left_point_cloud[1], left_all_stamps[j - begin], left_all_stamps[j - begin + 1]);
            //     ROS_INFO("done %d  is transformed ...\n", j - begin);
            // }
            // ROS_INFO("bag_right write %s, %u points\n", topic.c_str(), pcl_cloud.points.size());
            // ROS_INFO("right_total %lu, already convert %lu, remain %lu\n", right_total, j + 1, right_total - j - 1);
            // j++;

            // sensor_msgs::PointCloud2 cloud;
            // pcl::toROSMsg(left_point_cloud[0], cloud);
            // cloud.header.stamp.fromNSec(left_all_stamps[j - begin]);
            // cloud.header.frame_id = frame_id;
            // bag.write(topic, cloud.header.stamp, cloud);
            // ROS_INFO("%d sweep is saved   remain %d  stamp: %f\n", j - begin + 1, left_total - j + begin - 1,cloud.header.stamp.fromNSec(left_all_stamps[j - begin]));

            // left_point_cloud.erase(left_point_cloud.begin());
            // std::vector<pcl::PointCloud<velodyne_ros::Point>>(left_point_cloud).swap(left_point_cloud);
            // ROS_INFO("left_points_cloud size :  %d",left_point_cloud.size());

            // ROS_INFO("bag write %s\n", topic.c_str());
        }

        // int m = 0, n = 0;
        // while (n < left_total)
        // {
        //     sensor_msgs::PointCloud2 cloud;
        //     pcl::toROSMsg(left_point_cloud[n], cloud);
        //     cloud.header.stamp.fromNSec(left_all_stamps[n]);
        //     cloud.header.frame_id = frame_id;
        //     bag.write(topic, cloud.header.stamp, cloud);
        //     n++;
        //     ROS_INFO("%d sweep is saved     remain %d\n", n+1, left_total - n - 1);
        //     ROS_INFO("bag write %s\n", topic.c_str());
        // }
        bag.close();
    }
    void VelodyneConverter::calculateTime(pcl::PointCloud<velodyne_ros::Point> &pcl_cloud, int64_t timestamp)
    {
        std::vector<bool> is_first;
        is_first.resize(N_SCAN);
        fill(is_first.begin(), is_first.end(), true);

        std::vector<double> yaw_first_point;
        yaw_first_point.resize(N_SCAN);
        fill(yaw_first_point.begin(), yaw_first_point.end(), 0.0);

        int i = 0;

        while (i < pcl_cloud.points.size())
        {
            // 计算竖直方向上的角度（雷达的第几线）
            verticalAngle = atan2(pcl_cloud.points[i].z, sqrt(pcl_cloud.points[i].x * pcl_cloud.points[i].x + pcl_cloud.points[i].y * pcl_cloud.points[i].y)) * 180 / M_PI;

            // rowIdn计算出该点激光雷达是竖直方向上第几线的
            // 从下往上计数，-15度记为初始线，第0线，一共16线(N_SCAN=16)
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            // std::cout << "ring : " << rowIdn << std::endl;
            // std::cout << i << " point rowIdn :  " << rowIdn << std::endl;

            if (rowIdn < 0 || rowIdn >= N_SCAN)
            {
                i=pcl_cloud.points.erase(pcl_cloud.points.begin() + i) - pcl_cloud.points.begin();
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
            if (relative_time / double(1000) >= 0.1 && relative_time / double(1000) < 0.0)
            {
                i=pcl_cloud.points.erase(pcl_cloud.points.begin() + i) - pcl_cloud.points.begin();
                continue;
            }
            pcl_cloud.points[i].time = relative_time / float(1000);
            i++;
        }
    }

    void VelodyneConverter::right2left(pcl::PointCloud<velodyne_ros::Point> &right_cloud, int64_t timestamp, pcl::PointCloud<velodyne_ros::Point> &left_cloud_0, pcl::PointCloud<velodyne_ros::Point> &left_cloud_1, int64_t timestamp_0, int64_t timestamp_1)
    {
        int i = 0;
        float timestamp_0_ = float(timestamp - timestamp_0) / 1000000000;
        float timestamp_1_ = float(timestamp_1 - timestamp) / 1000000000;
        int left_0_size = left_cloud_0.points.size(), left_1_size = left_cloud_1.points.size();
        left_cloud_0.points.reserve(left_0_size + right_cloud.points.size());
        left_cloud_1.points.reserve(left_1_size + right_cloud.points.size());
        while (i < right_cloud.points.size())
        {
            // ROS_INFO("convert    %d size     %d i\n", right_cloud.points.size(), i);

            if (right_cloud.points[i].time >= 0.1)
            {
                i++;
                continue;
            }
            if (right_cloud.points[i].time + timestamp_0_ < 0)
            {
                i++;
                continue;
            }
            if (right_cloud.points[i].time + timestamp_0_ >= 0 && right_cloud.points[i].time <= timestamp_1_)
            {
                right_cloud.points[i].time = right_cloud.points[i].time + timestamp_0_;
                if (right_cloud.points[i].time >= 0.1 || right_cloud.points[i].time < 0.0)
                {
                    i++;
                    continue;
                }
                left_cloud_0.points.push_back(right_cloud.points[i]);
                i++;
            }
            if (right_cloud.points[i].time > timestamp_1_)
            {
                right_cloud.points[i].time = right_cloud.points[i].time - timestamp_1_;
                if (right_cloud.points[i].time >= 0.1 || right_cloud.points[i].time < 0.0)
                {
                    i++;
                    continue;
                }
                left_cloud_1.points.push_back(right_cloud.points[i]);
                i++;
            }
        }
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

        while (i < right_cloud.size())
        {
            Eigen::Vector3d raw_point = Eigen::Vector3d(right_cloud.points[i].x, right_cloud.points[i].y, right_cloud.points[i].z);
            Eigen::Vector3d L_point = R_v_L_lidar.inverse() * (R_v_R_lidar * raw_point + t_v_R_lidar - t_v_L_lidar);
            if (isnan(L_point.x()) || isnan(L_point.y()) || isnan(L_point.z()))
            {
                ROS_INFO("point is nan \n");
                right_cloud.points.erase(right_cloud.points.begin() + i);
                i++;
            }
            else
            {
                right_cloud.points[i].x = L_point.x();
                right_cloud.points[i].y = L_point.y();
                right_cloud.points[i].z = L_point.z();
                i++;
            }
        }
    }
}
// namespace kaist2bag