//
// Created by tao on 7/25/21.
//

#ifndef SRC_VELODYNE_CONVERTER_H
#define SRC_VELODYNE_CONVERTER_H
#include <string>
#include "converter.h"
#include <Eigen/Core>
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace velodyne_ros {
	struct EIGEN_ALIGN16 Point {
	  PCL_ADD_POINT4D;
	  float intensity;
	  float time;
	  uint16_t ring;
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, time, time)
    (uint16_t, ring, ring)
)

namespace kaist2bag {

class VelodyneConverter : public Converter {
public:
    VelodyneConverter(const std::string& dataset_dir, const std::string& save_dir,
                      const std::string& topic, const std::string& right_topic);
    virtual ~VelodyneConverter() = default;

    int Convert() override;

    std::string default_left_stamp_file = "sensor_data/VLP_left_stamp.csv";
    std::string default_left_data_dir = "sensor_data/VLP_left";
    std::string default_right_stamp_file = "sensor_data/VLP_right_stamp.csv";
    std::string default_right_data_dir = "sensor_data/VLP_right";

private:
    std::string left_topic_;
    std::string left_bag_name_;
    std::string right_topic_;
    std::string right_bag_name_;

    float verticalAngle;
    size_t rowIdn;
    double relative_time;

    const int N_SCAN = 16;  //16线
    const int Horizon_SCAN = 1800; // 每线1800个点的数据
    const float ang_res_x = 0.2; // 水平上每条线间隔0.2°
    const float ang_res_y = 2.0; // 竖直方向上每条线间隔2°
    const float ang_bottom = 15.0+0.1; // 竖直方向上起始角度是负角度，与水平方向相差15.1°
    const int groundScanInd = 7; // 以多少个扫描圈来表示地面
    const int  SCAN_RATE = 10; 

    void Convert(const std::string& left_stamp_file, const std::string& left_data_dir,
                 const std::string& left_bag_file/*, const std::string& left_topic*/,
                 const std::string& right_stamp_file, const std::string& right_data_dir,
                 const std::string& right_bag_file, const std::string& topic,
                 const std::string& frame_id);
    // void Convert(const std::string& left_stamp_file, const std::string& left_data_dir,
    //              const std::string& left_bag_file, const std::string& left_topic,
    //              const std::string& right_stamp_file, const std::string& right_data_dir,
    //              const std::string& right_bag_file, const std::string& right_topic,
    //              const std::string& frame_id);
    void calculateTime(pcl::PointCloud<velodyne_ros::Point> &pcl_cloud, int64_t timestamp/*, pcl::PointCloud<velodyne_ros::Point> &pcl_cloud_, std::vector<int64_t> &timestamps*/);
    void right2left(pcl::PointCloud< velodyne_ros::Point> &right_cloud,int64_t timestamp,pcl::PointCloud< velodyne_ros::Point> &left_cloud_0,pcl::PointCloud< velodyne_ros::Point> &left_cloud_1,int64_t timestamp_0,int64_t timestamp_1);
    void R2Lconvert(pcl::PointCloud<velodyne_ros::Point> &right_cloud);
};


} // namespace kaist2bag
#endif //SRC_VELODYNE_CONVERTER_H
