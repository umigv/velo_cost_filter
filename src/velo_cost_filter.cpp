
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <iostream>

struct VeloCostFilter {
public:
    explicit VeloCostFilter(ros::NodeHandle &node)
    : last_stamp_{ 0, 0 },
      publisher_{ node.advertise<sensor_msgs::PointCloud2>("velodyne_points_filtered", 10) }
    { }

    void filter(const sensor_msgs::PointCloud2::ConstPtr &velo_ptr) {
        //const auto now = ros::Time::now();
        //const auto stamp = imu_ptr->header.stamp;

        sensor_msgs::PointCloud2 new_data = *velo_ptr;

        for (int i = 0; i < (new_data.height*new_data.width); ++i) {
            if (new_data.data[i] == 0) new_data.data[i] = 255;
        }
        publisher_.publish(new_data);
    }
    

private:
    ros::Time last_stamp_;
    ros::Publisher publisher_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "velodyne_filter_node");
    ros::NodeHandle node;
    ROS_INFO("Velodyne Cost Filter launched");

    VeloCostFilter filter{ node };
    const auto subscriber =
        node.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 10,
                                         &VeloCostFilter::filter, &filter);

    ros::spin();
}
