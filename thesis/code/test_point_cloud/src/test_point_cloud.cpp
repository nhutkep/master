#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_point_cloud");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("test_point_cloud", 1);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;
    // Fill in the cloud data
    cloud.width    = 5;
    cloud.height   = 5;
    cloud.is_dense = false;
    cloud.resize (cloud.width * cloud.height);

    // for (auto& point: cloud)
    // {
    //     point.x = 0.2 * rand () / (RAND_MAX + 1.0f);
    //     point.y = 0.2 * rand () / (RAND_MAX + 1.0f);
    //     point.z = 0.5 * rand () / (RAND_MAX + 1.0f);
    // }
    // cloud.width = 50000;
    // cloud.height = 2;
    // cloud.points.resize(cloud.width * cloud.height);
    // for (size_t i = 0; i < cloud.points.size (); ++i)
    // {
    //     cloud.points[i].x = 512 * rand () / (RAND_MAX + 1.0f);
    //     cloud.points[i].y = 512 * rand () / (RAND_MAX + 1.0f);
    //     cloud.points[i].z = 512 * rand () / (RAND_MAX + 1.0f);
    // }

    // pcl::toROSMsg(cloud, output);
    // output.header.frame_id = "point_cloud";
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        for (auto& point: cloud)
        {
            point.x = 0.2 * rand () / (RAND_MAX + 1.0f);
            point.y = 0.2 * rand () / (RAND_MAX + 1.0f);
            point.z = 0.5 * rand () / (RAND_MAX + 1.0f);
        }
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "point_cloud";
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}