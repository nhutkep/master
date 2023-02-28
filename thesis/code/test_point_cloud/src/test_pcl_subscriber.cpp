#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/convex_hull.h>
#include "my_quickhull/QuickHull.hpp"


class CloudHandler
{
    protected:
        ros::NodeHandle nh;
        ros::Subscriber pcl_sub;
        ros::Publisher pcl_pub;
        quickhull::QuickHull<float> qh;

    public:
        CloudHandler()
        {
            pcl_sub = nh.subscribe("test_point_cloud", 10, &CloudHandler::cloudCB, this);
            pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("my_convex_hull_points", 1);
        }
        void cloudCB(const sensor_msgs::PointCloud2& input)
        {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::PointCloud<pcl::PointXYZ> cHull_points;
            // pcl::ConvexHull<pcl::PointXYZ> cHull;
            
            sensor_msgs::PointCloud2 output;

            pcl::fromROSMsg(input, cloud);

            std::vector<quickhull::Vector3<float>> point_cloud;
            for(auto& point : cloud)
            {
                point_cloud.emplace_back(quickhull::Vector3<float>(point.x, point.y, point.z));
            }
            auto hull = qh.getConvexHull(point_cloud, true, false);
            const auto& vertexBuffer = hull.getVertexBuffer();
            for(auto& vertex : vertexBuffer)
            {
                cHull_points.emplace_back(pcl::PointXYZ(vertex.x, vertex.y, vertex.z));
            }

            // cHull.setInputCloud(cloud.makeShared());
            // cHull.reconstruct(cHull_points);

            // pcl::VoxelGrid<pcl::PointXYZ> vox_obj;
            // vox_obj.setInputCloud (cloud.makeShared());

            // vox_obj.setLeafSize (0.1f, 0.1f, 0.1f);

            // vox_obj.filter(cloud_filtered);

            pcl::toROSMsg(cHull_points, output);
            output.header.frame_id = "point_cloud";

            pcl_pub.publish(output);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qh_compute_node");
    ROS_INFO("Start computing convex hull");
    CloudHandler handler;

    ros::spin();

    return 0;
}