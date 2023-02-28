#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/convex_hull.h>
#include "my_quickhull/QuickHull.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class CloudHandler
{
    protected:
        ros::NodeHandle nh;
        ros::Subscriber pcl_sub;
        ros::Publisher pcl_pub;
        quickhull::QuickHull<float> qh;
        visualization_msgs::Marker point;
        geometry_msgs::Point p;

        Eigen::Vector3d center_point;
        std::vector<Eigen::Vector3d> convex_points;
        pcl::PointCloud<pcl::PointXYZ> cloud;

    public:
        CloudHandler()
        {
            pcl_sub = nh.subscribe("my_convex_hull_points", 10, &CloudHandler::cloudCB, this);
            pcl_pub = nh.advertise<visualization_msgs::Marker>("center", 10);
            
            point.header.frame_id = "point_cloud";
            point.ns = "point";
            point.action = visualization_msgs::Marker::ADD;
            point.pose.orientation.w = 1.0;
            point.id = 0;
            point.type = visualization_msgs::Marker::POINTS;
            point.scale.x = 0.01;
            point.scale.y = 0.01;
            point.color.g = 1.0f;
            point.color.a = 1.0;
        }
        void cloudCB(const sensor_msgs::PointCloud2& input)
        {
            pcl::fromROSMsg(input, cloud);
            double x, y, z;
            x = y = z = 0;
            for(auto& point : cloud)
            {
                x += point.x;
                y += point.y;
                z += point.z;
                convex_points.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
            }
            center_point.x() = p.x = x/cloud.size();
            center_point.y() = p.y = y/cloud.size();
            center_point.z() = p.z = z/cloud.size();

            point.points.clear();
            point.points.push_back(p);
            pcl_pub.publish(point);
        }
        const Eigen::Vector3d& getCenterPoint()
        {
            return center_point;
        }
        const std::vector<Eigen::Vector3d>& getConvexPoints()
        {
            return convex_points;
        }
};

namespace Gjk
{
    class GjkObject
    {
        private:
            Eigen::Isometry3d T_bc;
            Eigen::Vector3d p_0b;
            shapes::ShapeConstPtr shape;
        public:
        GjkObject(){};
        GjkObject(const shapes::ShapeConstPtr& shape, const Eigen::Isometry3d& geom_tf):
                  shape(shape)
        {
            const shapes::Cylinder* cylinder = static_cast<const shapes::Cylinder*>(shape.get());

            Eigen::Matrix3d rot_mat;
            Eigen::Vector3d p_1c(0, 0, (cylinder->length)/2.0);
            Eigen::Vector3d p_2c(0, 0, -(cylinder->length)/2.0);
            Eigen::Vector3d p_1b, p_2b;
            p_0b = geom_tf.translation();
            p_1b = geom_tf * p_1c;
            p_2b = geom_tf * p_2c;

            double beta = atan2(-(p_1b.x()-p_0b.x()), p_1b.z()-p_0b.z());
            double a1 = -(p_1b.y()-p_0b.y());
            double a2 = sin(beta)*(p_1b.x()-p_0b.x()) - cos(beta)*(p_1b.z()-p_0b.z());
            double alpha = atan2(a1, a2);

            double r11 = cos(beta);
            double r12 = 0;
            double r13 = sin(beta);
            double r14 = -cos(beta)*p_0b.x() - sin(beta)*p_0b.z();

            double r21 = sin(alpha)*sin(beta);
            double r22 =  cos(alpha);
            double r23 = -sin(alpha)*cos(beta);
            double r24 = -sin(alpha)*sin(beta)*p_0b.x() - cos(alpha)*p_0b.y() + sin(alpha)*cos(beta)*p_0b.z();

            double r31 = -cos(alpha)*sin(beta);
            double r32 = sin(alpha);
            double r33 = cos(alpha)*cos(beta);
            double r34 = cos(alpha)*sin(beta)*p_0b.x() - sin(alpha)*p_0b.y() - cos(alpha)*cos(beta)*p_0b.z();


            rot_mat << r11, r12, r13,
                       r21, r22, r23,
                       r31, r32, r33;
            T_bc.setIdentity();
            T_bc.linear() = rot_mat;
            T_bc.translation() = Eigen::Vector3d(r14, r24, r34);
        }
        const Eigen::Isometry3d getT_bc()
        {
            return this->T_bc;
        }
        const Eigen::Vector3d getCenterPoint()
        {
            return this->p_0b;
        }
        Eigen::Vector3d supportFunction(const Eigen::Vector3d& d_b, const Eigen::Isometry3d& geom_tf)
        {
            const shapes::Cylinder* cylinder = static_cast<const shapes::Cylinder*>(shape.get());
            ROS_INFO("radius: %f, length: %f", cylinder->radius, cylinder->length);
            
            Eigen::Vector3d d_c = T_bc * d_b;

            double mu = sqrt((d_c.x()*d_c.x() + d_c.y()*d_c.y()));
            
            double x_c = cylinder->radius * d_c.x() / mu;
            double y_c = cylinder->radius * d_c.y() / mu;
            double z_c = sgn(d_c.z())*cylinder->length/2;

            Eigen::Vector3d s_c(x_c, y_c, z_c);
            Eigen::Vector3d s_b = geom_tf * s_c;
            return s_b;
        }
    };
    Eigen::Vector3d supportFunction(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& dir)
    {
        Eigen::Vector3d maxPoint;
        double maxDistance = -DBL_MAX;
        for(auto& point : points)
        {
            double distance = point.dot(dir);
            if(distance > maxDistance)
            {
                maxDistance = distance;
                maxPoint = point;
            }
        }
        return maxPoint;
    }
    struct Simplex{
        typedef Eigen::Vector3d Vector3d;
        private:
            std::array<Vector3d, 4> m_points;
            unsigned m_size;
        public:
            Simplex() : m_size(0)
            {}

            Simplex& operator=(std::initializer_list<Vector3d> list)
            {
                for(auto v = list.begin(); v != list.end(); v++)
                {
                    m_points[std::distance(list.begin(), v)] = *v;
                }
                m_size = list.size();
                return *this;
            }

            void push_front(Vector3d point)
            {
                m_points = {point, m_points[0], m_points[1], m_points[2]};
                m_size = std::min(m_size + 1, 4U);
            }

            Vector3d& operator[](unsigned i) { return m_points[i]; }
            unsigned size() const { return m_size; }
            auto begin() const { return m_points.begin(); }
            auto end() const {return m_points.end() - (4 - m_size); }
    };
    bool isSameDirection(const Eigen::Vector3d& direction, const Eigen::Vector3d& ao)
    {
        return direction.dot(ao) > 0;
    };
    bool Line(Simplex& points, Eigen::Vector3d direction)
    {
        Eigen::Vector3d ab = points[1] - points[0];
        Eigen::Vector3d ao = -points[0];
        if(isSameDirection(ab, ao))
        {
            direction = ab.cross(ao).cross(ab);
        }
        else
        {
            points = {points[0]};
            direction = ao;
        }
    };
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "compute_center_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(1);
    ROS_INFO("Start computing center of convex hull");
    CloudHandler cloudHandler;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));

    planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    /* Listen for planning scene messages on topic /XXX and apply them to the internal planning scene accordingly */
    psm->startSceneMonitor();
    /* Listens to changes of world geometry, collision objects, and (optionally) octomaps */
    psm->startWorldGeometryMonitor();
    /* Listen to joint state updates as well as changes in attached collision objects
                      and update the internal planning scene accordingly*/
    psm->startStateMonitor();
    /* Get robot model*/
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader->getModel();
    while (ros::ok())
    {
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
        const std::vector<const moveit::core::LinkModel*>& links = robot_model->getLinkModelsWithCollisionGeometry();
        Eigen::Vector3d center_point = cloudHandler.getCenterPoint();
        std::vector<Eigen::Vector3d> convex_points = cloudHandler.getConvexPoints();
        ROS_INFO_STREAM("Center point: " << center_point << "\n");
        for(auto link : links)
        {
            ROS_INFO("Link: %s", link->getName().c_str());
            ROS_INFO_STREAM("Global Translation: \n" << robot_state->getGlobalLinkTransform(link->getName()).translation() << "\n");
            for (std::size_t j = 0; j < link->getShapes().size(); ++j)
            {
                const shapes::ShapeConstPtr& shape = link->getShapes()[j];
                const Eigen::Isometry3d& geom_tf = robot_state->getCollisionBodyTransform(link->getName(), j);
                switch(shape->type)
                {
                    case(shapes::CYLINDER):
                    {
                        Gjk::GjkObject gjkObj(shape, geom_tf);
                        //ROS_INFO_STREAM("Transform matrix " << j <<": \n" << gjkObj.getT_bc().matrix() << "\n");

                        Eigen::Vector3d d_0 = cloudHandler.getCenterPoint() - gjkObj.getCenterPoint();
                        //ROS_INFO_STREAM("Direction: " << d_b << "\n");
                        Eigen::Vector3d s_11 = gjkObj.supportFunction(d_0, geom_tf);
                        //ROS_INFO_STREAM("Support point: " << s_1 << "\n");
                        Eigen::Vector3d s_12 = Gjk::supportFunction(convex_points, d_0);
                        Eigen::Vector3d s_21 = gjkObj.supportFunction(-d_0, geom_tf);
                        Eigen::Vector3d s_22 = Gjk::supportFunction(convex_points, -d_0);
                        Eigen::Vector3d s1 = s_11 - s_21;
                        Eigen::Vector3d s2 = s_12 - s_22;
                        Eigen::Vector3d d_1 = (s2 - s1).cross(-s1).cross(s2-s1);
                        Eigen::Vector3d s_13 = gjkObj.supportFunction(d_1, geom_tf);
                        Eigen::Vector3d s_23 = Gjk::supportFunction(convex_points, d_1);
                        Eigen::Vector3d s3 = s_13 - s_23;

                    }
                    break;
                    default:
                    {
                        ROS_INFO("Skip");
                    }
                }
            }
        }
        r.sleep();
    }

    ros::waitForShutdown();
}