#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/shapes.h>
namespace Gjk
{
    void HelloWorld()
    {
        ROS_INFO("Hello World");
    }
} // namespace Gjk
int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visual", 10);

    visualization_msgs::Marker point;
    point.header.frame_id = "dummy_link";
    point.ns = "point";
    point.action = visualization_msgs::Marker::ADD;
    point.pose.orientation.w = 1.0;
    point.id = 0;
    point.type = visualization_msgs::Marker::POINTS;
    point.scale.x = 0.01;
    point.scale.y = 0.01;
    point.color.g = 1.0f;
    point.color.a = 1.0;
    geometry_msgs::Point p;
    ros::Rate r(30);
    
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

    const std::vector<const moveit::core::LinkModel*>& links = robot_model->getLinkModelsWithCollisionGeometry();

    for(auto link : links)
    {
        ROS_INFO("Link: %s", link->getName().c_str());
        // const EigenSTL::vector_Isometry3d& cylinder_tf = link->getCollisionOriginTransforms();
        // const Eigen::Isometry3d& join_origin = link->getJointOriginTransform();
        ROS_INFO_STREAM("Global Translation: \n" << robot_state->getGlobalLinkTransform(link->getName()).translation() << "\n");
        // ROS_INFO_STREAM("First Body Translation: \n" << robot_state->getCollisionBodyTransform(link->getName(), 0).translation() << "\n");
        // robot_state->getGlobalLinkTransform(link->getName());
        // robot_state->getCollisionBodyTransform(link->getName(), 0);
        // ROS_INFO_STREAM("Joint Translation: \n" << join_origin.translation() << "\n");
        for (std::size_t j = 0; j < link->getShapes().size(); ++j)
        {
            const shapes::ShapeConstPtr& shape = link->getShapes()[j];
            ROS_INFO("Shape: %s", (shape->type == 2 ? "CYLINDER" : "DONT CARE"));
            // ROS_INFO_STREAM("Translation: \n" << cylinder_tf[j].translation() << "\n");
            const Eigen::Isometry3d& geom_tf = robot_state->getCollisionBodyTransform(link->getName(), j);
            // ROS_INFO_STREAM("Body Translation " << j <<": \n" << geom_tf.translation() << "\n");
            ROS_INFO_STREAM("Transform matrix " << j <<": \n" << geom_tf.matrix() << "\n");
            // ROS_INFO_STREAM("Local Translation: \n" << cylinder_tf[j].translation() << "\n");
            Eigen::Vector3d res = geom_tf * Eigen::Vector3d(1,1,1);
            ROS_INFO_STREAM("Result vector " << j <<": \n" << res << "\n");

            switch(shape->type)
            {
                case(shapes::CYLINDER):
                {
                    ROS_INFO("Calculating for shape: %d", shape->type);
                    const shapes::Cylinder* cylinder = static_cast<const shapes::Cylinder*>(shape.get());
                    ROS_INFO("radius: %f, length: %f", cylinder->radius, cylinder->length);
                    

                    Eigen::Isometry3d trans;
                    Eigen::Matrix3d rot_mat;
                    Eigen::Vector3d p_1c(0, 0, (cylinder->length)/2.0);
                    Eigen::Vector3d p_2c(0, 0, -(cylinder->length)/2.0);
                    Eigen::Vector3d p_0b = geom_tf.translation();
                    Eigen::Vector3d p_1b, p_2b;
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
                    trans.setIdentity();
                    trans.linear() = rot_mat;
                    // trans.translation() = geom_tf.translation();
                    trans.translation() = Eigen::Vector3d(r14, r24, r34);

                    // trans.matrix().block<3,3>(0,0).setRandom();
                    // trans.matrix().block<3,1>(0,3).setRandom();
                    ROS_INFO_STREAM("Random trans " << j <<": \n" << trans.matrix() << "\n");
                    ROS_INFO_STREAM("Translation " << j <<": \n" << Eigen::Vector3d(r14, r24, r34) << "\n");
                    ROS_INFO_STREAM("Rot determinant " << j <<": \n" << trans.rotation().determinant() << "\n");
                    // ROS_INFO_STREAM("Random vector " << j <<": \n" << p_1c << "\n");
                    ROS_INFO_STREAM("Top cylinder vector " << j <<": \n" << p_1b << "\n");
                    ROS_INFO_STREAM("Bottom cylinder vector " << j <<": \n" << p_2b << "\n");

                    p.x = p_1b[0];
                    p.y = p_1b[1];
                    p.z = p_1b[2];
                    point.points.push_back(p);

                    p.x = p_2b[0];
                    p.y = p_2b[1];
                    p.z = p_2b[2];
                    point.points.push_back(p);
                    
                    Gjk::HelloWorld();
                }
                break;
                default:
                {
                    ROS_INFO("Skip shape: %d", shape->type);
                }
            }
        }
    }
    while (ros::ok())
    {
        marker_pub.publish(point);
        r.sleep();
    }

    ros::waitForShutdown();
}