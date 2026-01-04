#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm1_ = std::make_shared<MoveGroupInterface>(node_, "arm1");
        arm1_ ->setMaxVelocityScalingFactor(1.0);
        arm1_ ->setMaxAccelerationScalingFactor(1.0);

        arm2_ = std::make_shared<MoveGroupInterface>(node_, "arm2");
        arm2_ ->setMaxVelocityScalingFactor(1.0);
        arm2_ ->setMaxAccelerationScalingFactor(1.0);

        agvBase_ = std::make_shared<MoveGroupInterface>(node_, "agvBase");
        agvBase_ ->setMaxVelocityScalingFactor(1.0);
        agvBase_ ->setMaxAccelerationScalingFactor(1.0);
    }

    // ------------------------------------------------------------
    void goToNamedTarget(const std::string &name)
    {
        arm1_ ->setStartStateToCurrentState();
        arm1_ ->setNamedTarget(name);
        planAndExecute(arm1_);
    }

    void goToJointTarget(const std::vector<double> &joints)
    {
        arm1_ ->setStartStateToCurrentState();
        arm1_ ->setJointValueTarget(joints);
        planAndExecute(arm1_);
    }

    void goToPoseTarget(double x, double y, double z, 
                        double roll, double pitch, double yaw, 
                        bool catesian_path=false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";

        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.w = q.getX();
        target_pose.pose.orientation.x = q.getY();
        target_pose.pose.orientation.y = q.getZ();
        target_pose.pose.orientation.z = q.getW();       

        arm1_ ->setStartStateToCurrentState();
 
        if(!catesian_path){
            arm1_ ->setPoseTarget(target_pose);
            planAndExecute(arm1_);
        }
        else{
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = arm1_->computeCartesianPath(waypoints, 0.001, trajectory);

            if(fraction == 1){
                arm1_ ->execute(trajectory);
            }
        }
    }

private:

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success){
            interface->execute(plan);
        }

    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm1_;
    std::shared_ptr<MoveGroupInterface> arm2_;
    std::shared_ptr<MoveGroupInterface> agvBase_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");

    auto commander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

