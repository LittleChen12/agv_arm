#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("Agv_Moveit_Controllers");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    auto agv_arm1 = moveit::planning_interface::MoveGroupInterface(node, "arm1");
    agv_arm1.setMaxVelocityScalingFactor(1.0);
    agv_arm1.setMaxAccelerationScalingFactor(1.0);


    // named goal
    std::cout << "Start named goal" << std::endl;
    agv_arm1.setStartStateToCurrentState();
    agv_arm1.setNamedTarget("arm1_pos1");

    moveit::planning_interface::MoveGroupInterface::Plan arm1_plan_named;
    bool arm1_success = (agv_arm1.plan(arm1_plan_named) == moveit::core::MoveItErrorCode::SUCCESS);

    if(arm1_success){
        agv_arm1.execute(arm1_plan_named);
    }
    // -----------------------------------------------------
    sleep(5);
    std::cout << "Start joint goal" << std::endl;
    // joint goal

    std::vector<double> joints = { 0.0, 0.5235, -1.57, 0.0, -1.0471, 0.0};

    agv_arm1.setStartStateToCurrentState();
    agv_arm1.setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan arm1_plan_joint;
    arm1_success = (agv_arm1.plan(arm1_plan_joint) == moveit::core::MoveItErrorCode::SUCCESS);

    if(arm1_success){
        agv_arm1.execute(arm1_plan_joint);
    }
    // ------------------------------------------------------
    sleep(5);
    std::cout << "Start pose goal" << std::endl;
    // Pose Goal

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.47772833704948425;
    target_pose.pose.position.y = 0.31734731793403625;
    target_pose.pose.position.z = 0.26556093096733093;

    target_pose.pose.orientation.w = -0.06570280343294144;
    target_pose.pose.orientation.x = -0.4206175208091736;
    target_pose.pose.orientation.y = 0.9042646884918213;
    target_pose.pose.orientation.z = 0.03270197659730911;

    agv_arm1.setStartStateToCurrentState();
    agv_arm1.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan arm1_plan_pose;
    arm1_success = (agv_arm1.plan(arm1_plan_pose) == moveit::core::MoveItErrorCode::SUCCESS);

    if(arm1_success){
        agv_arm1.execute(arm1_plan_pose);
    }

    // -----------------------------------------------------------------
    sleep(5);
    std::cout << "Start Cartesian Path" << std::endl;
    // Cartesian Path
    agv_arm1.setMaxVelocityScalingFactor(0.1);
    agv_arm1.setMaxAccelerationScalingFactor(0.5);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose pose1 = agv_arm1.getCurrentPose().pose;
    pose1.position.x += 0.2;
    waypoints.push_back(pose1);

    geometry_msgs::msg::Pose pose2 = pose1;
    pose2.position.y += 0.3;
    waypoints.push_back(pose2);

    geometry_msgs::msg::Pose pose3 = pose2;
    pose3.position.x -= 0.2;
    waypoints.push_back(pose3);
    
    geometry_msgs::msg::Pose pose4 = pose3;
    pose4.position.y -= 0.3;
    waypoints.push_back(pose4);

    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = agv_arm1.computeCartesianPath(waypoints, 0.001, trajectory);

    if(fraction == 1){
        agv_arm1.execute(trajectory);
    }


    rclcpp::shutdown();
    spinner.join();
    return 0;

}