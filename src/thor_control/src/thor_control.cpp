
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "thor_control");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string GROUP_ARM = "arm";
    static const std::string GROUP_GRIPPER = "hand";

    static const double GRIPPER_STATE_OPENED = 0.0;
    static const double GRIPPER_STATE_CLOSED = 0.06;

    // MoveGroupInterface init
    moveit::planning_interface::MoveGroupInterface move_group_arm(GROUP_ARM);
    moveit::planning_interface::MoveGroupInterface move_group_gripper(GROUP_GRIPPER);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    const moveit::core::JointModelGroup *joint_model_group_from_state = move_group_arm.getCurrentState()->getJointModelGroup(GROUP_ARM);

    // RViz init
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("dummy_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Text
    Eigen::Isometry3d pose_text = Eigen::Isometry3d::Identity();
    pose_text.translation().z() = 0.8;
    visual_tools.publishText(pose_text, "Thor Arm Control Demo", rvt::WHITE, rvt::XLARGE);

    ROS_INFO_NAMED("thor_control", "Available Planning Groups:");
    std::copy(move_group_arm.getJointModelGroupNames().begin(),
            move_group_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    visual_tools.trigger();
    visual_tools.prompt("Next to move arm in Grab position");

    // ----------------- move ARM to GRAB position -----------------

    move_group_arm.setNamedTarget("grab");
    move_group_arm.move();

    visual_tools.prompt("Next to visualize plan to go back Vertical");

    // ----------------- visualize plan without obstacles -----------------

    move_group_arm.setStartState(*move_group_arm.getCurrentState());
    move_group_arm.setNamedTarget("vertical");

    bool success = (move_group_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("thor_control", "Visualizing plan without obstacles: %s", success ? "SUCCESS" : "FAILED");

    visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_from_state);
    visual_tools.publishText(pose_text, "Plan without obstacles", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Next to add walls around Thor");

    // ----------------- adding objects to environment -----------------

    moveit_msgs::CollisionObject collision_object_walls;
    collision_object_walls.header.frame_id = move_group_arm.getPlanningFrame();
    collision_object_walls.id = "collision_object_walls";

    shape_msgs::SolidPrimitive primitive_wall;
    primitive_wall.type = primitive_wall.BOX;
    primitive_wall.dimensions.resize(3);
    primitive_wall.dimensions[primitive_wall.BOX_X] = 0.1;
    primitive_wall.dimensions[primitive_wall.BOX_Y] = 1.5;
    primitive_wall.dimensions[primitive_wall.BOX_Z] = 0.5;

    // RIGHT wall creation
    geometry_msgs::Pose pose_wall_right;
    pose_wall_right.orientation.w = 1.0;
    pose_wall_right.position.x = 0.25;
    pose_wall_right.position.y = 0.0;
    pose_wall_right.position.z = 0.25;
    collision_object_walls.primitives.push_back(primitive_wall);
    collision_object_walls.primitive_poses.push_back(pose_wall_right);

    // LEFT wall creation
    geometry_msgs::Pose pose_wall_left;
    pose_wall_left.orientation.w = 1.0;
    pose_wall_left.position.x = -0.25;
    pose_wall_left.position.y = 0;
    pose_wall_left.position.z = 0.25;
    collision_object_walls.primitives.push_back(primitive_wall);
    collision_object_walls.primitive_poses.push_back(pose_wall_left);

    // add walls to world
    collision_object_walls.operation = collision_object_walls.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object_walls);
    planning_scene_interface.addCollisionObjects(collision_objects);
    ROS_INFO_NAMED("thor_control", "Added walls");

    visual_tools.publishText(pose_text, "Added walls", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Next to visualize plan with obstacles");

    // ----------------- visualize plan with obstacles -----------------

    success = (move_group_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("thor_control", "Visualizing plan with obstacles: %s", success ? "SUCCESS" : "FAILED");

    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group_from_state);
    visual_tools.publishText(pose_text, "Plan with obstacles", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Next to execute the plan");

    // ----------------- execute the plan -----------------

    if (success) {
        move_group_arm.execute(my_plan);
    }

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(pose_text, "Goal reached", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Next to add ball");

    // ----------------- add a ball -----------------

    moveit_msgs::CollisionObject collision_object_ball;
    collision_object_ball.header.frame_id = move_group_arm.getPlanningFrame();
    collision_object_ball.id = "collision_object_ball";

    // ball creation
    shape_msgs::SolidPrimitive primitive_ball;
    primitive_ball.type = primitive_ball.SPHERE;
    primitive_ball.dimensions.resize(1);
    primitive_ball.dimensions[primitive_ball.SPHERE_RADIUS] = 0.02;

    geometry_msgs::Pose pose_ball;
    pose_ball.orientation.w = 1.0;
    pose_ball.position.x = 0;
    pose_ball.position.y = -0.33;
    pose_ball.position.z = 0.02;
    collision_object_ball.primitives.push_back(primitive_ball);
    collision_object_ball.primitive_poses.push_back(pose_ball);

    // add ball to world
    collision_object_ball.operation = collision_object_ball.ADD;
    collision_objects.push_back(collision_object_ball);
    planning_scene_interface.addCollisionObjects(collision_objects);
    ROS_INFO_NAMED("thor_control", "Added ball");

    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    visual_tools.prompt("Next to pickup the ball");

    // ----------------- pick up the ball -----------------

    // to check the current end_effector position/orientation of Thor in RViz
    // rosrun tf tf_echo /dummy_link /link_art6
    geometry_msgs::Pose pose_pickup_ball; 
    pose_pickup_ball.orientation.x = 0.955;
    pose_pickup_ball.orientation.y = -0.004;
    pose_pickup_ball.orientation.z = -0.001;
    pose_pickup_ball.orientation.w = 0.297;
    pose_pickup_ball.position.x = 0.001;
    pose_pickup_ball.position.y = -0.239;
    pose_pickup_ball.position.z = 0.137;

    move_group_arm.setPoseTarget(pose_pickup_ball);
    move_group_arm.setStartStateToCurrentState();
    move_group_arm.move();

    move_group_gripper.setJointValueTarget("joint_hand_finger_left", GRIPPER_STATE_CLOSED * 0.2);
    move_group_gripper.move();

    ros::shutdown();
    return 0;
}
