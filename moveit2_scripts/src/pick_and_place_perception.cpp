#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "custom_msgs/msg/detected_objects.hpp"
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickAndPlaceTrajectory {
public:
  PickAndPlaceTrajectory(rclcpp::Node::SharedPtr base_node_);

  ~PickAndPlaceTrajectory() {
    RCLCPP_INFO(LOGGER, "Class Terminated: Pick And Place Trajectory");
  }

  void execute_trajectory_plan();

private:
  // using shorthand for lengthy class references
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;

  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::MultiThreadedExecutor executor_;

  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;

  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;

  rclcpp::CallbackGroup::SharedPtr object_detection_cb_group_;
  rclcpp::Subscription<custom_msgs::msg::DetectedObjects>::SharedPtr
      object_detection_subscription_;
  bool object_detected_ = false;
  geometry_msgs::msg::Point object_position_;

  // declare trajectory planning variables for robot and gripper
  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false;

  std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;

  // declare cartesian trajectory planning variables for robot
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;

  // methods
  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5);
  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w);
  void plan_trajectory_kinematics();
  void execute_trajectory_kinematics();
  void setup_waypoints_target(float x_delta, float y_delta, float z_delta);
  void plan_trajectory_cartesian();
  void execute_trajectory_cartesian();
  void setup_joint_value_gripper(float angle);
  void setup_named_pose_gripper(std::string pose_name);
  void plan_trajectory_gripper();
  void execute_trajectory_gripper();
  void object_detection_callback(
      const custom_msgs::msg::DetectedObjects::SharedPtr msg);
}; // class PickAndPlaceTrajectory

void PickAndPlaceTrajectory::setup_joint_value_target(
    float angle0, float angle1, float angle2, float angle3, float angle4,
    float angle5) {
  // set the joint values for each joint of robot arm
  joint_group_positions_robot_[0] = angle0; // Shoulder Pan
  joint_group_positions_robot_[1] = angle1; // Shoulder Lift
  joint_group_positions_robot_[2] = angle2; // Elbow
  joint_group_positions_robot_[3] = angle3; // Wrist 1
  joint_group_positions_robot_[4] = angle4; // Wrist 2
  joint_group_positions_robot_[5] = angle5; // Wrist 3
  move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
}

void PickAndPlaceTrajectory::setup_goal_pose_target(float pos_x, float pos_y,
                                                    float pos_z, float quat_x,
                                                    float quat_y, float quat_z,
                                                    float quat_w) {
  // set the pose values for end effector of robot arm
  target_pose_robot_.position.x = pos_x;
  target_pose_robot_.position.y = pos_y;
  target_pose_robot_.position.z = pos_z;
  target_pose_robot_.orientation.x = quat_x;
  target_pose_robot_.orientation.y = quat_y;
  target_pose_robot_.orientation.z = quat_z;
  target_pose_robot_.orientation.w = quat_w;
  move_group_robot_->setPoseTarget(target_pose_robot_);
}

void PickAndPlaceTrajectory::plan_trajectory_kinematics() {
  // plan the trajectory to target using kinematics
  plan_success_robot_ = (move_group_robot_->plan(kinematics_trajectory_plan_) ==
                         moveit::core::MoveItErrorCode::SUCCESS);
}

void PickAndPlaceTrajectory::execute_trajectory_kinematics() {
  // execute the planned trajectory to target using kinematics
  if (plan_success_robot_) {
    move_group_robot_->execute(kinematics_trajectory_plan_);
  } else {
    RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
  }
}

void PickAndPlaceTrajectory::setup_waypoints_target(float x_delta,
                                                    float y_delta,
                                                    float z_delta) {
  target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
  target_pose_robot_.position.x += x_delta;
  target_pose_robot_.position.y += y_delta;
  target_pose_robot_.position.z += z_delta;
  target_pose_robot_.orientation.x = -1.0;
  target_pose_robot_.orientation.y = 0.0;
  target_pose_robot_.orientation.z = 0.0;
  target_pose_robot_.orientation.w = 0.0;
  cartesian_waypoints_.push_back(target_pose_robot_);
}

void PickAndPlaceTrajectory::plan_trajectory_cartesian() {
  // plan the trajectory to target using cartesian path
  plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
      cartesian_waypoints_, end_effector_step_, jump_threshold_,
      cartesian_trajectory_plan_);
}

void PickAndPlaceTrajectory::execute_trajectory_cartesian() {
  // execute the planned trajectory to target using cartesian path
  if (plan_fraction_robot_ >= 0.0) {
    // 0.0 to 1.0 = success and -1.0 = failure
    move_group_robot_->execute(cartesian_trajectory_plan_);
  } else {
    RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
  }
  // clear cartesian waypoints vector
  cartesian_waypoints_.clear();
}

void PickAndPlaceTrajectory::setup_joint_value_gripper(float angle) {
  // set the joint values for each joint of gripper
  // based on values provided
  joint_group_positions_gripper_[2] = angle;
  move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
}

void PickAndPlaceTrajectory::setup_named_pose_gripper(std::string pose_name) {
  // set the joint values for each joint of gripper
  // based on predefined pose names
  move_group_gripper_->setNamedTarget(pose_name);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void PickAndPlaceTrajectory::plan_trajectory_gripper() {
  // plan the gripper action
  plan_success_gripper_ =
      (move_group_gripper_->plan(gripper_trajectory_plan_) ==
       moveit::core::MoveItErrorCode::SUCCESS);
}

void PickAndPlaceTrajectory::execute_trajectory_gripper() {
  // execute the planned gripper action
  if (plan_success_gripper_) {
    move_group_gripper_->execute(gripper_trajectory_plan_);
  } else {
    RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
  }
  // Tested: Don't change
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}

PickAndPlaceTrajectory::PickAndPlaceTrajectory(
    rclcpp::Node::SharedPtr base_node_)
    : base_node_(base_node_) {
  //   RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place Trajectory...");
  object_detection_cb_group_ = base_node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  auto object_detection_sub_options = rclcpp::SubscriptionOptions();
  object_detection_sub_options.callback_group = object_detection_cb_group_;
  // Subscribe to the /object_detected topic
  object_detection_subscription_ =
      base_node_->create_subscription<custom_msgs::msg::DetectedObjects>(
          "/object_detected", 10,
          std::bind(&PickAndPlaceTrajectory::object_detection_callback, this,
                    std::placeholders::_1),
          object_detection_sub_options);

  // configure node options
  rclcpp::NodeOptions node_options;
  // auto-declare node_options parameters from overrides
  node_options.automatically_declare_parameters_from_overrides(true);

  // initialize move_group node
  move_group_node_ = rclcpp::Node::make_shared("move_group_node", node_options);
  // start move_group node in a new executor thread and spin it
  executor_.add_node(move_group_node_);
  executor_.add_node(base_node_);
  std::thread([this]() { this->executor_.spin(); }).detach();

  // initialize move_group interfaces
  move_group_robot_ = std::make_shared<MoveGroupInterface>(
      move_group_node_, PLANNING_GROUP_ROBOT);
  move_group_gripper_ = std::make_shared<MoveGroupInterface>(
      move_group_node_, PLANNING_GROUP_GRIPPER);

  // get initial state of robot and gripper
  joint_model_group_robot_ =
      move_group_robot_->getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_ROBOT);
  joint_model_group_gripper_ =
      move_group_gripper_->getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  // print out basic system information
  RCLCPP_INFO(LOGGER, "Planning Frame: %s",
              move_group_robot_->getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End Effector Link: %s",
              move_group_robot_->getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::vector<std::string> group_names =
      move_group_robot_->getJointModelGroupNames();
  // more efficient method than std::copy() method used in the docs
  for (long unsigned int i = 0; i < group_names.size(); i++) {
    RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
  }

  // get current state of robot and gripper
  current_state_robot_ = move_group_robot_->getCurrentState(10);
  current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                joint_group_positions_robot_);
  RCLCPP_DEBUG(LOGGER, "HOME: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
               joint_group_positions_robot_[0], joint_group_positions_robot_[1],
               joint_group_positions_robot_[2], joint_group_positions_robot_[3],
               joint_group_positions_robot_[4],
               joint_group_positions_robot_[5]);
  current_state_gripper_ = move_group_gripper_->getCurrentState(10);
  current_state_gripper_->copyJointGroupPositions(
      joint_model_group_gripper_, joint_group_positions_gripper_);

  // set start state of robot and gripper to current state
  move_group_robot_->setStartStateToCurrentState();
  move_group_gripper_->setStartStateToCurrentState();

  // indicate initialization
  RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place Trajectory");
  RCLCPP_INFO(LOGGER, "------------------------------------------");
}

void PickAndPlaceTrajectory::execute_trajectory_plan() {
  float offset = 0.07;
  RCLCPP_INFO(LOGGER, "Planning and Executing Pick And Place Trajectory...");

  RCLCPP_INFO(LOGGER, "Going to Home Position...");
  setup_joint_value_target(0.0, -1.57, 0.0, -1.57, 0.0, 0.0);
  plan_trajectory_kinematics();
  execute_trajectory_kinematics();

  // Wait till object is detected
  // Exit after 3 times
  int attempt = 1;
  while (!object_detected_ && attempt <= 3) {
    attempt += 1;
    rclcpp::sleep_for(std::chrono::seconds(10));
  }
  if (!object_detected_) {
    RCLCPP_ERROR(LOGGER, "Object detection failed. Exiting...");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(LOGGER, "Going to Pregrasp Position...");
  setup_goal_pose_target(object_position_.x, object_position_.y, 0.24, -1.0,
                         0.0, 0.0, 0.0);
  plan_trajectory_kinematics();
  execute_trajectory_kinematics();

  RCLCPP_INFO(LOGGER, "Opening Gripper...");
  setup_named_pose_gripper("gripper_open");
  plan_trajectory_gripper();
  execute_trajectory_gripper();

  RCLCPP_INFO(LOGGER, "Approaching...");
  setup_waypoints_target(0.0, 0.0, -offset);
  plan_trajectory_cartesian();
  execute_trajectory_cartesian();

  RCLCPP_INFO(LOGGER, "Closing Gripper...");
  setup_joint_value_gripper(0.645);
  plan_trajectory_gripper();
  execute_trajectory_gripper();

  RCLCPP_INFO(LOGGER, "Retreating...");
  setup_waypoints_target(0.0, 0.0, +offset);
  plan_trajectory_cartesian();
  execute_trajectory_cartesian();

  RCLCPP_INFO(LOGGER, "Rotating 180degrees to Release Position...");
  current_state_robot_ = move_group_robot_->getCurrentState(10);
  current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                joint_group_positions_robot_);
  setup_joint_value_target(
      joint_group_positions_robot_[0] + 3.1415, joint_group_positions_robot_[1],
      joint_group_positions_robot_[2], joint_group_positions_robot_[3],
      joint_group_positions_robot_[4], joint_group_positions_robot_[5]);
  plan_trajectory_kinematics();
  execute_trajectory_kinematics();

  RCLCPP_INFO(LOGGER, "Opening Gripper...");
  setup_named_pose_gripper("gripper_open");
  plan_trajectory_gripper();
  execute_trajectory_gripper();

  RCLCPP_INFO(LOGGER, "Returning to Home Position...");
  setup_joint_value_target(0.0, -1.57, 0.0, -1.57, 0.0, 0.0);
  plan_trajectory_kinematics();
  execute_trajectory_kinematics();

  RCLCPP_INFO(LOGGER, "Pick And Place Trajectory Execution Complete");
}

// Callback function to handle incoming messages
void PickAndPlaceTrajectory::object_detection_callback(
    const custom_msgs::msg::DetectedObjects::SharedPtr msg) {
  // correct pose error
  object_position_.x = round(msg->position.x * 100.0) / 100.0 + 0.01;
  object_position_.y = round(msg->position.y * 100.0) / 100.0 - 0.01;
  RCLCPP_DEBUG(LOGGER, "Object detected at: [X: %f, Y: %f, Z: %f]",
               object_position_.x, object_position_.y, msg->position.z);
  object_detected_ = true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place_perception");
  PickAndPlaceTrajectory pick_and_place_trajectory_node(base_node);
  pick_and_place_trajectory_node.execute_trajectory_plan();
  rclcpp::shutdown();

  return 0;
}

// End of Code