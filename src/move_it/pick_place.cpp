/* Author: Hyoseok Hwang & Jungwoon Lee @ AIRLAB, KHU*/

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <iostream>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <control_msgs/GripperCommandActionGoal.h>


void move_to_pose(moveit::planning_interface::MoveGroupInterface &move_group, float x, float y, float z, float roll, float pitch, float yaw) 
{
  // Create pose
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  tf2::Quaternion orientation;
  orientation.setRPY(roll, pitch, yaw);
  pose.orientation = tf2::toMsg(orientation);

  // Compute and execute trajectory
  std::vector<geometry_msgs::Pose> waypoints = {pose};
  moveit_msgs::RobotTrajectory trajectory;
  ROS_DEBUG("Trying to compute cartesian path");
  double fraction_completed = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  if(fraction_completed == 1) {

    ROS_DEBUG("%f of path computed", fraction_completed);

  }
  ROS_DEBUG("Trying to exectue cartesian path");
  move_group.execute(trajectory);

}

void go_to_pose_goal(moveit::planning_interface::MoveGroupInterface &move_group_interface,
geometry_msgs::Pose &target_pose) {
  // .. _move_group_interface-planning-to-pose-goal:

  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;


  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
  // Note that this can lead to problems if the robot moved in the meanwhile.
  move_group_interface.execute(my_plan);
  

}

geometry_msgs::Pose list_to_pose(double x,double y,double z,double roll,double pitch,double yaw)
{

  geometry_msgs::Pose target_pose;
  tf2::Quaternion orientation;
  orientation.setRPY(roll,pitch, yaw);
  target_pose.orientation= tf2::toMsg(orientation);
  target_pose.position.x=x;
  target_pose.position.y=y;
  target_pose.position.z=z;

  return target_pose;
}

void move_initial(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  move_to_pose(move_group_interface, 0.5, 0, 0.5, M_PI/2, 0, M_PI/2);
}

void prePick(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start = move_group_interface.getCurrentPose().pose;

  start.position.x+=0.15;
  waypoints.push_back(start);

  // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  move_group_interface.execute(trajectory);
}


void postPick(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start = move_group_interface.getCurrentPose().pose;

  start.position.z+=0.2;
  waypoints.push_back(start);

  // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  move_group_interface.execute(trajectory);
}

void prePlace(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  move_to_pose(move_group_interface, 0, 0.6, 0.75, M_PI/2, 0, M_PI);
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start = move_group_interface.getCurrentPose().pose;

  start.position.z-=0.15;
  waypoints.push_back(start);

  // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  move_group_interface.execute(trajectory);
}

void postPlace(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start = move_group_interface.getCurrentPose().pose;

  start.position.y-=0.2;
  waypoints.push_back(start);

  // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  move_group_interface.execute(trajectory);
}

void genSpiral(const std::vector<float>& params, std::vector<float>& xX, std::vector<float>& yY, std::vector<float>& zZ)
{
  // params: [radius_start, radius_end, height, num_points, turns]
  float radius_start = params[0];
  float radius_end = params[1];
  float height = params[2];
  int num_points = static_cast<int>(params[3]);
  float turns = params[4];

  for (int i = 0; i < num_points; i++)
  {
    float t = static_cast<float>(i) / static_cast<float>(num_points);
    float angle = t * turns * 2 * M_PI;
    float radius = radius_start * (1.0f - t) + radius_end * t;
    float z = height * t;

    float x = radius * cos(angle);
    float y = radius * sin(angle);

    xX.push_back(x);
    yY.push_back(y);
    zZ.push_back(-z);
  }
}

float getAngle(std::array<float,2> start, std::array<float,2> center, std::array<float,2> curr)
{
  float x0 = start[0] - center[0];
  float y0 = start[1] - center[1];

  float x = curr[0] - center[0];
  float y = curr[1] - center[1];

  float ct = (x0*x + y0*y) / (sqrt(pow(x0, 2) + pow(y0, 2))*sqrt(pow(x, 2) + pow(y, 2)));
  float st = sqrt(1 - pow(ct, 2));
  float theta = atan2(st, ct);
  
  return theta;
}

void spiralTrajectory(moveit::planning_interface::MoveGroupInterface& group)
{
  geometry_msgs::Pose current_pose = group.getCurrentPose().pose;

  // Get the starting point from the current pose
  float start_x = current_pose.position.x;
  float start_y = current_pose.position.y;
  float start_z = current_pose.position.z;

  // Get the waypoints of conical spiral
  std::vector<float> params = {0.01, 0.001, 0.05, 100, 3.0};
  std::vector<float> xX, yY, zZ;
  genSpiral(params, xX, yY, zZ);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<geometry_msgs::Pose> waypoints;

  float end_x = start_x + xX.back();
  float end_y = start_y + yY.back();
  float end_z = start_z + zZ.back();

  for (int i = 0; i < xX.size(); i++)
  {
    // Planning for the next pose
    geometry_msgs::Pose next_pose = current_pose;
    next_pose.position.x = start_x + xX[i];
    next_pose.position.y = start_y + yY[i];
    next_pose.position.z = start_z + zZ[i];
    
    // Add orientation & setRPY
    // Compute direction vector
    float dx = end_x - next_pose.position.x;
    float dy = end_y - next_pose.position.y;
    float dz = end_z - next_pose.position.z;

    // Compute Roll, Pitch, Yaw
    float roll = M_PI / 2 - atan2(dy, dz);
    float pitch = atan2(dx, dz);
    float yaw = atan2(next_pose.position.x, next_pose.position.y);
    // float roll = -M_PI / 2 - atan2(next_pose.position.x, next_pose.position.y);
    // float pitch = -M_PI / 4 - atan2(dx, dz);
    // float yaw = -M_PI / 2 - atan2(dy, dz);
    
    // M_PI / 2
    

    // Set orientation
    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);
    next_pose.orientation = tf2::toMsg(orientation);

    waypoints.push_back(next_pose);
  }
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  group.execute(trajectory);
}

void partialSFT(moveit::planning_interface::MoveGroupInterface& group, float tlim)
{
  geometry_msgs::Pose current_pose = group.getCurrentPose().pose;

  // Get the starting point from the current pose
  float start_x = current_pose.position.x;
  float start_y = current_pose.position.y;
  float start_z = current_pose.position.z;

  // Get the waypoints of conical spiral
  std::vector<float> params = {0.05, 0.01, 0.1, 100, 3.0};
  std::vector<float> xX, yY, zZ;
  genSpiral(params, xX, yY, zZ);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<geometry_msgs::Pose> waypoints;

  float end_x = start_x + xX.back();
  float end_y = start_y + yY.back();
  float end_z = start_z + zZ.back();

  std::array<float, 2> start = {start_x, start_y};
  std::array<float, 2> center = {end_x, end_y};

  for (int i = 0; i < xX.size(); i++)
  {
    // Planning for the next pose
    geometry_msgs::Pose next_pose = current_pose;
    std::array<float, 2> curr = {xX[i], yY[i]};
    float theta = getAngle(start, center, curr);
    if (theta < tlim)
    {

      next_pose.position.x = start_x + xX[i];
      next_pose.position.y = start_y + yY[i];
      next_pose.position.z = start_z + zZ[i];

      waypoints.push_back(next_pose);
    }
  

  }
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  group.execute(trajectory);
}

void openGripper(ros::Publisher &gripper_pub)
{
  // Create a message of type control_msgs/GripperCommandActionGoal
  control_msgs::GripperCommandActionGoal msg;

  // Set the command parameters
  msg.goal.command.position = 0.05; // Target gripper position
  msg.goal.command.max_effort = 0; // Maximum force allowed

  // Publish the message
  gripper_pub.publish(msg);

  ROS_INFO("Gripper opened");
}

void closeGripper(ros::Publisher &gripper_pub)
{
  // Create a message of type control_msgs/GripperCommandActionGoal
  control_msgs::GripperCommandActionGoal msg;

  // Set the command parameters
  msg.goal.command.position = 0.8; // Target gripper position
  msg.goal.command.max_effort = 5; // Maximum force allowed

  // Publish the message
  gripper_pub.publish(msg);

  ROS_INFO("Gripper closed");
}


void approach(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start = move_group_interface.getCurrentPose().pose;

  start.position.y+=0.115;
  waypoints.push_back(start);

  // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  move_group_interface.execute(trajectory);

}

void gripper_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,double value)
{
  ROS_INFO("gripper sample"); 
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;

  joint_group_positions=move_group_interface.getCurrentJointValues();
  joint_group_positions[0]=0;
  joint_group_positions[1]=0;
  joint_group_positions[2]=value;
  joint_group_positions[3]=0;
  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  move_group_interface.setJointValueTarget(joint_group_positions);
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_interface.execute(my_plan);
}

void rotate(moveit::planning_interface::MoveGroupInterface &move_group_interface, float r, float p, float y)
{
  ROS_INFO("rotation");
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start = move_group_interface.getCurrentPose().pose;


  tf2::Quaternion orientation;
  orientation.setRPY(r,p,y);
  start.orientation= tf2::toMsg(orientation);

  waypoints.push_back(start);

  // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  move_group_interface.execute(trajectory);
}

void post_grasp(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start = move_group_interface.getCurrentPose().pose;

  start.position.z+=0.25;
  waypoints.push_back(start);

  // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  move_group_interface.execute(trajectory);
}

void draw_circle(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  ROS_INFO("Draw Circle");
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start = move_group_interface.getCurrentPose().pose;
  
  double radius = 0.1;
  int points_num = 40;
  
  start.position.x -= radius;
  geometry_msgs::Pose center = start;

  for (int i = 0; i < points_num; i++) {
    double angle = 2 * M_PI * i / points_num;
    //let start point as the center of the circle
    geometry_msgs::Pose point = center;

    point.position.x += radius * cos(angle);
    point.position.y += radius * sin(angle);
    waypoints.push_back(point);
  }

  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  move_group_interface.execute(trajectory);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm("arm");
  // moveit::planning_interface::MoveGroupInterface gripper("gripper");
  arm.setPlanningTime(45.0);

  ros::Publisher gripper_pub = nh.advertise<control_msgs::GripperCommandActionGoal>(
        "/my_gen3/robotiq_2f_85_gripper_controller/gripper_cmd/goal", 10);

  // Wait a bit for ROS things to initialize
  // ros::WallDuration(1.0).sleep();
  // geometry_msgs::Pose target_pose;
  // target_pose=list_to_pose(0.415, 0, 0.5,M_PI / 2,0,-M_PI / 2);
  // go_to_pose_goal(arm,target_pose);
    // place(arm,gripper);
	// pick(arm,gripper);

  bool loop = true;
  while(loop)
  {
    int cmd;
    std::cout << "Menu? (0:exit / 1:move to initail pose / 2:pre pick / 3:post pick / 4:pre place / 5:post place / 6:open gripper / 7:close gripper) ";
    std::cin >> cmd;
    switch(cmd) {
      case 1:
        move_initial(arm);
        ros::WallDuration(1.0).sleep();
        break;
      case 2:
        prePick(arm);
        ros::WallDuration(1.0).sleep();
        break;
      case 3:
        postPick(arm);
        ros::WallDuration(1.0).sleep();
        break;
      case 4:
        prePlace(arm);
        ros::WallDuration(1.0).sleep();
        break;
      case 5:
        postPlace(arm);
        ros::WallDuration(1.0).sleep();
        break;
      case 6:
        openGripper(gripper_pub);
        ros::WallDuration(1.0).sleep();
        break;
      case 7:
        closeGripper(gripper_pub);
        ros::WallDuration(1.0).sleep();
        break;
      case 8:
        spiralTrajectory(arm);
        ros::WallDuration(1.0).sleep();
        break;
      case 0:
        loop = false;
        ROS_INFO("Quit");
        break;
    }
  }
  ros::waitForShutdown();
  return 0;
}

