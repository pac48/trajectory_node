//
// Created by picknik on 6/1/22.
//

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.h"
#include <math.h>
//RL
#include "rl/math/Transform.h"
#include "rl/math/Vector.h"
#include "rl/math/Matrix.h"
#include "rl/math/Unit.h"
#include "rl/mdl/Dynamic.h"
#include "rl/mdl/Model.h"
#include "rl/mdl/Body.h"
#include "rl/mdl/Joint.h"
#include "rl/mdl/UrdfFactory.h"


void printT(double* data){
    for (int r =0; r < 4; r++){
        printf("[");
        for (int c =0; c < 4; c++){
            printf("%f, ", data[r + c*4]);
        }
        printf("]\n");
    }
}


struct Transform
{
    double        _11, _21, _31, _41;
    double        _12, _22, _32, _42;
    double        _13, _23, _33, _43;
    double        _14, _24, _34, _44;
};


void GetRotation(double& Yaw, double& Pitch, double& Roll, Transform& T)
{
    if (T._11 == 1.0f)
    {
        Yaw = atan2f(T._13, T._34);
        Pitch = 0;
        Roll = 0;

    }else if (T._11 == -1.0f)
    {
        Yaw = atan2f(T._13, T._34);
        Pitch = 0;
        Roll = 0;
    }else
    {

        Yaw = atan2(-T._31, T._11);
        Pitch = asin(T._21);
        Roll = atan2(-T._23, T._22);
    }
}

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("trajectory_node");
    trajectory_msgs::msg::JointTrajectory jointTrajectoryMsg;
    rl::mdl::UrdfFactory urdf;
    rl::mdl::Dynamic model;
    urdf.load("robot.urdf", &model);

    rclcpp::Clock ros_clock(rcl_clock_type_e::RCL_ROS_TIME);
    jointTrajectoryMsg.header.stamp = ros_clock.now();
    jointTrajectoryMsg.header.frame_id = "world";

    std::unordered_set<std::string> control_joint = {"shoulder_pan_joint",
                                                     "shoulder_lift_joint",
                                                     "elbow_joint",
                                                     "wrist_1_joint",
                                                     "wrist_2_joint",
                                                     "wrist_3_joint"};
    std::vector<int> control_inds;
    for (int  i =0; i < model.getJoints(); i++){
        if (control_joint.find(model.getJoint(i)->getName()) != control_joint.end()){
            jointTrajectoryMsg.joint_names.push_back(model.getJoint(i)->getName());
            control_inds.push_back(i);
        }
    }

//    auto command_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//            std::string("admittance_controller/joint_trajectory"), rclcpp::SystemDefaultsQoS());
    auto command_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            std::string("admittance_controller/joint_trajectory"), rclcpp::SystemDefaultsQoS());


            int endEffectorIndex = 0; // if more than one ee
    int numEE = model.getOperationalDof();
    int numDof = model.getDof();
    int offseti = endEffectorIndex*6;
    int offsetj = 0; // maybe used for multi-arm systems

    auto all_jacobians_ = rl::math::Matrix(6*numEE, numDof);
    auto jacobian_ = rl::math::Matrix(6, numDof);
    auto pseudo_inverse_ = rl::math::Matrix(numDof, 6);


//    rl::math::Transform startEEPos = model.getOperationalPosition(endEffectorIndex);
//    rl::math::Transform curEEPos = model.getOperationalPosition(endEffectorIndex);
//    auto startPos = model.getPosition();
    std::vector<double> startPos;
    startPos.resize(6);
        startPos[0] =  0;
    startPos[1] =  -1.5700000000000001;
    startPos[2] =  1.5700000000000001;
    startPos[3] =  -1.5700000000000001;
    startPos[4] =  -1.5700000000000001;
    startPos[5] =  0;

//    auto curPos = model.getPosition();

    model.forwardPosition();

    double totalTime = 8; // seconds
    int numPoints = 50;
    double mag = 0.2;
    int numLoops = 3;

//    double* T = startEEPos.data();
//    printT(T);
//    double yaw;
//    double pitch;
//    double roll;
//    GetRotation(yaw, pitch,  roll, *((Transform*)T));

//    for (double phase =0; phase < 2*M_PI; phase+= 2*M_PI/numPoints){
double timeOffset = 0;
for (int n= 0 ; n < numLoops; n++){
    for (double time =timeOffset; time < totalTime+timeOffset; time += totalTime/numPoints){
//        double time = (phase/(2*M_PI))*totalTime;
        double phase = 2*M_PI*(time/totalTime);
        int sec = time;
        double x = mag*(sin(phase));
        double y = mag*(cos(phase)-1);
        trajectory_msgs::msg::JointTrajectoryPoint point;
        auto newTime = rclcpp::Time(sec,(time-sec)*1E9, rcl_clock_type_e::RCL_ROS_TIME);
        point.time_from_start =  newTime-rclcpp::Time(0, 0, rcl_clock_type_e::RCL_ROS_TIME);

        // cartesian
//        point.positions.resize(6);
//        point.velocities.resize(6, 0.0);
//        point.positions[0] = startEEPos.translation().x() + x;
//        point.positions[1] = startEEPos.translation().y() + y;
//        point.positions[2] = startEEPos.translation().z() + 0;
//        point.positions[3] = roll;
//        point.positions[4] = pitch;
//        point.positions[5] = yaw;
//        point.velocities[0] = mag*(cos(time))*totalTime/(2*M_PI);
//        point.velocities[1] = -mag*(sin(time))*totalTime/(2*M_PI);

        // joint space
        point.positions.resize(6);
        point.velocities.resize(6, 0.0);

        for (int  i =0; i < startPos.size(); i++){
            point.positions[i] = startPos[i];
        }


        point.positions[1] += x;
        point.positions[4] += 2*y;


        point.velocities[1] = mag*(cos(phase))*2*M_PI/totalTime;
        point.velocities[4] = -2*mag*(sin(phase))*2*M_PI/totalTime;

        printf("time values %f pose values [%f, %f, %f] \n", time, point.positions[0],point.positions[1],point.positions[2]);
        printf("time values %f vel values [%f, %f, %f] \n", time, point.velocities[0],point.velocities[1],point.velocities[2]);

        jointTrajectoryMsg.points.push_back(point);
    }
    timeOffset += totalTime;
}
//while(true){

    command_pub->publish(jointTrajectoryMsg);
//}


    rclcpp::shutdown();
    return 0;

//    colcon build --packages-up-to trajectory_node  --cmake-args -DCMAKE_BUILD_TYPE=Debug

}