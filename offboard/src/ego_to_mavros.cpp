/**
 * @file ego_to_mavros.cpp
 * @brief 接收ego发出的目标点，并发送给mavros
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
// Quaternion from euler angles
#include <tf/tf.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <offboard/PositionCommand.h>
#include <Eigen/Eigen>

int state_flag = 0;
int ego_poscmd_count = 0;
int id = 0;


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

Eigen::Vector3d currentPose;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    currentPose(0) = msg->pose.position.x;
    currentPose(1) = msg->pose.position.y;
    currentPose(2) = msg->pose.position.z;
}

mavros_msgs::PositionTarget setpoint_raw_local;
double yaw_set;
void ego_pos_cb(const offboard::PositionCommand::ConstPtr& msg)
{
    ROS_INFO("Received ego cmd");
    setpoint_raw_local.header.stamp = msg->header.stamp;
    setpoint_raw_local.coordinate_frame = 1; // MAV_FRAME_LOCAL_NED
    setpoint_raw_local.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                    mavros_msgs::PositionTarget::IGNORE_VY |
                                    mavros_msgs::PositionTarget::IGNORE_VZ |
                                    mavros_msgs::PositionTarget::IGNORE_AFX |
                                    mavros_msgs::PositionTarget::IGNORE_AFY |
                                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                                    mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    setpoint_raw_local.position = msg->position;
    setpoint_raw_local.velocity = msg->velocity;
    setpoint_raw_local.acceleration_or_force = msg->acceleration;
    setpoint_raw_local.yaw = msg->yaw;

    ego_poscmd_count++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ego_to_mavros");
    ros::NodeHandle nh("~");

    float TAKEOFF_HEIGHT = 2;

    nh.getParam("id", id);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav"+std::to_string(id)+"/mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber ego_pos_sub = nh.subscribe<offboard::PositionCommand>("/position_cmd", 10, ego_pos_cb);
    
    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav"+std::to_string(id)+"/mavros/setpoint_raw/local", 10);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav"+std::to_string(id)+"/mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav"+std::to_string(id)+"/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav"+std::to_string(id)+"/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav"+std::to_string(id)+"/mavros/set_mode");

    // timer_ = nh.createTimer(ros::Duration(0.005), std::bind(timer_cb, this));


    ros::Rate rate(20.0);
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_WARN("Waiting for connect");
    }
    ROS_INFO("Connected Success!");

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = TAKEOFF_HEIGHT;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){  
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_WARN("Switching to Offboard and arming the vehicle...");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if( current_state.mode == "OFFBOARD" ){
            if (ego_poscmd_count > 0) {
                ROS_INFO("ego_poscmd_count: %d", ego_poscmd_count);
                setpoint_raw_local_pub.publish(setpoint_raw_local);
            } else {
                // Takeoff mode
                ROS_INFO("Still waiting for ego pos cmd, using local position as target");
                local_pos_pub.publish(pose);
            }
        }
        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
