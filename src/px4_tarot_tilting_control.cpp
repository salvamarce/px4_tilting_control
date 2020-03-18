#include "ros/ros.h"
#include "tilting_mc_control.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <unistd.h>

#define TC 0.001

using namespace std;

TiltingMcControl controllore;
TiltingMcStates new_state;
bool pose_init, vel_acc_init;


void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose){

    new_state.position << pose->pose.position.x, pose->pose.position.y, pose->pose.position.z;
    
    new_state.quat_attitude.x() = pose->pose.orientation.x;
    new_state.quat_attitude.y() = pose->pose.orientation.y;
    new_state.quat_attitude.z() = pose->pose.orientation.z;
    new_state.quat_attitude.w() = pose->pose.orientation.w;

    pose_init= true;
}

void vel_acc_callback(const sensor_msgs::Imu::ConstPtr& data){
    TiltingMcStates old_state;
    old_state = controllore.getStates();

    new_state.RPY_velocity << data->angular_velocity.x, data->angular_velocity.y, data->angular_velocity.z;
    new_state.acceleration << data->linear_acceleration.x, data->linear_acceleration.y, data->linear_acceleration.z;

    new_state.RPY_acceleration(0) =  (data->angular_velocity.x - old_state.RPY_acceleration(0))/TC;
    new_state.RPY_acceleration(1) =  (data->angular_velocity.y - old_state.RPY_acceleration(1))/TC;
    new_state.RPY_acceleration(2) =  (data->angular_velocity.z - old_state.RPY_acceleration(2))/TC;

    new_state.velocity(0) = old_state.velocity(0) + data->linear_acceleration.x * TC;
    new_state.velocity(1) = old_state.velocity(1) + data->linear_acceleration.y * TC;
    new_state.velocity(2) = old_state.velocity(2) + data->linear_acceleration.z * TC;

    vel_acc_init= true;
}

int main (int argc, char** argv){

    ros::init( argc, argv, "tarot_tilting_control");
    ros::NodeHandle nh;
    Eigen::Vector3f Kpp, Kpd, Kpdd, Kap, Kad, Kadd;
    TiltingMcPosSetpoint set_pos;
    TiltingMcAttSetpoint set_att;

    pose_init = vel_acc_init = false;

    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32MultiArray>("/tarot/motor_vel", 0);
    ros::Subscriber pose_sub = nh.subscribe("/tarot/local_pose", 1000, pose_callback); 
    ros::Subscriber vel_acc_sub = nh.subscribe("/tarot/imu/data", 1000, vel_acc_callback);

    Kpp << 1.0, 1.0, 5.0;
    Kpd << 0.5, 0.5, 1.0;
    Kpdd << 0.1, 0.1, 0.5;
    Kap << 0.0001, 0.0001, 0.0001;
    Kad << 0.0001, 0.0001, 0.0001;
    Kadd << 0.0001, 0.0001, 0.0001;

    controllore.setPosGains(Kpp, Kpd, Kpdd);
    controllore.setAttGains(Kap, Kad, Kadd);
    controllore.setOptGains(5, 5, 1);

    std_msgs::Float32MultiArray m_data;
    Eigen::Matrix<float, 4, 1> motor_speed;

    m_data.data.resize(4);

    ros::Rate r(1000);
    uint loops = 0;

    while(!pose_init && !vel_acc_init){
        ros::spinOnce();
    }

    while(ros::ok()){
        controllore.updateStates(&new_state);

        if(loops >1500){
            
            controllore.Controller(0.01);
            motor_speed = controllore.getMotorSpeed();
            
            m_data.data[0] = abs(motor_speed(0));
            m_data.data[1] = abs(motor_speed(1));
            m_data.data[2] = abs(motor_speed(2));
            m_data.data[3] = abs(motor_speed(3));
            vel_pub.publish(m_data);
            
        }
        else{
            //Come se armassi il drone
            m_data.data[0] = 200;
            m_data.data[1] = 200;
            m_data.data[2] = 200;
            m_data.data[3] = 200;
            motor_speed(0) = 200;
            motor_speed(1) = 200;
            motor_speed(2) = 200;
            motor_speed(3) = 200;
            vel_pub.publish(m_data);
            controllore.updateMotorsSpeed(motor_speed);
        }

        loops ++;
        ros::spinOnce();
        r.sleep();
    }
    
    return(0);
}