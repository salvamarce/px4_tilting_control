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


void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose){

    new_state.position << pose->pose.position.x, pose->pose.position.y, pose->pose.position.z;
    new_state.quat_attitude.x() = pose->pose.orientation.x;
    new_state.quat_attitude.y() = pose->pose.orientation.y;
    new_state.quat_attitude.z() = pose->pose.orientation.z;
    new_state.quat_attitude.w() = pose->pose.orientation.w;

}

void vel_acc_callback(const sensor_msgs::Imu::ConstPtr& data){

    new_state.RPY_velocity << data->angular_velocity.x, data->angular_velocity.y, data->angular_velocity.z;
    new_state.acceleration << data->linear_acceleration.x, data->linear_acceleration.y, data->linear_acceleration.z;

    new_state.RPY_acceleration(1) =  (data->angular_velocity.x - controllore.getStates().RPY_acceleration(1))/TC;
    new_state.RPY_acceleration(2) =  (data->angular_velocity.y - controllore.getStates().RPY_acceleration(2))/TC;
    new_state.RPY_acceleration(3) =  (data->angular_velocity.z - controllore.getStates().RPY_acceleration(3))/TC;

    new_state.velocity(1) = controllore.getStates().velocity(1) + data->linear_acceleration.x * TC;
    new_state.velocity(2) = controllore.getStates().velocity(2) + data->linear_acceleration.y * TC;
    new_state.velocity(3) = controllore.getStates().velocity(3) + data->linear_acceleration.z * TC;
    cout << data->linear_acceleration.x << endl;

}

int main (int argc, char** argv){

    ros::init( argc, argv, "tarot_tilting_control");
    ros::NodeHandle nh;
    Eigen::Vector3f Kpp, Kpd, Kpdd, Kap, Kad, Kadd;
    TiltingMcPosSetpoint set_pos;
    TiltingMcAttSetpoint set_att;

    set_pos.point << 0.0, 0.0, 10.0;
    set_pos.vel = Eigen::Vector3f::Zero();
    set_pos.acc = Eigen::Vector3f::Zero();
    set_pos.jerk = Eigen::Vector3f::Zero();

    set_att.quat.vec() = Eigen::Vector3f::Zero();
    set_att.quat.w() = 1;
    set_att.vel = Eigen::Vector3f::Zero();
    set_att.acc = Eigen::Vector3f::Zero();
    set_att.jerk = Eigen::Vector3f::Zero();

    controllore.updatePosSetpoint(&set_pos);
    controllore.updateAttSetpoint(&set_att);

    //The motor velocities will be published in:
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

    
    /*I versi dei motori sono ccw, ccw, cw, cw
    Però le velocità pare le voglia senza segno
    */

    ros::Rate r(1/TC);
    uint loops = 0;
    while(ros::ok()){
        if(loops >1500){
            controllore.updateStates(&new_state);
            
            controllore.Controller(TC);
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
    
    //ros::spin();
    return(0);
}