#include "ros/ros.h"
#include "tilting_mc_control.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include "MotionProfile.h"

#define TC 0.002 //local_pose e loca_vel vanno a 500Hz
#define V_MAX 0.2
#define A_MAX 0.1
#define J_MAX 0.05
#define DIFF_POS_THRESHOLD 0.001
#define DIFF_LIN_VEL_THRESHOLD 0.015
#define DIFF_ANG_VEL_THRESHOLD 0.035

using namespace std;

TiltingMcControl controllore;
TiltingMcStates new_state, old_state;
bool pose_init;
bool vel_init;


void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose){
    
    Eigen::Quaternionf quat;
    Eigen::Vector3f pos_temp, new_eul, old_eul, diff_pos, diff_vel;
    float temp;
    Eigen::Quaternionf NED2contr;
    NED2contr.vec() << 0.707388, -0.706825, 0.0005629;
    NED2contr.w() = 0.0005633;

    new_state.position << pose->pose.position.x, pose->pose.position.y, pose->pose.position.z;
    //new_state.position = NED2contr.toRotationMatrix() * new_state.position;

    quat.x() = pose->pose.orientation.x;
    quat.y() = pose->pose.orientation.y;
    quat.z() = pose->pose.orientation.z;
    quat.w() = pose->pose.orientation.w;
    new_state.quat_attitude = quat.normalized();
    //new_state.quat_attitude = NED2contr*quat.normalized();
    
    pose_init= true;
}

void vel_callback(const geometry_msgs::TwistStamped::ConstPtr& data){
    Eigen::Vector3f diff_ang_vel, diff_lin_vel;
    Eigen::Quaternionf NED2contr;
    NED2contr.vec() << 0.707388, -0.706825, 0.0005629;
    NED2contr.w() = 0.0005633;

    new_state.velocity << data->twist.linear.x, data->twist.linear.y, data->twist.linear.z;
    //new_state.velocity =  NED2contr.toRotationMatrix() * new_state.velocity;

    diff_lin_vel(0) = data->twist.linear.x - old_state.velocity(0);
    diff_lin_vel(1) = data->twist.linear.y - old_state.velocity(1);
    diff_lin_vel(2) = data->twist.linear.z - old_state.velocity(2);
    diff_lin_vel= controllore.threshold(diff_lin_vel, 3, DIFF_LIN_VEL_THRESHOLD);
    new_state.acceleration(0) = diff_lin_vel(0)/TC;
    new_state.acceleration(1) = diff_lin_vel(1)/TC;
    new_state.acceleration(2) = diff_lin_vel(2)/TC;//+ 9.81 +g because we are in NED, serve?
    //cout << "Lin acc: \n" << new_state.acceleration << endl;
    //new_state.acceleration =  NED2contr.toRotationMatrix() * new_state.acceleration;

    new_state.RPY_velocity << data->twist.angular.x, data->twist.angular.y, data->twist.angular.z;
    //new_state.RPY_velocity =  NED2contr.toRotationMatrix() * new_state.acceleration;
    
    diff_ang_vel(0) = data->twist.angular.x - old_state.RPY_velocity(0);
    diff_ang_vel(1) = data->twist.angular.y - old_state.RPY_velocity(1);
    diff_ang_vel(2) = data->twist.angular.z - old_state.RPY_velocity(2);
    diff_ang_vel = controllore.threshold(diff_ang_vel, 3, DIFF_ANG_VEL_THRESHOLD);
    new_state.RPY_acceleration = diff_ang_vel/TC;
    //cout << "Ang acc: \n" << new_state.RPY_acceleration << endl;
    //new_state.RPY_acceleration =  NED2contr.toRotationMatrix() * new_state.RPY_acceleration;

    vel_init= true;
}


int main (int argc, char** argv){

    ros::init( argc, argv, "tarot_tilting_control");
    ros::NodeHandle nh;
    Eigen::Vector3f Kpp, Kpd, Kpdd, Kap, Kad, Kadd;
    PosSetpoint pos_setpoint;
    AttSetpoint att_setpoint;
    double px, py, pz;
    double vx, vy, vz;
    double ax, ay, az;
    double jx, jy, jz;
    px=py=pz=vx=vy=vz=ax=ay=az=jx=jy=jz= 0.0;

    pose_init = false;
    vel_init = false;

    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32MultiArray>("/tarot_standalone/motor_vel", 0);
    ros::Publisher tilt_pub0 = nh.advertise<std_msgs::Float64>("/tilt/tilt_rotor_0_joint_controller/command", 0);
    ros::Publisher tilt_pub1 = nh.advertise<std_msgs::Float64>("/tilt/tilt_rotor_1_joint_controller/command", 0);
    ros::Publisher tilt_pub2 = nh.advertise<std_msgs::Float64>("/tilt/tilt_rotor_2_joint_controller/command", 0);
    ros::Publisher tilt_pub3 = nh.advertise<std_msgs::Float64>("/tilt/tilt_rotor_3_joint_controller/command", 0);
    ros::Subscriber pose_sub = nh.subscribe("/tarot/local_pose", 0, pose_callback); 
    ros::Subscriber vel_sub = nh.subscribe("/tarot/local_vel", 0, vel_callback);

    Kpp  <<   850.0, 850.0, 700.0;    
    Kpd  <<   580.0, 580.0, 600.0;    
    Kpdd <<   270.0, 270.0, 200.0;    
    Kap  <<   200.0, 200.0, 750.0;    
    Kad  <<   100.0, 100.0, 380.0;    
    Kadd <<    20.0,  20.0, 150.0;   

    controllore.setPosGains(Kpp, Kpd, Kpdd);
    controllore.setAttGains(Kap, Kad, Kadd);
    controllore.setOptGains(0.05, 0.05, 0.01, 1);

    std_msgs::Float32MultiArray m_data;
    Eigen::Matrix<float, 4, 1> motor_speed, motor_tilt;
    std_msgs::Float64 motor_tilt0, motor_tilt1, motor_tilt2, motor_tilt3;
    
    m_data.data.resize(4);

    while(!pose_init && !vel_init){
        controllore.updateStates(&new_state);
        ros::spinOnce();
    }
    old_state= new_state;

    pos_setpoint.point = new_state.position;
    pos_setpoint.vel= new_state.velocity;
    pos_setpoint.acc= new_state.acceleration;
    pos_setpoint.jerk << 0.0, 0.0, 0.0;

    att_setpoint.quat= new_state.quat_attitude;
    att_setpoint.vel= new_state.RPY_velocity;
    att_setpoint.acc= new_state.RPY_acceleration;
    att_setpoint.jerk << 0.0, 0.0, 0.0; 

    controllore.updateAttSetpoint(&att_setpoint);
    controllore.updatePosSetpoint(&pos_setpoint);

    MotionProfile trajZ(double(-Z_POS_OFFSET), double(-4.0), V_MAX, A_MAX, J_MAX);
    MotionProfile trajX(double(new_state.position(0)), double(0.0), V_MAX, A_MAX, J_MAX);
    MotionProfile trajY(double(new_state.position(1)), double(0.0), V_MAX, A_MAX, J_MAX);

    ros::Rate r(1/TC);
    bool first = true;
    float t0=ros::Time::now().toSec();
    double start = 5.0;


    while(ros::ok()){     
        if(first){ 
            cout << "Armo \n";
            //Arming mc
            m_data.data[0] = 200;
            m_data.data[1] = 200;
            m_data.data[2] = 200;
            m_data.data[3] = 200;
            motor_speed(0) = -200*200;
            motor_speed(1) = 200*200;
            motor_speed(2) = -200*200;
            motor_speed(3) = 200*200;
            vel_pub.publish(m_data);
            tilt_pub0.publish(-0.2);
            tilt_pub1.publish(0.2);
            tilt_pub2.publish(-0.2);
            tilt_pub3.publish(0.2);
            motor_tilt << -0.2, 0.2, -0.2, 0.2;

            controllore.updateMotorsSpeed(motor_speed);
            controllore.updateTiltAngles(motor_tilt); 
            first = false;                    
        }
        else if((ros::Time::now().toSec() - t0) < start){
            //cout << "Aspetto \n";
            controllore.updateStates(&new_state);
            controllore.updateControl(ros::Time::now().toSec()-t0, TC);
            motor_speed = controllore.getMotorsSpeed();
            motor_tilt = controllore.getMotorTiltAngle();
            motor_tilt0.data = motor_tilt(0);
            motor_tilt1.data = motor_tilt(1);
            motor_tilt2.data = motor_tilt(2);
            motor_tilt3.data = motor_tilt(3);
            
            m_data.data[0] = sqrt(abs(motor_speed(0)));//*controllore.sign(motor_speed(0));
            m_data.data[1] = sqrt(abs(motor_speed(1)));//*controllore.sign(motor_speed(1));
            m_data.data[2] = sqrt(abs(motor_speed(2)));//*controllore.sign(motor_speed(2));
            m_data.data[3] = sqrt(abs(motor_speed(3)));//*controllore.sign(motor_speed(3));
            
            vel_pub.publish(m_data);
            tilt_pub0.publish(motor_tilt0);
            tilt_pub1.publish(motor_tilt1);
            tilt_pub2.publish(motor_tilt2);
            tilt_pub3.publish(motor_tilt3);   
            
        }
        else{
            //cout << "Traiettoria \n";
            trajX.Compute(ros::Time::now().toSec()-t0, start, px, vx, ax, jx);
            trajY.Compute(ros::Time::now().toSec()-t0, start, py, vy, ay, jy);
            trajZ.Compute(ros::Time::now().toSec()-t0, start, pz, vz, az, jz);
            pos_setpoint.point << float(px), float(py), float(pz);
            pos_setpoint.vel << float(vx), float(vy), float(vz);
            pos_setpoint.acc << float(ax), float(ay), float(az);
            pos_setpoint.jerk << float(jx), float(jy), float(jz);
            controllore.updatePosSetpoint(&pos_setpoint);

            controllore.updateStates(&new_state);
            controllore.updateControl(ros::Time::now().toSec()-t0, TC);
            motor_speed = controllore.getMotorsSpeed();
            motor_tilt = controllore.getMotorTiltAngle();
            motor_tilt0.data = motor_tilt(0);
            motor_tilt1.data = motor_tilt(1);
            motor_tilt2.data = motor_tilt(2);
            motor_tilt3.data = motor_tilt(3);
            
            m_data.data[0] = sqrt(abs(motor_speed(0)));//*controllore.sign(motor_speed(0));
            m_data.data[1] = sqrt(abs(motor_speed(1)));//*controllore.sign(motor_speed(1));
            m_data.data[2] = sqrt(abs(motor_speed(2)));//*controllore.sign(motor_speed(2));
            m_data.data[3] = sqrt(abs(motor_speed(3)));//*controllore.sign(motor_speed(3));
            
            vel_pub.publish(m_data);
            tilt_pub0.publish(motor_tilt0);
            tilt_pub1.publish(motor_tilt1);
            tilt_pub2.publish(motor_tilt2);
            tilt_pub3.publish(motor_tilt3);

        }
        old_state = new_state;
        
        r.sleep();
        ros::spinOnce();
    }
    
    return(0);
}