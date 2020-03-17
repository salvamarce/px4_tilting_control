/***
 * Position control for a tilting quadrotor
 ***/

//tarot.xacro values
#define MASS 1.5 //Kg
#define ARM_LENGHT 0.21 //m
#define THRUST_COEFF 0.0008
#define TORQUE_COEFF 0.000001
#define INERTIAL_MATRIX1_1 0.0347563
#define INERTIAL_MATRIX2_2 0.0458929
#define INERTIAL_MATRIX3_3 0.0977
#define MOTOR_SPEED_MIN 100 //to define
#define MOTOR_SPEED_MAX 1100
#define MOTOR_SPEED_HOVER 400 //to define
#define MOTOR_TILT_MAX 1.0472
#define ZERO_POS_Z 0.264

#include "Eigen/Dense"
#include <iostream>
#pragma once

struct TiltingMcPosSetpoint{
    Eigen::Vector3f point;
    Eigen::Vector3f vel;
    Eigen::Vector3f acc;
    Eigen::Vector3f jerk;
};

struct TiltingMcAttSetpoint{
    Eigen::Quaternionf quat;
    Eigen::Vector3f vel;
    Eigen::Vector3f acc;
    Eigen::Vector3f jerk;
};

struct TiltingMcStates{
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration;
    Eigen::Vector3f RPY_velocity;
    Eigen::Vector3f RPY_acceleration;
    Eigen::Quaternionf quat_attitude;
};

class TiltingMcControl{
    public:
        TiltingMcControl();
        ~TiltingMcControl() = default;

        void updateStates(const TiltingMcStates *states);
        void updateMotorsSpeed(const Eigen::Matrix<float, 4, 1> speeds);
        void updateTiltAngles(const Eigen::Matrix<float, 4, 1> angles);
        void updatePosSetpoint(const TiltingMcPosSetpoint *setpoint);
        void updateSetpoint(const TiltingMcPosSetpoint *setpoint_pos, const TiltingMcAttSetpoint *setpoint_att);
        void updateAttSetpoint(const TiltingMcAttSetpoint *setpoint);
        void setPosGains(const Eigen::Vector3f Kp, const Eigen::Vector3f Kd, const Eigen::Vector3f Kdd);
        void setAttGains(const Eigen::Vector3f Kp, const Eigen::Vector3f Kd, const Eigen::Vector3f Kdd);
        void setOptGains(const float Kh1, const float Kh2, const float KH);
        
        void Controller(const float dt);

        TiltingMcStates getStates();
        Eigen::Matrix3f getRotMatrix() { evaluateRbw(); return Rbw;}
        TiltingMcPosSetpoint getPosSetpoint() { return pos_setpoint;}
        TiltingMcAttSetpoint getPosAttpoint() { return att_setpoint;}
        Eigen::Matrix3f getPosKp() { return Kp_pos;}
        Eigen::Matrix3f getPosKd() { return Kd_pos;}
        Eigen::Matrix3f getPosKdd() { return Kdd_pos;}
        Eigen::Matrix3f getAttKp() { return Kp_att;}
        Eigen::Matrix3f getAttKd() { return Kd_att;}
        Eigen::Matrix3f getAttKdd() { return Kdd_att;}
        Eigen::Matrix<float, 4, 1> getMotorSpeed() { return motor_speed;}
        Eigen::Matrix<float, 4, 1> getMotorTiltAngle() { return motor_tilt;}


    private:      
        float mass;
        float arm_lenght;
        float torque_coeff;
        float thrust_coeff;
        Eigen::Matrix3f inertial_matrix;
        Eigen::Vector3f pos;
        Eigen::Vector3f vel;
        Eigen::Vector3f acc;
        Eigen::Vector3f RPY_att; //XYZ system
        Eigen::Vector3f RPY_vel;
        Eigen::Vector3f RPY_acc;
        Eigen::Quaternionf quat_att;
        Eigen::Matrix<float, 4, 1> motor_speed; //rotors speed in rad/s
        Eigen::Matrix<float, 4, 1> motor_tilt; //motors tilt angle 

        Eigen::Matrix3f Rbw; //rotation matrix, body to world frame
        Eigen::Matrix3f d_Rbw; //derivative of Rbw

        Eigen::Matrix<float, 8, 1> opt_fcn; //Vector from the optimizzation function, [8x1]

        TiltingMcPosSetpoint pos_setpoint;
        TiltingMcAttSetpoint att_setpoint;

        Eigen::Matrix3f Kp_pos;
        Eigen::Matrix3f Kd_pos;
        Eigen::Matrix3f Kdd_pos;
        Eigen::Matrix3f Kp_att;
        Eigen::Matrix3f Kd_att;
        Eigen::Matrix3f Kdd_att;
        float Kh1, Kh2, KH; //Gains for the opt function

        Eigen::Vector3f pos_controller_input();
        Eigen::Vector3f att_controller_input();
        Eigen::Matrix3f RPYRotMat(const float phi, const float theta, const float psi); //angle in rad
        void optimizzation_function();
        void evaluateRbw();
        void evaluate_d_Rbw();
        void zero_values();
        int sign(const float num);

};

