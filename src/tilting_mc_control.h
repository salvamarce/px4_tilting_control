/***
 * Position control for a tilting quadrotor
 ***/

//tarot.xacro values
#define MASS 1.5 //Kg
#define ARM_LENGHT 0.30 //m
#define THRUST_COEFF 8.54858e-06
#define TORQUE_COEFF 0.000001
#define INERTIAL_MATRIX1_1 0.0347563
#define INERTIAL_MATRIX2_2 0.0458929
#define INERTIAL_MATRIX3_3 0.0977
#define Z_POS_OFFSET 0.264

#define MOTOR_SPEED_MIN  10000.0 //to define
#define MOTOR_SPEED_MAX 1210000.0
#define MOTOR_TILT_MAX 1.57

#define POS_THRESHOLD 0.008
#define VEL_THRESHOLD 0.01
#define ACC_THRESHOLD 0.01
#define ATT_THRESHOLD 0.05
#define VEL_ATT_THRESHOLD 0.03
#define ACC_ATT_THRESHOLD 0.03
#define SING_THRESHOLD_1 0.15
#define SING_THRESHOLD_2 0.25
#define MOTOR_TILT_SPEED_THRESHOLD 3.14

#define SMOOTH_SPEED_CONST 3.0
#define SMOOTH_TILT_CONST 2.0

#include "Eigen/Dense"
#include <iostream>
#pragma once

struct PosSetpoint{
    Eigen::Vector3f point;
    Eigen::Vector3f vel;
    Eigen::Vector3f acc;
    Eigen::Vector3f jerk;
};

struct AttSetpoint{
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

        void updateStates(const TiltingMcStates *states);
        void updateMotorsSpeed(const Eigen::Matrix<float, 4, 1> speeds);
        void updateTiltAngles(const Eigen::Matrix<float, 4, 1> angles);
        void updatePosSetpoint(const PosSetpoint *setpoint);
        void updateSetpoint(const PosSetpoint *setpoint_pos, const AttSetpoint *setpoint_att);
        void updateAttSetpoint(const AttSetpoint *setpoint);
        void setPosGains(const Eigen::Vector3f Kp, const Eigen::Vector3f Kd, const Eigen::Vector3f Kdd);
        void setAttGains(const Eigen::Vector3f Kp, const Eigen::Vector3f Kd, const Eigen::Vector3f Kdd);
        void setOptGains(const float kh1, const float kh2, const float kh3, const float kH);
        
        void updateControl(const double time, const float dt);
        void updateControlMatrix();

        TiltingMcStates getStates();
        Eigen::Matrix3f getRotMatrix() { return Rbw;}
        PosSetpoint getPosSetpoint() { return pos_setpoint;}
        AttSetpoint getAttSetpoint() { return att_setpoint;}
        Eigen::Matrix3f getPosKp() { return Kp_pos;}
        Eigen::Matrix3f getPosKd() { return Kd_pos;}
        Eigen::Matrix3f getPosKdd() { return Kdd_pos;}
        Eigen::Matrix3f getAttKp() { return Kp_att;}
        Eigen::Matrix3f getAttKd() { return Kd_att;}
        Eigen::Matrix3f getAttKdd() { return Kdd_att;}
        Eigen::Matrix<float, 4, 1> getMotorsSpeed() { return motor_speed;}
        Eigen::Matrix<float, 4, 1> getMotorTiltAngle() { return motor_tilt;}
        int sign(const float num);
        Eigen::VectorXf threshold(const Eigen::VectorXf&, const int size, const float&);



    private:      
        float mass;
        float arm_lenght;
        float torque_coeff;
        float thrust_coeff;
        Eigen::Matrix3f inertial_matrix;
        Eigen::Vector3f pos;
        Eigen::Vector3f vel;
        Eigen::Vector3f acc;
        //Eigen::Vector3f RPY_att; //XYZ system
        Eigen::Vector3f RPY_vel;
        Eigen::Vector3f RPY_acc;
        Eigen::Quaternionf quat_att;
        Eigen::Matrix<float, 4, 1> motor_speed; //rotors speed in rad/s
        Eigen::Matrix<float, 4, 1> motor_tilt; //motors tilt angle 

        Eigen::Matrix<float, 6, 8> matrix_A; //matrix for the controller
        Eigen::Matrix<float, 6, 1> vector_B; //vector for the controller

        Eigen::Matrix3f Rbw; //rotation matrix, body to world frame
        Eigen::Matrix3f d_Rbw; //derivative of Rbw

        Eigen::Matrix<float, 8, 1> opt_fcn; //Vector from the optimizzation function, [8x1]

        PosSetpoint pos_setpoint;
        AttSetpoint att_setpoint;

        Eigen::Matrix3f Kp_pos;
        Eigen::Matrix3f Kd_pos;
        Eigen::Matrix3f Kdd_pos;
        Eigen::Matrix3f Kp_att;
        Eigen::Matrix3f Kd_att;
        Eigen::Matrix3f Kdd_att;
        float Kh1, Kh2, Kh3, KH; //Gains for the opt function

        Eigen::Vector3f pos_controller_input();
        Eigen::Vector3f att_controller_input();
        Eigen::Matrix<float, 4, 1> normalize_tilt(const Eigen::Matrix<float, 4, 1>&);
        //Eigen::Matrix3f RPYRotMat(const float phi, const float theta, const float psi); //angle in rad
        void optimizzation_function();
        void evaluate_d_Rbw();
        void zero_values();
};

