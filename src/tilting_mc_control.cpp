#include "tilting_mc_control.h"
#include <math.h>

TiltingMcControl::TiltingMcControl(){
    mass= MASS;
    arm_lenght= ARM_LENGHT;
    thrust_coeff= THRUST_COEFF;
    torque_coeff= TORQUE_COEFF;
    inertial_matrix << INERTIAL_MATRIX1_1, 0.0, 0.0, 0.0, INERTIAL_MATRIX2_2, 0.0, 0.0, 0.0, INERTIAL_MATRIX3_3;
    zero_values();
}  

void TiltingMcControl::updateStates(const TiltingMcStates *stat){
    pos= stat->position;
    vel= stat->velocity;
    acc= stat->acceleration;
    RPY_vel= stat->RPY_velocity;
    RPY_acc= stat->RPY_acceleration;
    quat_att = stat->quat_attitude;
    //Capire quale terna utilizza gazebo per l'orientamento, io uso XYZ
    /*
    std::cout << "Pose: \n";
    std::cout << "position: \n" << pos << std::endl;
    std::cout << "velocity: \n" << vel << std::endl;
    std::cout << "acceleration: \n" << acc << std::endl;
    std::cout << "Quaternion: \n" << quat_att.w() << std::endl << quat_att.vec() << std::endl;
    std::cout << "Angular velocity: \n" << RPY_vel << std::endl;
    std::cout << "Angular acceleration: \n" << RPY_acc << std::endl;
    */

}

void TiltingMcControl::updatePosSetpoint(const TiltingMcPosSetpoint *setpoint){
    pos_setpoint= *setpoint;
    pos_setpoint.point(2) -= Z_POS_OFFSET;
}

void TiltingMcControl::updateAttSetpoint(const TiltingMcAttSetpoint *setpoint){
    att_setpoint= *setpoint;
}

void TiltingMcControl::updateMotorsSpeed(const Eigen::Matrix<float, 4, 1> speeds){
    motor_speed = speeds;
    //std::cout << "New speeds: \n" << speeds << std::endl;
}
void TiltingMcControl::updateTiltAngles(const Eigen::Matrix<float, 4, 1> angles){
    motor_tilt = angles;
    //std::cout << "New angles: \n" << angles << std::endl;
}

void TiltingMcControl::updateSetpoint(const TiltingMcPosSetpoint *setpoint_pos, const TiltingMcAttSetpoint *setpoint_att){
    pos_setpoint= *setpoint_pos;
    pos_setpoint.point(2) -= Z_POS_OFFSET;
    att_setpoint= *setpoint_att;
}

void TiltingMcControl::setPosGains(const Eigen::Vector3f Kp, const Eigen::Vector3f Kd, const Eigen::Vector3f Kdd){
    Kp_pos = Kp.asDiagonal();

    Kd_pos = Kd.asDiagonal();

    Kdd_pos = Kdd.asDiagonal();

}

void TiltingMcControl::setAttGains(const Eigen::Vector3f Kp, const Eigen::Vector3f Kd, const Eigen::Vector3f Kdd){
    Kp_att = Kp.asDiagonal();

    Kd_att = Kd.asDiagonal();

    Kdd_att = Kdd.asDiagonal();
}

void TiltingMcControl::setOptGains(const float kh1, const float kh2, const float kH){
    Kh1 = kh1;
    Kh2 = kh2;
    KH = kH;
}

void TiltingMcControl::zero_values(){
    pos << 0.0, 0.0, -Z_POS_OFFSET;
    vel = Eigen::VectorXf::Zero(3);
    acc = Eigen::VectorXf::Zero(3);
    RPY_att = Eigen::VectorXf::Zero(3);
    RPY_vel = Eigen::VectorXf::Zero(3);
    RPY_acc = Eigen::VectorXf::Zero(3);
    quat_att.w()= 1;
    quat_att.x()= 0;
    quat_att.y()= 0;
    quat_att.z()= 0;
    Rbw = Eigen::Matrix3f::Zero();
    d_Rbw = Eigen::Matrix3f::Zero();
    opt_fcn = Eigen::VectorXf::Zero(8);
    pos_setpoint.point << 0.0, 0.0, -Z_POS_OFFSET;
    pos_setpoint.vel = Eigen::VectorXf::Zero(3);
    pos_setpoint.acc = Eigen::VectorXf::Zero(3);
    pos_setpoint.jerk = Eigen::VectorXf::Zero(3);
    att_setpoint.quat.vec() << 0.0, 0.0, 0.705;
    att_setpoint.quat.w() = 0.708;
    att_setpoint.vel = Eigen::VectorXf::Zero(3);
    att_setpoint.acc = Eigen::VectorXf::Zero(3);
    att_setpoint.jerk = Eigen::VectorXf::Zero(3);
    Kp_pos = Eigen::Matrix3f::Zero();
    Kd_pos = Eigen::Matrix3f::Zero();
    Kdd_pos = Eigen::Matrix3f::Zero();
    Kp_att = Eigen::Matrix3f::Zero();
    Kd_att = Eigen::Matrix3f::Zero();
    Kdd_att = Eigen::Matrix3f::Zero();
}

TiltingMcStates TiltingMcControl::getStates(){
    TiltingMcStates output;
    output.position= pos;
    output.velocity= vel;
    output.acceleration= acc;
    output.RPY_velocity= RPY_vel;
    output.RPY_acceleration= RPY_acc;
    output.quat_attitude= quat_att;

    return output;
}

Eigen::Vector3f TiltingMcControl::pos_controller_input(){
    Eigen::Vector3f output;

    output = Kp_pos*(pos_setpoint.point - pos) + Kd_pos*(pos_setpoint.vel - vel) + Kdd_pos*(pos_setpoint.acc - acc) + pos_setpoint.jerk;

    //std::cout << "Err pos: \n" << pos_setpoint.point - pos << std::endl;
    return output;    
}

Eigen::Vector3f TiltingMcControl::att_controller_input(){
    Eigen::Matrix3f S;
    Eigen::Vector3f err, output;

    S << 0.0, -att_setpoint.quat.z(), att_setpoint.quat.y(),
         att_setpoint.quat.z(), 0.0, -att_setpoint.quat.x(),
         -att_setpoint.quat.y(), att_setpoint.quat.x(), 0.0;

    err= quat_att.w()*att_setpoint.quat.vec() - att_setpoint.quat.w()*quat_att.vec() - S*quat_att.vec();
    output = Kp_att*err + Kd_att*(att_setpoint.vel - RPY_vel) + Kdd_att*(att_setpoint.acc - RPY_acc) + att_setpoint.jerk;
    
    return output;
}

Eigen::Matrix3f TiltingMcControl::RPYRotMat(const float phi, const float theta, const float psi){ //angle in rad
    Eigen::Matrix3f rotX, rotY, rotZ;
    Eigen::Matrix3f output;

    rotX << 1.0, 0.0, 0.0,
            0.0, cos(phi), -sin(phi),
            0.0, sin(phi), cos(phi);

    rotY << cos(theta), 0.0, sin(theta),
            0.0, 1.0, 0.0,
            -sin(theta), 0.0, cos(theta);

    rotZ << cos(psi), -sin(psi), 0.0,
            sin(psi), cos(psi), 0.0,
            0.0, 0.0, 1.0;
    
    output= rotX*rotY*rotZ;
    return output;
}

void TiltingMcControl::evaluateRbw(){
    float w= quat_att.w();
    float x= quat_att.x();
    float y= quat_att.y();
    float z= quat_att.z();
    
    Rbw << 2*((w*w)+ (x*x))-1, 2*(x*y - w*z), 2*(x*z + w*y),
           2*(x*y + w*z), 2*((w*w) + (y*y))-1, 2*(y*z - w*x),
           2*(x*z - w*y), 2*(y*z + w*x), 2*((w*w) + (z*z))-1;
    //std::cout << "Matrice Rbw: \n" << Rbw << std::endl;
}

void TiltingMcControl::evaluate_d_Rbw(){
    Eigen::Matrix3f S;

    S << 0.0, -RPY_vel(2), RPY_vel(1),
         RPY_vel(2), 0.0, -RPY_vel(0),
         -RPY_vel(1), RPY_vel(0), 0.0;

    d_Rbw= S*Rbw;
    //std::cout << "Matrice d_Rbw: \n" << d_Rbw << std::endl;
}

void TiltingMcControl::Controller(const float dt){
   //Scrivo direttamente le matrici calcolate
    Eigen::Matrix<float, 6, 8> matrix_A;
    Eigen::Matrix<float, 8, 6> pinv_matrix_A;
    Eigen::Matrix<float, 6, 1> vector_B;
    Eigen::Vector3f ddd_pos, ddd_att;
    Eigen::Matrix<float, 4, 1> d_motor_speed, d_motor_tilt, new_motor_speed, new_motor_tilt;
    Eigen::Matrix<float, 8, 1> full_vect_out;
    Eigen::Matrix<float, 6, 1> full_vect_in;

    if(abs(motor_speed(0)) < MOTOR_SPEED_MIN && abs(motor_speed(1)) < MOTOR_SPEED_MIN &&
       abs(motor_speed(2)) < MOTOR_SPEED_MIN && abs(motor_speed(3)) < MOTOR_SPEED_MIN)
        std::cout << "The controller will start with the multirotor armed \n";
    else{

        float t2 = 1.0/mass;
        float t3 = cos(motor_tilt(0));
        float t4 = sin(motor_tilt(0));
        float t5 = cos(motor_tilt(1));
        float t6 = sin(motor_tilt(1));
        float t7 = cos(motor_tilt(2));
        float t8 = sin(motor_tilt(2));
        float t9 = cos(motor_tilt(3));
        float t10 = sin(motor_tilt(3));
        float t11 = inertial_matrix(0,0)*inertial_matrix(1,1)*inertial_matrix(2,2);
        float t12 = inertial_matrix(0,1)*inertial_matrix(1,2)*inertial_matrix(2,0);
        float t13 = inertial_matrix(0,2)*inertial_matrix(1,0)*inertial_matrix(2,1);
        float t18 = inertial_matrix(0,0)*inertial_matrix(1,2)*inertial_matrix(2,1);
        float t19 = inertial_matrix(0,1)*inertial_matrix(1,0)*inertial_matrix(2,2);
        float t20 = inertial_matrix(0,2)*inertial_matrix(1,1)*inertial_matrix(2,0);
        float t14 = t11+t12+t13-t18-t19-t20;
        float t15 = 1.0/t14;
        float t16 = inertial_matrix(0,1)*inertial_matrix(1,2);
        float t21 = inertial_matrix(0,2)*inertial_matrix(1,1);
        float t17 = t16-t21;
        float t22 = inertial_matrix(0,1)*inertial_matrix(2,2);
        float t28 = inertial_matrix(0,2)*inertial_matrix(2,1);
        float t23 = t22-t28;
        float t24 = inertial_matrix(1,1)*inertial_matrix(2,2);
        float t34 = inertial_matrix(1,2)*inertial_matrix(2,1);
        float t25 = t24-t34;
        float t26 = torque_coeff*t4;
        float t48 = arm_lenght*thrust_coeff*t3;
        float t27 = t26-t48;
        float t29 = torque_coeff*t3;
        float t30 = arm_lenght*thrust_coeff*t4;
        float t31 = t29+t30;
        float t32 = torque_coeff*t6;
        float t51 = arm_lenght*thrust_coeff*t5;
        float t33 = t32-t51;
        float t35 = torque_coeff*t5;
        float t36 = arm_lenght*thrust_coeff*t6;
        float t37 = t35+t36;
        float t38 = torque_coeff*t8;
        float t55 = arm_lenght*thrust_coeff*t7;
        float t39 = t38-t55;
        float t40 = torque_coeff*t7;
        float t41 = arm_lenght*thrust_coeff*t8;
        float t42 = t40+t41;
        float t43 = torque_coeff*t10;
        float t58 = arm_lenght*thrust_coeff*t9;
        float t44 = t43-t58;
        float t45 = torque_coeff*t9;
        float t46 = arm_lenght*thrust_coeff*t10;
        float t47 = t45+t46;
        float t49 = inertial_matrix(0,0)*inertial_matrix(1,2);
        float t52 = inertial_matrix(0,2)*inertial_matrix(1,0);
        float t50 = t49-t52;
        float t53 = inertial_matrix(0,0)*inertial_matrix(2,2);
        float t59 = inertial_matrix(0,2)*inertial_matrix(2,0);
        float t54 = t53-t59;
        float t56 = inertial_matrix(1,0)*inertial_matrix(2,2);
        float t60 = inertial_matrix(1,2)*inertial_matrix(2,0);
        float t57 = t56-t60;
        float t61 = inertial_matrix(0,0)*inertial_matrix(1,1);
        float t63 = inertial_matrix(0,1)*inertial_matrix(1,0);
        float t62 = t61-t63;
        float t64 = inertial_matrix(0,0)*inertial_matrix(2,1);
        float t68 = inertial_matrix(0,1)*inertial_matrix(2,0);
        float t65 = t64-t68;
        float t66 = inertial_matrix(1,0)*inertial_matrix(2,1);
        float t69 = inertial_matrix(1,1)*inertial_matrix(2,0);
        float t67 = t66-t69;

        evaluateRbw();

        matrix_A << -Rbw(0,1)*thrust_coeff*t2*t4 + Rbw(0,2)*thrust_coeff*t2*t3,
                    -Rbw(1,1)*thrust_coeff*t2*t4 + Rbw(1,2)*thrust_coeff*t2*t3,
                    -Rbw(2,1)*thrust_coeff*t2*t4 + Rbw(2,2)*thrust_coeff*t2*t3,
                    -t15*t17*t31 - t15*t23*t27, t15*t27*t54 + t15*t31*t50,
                    -t15*t27*t65 - t15*t31*t62, 
                    -Rbw(0,0)*thrust_coeff*t2*t6 - Rbw(0,2)*thrust_coeff*t2*t5,
                    -Rbw(1,0)*thrust_coeff*t2*t6 - Rbw(1,2)*thrust_coeff*t2*t5,
                    -Rbw(2,0)*thrust_coeff*t2*t6 - Rbw(2,2)*thrust_coeff*t2*t5,
                    -t15*t17*t37 - t15*t25*t33, t15*t37*t50 + t15*t33*t57,
                    -t15*t37*t62 - t15*t33*t67,
                    Rbw(0,1)*thrust_coeff*t2*t8 + Rbw(0,2)*thrust_coeff*t2*t7,
                    Rbw(1,1)*thrust_coeff*t2*t8 + Rbw(1,2)*thrust_coeff*t2*t7,
                    Rbw(2,1)*thrust_coeff*t2*t8 + Rbw(2,2)*thrust_coeff*t2*t7,
                    -t15*t17*t42 + t15*t23*t39, t15*t42*t50 - t15*t39*t54,
                    t15*t39*t65 - t15*t42*t62,
                    Rbw(0,0)*thrust_coeff*t2*t10 - Rbw(0,2)*thrust_coeff*t2*t9,
                    Rbw(1,0)*thrust_coeff*t2*t10 - Rbw(1,2)*thrust_coeff*t2*t9,
                    Rbw(2,0)*thrust_coeff*t2*t10 - Rbw(2,2)*thrust_coeff*t2*t9,
                    -t15*t17*t47 + t15*t25*t44, t15*t47*t50 - t15*t44*t57,
                    -t15*t47*t62 + t15*t44*t67,
                    -Rbw(0,1)*thrust_coeff*t2*t3*motor_speed(0) - Rbw(0,2)*thrust_coeff*t2*t4*motor_speed(0),
                    -Rbw(1,1)*thrust_coeff*t2*t3*motor_speed(0) - Rbw(1,2)*thrust_coeff*t2*t4*motor_speed(0),
                    -Rbw(2,1)*thrust_coeff*t2*t3*motor_speed(0) - Rbw(2,2)*thrust_coeff*t2*t4*motor_speed(0),
                    t15*t17*t27*motor_speed(0) - t15*t23*t31*motor_speed(0),    -t15*t27*t50*motor_speed(0) + t15*t31*t54*motor_speed(0),
                    t15*t27*t62*motor_speed(0) - t15*t31*t65*motor_speed(0),
                    -Rbw(0,0)*thrust_coeff*t2*t5*motor_speed(1) + Rbw(0,2)*thrust_coeff*t2*t6*motor_speed(1),
                    -Rbw(1,0)*thrust_coeff*t2*t5*motor_speed(1) + Rbw(1,2)*thrust_coeff*t2*t6*motor_speed(1),
                    -Rbw(2,0)*thrust_coeff*t2*t5*motor_speed(1) + Rbw(2,2)*thrust_coeff*t2*t6*motor_speed(1),
                    t15*t17*t33*motor_speed(1) - t15*t25*t37*motor_speed(1),    -t15*t33*t50*motor_speed(1) + t15*t37*t57*motor_speed(1),
                    t15*t33*t62*motor_speed(1) - t15*t37*t67*motor_speed(1),    
                    Rbw(0,1)*thrust_coeff*t2*t7*motor_speed(2) - Rbw(0,2)*thrust_coeff*t2*t8*motor_speed(2),
                    Rbw(1,1)*thrust_coeff*t2*t7*motor_speed(2) - Rbw(1,2)*thrust_coeff*t2*t8*motor_speed(2),
                    Rbw(2,1)*thrust_coeff*t2*t7*motor_speed(2) - Rbw(2,2)*thrust_coeff*t2*t8*motor_speed(2),
                    t15*t17*t39*motor_speed(2) + t15*t23*t42*motor_speed(2),    -t15*t39*t50*motor_speed(2) - t15*t42*t54*motor_speed(2),
                    t15*t39*t62*motor_speed(2) + t15*t42*t65*motor_speed(2),
                    Rbw(0,0)*thrust_coeff*t2*t9*motor_speed(3) + Rbw(0,2)*thrust_coeff*t2*t10*motor_speed(3),
                    Rbw(1,0)*thrust_coeff*t2*t9*motor_speed(3) + Rbw(1,2)*thrust_coeff*t2*t10*motor_speed(3),
                    Rbw(2,0)*thrust_coeff*t2*t9*motor_speed(3) + Rbw(2,2)*thrust_coeff*t2*t10*motor_speed(3),
                    t15*t17*t44*motor_speed(3) + t15*t25*t47*motor_speed(3),    -t15*t44*t50*motor_speed(3) - t15*t47*t57*motor_speed(3),
                    t15*t44*t62*motor_speed(3) + t15*t47*t67*motor_speed(3);

        evaluate_d_Rbw();

        vector_B << -motor_speed(0)*(d_Rbw(0,1)*thrust_coeff*t2*t4 - d_Rbw(0,2)*thrust_coeff*t2*t3)
                    -motor_speed(1)*(d_Rbw(0,0)*thrust_coeff*t2*t6 + d_Rbw(0,2)*thrust_coeff*t2*t5)
                    +motor_speed(2)*(d_Rbw(0,1)*thrust_coeff*t2*t8 + d_Rbw(0,2)*thrust_coeff*t2*t7)
                    +motor_speed(3)*(d_Rbw(0,0)*thrust_coeff*t2*t10 - d_Rbw(0,2)*thrust_coeff*t2*t9),
                    -motor_speed(0)*(d_Rbw(1,1)*thrust_coeff*t2*t4 - d_Rbw(1,2)*thrust_coeff*t2*t3)
                    -motor_speed(1)*(d_Rbw(1,0)*thrust_coeff*t2*t6 + d_Rbw(1,2)*thrust_coeff*t2*t5)
                    +motor_speed(2)*(d_Rbw(1,1)*thrust_coeff*t2*t8 + d_Rbw(1,2)*thrust_coeff*t2*t7)
                    +motor_speed(3)*(d_Rbw(1,0)*thrust_coeff*t2*t10 - d_Rbw(1,2)*thrust_coeff*t2*t9),
                    -motor_speed(0)*(d_Rbw(2,1)*thrust_coeff*t2*t4 - d_Rbw(2,2)*thrust_coeff*t2*t3)
                    -motor_speed(1)*(d_Rbw(2,0)*thrust_coeff*t2*t6 + d_Rbw(2,2)*thrust_coeff*t2*t5)
                    +motor_speed(2)*(d_Rbw(2,1)*thrust_coeff*t2*t8 + d_Rbw(2,2)*thrust_coeff*t2*t7)
                    +motor_speed(3)*(d_Rbw(2,0)*thrust_coeff*t2*t10 - d_Rbw(2,2)*thrust_coeff*t2*t9),
                    0.0, 0.0, 0.0;

        //std::cout << "Matrice A: \n" << matrix_A << std::endl << "Vettore B: \n" << vector_B << std::endl;

        ddd_pos = pos_controller_input();
        ddd_att = Eigen::Vector3f::Zero();//att_controller_input();
        optimizzation_function();

        full_vect_in << ddd_pos, ddd_att;
        pinv_matrix_A = matrix_A.transpose()*(matrix_A*matrix_A.transpose()).inverse();
        full_vect_out = pinv_matrix_A*(full_vect_in - vector_B) + (Eigen::MatrixXf::Identity(8,8) - pinv_matrix_A*matrix_A)*opt_fcn;
        //std::cout << "err pos: \n" << full_vect_in - vector_B << std::endl  ;  
        
        for (int i=0; i<4; i++)
                d_motor_speed(i) = sign(full_vect_out(i))*sqrt(full_vect_out(i));

        d_motor_speed << full_vect_out(0), full_vect_out(1), full_vect_out(2), full_vect_out(3);
        d_motor_tilt << full_vect_out(4), full_vect_out(5), full_vect_out(6), full_vect_out(7);
        new_motor_speed = motor_speed + d_motor_speed*dt;
        new_motor_tilt = motor_tilt + d_motor_tilt*dt;
        //std::cout << "--- D_speeds: \n" << d_motor_speed << std::endl;
        
        for(int i=0; i<4; i++){
            if(abs(new_motor_speed(i)) < MOTOR_SPEED_MIN ){
                new_motor_speed(i) = sign(new_motor_speed(i))*MOTOR_SPEED_MIN;
                std::cout << "sono in min \n";
            }
            else if(abs(new_motor_speed(i)) > MOTOR_SPEED_MAX){
                new_motor_speed(i) = sign(new_motor_speed(i))*MOTOR_SPEED_MAX;
                std::cout << "sono in max \n";
            }
            else
                std::cout << "sono in range \n";

            if(abs(new_motor_tilt(i)) > MOTOR_TILT_MAX)
                new_motor_tilt(i) = sign(new_motor_tilt(i))*MOTOR_TILT_MAX;
        }
        
        //std::cout << "Nuove vel: \n" << new_motor_speed << std::endl;
        motor_speed = new_motor_speed;
        motor_tilt = Eigen::Matrix<float, 4, 1>::Zero();//new_motor_tilt;
        
        //std::cout << "--- Speeds: \n" << motor_speed << "\n --- Angles : \n" <<  motor_tilt << std::endl;
    }
}

void TiltingMcControl::optimizzation_function(){
    Eigen::Matrix<float, 8, 1> gradient;
    float t1= 3.14/(2*(MOTOR_SPEED_HOVER - MOTOR_SPEED_MIN));
    float t2= -t1*MOTOR_SPEED_HOVER;  
    
    for (int i=0; i<4; i++){
    
        if(abs(motor_speed(i)) > MOTOR_SPEED_MIN && abs(motor_speed(i)) <= MOTOR_SPEED_HOVER)
            gradient(i) = 2*Kh1*t1*sign(motor_speed(i))*tan(t2 + t1*abs(motor_speed(i)))*(pow(tan(t2 + t1*abs(motor_speed(i))),2) + 1);
        else if(abs(motor_speed(i)) > MOTOR_SPEED_HOVER)
            gradient(i) = -2*Kh2*sign(motor_speed(i))*(MOTOR_SPEED_HOVER - abs(motor_speed(i)));
    }

    opt_fcn = -KH * gradient;
    
}

int TiltingMcControl::sign(const float num){
    if (num > 0)
        return 1;
    else if(num < 0)
        return -1;
    else
        return 0;
}
