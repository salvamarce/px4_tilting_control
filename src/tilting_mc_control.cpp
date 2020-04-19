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
    
    //std::cout << "Pose: \n";
    //std::cout << "position: \n" << pos << std::endl;
    //std::cout << "velocity: \n" << vel << std::endl;
    //std::cout << "acceleration: \n" << acc << std::endl;
    //std::cout << "Quaternion: \n" << quat_att.w() << std::endl << quat_att.vec() << std::endl;
    //std::cout << "Angular velocity: \n" << RPY_vel << std::endl;
    //std::cout << "Angular acceleration: \n" << RPY_acc << std::endl;

}

void TiltingMcControl::updatePosSetpoint(const PosSetpoint *setpoint){
    pos_setpoint= *setpoint;
    //std::cout << "Pos setpoint: \n" << pos_setpoint.point << std::endl;
}

void TiltingMcControl::updateAttSetpoint(const AttSetpoint *setpoint){
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

void TiltingMcControl::updateSetpoint(const PosSetpoint *setpoint_pos, const AttSetpoint *setpoint_att){
    pos_setpoint= *setpoint_pos;
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

void TiltingMcControl::setOptGains(const float kh1, const float kh2, const float kh3, const float kH){
    Kh1 = kh1;
    Kh2 = kh2;
    Kh3 = kh3;
    KH = kH;
}

void TiltingMcControl::zero_values(){
    pos << 0.0, 0.0, Z_POS_OFFSET;
    vel = Eigen::VectorXf::Zero(3);
    acc = Eigen::VectorXf::Zero(3);
    //RPY_att = Eigen::VectorXf::Zero(3);
    RPY_vel = Eigen::VectorXf::Zero(3);
    RPY_acc = Eigen::VectorXf::Zero(3);
    quat_att.w()= 1;
    quat_att.x()= 0;
    quat_att.y()= 0;
    quat_att.z()= 0;
    Rbw = Eigen::Matrix3f::Zero();
    d_Rbw = Eigen::Matrix3f::Zero();
    opt_fcn = Eigen::VectorXf::Zero(8);
    pos_setpoint.point << 0.0, 0.0, Z_POS_OFFSET;
    pos_setpoint.vel = Eigen::VectorXf::Zero(3);
    pos_setpoint.acc = Eigen::VectorXf::Zero(3);
    pos_setpoint.jerk = Eigen::VectorXf::Zero(3);
    att_setpoint.quat.vec() << 0.0, 0.0, 0.0;
    att_setpoint.quat.w() = 1.0;
    att_setpoint.vel = Eigen::VectorXf::Zero(3);
    att_setpoint.acc = Eigen::VectorXf::Zero(3);
    att_setpoint.jerk = Eigen::VectorXf::Zero(3);
    Kp_pos = Eigen::Matrix3f::Zero();
    Kd_pos = Eigen::Matrix3f::Zero();
    Kdd_pos = Eigen::Matrix3f::Zero();
    Kp_att = Eigen::Matrix3f::Zero();
    Kd_att = Eigen::Matrix3f::Zero();
    Kdd_att = Eigen::Matrix3f::Zero();
    motor_speed = Eigen::VectorXf::Zero(3);
    motor_tilt << 0.1, -0.01, 0.01, -0.01;
    matrix_A = Eigen::MatrixXf::Zero(6,8);
    vector_B = Eigen::MatrixXf::Zero(6,1);
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

Eigen::VectorXf TiltingMcControl::threshold(const Eigen::VectorXf& vec, const int size, const float& threshold){
    Eigen::VectorXf output(size);
    
    for(int i=0; i<size; i++){
        if(abs(vec(i)) < threshold){
            output(i) = 0.0;
        }
        else{
            output(i) = vec(i);
        }          
    }

    return output;
}

Eigen::Vector3f TiltingMcControl::pos_controller_input(){
    Eigen::Vector3f output;
    Eigen::Vector3f err_pos, err_vel, err_acc;

    err_pos = threshold(pos_setpoint.point - pos, 3, POS_THRESHOLD);
    err_vel = threshold(pos_setpoint.vel - vel, 3, VEL_THRESHOLD);
    err_acc = threshold(pos_setpoint.acc - acc, 3, ACC_THRESHOLD);

    output = Kp_pos*err_pos + Kd_pos*err_vel + Kdd_pos*err_acc + pos_setpoint.jerk;
    
    //std::cout << "Err pos: \n" << err_pos << std::endl;
    //std::cout << "Err vel: \n" << err_vel << std::endl;
    //std::cout << "Err acc: \n" << err_acc << std::endl;
       
    return output;    
}

Eigen::Vector3f TiltingMcControl::att_controller_input(){
    Eigen::Matrix3f S, RR;
    Eigen::Vector3f err, err_att, err_vel, err_acc, output, vect;
    Eigen::Matrix<float, 3, 3> Rbw, Rd;

    Rd = att_setpoint.quat.toRotationMatrix();
    Rbw = quat_att.toRotationMatrix();
    RR = Rbw.transpose()*Rd - Rd.transpose()*Rbw;
    vect << RR(2,1), RR(0,2), RR(1,0);
    S << 0.0, -att_setpoint.quat.z(), att_setpoint.quat.y(),
         att_setpoint.quat.z(), 0.0, -att_setpoint.quat.x(),
         -att_setpoint.quat.y(), att_setpoint.quat.x(), 0.0;

    //err= quat_att.w()*att_setpoint.quat.vec() - att_setpoint.quat.w()*quat_att.vec() - S*quat_att.vec();
    err = 0.5 * vect;
    err_att= threshold(err, 3, ATT_THRESHOLD);
    err_vel= threshold(att_setpoint.vel - RPY_vel, 3, VEL_ATT_THRESHOLD);
    err_acc= threshold(att_setpoint.acc - RPY_acc, 3, ACC_ATT_THRESHOLD);
    output = Kp_att*err_att + Kd_att*err_vel + Kdd_att*err_acc + att_setpoint.jerk;
    
    //std::cout << "Err att: \n" << err_att << std::endl;
    //std::cout << "Err vel: \n" << err_vel << std::endl;
    //std::cout << "Err acc: \n" << err_acc << std::endl;

    return output;
}

/*
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
*/

void TiltingMcControl::evaluate_d_Rbw(){
    Eigen::Matrix3f S;

    S << 0.0, -RPY_vel(2), RPY_vel(1),
         RPY_vel(2), 0.0, -RPY_vel(0),
         -RPY_vel(1), RPY_vel(0), 0.0;

    d_Rbw= Rbw*S;
    //std::cout << "Matrice d_Rbw: \n" << d_Rbw << std::endl;
}

void TiltingMcControl::updateControl(const double time, const float dt){
   //Scrivo direttamente le matrici calcolate
    float k= 0.00001;
    Eigen::Matrix<float, 8, 6> dls_matrix_A;
    Eigen::Vector3f ddd_pos, ddd_att;
    Eigen::Matrix<float, 4, 1> d_motor_speed, d_motor_tilt, new_motor_speed, new_motor_tilt;
    Eigen::Matrix<float, 8, 1> full_vect_out;
    Eigen::Matrix<float, 6, 1> full_vect_in;
    float alpha = 0.0;
    float smooth_tilt, smooth_speed;
    
    //Update controller status
    Rbw = quat_att.toRotationMatrix();
    evaluate_d_Rbw();
    updateControlMatrix();
    //std::cout << "Matrice A: \n" << matrix_A << std::endl << "Vettore B: \n" << vector_B << std::endl;
    
    //Evaluate errors
    ddd_pos = pos_controller_input();
    ddd_att = att_controller_input();

    //Evaluate opt function
    optimizzation_function();

    //Evaluate controller input
    full_vect_in << ddd_pos, ddd_att;

    //Change the value of k according to the distance from singularity
    alpha = (pow(motor_tilt(0),2)+pow(motor_tilt(1),2)+pow(motor_tilt(2),2)+pow(motor_tilt(3),2));
    if(alpha <= SING_THRESHOLD_1){
        k = 0.00001;
        dls_matrix_A = matrix_A.transpose()*(matrix_A*matrix_A.transpose() + (k*k)*Eigen::MatrixXf::Identity(6,6)).inverse();
    }else if( alpha > SING_THRESHOLD_1 && alpha < SING_THRESHOLD_2){
        k = 0.00001 - (0.00001/SING_THRESHOLD_2)*(alpha - SING_THRESHOLD_1);
        dls_matrix_A = matrix_A.transpose()*(matrix_A*matrix_A.transpose() + (k*k)*Eigen::MatrixXf::Identity(6,6)).inverse();
    }else{
        k= 0.0;        
        dls_matrix_A = matrix_A.transpose()*(matrix_A*matrix_A.transpose()).inverse();
    }
    //std::cout << "K: " << k << "\t alpha: " << alpha << std::endl;
    
    //Evaluate controller output
    full_vect_out = dls_matrix_A*(full_vect_in - vector_B) + (Eigen::MatrixXf::Identity(8,8) - dls_matrix_A*matrix_A)*opt_fcn;  

    d_motor_speed << full_vect_out(0), full_vect_out(1), full_vect_out(2), full_vect_out(3);
    d_motor_tilt << full_vect_out(4), full_vect_out(5), full_vect_out(6), full_vect_out(7);
    smooth_speed = exp(-1/(SMOOTH_SPEED_CONST*time));
    smooth_tilt = exp(-1/(SMOOTH_TILT_CONST*time));
    d_motor_speed *= smooth_speed;
    d_motor_tilt *= smooth_tilt;
    
    //Aplly thresholds
    d_motor_tilt(0) = (abs(d_motor_tilt(0)) > MOTOR_TILT_SPEED_THRESHOLD) ? sign(d_motor_tilt(0))*MOTOR_TILT_SPEED_THRESHOLD : d_motor_tilt(0);
    d_motor_tilt(1) = (abs(d_motor_tilt(1)) > MOTOR_TILT_SPEED_THRESHOLD) ? sign(d_motor_tilt(1))*MOTOR_TILT_SPEED_THRESHOLD : d_motor_tilt(1);
    d_motor_tilt(2) = (abs(d_motor_tilt(2)) > MOTOR_TILT_SPEED_THRESHOLD) ? sign(d_motor_tilt(2))*MOTOR_TILT_SPEED_THRESHOLD : d_motor_tilt(2);
    d_motor_tilt(3) = (abs(d_motor_tilt(3)) > MOTOR_TILT_SPEED_THRESHOLD) ? sign(d_motor_tilt(3))*MOTOR_TILT_SPEED_THRESHOLD : d_motor_tilt(3);

    //Integrate speeds
    new_motor_speed = motor_speed + d_motor_speed*dt;
    new_motor_tilt = motor_tilt + d_motor_tilt*dt;

    //Normalize tilt between 
    new_motor_tilt = normalize_tilt(new_motor_tilt);
    
    for(int i=0; i<4; i++){
        if(abs(new_motor_speed(i)) < MOTOR_SPEED_MIN ){
            new_motor_speed(i) = sign(new_motor_speed(i))*MOTOR_SPEED_MIN;
            //std::cout << "sono in min \n";
        }
        else if(abs(new_motor_speed(i)) > MOTOR_SPEED_MAX){
            new_motor_speed(i) = sign(new_motor_speed(i))*MOTOR_SPEED_MAX;
            //std::cout << "sono in max \n";
        }
        
        if(abs(new_motor_tilt(i)) > MOTOR_TILT_MAX)
            new_motor_tilt(i) = sign(new_motor_tilt(i))*MOTOR_TILT_MAX;
            
        
    }
    if (!isnan(new_motor_speed(0)) && !isnan(new_motor_speed(1)) && !isnan(new_motor_speed(2)) && !isnan(new_motor_speed(3)) &&
        !isnan(new_motor_tilt(0)) && !isnan(new_motor_tilt(1)) && !isnan(new_motor_tilt(2)) && !isnan(new_motor_tilt(3)))
    {    
        updateMotorsSpeed(new_motor_speed);
        updateTiltAngles(new_motor_tilt);
    }else
    {
        std::cout << "----- Valori NaN, azzero tutto ----- \n";
        updateMotorsSpeed(Eigen::MatrixXf::Zero(4,1));
        updateTiltAngles(Eigen::MatrixXf::Zero(4,1));

    }
    
    
}

void TiltingMcControl::optimizzation_function(){
    Eigen::Matrix<float, 8, 1> gradient;
    Eigen::Matrix<float, 4, 1> gradient_W, gradient_A;
    gradient = Eigen::MatrixXf::Zero(8,1);
    gradient_W = Eigen::MatrixXf::Zero(4,1);
    gradient_A = Eigen::MatrixXf::Zero(4,1);
    float motor_hover_speed = (MASS*9.8)/(4*THRUST_COEFF);
    float t1= 3.14/(2*(motor_hover_speed - MOTOR_SPEED_MIN));
    float t2= -t1*motor_hover_speed;  
    
    for (int i=0; i<4; i++){
    
        if(abs(motor_speed(i)) > MOTOR_SPEED_MIN && abs(motor_speed(i)) <= motor_hover_speed)
            gradient_W(i) = 2*Kh1*t1*sign(motor_speed(i))*tan(t2 + t1*abs(motor_speed(i)))*pow(1/(cos(t2 + t1*abs(motor_speed(i)))),2);
        else if(abs(motor_speed(i)) > motor_hover_speed)
            gradient_W(i) = 2*Kh2*sign(motor_speed(i))*(abs(motor_speed(i)) - motor_hover_speed);

        //questa vale per ub= 0.785 rad, lb=-0.785 rad, mean_value= 0.0
        gradient_A(i) = Kh3*(1/8)*2/(1.57*1.57)*2*motor_tilt(i); 
    }
    gradient << gradient_W, gradient_A;
    opt_fcn = -KH * gradient;
    
}

Eigen::Matrix<float, 4, 1> TiltingMcControl::normalize_tilt(const Eigen::Matrix<float, 4, 1>& tilts){
    Eigen::Matrix<float, 4, 1> angle;
    angle = Eigen::MatrixXf::Zero(4,1);
    float PI = 3.145;

    //reduce the angle
    angle(0) = fmod(tilts(0), (2*PI));
    angle(1) = fmod(tilts(1), (2*PI));
    angle(2) = fmod(tilts(2), (2*PI));
    angle(3) = fmod(tilts(3), (2*PI));
    
    //force it to be the positive remainder, so that 0 <= angle < 2*PI 
    angle(0) = fmod((angle(0)+2*PI), (2*PI));
    angle(1) = fmod((angle(1)+2*PI), (2*PI));
    angle(2) = fmod((angle(2)+2*PI), (2*PI));
    angle(3) = fmod((angle(3)+2*PI), (2*PI)); 
    
    //force into the minimum absolute value residue class, so that -PI < angle <= PI 
    if(angle(0) > PI)
        angle(0) =  (angle(0) -2*PI);

    if(angle(1) > PI)
        angle(1) =  (angle(1) -2*PI);

    if(angle(2) > PI)
        angle(2) =  (angle(2) -2*PI);

    if(angle(3) > PI)
        angle(3) =  (angle(3) -2*PI);
   
    return angle;

}

int TiltingMcControl::sign(const float num){
    if (num > 0)
        return 1;
    else if(num < 0)
        return -1;
    else
        return 0;
}

void TiltingMcControl::updateControlMatrix(){

    float t2;  float t4;  float t5;  float t6;  float t7;  float t8;
    float t9;  float t10;  float t11;  float t14_tmp_tmp;  float t50;
    float t14;  float t19_tmp;  float b_t19_tmp;  float t57;
    float c_t19_tmp;  float d_t19_tmp;  float e_t19_tmp;  float t19;
    float t24;  float t26;  float t28;  float t35;  float t38;  float t40;
    float t43;  float t45;  float t48;  float t52;  float t55;  float t60;
    float t65;  float t69;  float t72;  float t75;  float t78;  float t81;
    float t84;  float t86;  float t88;  float A_tmp;  float b_A_tmp;  float c_A_tmp;
    float d_A_tmp;  float e_A_tmp;  float f_A_tmp;  float g_A_tmp;  float h_A_tmp;
    float i_A_tmp;  float j_A_tmp;  float k_A_tmp;
    float w1; float w2; float w3; float w4; float a1; float a2; float a3; float a4;
    float Rbw1_1; float Rbw1_2; float Rbw1_3; float Rbw2_1; float Rbw2_2; float Rbw2_3;
    float Rbw3_1; float Rbw3_2; float Rbw3_3; float d_Rbw1_1; float d_Rbw1_2; float d_Rbw1_3;
    float d_Rbw2_1; float d_Rbw2_2; float d_Rbw2_3; float d_Rbw3_1; float d_Rbw3_2; float d_Rbw3_3;
    float IB1_1; float IB1_2; float IB1_3; float IB2_1; float IB2_2; float IB2_3;
    float IB3_1; float IB3_2; float IB3_3; float m; float L; float km; float kf;
    float A[48], b[8];

    m= MASS; L= ARM_LENGHT; km= TORQUE_COEFF; kf= THRUST_COEFF;
    w1= motor_speed(0); w2=motor_speed(1); w3=motor_speed(2); w4=motor_speed(3);
    a1= motor_tilt(0); a2=motor_tilt(1); a3=motor_tilt(2); a4=motor_tilt(3);
    Rbw1_1 = Rbw(0,0); Rbw1_2 = Rbw(0,1); Rbw1_3 = Rbw(0,2); 
    Rbw2_1 = Rbw(1,0); Rbw2_2 = Rbw(1,1); Rbw2_3 = Rbw(1,2);
    Rbw3_1 = Rbw(2,0); Rbw3_2 = Rbw(2,1); Rbw3_3 = Rbw(2,2);
    d_Rbw1_1 = d_Rbw(0,0); d_Rbw1_2 = d_Rbw(0,1); d_Rbw1_3 = d_Rbw(0,2); 
    d_Rbw2_1 = d_Rbw(1,0); d_Rbw2_2 = d_Rbw(1,1); d_Rbw2_3 = d_Rbw(1,2);
    d_Rbw3_1 = d_Rbw(2,0); d_Rbw3_2 = d_Rbw(2,1); d_Rbw3_3 = d_Rbw(2,2);
    IB1_1 = inertial_matrix(0,0); IB1_2 = inertial_matrix(0,1); IB1_3 = inertial_matrix(0,2);
    IB2_1 = inertial_matrix(1,0); IB2_2 = inertial_matrix(1,1); IB2_3 = inertial_matrix(1,2);
    IB3_1 = inertial_matrix(2,0); IB3_2 = inertial_matrix(2,1); IB3_3 = inertial_matrix(2,2);

    t2 = 1.0 / m;
    t4 = std::sin(a1);
    t5 = std::sin(a2);
    t6 = std::sin(a3);
    t7 = std::sin(a4);
    t8 = std::cos(a1);
    t9 = std::cos(a2);
    t10 = std::cos(a3);
    t11 = std::cos(a4);
    t14_tmp_tmp = L * kf;
    t50 = t14_tmp_tmp * 1.4142135623730951;
    t14 = km * 1.4142135623730951 * t4 / 2.0 + t50 * t8 / 2.0;
    t19_tmp = IB1_2 * IB2_3;
    b_t19_tmp = IB1_3 * IB2_2;
    t57 = IB1_1 * IB2_3;
    c_t19_tmp = IB1_3 * IB2_1;
    d_t19_tmp = IB1_1 * IB2_2;
    e_t19_tmp = IB1_2 * IB2_1;
    t19 = 1.0 / (((((d_t19_tmp * IB3_3 + t19_tmp * IB3_1) + c_t19_tmp * IB3_2) - t57 * IB3_2) - e_t19_tmp * IB3_3) - b_t19_tmp * IB3_1);
    t24 = IB1_2 * IB3_3 - IB1_3 * IB3_2;
    t26 = IB2_2 * IB3_3 - IB2_3 * IB3_2;
    t28 = km * 1.4142135623730951 * t5 / 2.0 - t50 * t9 / 2.0;
    t35 = km * 1.4142135623730951 * t6 / 2.0 + t50 * t10 / 2.0;
    t38 = km * 1.4142135623730951 * t7 / 2.0 - t50 * t11 / 2.0;
    t40 = km * 1.4142135623730951 * t8 / 2.0 - t50 * t4 / 2.0;
    t43 = km * 1.4142135623730951 * t9 / 2.0 + t50 * t5 / 2.0;
    t45 = km * 1.4142135623730951 * t10 / 2.0 - t50 * t6 / 2.0;
    t48 = km * 1.4142135623730951 * t11 / 2.0 + t50 * t7 / 2.0;
    t50 = km * t8 - t14_tmp_tmp * t4;
    t52 = IB1_1 * IB3_3 - IB1_3 * IB3_1;
    t55 = IB2_1 * IB3_3 - IB2_3 * IB3_1;
    t57 -= c_t19_tmp;
    t60 = km * t9 + t14_tmp_tmp * t5;
    t65 = km * t10 - t14_tmp_tmp * t6;
    t69 = km * t11 + t14_tmp_tmp * t7;
    t72 = km * t4 + t14_tmp_tmp * t8;
    t75 = km * t5 - t14_tmp_tmp * t9;
    t78 = km * t6 + t14_tmp_tmp * t10;
    t81 = km * t7 - t14_tmp_tmp * t11;
    t84 = IB1_1 * IB3_2 - IB1_2 * IB3_1;
    t86 = IB2_1 * IB3_2 - IB2_2 * IB3_1;
    t88 = d_t19_tmp - e_t19_tmp;
    A_tmp = Rbw1_1 * kf * t2 * 1.4142135623730951;
    b_A_tmp = Rbw1_2 * kf * t2 * 1.4142135623730951;
    c_A_tmp = -Rbw1_3 * kf * t2;
    A[0] = (c_A_tmp * t8 - A_tmp * t4 / 2.0) + b_A_tmp * t4 / 2.0;
    d_A_tmp = Rbw2_1 * kf * t2 * 1.4142135623730951;
    e_A_tmp = Rbw2_2 * kf * t2 * 1.4142135623730951;
    f_A_tmp = -Rbw2_3 * kf * t2;
    A[1] = (f_A_tmp * t8 - d_A_tmp * t4 / 2.0) + e_A_tmp * t4 / 2.0;
    g_A_tmp = Rbw3_1 * kf * t2 * 1.4142135623730951;
    h_A_tmp = Rbw3_2 * kf * t2 * 1.4142135623730951;
    i_A_tmp = -Rbw3_3 * kf * t2;
    A[2] = (i_A_tmp * t8 - g_A_tmp * t4 / 2.0) + h_A_tmp * t4 / 2.0;
    j_A_tmp = t14 * t19;
    k_A_tmp = -t14 * t19;
    t14 = t19 * (t19_tmp - b_t19_tmp);
    A[3] = (k_A_tmp * t24 - j_A_tmp * t26) - t14 * t50;
    d_t19_tmp = t19 * t50;
    A[4] = (j_A_tmp * t52 + j_A_tmp * t55) + d_t19_tmp * t57;
    A[5] = (k_A_tmp * t84 - j_A_tmp * t86) - d_t19_tmp * t88;
    j_A_tmp = Rbw1_3 * kf * t2;
    A[6] = (j_A_tmp * t9 + A_tmp * t5 / 2.0) + b_A_tmp * t5 / 2.0;
    k_A_tmp = Rbw2_3 * kf * t2;
    A[7] = (k_A_tmp * t9 + d_A_tmp * t5 / 2.0) + e_A_tmp * t5 / 2.0;
    d_t19_tmp = Rbw3_3 * kf * t2;
    A[8] = (d_t19_tmp * t9 + g_A_tmp * t5 / 2.0) + h_A_tmp * t5 / 2.0;
    e_t19_tmp = t19 * t24;
    c_t19_tmp = t19 * t26;
    A[9] = (e_t19_tmp * t28 - c_t19_tmp * t28) - t14 * t60;
    t14_tmp_tmp = t19 * t28;
    t57 *= t19;
    A[10] = (-t19 * t28 * t52 + t14_tmp_tmp * t55) + t57 * t60;
    A[11] = (t14_tmp_tmp * t84 - t14_tmp_tmp * t86) - t19 * t60 * t88;
    A[12] = (c_A_tmp * t10 + A_tmp * t6 / 2.0) - b_A_tmp * t6 / 2.0;
    A[13] = (f_A_tmp * t10 + d_A_tmp * t6 / 2.0) - e_A_tmp * t6 / 2.0;
    A[14] = (i_A_tmp * t10 + g_A_tmp * t6 / 2.0) - h_A_tmp * t6 / 2.0;
    A[15] = (e_t19_tmp * t35 + c_t19_tmp * t35) - t14 * t65;
    t14_tmp_tmp = t19 * t35;
    A[16] = (-t19 * t35 * t52 - t14_tmp_tmp * t55) + t57 * t65;
    A[17] = (t14_tmp_tmp * t84 + t14_tmp_tmp * t86) - t19 * t65 * t88;
    A[18] = (j_A_tmp * t11 - A_tmp * t7 / 2.0) - b_A_tmp * t7 / 2.0;
    A[19] = (k_A_tmp * t11 - d_A_tmp * t7 / 2.0) - e_A_tmp * t7 / 2.0;
    A[20] = (d_t19_tmp * t11 - g_A_tmp * t7 / 2.0) - h_A_tmp * t7 / 2.0;
    t14_tmp_tmp = -t19 * t24;
    A[21] = (t14_tmp_tmp * t38 + c_t19_tmp * t38) - t14 * t69;
    t50 = t19 * t38;
    A[22] = (t50 * t52 - t50 * t55) + t57 * t69;
    A[23] = (-t19 * t38 * t84 + t50 * t86) - t19 * t69 * t88;
    A[24] = (j_A_tmp * t4 * w1 - A_tmp * t8 * w1 / 2.0) + b_A_tmp * t8 * w1 / 2.0;
    A[25] = (k_A_tmp * t4 * w1 - d_A_tmp * t8 * w1 / 2.0) + e_A_tmp * t8 * w1 / 2.0;
    A[26] = (d_t19_tmp * t4 * w1 - g_A_tmp * t8 * w1 / 2.0) + h_A_tmp * t8 * w1 / 2.0;
    A[27] = (t14_tmp_tmp * t40 * w1 - c_t19_tmp * t40 * w1) + t14 * t72 * w1;
    t50 = t19 * t40;
    A[28] = (t50 * t52 * w1 + t50 * t55 * w1) - t57 * t72 * w1;
    A[29] = (-t19 * t40 * t84 * w1 - t50 * t86 * w1) + t19 * t72 * t88 * w1;
    A[30] = (c_A_tmp * t5 * w2 + A_tmp * t9 * w2 / 2.0) + b_A_tmp * t9 * w2 / 2.0;
    A[31] = (f_A_tmp * t5 * w2 + d_A_tmp * t9 * w2 / 2.0) + e_A_tmp * t9 * w2 / 2.0;
    A[32] = (i_A_tmp * t5 * w2 + g_A_tmp * t9 * w2 / 2.0) + h_A_tmp * t9 * w2 / 2.0;
    A[33] = (e_t19_tmp * t43 * w2 - c_t19_tmp * t43 * w2) + t14 * t75 * w2;
    t50 = t19 * t43;
    A[34] = (-t19 * t43 * t52 * w2 + t50 * t55 * w2) - t57 * t75 * w2;
    A[35] = (t50 * t84 * w2 - t50 * t86 * w2) + t19 * t75 * t88 * w2;
    A[36] = (j_A_tmp * t6 * w3 + A_tmp * t10 * w3 / 2.0) - b_A_tmp * t10 * w3 / 2.0;
    A[37] = (k_A_tmp * t6 * w3 + d_A_tmp * t10 * w3 / 2.0) - e_A_tmp * t10 * w3 / 2.0;
    A[38] = (d_t19_tmp * t6 * w3 + g_A_tmp * t10 * w3 / 2.0) - h_A_tmp * t10 * w3 / 2.0;
    A[39] = (e_t19_tmp * t45 * w3 + c_t19_tmp * t45 * w3) + t14 * t78 * w3;
    j_A_tmp = t19 * t45;
    A[40] = (-t19 * t45 * t52 * w3 - j_A_tmp * t55 * w3) - t57 * t78 * w3;
    A[41] = (j_A_tmp * t84 * w3 + j_A_tmp * t86 * w3) + t19 * t78 * t88 * w3;
    A[42] = (c_A_tmp * t7 * w4 - A_tmp * t11 * w4 / 2.0) - b_A_tmp * t11 * w4 / 2.0;
    A[43] = (f_A_tmp * t7 * w4 - d_A_tmp * t11 * w4 / 2.0) - e_A_tmp * t11 * w4 / 2.0;
    A[44] = (i_A_tmp * t7 * w4 - g_A_tmp * t11 * w4 / 2.0) - h_A_tmp * t11 * w4 / 2.0;
    A[45] = (t14_tmp_tmp * t48 * w4 + c_t19_tmp * t48 * w4) + t14 * t81 * w4;
    A_tmp = t19 * t48;
    A[46] = (A_tmp * t52 * w4 - A_tmp * t55 * w4) - t57 * t81 * w4;
    A[47] = (-t19 * t48 * t84 * w4 + A_tmp * t86 * w4) + t19 * t81 * t88 * w4;
    t50 = d_Rbw1_3 * kf * t2;
    t57 = d_Rbw1_1 * kf * t2 * 1.4142135623730951;
    c_t19_tmp = d_Rbw1_2 * kf * t2 * 1.4142135623730951;
    b[0] = ((-w1 * ((t50 * t8 + t57 * t4 / 2.0) - c_t19_tmp * t4 / 2.0) + w2 *((t50 * t9 + t57 * t5 / 2.0) + c_t19_tmp * t5 / 2.0)) - w3 * ((t50 *t10 - t57 * t6 / 2.0) + c_t19_tmp * t6 / 2.0)) - w4 * ((-d_Rbw1_3 *kf * t2 * t11 + t57 * t7 / 2.0) + c_t19_tmp * t7 / 2.0);
    t50 = d_Rbw2_3 * kf * t2;
    t57 = d_Rbw2_1 * kf * t2 * 1.4142135623730951;
    c_t19_tmp = d_Rbw2_2 * kf * t2 * 1.4142135623730951;
    b[1] = ((-w1 * ((t50 * t8 + t57 * t4 / 2.0) - c_t19_tmp * t4 / 2.0) + w2 *((t50 * t9 + t57 * t5 / 2.0) + c_t19_tmp * t5 / 2.0)) - w3 * ((t50 *t10 - t57 * t6 / 2.0) + c_t19_tmp * t6 / 2.0)) - w4 * ((-d_Rbw2_3 *kf * t2 * t11 + t57 * t7 / 2.0) + c_t19_tmp * t7 / 2.0);
    t50 = d_Rbw3_3 * kf * t2;
    t57 = d_Rbw3_1 * kf * t2 * 1.4142135623730951;
    c_t19_tmp = d_Rbw3_2 * kf * t2 * 1.4142135623730951;
    b[2] = ((-w1 * ((t50 * t8 + t57 * t4 / 2.0) - c_t19_tmp * t4 / 2.0) + w2 *((t50 * t9 + t57 * t5 / 2.0) + c_t19_tmp * t5 / 2.0)) - w3 * ((t50 *t10 - t57 * t6 / 2.0) + c_t19_tmp * t6 / 2.0)) - w4 * ((-d_Rbw3_3 *kf * t2 * t11 + t57 * t7 / 2.0) + c_t19_tmp * t7 / 2.0);
    b[3] = 0.0;
    b[4] = 0.0;
    b[5] = 0.0;

    /*
        From matlab to Eigen::Matrix, for example:
        A(1) = A(1,1) = 1; 
        A(2) = A(2,1) = 4; 
        A(3) = A(3,1) = 7; 
        A(4) = A(1,2) = 2; 
        A(5) = A(2,2) = 5;
    */
    //Do the inverse operation, avoiding "for" cycles
    matrix_A(0,0)= A[0]; matrix_A(0,1)= A[6];  matrix_A(0,2)= A[12]; matrix_A(0,3)= A[18];
    matrix_A(1,0)= A[1]; matrix_A(1,1)= A[7];  matrix_A(1,2)= A[13]; matrix_A(1,3)= A[19];
    matrix_A(2,0)= A[2]; matrix_A(2,1)= A[8];  matrix_A(2,2)= A[14]; matrix_A(2,3)= A[20];
    matrix_A(3,0)= A[3]; matrix_A(3,1)= A[9];  matrix_A(3,2)= A[15]; matrix_A(3,3)= A[21];
    matrix_A(4,0)= A[4]; matrix_A(4,1)= A[10]; matrix_A(4,2)= A[16]; matrix_A(4,3)= A[22];
    matrix_A(5,0)= A[5]; matrix_A(5,1)= A[11]; matrix_A(5,2)= A[17]; matrix_A(5,3)= A[23];

    matrix_A(0,4)= A[24]; matrix_A(0,5)= A[30]; matrix_A(0,6)= A[36]; matrix_A(0,7)= A[42];
    matrix_A(1,4)= A[25]; matrix_A(1,5)= A[31]; matrix_A(1,6)= A[37]; matrix_A(1,7)= A[43];
    matrix_A(2,4)= A[26]; matrix_A(2,5)= A[32]; matrix_A(2,6)= A[38]; matrix_A(2,7)= A[44];
    matrix_A(3,4)= A[27]; matrix_A(3,5)= A[33]; matrix_A(3,6)= A[39]; matrix_A(3,7)= A[45];
    matrix_A(4,4)= A[28]; matrix_A(4,5)= A[34]; matrix_A(4,6)= A[40]; matrix_A(4,7)= A[46];
    matrix_A(5,4)= A[29]; matrix_A(5,5)= A[35]; matrix_A(5,6)= A[41]; matrix_A(5,7)= A[47];

    vector_B(0) = b[0]; 
    vector_B(1) = b[1];
    vector_B(2) = b[2];
    vector_B(3) = b[3];
    vector_B(4) = b[4];
    vector_B(5) = b[5];
    
}
