#include "MotionProfile.h"


// MotionProfile
// Generator of a smoth trajectory with an assigned max velocity, 
// acceleration and jerk
// Parameters:
//  double pi       Initial position
//  double pf       Final position
//  double vel_max  Max/Cruise velocity
//  double acc_max  Max acceleration
//  double jerk_max Max jerk
MotionProfile::MotionProfile(){
    pi= 0;
    pf= 0;
    v_max = 0;
    a_max = 0;
    j_max = 0;    
    s = 0;
    a_max2 = 0;
    va = 0;
    sa = 0;
    sv = 0;
    tj = 0;
    ta = 0;
    tv = 0;
    a_max_div_j_max = 0;
    sqrt_v_max_div_j_max = 0;
    t1 = t2 = t3 = t4 = t5 = t6 = t7 = 0;
    a1 = v1 =  p1 = 0;
    a2 = v2 =  p2 = 0;
    v3 = p3 = 0; 
    v4 = p4 = 0;
    a5 = v5 =  p5 = 0;
    a6 = v6 =  p6 = 0;
}

void MotionProfile::setParam(double pos_i,double pos_f,double vel_max,double acc_max,double jerk_max)
{
    pi = pos_i;
    pf = pos_f;
    v_max = vel_max;
    a_max = acc_max;
    j_max = jerk_max;
    
    if(pi > pf){
        v_max = -v_max;
        a_max = -a_max;
        j_max = -j_max;
    }

    s = pf-pi;
    a_max2 = a_max*a_max;
    va = fabs(a_max2/j_max);
    sa = fabs(2.0*va*a_max/j_max);

    a_max_div_j_max = fabs(a_max/j_max);
    sqrt_v_max_div_j_max = sqrt(v_max/j_max);

    if (v_max*j_max < a_max2)
        sv = fabs(v_max*2.0*sqrt_v_max_div_j_max);
    else
        sv = fabs(v_max*(v_max/a_max+a_max_div_j_max));

    if (v_max <= va && s >= sa) {
        tj = sqrt_v_max_div_j_max;
        ta = tj;
        tv = s/v_max;
    } else if (v_max >= va && s <= sa) {
        tj = cbrt(s/(2.0*j_max));
        ta = tj;
        tv = 2.0*tj;
    } else if (v_max <= va && s <= sa) {
        if (s >= sv) {
            tj = sqrt_v_max_div_j_max;
            ta = tj;
            tv = s/v_max;
        } else {
            tj = cbrt(s/(2.0*j_max));
            ta = tj;
            tv = 2.0*tj;
        }
    } else if (v_max >= va && s >= sa) {
        tj = a_max_div_j_max;
        if (s >= sv) {
           ta = v_max/a_max;
           tv = s/v_max;
        } else {
           ta = 0.5*(sqrt((4.0*s+a_max*a_max_div_j_max*a_max_div_j_max)/a_max) - a_max_div_j_max);
           tv = ta+tj;
        }
    }
    //Time intervals
    t1 = tj;
    t2 = ta;
    t3 = ta+tj;
    t4 = tv;
    t5 = tv+tj;
    t6 = tv+ta;
    t7 = tv+tj+ta;

    a1 = j_max*t1;
    v1 = a1*t1/2.0;
    p1 = pi + v1*t1/3.0;
    
    double dt;
    dt = t2-t1;
    a2 = a1;
    v2 = v1 + a1*dt;
    p2 = p1 + (v1+a1/2.0*dt)*dt;

    dt = t3-t2;
    v3 = v2+(a2-j_max/2.0*dt)*dt;
    p3 = p2+(v2+(a2-j_max/3.0*dt)/2.0*dt)*dt;
    
    v4 = v3;
    p4 = p3 + v3*(t4-t3);

    dt = t5-t4;
    a5 = -j_max*dt;
    v5 = v4-j_max/2.0*dt*dt;
    p5 = p4+(v4-j_max/6.0*dt*dt)*dt;

    dt = t6-t5;
    a6 = a5;
    v6 = v5-a_max*dt;
    p6 = p5+(v5+a5/2.0*dt)*dt;

}


// Compute the trajectory at an assigned time (t0 = 0)
// Input:
//  - t     Time at which compute the trajectory data
// Output:
//  - p     Position at time t
//  - v     Velocity at time t
//  - a     Acceleration at time t
//  - j     Jerk at time t
// Return: the computed position p at time t
void MotionProfile::Compute(double t, double &p, double &v, double &a, double &j)
{
    double dt;
    if (t < 0) {
        j = 0;
        a = 0;
        v = 0;
        p = pi;
    } else if (t < t1) {
        j = j_max;
        a = j_max*t;
        v = a/2.0*t;
        p = pi + v*t/3.0;
    } else {
        if (t < t2) {
            dt = t-t1;
            j = 0;
            a = a1;
            v = v1 + a1*dt;
            p = p1 + (v1+a1/2.0*dt)*dt;
        } else {
            if (t < t3) {
                dt = t-t2;
                j = -j_max;
                a = a2-j_max*dt;
                v = v2+(a2-j_max/2.0*dt)*dt;
                p = p2+(v2+(a2-j_max/3.0*dt)/2.0*dt)*dt;
            } else {
                if (t < t4) {
                    j = 0;
                    a = 0;
                    v = v3;
                    p = p3 + v3*(t-t3);
                } else {
                    if (t < t5) {
                        dt = t-t4;
                        j = -j_max;
                        a = -j_max*dt;
                        v = v4-j_max/2.0*dt*dt;
                        p = p4+(v4-j_max/6.0*dt*dt)*dt;
                    } else {
                        if (t < t6) {
                            dt = t-t5;
                            j = 0;
                            a = a5;
                            v = v5-a_max*dt;
                            p = p5+(v5+a5/2.0*dt)*dt;
                        } else {
                            if (t < t7) {
                                dt = t-t6;
                                j = j_max;
                                a = a6+j_max*dt;
                                v = v6+(a6+j_max/2.0*dt)*dt;
                                p = p6+(v6+(a6+j_max/3.0*dt)/2.0*dt)*dt;
                            } else {
                                j = 0;
                                a = 0;
                                v = 0;
                                p = pf;
                            }
                        }
                    }
                }
            }
        }
    }
}    




