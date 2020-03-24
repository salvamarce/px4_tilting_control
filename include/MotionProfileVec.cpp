#include "MotionProfileVec.h"

MotionProfileVec::MotionProfileVec(
                    Eigen::Vector3d& pos_i,
                    Eigen::Vector3d& pos_f,
                    Eigen::Vector3d& vel_max,
                    Eigen::Vector3d& acc_max,
                    Eigen::Vector3d& jerk_max
                )
{
    pi = pos_i;
    pf = pos_f;
    v_max = vel_max;
    a_max = acc_max;
    j_max = jerk_max;
}

void MotionProfileVec::Compute(
                        double t,  
                        Eigen::Vector3d& p, 
                        Eigen::Vector3d& v, 
                        Eigen::Vector3d& a, 
                        Eigen::Vector3d& j
                    )
{
    MotionProfile p1(pi(0), pf(0), v_max(0), a_max(0), j_max(0));
    MotionProfile p2(pi(1), pf(1), v_max(1), a_max(1), j_max(1));
    MotionProfile p3(pi(2), pf(2), v_max(2), a_max(2), j_max(2));

    if(p1.Duration() >= p2.Duration() && p1.Duration() >= p3.Duration())
        tf_max = p1.Duration();
    else if(p2.Duration() >= p1.Duration() && p2.Duration() >= p3.Duration())
        tf_max = p2.Duration();
    else if(p3.Duration() >= p1.Duration() && p3.Duration() >= p1.Duration()) 
        tf_max = p3.Duration();

    p1.Compute(t,p(0),v(0),a(0),j(0));
    p2.Compute(t,p(1),v(1),a(1),j(1));
    p3.Compute(t,p(2),v(2),a(2),j(2));

}