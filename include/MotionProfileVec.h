/*------

Motion profile variant with vector variables

------ */

#include "MotionProfile.h"
#include "Eigen/Dense"

class MotionProfileVec{

    public:
        MotionProfileVec(
            Eigen::Vector3d& pos_i,
            Eigen::Vector3d& pos_f,
            Eigen::Vector3d& vel_max,
            Eigen::Vector3d& acc_max,
            Eigen::Vector3d& jerk_max
        );

        double Duration() {return tf_max;}

        void Compute(
            double t,  
            Eigen::Vector3d& p, 
            Eigen::Vector3d& v, 
            Eigen::Vector3d& a, 
            Eigen::Vector3d& j
        );

        void Compute(
            double t, 
            double t0, 
            Eigen::Vector3d& p, 
            Eigen::Vector3d& v, 
            Eigen::Vector3d& a, 
            Eigen::Vector3d& j
        ){ Compute(t-t0,p,v,a,j); }

    private:
        Eigen::Vector3f pi;
        Eigen::Vector3f pf;
        Eigen::Vector3f v_max;
        Eigen::Vector3f a_max;
        Eigen::Vector3f j_max;

        double tf_max;
};

