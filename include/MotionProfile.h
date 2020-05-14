///////////////////////////////////////////////////////////////////////////////
//////////////////////////MOTION PROFILE///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#include<numeric>
#include<math.h>

class MotionProfile
{
    public:
    
    // MotionProfile
    // Generator of a smoth trajectory with an assigned max velocity, 
    // acceleration and jerk

    MotionProfile();
    
    //Set trajectory parameters
    // Parameters:
    //  double pi       Initial position
    //  double pf       Final position
    //  double vel_max  Max/Cruise velocity
    //  double acc_max  Max acceleration
    //  double jerk_max Max jerk
    void setParam(
        const double pos_i,
        const double pos_f,
        const double vel_max,
        const double acc_max,
        const double jerk_max);

    // Trajectory duration
    double Duration() { return t7; };
    
    // Compute the trajectory at an assigned time (t0 = 0)
    // Input:
    //  - t     Time at which compute the trajectory data
    // Output:
    //  - p     Position at time t
    //  - v     Velocity at time t
    //  - a     Acceleration at time t
    //  - j     Jerk at time t
    // Return: the computed position p at time t
    void Compute( double t, double &p, double &v, double &a, double &j);
    
    // Compute the trajectory at an assigned time
    // Input:
    //  - t     Time at which compute the trajectory data
    //  - t0    Starting trajectory time
    // Output:
    //  - p     Position at time t
    //  - v     Velocity at time t
    //  - a     Acceleration at time t
    //  - j     Jerk at time t
    // Return: the computed position p at time t
    void Compute(double t, double t0, double &p, double &v, double &a, double &j){
        Compute(t-t0,p,v,a,j);
    }
    
    private:

    double pi;
    double pf;
    double v_max;
    double a_max;
    double j_max;    
    double s;
    double a_max2;
    double va;
    double sa;
    double sv;
    double tj;
    double ta;
    double tv;

    double a_max_div_j_max;
    double sqrt_v_max_div_j_max;

    double t1, t2, t3, t4, t5, t6, t7;
    double a1, v1, p1;
    double a2, v2, p2;
    double v3, p3;
    double v4, p4;
    double a5, v5, p5;
    double a6, v6, p6;
    
};