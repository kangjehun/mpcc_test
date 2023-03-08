#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <tf/transform_datatypes.h>

#include <std_msgs/Bool.h>
#include <ad_challenge_msgs/UAQ_Out.h>

// Constants
const double m_ego = 1806.8;          // [kg] total mass of ego vehicle
const double lr_ego = 2.635;          // [m] rear length of ego vehicle
const double lf_ego = 2.0;            // [m] front length of ego vehicle
const double l_ego = lr_ego + lf_ego; // [m] total length of ego vehicle
const double Iz_ego = 2900.3;         // [kgm^2] Moment of inertia to the center of mass

typedef struct
{
    double vx_ego;
    double vy_ego;
    double ax_ego;
    double ay_ego;
    double r_ego;
    double rdot_ego;
    double d_ego;
    double theta_ego; // steering handle angle
    double delta_ego; // steering angle
} EgoVehicleState;

typedef struct
{
    double Ffy;
    double Fry;
    double alpha_f;
    double alpha_r;
} PacejkaVariable;

typedef struct
{
    double vx;
    double d;
    double Fx;
} FxVariable;

class mpcc_exp2
{
public:
    mpcc_exp2(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    double deg2rad(double deg);
    double rad2deg(double rad);
    double theta2delta(double theta);
    ros::Subscriber carmaker_sub_;
    EgoVehicleState state_ego_;
    const char *fmt_str_debug, *fmt_str_pacejka, *fmt_str_fx;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    void carmakerCallback(const ad_challenge_msgs::UAQ_Out::ConstPtr &msg);
};

mpcc_exp2::mpcc_exp2(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), private_nh_(private_nh)
{
    carmaker_sub_ = nh_.subscribe("/UAQ_Out", 1, &mpcc_exp2::carmakerCallback, this);
    state_ego_ = {0};
    fmt_str_debug =
        "vx, vy: %.3f[m/s], %.3f[m/s]\n"
        "ax, ay: %.3f[m/s^2], %.3f[m/s^2]\n"
        "theta: %.3f[rad], delta: %.3f[rad]\n"
        "theta: %.3f[deg], delta: %.3f[deg]\n"
        "r: %.3f[rad/s], rdot: %.3f[rad/s^2]\n"
        // "r: %.3f[deg/s], rdot: %.3f[deg/s^2]\n"
        "d, theta: %.3f[-1~1], %.3f[0~1]\n";
    fmt_str_pacejka = "Ffy : %.4f[N] Fry: %.4f[N] alpha_f: %.4f[deg] alpha_r: %.4f[deg]\n"; // Ffy, Fry, alpha_f, alpha_r
    fmt_str_fx = "vx: %.4f[m/s] d: %.4f [-] Fx: %.4f[N]\n";                                 // vx, d, Fx
}
// Function to convert degrees to radians
double mpcc_exp2::deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

// Function to convert radians to degrees
double mpcc_exp2::rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}

// Function to convert theta (steering handle angle) [rad] to delta (steering angle) [rad] by using LUT in carmaker with linear interpolation
double mpcc_exp2::theta2delta(double theta)
{
    double delta;
    double steer_ratio; // steering ratio
    return delta;
    if (theta < -180)
    {
        delta = -0.235619449;
    }
    else if (-180.0 <= theta && theta < -90.0)
    {
        steer_ratio = (1 / 18.0) * theta + 85.0;
        steer_ratio *= 0.001;
        delta = steer_ratio * theta;
    }
    else if (-90.0 <= theta && theta < -60.0)
    {
        steer_ratio = (1 / 6.0) * theta + 95.0;
        steer_ratio *= 0.001;
        delta = steer_ratio * theta;
    }
    else if (-60.0 <= theta && theta < -30.0)
    {
        steer_ratio = (1 / 6.0) * theta + 95.0;
        steer_ratio *= 0.001;
        delta = steer_ratio * theta;
    }
    else if (-30.0 <= theta && theta < -20.0)
    {
        steer_ratio = 1 * theta + 120.0;
        steer_ratio *= 0.001;
        delta = steer_ratio * theta;
    }
    else if (-20.0 <= theta && theta <= 20.0)
    {
        steer_ratio = 100.0;
        steer_ratio *= 0.001;
        delta = steer_ratio * theta;
    }
    else if (20.0 < theta && theta <= 30.0)
    {
        steer_ratio = -1 * theta + 120.0;
        steer_ratio *= 0.001;
        delta = steer_ratio * theta;
    }
    else if (30.0 < theta && theta <= 60.0)
    {
        steer_ratio = -(1 / 6.0) * theta + 95.0;
        steer_ratio *= 0.001;
        delta = steer_ratio * theta;
    }
    else if (60.0 < theta && theta <= 90.0)
    {
        steer_ratio = -(1 / 6.0) * theta + 95.0;
        steer_ratio *= 0.001;
        delta = steer_ratio * theta;
    }
    else if (90.0 < theta && theta <= 180.0)
    {
        steer_ratio = -(1 / 18.0) * theta + 85.0;
        steer_ratio *= 0.001;
        delta = steer_ratio * theta;
    }
    else
    {
        delta = -0.235619449;
    }
    return delta;
}

void mpcc_exp2::carmakerCallback(const ad_challenge_msgs::UAQ_Out::ConstPtr &msg)
{
    state_ego_.vx_ego = msg->Car_vx;
    state_ego_.vy_ego = msg->Car_vy;
    state_ego_.ax_ego = msg->Car_ax;
    state_ego_.ax_ego = msg->Car_ay;
    state_ego_.r_ego = msg->Car_YawVel;
    state_ego_.rdot_ego = msg->Car_YawAcc;
    state_ego_.theta_ego = msg->Steer_WhlAng; // steering hanle angle! not steering angle
    state_ego_.delta_ego = this->theta2delta(state_ego_.theta_ego);
    state_ego_.d_ego = msg->VC_Gas - msg->VC_Brake; // Merge Acc and Brake
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpcc_exp2");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    mpcc_exp2 mpcc_exp2(nh, private_nh);
    std::ofstream outfile_1, outfile_2;
    std::ostringstream oss_1, oss_2;
    outfile_1.open("/home/usrg/catkin_ws/src/mpcc_ros/carla_python/pacejka_data_file.txt");
    if (!outfile_1.is_open())
    {
        ROS_ERROR("Failed to open file");
    }
    outfile_2.open("/home/usrg/catkin_ws/src/mpcc_ros/carla_python/fx_data_file.txt");
    if (!outfile_2.is_open())
    {
        ROS_ERROR("Failed to open file");
    }
    double vx = 0, vy = 0;
    double ax = 0, ay = 0;
    double theta = 0, delta = 0;
    double theta_deg = 0, delta_deg = 0;
    double d = 0;
    double r = 0, rdot = 0;
    double m = m_ego;
    double l = l_ego;
    double lf = lf_ego;
    double lr = lr_ego;
    double Iz = Iz_ego;
    PacejkaVariable pacejka_variable = {0};
    FxVariable fx_variable = {0};

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        vx = mpcc_exp2.state_ego_.vx_ego;
        vy = mpcc_exp2.state_ego_.vy_ego;
        ax = mpcc_exp2.state_ego_.ax_ego;
        ay = mpcc_exp2.state_ego_.ay_ego;
        theta = mpcc_exp2.state_ego_.theta_ego;
        theta_deg = mpcc_exp2.rad2deg(theta);
        delta = mpcc_exp2.state_ego_.delta_ego;
        delta_deg = mpcc_exp2.rad2deg(delta);
        d = mpcc_exp2.state_ego_.d_ego;
        r = mpcc_exp2.state_ego_.r_ego;
        rdot = mpcc_exp2.state_ego_.rdot_ego;
        if (vx > 5) // If the vehicle is too slow, the dynamic model will be ill-defined
        {
            pacejka_variable.Ffy = (m * lf * (ay + vx * r) + Iz * rdot) / (l * std::cos(delta));
            pacejka_variable.Fry = (m * lr * (ay + vx * r) - Iz * rdot) / l;
            pacejka_variable.alpha_f = std::atan2(vy + r * lf, vx) - delta;
            pacejka_variable.alpha_r = std::atan2(vy - r * lr, vx);
            fx_variable.vx = vx;
            fx_variable.d = d;
            fx_variable.Fx = m * (ax - vy * r) + pacejka_variable.Ffy * std::sin(delta);
            oss_1 << std::fixed << std::setprecision(3) << pacejka_variable.Ffy << " "
                  << std::fixed << std::setprecision(3) << pacejka_variable.Fry << " "
                  << std::fixed << std::setprecision(3) << pacejka_variable.alpha_f << " "
                  << std::fixed << std::setprecision(3) << pacejka_variable.alpha_r << "\n";
            oss_2 << std::fixed << std::setprecision(3) << fx_variable.vx << " "
                  << std::fixed << std::setprecision(3) << fx_variable.d << " "
                  << std::fixed << std::setprecision(3) << fx_variable.Fx << " ";
            outfile_1 << oss_1.str();
            outfile_2 << oss_2.str();
            printf(mpcc_exp2.fmt_str_debug, vx, vy, ax, ay, theta, delta, theta_deg, delta_deg, r, rdot, d, delta);
            printf(mpcc_exp2.fmt_str_pacejka, pacejka_variable.Ffy, pacejka_variable.Fry,
                   mpcc_exp2.rad2deg(pacejka_variable.alpha_f), mpcc_exp2.rad2deg(pacejka_variable.alpha_r));
            printf(mpcc_exp2.fmt_str_fx, fx_variable.vx, fx_variable.d, fx_variable.Fx);
        }
        // ROS_INFO_STREAM("Data written to file: " << oss_1.str());
        // ROS_INFO_STREAM("Data written to file: " << oss_2.str());
        loop_rate.sleep();
    }
    return 0;
}
// const double Cm1 = 21424.24;
// const double Cm2 = 1990.76;
// const double Cr0 = 5997.38;
// const double Cr2 = -59.35;
// const double m_ego = 1845.0; // [kg] total mass of ego vehicle
// const delta_map = 1.2271;    // map delta (steering handle angle) to steering angle
// const double lr_ego = 2.846; // [m] rear length of ego vehicle
// const double lf_ego = 1.946; // [m] front length of ego vehicle