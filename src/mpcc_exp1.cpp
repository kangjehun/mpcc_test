#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <derived_object_msgs/ObjectArray.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>

const double m_ego = 1845.0; // [kg] total mass of ego vehicle

typedef struct
{
    double vx_ego;
    double ax_ego;
    double ay_ego;
    double d_ego;
    double delta_ego;
} EgoVehicleState_Exp1;

class mpcc_exp1
{
public:
    mpcc_exp1(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber status_sub_;
    EgoVehicleState_Exp1 state_ego_;
    const char *fmt_str_;
    const char *fmt_str_exp1;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void statusCallback(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr &msg);
};

mpcc_exp1::mpcc_exp1(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), private_nh_(private_nh)
{
    odom_sub_ = nh_.subscribe("/carla/ego_vehicle/odometry", 1, &mpcc_exp1::odomCallback, this);
    imu_sub_ = nh_.subscribe("/carla/ego_vehicle/imu", 1, &mpcc_exp1::imuCallback, this);
    status_sub_ = nh_.subscribe("/carla/ego_vehicle/vehicle_status", 1, &mpcc_exp1::statusCallback, this);
    state_ego_ = {0};
    fmt_str_ =
        "\nvx: %.3f[m/s]\n"
        "d: %.3f\n"
        "ax: %.3f[m/s^2]\n"
        "delta: %.3f";
    fmt_str_exp1 = "%.3f %.3f %.3f\n";
}

void mpcc_exp1::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    state_ego_.vx_ego = msg->twist.twist.linear.x;
}

void mpcc_exp1::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    state_ego_.ax_ego = msg->linear_acceleration.x;
    state_ego_.ay_ego = msg->linear_acceleration.y;
}

void mpcc_exp1::statusCallback(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr &msg)
{
    state_ego_.d_ego = msg->control.throttle - msg->control.brake;
    state_ego_.delta_ego = msg->control.steer;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpcc_exp1");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    mpcc_exp1 mpcc_exp1(nh, private_nh);

    double delta_prev, vx_prev, d_prev, ax_prev;
    double delta_diff, vx_diff, d_diff, ax_diff;

    std::ofstream outfile_;
    std::ostringstream oss;
    outfile_.open("/home/usrg/catkin_ws/src/mpcc_ros/src/test1_data_file_1.txt");
    if (!outfile_.is_open())
    {
        ROS_ERROR("Failed to open file");
    }

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        vx_prev = mpcc_exp1.state_ego_.vx_ego;
        d_prev = mpcc_exp1.state_ego_.d_ego;
        ax_prev = mpcc_exp1.state_ego_.ax_ego;
        ros::spinOnce();
        // Write the values to file with the proper condition
        vx_diff = mpcc_exp1.state_ego_.vx_ego - vx_prev;
        d_diff = mpcc_exp1.state_ego_.d_ego - d_prev;
        ax_diff = mpcc_exp1.state_ego_.ax_ego - ax_prev;
        if (mpcc_exp1.state_ego_.delta_ego >= -0.005 && mpcc_exp1.state_ego_.delta_ego <= 0.005 &&// to ignore centripetal acceleration
            mpcc_exp1.state_ego_.ay_ego >= -1 && mpcc_exp1.state_ego_.ay_ego <= 5 &&// to ignore slip on tires
            mpcc_exp1.state_ego_.vx_ego > 0 // to prevent zero problem
            )
        {
            if (vx_diff >= 0.1 || d_diff >= 0.1 || ax_diff >= 50) // if variation is too small, ignore them to prevent duplicate
            {
                mpcc_exp1.state_ego_.ax_ego *= m_ego; // consider the mass of the vehicle
                oss << std::fixed << std::setprecision(3) << mpcc_exp1.state_ego_.vx_ego << " "
                    << std::fixed << std::setprecision(3) << mpcc_exp1.state_ego_.d_ego << " "
                    << std::fixed << std::setprecision(3) << mpcc_exp1.state_ego_.ax_ego << "\n";
                outfile_ << oss.str();
                ROS_INFO_STREAM("Data written to file: " << oss.str());
            }
        }
        loop_rate.sleep();
    }
    return 0;
}