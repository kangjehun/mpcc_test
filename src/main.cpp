// /*
//  * mpcc_ros
//  * MPCC for ROS1
//  * 2022.03.05
//  * KAIST USRG KangJehun
//  */

// // Copyright 2019 Alexander Liniger

// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at

// //     http://www.apache.org/licenses/LICENSE-2.0

// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.
// ///////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////

// #include <ros/ros.h>
// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <string>
// #include <carla_msgs/CarlaEgoVehicleControl.h>
// #include <carla_msgs/CarlaEgoVehicleStatus.h>
// #include <derived_object_msgs/ObjectArray.h>
// // ROS msgs
// #include <tf/transform_datatypes.h>
// #include <nav_msgs/Odometry.h>
// #include <sensor_msgs/Imu.h>
// #include <std_msgs/Bool.h>
// // Other headers

// // rostopic pub /carla/ego_vehicle/vehicle_control_cmd carla_msgs/CarlaEgoVehicleControl

// // Constant for Tesla Model 3
// // const double m_ego = 1845.0; // [kg] total mass of ego vehicle
// // const double l_ego = 4.792; // [m] total length of ego vehicle
// // const double lr_ego = 2.846; // [m] rear length of ego vehicle
// // const double lf_ego = 1.946; // [m] front length of ego vehicle
// // const double Iz_ego = 1868.59; // [kg/m^2] moment of inertia for z axis

// typedef struct
// {
//   // 10 States, 2 inputs (current)
//   double X_ego;     // position x in the global coordinate system
//   double Y_ego;     // position y in the global coordinate system
//   double Psi_ego;   // heading (Yaw) angle Psi to the reference axis (x axis of the global cordinate)
//   double vx_ego;    // longitudinal velocity
//   double vy_ego;    // latitudinal velocity
//   double r_ego;     // yaw rate
//   double ax_ego;    // longitudinal acc
//   double ay_ego;    // latitudinal acc
//   double s_ego;     // progress along the reference path
//   double vs_ego;    // delta s
//   double d_ego;     // driver command [-1, 1]
//   double delta_ego; // steering command [-1, 1], -35'~+35', -1.2217rad ~ +1.2217rad
//   // 2 inputs calculated from mpcc
//   double d_mpcc;     // driver command [-1, 1]
//   double delta_mpcc; // steering command [-1, 1], -35'~+35', -1.2217rad ~ +1.2217rad
// } VehicleState;

// typedef struct
// {
//   double x;
//   double y;
//   double z;
// } Waypoint;

// std::vector<Waypoint> waypoints;
// VehicleState state = {0};

// // Callback functions
// void objectCallback(const derived_object_msgs::ObjectArray::ConstPtr &msg)
// {
//   for (const auto &object : msg->objects)
//   {
//     if (object.object_classified && object.classification == derived_object_msgs::Object::CLASSIFICATION_CAR && object.id == 137)
//     {
//       state.X_ego = object.pose.position.x;
//       state.Y_ego = object.pose.position.y;
//     }
//   }
// }

// void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//   // Get the x and y position of the ego vehicle from the message
//   state.vx_ego = msg->twist.twist.linear.x;
//   state.vy_ego = msg->twist.twist.linear.y;
// }

// void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
// {
//   // Get the linear acceleration in x and y direction from the message
//   state.ax_ego = msg->linear_acceleration.x;
//   state.ay_ego = msg->linear_acceleration.y;
//   // Get the angular velocity in z direction from the message
//   state.r_ego = msg->angular_velocity.z;
//   // Get the yaw angle from the message
//   state.Psi_ego = tf::getYaw(msg->orientation);
// }

// void statusCallback(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr &msg)
// {
//   state.d_ego = msg->control.throttle - msg->control.brake;
//   state.delta_ego = msg->control.steer;
// }

// int main(int argc, char **argv)
// {
//   // ROS INITIALIZATION
//   ros::init(argc, argv, "mpcc_ros");
//   ros::NodeHandle nh;
//   ros::NodeHandle private_nh("~");

//   // Load waypoint file
//   std::ifstream infile("/home/usrg/catkin_ws/src/mpcc_ros/waypoint/waypoint_Town07_Opt.txt"); // Path for the waypoint file
//   std::string line;
//   int index;
//   Waypoint waypoint;
//   while (std::getline(infile, line))
//   {
//     std::istringstream iss(line);
//     std::string token;

//     iss >> token >> index >> token >> waypoint.x >> token >> waypoint.y >> token >> waypoint.z;
//     waypoints.push_back(waypoint);
//     // just for debugging
//     // std::cout << index << " " << waypoint.x << " " << waypoint.y << " " << waypoint.z << " " << std::endl;
//   }
//   infile.close();

//   // PUB, SUB
//   // create a publisher for the vehicle control commands
//   ros::Publisher control_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 1);
//   ros::Publisher enable_autopilot_pub = nh.advertise<std_msgs::Bool>("/carla/ego_vehicle/enable_autopilot", 1);
//   // ros::Publisher manual_override_pub = nh.advertise<std_msgs::Bool>("/carla/ego_vehicle/vehicle_control_manual_override", 1);
//   ros::Subscriber vehicle_state_sub = nh.subscribe<derived_object_msgs::ObjectArray>("/carla/objects", 1, objectCallback);
//   ros::Subscriber odom_sub = nh.subscribe("/carla/ego_vehicle/odometry", 1, odomCallback);
//   ros::Subscriber imu_sub = nh.subscribe("/carla/ego_vehicle/imu", 1, imuCallback);
//   ros::Subscriber status_sub = nh.subscribe("/carla/ego_vehicle/vehicle_status", 1, statusCallback);
//   // MSG
//   // carla_msgs::CarlaEgoVehicleControl control_msg;
//   std_msgs::Bool enable_autopilot_msg;
//   std_msgs::Bool manual_override_msg;

//   // Define a format string for printing the variables
//   const char *fmt_str =
//       "X, Y: %.3f[m], %.3f[m]\n"
//       "Psi(Yaw Angle): %.3f[rad]\n"
//       "vx, vy: %.3f[m/s], %.3f[m/s]\n"
//       "r: %.3f[rad/s]\n"
//       "ax, ay: %.3f[m/s^2], %.3f[m/s^2]\n"
//       "s: %.3f[m]\n"
//       "vs: %.3f[m/s]\n"
//       "d, delta: %.3f, %.3f\n"
//       "d_mpcc, delta_mpcc: %.3f, %.3f\n";

//   ros::Rate loop_rate(10);
//   // double time = 0.0;
//   while (ros::ok())
//   {
//     // Set control inputs calculated from the MPCC
//     // control_msg.throttle = 0.5 * (1.0+sin(time));
//     // control_msg.steer = 0.1 * sin(time) ;
//     // control_msg.brake = 0.0;
//     // time += 0.001;

//     // Print the variables using the format string and struct members
//     ROS_INFO(fmt_str,
//              state.X_ego, state.Y_ego, state.Psi_ego, state.vx_ego, state.vy_ego, state.r_ego,
//              state.ax_ego, state.ay_ego, state.s_ego, state.vs_ego, state.d_ego, state.delta_ego,
//              state.d_mpcc, state.delta_mpcc);

//     // publish
//     // enable_autopilot_pub.publish(enable_autopilot_msg);
//     // co

// const double Cm1 = 21424.24;
// const double Cm2 = 1990.76;
// const double Cr0 = 5997.38;
// const double Cr2 = -59.35;
// const double m_ego = 1845.0; // [kg] total mass of ego vehicle
// const delta_map = 1.2271;    // map delta (steering handle angle) to steering angle
// const double lr_ego = 2.846; // [m] rear length of ego vehicle
// const double lf_ego = 1.946; // [m] front length of ego vehiclentrol_pub.publish(control_msg);
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
//   return 0;
// }

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>

#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <ad_challenge_msgs/UAQ_Out.h>
#include <ad_challenge_msgs/Control_Signal.h>

// Constants
const double m_ego = 1806.8;          // [kg] total mass of ego vehicle
const double lr_ego = 2.635;          // [m] rear length of ego vehicle
const double lf_ego = 2.0;            // [m] front length of ego vehicle
const double l_ego = lr_ego + lf_ego; // [m] total length of ego vehicle
const double Iz_ego = 2900.3;         // [kgm^2] Moment of inertia to the center of mass

typedef struct
{
    double X_ego;     // Global X position
    double Y_ego;     // Global Y position
    double Psi_ego;   // Yaw angle
    double vx_ego;    // Longitudinal velocity
    double vy_ego;    // Latitudinal velocity
    double ax_ego;    // Longitudinal acceleration
    double ay_ego;    // Latitudinal acceleration
    double r_ego;     // Yaw rate
    double rdot_ego;  // Yaw acc
    double d_ego;     // Drive Command
    double theta_ego; // steering handle angle
    double delta_ego; // steering angle
    double s_ego;     // process
    double vs_ego;    // vehicle velocity along the reference path
} VehicleState;

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

class mpcc_ros
{
public:
    mpcc_ros(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
    double deg2rad(double deg);
    double rad2deg(double rad);
    double theta2delta(double theta);
    ros::Subscriber carmaker_sub_;
    ros::Publisher control_pub_;
    VehicleState vehicle_state_;
    ad_challenge_msgs::Control_Signal control_msg;
    const char *fmt_str_debug, *fmt_str_pacejka, *fmt_str_fx;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    void carmakerCallback(const ad_challenge_msgs::UAQ_Out::ConstPtr &msg);
};

mpcc_ros::mpcc_ros(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : nh_(nh), private_nh_(private_nh)
{
    carmaker_sub_ = nh_.subscribe("/UAQ_Out", 1, &mpcc_ros::carmakerCallback, this);
    control_pub_ = nh.advertise<ad_challenge_msgs::Control_Signal>("/Control_Signal", 1);
    // Initialize Vehicle State
    vehicle_state_ = {0};
    // Initialize Control msg
    control_msg.header.stamp = ros::Time::now();
    control_msg.steerangle = 0; // deg -270 ~ 270
    control_msg.brake = 0;      // 0: no brake, 1: full brake
    control_msg.gas = 0.5;      // 0: no accel pedal, 1: full accel pedal
    control_msg.gear = 1;       // 1: drive, 0: neutral, -1: rear, -9: parking
    fmt_str_debug =
        "X, Y: %.3f[m], %.3f[m]\n"
        "Psi: %.3f[rad]\n"
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
double mpcc_ros::deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}

// Function to convert radians to degrees
double mpcc_ros::rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}

// Function to convert theta (steering handle angle) [rad] to delta (steering angle) [rad] by using LUT in carmaker with linear interpolation
double mpcc_ros::theta2delta(double theta)
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

void mpcc_ros::carmakerCallback(const ad_challenge_msgs::UAQ_Out::ConstPtr &msg)
{
    vehicle_state_.X_ego = 0;   // [CHECK]
    vehicle_state_.Y_ego = 0;   // [CHECK]
    vehicle_state_.Psi_ego = 0; // [CHECK]
    vehicle_state_.vx_ego = msg->Car_vx;
    vehicle_state_.vy_ego = msg->Car_vy;
    vehicle_state_.ax_ego = msg->Car_ax;
    vehicle_state_.ax_ego = msg->Car_ay;
    vehicle_state_.r_ego = msg->Car_YawVel;
    vehicle_state_.rdot_ego = msg->Car_YawAcc;
    vehicle_state_.theta_ego = msg->Steer_WhlAng; // steering hanle angle!
    vehicle_state_.delta_ego = this->theta2delta(vehicle_state_.theta_ego);
    vehicle_state_.d_ego = msg->VC_Gas - msg->VC_Brake; // Merge Acc and Brake
    vehicle_state_.s_ego = 0;                           // [Check]
    vehicle_state_.vs_ego = 0;                          // [Check]
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpcc_ros");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    mpcc_ros mpcc_ros(nh, private_nh);

    ros::Time prev_time = ros::Time::now();

    double X = 0, Y = 0, Psi = 0;
    double vx = 0, vy = 0;
    double ax = 0, ay = 0;
    double theta = 0, delta = 0;
    double theta_deg = 0, delta_deg = 0;
    double d = 0;
    double r = 0, rdot = 0;
    double s = 0, vs = 0;
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
        X = mpcc_ros.vehicle_state_.X_ego;
        Y = mpcc_ros.vehicle_state_.Y_ego;
        Psi = mpcc_ros.vehicle_state_.Psi_ego;
        vx = mpcc_ros.vehicle_state_.vx_ego;
        vy = mpcc_ros.vehicle_state_.vy_ego;
        ax = mpcc_ros.vehicle_state_.ax_ego;
        ay = mpcc_ros.vehicle_state_.ay_ego;
        theta = mpcc_ros.vehicle_state_.theta_ego;
        theta_deg = mpcc_ros.rad2deg(theta);
        delta = mpcc_ros.vehicle_state_.delta_ego;
        delta_deg = mpcc_ros.rad2deg(delta);
        d = mpcc_ros.vehicle_state_.d_ego;
        r = mpcc_ros.vehicle_state_.r_ego;
        rdot = mpcc_ros.vehicle_state_.rdot_ego;
        s = mpcc_ros.vehicle_state_.s_ego;
        vs = mpcc_ros.vehicle_state_.vs_ego;

        // Print out values for debug (1Hz)
        if ((ros::Time::now() - prev_time).toSec() >= 1.0)
        {
            pacejka_variable.Ffy = (m * lf * (ay + vx * r) + Iz * rdot) / (l * std::cos(delta));
            pacejka_variable.Fry = (m * lr * (ay + vx * r) - Iz * rdot) / l;
            pacejka_variable.alpha_f = std::atan2(vy + r * lf, vx) - delta;
            pacejka_variable.alpha_r = std::atan2(vy - r * lr, vx);
            fx_variable.vx = vx;
            fx_variable.d = d;
            fx_variable.Fx = m * (ax - vy * r) + pacejka_variable.Ffy * std::sin(delta);
            printf(mpcc_ros.fmt_str_debug, X, Y, Psi, vx, vy, ax, ay, theta, delta, theta_deg, delta_deg, r, rdot, d, theta);
            printf(mpcc_ros.fmt_str_pacejka, pacejka_variable.Ffy, pacejka_variable.Fry,
                   mpcc_ros.rad2deg(pacejka_variable.alpha_f), mpcc_ros.rad2deg(pacejka_variable.alpha_r));
            printf(mpcc_ros.fmt_str_fx, fx_variable.vx, fx_variable.d, fx_variable.Fx);
            prev_time = ros::Time::now();
        }
        loop_rate.sleep();
    }
    return 0;
}
