#ifndef CONTROLLER_CIRCLE_H
#define CONTROLLER_CIRCLE_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>
#include <px4_command_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <px4_command/TrajectoryPoint.h>

using namespace std;
 
class Controller_Circle
{
    public:

        Controller_Circle(void):
        Controller_Circle_nh("~")
        {
            Controller_Circle_nh.param<float>("Controller_Circle/Center_x", circle_center[0], 0.0);
            Controller_Circle_nh.param<float>("Controller_Circle/Center_y", circle_center[1], 0.0);
            Controller_Circle_nh.param<float>("Controller_Circle/Center_z", circle_center[2], 0.8);
            Controller_Circle_nh.param<float>("Controller_Circle/circle_radius", circle_radius, 0.5);
            Controller_Circle_nh.param<float>("Controller_Circle/linear_vel", linear_vel, 0.1);
            Controller_Circle_nh.param<float>("Controller_Circle/direction", direction, 1.0);
        }

        px4_command::TrajectoryPoint Circle_trajectory_generation(float time_from_start);

    private:

        ros::NodeHandle Controller_Circle_nh;

        Eigen::Vector3f circle_center;
        float circle_radius;
        float linear_vel;
        float direction;            // direction == 1 for anti clockwise, direction == -1 for clockwise    

};

px4_command::TrajectoryPoint Controller_Circle::Circle_trajectory_generation(float time_from_start)
{
    px4_command::TrajectoryPoint Circle_trajectory;
    float omega;
    if (circle_radius != 0)
    {
        omega =  direction * fabs(linear_vel / circle_radius);
       
    }
    else
    {
        omega = 0.0;
    }
    const float angle = time_from_start * omega;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);
    
    Circle_trajectory.header.stamp = ros::Time::now();
    Circle_trajectory.time_from_start = time_from_start;
    Circle_trajectory.Sub_mode = 0;

    Circle_trajectory.position_ref[0] = circle_radius * cos_angle + circle_center[0];
    Circle_trajectory.position_ref[1] = circle_radius * sin_angle + circle_center[1];
    Circle_trajectory.position_ref[2] = circle_center[2];

    Circle_trajectory.velocity_ref[0] = - circle_radius * omega * sin_angle;
    Circle_trajectory.velocity_ref[1] = circle_radius * omega * cos_angle; 
    Circle_trajectory.velocity_ref[2] = 0;

    Circle_trajectory.acceleration_ref[0] = -circle_radius * pow(omega,2.0) * cos_angle; 
    Circle_trajectory.acceleration_ref[1] = -circle_radius * pow(omega,2.0) * sin_angle;
    Circle_trajectory.acceleration_ref[2] = 0;

    Circle_trajectory.yaw_ref = 0;
    
    return Circle_trajectory;
}

#endif
