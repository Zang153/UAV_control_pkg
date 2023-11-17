//Wednesday10-20 v1.0

#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <px4_command/VectorPayloadtoQuad.h>
#include <px4_command_utils.h>

using namespace std;


Eigen::Vector3d pos_drone_opt;
Eigen::Vector3d pos_payload_opt;
Eigen::Vector3d dronetopayload;
Eigen::Vector3d Z_unitvector;
Eigen::Vector3d velocity;

ros::Publisher payload_quad_pub;
px4_command::VectorPayloadtoQuad _VectorPtoQ;
px4_command::VectorPayloadtoQuad _Last_VectorPtoQ;

float angle_Z;
float angle_X;
float distan;
float cur_time;

void printf_info();

void quad_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped quad_pose;

    quad_pose = *msg;
    pos_drone_opt[0] = quad_pose.pose.position.x;
    pos_drone_opt[1] = quad_pose.pose.position.y;
    pos_drone_opt[2] = quad_pose.pose.position.z;
}

void payload_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseStamped payload_pose;

    payload_pose = *msg;
    pos_payload_opt[0] = payload_pose.pose.position.x;
    pos_payload_opt[1] = payload_pose.pose.position.y;
    pos_payload_opt[2] = payload_pose.pose.position.z;
}
/*
void PtoQVector_pub();
{
    geometry_msgs::PoseStamped PtoQVector;

    PtoQVector.pose.position.x = dronetopayload[0];
    PtoQVector.pose.position.y = dronetopayload[1];
    PtoQVector.pose.position.z = dronetopayload[2];

    PtoQVector.header.stamp = ros::Time::now();
    payload_quad_pub.publish(PtoQVector);
}
*/

int main (int argc, char **argv)
{
    ros::init(argc, argv, "payload_quad_caculator");
    ros::NodeHandle n("~");

    ros::Subscriber quad_sub = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/quad/pose", 10, quad_cb);

    ros::Subscriber payload_sub = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/payload/pose", 10, payload_cb);

    ros::Publisher payload_quad_pub = n.advertise<px4_command::VectorPayloadtoQuad>("/PtoQVector/pose", 10);

    ros::Rate rate(50.0);

    ros::Time begin_time = ros::Time::now();
    float last_time = px4_command_utils::get_time_in_sec(begin_time);
    float dt = 0;

    float r2d = 180.0/3.14;

    for (int i=0; i<50; i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    for (int i=0; i<3; i++)
    {
        _Last_VectorPtoQ.position[i] = dronetopayload[i];
    }


    while (ros::ok())
    {
        cur_time = px4_command_utils::get_time_in_sec(begin_time);
        dt = cur_time - last_time;
        dt = constrain_function2(dt, 0.01, 0.03);
        last_time = cur_time;

        for (int i=0; i<3; i++)
        {
            _VectorPtoQ.velocity[i] = (_VectorPtoQ.position[i] - _Last_VectorPtoQ.position[i]) / dt;
	    _Last_VectorPtoQ.position[i] = _VectorPtoQ.position[i];
            velocity[i] = _VectorPtoQ.velocity[i];
        }
        ros::spinOnce();
       


        for (int i=0; i<3; i++)
        {
            dronetopayload[i] = pos_drone_opt[i] - pos_payload_opt[i];

            _VectorPtoQ.position[i] = dronetopayload[i];
        }

        distan = sqrt(dronetopayload[0]*dronetopayload[0]+dronetopayload[1]*dronetopayload[1]+dronetopayload[2]*dronetopayload[2]);

        angle_Z = atan(sqrt(dronetopayload[0]*dronetopayload[0]+dronetopayload[1]*dronetopayload[1])/dronetopayload[2]) * r2d;

        if (dronetopayload[0] > 0)
        {
            if (dronetopayload[1] > 0)
            {
                angle_X = atan(dronetopayload[1]/dronetopayload[0]) * r2d;
            }
            else
            {
                angle_X = -atan(-dronetopayload[1]/dronetopayload[0]) * r2d;
            }
        }
        else
        {
            if (dronetopayload[1] > 0)
            {
                angle_X = 180 - atan(-dronetopayload[1]/dronetopayload[0]) * r2d;
            }
            else
            {
                angle_X = atan(dronetopayload[1]/dronetopayload[0]) * r2d - 180;
            }
        }

        _VectorPtoQ.header.stamp = ros::Time::now();

        payload_quad_pub.publish(_VectorPtoQ);

        //PtoQVector_pub();

        printf_info();
        rate.sleep();
    }
   
   
    return 0;
}

void printf_info()
{
    cout << ">>>>>>>>>>>>>>>>>>>>drone_pos to payload_pos[vector]<<<<<<<<<<<<<<<<<<<<" << endl;

    cout.setf(ios::fixed);

    cout<<setprecision(2);

    cout.setf(ios::left);

    cout.setf(ios::showpoint);

    cout.setf(ios::showpos);

   
    cout << ">>>>>>>>>>>>>>>>>>>>>>>>vector info [world frame]<<<<<<<<<<<<<<<<<<<<<<<" << endl;

    cout <<"pos_opt[X Y Z]: " << dronetopayload[0] << " [m] " << dronetopayload[1] << " [m] " << dronetopayload[2] << " [m] " << endl;

    cout <<"velocity[x y z]:" << velocity[0] << "[m/s]" << velocity[1] << "[m/s]" << velocity[2] << "[m/s]" << endl;
    //cout <<"distance between drone and payload is " << distance << endl;

    cout <<"Angle(z,n) = " << angle_Z << "\t" << "Angle(x,n)" << angle_X << endl;

    cout << endl;
    cout << endl;
    cout << endl;

}
