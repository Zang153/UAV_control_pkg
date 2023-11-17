#include <ros/ros.h>

#include <iostream>
#include <Eigen/Eigen>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <px4_command/VectorPayloadtoQuad.h>
#include <px4_command_utils.h>

using namespace std;

ros::Publisher payload_quad_pub;
px4_command::VectorPayloadtoQuad _VectorPtoQ;

int main (int argc, char **argv)
{
    ros::init(argc, argv, "fake_vector_publisher");
    ros::NodeHandle n("~");

    ros::Publisher payload_quad_pub = n.advertise<px4_command::VectorPayloadtoQuad>("/PtoQVector/pose", 10);

    ros::Rate rate(50.0);

    while (ros::ok())
    {
        _VectorPtoQ.position[0] = 0;
        _VectorPtoQ.position[1] = 0;
        _VectorPtoQ.position[2] = -1;
        _VectorPtoQ.velocity[0] = 0;
        _VectorPtoQ.velocity[1] = 0;
        _VectorPtoQ.velocity[2] = 0;
    
        payload_quad_pub.publish(_VectorPtoQ);

        
        rate.sleep();
    }

    return 0;

}
