#include "ros/ros.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelConfiguration.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_control");
    ros::NodeHandle nh;
    std::string model_name = "uds";
    ros::ServiceClient getmodelstate = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState gms;
    gms.request.model_name = model_name;
    if(getmodelstate.call(gms))
    {
        ROS_INFO("X = %1f", gms.response.pose.position.x);
    }
    else
        ROS_INFO("FAILED");
}



