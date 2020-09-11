#include "ros/ros.h"
#include "math.h"
#include <iosteam>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64MultiArray.h>

void pareto_front(sensor_msgs::CameraInfo info, geometry_msgs::Point p, float motor_resolution, std::vector<bool> & pareto_result, )
{
    //think camera is ideal
    // Intrinsic camera matrix for the raw (distorted) images.
    //    [fx  0 cx]
    //K = [ 0 fy cy]
    //    [ 0  0  1]
    
    foc = info.K[0];
    //camera location where center of the base_link is [X:0,Z:0]
    std::array<float, 2> camera_R = {-0.35,0};
    std::array<float, 2> camera_L = {0.35,0};
    //stereo base line
    B = camera_R[0] - camera_L[0];
    //field of view
    float FOV = 80/180*M_PI;
    


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pareto_front");
    ros::NodeHandle nh;
    ros::Publisher pub_pareto = np.advertise<std_msgs::Float64MultiArray>("/uds/optimized_angle");
    ros::Rate loop_rate(20);
    //radian
    float motor_resolution = 1;

    while(ros::ok())
    {
        //initialization 
        sernsor_msgs::CameraInfo camerainfo;
        geometry_msgs::Point roi_pt;
        std::vector<bool> pareto_result;
        //recieve camera information
        boost::shared_ptr<sernsor_msgs::CameraInfo> sharedPtr_camerainfo;
        sharedPtr_camerainfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/uds/left/camera_info");
        if(sharedPtr_camerainfo == NULL)
            ROS_INFO("No camera infomation recieved");
        else
            camerainfo = *sharedPtr_camerainfo;
        //recieve roi point 
        boost::shard_ptr<geometry_msgs::Point> sharedPtr_pt;
        sharedPtr_pt = ros::topic::waitForMesage<geometry_msgs::Point>("/uds/point2");
        if(sharedPtr_pt == NULL)
            ROS_INFO("No roi point is recieved");
        else
            roi_pt = *sharedPtr_pt;

        pareto_front(camerainfo, roi_pt, motor_resolution, pareto_result);

        
        
    }
}
