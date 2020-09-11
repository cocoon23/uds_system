#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <math.h>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <geometry_msgs/Point.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


geometry_msgs::Point save_point(const sensor_msgs::PointCloud2& msg_org)
{
    geometry_msgs::Point point_result;
	sensor_msgs::PointCloud msg;
    sensor_msgs::convertPointCloud2ToPointCloud(msg_org, msg);
        //printf("%f\n", msg.points[0].x);

        //printf("%d\n", msg.points.size());

        int point_num = msg.points.size();

     	//std::ofstream point_result("/mnt/hgfs/shared_folder/point_result.txt");
        //for (int i = 0; i < point_num; i++){
        //      printf("\t(%f, %f, %f)\n", msg.points[i], msg.points[i], msg.points[i]);
        //}


    double left_camera_y_cdn = 0.0;
    double right_camera_y_cdn = -0.7;
    double z_filter = 0.01;

    double pc_sum_x = 0.0;
    double pc_sum_y = 0.0;
    double pc_sum_z = 0.0;

    double pc_cof_x = 0.0;
    double pc_cof_y = 0.0;
    double pc_cof_z = 0.0;

    for (int i = 0; i < point_num; i++){
            if (msg.points[i].z <= z_filter){
                    continue;
            }

            pc_sum_x += msg.points[i].x;
            pc_sum_y += msg.points[i].y;
            pc_sum_z += msg.points[i].z;
    }

    pc_cof_x = pc_sum_x / point_num;
    pc_cof_y = pc_sum_y / point_num;
    pc_cof_z = pc_sum_z / point_num;

    int object_position_case;
    if (pc_cof_y < left_camera_y_cdn){
            object_position_case = 0;
	}
    else if (pc_cof_y >= left_camera_y_cdn && pc_cof_y < right_camera_y_cdn){
            object_position_case = 1;
    }
    else{
            object_position_case = 2;
    }


        // Find the point that has the widest angle relative to the base (left) camera's forward vector (0, 0, 1)
    double forward_vector_x = 0.0;
    double forward_vector_y = 0.0;
    double forward_vector_z = 1.0;
    double mag_forward_vector = sqrt(pow(forward_vector_x, 2) + pow(forward_vector_y, 2) + pow(forward_vector_z, 2));

    double widest_angle = 0.0;
    int widest_index = -1;
    if (object_position_case == 0 || object_position_case == 2){
        double curr_x = 0.0;
        double curr_y = 0.0;
        double curr_z = 0.0;

        double dot_product = 0.0;
        double mag_curr_point = 0.0;
        double angle = 0.0;
        for (int i = 0; i < point_num; i++){
                curr_x = msg.points[i].x;
                curr_y = msg.points[i].y;
                curr_z = msg.points[i].z;
                if (curr_z <= z_filter){
                       continue;
                }

                dot_product = forward_vector_x * curr_x + forward_vector_y * curr_y + forward_vector_z * curr_z;
                mag_curr_point = sqrt(pow(curr_x, 2) + pow(curr_y, 2) + pow(curr_z, 2));
                angle = std::acos(dot_product/(mag_forward_vector * mag_curr_point));
                if (std::abs(angle) > widest_angle){
                      widest_index = i;
                }
        }

        printf("Case: %d, (%f, %f, %f)\n", object_position_case, msg.points[widest_index].x, msg.points[widest_index].y, msg.points[widest_index].z);
        //point_result << "1 "<<msg.points[widest_index].x<< " " << msg.points[widest_index].y << " " << msg.points[widest_index].z<<std::endl;
        point_result.x = msg.points[widest_index].x;
        point_result.y = msg.points[widest_index].y;
        point_result.z = msg.points[widest_index].z;
        return point_result;
        
	}
    else if (object_position_case == 1){
        printf("Case: %d, (%f, %f, %f)\n", object_position_case, pc_cof_x, pc_cof_y, pc_cof_z);
	    //point_result <<"1 " <<pc_cof_x<< " " << pc_cof_y << " " << pc_cof_z<<std::endl;
        point_result.x=pc_cof_x;
        point_result.y=pc_cof_y;
        point_result.z=pc_cof_z;
        return point_result;
    }
	//point_result.close();
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"roi_point");
    ros::NodeHandle nh;
    ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Point>("uds/points2/roi",1000);
    ros::Rate loop_rate(10);
    geometry_msgs::Point pub_point;
    

    while(ros::ok())
    {
        boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedPtr_pt;
        sensor_msgs::PointCloud2 pt;
        sharedPtr_pt=ros::topic::waitForMessage<sensor_msgs::PointCloud2>("uds/points2");
        if(sharedPtr_pt == NULL)
            ROS_INFO("No point cloud message recieved");
        else
            pt=*sharedPtr_pt;
        pub_point = save_point(pt);
        chatter_pub.publish(pt);
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}


