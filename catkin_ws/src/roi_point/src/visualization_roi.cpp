#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <cmath>

#include <errno.h>

bool check_distance(visualization_msgs::Marker marker, geometry_msgs::Point pt2, float limit)
{
    geometry_msgs::Point pt1 = marker.points.back();
    
    float distance_x = pt1.x - pt2.x;
    float distance_y = pt1.y - pt2.y;
    float distance_z = pt1.z = pt2.y;
    float distance = sqrt(pow(distance_x,2) + pow(distance_y,2) + pow(distance_z,2));

    if(distance < limit)
        return false;
    else
        return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_roi", 10);
    boost::shared_ptr<geometry_msgs::Point const> sharedPtr_roi;
    geometry_msgs::Point temp;
    float limit = 1;
    n.getParam("limit", limit);
    ros::Rate r(30);
    int count = 0;
    while(ros::ok())
    {
        visualization_msgs::Marker points, line_strip, line_list;

        points.header.frame_id="/my_frame";
        points.ns ="roi_point";
        points.action=visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;
        line_list.id = 2;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        points.scale.x = 0.2;
        points.scale.y = 0.2;

        line_strip.scale.x = 0.1;
        line_list.scale.x = 0.1;

        points.color.g = 1.0;
        points.color.a = 1.0;

        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        line_list.color.r = 1.0;
        line_list.color.a = 1.0;
        
        sharedPtr_roi = ros::topic::waitForMessage<geometry_msgs::Point>("/uds/point2/roi");
        if(sharedPtr_roi == NULL)
            ROS_INFO("No /uds/point2/roi message recieved");
        else
        {
            temp = *sharedPtr_roi;

            if(count == 0)
            {
                points.points.push_back(temp);
                line_strip.points.push_back(temp);
                temp.z += 1.0;
                line_list.points.push_back(temp);
            }
            else
            {
                if(check_distance(points, temp, limit))
                {
                    points.points.push_back(temp);
                    line_strip.points.push_back(temp);
                    temp.z += 1.0;
                    line_list.points.push_back(temp);
                }
            }
            count++;
        }
        marker_pub.publish(points);
    
    }
}
