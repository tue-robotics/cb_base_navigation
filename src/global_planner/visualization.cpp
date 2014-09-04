#include "cb_base_navigation/global_planner/visualization.h"

namespace cb_global_planner {

Visualization::Visualization()
{
    ros::NodeHandle nh("~/visualization");
    goal_positions_marker_pub_ = nh.advertise<visualization_msgs::Marker>("markers/goal_positions_marker",1);
    global_plan_marker_pub_ = nh.advertise<visualization_msgs::Marker>("markers/global_plan_marker",1);
}

void Visualization::publishGlobalPlanMarker(const std::vector<geometry_msgs::PoseStamped>& plan, const std::string& frame) {
    // General properties
    visualization_msgs::Marker line_strip;
    line_strip.scale.x = 0.05;
    line_strip.header.frame_id = frame;
    line_strip.header.stamp = ros::Time::now();
    line_strip.color.a = 1;
    line_strip.color.r = 0;
    line_strip.color.g = 1;
    line_strip.color.b = 1;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;

    // Push back all pnts
    std::vector<geometry_msgs::PoseStamped>::const_iterator it = plan.begin();
    for (; it != plan.end(); ++it) { line_strip.points.push_back(it->pose.position); }

    // Publish plan
    global_plan_marker_pub_.publish(line_strip);
}

void Visualization::publishGoalPositionsMarker(const std::vector<tf::Point>& positions, const std::string& frame)
{
    // General properties
    visualization_msgs::Marker cube_list;
    cube_list.scale.x = 0.05;
    cube_list.scale.y = 0.05;
    cube_list.scale.z = 0.05;
    cube_list.header.frame_id = frame;
    cube_list.header.stamp = ros::Time::now();
    cube_list.color.a = 1;
    cube_list.color.r = 1;
    cube_list.color.g = 0;
    cube_list.color.b = 1;
    cube_list.id = 0;
    cube_list.type = visualization_msgs::Marker::CUBE_LIST;
    cube_list.action = visualization_msgs::Marker::ADD;

    // Push back all pnts
    std::vector<tf::Point>::const_iterator it = positions.begin();
    for (; it != positions.end(); ++it) {
        geometry_msgs::Point p;
        p.x = it->x();
        p.y = it->y();
        cube_list.points.push_back(p);
    }

    // Publish plan
    goal_positions_marker_pub_.publish(cube_list);
}

}
