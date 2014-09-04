#include "cb_base_navigation/local_planner/visualization.h"

namespace cb_local_planner {

Visualization::Visualization()
{
    ros::NodeHandle nh("~/visualization");
    goal_pose_marker_pub_ = nh.advertise<visualization_msgs::Marker>("markers/goal_pose_marker",1);
}

void Visualization::publishGoalPoseMarker(const geometry_msgs::PoseStamped& goal, const std::string& frame) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.scale.x = .8;
    marker.scale.y = .1;
    marker.scale.z = .1;
    marker.color.a = 1;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose = goal.pose;
    marker.action = visualization_msgs::Marker::ADD;
    goal_pose_marker_pub_.publish(marker);
}

}
