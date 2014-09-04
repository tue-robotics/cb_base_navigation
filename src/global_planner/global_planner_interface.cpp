#include "cb_base_navigation/global_planner/global_planner_interface.h"

namespace cb_global_planner {

GlobalPlannerInterface::~GlobalPlannerInterface()
{
}

GlobalPlannerInterface::GlobalPlannerInterface(costmap_2d::Costmap2DROS& costmap) :
    gp_loader_("cb_base_navigation", "cb_global_planner::GlobalPlannerPlugin"),
    tf_(new tf::TransformListener(ros::Duration(10))),
    costmap_(costmap)
{
    ros::NodeHandle nh("~");

    // Parameter setup
    std::string global_planner;
    nh.param("global_planner", global_planner, std::string("cb_global_planner::AStarPlannerGPP"));
    nh.param("robot_base_frame", robot_base_frame_, std::string("/amigo/base_link"));;
    nh.param("global_frame", global_frame_, std::string("/map"));

    // Initialize the global planner
    try {
        if(!gp_loader_.isClassAvailable(global_planner)){
            std::vector<std::string> classes = gp_loader_.getDeclaredClasses();
            for(unsigned int i = 0; i < classes.size(); ++i){
                if(global_planner == gp_loader_.getName(classes[i])){
                    global_planner = classes[i]; break;
                }
            }
        }
        global_planner_ = gp_loader_.createInstance(global_planner);
        global_planner_->initialize(gp_loader_.getName(global_planner), tf_, &costmap_);
    } catch (const pluginlib::PluginlibException& ex)
    {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
        exit(0);
    }

    // Register the ROS Service Servers
    get_plan_srv_   = nh.advertiseService("get_plan_srv",   &GlobalPlannerInterface::getPlan,   this);
    check_plan_srv_ = nh.advertiseService("check_plan_srv", &GlobalPlannerInterface::checkPlan, this);

    // Pose callback and plan pub
    pose_sub_ = nh.subscribe("/move_base_simple/goal", 1, &GlobalPlannerInterface::poseCallback, this);
    plan_pub_ = nh.advertise<LocalPlannerActionGoal>("/cb_base_navigation/local_planner_interface/action_server/goal",1);

    ROS_INFO_STREAM("GPI: Subsribed to '" << pose_sub_.getTopic() << "' for simple pose callbacks and I will send the plans to '" << plan_pub_.getTopic() << "'.");
}

void GlobalPlannerInterface::poseCallback(const geometry_msgs::PoseStampedConstPtr &pose)
{
    ROS_INFO("GPI: Simple Pose callback");

    GetPlanRequest req;
    GetPlanResponse resp;

    PositionConstraint pc;
    pc.frame = "/map";
    std::stringstream str;
    str << "(x-" << pose->pose.position.x << ")^2 + (y-" << pose->pose.position.y << ")^2 < .05^2";
    pc.constraint = str.str();
    req.goal_position_constraints.push_back(pc);

    if (getPlan(req,resp)) {
        if(resp.succes) {
            OrientationConstraint oc;
            oc.frame = "/map";
            oc.look_at.x = pose->pose.position.x + cos(tf::getYaw(pose->pose.orientation));
            oc.look_at.y = pose->pose.position.y + sin(tf::getYaw(pose->pose.orientation));

            LocalPlannerActionGoal goal;
            goal.goal.orientation_constraint = oc;
            goal.goal.plan = resp.plan;
            plan_pub_.publish(goal);
            ROS_INFO("GPI: Succesfully published plan to local planner :)");
        } else {
            ROS_INFO("GPI: Sorry, I can't find a valid plan to there :(.");
        }
    } else {
        ROS_ERROR("GPI: Could not get a plan, something wrong here?")   ;
    }
}

bool GlobalPlannerInterface::checkPlan(CheckPlanRequest &req, CheckPlanResponse &resp)
{
    // Lock the costmap for a sec
    boost::unique_lock< boost::shared_mutex > lock(*(costmap_.getCostmap()->getLock()));

    if(req.plan.size() == 0) { ROS_ERROR("No plan specified so no check can be performed."); return false; }
    resp.valid = global_planner_->checkPlan(req.plan);
    return true;
}

bool GlobalPlannerInterface::getPlan(GetPlanRequest &req, GetPlanResponse &resp)
{
    // Lock the costmap for a sec
    boost::unique_lock< boost::shared_mutex > lock(*(costmap_.getCostmap()->getLock()));

    // Check the input
    if(req.goal_position_constraints.size() > 1) { ROS_ERROR("You have specified more than 1 constraint, this is not yet supported."); return false; }
    if(req.goal_position_constraints.size() == 0) { ROS_ERROR("No goal position constraint specified, planner cannot create plan."); return false; }

    // Get if the robot pose is available
    if ( ! costmap_.getRobotPose(global_pose_)) { ROS_ERROR("Could not get global robot pose. We can't generate a plan, sorry."); return false; }

    // Container for goal positions in map frame
    std::vector<tf::Point> goal_positions;

    // Plan the global path
    if(global_planner_->makePlan(global_pose_,req.goal_position_constraints[0],resp.plan,goal_positions)) {
        // Visualize me something
        vis_.publishGlobalPlanMarker(resp.plan);
        vis_.publishGoalPositionsMarker(goal_positions);
        resp.succes = true;
    } else {
        resp.succes = false;
    }
    return true;
}

}
