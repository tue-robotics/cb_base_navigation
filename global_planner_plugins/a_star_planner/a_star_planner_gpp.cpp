#include <pluginlib/class_list_macros.h>
#include "a_star_planner_gpp.h"
#include "costmap_2d/cost_values.h"

#include <ed/SimpleQuery.h>

namespace cb_global_planner
{

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(cb_global_planner::AStarPlannerGPP, cb_global_planner::GlobalPlannerPlugin)

// ----------------------------------------------------------------------------------------------------

bool AStarPlannerGPP::queryEntityPose(const std::string& id, tf::Transform& pose)
{
    if (id == "map" || id == "/map")
    {
        pose = tf::Transform::getIdentity();
        return true;
    }

    ed::SimpleQuery ed_query;
    ed_query.request.id = id;

    std::cout << "[A* Planner] Querying ed for entity '" << id << "'." << std::endl;

    if (!ed_client_.call(ed_query))
    {
        ROS_ERROR_STREAM("[A* Planner] Failed to get pose for entity '" << id << "': ED could not be queried.");
        return false;
    }

    if (ed_query.response.entities.empty())
    {
        ROS_ERROR_STREAM("[A* Planner] Failed to get pose for entity '" << id << "': ED returns 'no such entity'.");
        return false;
    }

    const ed::EntityInfo& e_info = ed_query.response.entities.front();

    tf::poseMsgToTF(e_info.pose, pose);
    return true;
}

// ----------------------------------------------------------------------------------------------------

AStarPlannerGPP::AStarPlannerGPP() : global_costmap_ros_(NULL),  planner_(NULL) {}

void AStarPlannerGPP::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* global_costmap_ros)
{
    // Store a local pointer to the global costmap and the tf_listener
    global_costmap_ros_ = global_costmap_ros;
    tf_ = tf;

    // Create AstarPlanner Object ( initialize with current costmap width and height )
    planner_ = new AStarPlanner(global_costmap_ros->getCostmap()->getSizeInCellsX(), global_costmap_ros->getCostmap()->getSizeInCellsY());
    initialized_ = true;

    ros::NodeHandle nh;
    ed_client_ = nh.serviceClient<ed::SimpleQuery>("ed/simple_query");

    ROS_INFO("A* Global planner initialized.");
}

AStarPlannerGPP::~AStarPlannerGPP()
{
    delete planner_;
}

bool AStarPlannerGPP::makePlan(const tf::Stamped<tf::Pose>& start, const PositionConstraint& position_constraint, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<tf::Point>& goal_positions)
{
    if (!initialized_) { ROS_WARN("The global planner is not initialized! It's not possible to create a global plan."); return false; }

    // Clear the plan and goal positions
    plan.clear();
    goal_positions.clear();

    // If nothing specified, do nothing :)
    if (position_constraint.frame == "" && position_constraint.constraint == "") return false;

    unsigned int mx_start, my_start;
    if(!global_costmap_ros_->getCostmap()->worldToMap(start.getOrigin().getX(), start.getOrigin().getY(), mx_start, my_start)) {
        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        ROS_ERROR_STREAM("Received position constraint: " << position_constraint);
        return false;
    }

    // Check whether the constraint has been changed
    if (constraintChanged(position_constraint)) {
        if (updateConstraintPositionsInConstraintFrame(position_constraint)) {
            position_constraint_ = position_constraint;
        } else {
            ROS_WARN("Failed to update constraint positions in constraint frame.");
            ROS_ERROR_STREAM("Received position constraint: " << position_constraint);
            return false;
        }
    }

    // Calculate the area in the map frame which meets the constraints
    std::vector<unsigned int> mx_goal, my_goal;
    if (!calculateMapConstraintArea(mx_goal,my_goal,goal_positions))
    {
        ROS_ERROR_STREAM("Received position constraint: " << position_constraint);
        return false;
    }

    if(mx_goal.size() == 0) {
        ROS_ERROR("There is no goal which meets the given constraints. Planning will always fail to this goal constraint.");
        ROS_ERROR_STREAM("Received position constraint: " << position_constraint);
        return false;
    }

    // Divide goal area in two: with costs above and below a certain threshold
    // This prevents the planner for planning unnecessarily close to obstacles
    std::vector<unsigned int> mx_free, my_free, mx_low, my_low, mx_high, my_high;
    for (unsigned int i = 0; i < mx_goal.size(); i++) {

        // Check cost
        double cost = global_costmap_ros_->getCostmap()->getCost(mx_goal[i], my_goal[i]);
        // ToDo: divided by four is magic number
        // ToDo: more levels?
        if (cost == costmap_2d::FREE_SPACE) {
            mx_free.push_back(mx_goal[i]);
            my_free.push_back(my_goal[i]);
        } else if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE/4) {
            mx_low.push_back(mx_goal[i]);
            my_low.push_back(my_goal[i]);
        } else {
            mx_high.push_back(mx_goal[i]);
            my_high.push_back(my_goal[i]);
        }
    }

    // Initialize plan
    std::vector<int> plan_xs, plan_ys;

    // Resize to current costmap dimensions and set costmap and do some path finding :)
    planner_->resize(global_costmap_ros_->getCostmap()->getSizeInCellsX(), global_costmap_ros_->getCostmap()->getSizeInCellsY());
    planner_->setCostmap(global_costmap_ros_->getCostmap()->getCharMap());

    //planner_->plan(mx_goal, my_goal, mx_start, my_start, plan_xs, plan_ys);
    // Try to find a plan with the endgoal in free space
    planner_->plan(mx_free, my_free, mx_start, my_start, plan_xs, plan_ys);
    
    // If no plan, retry with low costs
    if (plan_xs.empty()) {
        planner_->plan(mx_low, my_low, mx_start, my_start, plan_xs, plan_ys);
    }

    // If still no plan, retry with the remaining goal poses
    if (plan_xs.empty()) {
        planner_->plan(mx_high, my_high, mx_start, my_start, plan_xs, plan_ys);
    }

//    if (plan_xs.empty()) {

//        // Try best heuristics path from the other way around
//        unsigned int mx_start_new = mx_goal[mx_goal.size()/2]; // middlepoint of area
//        unsigned int my_start_new = my_goal[my_goal.size()/2]; // middlepoint of area

//        mx_goal.clear(); mx_goal.push_back(mx_start);
//        my_goal.clear(); my_goal.push_back(my_start);

//        planner_->plan(mx_goal, my_goal, mx_start_new, my_start_new, plan_xs, plan_ys,true);

//        // Reverse the plan
//        std::reverse(plan_xs.begin(),plan_xs.end());
//        std::reverse(plan_ys.begin(),plan_ys.end());

//    }

    // Convert plan to world coordinates
    planToWorld(plan_xs,plan_ys,plan);

    // If no plan was found, return false
    if (plan.empty()) {
        ROS_ERROR("No connectivity to specified constraint found, sorry :(");
    } else {
        ROS_INFO("A* planner succesfully generated plan :)");
    }
    return true;
}

bool AStarPlannerGPP::updateConstraintPositionsInConstraintFrame(PositionConstraint position_constraint)
{
    ROS_INFO("Position constraint has been changed, updating positions in constraint frame.");

    // Clear the constraint positions in constraint world
    goal_positions_in_constraint_frame_.clear();

    // Request the constraint frame transform from map
    tf::Transform world_to_constraint_tf;
    if (!queryEntityPose(position_constraint.frame, world_to_constraint_tf))
        return false;

    tf::Transform constraint_to_world_tf = world_to_constraint_tf.inverse();

//    try {
//        tf_->lookupTransform(position_constraint.frame, global_costmap_ros_->getGlobalFrameID(), ros::Time(0), constraint_to_world_tf);
//    } catch(tf::TransformException& ex) {
//        ROS_ERROR_STREAM( "Transform error calculating constraint positions in global planner: " << ex.what());
//        return false;
//    }

    ConstraintEvaluator ce;

    if (!ce.init(position_constraint.constraint)) {
        ROS_ERROR("Could not setup goal constraints...");
        return false;
    }

    // Iterate over all costmap cells and evaluate the constraint in the constraint frame
    for (unsigned int i = 0; i < global_costmap_ros_->getCostmap()->getSizeInCellsX(); ++i) {
        for (unsigned int j = 0; j < global_costmap_ros_->getCostmap()->getSizeInCellsY(); ++j) {

            double wx,wy;
            global_costmap_ros_->getCostmap()->mapToWorld(i,j,wx,wy);
            tf::Point pw(wx,wy,0);
            tf::Point pc = constraint_to_world_tf*pw;

            if(ce.evaluate(pc.x(),pc.y())) {
                //ROS_INFO_STREAM("Pushing back in constraint frame point: " << pc.x() << " - " << pc.y());
                goal_positions_in_constraint_frame_.push_back(pc);
            }
        }
    }
    return true;
}

bool AStarPlannerGPP::calculateMapConstraintArea(std::vector<unsigned int>& mx, std::vector<unsigned int>& my, std::vector<tf::Point>& goal_positions)
{
    ROS_INFO("Calculating map constraint area ...");

    // Request the constraint frame transform from map
    tf::StampedTransform world_to_constraint_tf;
    if (!queryEntityPose(position_constraint_.frame, world_to_constraint_tf))
        return false;

//    try {
//        tf_->lookupTransform(global_costmap_ros_->getGlobalFrameID(), position_constraint_.frame, ros::Time(0), world_to_constraint_tf);
//    } catch(tf::TransformException& ex) {
//        ROS_ERROR_STREAM( "Transform error calculating constraint positions in global planner: " << ex.what());
//        return false;
//    }

    // Loop over the positions in the constraint frame and convert these to map points
    std::vector<tf::Point>::const_iterator it = goal_positions_in_constraint_frame_.begin();
    for (; it != goal_positions_in_constraint_frame_.end(); ++it) {
        tf::Point pw = world_to_constraint_tf * *it;
        //ROS_INFO_STREAM("Pushing back in world frame point: " << pw.x() << " - " << pw.y());
        unsigned int x,y;
        if (global_costmap_ros_->getCostmap()->worldToMap(pw.x(),pw.y(),x,y)) { // This should guarantee that we do not go off map, however the a* crashes sometimes

            // Check whether this point is blocked by an obstacle
            unsigned char goal_cell_cost = global_costmap_ros_->getCostmap()->getCost(x, y);
            if (goal_cell_cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || goal_cell_cost == costmap_2d::LETHAL_OBSTACLE) {
                continue;
            }

            goal_positions.push_back(pw);
            mx.push_back(x); my.push_back(y);
            //ROS_INFO_STREAM("Pushing back in map point: " << x << ";" << y);
        }
    }

    return true;
}

void AStarPlannerGPP::planToWorld(const std::vector<int>& plan_xs, const std::vector<int>& plan_ys, std::vector<geometry_msgs::PoseStamped>& plan)
{
    ros::Time plan_time = ros::Time::now();
    std::string global_frame = global_costmap_ros_->getGlobalFrameID();

    plan.resize(plan_xs.size());
    for(unsigned int i = 0; i < plan_xs.size(); ++i) {
        double world_x, world_y;
        global_costmap_ros_->getCostmap()->mapToWorld(plan_xs[i], plan_ys[i], world_x, world_y);

        plan[i].header.stamp = plan_time;
        plan[i].header.frame_id = global_frame;
        plan[i].pose.position.x = world_x;
        plan[i].pose.position.y = world_y;
        plan[i].pose.position.z = 0;

        unsigned int size = 5;

        if (i+size < plan_xs.size())
        {
            global_costmap_ros_->getCostmap()->mapToWorld(plan_xs[i+size], plan_ys[i+size], world_x, world_y);
            double yaw = atan2(world_y - plan[i].pose.position.y, world_x - plan[i].pose.position.x);
            plan[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        else if (plan_xs.size() > size)
        {
            plan[i].pose.orientation = plan[i-1].pose.orientation;
        }
    }
}

bool AStarPlannerGPP::checkPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    unsigned int mx,my;
    unsigned char cost;
    std::vector<geometry_msgs::PoseStamped>::const_iterator it = plan.begin();
    for(; it != plan.end(); ++it) {
        geometry_msgs::PoseStamped p;
        if (global_costmap_ros_->getCostmap()->worldToMap(it->pose.position.x,it->pose.position.y,mx,my)) {
            cost = global_costmap_ros_->getCostmap()->getCost(mx, my);

            if (cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::LETHAL_OBSTACLE) { // This also has to be removed and replaced by a proper implementation
                return false;
            }
        }
    }
    return true;
}

} // end namespace

