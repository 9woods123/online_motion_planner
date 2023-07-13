#include <ros/ros.h>
#include <online_motion_planner/online_motion_planner.h>

using namespace std;


online_motion_planner::online_motion_planner(const ros::NodeHandle &nh,
                                                     const ros::NodeHandle &nh_private)
                          : nh_(nh), nh_private_(nh_private), rate_(10.0) {

    initPlanner();

    esdf_server_.reset(new voxblox::EsdfServer(nh_, nh_private_));   // begin an esdf ros process
    esdf_server_->setTraversabilityRadius(0.2);
    configurationSpace.SetMapPtr(esdf_server_.get());




    traj_pub=nh_.advertise<uuv_control_msgs::Trajectory>(robot_name_+"/input_trajectory", 1, true);
    holdOnClient=nh_private_.serviceClient<uuv_control_msgs::Hold>(robot_name_+"/hold_vehicle");

    ctrl_ref_sub=nh_.subscribe(robot_name_+"/reference_marker",
                                 10, &online_motion_planner::refCallback,this);
    robot_pose_sub=nh_.subscribe(robot_name_+"/ground_truth_to_tf_eca_a9/pose",
                                 10, &online_motion_planner::poseCallback,this);
    goal_sub=nh_.subscribe("/goal",
                           10, &online_motion_planner::goalCallback,this);



    local_planning_timer=nh_.createTimer(rate_.expectedCycleTime(),
                                         &online_motion_planner::planning_loop,this);

}

void online_motion_planner::initPlanner() {

    // init planning_state_ as uninitialized, waitting

    nh_private_.param<string>("robot_name", robot_name_, "/eca_a9");
    nh_private_.param<float>("planning_horizon", planning_horizon_, 35);
    nh_private_.param<float>("replan_horizon", replan_horizon_, 5);
    nh_private_.param<float>("step_min", step_min_, 2);
    nh_private_.param<float>("radius_min", radius_min_, 10);
    nh_private_.param<float>("planning_resolution", planning_resolution_, 0.5);
    nh_private_.param<int>("astar_iteration", astar_iteration_, 15000);

    nh_private_.param<float>("bound_box_length", bound_box_length_, 2);
    nh_private_.param<float>("bound_box_width", bound_box_width_, 1);
    nh_private_.param<float>("bound_box_height", bound_box_height_, 1);
    nh_private_.param<float>("collision_distance", collision_distance_, 1);

    nh_private_.param<float>("v_max", v_max_, 2);
    nh_private_.param<float>("a_max", a_max_, 2);
    nh_private_.param<float>("dt", dt_, 0.5);


    hybridAstarPlanner.setSearchHorizon(planning_horizon_);
    hybridAstarPlanner.setMinStep(step_min_);
    hybridAstarPlanner.setResolution(planning_resolution_);
    hybridAstarPlanner.setMinRadius(radius_min_);
    HybridAStar::Node4D::setMotionPrims();

    configurationSpace.setBoundBox(bound_box_length_,bound_box_width_,bound_box_height_);
    configurationSpace.setCollisionDistance(collision_distance_);


    GoalRecevied=false;
    planning_state_=PlanningState::UNINIT;

}

void online_motion_planner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

void online_motion_planner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        new_goal_pose=*msg;
        GoalRecevied=true;
        new_goal=true;
}

void online_motion_planner::refCallback(const visualization_msgs::MarkerConstPtr &msg) {
    ctrl_ref=*msg;
    ctrl_ref_pose.pose.position=ctrl_ref.pose.position;
    ctrl_ref_pose.pose.orientation=ctrl_ref.pose.orientation;
}

void online_motion_planner::pub_planned_traj(std::vector<HybridAStar::Node4D> path_nodes,ros::Time traj_start_time) {

//=========  publish planned trajectory to auv controller  ===========
//  v=0.7/d

 //   ros::Time d(step_min_/v_max_);   
    ros::Time d(dt_);   
    ros::Time last_time=traj_start_time;
    ros::Time traj_stamp;
    int i=0;

    planned_traj.header.frame_id="world";
    planned_traj.header.stamp=last_time;
    planned_traj.points.clear();

    for (auto iter = path_nodes.begin(); iter != path_nodes.end(); ++iter)
    {
        uuv_control_msgs::TrajectoryPoint traj_point;

        traj_point.header.stamp=traj_stamp.fromNSec(last_time.toNSec()+i*d.toNSec());
        i++;
        traj_point.header.frame_id="world";
        traj_point.pose.position.x=iter->getX();
        traj_point.pose.position.y=iter->getY();
        traj_point.pose.position.z=iter->getZ();
        traj_point.pose.orientation = tf::createQuaternionMsgFromYaw(iter->getT());
        planned_traj.points.push_back(traj_point);
    }

    traj_pub.publish(planned_traj);

}

geometry_msgs::PoseStamped online_motion_planner::getTrajPointFromTime(ros::Time current_time) {

    geometry_msgs::PoseStamped curr_traj_target;

    if(current_time.toNSec()< planned_traj.points[0].header.stamp.toNSec())
    {
        curr_traj_target.pose.position=planned_traj.points[0].pose.position;
        curr_traj_target.pose.orientation=planned_traj.points[0].pose.orientation;
        return curr_traj_target;
    }
    else if(current_time.toNSec()>planned_traj.points.back().header.stamp.toNSec())
    {
        curr_traj_target.pose.position=planned_traj.points.back().pose.position;
        curr_traj_target.pose.orientation=planned_traj.points.back().pose.orientation;
        return curr_traj_target;
    }

    for (auto traj_point:planned_traj.points)
    {
        if (current_time.toNSec()<=traj_point.header.stamp.toNSec())
        {
            curr_traj_target.pose.position=traj_point.pose.position;
            curr_traj_target.pose.orientation=traj_point.pose.orientation;
            return curr_traj_target;
        }
    }

}


void online_motion_planner::setStart(geometry_msgs::PoseStamped point) {

    start[0]=point.pose.position.x;
    start[1]=point.pose.position.y;
    start[2]=point.pose.position.z;
    start[3]=HybridAStar::Helper::normalizeHeadingRad(tf::getYaw(point.pose.orientation));


}

void online_motion_planner::setGoal(geometry_msgs::PoseStamped point){

    goal[0]=point.pose.position.x;
    goal[1]=point.pose.position.y;
    goal[2]=point.pose.position.z;
    goal[3]=HybridAStar::Helper::normalizeHeadingRad(tf::getYaw(point.pose.orientation));
    new_goal=false;

}


bool online_motion_planner::ReachGoal() {

    float e_dis=std::sqrt(
            (current_pose.pose.position.x-goal.x())*(current_pose.pose.position.x-goal.x())
            + (current_pose.pose.position.y-goal.y())*(current_pose.pose.position.y-goal.y())
            + (current_pose.pose.position.z-goal.z())*(current_pose.pose.position.z-goal.z())
    );

    double yaw=tf::getYaw(current_pose.pose.orientation);
    double yaw_dis=std::abs(yaw-goal[3]);
    yaw_dis=std::min(yaw_dis,(2*M_PI-yaw_dis));


    if(e_dis>1 or yaw_dis>0.2)
    {
        return false;
    }
    else
    {
        GoalRecevied=false;
        return true;
    }

}

bool online_motion_planner::trajReplanHorizon() {

    double distance_start=
            (Eigen::Vector3d (ctrl_ref_pose.pose.position.x,
                              ctrl_ref_pose.pose.position.y,
                              ctrl_ref_pose.pose.position.z)
             -start.head(3)).norm();

    double distance_goal=
            (Eigen::Vector3d (ctrl_ref_pose.pose.position.x,
                              ctrl_ref_pose.pose.position.y,
                              ctrl_ref_pose.pose.position.z)
             -goal.head(3)).norm();

    if(distance_goal<replan_horizon_)
    {
        //near the goal
        return false;
    }

    if(distance_start>replan_horizon_)
    {
        ROS_INFO_STREAM("\033[1;33m Get the Replan Horizon      \033[0m");
        return true;
    }
    else
    {
        return false;
    }

}


bool online_motion_planner::trajCollisionDetection() {


    for (auto it = smoother.get4DPath().rbegin(); it != smoother.get4DPath().rend(); ++it) {

        const HybridAStar::Node4D& node = *it;
        if(!configurationSpace.isTraversable(&node))
        {
            ROS_INFO_STREAM("\033[1;33m Future Traj Collision     \033[0m");
            return true;
        }

    }


    return false;
}

void online_motion_planner::holdOn() {

    uuv_control_msgs::HoldRequest Req;
    uuv_control_msgs::HoldResponse Res;

    // 调用服务
    if (holdOnClient.call(Req, Res)) {
        if (Res.success) {
            ROS_INFO_STREAM("\033[1;33m Hold On The Vehicle  \033[0m");
        } else {
            ROS_ERROR("Failed to call hold vehicle service");
        }
    }
}

void online_motion_planner::planning_loop(const ros::TimerEvent &event) {


    switch (planning_state_) {

        case PlanningState::UNINIT:
        {
            if(!GoalRecevied)
            {
                ROS_INFO_STREAM("\033[31m Uninitialized, requiring a goal \033[0m");
            }
            else{
                planning_state_=PlanningState::NEWPLAN;
            }
            break;
        }

        case PlanningState::NEWPLAN:

        {
            ROS_INFO_STREAM("\033[1;32m New Plan     \033[0m");

            traj_start_time=ros::Time::now();
            setStart(current_pose);
            setGoal(new_goal_pose);
            HybridAStar::Node4D Start(start[0],start[1],start[2],start[3]
                                      , 0, 0, nullptr);

            HybridAStar::Node4D Goal( goal[0],goal[1],goal[2],goal[3]
                    ,0, 0, nullptr);


            visualization.clear();

            // using hybridastar to find the path

            HybridAStar::Node4D* nSolution = hybridAstarPlanner.hybridAStar
                    (Start, Goal, configurationSpace, visualization);
            double cost=ros::Time::now().toSec()-traj_start_time.toSec();
            ROS_INFO_STREAM("\033[1;32m HybridAStar calculation cost "<<cost<<"s \033[0m");


            // TRACE THE PATH
            smoother.tracePath(nSolution);
            smoother.smooth(&configurationSpace, dt_ ,0 , v_max_,  a_max_);
            smoothedPath.updatePath(smoother.get4DPath());
            if(smoother.get4DPath().size()>1)
            {
                planning_state_=PlanningState::EXECUTE;
                smoothedPath.publishPathVehicles();
                pub_planned_traj(smoother.get4DPath(),traj_start_time);
                break;
            } else
            {
                ROS_INFO_STREAM("\033[1;31m Failed to find a path \033[0m");
//                holdOn();
                break;
            }
        }

        case PlanningState::REPLAN:
        {
            ROS_INFO_STREAM("\033[1;32m REPLAN         \033[0m");

            traj_start_time=ros::Time::now();
            setStart(getTrajPointFromTime(traj_start_time));
            setGoal(new_goal_pose);

            HybridAStar::Node4D Start(start[0],start[1],start[2],start[3]
                    , 0, 0, nullptr);
            HybridAStar::Node4D Goal( goal[0],goal[1],goal[2],goal[3]
                    ,0, 0, nullptr);

            visualization.clear();

            HybridAStar::Node4D* nSolution = hybridAstarPlanner.hybridAStar
                    (Start, Goal, configurationSpace, visualization);
            double cost=ros::Time::now().toSec()-traj_start_time.toSec();
            ROS_INFO_STREAM("\033[1;32m hybridAStar calculation cost "<<cost<<"s \033[0m");

            //TODO   v_init  should be   v_current_ref, we use v_max_ to test  raw codes.
            smoother.tracePath(nSolution);
            smoother.smooth(&configurationSpace, dt_ ,v_max_ , v_max_,  a_max_);
            smoothedPath.updatePath(smoother.get4DPath());

            if(smoother.get4DPath().size()>1)
            {
                planning_state_=PlanningState::EXECUTE;
                smoothedPath.publishPathVehicles();
                pub_planned_traj(smoother.get4DPath(),traj_start_time);

            } else
            {
                ROS_INFO_STREAM("\033[1;31m Failed to find a path \033[0m");
                planning_state_=PlanningState::NEWPLAN;
            }
            break;
        }

        case PlanningState::EXECUTE:
        {

            if (ReachGoal())
            {
                planning_state_=PlanningState::GETGOAL;
            }
            else if(trajCollisionDetection()||trajReplanHorizon()||new_goal)
            {
                planning_state_=PlanningState::REPLAN;
            }
            else
            {
                planning_state_=PlanningState::EXECUTE;
            }

            break;
        }

        case PlanningState::GETGOAL:
        {
            ROS_INFO_STREAM("\033[1;31m Reached Goal \033[0m");
            planning_state_=PlanningState::UNINIT;
            break;
        }
    }

    ros::spinOnce();

}



