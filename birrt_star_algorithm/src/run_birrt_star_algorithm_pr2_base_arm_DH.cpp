/*
 *run_birrt_star_algorithm_pr2_base_arm.cpp
 *
 *  Created on: Jul 3, 2021
 *      Author: Daniel Honerkamp
 */


#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <birrt_star_algorithm/birrt_star.h>
#include <planning_world_builder/planning_world_builder.h>

using namespace std;




void publishRobotState(ros::NodeHandle nh,
                       planning_scene::PlanningScenePtr scene,
                       robot_state::RobotState state,
                       string ns_prefix_robot,
                       map<string,double> &nvalues){
    state.setToDefaultValues();

    //Set current robot state
    state.setVariablePositions(nvalues);

    //Apply robot state to planning scene
    //psm_->getPlanningScene()->setCurrentState(state);
    scene->setCurrentState(state);

    //Publish state on planning scene
    moveit_msgs::PlanningScene psmsg;
    scene->getPlanningSceneMsg(psmsg);
    psmsg.robot_state.is_diff = true;
    psmsg.is_diff = true;

    ros::Publisher scene_pub = nh.advertise<moveit_msgs::PlanningScene>(ns_prefix_robot + "planning_scene", 10);;
    scene_pub.publish(psmsg);
}


int main(int argc, char** argv)
{
    //Init Node
    ros::init(argc, argv, "birrt_star_algorithm_pr2_base_arm_node_DH");

    //Node Handle
    ros::NodeHandle nh;



    // -------------------- Planner Setup ----------------------------
    //TODO: Read planning group from terminal input
    string planning_group = "pr2_base_arm";
    birrt_star_motion_planning::BiRRTstarPlanner planner(planning_group);

    //Set planning scene
    // TODO: read out / pass in the map size (-x, x), (-y, y)
    vector<double> size_x{10, 10};
    vector<double> size_y{10, 10};
    planner.setPlanningSceneInfo(size_x, size_y, "my_planning_scene", true);

    //Set default values
    int SEARCH_SPACE = 1;
    int MAX_ITERATIONS_TIME = 200;
    bool MAX_ITERATIONS_OR_TIME = 1;
    bool RVIZ_SHOW_TREE = 1;
    double ITERATION_SLEEP_TIME = 0.0;


    //Set search space for planner (0 = cartesian , 1 = c-space)
    if(argc > 1) {
      stringstream s(argv[1]);
      s >> SEARCH_SPACE;
    }

    //Set maximum iterations for planner
    if(argc > 2) {
      stringstream s(argv[2]);
      s >> MAX_ITERATIONS_OR_TIME;
    }

    //Set maximum iterations for planner
    if(argc > 3) {
      stringstream s(argv[3]);
      s >> MAX_ITERATIONS_TIME;
    }

    //Activate/Deactivate tree visualization in RVIZ
    if(argc > 4) {
      stringstream s(argv[4]);
      s >> RVIZ_SHOW_TREE;
    }

//    //Set sleep time between planner iterations (used to better see tree cosntruction process in RVIZ)
//    if(argc > 4) {
//      stringstream s(argv[4]);
//      s >> ITERATION_SLEEP_TIME;
//    }


    //Global Frame (top view) -> Shows end-effector in base_link frame
    //       ^ Y
    //       |
    //       | f
    //       |Z
    //   f   X-----> X
    //
    //         f

    //Set EE Start endeffector pose and constraint variables
    vector<double> start_ee_pose(6);
    start_ee_pose[0] = 1.5;  //3.6;  //X
    start_ee_pose[1] = 8.0;  //Y
    start_ee_pose[2] = 0.9;//obj_pos[2];        //Z
    start_ee_pose[3] = -1.57;    //RotX
    start_ee_pose[4] = 0.0;    //RotY
    start_ee_pose[5] = 0.0;    //RotZ
    vector<int> constraint_vec_start_pose(6); // (0 = don't care, 1 = constraint)
    constraint_vec_start_pose[0] = 1; //X
    constraint_vec_start_pose[1] = 1; //Y
    constraint_vec_start_pose[2] = 1; //Z
    constraint_vec_start_pose[3] = 1; //RotX
    constraint_vec_start_pose[4] = 1; //RotY
    constraint_vec_start_pose[5] = 1; //RotZ

    //Set EE Goal endeffector pose and constraint variables
    vector<double> ee_goal_pose(6);
    ee_goal_pose[0] = 1.5;       //X
    ee_goal_pose[1] = 3.0;   //Y
    ee_goal_pose[2] = 0.9;                   //Z
    ee_goal_pose[3] = 1.57;    //RotX
    ee_goal_pose[4] = 0.0;    //RotY
    ee_goal_pose[5] = 1.57;    //RotZ
    vector<int> constraint_vec_goal_pose(6);  // (0 = don't care, 1 = constraint)
    constraint_vec_goal_pose[0] = 1; //X
    constraint_vec_goal_pose[1] = 1; //Y
    constraint_vec_goal_pose[2] = 1; //Z
    constraint_vec_goal_pose[3] = 0; //RotX
    constraint_vec_goal_pose[4] = 0; //RotY
    constraint_vec_goal_pose[5] = 0; //RotZ

    //Permitted displacement for ee coordinates w.r.t desired target frame
    vector<pair<double,double> > target_coordinate_dev(6);
    target_coordinate_dev[0].first = -0.005;    //negative X deviation [m]
    target_coordinate_dev[0].second = 0.005;    //positive X deviation
    target_coordinate_dev[1].first = -0.005;    //negative Y deviation
    target_coordinate_dev[1].second = 0.005;    //positive Y deviation
    target_coordinate_dev[2].first = -0.005;    //negative Z deviation
    target_coordinate_dev[2].second = 0.005;    //positive Z deviation
    target_coordinate_dev[3].first = -0.05;     //negative Xrot deviation [rad]
    target_coordinate_dev[3].second = 0.05;     //positive Xrot deviation
    target_coordinate_dev[4].first = -0.05;     //negative Yrot deviation
    target_coordinate_dev[4].second = 0.05;     //positive Yrot deviation
    target_coordinate_dev[5].first = -0.05;     //negative Zrot deviation
    target_coordinate_dev[5].second = 0.05;     //positive Zrot deviation


    //Set constraint parameters / permitted axes for displacement (x,y,z,roll,pitch,yaw) relative to start ee pose during planning
    //  1 -> constraint
    //  0 -> unconstraint
    vector<int> constraint_vector(6);
    constraint_vector[0] = 1.0; //X translation
    constraint_vector[1] = 0.0; //Y translation
    constraint_vector[2] = 0.0; //Z translation
    constraint_vector[3] = 0.0; //X rotation
    constraint_vector[4] = 1.0; //Y rotation
    constraint_vector[5] = 1.0; //Z rotation
    //Permitted displacement for ee coordinates w.r.t task frame
    vector<pair<double,double> > permitted_coordinate_dev(6);
    permitted_coordinate_dev[0].first  = 0.0;    //negative X deviation [m]
    permitted_coordinate_dev[0].second = 0.0;   //positive X deviation
    permitted_coordinate_dev[1].first  = 0.0;    //negative Y deviation
    permitted_coordinate_dev[1].second = 0.0;   //positive Y deviation
    permitted_coordinate_dev[2].first  = 0.0;    //negative Z deviation
    permitted_coordinate_dev[2].second = 0.0;   //positive Z deviation
    permitted_coordinate_dev[3].first  = 0.0;    //negative Xrot deviation [rad]
    permitted_coordinate_dev[3].second = 0.0;   //positive Xrot deviation
    permitted_coordinate_dev[4].first  = -0.52;    //negative Yrot deviation
    permitted_coordinate_dev[4].second = 0.0;   //positive Yrot deviation
    permitted_coordinate_dev[5].first  = 0.0;    //negative Zrot deviation
    permitted_coordinate_dev[5].second = 0.0;   //positive Zrot deviation

    //Set edge cost variable weights (to apply motion preferences)
    vector<double> edge_cost_weights(13);
    edge_cost_weights[0] = 1.0; //base_x
    edge_cost_weights[1] = 1.0; //base_y
    edge_cost_weights[2] = 1.0; //base_theta
    edge_cost_weights[3] = 1.0; //manipulator joint 1
    edge_cost_weights[4] = 1.0; //manipulator joint 2
    edge_cost_weights[5] = 1.0; //manipulator joint 3
    edge_cost_weights[6] = 1.0; //manipulator joint 4
    edge_cost_weights[7] = 1.0; //manipulator joint 5
    edge_cost_weights[8] = 1.0; //manipulator joint 6
    edge_cost_weights[9] = 1.0; //manipulator joint 7
    edge_cost_weights[10] = 1.0; //manipulator joint 8
    edge_cost_weights[11] = 1.0; //manipulator joint 9
    edge_cost_weights[12] = 1.0; //manipulator joint 10


    // -------------------- Motion Planning Execution ----------------------------
    if(SEARCH_SPACE == 0)
        cout<<"Control-based Planner running......!"<<endl;
    else if (SEARCH_SPACE == 1)
        cout<<"C-Space Planner running......!"<<endl;
    else
        ROS_ERROR("PLANNER NOT KNOWN!!!");

    //Initialize planner (with start and ee goal pose)
    planner.init_planner(start_ee_pose, constraint_vec_start_pose, ee_goal_pose, constraint_vec_goal_pose, target_coordinate_dev, SEARCH_SPACE);
    //Activate the constraint
    // -> Syntax: planner.setParameterizedTaskFrame(constraint_vector, permitted_coordinate_dev, bool task_pos_global, bool task_orient_global);
    // bool task_pos_global -> indicates whether task frame position is expressed w.r.t near node ee pos or always w.r.t start frame ee pos
    // bool task_orient_global -> indicates whether task frame orientation is expressed w.r.t near node ee orientation or always w.r.t start frame ee orientation
    //planner.setParameterizedTaskFrame(constraint_vector, permitted_coordinate_dev, true, true);
    planner.setEdgeCostWeights(edge_cost_weights);

    //Run planner
    int planner_run_number = 0;
    //planner.run_planner(SEARCH_SPACE, 0, MAX_ITERATIONS, RVIZ_SHOW_TREE, ITERATION_SLEEP_TIME, planner_run_number);
    planner.run_planner(SEARCH_SPACE, MAX_ITERATIONS_OR_TIME, MAX_ITERATIONS_TIME, RVIZ_SHOW_TREE, ITERATION_SLEEP_TIME, planner_run_number);

    //End of planning phase
    cout<<"..... Planner finished"<<endl;


    // Visualize solution
    vector<vector<double>> joint_trajectory = planner.getJointTrajectory();
    vector<vector<double>> ee_trajectory = planner.getEndeffectorTrajectory();

    //Get curent planning scene and robot state
    string robot_description_robot;
    nh.param("robot_description_robot", robot_description_robot, std::string("robot_description"));
    string ns_prefix_robot;
    nh.param("ns_prefix_robot", ns_prefix_robot, std::string(""));
    string planning_scene_service_ns = ns_prefix_robot + "get_planning_scene";

    planning_scene_monitor::PlanningSceneMonitorPtr psm;
    psm.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description_robot));
    psm->requestPlanningSceneState(planning_scene_service_ns);
    planning_scene_monitor::LockedPlanningSceneRW ps(psm);
    ps->getCurrentStateNonConst().update();
    //if you want to modify it
    planning_scene::PlanningScenePtr scene = ps->diff();
    scene->decoupleParent();
    robot_state::RobotState state(psm->getRobotModel());

    vector<string> joint_names = planner.getJointNames();
    int step = 5;

    while (true) {
        for (int i = 0; i < joint_trajectory.size(); i += step) {
            vector<double> joint_values = joint_trajectory[i];
            map<string, double> nvalues;
            assert(joint_names.size() == joint_values.size());
            for (int ii = 0; ii < joint_names.size(); ++ii) {
                nvalues[joint_names[ii]] = joint_values[ii];
            }

            publishRobotState(nh, scene, state, ns_prefix_robot, nvalues);
            ros::Duration(0.01).sleep();
        }
    }


    ros::shutdown();

    return 0;
}


