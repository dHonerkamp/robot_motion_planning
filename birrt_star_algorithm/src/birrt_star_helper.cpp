//
// Created by honerkam on 7/7/21.
//
#include <birrt_star_algorithm/birrt_star_helper.h>

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

void visualiseResult(ros::NodeHandle nh, birrt_star_motion_planning::BiRRTstarPlanner planner){
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

    for (int i = 0; i < joint_names.size(); i ++) {
        cout<<joint_names[i]<<std::endl;
    }

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
            ros::Duration(0.05).sleep();
        }
        ros::Duration(0.5).sleep();
    }
}

map<string, double> runScenario(const string &planning_group,
                                const vector<double> &env_size_x,
                                const vector<double> &env_size_y,
                                const vector<double> &start_ee_pose,
                                const vector<int> &constraint_vec_start_pose,
                                const vector<double> &ee_goal_pose,
                                const vector<int> &constraint_vec_goal_pose,
                                const vector <pair<double, double>> &target_coordinate_dev,
                                const int &search_space,
                                const int &max_iterations_time,
                                const bool &max_iterations_or_time,
                                const bool &rviz_show_tree,
                                const double &iteration_sleep_time) {
    // -------------------- Motion Planning Execution ----------------------------
    if(search_space == 0)
        cout<<"Control-based Planner running......!"<<endl;
    else if (search_space == 1)
        cout<<"C-Space Planner running......!"<<endl;
    else
        ROS_ERROR("PLANNER NOT KNOWN!!!");

    int blub = 0;
    ros::init(blub, NULL, "birrt_star_algorithm_pr2_base_arm_node_DH");
    ros::NodeHandle nh;

    birrt_star_motion_planning::BiRRTstarPlanner planner(planning_group);
    planner.setPlanningSceneInfo(env_size_x, env_size_y, "my_planning_scene", true);

    //Initialize planner (with start and ee goal pose)
    planner.init_planner(start_ee_pose, constraint_vec_start_pose, ee_goal_pose, constraint_vec_goal_pose, target_coordinate_dev, search_space);
    //Activate the constraint
    // -> Syntax: planner.setParameterizedTaskFrame(constraint_vector, permitted_coordinate_dev, bool task_pos_global, bool task_orient_global);
    // bool task_pos_global -> indicates whether task frame position is expressed w.r.t near node ee pos or always w.r.t start frame ee pos
    // bool task_orient_global -> indicates whether task frame orientation is expressed w.r.t near node ee orientation or always w.r.t start frame ee orientation
    //planner.setParameterizedTaskFrame(constraint_vector, permitted_coordinate_dev, true, true);

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
    planner.setEdgeCostWeights(edge_cost_weights);

    //Run planner
    int planner_run_number = 0;
    //planner.run_planner(search_space, 0, MAX_ITERATIONS, rviz_show_tree, iteration_sleep_time, planner_run_number);
    bool success = planner.run_planner(search_space,
                                       max_iterations_or_time,
                                       max_iterations_time,
                                       rviz_show_tree,
                                       iteration_sleep_time,
                                       planner_run_number);
    cout << "..... Planner finished" << endl;

    if (rviz_show_tree) {
        visualiseResult(nh, planner);
    }

    return planner.getMetrics();
}