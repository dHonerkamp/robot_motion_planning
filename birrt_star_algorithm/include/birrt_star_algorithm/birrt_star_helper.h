//
// Created by honerkam on 7/7/21.
//

#ifndef BIRRT_STAR_ALGORITHM_BIRRT_STAR_HELPER_H
#define BIRRT_STAR_ALGORITHM_BIRRT_STAR_HELPER_H

#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <birrt_star_algorithm/birrt_star.h>

namespace birrthelper {
    void publishRobotState(ros::NodeHandle nh,
                           planning_scene::PlanningScenePtr scene,
                           robot_state::RobotState state,
                           string ns_prefix_robot,
                           map<string, double> &nvalues);

    void visualiseResult(birrt_star_motion_planning::BiRRTstarPlanner initialised_planner,
                         int n_loops,
                         std::map<std::string, double> extra_configuration = std::map<std::string, double>());

    map<string, double> runScenario(const string &planning_group,
                                    const vector<double> &env_size_x,
                                    const vector<double> &env_size_y,
                                    const vector<double> &start_ee_pose,
                                    const vector<int> &constraint_vec_start_pose,
                                    const vector<double> &ee_goal_pose,
                                    const vector<int> &constraint_vec_goal_pose,
                                    const vector <pair<double, double>> &target_coordinate_dev,
                                    const int &search_space = 1,
                                    const int &max_iterations_time = 200,
                                    const bool &max_iterations_or_time = 1,
                                    const bool &rviz_show_tree = 1,
                                    const double &iteration_sleep_time = 0.0,
                                    const int &n_loops = 5);
    map<string, double> runScenario(const string &planning_group,
                                    const vector<double> &env_size_x,
                                    const vector<double> &env_size_y,
                                    const vector<double> &start_conf,
                                    const std::map<std::string, double> &extra_configuration,
                                    const vector<double> &ee_goal_pose,
                                    const vector<int> &constraint_vec_goal_pose,
                                    const vector <pair<double, double>> &target_coordinate_dev,
                                    const int &search_space = 1,
                                    const int &max_iterations_time = 200,
                                    const bool &max_iterations_or_time = 1,
                                    const bool &rviz_show_tree = 1,
                                    const double &iteration_sleep_time = 0.0,
                                    const int &n_loops = 5);

    map<string, double> _runScenario(birrt_star_motion_planning::BiRRTstarPlanner &initialised_planner,
                                     const vector<double> &env_size_x,
                                     const vector<double> &env_size_y,
                                     const int &search_space,
                                     const int &max_iterations_time,
                                     const bool &max_iterations_or_time,
                                     const bool &rviz_show_tree,
                                     const double &iteration_sleep_time,
                                     const int &n_loops,
                                     const std::map<std::string, double> &extra_configuration = std::map<std::string, double>());
}
#endif //BIRRT_STAR_ALGORITHM_BIRRT_STAR_HELPER_H
