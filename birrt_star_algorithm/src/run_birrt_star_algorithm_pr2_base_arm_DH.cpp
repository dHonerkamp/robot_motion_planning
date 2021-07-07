/*
 *run_birrt_star_algorithm_pr2_base_arm.cpp
 *
 *  Created on: Jul 3, 2021
 *      Author: Daniel Honerkamp
 */


#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <birrt_star_algorithm/birrt_star.h>
#include <birrt_star_algorithm/birrt_star_helper.h>
//#include <planning_world_builder/planning_world_builder.h>

using namespace std;


// TODO: option to pass in full initial robot configuration (should already exist, just replace the reading from a txt file)
// TODO: move left arm into same initial pose as for modulation_rl (in same position in gazebo, but does not seem to update the robot model with this), torso link is not getting updated either


int main(int argc, char** argv)
{
    // -------------------- Planner Setup ----------------------------
    //TODO: Read planning group from terminal input
    string planning_group = "pr2_base_arm";

    //Set planning scene
    // NOTE: just set to something definitely larger than our maps (-x, x), (-y, y)
    vector<double> env_size_x{-20, 20};
    vector<double> env_size_y{-20, 20};

    //Set default values
    int SEARCH_SPACE = 1;
    int MAX_ITERATIONS_TIME = 75;
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
// modulation_tasks.world
//    start_ee_pose[0] = 1.5;  //X
//    start_ee_pose[1] = 8.0;  //Y
//    start_ee_pose[2] = 1.3;  //Z
//    start_ee_pose[3] = 0.0;  //RotX
//    start_ee_pose[4] = 0.0;  //RotY
//    start_ee_pose[5] = 1.57;  //RotZ
// aws apartment
//    start_ee_pose[0] = 6.5;  //X
//    start_ee_pose[1] = -3.0;  //Y
//    start_ee_pose[2] = 1.3;  //Z
//    start_ee_pose[3] = 0.0;  //RotX
//    start_ee_pose[4] = 0.0;  //RotY
//    start_ee_pose[5] = 1.57;  //RotZ
// aws bookstore
    start_ee_pose[0] = -3.0;  //X
    start_ee_pose[1] = 5.0;  //Y
    start_ee_pose[2] = 1.3;  //Z
    start_ee_pose[3] = 0.0;  //RotX
    start_ee_pose[4] = 0.0;  //RotY
    start_ee_pose[5] = 1.57;  //RotZ
// aws bookstore
    start_ee_pose[0] = 0.0;  //X
    start_ee_pose[1] = 0.0;  //Y
    start_ee_pose[2] = 1.3;  //Z
    start_ee_pose[3] = 0.0;  //RotX
    start_ee_pose[4] = 0.0;  //RotY
    start_ee_pose[5] = 1.57;  //RotZ
    vector<int> constraint_vec_start_pose(6); // (0 = don't care, 1 = constraint)
    constraint_vec_start_pose[0] = 1; //X
    constraint_vec_start_pose[1] = 1; //Y
    constraint_vec_start_pose[2] = 1; //Z
    constraint_vec_start_pose[3] = 1; //RotX
    constraint_vec_start_pose[4] = 1; //RotY
    constraint_vec_start_pose[5] = 1; //RotZ

    //Set EE Goal endeffector pose and constraint variables
    vector<double> ee_goal_pose(6);
// modulation_tasks.world
//    ee_goal_pose[0] = 0.0;   //X
//    ee_goal_pose[1] = 0.0;   //Y
//    ee_goal_pose[2] = 0.9;   //Z
//    ee_goal_pose[3] = 1.57;  //RotX
//    ee_goal_pose[4] = 0.0;   //RotY
//    ee_goal_pose[5] = 1.57;  //RotZ
// aws apartment
//    ee_goal_pose[0] = -5.0;   //X
//    ee_goal_pose[1] = -3.5;   //Y
//    ee_goal_pose[2] = 0.9;   //Z
//    ee_goal_pose[3] = 1.57;  //RotX
//    ee_goal_pose[4] = 0.0;   //RotY
//    ee_goal_pose[5] = 1.57;  //RotZ
// aws bookstore
//    ee_goal_pose[0] = 1.0;   //X
//    ee_goal_pose[1] = -3.0;   //Y
//    ee_goal_pose[2] = 0.4;   //Z
//    ee_goal_pose[3] = 1.57;  //RotX
//    ee_goal_pose[4] = 0.0;   //RotY
//    ee_goal_pose[5] = 1.57;  //RotZ
// aws bookstore 2
    ee_goal_pose[0] = -7.0;   //X
    ee_goal_pose[1] = 1.0;   //Y
    ee_goal_pose[2] = 0.7;   //Z
    ee_goal_pose[3] = 1.57;  //RotX
    ee_goal_pose[4] = 0.0;   //RotY
    ee_goal_pose[5] = 1.57;  //RotZ
    vector<int> constraint_vec_goal_pose(6);  // (0 = don't care, 1 = constraint)
    constraint_vec_goal_pose[0] = 1; //X
    constraint_vec_goal_pose[1] = 1; //Y
    constraint_vec_goal_pose[2] = 1; //Z
    constraint_vec_goal_pose[3] = 1; //RotX
    constraint_vec_goal_pose[4] = 1; //RotY
    constraint_vec_goal_pose[5] = 1; //RotZ

    //Permitted displacement for ee coordinates w.r.t desired target frame
    vector<pair<double,double>> target_coordinate_dev(6);
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


//    //Set constraint parameters / permitted axes for displacement (x,y,z,roll,pitch,yaw) relative to start ee pose during planning
//    //  1 -> constraint
//    //  0 -> unconstraint
//    vector<int> constraint_vector(6);
//    constraint_vector[0] = 1.0; //X translation
//    constraint_vector[1] = 0.0; //Y translation
//    constraint_vector[2] = 0.0; //Z translation
//    constraint_vector[3] = 0.0; //X rotation
//    constraint_vector[4] = 1.0; //Y rotation
//    constraint_vector[5] = 1.0; //Z rotation
//    //Permitted displacement for ee coordinates w.r.t task frame
//    vector<pair<double,double> > permitted_coordinate_dev(6);
//    permitted_coordinate_dev[0].first  = 0.0;    //negative X deviation [m]
//    permitted_coordinate_dev[0].second = 0.0;   //positive X deviation
//    permitted_coordinate_dev[1].first  = 0.0;    //negative Y deviation
//    permitted_coordinate_dev[1].second = 0.0;   //positive Y deviation
//    permitted_coordinate_dev[2].first  = 0.0;    //negative Z deviation
//    permitted_coordinate_dev[2].second = 0.0;   //positive Z deviation
//    permitted_coordinate_dev[3].first  = 0.0;    //negative Xrot deviation [rad]
//    permitted_coordinate_dev[3].second = 0.0;   //positive Xrot deviation
//    permitted_coordinate_dev[4].first  = -0.52;    //negative Yrot deviation
//    permitted_coordinate_dev[4].second = 0.0;   //positive Yrot deviation
//    permitted_coordinate_dev[5].first  = 0.0;    //negative Zrot deviation
//    permitted_coordinate_dev[5].second = 0.0;   //positive Zrot deviation

    runScenario(planning_group,
                env_size_x,
                env_size_y,
                start_ee_pose,
                constraint_vec_start_pose,
                ee_goal_pose,
                constraint_vec_goal_pose,
                target_coordinate_dev,
                SEARCH_SPACE,
                MAX_ITERATIONS_TIME,
                MAX_ITERATIONS_OR_TIME,
                RVIZ_SHOW_TREE,
                ITERATION_SLEEP_TIME);

    ros::shutdown();

    return 0;
}


