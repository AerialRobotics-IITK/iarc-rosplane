#include <path_planner.h>

void RosplaneDubins::init(ros::NodeHandle& nh) {
    nh.param<double>("R_min", r, 25.0);
    nh.param<double>("left_pylon_y", left_y, -8.66);
    nh.param<double>("right_pylon_y", right_y, -408.66);
    nh.param<double>("hunter_killer_x", hunter_killer_x, 5.0);
    nh.param<double>("hunter_killer_y", hunter_killer_y, 0.0);
    nh.param<double>("launch_pos_x", launch_pos_x, -5.0);
    nh.param<double>("launch_pos_y", launch_pos_y, 0.0);
    nh.param<double>("height", height, -20);
    nh.param<float>("Va", Va, 12.0);
    nh.param<int>("num_waypoints", num_waypoints, 86);
    nh.param<int>("num_loops", num_loops, 8);
    nh.param<int>("speed_red_factor", speed_red_factor, 1);
    nh.param<int>("outlier_distance", o_d, 50);
    nh.param<int>("outlier_radius", o_r, 25);

    wps.resize(5 * num_waypoints);
    backward_wps.resize(5 * num_waypoints);
    RosplaneDubins::computeForwardPoints();
    RosplaneDubins::computeTransientPoints();
    RosplaneDubins::computeBackwardPoints();
}

void RosplaneDubins::computeForwardPoints() {
    /*
    Each loop contains 10 waypoints(8 loops, so 80 waypoints), 5 on each side.
    Need 3 waypoints to make a fillet path. The two fillet paths on each side has a
    waypoint in common.
     */
    for (int i = 0; i < num_loops; i++) {
        if (i == 0) {
            wps[50 * i] = r;
            wps[50 * i + 1] = -80.66;  // It is not a well calculated point(could be 5 to 10 units left or right). Just had to put a waypoint sufficiently far
                                       // from the launch position so that the quadplane can smoothly enter the loop while maintaining a decent airspeed.
            wps[50 * i + 2] = -20;
            wps[50 * i + 3] = -M_PI / 2;
            wps[50 * i + 4] = Va;

            wps[50 * i + 5] = r;
            wps[50 * i + 6] = -200.66;  // Also not a well calculated point. Same explanation as that of the previous waypoint.
            wps[50 * i + 7] = -20;
            wps[50 * i + 8] = -M_PI / 2;
            wps[50 * i + 9] = Va;
        } else {
            wps[50 * i] = r;
            wps[50 * i + 1] = left_y + r;
            wps[50 * i + 2] = -20;
            wps[50 * i + 3] = -M_PI / 2;
            wps[50 * i + 4] = Va / speed_red_factor;

            wps[50 * i + 5] = r;
            wps[50 * i + 6] = left_y;
            wps[50 * i + 7] = -20;
            wps[50 * i + 8] = -M_PI / 2;
            wps[50 * i + 9] = Va;
        }

        wps[50 * i + 10] = r;
        wps[50 * i + 11] = right_y;
        wps[50 * i + 12] = -20;
        wps[50 * i + 13] = -M_PI / 2;
        wps[50 * i + 14] = Va;

        wps[50 * i + 15] = r;
        wps[50 * i + 16] = right_y - r;
        wps[50 * i + 17] = -20;
        wps[50 * i + 18] = -M_PI / 2;
        wps[50 * i + 19] = Va / speed_red_factor;

        wps[50 * i + 20] = 0;
        wps[50 * i + 21] = right_y - r;
        wps[50 * i + 22] = -20;
        wps[50 * i + 23] = M_PI;
        wps[50 * i + 24] = Va;

        // Lower half points start from here:
        wps[50 * i + 25] = -r;
        wps[50 * i + 26] = right_y - r;
        wps[50 * i + 27] = -20;
        wps[50 * i + 28] = M_PI / 2;
        wps[50 * i + 29] = Va / speed_red_factor;

        wps[50 * i + 30] = -r;
        wps[50 * i + 31] = right_y;
        wps[50 * i + 32] = -20;
        wps[50 * i + 33] = M_PI / 2;
        wps[50 * i + 34] = Va;

        if (i != num_loops - 1) {
            wps[50 * i + 35] = -r;
            wps[50 * i + 36] = left_y;
            wps[50 * i + 37] = -20;
            wps[50 * i + 38] = M_PI / 2;
            wps[50 * i + 39] = Va;

            wps[50 * i + 40] = -r;
            wps[50 * i + 41] = left_y + r;
            wps[50 * i + 42] = -20;
            wps[50 * i + 43] = M_PI / 2;
            wps[50 * i + 44] = Va;

            wps[50 * i + 45] = 0;
            wps[50 * i + 46] = left_y + r;
            wps[50 * i + 47] = -20;
            wps[50 * i + 48] = 0;
            wps[50 * i + 49] = Va / speed_red_factor;
        } else {
            wps[50 * i + 35] = -r;
            wps[50 * i + 36] = -200.66;  // Again not a well calculated waypoint.
            wps[50 * i + 37] = -20;
            wps[50 * i + 38] = M_PI / 2;
            wps[50 * i + 39] = Va / speed_red_factor;

            wps[50 * i + 40] = -r;
            wps[50 * i + 41] = -80.66 + r;
            wps[50 * i + 42] = -20;
            wps[50 * i + 43] = M_PI / 2;
            wps[50 * i + 44] = Va;

            wps[50 * i + 45] = hunter_killer_x;
            wps[50 * i + 46] = hunter_killer_y;
            wps[50 * i + 47] = -20;
            wps[50 * i + 48] = M_PI / 4;
            wps[50 * i + 49] = Va / speed_red_factor;
        }
    }
}

void RosplaneDubins::computeTransientPoints() {
    /*
    6 extra points needed for this function, so num_waypoints should be 86 instead of 80.
    These are the points that need to be traversed in order to flip the orientation of the
    rosplane by 180 so that it can follow the backward path towards
    the launch position.
    */
    wps[400] = hunter_killer_x + o_d / (sqrt(2));
    wps[401] = hunter_killer_y + o_d / (sqrt(2));
    wps[402] = -20;
    wps[403] = M_PI / 4;
    wps[404] = Va;

    wps[405] = hunter_killer_x + (o_d + o_r) / (sqrt(2));
    wps[406] = hunter_killer_y + (o_d + o_r) / (sqrt(2));
    wps[407] = -20;
    wps[408] = M_PI / 4;
    wps[409] = Va;

    wps[410] = hunter_killer_x + (o_d + 2 * o_r) / (sqrt(2));
    wps[411] = hunter_killer_y + o_d / (sqrt(2));
    wps[412] = -20;
    wps[413] = -M_PI / 4;
    wps[414] = Va;

    wps[415] = hunter_killer_x + (o_d + 3 * o_r) / (sqrt(2));
    wps[416] = hunter_killer_y + (o_d - o_r) / (sqrt(2));
    wps[417] = -20;
    wps[418] = -M_PI / 4;
    wps[419] = Va;

    wps[420] = hunter_killer_x + (o_d + 2 * o_r) / (sqrt(2));
    wps[421] = hunter_killer_y + (o_d - 2 * o_r) / (sqrt(2));
    wps[422] = -20;
    wps[423] = -3 * M_PI / 4;
    wps[424] = Va;

    wps[425] = hunter_killer_x;
    wps[426] = hunter_killer_y;
    wps[427] = -20;
    wps[428] = -3 * M_PI / 4;
    wps[429] = Va;
}

void RosplaneDubins::computeBackwardPoints() {
    // Waypoints has been placed in a similar fashion as that in computeForwardPoints.
    for (int i = 0; i < num_loops; i++) {
        if (i == 0) {
            backward_wps[50 * i] = -r;
            backward_wps[50 * i + 1] = -80.66 + r;
            backward_wps[50 * i + 2] = -20;
            backward_wps[50 * i + 3] = -M_PI / 2;
            backward_wps[50 * i + 4] = Va;

            backward_wps[50 * i + 5] = -r;
            backward_wps[50 * i + 6] = -200.66;
            backward_wps[50 * i + 7] = -20;
            backward_wps[50 * i + 8] = -M_PI / 2;
            backward_wps[50 * i + 9] = Va / speed_red_factor;

        } else {
            backward_wps[50 * i] = -r;
            backward_wps[50 * i + 1] = left_y + r;
            backward_wps[50 * i + 2] = -20;
            backward_wps[50 * i + 3] = -M_PI / 2;
            backward_wps[50 * i + 4] = Va / speed_red_factor;

            backward_wps[50 * i + 5] = -r;
            backward_wps[50 * i + 6] = left_y;
            backward_wps[50 * i + 7] = -20;
            backward_wps[50 * i + 8] = -M_PI / 2;
            backward_wps[50 * i + 9] = Va;
        }

        backward_wps[50 * i + 10] = -r;
        backward_wps[50 * i + 11] = right_y;
        backward_wps[50 * i + 12] = -20;
        backward_wps[50 * i + 13] = -M_PI / 2;
        backward_wps[50 * i + 14] = Va / speed_red_factor;

        backward_wps[50 * i + 15] = -r;
        backward_wps[50 * i + 16] = right_y - r;
        backward_wps[50 * i + 17] = -20;
        backward_wps[50 * i + 18] = -M_PI / 2;
        backward_wps[50 * i + 19] = Va / speed_red_factor;

        backward_wps[50 * i + 20] = 0;
        backward_wps[50 * i + 21] = right_y - r;
        backward_wps[50 * i + 22] = -20;
        backward_wps[50 * i + 23] = 0;
        backward_wps[50 * i + 24] = Va;

        // Upper half points start from here:
        backward_wps[50 * i + 25] = r;
        backward_wps[50 * i + 26] = right_y - r;
        backward_wps[50 * i + 27] = -20;
        backward_wps[50 * i + 28] = M_PI / 2;
        backward_wps[50 * i + 29] = Va / speed_red_factor;

        backward_wps[50 * i + 30] = r;
        backward_wps[50 * i + 31] = right_y;
        backward_wps[50 * i + 32] = -20;
        backward_wps[50 * i + 33] = M_PI / 2;
        backward_wps[50 * i + 34] = Va;

        if (i != num_loops - 1) {
            backward_wps[50 * i + 35] = r;
            backward_wps[50 * i + 36] = left_y;
            backward_wps[50 * i + 37] = -20;
            backward_wps[50 * i + 38] = M_PI / 2;
            backward_wps[50 * i + 39] = Va / speed_red_factor;

            backward_wps[50 * i + 40] = r;
            backward_wps[50 * i + 41] = left_y + r;
            backward_wps[50 * i + 42] = -20;
            backward_wps[50 * i + 43] = M_PI / 2;
            backward_wps[50 * i + 44] = Va;

            backward_wps[50 * i + 45] = 0;
            backward_wps[50 * i + 46] = left_y + r;
            backward_wps[50 * i + 47] = -20;
            backward_wps[50 * i + 48] = 0;
            backward_wps[50 * i + 49] = Va / speed_red_factor;
        } else {
            backward_wps[50 * i + 35] = r;
            backward_wps[50 * i + 36] = -200.66;
            backward_wps[50 * i + 37] = -20;
            backward_wps[50 * i + 38] = M_PI / 2;
            backward_wps[50 * i + 39] = Va / speed_red_factor;

            backward_wps[50 * i + 40] = r;
            backward_wps[50 * i + 41] = -80.66 + r;
            backward_wps[50 * i + 42] = -20;
            backward_wps[50 * i + 43] = M_PI / 2;
            backward_wps[50 * i + 44] = Va;

            backward_wps[50 * i + 45] = launch_pos_x;
            backward_wps[50 * i + 46] = launch_pos_y;
            backward_wps[50 * i + 47] = -20;
            backward_wps[50 * i + 48] = 3 * M_PI / 4;
            backward_wps[50 * i + 49] = Va / speed_red_factor;
        }
    }
}
