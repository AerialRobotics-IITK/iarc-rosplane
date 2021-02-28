#include <ros/ros.h>
#include <rosplane_msgs/Waypoint.h>
#include <vector>

class RosplaneDubins {
  public:
    void init(ros::NodeHandle& nh);
    void computeForwardPoints();
    void computeBackwardPoints();
    void computeTransientPoints();
    std::vector<float> wps;
    std::vector<float> backward_wps;

  private:
    double left_y;           // y coordinate of the left pylon.
    double right_y;          // y coordinate of the right pylon.
    double r;                // stores minimum radius of curvature of the Dubins path.
    double height;           // height of the horizontal plane from the ground in which the quadplane has to fly OR z coordinate of the waypoints
    double hunter_killer_x;  // x coordinate of the ship.
    double hunter_killer_y;  // y coordinate of the ship
    double launch_pos_x;     // x coordinate of the launch position
    double launch_pos_y;     // y coordinate of the launch position
    float Va;                // Maximum airspeed with which the quadplane has to fly.
    int num_waypoints;       // Total number of waypoints between launch position and the ship.
    int num_loops;           // Total number of loops that the quadplane makes around the pylons in one way journey.
    int speed_red_factor;    // The factor by which the speed of the quadplane has to be reduced while it is on the curved path
    int o_d;                 // Outlier distance
    int o_r;                 // Outlier radius
};