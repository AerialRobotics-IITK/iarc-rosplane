#include <ros/ros.h>
#include <rosplane_msgs/Waypoint.h>
#include <vector>

class RosplaneDubins {
  public:
    void init(ros::NodeHandle& nh);
    void computeForwardPoints();
    void computeBackwardPoints();
    std::vector<float> wps;
    std::vector<float> backward_wps;

  private:
    double left_y;
    double right_y;
    double r;
    double height;
    double hunter_killer_x;
    double hunter_killer_y;
    double launch_pos_x;
    double launch_pos_y;
    float Va;
    int num_waypoints;
    int num_loops;
    int speed_red_factor;

    ros::Publisher waypointPublisher;
};