#ifndef CHANNEL_SIMULATOR_CHANNEL_SIMULATOR_H
#define CHANNEL_SIMULATOR_CHANNEL_SIMULATOR_H

#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/Point.h>

#include <vector>
#include <math.h>
#include <sstream>


namespace channel_simulator {


class ChannelModel {
  protected:
    double L0;       // transmit power (dBm)
    //double PL0;      // transmit power (mW)
    double n;        // decay exponent
    //double sigma_F2; // noise variance
    double N0;       // noise at receiver (dBm)
    double PN0;      // noise at receiver (mW)
    double a;        // sigmoid param 1
    double b;        // sigmoid param 2

  public:
    ChannelModel() :
      L0(-53.0), n(2.52), N0(-70.0), a(0.2), b(6.0)
    {
      PN0 = dBm2mW(N0);
      printf("channel model parameters:\n");
      printf("L0 = %.1f\n", L0);
      printf("n  = %.2f\n", n);
      printf("N0 = %.1f\n", N0);
      printf("a  = %.2f\n", a);
      printf("b  = %.2f\n", b);
    }

    ChannelModel(double L0_, double n_, double N0_, double a_, double b_) :
      L0(L0_), n(n_), N0(N0_), a(a_), b(b_)
    {
      PN0 = dBm2mW(N0);
      printf("channel model parameters:\n");
      printf("L0 = %.1f\n", L0);
      printf("n  = %.2f\n", n);
      printf("N0 = %.1f\n", N0);
      printf("a  = %.2f\n", a);
      printf("b  = %.2f\n", b);
    }

    template<typename T>
    void getParamWarn(const ros::NodeHandle& nh, std::string name, T& param, T default_val)
    {
      if (!nh.getParam(name, param)) {
        std::stringstream ss;
        ss << default_val;
        std::string default_val_str = ss.str();
        ROS_WARN("[ChannelModel] failed to get ROS param \"%s\"; using default value %s", name.c_str(), default_val_str.c_str());
      } else {
        std::stringstream ss;
        ss << param;
        std::string val_str = ss.str();
        ROS_INFO("[ChannelModel] using %s for %s", val_str.c_str(), name.c_str());
      }
    }

    ChannelModel(const ros::NodeHandle& nh)
    {
      getParamWarn(nh, "/N0", N0, -70.0);
      getParamWarn(nh, "/n", n, 2.52);
      getParamWarn(nh, "/L0", L0, -53.0);
      getParamWarn(nh, "/a", a, 0.2);
      getParamWarn(nh, "/b", b, 6.0);
      PN0 = dBm2mW(N0);
    }

    double dBm2mW(const double dBm) { return pow(10.0, dBm/10.0); }
    void predict(const double d, double& mean, double& var)
    {
      mean = erf(sqrt(dBm2mW(L0 - 10*n*log10(d)) / PN0));
      var = pow((a * d / (b + d)), 2.0);
    }
};


class ChannelSimulator
{
  protected:
    octomap::OcTree* tree;
    ChannelModel model;

    bool rayIntersection(const octomap::point3d& origin,
                         const octomap::point3d& direction,
                         octomap::point3d& intersection);

  public:
    ChannelSimulator();
    ChannelSimulator(double L0_, double n_, double N0_, double a_, double b_);
    ChannelSimulator(const ros::NodeHandle& nh);

    std::vector<geometry_msgs::Point> computeSegments(octomap::point3d p1,
                                                      octomap::point3d p2);
    std::vector<geometry_msgs::Point> computeSegments(geometry_msgs::Point p1,
                                                      geometry_msgs::Point p2);
    void mapCB(const octomap_msgs::Octomap::ConstPtr& msg);
    void predict(const geometry_msgs::Point& pose1,
                 const geometry_msgs::Point& pose2,
                 double& mean, double& var);
    bool receivedMap() { return tree == nullptr; }

};


} // namespace channel_simulator

#endif
