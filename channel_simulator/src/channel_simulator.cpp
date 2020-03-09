#include <channel_simulator/channel_simulator.h>
#include <octomap_msgs/conversions.h>


namespace channel_simulator {


#define CS_DEBUG(fmt, ...) ROS_DEBUG("[ChannelSimulator] " fmt, ##__VA_ARGS__)
#define CS_INFO(fmt, ...) ROS_INFO("[ChannelSimulator] " fmt, ##__VA_ARGS__)
#define CS_WARN(fmt, ...) ROS_WARN("[ChannelSimulator] " fmt, ##__VA_ARGS__)
#define CS_ERROR(fmt, ...) ROS_ERROR("[ChannelSimulator] " fmt, ##__VA_ARGS__)
#define CS_FATAL(fmt, ...) {ROS_FATAL("[ChannelSimulator] " fmt, ##__VA_ARGS__); exit(EXIT_FAILURE);}


ChannelSimulator::ChannelSimulator() :
  tree(nullptr)
{}


ChannelSimulator::ChannelSimulator(double L0_, double n_, double N0_, double a_, double b_) :
  tree(nullptr),
  model(L0_, n_, N0_, a_, b_)
{}


ChannelSimulator::ChannelSimulator(const ros::NodeHandle& nh) :
  tree(nullptr),
  model(nh)
{}


void ChannelSimulator::mapCB(const octomap_msgs::Octomap::ConstPtr& msg)
{
  tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));
  if (!tree)
    CS_WARN("failed to load octomap from ROS msg");
  CS_INFO("received new octomap");
}


bool ChannelSimulator::rayIntersection(const octomap::point3d& origin,
                                       const octomap::point3d& direction,
                                       octomap::point3d& intersection)
{
  // cast ray
  octomap::point3d ray_end;
  bool success = tree->castRay(origin, direction, ray_end);
  if (!success) {
    ROS_ERROR_STREAM("[ChannelSimulator] rayIntersection(...) failed: castRay(...) returned false with origin " << origin << " and direction " << direction << "; ray_end " << ray_end << " may be in unknown space");
    return false;
  }

  // compute intersection
  success = tree->getRayIntersection(origin, direction, ray_end, intersection);
  if (!success) {
    ROS_ERROR_STREAM("[ChannelSimulator] rayIntersection(...) failed: getRayIntersection(...) returned false with origin " << origin << ", direction " << direction << ", and end " << ray_end);
    return false;
  }

  return true;
}


octomap::point3d pointToLineSegment(const octomap::point3d& p1,
                                    const octomap::point3d& p2,
                                    const octomap::point3d& p3)
{
  octomap::point3d r = (p2 - p1).normalize();
  return p1 + r * ((p3 - p1).dot(r));
}


octomap::point3d getTreeBBXMax(const octomap::OcTree* tree)
{
  if (!tree) {
    CS_WARN("tried to find bbx max but no tree exists");
    return octomap::point3d();
  }
  double x,y,z;
  tree->getMetricMax(x,y,z);
  return octomap::point3d(x,y,z);
}


octomap::point3d getTreeBBXMin(const octomap::OcTree* tree)
{
  if (!tree) {
    CS_WARN("tried to find bbx min but no tree exists");
    return octomap::point3d();
  }
  double x,y,z;
  tree->getMetricMin(x,y,z);
  return octomap::point3d(x,y,z);
}


geometry_msgs::Point makeROSPoint(octomap::point3d pt)
{
  geometry_msgs::Point gm_pt;
  gm_pt.x = pt.x();
  gm_pt.y = pt.y();
  gm_pt.z = pt.z();
  return gm_pt;
}


std::vector<geometry_msgs::Point>
ChannelSimulator::computeSegments(octomap::point3d p1, octomap::point3d p2)
{
  std::vector<geometry_msgs::Point> segment_pts;
  segment_pts.push_back(makeROSPoint(p1));
  segment_pts.push_back(makeROSPoint(p2));

  // ensure p1 and p2 are inside the octomap

  if (!tree) {
    CS_WARN("cannot compute segments: octree not valid");
    return segment_pts;
  }

  octomap::OcTreeNode* p1_ptr = tree->search(p1);
  octomap::OcTreeNode* p2_ptr = tree->search(p2);

  if (!p1_ptr) {
    ROS_WARN_STREAM("[ChannelSimulator] p1 " << p1 << " out of bounds of octomap with min_bbx " << getTreeBBXMin(tree) << " and max_bbx " << getTreeBBXMax(tree));
    return segment_pts;
  } else if (!p2_ptr) {
    ROS_WARN_STREAM("[ChannelSimulator] p2 " << p2 << " out of bounds of octomap with min_bbx " << getTreeBBXMin(tree) << " and max_bbx " << getTreeBBXMax(tree));
    return segment_pts;
  }

  // ensure p1, p2 are in free space
  if (p1_ptr->getValue() > 0.0) {
    ROS_WARN_STREAM("[ChannelSimulator] p1 " << p1 << " in occupied space");
    return segment_pts;
  } else if (p2_ptr->getValue() > 0.0) {
    ROS_WARN_STREAM("[ChannelSimulator] p2 " << p2 << " in occupied space");
    return segment_pts;
  }

  // compute octomap node keys along ray
  octomap::KeyRay ray_keys;
  bool success = tree->computeRayKeys(p1, p2, ray_keys);
  if (!success) {
    ROS_ERROR_STREAM("[ChannelSimulator] computeRayKeys(...) failed between p1 " << p1 << " and p2 " << p2);
    return segment_pts;
  }
  ray_keys.addKey(tree->coordToKey(p2));

  // find interior segment points (points belonging to free space segments
  // between obstacles that don't include p1 or p2)
  std::vector<octomap::point3d> interior_segment_points;
  auto it = ray_keys.begin();
  octomap::OcTreeNode* prev_node = tree->search(*it++);
  while (it != ray_keys.end()) {

    octomap::OcTreeNode* curr_node = tree->search(*it);
    if (!curr_node) {
      CS_ERROR("failed to find node from ray key");
      return segment_pts;
    }

    if (prev_node->getValue() >= 0.0 && curr_node->getValue() < 0.0)
      interior_segment_points.push_back(tree->keyToCoord(*it));

    prev_node = curr_node;
    ++it;
  }

  // compute segment distances if in NLOS condition
  if (!interior_segment_points.empty()) {

    // unit vector pointing along ray from p1 towards p2
    octomap::point3d r = (p2 - p1).normalize();

    // form vector of points where the line segment between p1 and p2 enters or
    // exits an obstacle
    std::vector<octomap::point3d> intersection_points;
    intersection_points.push_back(p1);

    // p1 to 1st obstacle
    octomap::point3d obstacle_entry;
    if (!rayIntersection(p1, r, obstacle_entry))
      return segment_pts;
    intersection_points.push_back(obstacle_entry);

    // obstacle exit to entry of next obstacle
    auto it = interior_segment_points.begin();
    for (; it+1 != interior_segment_points.end(); ++it) {

      // find nearest point to interior_segment_points[i] on the line segment
      // connecting p1 and p2 (NOTE: interior segment points are snapped to
      // voxel centers which may not lie on the line between p1 and p2)
      octomap::point3d segment_point = pointToLineSegment(p1, p2, *it);

      // obstacle exit to segment_point then segment_point to next obstacle entry
      octomap::point3d prev_obstacle_exit, next_obstacle_entry;
      if (!rayIntersection(segment_point, -r, prev_obstacle_exit) ||
          !rayIntersection(segment_point, r, next_obstacle_entry))
        return segment_pts;

      intersection_points.push_back(prev_obstacle_exit);
      intersection_points.push_back(next_obstacle_entry);
    }

    // last obstacle to p2
    octomap::point3d obstacle_exit;
    if (!rayIntersection(p2, -r, obstacle_exit))
      return segment_pts;
    intersection_points.push_back(obstacle_exit);
    intersection_points.push_back(p2);

    segment_pts.clear();
    for (auto& pt : intersection_points)
      segment_pts.push_back(makeROSPoint(pt));
  }

  return segment_pts;
}


std::vector<geometry_msgs::Point>
ChannelSimulator::computeSegments(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  octomap::point3d om_pt1(p1.x, p1.y, p1.z);
  octomap::point3d om_pt2(p2.x, p2.y, p2.z);
  return computeSegments(om_pt1, om_pt2);
}


void ChannelSimulator::predict(const geometry_msgs::Point& pt1,
                               const geometry_msgs::Point& pt2,
                               double& mean, double& var)
{
  octomap::point3d p1(pt1.x, pt1.y, pt1.z);
  octomap::point3d p2(pt2.x, pt2.y, pt2.z);

  if (!tree) {
    CS_DEBUG("predicting without a map");
    return model.predict((p2 - p1).norm(), mean, var);
  }

  return model.predict((p2 - p1).norm(), mean, var);
}


} // namespace channel_simulator
