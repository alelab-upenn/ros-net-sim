#include <example/nav_base.h>
#include <nodelet/nodelet.h>


namespace example {


class NavBaseNodelet : public nodelet::Nodelet
{
  protected:
    ros::NodeHandle nh, pnh;
    std::shared_ptr<NavBase> nav_ptr;

  public:
    virtual void onInit()
    {
      nh = getNodeHandle();
      pnh = getPrivateNodeHandle();

      nav_ptr.reset(new NavBase(getName(), nh, pnh));
    }
};


} // namespace example


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(example::NavBaseNodelet, nodelet::Nodelet);
