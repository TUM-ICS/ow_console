#ifndef OW_CONSOLE_CONSOLE_H_
#define OW_CONSOLE_CONSOLE_H_

#include <ow_core/interfaces/generic_module_base.h>

#include <ow_core/types.h>

#include <ow_msgs/FlagsStamped.h>
#include <ow_msgs/WalkingPlan.h>
#include <geometry_msgs/WrenchStamped.h>

namespace ow_consol
{

  class Console : 
    public ow::GenericModuleBase
  {
  public:
    typedef ow::GenericModuleBase Base;

  private:
    ow::Flags flags_;
    ros::Time flags_time_;

    ros::Duration ds_dur_;
    ros::Duration ss_dur_;
    ros::Duration step_dur_;

    ow::DCMPointSetList dcm_list_;
    ow::FootStepList fs_list_;

    ow::Wrench W_l_ft_;
    ow::Wrench W_r_ft_;

    ros::Subscriber flags_sub_;
    ros::Subscriber ft_left_sub_;
    ros::Subscriber ft_right_sub_;
    ros::Subscriber plan_sub_;

  public:
    Console();
    
    virtual ~Console();

    virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh);

    void update(const ros::Time& time);

  private:
    std::string print() const;

  private:
    void flagsCallback(const ow_msgs::FlagsStampedConstPtr& msg);
    void ftCallback(const geometry_msgs::WrenchStampedConstPtr& msg, ow::FootId id);
    void planCallback(const ow_msgs::WalkingPlanConstPtr &msg);
  };

}

#endif