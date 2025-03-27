#include <ow_console/console.h>

namespace ow_consol
{

  Console::Console() : 
    Base("Console"),
    W_l_ft_(ow::Wrench::Zero()),
    W_r_ft_(ow::Wrench::Zero())
  {
  }

  Console::~Console()
  {
  }

  bool Console::init(const ow::Parameter &parameter, ros::NodeHandle &nh)
  {
    flags_sub_ = 
      nh.subscribe<ow_msgs::FlagsStamped>("/open_walker/flags", 1, &Console::flagsCallback, this);
    ft_left_sub_ = 
      nh.subscribe<geometry_msgs::WrenchStamped>("/open_walker/robot/ft_left", 1, boost::bind(&Console::ftCallback, this, _1, ow::FootId::LEFT));
    ft_right_sub_ = 
      nh.subscribe<geometry_msgs::WrenchStamped>("/open_walker/robot/ft_right", 1, boost::bind(&Console::ftCallback, this, _1, ow::FootId::RIGHT));
    
    plan_sub_ = 
      nh.subscribe("/open_walker/walking_plan", 10, &Console::planCallback, this);

    ds_dur_ = ros::Duration(parameter.get<ow::Scalar>("t_double_support"));
    ss_dur_ = ros::Duration(parameter.get<ow::Scalar>("t_single_support"));
    step_dur_ = ds_dur_ + ss_dur_;
    
    return true;
  }

  void Console::update(const ros::Time& time)
  {
    ROS_INFO_STREAM("\n" << print());
  }

  std::string Console::print() const
  {
    std::stringstream out;
    out << std::fixed << std::setprecision(3);
    out << "state  :                  " << flags_.enumToString(flags_.state()) << "\n";
    out << "walking_phase  :          " << flags_.enumToString(flags_.walkingPhase()) << "\n";
    out << "quater  :                 " << flags_.enumToString(flags_.quater()) << "\n";
    out << "step   :                  " << flags_.enumToString(flags_.step()) << "\n";
    out << "event_out  :              " << flags_.enumToString(flags_.eventOut()) << "\n";
    out << "event_in :                " << flags_.enumToString(flags_.eventIn()) << "\n";
    out << "n_step  :                 " << flags_.nStep() << "/" << fs_list_.size() << "\n";
    out << "support_foot :            " << flags_.supportFoot().toString() << "\n";
    out << "swing_foot :              " << flags_.swingFoot().toString() << "\n";
    out << "is_standing  :            " << flags_.isStanding() << "\n";
    out << "has_ground_contact  :     " << flags_.hasGroundContact() << "\n";
    out << "elapsed_plan:             " << flags_.elapsedPlan().toSec() << "/" << fs_list_.size()*step_dur_.toSec() << "\n";
    out << "elapsed_step:             " << flags_.elapsedStep().toSec() << "/" << step_dur_.toSec() << "\n";
    out << "elapsed_double_support:   " << flags_.elapsedDoubleSupport().toSec() << "/" << ds_dur_.toSec() << "\n";
    out << "elapsed_single_support:   " << flags_.elapsedSingleSupport().toSec() << "/" << ss_dur_.toSec() << "\n";
    out << "feet_in_contact[LEFT]:    " << flags_.feetInContact()[ow_core::FootId::LEFT] << "\n";
    out << "feet_in_contact[RIGHT]:   " << flags_.feetInContact()[ow_core::FootId::RIGHT] << "\n";
    out << "Left Normal Force:        " << W_l_ft_.z() << "\n";
    out << "Right Normal Force:       " << W_r_ft_.z() << "\n";
    return out.str();
  }

  void Console::flagsCallback(const ow_msgs::FlagsStampedConstPtr& msg)
  { 
    flags_ = msg->flags;
    flags_time_ = msg->header.stamp;
  }

  void Console::ftCallback(const geometry_msgs::WrenchStampedConstPtr& msg, ow::FootId id)
  {
    if(id == ow::FootId::LEFT)
    {
      W_l_ft_ = msg->wrench;
    }
    if(id == ow::FootId::RIGHT)
    {
      W_r_ft_ = msg->wrench;
    }
  }

  void Console::planCallback(const ow_msgs::WalkingPlanConstPtr &msg)
  {
    dcm_list_.resize(msg->dcm_plan.size());
    for(size_t i = 0; i < dcm_list_.size(); ++i)
    {
      dcm_list_[i] = msg->dcm_plan[i];
    }

    fs_list_.resize(msg->footstep_plan.size());
    for(size_t i = 0; i < fs_list_.size(); ++i)
    {
      fs_list_[i] = msg->footstep_plan[i];
    }
  }
}