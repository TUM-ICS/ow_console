#include <ow_console/console.h>

int main(int argc, char **argv)
{
  ros::init(argc,argv,"testFootstepPlanner",ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  ow_consol::Console console;

  ow::Parameter parameter;
  parameter.add<ow::Scalar>("t_double_support", 0.3);
  parameter.add<ow::Scalar>("t_single_support", 0.9);
  if (!parameter.load("/open_walker"))
  {
    ROS_ERROR("OWConsole::init: error loading parameters");
    return -1;
  }

  if(!console.initRequest(parameter, nh))
  {
    ROS_ERROR("Console: Error initalizing");
    return -1;
  }

  ros::Rate rate(30);
  while(ros::ok())
  {
    console.update(ros::Time::now());
    
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
