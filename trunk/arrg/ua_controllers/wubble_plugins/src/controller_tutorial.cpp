#include <pr2_controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>


class JointEffortController
  : public controller::Controller
{
public:
  virtual bool starting();
  virtual void update(void);
  virtual bool stopping();
  virtual bool init(pr2_mechanism::RobotState *robot, const ros::NodeHandle &n);
};


bool JointEffortController::
starting()
{
  return true;
}


void JointEffortController::
update(void)
{
}


bool JointEffortController::
stopping()
{
  return true;
}


bool JointEffortController::
init(pr2_mechanism::RobotState *robot, const ros::NodeHandle &n)
{
  return true;
}


PLUGINLIB_REGISTER_CLASS (JointEffortController, JointEffortController, controller::Controller)
