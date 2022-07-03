#ifndef _ROS_SERVICE_CmdJointTrajectoryEx_h
#define _ROS_SERVICE_CmdJointTrajectoryEx_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "industrial_msgs/ServiceReturnCode.h"
#include "motoman_msgs/DynamicJointTrajectory.h"

namespace motoman_msgs
{

static const char CMDJOINTTRAJECTORYEX[] = "motoman_msgs/CmdJointTrajectoryEx";

  class CmdJointTrajectoryExRequest : public ros::Msg
  {
    public:
      typedef motoman_msgs::DynamicJointTrajectory _trajectory_type;
      _trajectory_type trajectory;

    CmdJointTrajectoryExRequest():
      trajectory()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->trajectory.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->trajectory.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return CMDJOINTTRAJECTORYEX; };
    virtual const char * getMD5() override { return "7896a81fd909fb14239085c546790e08"; };

  };

  class CmdJointTrajectoryExResponse : public ros::Msg
  {
    public:
      typedef industrial_msgs::ServiceReturnCode _code_type;
      _code_type code;

    CmdJointTrajectoryExResponse():
      code()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->code.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->code.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return CMDJOINTTRAJECTORYEX; };
    virtual const char * getMD5() override { return "50b1f38f75f5677e5692f3b3e7e1ea48"; };

  };

  class CmdJointTrajectoryEx {
    public:
    typedef CmdJointTrajectoryExRequest Request;
    typedef CmdJointTrajectoryExResponse Response;
  };

}
#endif
