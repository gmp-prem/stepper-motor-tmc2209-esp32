#ifndef _ROS_motoman_msgs_DynamicJointTrajectoryFeedback_h
#define _ROS_motoman_msgs_DynamicJointTrajectoryFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "motoman_msgs/DynamicJointState.h"

namespace motoman_msgs
{

  class DynamicJointTrajectoryFeedback : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int16_t _num_groups_type;
      _num_groups_type num_groups;
      uint32_t joint_feedbacks_length;
      typedef motoman_msgs::DynamicJointState _joint_feedbacks_type;
      _joint_feedbacks_type st_joint_feedbacks;
      _joint_feedbacks_type * joint_feedbacks;

    DynamicJointTrajectoryFeedback():
      header(),
      num_groups(0),
      joint_feedbacks_length(0), st_joint_feedbacks(), joint_feedbacks(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_num_groups;
      u_num_groups.real = this->num_groups;
      *(outbuffer + offset + 0) = (u_num_groups.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_groups.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->num_groups);
      *(outbuffer + offset + 0) = (this->joint_feedbacks_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_feedbacks_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_feedbacks_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_feedbacks_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_feedbacks_length);
      for( uint32_t i = 0; i < joint_feedbacks_length; i++){
      offset += this->joint_feedbacks[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_num_groups;
      u_num_groups.base = 0;
      u_num_groups.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_groups.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num_groups = u_num_groups.real;
      offset += sizeof(this->num_groups);
      uint32_t joint_feedbacks_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_feedbacks_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_feedbacks_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_feedbacks_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_feedbacks_length);
      if(joint_feedbacks_lengthT > joint_feedbacks_length)
        this->joint_feedbacks = (motoman_msgs::DynamicJointState*)realloc(this->joint_feedbacks, joint_feedbacks_lengthT * sizeof(motoman_msgs::DynamicJointState));
      joint_feedbacks_length = joint_feedbacks_lengthT;
      for( uint32_t i = 0; i < joint_feedbacks_length; i++){
      offset += this->st_joint_feedbacks.deserialize(inbuffer + offset);
        memcpy( &(this->joint_feedbacks[i]), &(this->st_joint_feedbacks), sizeof(motoman_msgs::DynamicJointState));
      }
     return offset;
    }

    virtual const char * getType() override { return "motoman_msgs/DynamicJointTrajectoryFeedback"; };
    virtual const char * getMD5() override { return "84d3bbf7103790ff0a8946017b895a1a"; };

  };

}
#endif
