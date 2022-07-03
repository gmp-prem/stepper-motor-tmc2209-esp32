#ifndef _ROS_motoman_msgs_DynamicJointPoint_h
#define _ROS_motoman_msgs_DynamicJointPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "motoman_msgs/DynamicJointsGroup.h"

namespace motoman_msgs
{

  class DynamicJointPoint : public ros::Msg
  {
    public:
      typedef int16_t _num_groups_type;
      _num_groups_type num_groups;
      uint32_t groups_length;
      typedef motoman_msgs::DynamicJointsGroup _groups_type;
      _groups_type st_groups;
      _groups_type * groups;

    DynamicJointPoint():
      num_groups(0),
      groups_length(0), st_groups(), groups(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_num_groups;
      u_num_groups.real = this->num_groups;
      *(outbuffer + offset + 0) = (u_num_groups.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_groups.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->num_groups);
      *(outbuffer + offset + 0) = (this->groups_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->groups_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->groups_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->groups_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->groups_length);
      for( uint32_t i = 0; i < groups_length; i++){
      offset += this->groups[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_num_groups;
      u_num_groups.base = 0;
      u_num_groups.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_groups.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num_groups = u_num_groups.real;
      offset += sizeof(this->num_groups);
      uint32_t groups_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      groups_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      groups_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      groups_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->groups_length);
      if(groups_lengthT > groups_length)
        this->groups = (motoman_msgs::DynamicJointsGroup*)realloc(this->groups, groups_lengthT * sizeof(motoman_msgs::DynamicJointsGroup));
      groups_length = groups_lengthT;
      for( uint32_t i = 0; i < groups_length; i++){
      offset += this->st_groups.deserialize(inbuffer + offset);
        memcpy( &(this->groups[i]), &(this->st_groups), sizeof(motoman_msgs::DynamicJointsGroup));
      }
     return offset;
    }

    virtual const char * getType() override { return "motoman_msgs/DynamicJointPoint"; };
    virtual const char * getMD5() override { return "f91ca86c2821b55c8430ab0088bfe5df"; };

  };

}
#endif
