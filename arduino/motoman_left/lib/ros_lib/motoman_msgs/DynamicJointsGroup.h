#ifndef _ROS_motoman_msgs_DynamicJointsGroup_h
#define _ROS_motoman_msgs_DynamicJointsGroup_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/duration.h"

namespace motoman_msgs
{

  class DynamicJointsGroup : public ros::Msg
  {
    public:
      typedef int16_t _group_number_type;
      _group_number_type group_number;
      typedef int16_t _num_joints_type;
      _num_joints_type num_joints;
      typedef int16_t _valid_fields_type;
      _valid_fields_type valid_fields;
      uint32_t positions_length;
      typedef float _positions_type;
      _positions_type st_positions;
      _positions_type * positions;
      uint32_t velocities_length;
      typedef float _velocities_type;
      _velocities_type st_velocities;
      _velocities_type * velocities;
      uint32_t accelerations_length;
      typedef float _accelerations_type;
      _accelerations_type st_accelerations;
      _accelerations_type * accelerations;
      uint32_t effort_length;
      typedef float _effort_type;
      _effort_type st_effort;
      _effort_type * effort;
      typedef ros::Duration _time_from_start_type;
      _time_from_start_type time_from_start;

    DynamicJointsGroup():
      group_number(0),
      num_joints(0),
      valid_fields(0),
      positions_length(0), st_positions(), positions(nullptr),
      velocities_length(0), st_velocities(), velocities(nullptr),
      accelerations_length(0), st_accelerations(), accelerations(nullptr),
      effort_length(0), st_effort(), effort(nullptr),
      time_from_start()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_group_number;
      u_group_number.real = this->group_number;
      *(outbuffer + offset + 0) = (u_group_number.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_group_number.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->group_number);
      union {
        int16_t real;
        uint16_t base;
      } u_num_joints;
      u_num_joints.real = this->num_joints;
      *(outbuffer + offset + 0) = (u_num_joints.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_joints.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->num_joints);
      union {
        int16_t real;
        uint16_t base;
      } u_valid_fields;
      u_valid_fields.real = this->valid_fields;
      *(outbuffer + offset + 0) = (u_valid_fields.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_valid_fields.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->valid_fields);
      *(outbuffer + offset + 0) = (this->positions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->positions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->positions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->positions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->positions_length);
      for( uint32_t i = 0; i < positions_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->positions[i]);
      }
      *(outbuffer + offset + 0) = (this->velocities_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocities_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocities_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocities_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocities_length);
      for( uint32_t i = 0; i < velocities_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->velocities[i]);
      }
      *(outbuffer + offset + 0) = (this->accelerations_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accelerations_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accelerations_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accelerations_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accelerations_length);
      for( uint32_t i = 0; i < accelerations_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->accelerations[i]);
      }
      *(outbuffer + offset + 0) = (this->effort_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->effort_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->effort_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->effort_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->effort_length);
      for( uint32_t i = 0; i < effort_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->effort[i]);
      }
      *(outbuffer + offset + 0) = (this->time_from_start.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_from_start.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_from_start.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_from_start.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_from_start.sec);
      *(outbuffer + offset + 0) = (this->time_from_start.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time_from_start.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time_from_start.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time_from_start.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time_from_start.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_group_number;
      u_group_number.base = 0;
      u_group_number.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_group_number.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->group_number = u_group_number.real;
      offset += sizeof(this->group_number);
      union {
        int16_t real;
        uint16_t base;
      } u_num_joints;
      u_num_joints.base = 0;
      u_num_joints.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_joints.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->num_joints = u_num_joints.real;
      offset += sizeof(this->num_joints);
      union {
        int16_t real;
        uint16_t base;
      } u_valid_fields;
      u_valid_fields.base = 0;
      u_valid_fields.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_valid_fields.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->valid_fields = u_valid_fields.real;
      offset += sizeof(this->valid_fields);
      uint32_t positions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->positions_length);
      if(positions_lengthT > positions_length)
        this->positions = (float*)realloc(this->positions, positions_lengthT * sizeof(float));
      positions_length = positions_lengthT;
      for( uint32_t i = 0; i < positions_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_positions));
        memcpy( &(this->positions[i]), &(this->st_positions), sizeof(float));
      }
      uint32_t velocities_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocities_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocities_length);
      if(velocities_lengthT > velocities_length)
        this->velocities = (float*)realloc(this->velocities, velocities_lengthT * sizeof(float));
      velocities_length = velocities_lengthT;
      for( uint32_t i = 0; i < velocities_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_velocities));
        memcpy( &(this->velocities[i]), &(this->st_velocities), sizeof(float));
      }
      uint32_t accelerations_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      accelerations_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      accelerations_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      accelerations_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->accelerations_length);
      if(accelerations_lengthT > accelerations_length)
        this->accelerations = (float*)realloc(this->accelerations, accelerations_lengthT * sizeof(float));
      accelerations_length = accelerations_lengthT;
      for( uint32_t i = 0; i < accelerations_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_accelerations));
        memcpy( &(this->accelerations[i]), &(this->st_accelerations), sizeof(float));
      }
      uint32_t effort_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->effort_length);
      if(effort_lengthT > effort_length)
        this->effort = (float*)realloc(this->effort, effort_lengthT * sizeof(float));
      effort_length = effort_lengthT;
      for( uint32_t i = 0; i < effort_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_effort));
        memcpy( &(this->effort[i]), &(this->st_effort), sizeof(float));
      }
      this->time_from_start.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_from_start.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_from_start.sec);
      this->time_from_start.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time_from_start.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time_from_start.nsec);
     return offset;
    }

    virtual const char * getType() override { return "motoman_msgs/DynamicJointsGroup"; };
    virtual const char * getMD5() override { return "fb56acaba1b90a9d7640af3e785681ca"; };

  };

}
#endif
