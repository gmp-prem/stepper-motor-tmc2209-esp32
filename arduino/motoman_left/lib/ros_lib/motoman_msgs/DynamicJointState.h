#ifndef _ROS_motoman_msgs_DynamicJointState_h
#define _ROS_motoman_msgs_DynamicJointState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace motoman_msgs
{

  class DynamicJointState : public ros::Msg
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
      uint32_t positions_desired_length;
      typedef float _positions_desired_type;
      _positions_desired_type st_positions_desired;
      _positions_desired_type * positions_desired;
      uint32_t positions_errors_length;
      typedef float _positions_errors_type;
      _positions_errors_type st_positions_errors;
      _positions_errors_type * positions_errors;
      uint32_t velocities_desired_length;
      typedef float _velocities_desired_type;
      _velocities_desired_type st_velocities_desired;
      _velocities_desired_type * velocities_desired;
      uint32_t velocities_errors_length;
      typedef float _velocities_errors_type;
      _velocities_errors_type st_velocities_errors;
      _velocities_errors_type * velocities_errors;
      uint32_t accelerations_desired_length;
      typedef float _accelerations_desired_type;
      _accelerations_desired_type st_accelerations_desired;
      _accelerations_desired_type * accelerations_desired;
      uint32_t accelerations_errors_length;
      typedef float _accelerations_errors_type;
      _accelerations_errors_type st_accelerations_errors;
      _accelerations_errors_type * accelerations_errors;
      uint32_t effort_errors_length;
      typedef float _effort_errors_type;
      _effort_errors_type st_effort_errors;
      _effort_errors_type * effort_errors;
      uint32_t effort_desired_length;
      typedef float _effort_desired_type;
      _effort_desired_type st_effort_desired;
      _effort_desired_type * effort_desired;

    DynamicJointState():
      group_number(0),
      num_joints(0),
      valid_fields(0),
      positions_length(0), st_positions(), positions(nullptr),
      velocities_length(0), st_velocities(), velocities(nullptr),
      accelerations_length(0), st_accelerations(), accelerations(nullptr),
      effort_length(0), st_effort(), effort(nullptr),
      positions_desired_length(0), st_positions_desired(), positions_desired(nullptr),
      positions_errors_length(0), st_positions_errors(), positions_errors(nullptr),
      velocities_desired_length(0), st_velocities_desired(), velocities_desired(nullptr),
      velocities_errors_length(0), st_velocities_errors(), velocities_errors(nullptr),
      accelerations_desired_length(0), st_accelerations_desired(), accelerations_desired(nullptr),
      accelerations_errors_length(0), st_accelerations_errors(), accelerations_errors(nullptr),
      effort_errors_length(0), st_effort_errors(), effort_errors(nullptr),
      effort_desired_length(0), st_effort_desired(), effort_desired(nullptr)
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
      *(outbuffer + offset + 0) = (this->positions_desired_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->positions_desired_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->positions_desired_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->positions_desired_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->positions_desired_length);
      for( uint32_t i = 0; i < positions_desired_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->positions_desired[i]);
      }
      *(outbuffer + offset + 0) = (this->positions_errors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->positions_errors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->positions_errors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->positions_errors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->positions_errors_length);
      for( uint32_t i = 0; i < positions_errors_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->positions_errors[i]);
      }
      *(outbuffer + offset + 0) = (this->velocities_desired_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocities_desired_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocities_desired_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocities_desired_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocities_desired_length);
      for( uint32_t i = 0; i < velocities_desired_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->velocities_desired[i]);
      }
      *(outbuffer + offset + 0) = (this->velocities_errors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocities_errors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocities_errors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocities_errors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocities_errors_length);
      for( uint32_t i = 0; i < velocities_errors_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->velocities_errors[i]);
      }
      *(outbuffer + offset + 0) = (this->accelerations_desired_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accelerations_desired_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accelerations_desired_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accelerations_desired_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accelerations_desired_length);
      for( uint32_t i = 0; i < accelerations_desired_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->accelerations_desired[i]);
      }
      *(outbuffer + offset + 0) = (this->accelerations_errors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->accelerations_errors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->accelerations_errors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->accelerations_errors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accelerations_errors_length);
      for( uint32_t i = 0; i < accelerations_errors_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->accelerations_errors[i]);
      }
      *(outbuffer + offset + 0) = (this->effort_errors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->effort_errors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->effort_errors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->effort_errors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->effort_errors_length);
      for( uint32_t i = 0; i < effort_errors_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->effort_errors[i]);
      }
      *(outbuffer + offset + 0) = (this->effort_desired_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->effort_desired_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->effort_desired_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->effort_desired_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->effort_desired_length);
      for( uint32_t i = 0; i < effort_desired_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->effort_desired[i]);
      }
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
      uint32_t positions_desired_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      positions_desired_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      positions_desired_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      positions_desired_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->positions_desired_length);
      if(positions_desired_lengthT > positions_desired_length)
        this->positions_desired = (float*)realloc(this->positions_desired, positions_desired_lengthT * sizeof(float));
      positions_desired_length = positions_desired_lengthT;
      for( uint32_t i = 0; i < positions_desired_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_positions_desired));
        memcpy( &(this->positions_desired[i]), &(this->st_positions_desired), sizeof(float));
      }
      uint32_t positions_errors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      positions_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      positions_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      positions_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->positions_errors_length);
      if(positions_errors_lengthT > positions_errors_length)
        this->positions_errors = (float*)realloc(this->positions_errors, positions_errors_lengthT * sizeof(float));
      positions_errors_length = positions_errors_lengthT;
      for( uint32_t i = 0; i < positions_errors_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_positions_errors));
        memcpy( &(this->positions_errors[i]), &(this->st_positions_errors), sizeof(float));
      }
      uint32_t velocities_desired_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocities_desired_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocities_desired_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocities_desired_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocities_desired_length);
      if(velocities_desired_lengthT > velocities_desired_length)
        this->velocities_desired = (float*)realloc(this->velocities_desired, velocities_desired_lengthT * sizeof(float));
      velocities_desired_length = velocities_desired_lengthT;
      for( uint32_t i = 0; i < velocities_desired_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_velocities_desired));
        memcpy( &(this->velocities_desired[i]), &(this->st_velocities_desired), sizeof(float));
      }
      uint32_t velocities_errors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocities_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocities_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocities_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocities_errors_length);
      if(velocities_errors_lengthT > velocities_errors_length)
        this->velocities_errors = (float*)realloc(this->velocities_errors, velocities_errors_lengthT * sizeof(float));
      velocities_errors_length = velocities_errors_lengthT;
      for( uint32_t i = 0; i < velocities_errors_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_velocities_errors));
        memcpy( &(this->velocities_errors[i]), &(this->st_velocities_errors), sizeof(float));
      }
      uint32_t accelerations_desired_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      accelerations_desired_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      accelerations_desired_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      accelerations_desired_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->accelerations_desired_length);
      if(accelerations_desired_lengthT > accelerations_desired_length)
        this->accelerations_desired = (float*)realloc(this->accelerations_desired, accelerations_desired_lengthT * sizeof(float));
      accelerations_desired_length = accelerations_desired_lengthT;
      for( uint32_t i = 0; i < accelerations_desired_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_accelerations_desired));
        memcpy( &(this->accelerations_desired[i]), &(this->st_accelerations_desired), sizeof(float));
      }
      uint32_t accelerations_errors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      accelerations_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      accelerations_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      accelerations_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->accelerations_errors_length);
      if(accelerations_errors_lengthT > accelerations_errors_length)
        this->accelerations_errors = (float*)realloc(this->accelerations_errors, accelerations_errors_lengthT * sizeof(float));
      accelerations_errors_length = accelerations_errors_lengthT;
      for( uint32_t i = 0; i < accelerations_errors_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_accelerations_errors));
        memcpy( &(this->accelerations_errors[i]), &(this->st_accelerations_errors), sizeof(float));
      }
      uint32_t effort_errors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      effort_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      effort_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      effort_errors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->effort_errors_length);
      if(effort_errors_lengthT > effort_errors_length)
        this->effort_errors = (float*)realloc(this->effort_errors, effort_errors_lengthT * sizeof(float));
      effort_errors_length = effort_errors_lengthT;
      for( uint32_t i = 0; i < effort_errors_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_effort_errors));
        memcpy( &(this->effort_errors[i]), &(this->st_effort_errors), sizeof(float));
      }
      uint32_t effort_desired_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      effort_desired_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      effort_desired_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      effort_desired_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->effort_desired_length);
      if(effort_desired_lengthT > effort_desired_length)
        this->effort_desired = (float*)realloc(this->effort_desired, effort_desired_lengthT * sizeof(float));
      effort_desired_length = effort_desired_lengthT;
      for( uint32_t i = 0; i < effort_desired_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_effort_desired));
        memcpy( &(this->effort_desired[i]), &(this->st_effort_desired), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "motoman_msgs/DynamicJointState"; };
    virtual const char * getMD5() override { return "c44649b8de969b98f15adea419fa49a4"; };

  };

}
#endif
