#ifndef _ROS_SERVICE_ReadGroupIO_h
#define _ROS_SERVICE_ReadGroupIO_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace motoman_msgs
{

static const char READGROUPIO[] = "motoman_msgs/ReadGroupIO";

  class ReadGroupIORequest : public ros::Msg
  {
    public:
      typedef uint32_t _address_type;
      _address_type address;

    ReadGroupIORequest():
      address(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->address >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->address >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->address >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->address >> (8 * 3)) & 0xFF;
      offset += sizeof(this->address);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->address =  ((uint32_t) (*(inbuffer + offset)));
      this->address |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->address |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->address |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->address);
     return offset;
    }

    virtual const char * getType() override { return READGROUPIO; };
    virtual const char * getMD5() override { return "7522928648cae826d000156b5f561c00"; };

  };

  class ReadGroupIOResponse : public ros::Msg
  {
    public:
      typedef const char* _message_type;
      _message_type message;
      typedef bool _success_type;
      _success_type success;
      typedef uint8_t _value_type;
      _value_type value;

    ReadGroupIOResponse():
      message(""),
      success(0),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      *(outbuffer + offset + 0) = (this->value >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      this->value =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->value);
     return offset;
    }

    virtual const char * getType() override { return READGROUPIO; };
    virtual const char * getMD5() override { return "2de2e2b0a489e8da4f560b339ccad847"; };

  };

  class ReadGroupIO {
    public:
    typedef ReadGroupIORequest Request;
    typedef ReadGroupIOResponse Response;
  };

}
#endif
