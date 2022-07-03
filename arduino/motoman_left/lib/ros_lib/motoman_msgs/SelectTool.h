#ifndef _ROS_SERVICE_SelectTool_h
#define _ROS_SERVICE_SelectTool_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace motoman_msgs
{

static const char SELECTTOOL[] = "motoman_msgs/SelectTool";

  class SelectToolRequest : public ros::Msg
  {
    public:
      typedef uint32_t _group_number_type;
      _group_number_type group_number;
      typedef uint32_t _tool_number_type;
      _tool_number_type tool_number;

    SelectToolRequest():
      group_number(0),
      tool_number(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->group_number >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->group_number >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->group_number >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->group_number >> (8 * 3)) & 0xFF;
      offset += sizeof(this->group_number);
      *(outbuffer + offset + 0) = (this->tool_number >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tool_number >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tool_number >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tool_number >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tool_number);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->group_number =  ((uint32_t) (*(inbuffer + offset)));
      this->group_number |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->group_number |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->group_number |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->group_number);
      this->tool_number =  ((uint32_t) (*(inbuffer + offset)));
      this->tool_number |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->tool_number |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->tool_number |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->tool_number);
     return offset;
    }

    virtual const char * getType() override { return SELECTTOOL; };
    virtual const char * getMD5() override { return "fa612b76af0dfd1addb4c87693871b10"; };

  };

  class SelectToolResponse : public ros::Msg
  {
    public:
      typedef const char* _message_type;
      _message_type message;
      typedef bool _success_type;
      _success_type success;

    SelectToolResponse():
      message(""),
      success(0)
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
     return offset;
    }

    virtual const char * getType() override { return SELECTTOOL; };
    virtual const char * getMD5() override { return "9bf829f07d795d3f9e541a07897da2c4"; };

  };

  class SelectTool {
    public:
    typedef SelectToolRequest Request;
    typedef SelectToolResponse Response;
  };

}
#endif
