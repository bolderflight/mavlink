/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef INCLUDE_MAVLINK_PARAMETER_H_
#define INCLUDE_MAVLINK_PARAMETER_H_

#include <array>
#include <string>
#include "core/core.h"
#include "mavlink_types.h"
#include "common/mavlink.h"
#include "mavlink/util.h"

namespace bfs {

template<std::size_t N>
class MavLinkParameter {
 public:
  static_assert(N < 1000, "Only up to 999 parameters supported");
  MavLinkParameter(HardwareSerial *bus) : bus_(bus), util_(bus) {
    PopulateParams();
  }
  MavLinkParameter(HardwareSerial *bus, const uint8_t sys_id) :
                   bus_(bus), sys_id_(sys_id), util_(bus, sys_id) {
    PopulateParams();
  }
  /* Getters */
  inline constexpr uint8_t sys_id() const {return sys_id_;}
  inline constexpr uint8_t comp_id() const {return comp_id_;}
  /* Get parameters */
  static constexpr std::size_t size() {return N;}
  inline std::array<float, N> parameters() const {
    std::array<float, N> ret;
    for (std::size_t i = 0; i < N; i++) {
      ret[i] = params_[i].val;
    }
    return ret;
  }
  void Update() {
    /*
    * Stream parameters, one per PARAM_PERIOD_MS. The param_index is initially
    * set to -1. Upon receiving a PARAM_REQUEST_LIST message, we set the index
    * to 0 to start the stream, which will occur until we get through the
    * entire set of parameters.
    */
    if ((param_index_ >= 0) && (param_index_ < N)) {
      if (param_timer_ms_ > PARAM_PERIOD_MS_) {
        SendParam(param_index_++);
        param_timer_ms_ = 0;
      }
    }
  }
  void MsgHandler(const mavlink_message_t &ref) {
    switch (ref.msgid) {
      case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
        mavlink_param_request_read_t request_read;
        mavlink_msg_param_request_read_decode(&ref, &request_read);
        ParamRequestReadHandler(request_read);
        break;
      }
      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
        mavlink_param_request_list_t request_list;
        mavlink_msg_param_request_list_decode(&ref, &request_list);
        ParamRequestListHandler(request_list);
        break;
      }
      case MAVLINK_MSG_ID_PARAM_SET: {
        mavlink_param_set_t param_set;
        mavlink_msg_param_set_decode(&ref, &param_set);
        ParamSetHandler(param_set);
        break;
      }
    }
  }

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  const uint8_t sys_id_ = 1;
  static const uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
  /* Message buffer */
  mavlink_message_t msg_;
  uint16_t msg_len_;
  uint8_t msg_buf_[MAVLINK_MAX_PACKET_LEN];
  /* Util class, for status text */
  MavLinkUtil util_;
  /* Parameter Microservice */
  struct Param {
    std::string param_id;
    float val;
    uint16_t param_index;
  };
  /* Currently only supporting float params */
  static const uint8_t param_type_ = MAV_PARAM_TYPE_REAL32;
  /* Array of params, fixed name and index */
  Param params_[N];
  /* Populate parameter names and indices */
  void PopulateParams() {
    for (std::size_t i = 0; i < N; i++) {
      params_[i].param_id = "PARAM_";
      if (i < 10) {
        params_[i].param_id += "00";
      } else if (i < 100) {
        params_[i].param_id += "0";
      }
      params_[i].param_id += std::to_string(i);
      params_[i].param_index = i;
    }
  }
  /* Sends parameter given an index value */
  void SendParam(const std::size_t i) {
    /* Index bounds checking */
    if ((i < 0) || (i > N - 1)) {
      util_.SendStatusText(bfs::Severity::NOTICE,
                           "Invalid Paramer ID Requested");
      return;
    }
    msg_len_ = mavlink_msg_param_value_pack(sys_id_, comp_id_, &msg_,
                                            params_[i].param_id.c_str(),
                                            params_[i].val,
                                            param_type_,
                                            N,
                                            params_[i].param_index
    );
    mavlink_msg_to_send_buffer(msg_buf_, &msg_);
    bus_->write(msg_buf_, msg_len_);
  }
  /* Message handlers */
  void ParamRequestReadHandler(const mavlink_param_request_read_t &ref) {
    if ((ref.target_system == sys_id_) &&
       ((ref.target_component == comp_id_) ||
        (ref.target_component == MAV_COMP_ID_ALL))) {
      /*
      * Param index is set to -1 if param id should be used instead. So, check
      * if param index is valid and use that, otherwise use param id.
      */
      if (ref.param_index > -1) {
        SendParam(ref.param_index);
      } else {
        for (std::size_t i = 0; i < N; i++) {
          if (std::string(ref.param_id) == params_[i].param_id) {
            SendParam(i);
            return;
          }
        }
        util_.SendStatusText(bfs::Severity::NOTICE,
                             "Invalid Paramer ID Requested");
      }
    }
  }
  void ParamRequestListHandler(const mavlink_param_request_list_t &ref) {
    if ((ref.target_system == sys_id_) &&
       ((ref.target_component == comp_id_) ||
        (ref.target_component == MAV_COMP_ID_ALL))) {
      /*
      * Set param_index_ to zero to start streaming a 
      * full list of parameters 
      */
      param_index_ = 0;
    }
  }
  void ParamSetHandler(const mavlink_param_set_t &ref) {
    if ((ref.target_system == sys_id_) &&
        (ref.target_component == comp_id_)) {
      for (std::size_t i = 0; i < N; i++) {
        if (std::string(ref.param_id) == params_[i].param_id) {
          /* Update the param value */
          params_[i].val = ref.param_value;
          /* Send the new param back */
          SendParam(i);
          return;
        }
      }
      util_.SendStatusText(bfs::Severity::NOTICE,
                           "Invalid Paramer ID Requested");
    }
  }
  /*
  * Sets up parameter streaming in response to a PARAM_REQUEST_LIST
  * message. One parameter will be sent each PARAM_PERIOD_MS. Default
  * to a param index of -1; when a PARAM_REQUEST_LIST is received,
  * update param index to 0 to start stream and continue until all
  * parameters are sent.
  */
  int param_index_ = -1;
  elapsedMillis param_timer_ms_;
  static constexpr int PARAM_PERIOD_MS_ = 250;
};

}  // namespace bfs

#endif  // INCLUDE_MAVLINK_PARAMETER_H_
