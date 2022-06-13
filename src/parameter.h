/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
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

#ifndef MAVLINK_SRC_PARAMETER_H_  // NOLINT
#define MAVLINK_SRC_PARAMETER_H_

#if defined(ARDUINO)
#include "Arduino.h"
#else
#include "core/core.h"
#endif
#include <array>
#include <string>
#include "mavlink/mavlink_types.h"
#include "mavlink/common/mavlink.h"
#include "util.h"  // NOLINT

namespace bfs {

template<std::size_t N>
class MavLinkParameter {
 public:
  static_assert(N < 1000, "Only up to 999 parameters supported");
  MavLinkParameter() {PopulateParams();}
  /* Config */
  inline void hardware_serial(HardwareSerial *bus) {
    bus_ = bus;
    util_.hardware_serial(bus);
  }
  inline void sys_id(const uint8_t sys_id) {
    sys_id_ = sys_id;
    util_.sys_id(sys_id);
  }
  inline void comp_id(const uint8_t comp_id) {
    comp_id_ = comp_id;
    util_.comp_id(comp_id);
  }
  /* System and component ID getters */
  inline uint8_t sys_id() const {return sys_id_;}
  inline uint8_t comp_id() const {return comp_id_;}
  /* Get parameters */
  static constexpr std::size_t size() {return N;}
  inline void params(const std::array<float, N> &val) {
    for (std::size_t i = 0; i < N; i++) {
      params_[i].val = val[i];
    }
  }
  inline std::array<float, N> params() const {
    std::array<float, N> ret;
    for (std::size_t i = 0; i < N; i++) {
      ret[i] = params_[i].val;
    }
    return ret;
  }
  inline void param(const int32_t idx, const float val) {
    if ((idx < 0) || (idx > N)) {
      return;
    }
    params_[idx].val = val;
  }
  inline float param(const int32_t idx) const {
    if ((idx < 0) || (idx > N)) {
      return 0.0f;
    }
    return params_[idx].val;
  }
  inline int32_t updated_param() {
    int32_t ret = updated_index_;
    updated_index_ = -1;
    return ret;
  }
  template<std::size_t NCHAR>
  inline void param_id(const int32_t idx, char const (&name)[NCHAR]) {
    static_assert(NCHAR < 18, "Parameter name limited to 16 characters");
    if ((idx < 0) || (idx > N)) {return;}
    params_[idx].param_id = name;
  }
  inline std::string param_id(const int32_t idx) const {
    if ((idx < 0) || (idx > N)) {
      return std::string();
    }
    return params_[idx].param_id;
  }
  /* Update and message handler methods */
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
        mavlink_msg_param_request_read_decode(&ref, &request_read_);
        ParamRequestReadHandler(request_read_);
        break;
      }
      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
        mavlink_msg_param_request_list_decode(&ref, &request_list_);
        ParamRequestListHandler(request_list_);
        break;
      }
      case MAVLINK_MSG_ID_PARAM_SET: {
        mavlink_msg_param_set_decode(&ref, &param_set_);
        ParamSetHandler(param_set_);
        break;
      }
    }
  }

 private:
  /* Serial bus */
  HardwareSerial *bus_;
  /* Config */
  bool configured_ = false;
  uint8_t sys_id_ = 1;
  uint8_t comp_id_ = MAV_COMP_ID_AUTOPILOT1;
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
  static constexpr uint8_t param_type_ = MAV_PARAM_TYPE_REAL32;
  /* Array of params, fixed name and index */
  Param params_[N];
  /* Whether the params have been updated */
  int32_t updated_index_ = -1;
  #if defined(ARDUINO)
  /* Buffer for itoa */
  char itoa_buf_[10];
  #endif
  /* Populate parameter names and indices */
  void PopulateParams() {
    for (std::size_t i = 0; i < N; i++) {
      params_[i].param_id = "PARAM_";
      if (i < 10) {
        params_[i].param_id += "00";
      } else if (i < 100) {
        params_[i].param_id += "0";
      }
      #if defined(ARDUINO)
      itoa(i, itoa_buf_, 10);
      params_[i].param_id += std::string(itoa_buf_);
      #else
      params_[i].param_id += std::to_string(i);
      #endif
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
                                            params_[i].param_index);
    mavlink_msg_to_send_buffer(msg_buf_, &msg_);
    bus_->write(msg_buf_, msg_len_);
  }
  /* Message handlers */
  mavlink_param_request_read_t request_read_;
  mavlink_param_request_list_t request_list_;
  mavlink_param_set_t param_set_;
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
          /* Set the updated index */
          updated_index_ = i;
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
  int32_t param_index_ = -1;
  elapsedMillis param_timer_ms_;
  static constexpr int32_t PARAM_PERIOD_MS_ = 250;
};

}  // namespace bfs

#endif  // MAVLINK_SRC_PARAMETER_H_ NOLINT
