/* BSD 3-Clause License

Copyright (c) 2023, Zhengyi Jiang, The University of Manchester, Ice Nine Robotics Solutions Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//#include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

//#include <nmea_msgs/Sentence.h>
//#include <nmea_msgs/Gpgga.h>
//#include <nmea_msgs/Gpgsa.h>
//#include <nmea_msgs/Gpgst.h>
//#include <nmea_msgs/Gpgsv.h>
//#include <nmea_msgs/GpgsvSatellite.h>
//#include <nmea_msgs/Gprmc.h>
#include "nmea_msgs/msg/sentence.h"
#include "nmea_msgs/msg/gpgsv.h"
#include "nmea_msgs/msg/gpgsa.h"
#include "nmea_msgs/msg/gprmc.h"
#include "nmea_msgs/msg/gpgsv_satellite.h"
#include "nmea_msgs/msg/gpgga.h"

#include "nmea/nmea_parser.h"
#include "nmea/nmea_sentence.h"
#include "reach_driver/reach_driver.h"

class ReachNode : public rclcpp::Node
{
public:
  ReachNode() : Node("reach_node")
  {
    declare_parameters();
    get_parameters();

    double sleep_time = 1.0 / polling_rate_;
    bool notPolling = true;

    if (commType_ == "serial")
    {
      serial_driver_ = std::make_shared<reach_driver::ReachSerialDriver>(shared_from_this());
      while (rclcpp::ok() && serial_driver_->ok())
      {
        bool polled = serial_driver_->poll();
        if (!polled)
        {
          auto& clk = *this->get_clock();
          RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "[REACH] Failed to poll device. Waiting for data...");
          notPolling = true;
        }
        else if (notPolling)
        {
          RCLCPP_INFO(this->get_logger(), "[REACH] Polling successful. Reach is now streaming data.");
          notPolling = false;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        rclcpp::spin_some(shared_from_this());
      }
    }
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("comm_type", "serial");
    declare_parameter<double>("polling_rate", 1.0);
  }

  void get_parameters()
  {
    commType_ = get_parameter("comm_type").as_string();
    polling_rate_ = get_parameter("polling_rate").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "[REACH] Communication type: " << commType_);
    RCLCPP_INFO_STREAM(this->get_logger(), "[REACH] Polling rate: " << polling_rate_);
  }

  std::string commType_;
  double polling_rate_;
  std::shared_ptr<reach_driver::ReachSerialDriver> serial_driver_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReachNode>());
  rclcpp::shutdown();
  return 0;
}
