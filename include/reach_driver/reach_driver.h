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

#pragma once

//#include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

#include <string>

#include <serial/serial.h>

//#include <geometry_msgs/Twist.h>
#include "geometry_msgs/msg/twist.hpp"

//#include <sensor_msgs/NavSatFix.h>
//#include <sensor_msgs/NavSatStatus.h>
//#include <sensor_msgs/TimeReference.h>
//#include "sensor_msgs/msg/nav_sat_fix.h"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/time_reference.hpp"

//#include <nmea_msgs/Sentence.h>
//#include <nmea_msgs/Gpgga.h>
//#include <nmea_msgs/Gpgsa.h>
//#include <nmea_msgs/Gpgst.h>
//#include <nmea_msgs/Gpgsv.h>
//#include <nmea_msgs/GpgsvSatellite.h>
//#include <nmea_msgs/Gprmc.h>
//#include <nmea_msgs/Gpzda.h>
//#include <nmea_msgs/Gpvtg.h>
#include "nmea_msgs/msg/sentence.hpp"
#include "nmea_msgs/msg/gpgsv.hpp"
#include "nmea_msgs/msg/gpgsa.hpp"
#include "nmea_msgs/msg/gprmc.hpp"
#include "nmea_msgs/msg/gpgsv_satellite.hpp"
#include "nmea_msgs/msg/gpgga.hpp"

#include "nmea/nmea_parser.h"
#include "sat_nav.h"

using namespace std;

namespace reach_driver
{
    class ReachDriver : public rclppp::Node
    {
    public:
        //ReachDriver(ros::NodeHandle node, ros::NodeHandle private_nh);
        ReachDriver(const rclcpp::NodeOptions &options);
        ~ReachDriver();

        virtual bool available();
        bool poll();

    private:
        //void setSentencePubs(ros::NodeHandle private_nh, ros::NodeHandle node);
        void setSentencePubs();
        virtual string readFromDevice();

        nmea::NMEAParser parser;
        std::string frame_id;

        //ros::Publisher sentence_pub;
        //ros::Publisher gpgga_pub;
        //ros::Publisher gpgsa_pub;
        //ros::Publisher gpgst_pub;
        //ros::Publisher gpgsv_pub;
        //ros::Publisher gprmc_pub;
        //ros::Publisher gpvtg_pub;
        //ros::Publisher gpzda_pub;
        //ros::Publisher fix_pub;
        //ros::Publisher timeref_pub;
        //ros::Publisher twist_pub;
        rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr sentence_pub;
        rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr gpgga_pub;
        rclcpp::Publisher<nmea_msgs::msg::Gpgsa>::SharedPtr gpgsa_pub;
        //rclcpp::Publisher<nmea_msgs::msg::Gpgst>::SharedPtr gpgst_pub;
        rclcpp::Publisher<nmea_msgs::msg::Gpgsv>::SharedPtr gpgsv_pub;
        rclcpp::Publisher<nmea_msgs::msg::Gprmc>::SharedPtr gprmc_pub;
        //rclcpp::Publisher<nmea_msgs::msg::Gpvtg>::SharedPtr gpvtg_pub;
        //rclcpp::Publisher<nmea_msgs::msg::Gpzda>::SharedPtr gpzda_pub;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub;
        rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr timeref_pub;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

        bool publish_ignored = false;
        bool publish_gpgga = false;
        bool publish_gpgsa = false;
        //bool publish_gpgst = false;
        bool publish_gpgsv = false;
        bool publish_gprmc = false;
        //bool publish_gpvtg = false;
        //bool publish_gpzda = false;
    };

    class ReachSerialDriver : public ReachDriver
    {

    public:
        //ReachSerialDriver(ros::NodeHandle node, ros::NodeHandle private_nh);
        ReachSerialDriver(const rclcpp::NodeOptions &options);
        ~ReachSerialDriver();

        void initialise();

        void disconnect();

        void reconnect();

        bool ok();

        bool available();

    private:
        string readFromDevice();

        string port;
        int baudrate;
        int timeout;
        serial::Serial ser;
        bool initialised;
    };
}
