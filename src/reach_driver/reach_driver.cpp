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

#include <sstream>
#include <cmath>

//#include "ros/console.h"
#include "rclcpp/rclcpp.hpp"

#include "reach_driver/reach_driver.h"
#include "nmea/nmea_sentence.h"
#include "nmea/conversion.h"

using namespace reach_driver;
using namespace nmea;

// class ReachDriverNode
// {
// public:
//     ReachDriverNode(const rclcpp::NodeOptions & options);
//     bool available();
//     std::string readFromDevice();
//     bool poll();

// private:
//     void setSentencePubs(const rclcpp::NodeOptions &options);

//     // ROS 2 publishers
//     rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr sentence_pub;
//     rclcpp::Publisher<nmea_msgs::msg::Gpgga>::SharedPtr gpgga_pub;
//     rclcpp::Publisher<nmea_msgs::msg::Gpgsa>::SharedPtr gpgsa_pub;
//     //rclcpp::Publisher<nmea_msgs::msg::Gpgst>::SharedPtr gpgst_pub;
//     rclcpp::Publisher<nmea_msgs::msg::Gpgsv>::SharedPtr gpgsv_pub;
//     rclcpp::Publisher<nmea_msgs::msg::Gprmc>::SharedPtr gprmc_pub;
//     //rclcpp::Publisher<nmea_msgs::msg::Gpvtg>::SharedPtr gpvtg_pub;
//     //rclcpp::Publisher<nmea_msgs::msg::Gpzda>::SharedPtr gpzda_pub;

//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
//     rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub;
//     rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr timeref_pub;

//     NMEAParser parser(this->get_logger());
//     std::string frame_id;

//     bool publish_ignored = false;
//     bool publish_gpgga = false;
//     bool publish_gpgsa = false;
//    // bool publish_gpgst = false;
//    // bool publish_gpgsv = false;
//     bool publish_gprmc = false;
//    // bool publish_gpvtg = false;
//    // bool publish_gpzda = false;
// }

ReachDriver::ReachDriver(std::shared_ptr<rclcpp::Node> my_node)
: logger_(my_node->get_logger()), my_node_(my_node), parser(my_node->get_logger())
{
        bool parser_debug;
        my_node->get_parameter_or("parser_debug", parser_debug, false);
        parser.log = parser_debug;
        RCLCPP_INFO(my_node->get_logger(), "[REACH] NMEA Parser debug: %s", parser_debug ? "on" : "off");

        my_node->get_parameter_or("frame_id", frame_id, std::string("gps"));

        setSentencePubs(my_node);

        twist_pub = my_node->create_publisher<geometry_msgs::msg::Twist>("reach/vel", 100);
        fix_pub = my_node->create_publisher<sensor_msgs::msg::NavSatFix>("reach/fix", 100);
        timeref_pub = my_node->create_publisher<sensor_msgs::msg::TimeReference>("reach/time_ref", 100);
}

ReachDriver::~ReachDriver()
{
}

void ReachDriver::setSentencePubs(std::shared_ptr<rclcpp::Node> my_node)
{
    bool pub_ignored;
    my_node->declare_parameter("pub_ignored", true);
    my_node->get_parameter("pub_ignored", pub_ignored);
    RCLCPP_INFO(my_node->get_logger(), "[REACH] Publish ignored sentences: %s", (pub_ignored ? "on" : "off"));

    if (pub_ignored)
    {
        sentence_pub = my_node->create_publisher<nmea_msgs::msg::Sentence>("reach/nmea/ignored_sentence", 100);
    }

    std::string sentences;
    my_node->declare_parameter("sentences", "GGA,GSA,GST,GSV,RMC,VTG,ZDA");
    my_node->get_parameter("sentences", sentences);
    istringstream f(sentences);
    string s;
    std::stringstream ss;
    while (getline(f, s, ','))
    {
        if (s == "GGA")
        {
            publish_gpgga = true;
            gpgga_pub = my_node->create_publisher<nmea_msgs::msg::Gpgga>("reach/nmea/gpgga", 100);
            ss << s << ",";
        }
        else if (s == "GSA")
        {
            publish_gpgsa = true;
            gpgsa_pub = my_node->create_publisher<nmea_msgs::msg::Gpgsa>("reach/nmea/gpgsa", 100);
            ss << s << ",";
        }
        //else if (s == "GST")
        //{
        //    publish_gpgst = true;
        //    gpgst_pub = this->create_publisher<nmea_msgs::msg::Gpgst>("reach/nmea/gpgst", 100);
        //    ss << s << ",";
        //}
        //else if (s == "GSV")
        //{
        //    publish_gpgsv = true;
        //    gpgsv_pub = this->create_publisher<nmea_msgs::msg::Gpgsv>("reach/nmea/gpgsv", 100);
        //    ss << s << ",";
        //}
        //else if (s == "RMC")
        //{
        //    publish_gprmc = true;
        //    gprmc_pub = this->create_publisher<nmea_msgs::msg::Gprmc>("reach/nmea/gprmc", 100);
        //    ss << s << ",";
        //}
        //else if (s == "VTG")
        //{
        //    publish_gpvtg = true;
        //    gpvtg_pub = this->create_publisher<nmea_msgs::msg::Gpvtg>("reach/nmea/gpvtg", 100);
        //    ss << s << ",";
        //}
        //else if (s == "ZDA")
        //{
        //    publish_gpzda = true;
        //    gpzda_pub = this->create_publisher<nmea_msgs::msg::Gpzda>("reach/nmea/gpzda", 100);
        //    ss << s << ",";
        //}
        else
        {
            RCLCPP_WARN(my_node->get_logger(), "Unknown sentence type \"%s\". Ignoring.", s.c_str());
        }
    }
    std::string sss = ss.str();
    RCLCPP_INFO(my_node->get_logger(), "[REACH] Sentences to publish: %s", (sss.empty() ? "none" : sss.substr(0, sss.size() - 1).c_str()));
}

bool ReachDriver::available() {return false;}

string ReachDriver::readFromDevice() {return "this is a test";}

bool ReachDriver::poll()
{
    if (available())
    {
        geometry_msgs::msg::Twist twist;
        sensor_msgs::msg::NavSatFix fix;
        sensor_msgs::msg::TimeReference timeref;
        sat_nav::SatNav satNav;
        rclcpp::Time now = my_node_->get_clock()->now();
        std::string text = readFromDevice();
        std::vector<NMEASentence> sentences = parser.getSentencesFromRawText(text);
        for (size_t i = 0; i < sentences.size(); i++)
        {
            if (!sentences[i].valid())
            {
                RCLCPP_INFO(my_node_->get_logger(), "[REACH] Invalid sentence: %s", sentences[i].text.c_str());
                continue;
            }
            if (sentences[i].name == "GGA" && publish_gpgga)
            {
                nmea_msgs::msg::Gpgga gpgga;
                gpgga.header.stamp = now;
                gpgga.header.frame_id = frame_id;
                parser.parseParameters(gpgga, sentences[i]);
                gpgga_pub->publish(gpgga);
                satNav.addData(gpgga);
            }
            else if (sentences[i].name == "GSA" && publish_gpgsa)
            {
                nmea_msgs::msg::Gpgsa gpgsa;
                gpgsa.header.stamp = now;
                gpgsa.header.frame_id = frame_id;
                parser.parseParameters(gpgsa, sentences[i]);
                gpgsa_pub->publish(gpgsa);
            }
           //else if (sentences[i].name == "GST" && publish_gpgst)
           //{
           //    nmea_msgs::msg::Gpgst gpgst;
           //    gpgst.header.stamp = now;
           //    gpgst.header.frame_id = frame_id;
           //    parser.parseParameters(gpgst, sentences[i]);
           //    gpgst_pub->publish(gpgst);
           //    satNav.addData(gpgst);
           //}
           //else if (sentences[i].name == "GSV" && publish_gpgsv)
           //{
           //    nmea_msgs::msg::Gpgsv gpgsv;
           //    gpgsv.header.stamp = now;
           //    gpgsv.header.frame_id = frame_id;
           //    parser.parseParameters(gpgsv, sentences[i]);
           //    gpgsv_pub->publish(gpgsv);
           //}
           //else if (sentences[i].name == "RMC" && publish_gprmc)
           //{
           //    nmea_msgs::msg::Gprmc gprmc;
           //    gprmc.header.stamp = now;
           //    gprmc.header.frame_id = frame_id;
           //    parser.parseParameters(gprmc, sentences[i]);
           //    gprmc_pub->publish(gprmc);
           //    satNav.addData(gprmc);
           //}
           //else if (sentences[i].name == "VTG" && publish_gpvtg)
           //{
           //    nmea_msgs::msg::Gpvtg gpvtg;
           //    gpvtg.header.stamp = now;
           //    gpvtg.header.frame_id = frame_id;
           //    parser.parseParameters(gpvtg, sentences[i]);
           //    gpvtg_pub->publish(gpvtg);
           //    satNav.addData(gpvtg);
           //}
           //else if (sentences[i].name == "ZDA" && publish_gpzda)
           //{
           //    nmea_msgs::msg::Gpzda gpzda;
           //    gpzda.header.stamp = now;
           //    gpzda.header.frame_id = frame_id;
           //    parser.parseParameters(gpzda, sentences[i]);
           //    gpzda_pub->publish(gpzda);
           //    satNav.addData(gpzda);
           //}
            else if (publish_ignored)
            {
                nmea_msgs::msg::Sentence sentence;
                sentence.header.stamp = now;
                sentence.header.frame_id = frame_id;
                parser.parseParameters(sentence, sentences[i]);
                sentence_pub->publish(sentence);
            }
        }
        if (satNav.setTwist(twist))
        {
            twist_pub->publish(twist);
        }
        if (satNav.setNavSatFix(fix))
        {
            fix.header.stamp = now;
            fix.header.frame_id = frame_id;
            fix_pub->publish(fix);
        }
        if (satNav.setTimeReference(timeref))
        {
            timeref.header.stamp = now;
            timeref.header.frame_id = frame_id;
            timeref_pub->publish(timeref);
        }
        return true;
    }
    return false;
}
