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

//#include <ros/console.h>
#include "rclcpp/rclcpp.hpp"

#include "reach_driver/reach_driver.h"

using namespace reach_driver;

ReachSerialDriver::ReachSerialDriver(std::shared_ptr<rclcpp::Node> my_node)
    : ReachDriver(my_node), initialised(false)
{
    // Declare parameters with default values
    my_node->declare_parameter("port", "/dev/ttyACM0");
    my_node->declare_parameter("baudrate", 115200);
    my_node->declare_parameter("timeout", 1000);
    // Get parameter values
    port = my_node->get_parameter("port").as_string();
    baudrate = my_node->get_parameter("baudrate").as_int();
    timeout = my_node->get_parameter("timeout").as_int();

    RCLCPP_INFO(my_node->get_logger(), "[REACH] Connecting to Reach...");
    RCLCPP_INFO(my_node->get_logger(), "[REACH] Port: %s | Baudrate: %d | Timeout: %d", port.c_str(), baudrate, timeout);
    initialise();
}

ReachSerialDriver::~ReachSerialDriver()
{
    ser.close();
}

bool ReachSerialDriver::ok()
{
    return initialised && ser.isOpen();
}

void ReachSerialDriver::initialise()
{
    ser.setPort(port);
    ser.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
    ser.setTimeout(to);

    if (ser.isOpen())
    {
        RCLCPP_WARN(logger_,"[REACH] Port \"%s\" is already open.", port.c_str());
        return;
    }
    while (rclcpp::ok() && !ser.isOpen())
    {
        try
        {
            ser.open();
        }
        catch (serial::SerialException &e)
        {
            RCLCPP_ERROR(logger_, "[REACH] Unable to open %s. Error:\n%s", port.c_str(), e.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        catch (serial::IOException &e)
        {
            if (e.getErrorNumber() == 13)
            {
                RCLCPP_WARN(logger_, "[REACH] Do not have read access for %s.", port.c_str());
            }
            else if (e.getErrorNumber() == 2)
            {
                RCLCPP_WARN(logger_, "[REACH] Waiting for port \"%s\" to appear. Reconnecting...", port.c_str());
            }
            else
            {
                RCLCPP_WARN(logger_, "[REACH] Can't open %s.\n%s", port.c_str(), e.what());
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }
    initialised = ser.isOpen();
    if (ok())
    {
        RCLCPP_INFO(logger_, "[REACH] Reach connected!");
    }
}

void ReachSerialDriver::disconnect()
{
    ser.close();
    initialised = false;
}

void ReachSerialDriver::reconnect()
{
    disconnect();
    initialise();
}

std::string ReachSerialDriver::readFromDevice()
{
    try
    {
        return ser.read(ser.available());
    }
    catch (serial::PortNotOpenedException &e)
    {
        RCLCPP_ERROR(logger_, "[REACH] Port \"%s\" not open. Reconnecting...", port.c_str());
        reconnect();
    }
    catch (serial::SerialException &e)
    {
        RCLCPP_ERROR(logger_, "[REACH] Serial Exception. Error:\n%s.\nReconnecting...", e.what());
        reconnect();
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(logger_, "[REACH] Unable to read from port. Error:\n%s.\nReconnecting...", e.what());
        reconnect();
    }
    return "";
}

bool ReachSerialDriver::available()
{
    try
    {
        return ser.available();
    }
    catch (serial::IOException &e)
    {
        RCLCPP_ERROR(logger_, "[REACH] Unable to read from port. Error:\n%s.\nReconnecting...", e.what());
        reconnect();
    }
    return false;
}