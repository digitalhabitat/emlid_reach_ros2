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


----------------------------- 3RD PARTY LICENSES -----------------------------


NemaTode is released under the ZLib license.

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

    1. The origin of this software must not be misrepresented; you must not
    claim that you wrote the original software. If you use this software
    in a product, an acknowledgment in the product documentation would be
    appreciated but is not required.
    2. Altered source versions must be plainly marked as such, and must not be
    misrepresented as being the original software.
    3. This notice may not be removed or altered from any source
    distribution.
 */

#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <exception>

#include "rclcpp/rclcpp.hpp"

#include "nmea_msgs/msg/gpgsa.hpp"
#include "nmea_msgs/msg/gpvtg.hpp"
#include "nmea_msgs/msg/gpgga.hpp"
#include "nmea_msgs/msg/gprmc.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "nmea_msgs/msg/gpzda.hpp"
#include "nmea_msgs/msg/gpgst.hpp"
#include "nmea_msgs/msg/gpgsv_satellite.hpp"
#include "nmea_msgs/msg/gpgsv.hpp"

#include <nmea/nmea_sentence.h>

namespace nmea
{
	//class NMEAParser;

	class NMEAParseError : public std::exception
	{
	public:
		std::string message;
		NMEASentence nmea;

		NMEAParseError(std::string msg);
		virtual ~NMEAParseError();

		std::string what();
	};

	class NMEAParser
	{
	private:
		void parseText(NMEASentence &nmea, std::string s);

		void logInfo(std::string s);
		void logWarning(std::string s);
		void logError(std::string s);

		void cleanUpSentence(std::string &txt);
		void trimSentenceStart(std::string &txt);
		void checkChecksum(std::string &txt);
		void parseName(NMEASentence &nmea, std::string &txt);

		static uint8_t calculateXORChecksum(std::string);

	public:
		NMEAParser(rclcpp::Logger logger);
		virtual ~NMEAParser();

		bool log;
		rclcpp::Logger logger_;
		bool ignoreEmptyChecksum;
		
		std::vector<NMEASentence> getSentencesFromRawText(std::string text);

		void parseParameters(NMEASentence &nmea);
		void parseParameters(nmea_msgs::msg::Sentence &sentence, NMEASentence &nmea);
		void parseParameters(nmea_msgs::msg::Gprmc &gprmc, NMEASentence &nmea);
		void parseParameters(nmea_msgs::msg::Gpgga &gpgga, NMEASentence &nmea);
		void parseParameters(nmea_msgs::msg::Gpgsa &gpgsa, NMEASentence &nmea);
		void parseParameters(nmea_msgs::msg::Gpgsv &gpgsv, NMEASentence &nmea);
		void parseParameters(nmea_msgs::msg::Gpgst &gpgst, NMEASentence &nmea);
		void parseParameters(nmea_msgs::msg::Gpvtg &gpvtg, NMEASentence &nmea);
		void parseParameters(nmea_msgs::msg::Gpzda &gpzda, NMEASentence &nmea);
	};

}
