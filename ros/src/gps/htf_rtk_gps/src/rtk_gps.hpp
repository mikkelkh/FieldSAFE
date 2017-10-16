/****************************************************************************
# Copyright (c) 2011-2013, author Dennis Tryk
# dennis.tryk@gmail.com
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name FroboMind nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

#ifndef RTK_GPS_H
#define RTK_GPS_H

#include <stdio.h>
#include <stdlib.h>

#include <iostream>


//#include <boost/tokenizer.hpp>
//#include <boost/lexical_cast.hpp>

#include <string.h>

#include <htf_safe_msgs/can.h>
#include <htf_safe_msgs/serial.h>
#include <nmea_msgs/Sentence.h>
#include "htf_safe_msgs/Course_Speed.h"
#include "htf_safe_msgs/GNSSPositionData.h"


#include "ros/ros.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

//typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

using namespace std;

class RtkGps
{
public:
	/*Public Methods*/
	RtkGps();
	~RtkGps();
	void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
	void getGPSData_CAN(const htf_safe_msgs::can::ConstPtr& rx_msg);
	void getGPSData_Serial(const htf_safe_msgs::serial::ConstPtr& rx_msg);
	//void processGPS_Serial_GPVTG(tokenizer& tokens,std::string raw);
	//void processGPS_Serial_GPGGA(tokenizer& tokens,std::string raw);

	/*Public Var*/
	string frame_id_gps;
	diagnostic_updater::Updater updater;
	ros::Publisher rtk_gps_pub_course_speed;
	ros::Publisher rtk_gps_pub_GNSSPositionData;
	ros::Publisher rtk_gps_pub_serialSentence;
private:
	/*Private Methods*/
	void parseCourseSpeedData(const htf_safe_msgs::can::ConstPtr& rx_msg);
	void StoreGNSSPositionData(const htf_safe_msgs::can::ConstPtr& rx_msg);
	void parseGNSSPositonData(const htf_safe_msgs::can::ConstPtr& rx_msg);


	/*Private Var*/
	htf_safe_msgs::Course_Speed CourseSpeedMsg;
	htf_safe_msgs::GNSSPositionData GNSSPositionDataMsg;
	nmea_msgs::Sentence serialSentence;
	diagnostic_updater::HeaderlessTopicDiagnostic pub_freq;
	socklen_t len;
	uint8_t GNSSFrame0[8];
	uint8_t GNSSFrame1[8];
	uint8_t GNSSFrame2[8];
	uint8_t GNSSFrame3[8];
	uint8_t GNSSFrame4[8];
	uint8_t GNSSFrame5[8];
	uint8_t GNSSFrame6[8];
	
	double min_freq;
	double max_freq;
};

#endif	
