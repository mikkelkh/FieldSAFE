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


#include "rtk_gps.hpp"
#include <boost/algorithm/string/replace.hpp>
#include <boost/range/as_array.hpp>
/**************************************************************
   * Constructor
**************************************************************/
RtkGps::RtkGps() : pub_freq("topic1", updater, diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10))
{
	min_freq = 1;
	max_freq = 10;
}

/**************************************************************
   * Descructor
  **************************************************************/
RtkGps::~RtkGps()
{
}

/**************************************************************
   * Diagnostic updater needs at least an empty method to call, 
   * when running.
  **************************************************************/
void RtkGps::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
}

/**************************************************************
   * Detect CAN identifier and call the right parser functions
  **************************************************************/
void RtkGps::getGPSData_CAN(const htf_safe_msgs::can::ConstPtr& rx_msg)
{
	int i;
	len = rx_msg->length;
	if(len = 8)
	{
		switch(rx_msg->id)
		{
			case 0x19F8021D:
				parseCourseSpeedData(rx_msg);
				break;
			case 0x19F8051D:
				StoreGNSSPositionData(rx_msg);
				break;
		}
	}
}

/**************************************************************
   * Detect serial identifier and call the right parser functions
  **************************************************************/
void RtkGps::getGPSData_Serial(const htf_safe_msgs::serial::ConstPtr& rx_msg)
{
	serialSentence.header.stamp = rx_msg->header.stamp;
	serialSentence.header.frame_id = frame_id_gps;

	string sentenceOld = rx_msg->data;
	//sentenceOld.erase(sentenceOld.size()-1);
	//sentenceOld.erase(0,1);
	boost::replace_all(sentenceOld, "\n","");
	serialSentence.sentence = sentenceOld;// sentenceOld.substr(0,sentenceOld.length()-1);
	rtk_gps_pub_serialSentence.publish(serialSentence);

/*
	boost::char_separator<char> sep("$*,");
	tokenizer tokens(rx_msg->data, sep);
	tokenizer::iterator tok_iter;


	ROS_ERROR("INPUT MESSAGE: %s",rx_msg->data.c_str());
	
	if(tokens.begin() != tokens.end())
	{
		tok_iter = tokens.begin();

		//ROS_ERROR("%s",(*tok_iter).c_str());

		// Check for identifier GPGGA
		if((*tok_iter).compare("GPGGA") == 0)
		{
			processGPS_Serial_GPGGA(tokens,rx_msg->data);
		}

		// Check for identifier GPVTG
		else if((*tok_iter).compare("GPVTG") == 0){
			processGPS_Serial_GPVTG(tokens,rx_msg->data);
		}
		else{
			ROS_INFO("Ignoring Unknown NMEA identifier %s", (*tok_iter).c_str());
		}
	}*/

}

/**************************************************************
   * Parse GPS string of type GPGGA 
  **************************************************************/
/*void RtkGps::processGPS_Serial_GPGGA(tokenizer& tokens,std::string raw){
	//GPGGA,112112.70,5629.37803512,N,00935.20192341,E,4,08,1.2,58.938,M,43.002,M,1.7,0026*73
	//GPGGA,hhmmss.ss,llll.ll      ,a,yyyyy.yy      ,a,x,xx,x.x, x.x  ,M,x.x   ,M,x.x,xxxx*hh


//uint8 SECONDS
//uint8 MINUTES
//uint8 HOURS
//uint8 DAY
//uint8 MONTH
//uint16 YEAR
//		float64 Latitude
//		float64 Longitude
//		float64 Altitude
//uint8 TypeOfSystem
//uint8 GNSSMethod
//uint8 GNSSIntegrity
//		float64 HDOP (horisontal delution of position)
//float64 PDOP
//		uint8 NumberOfSatelites
//uint8 NumberOfRefStations

	tokenizer::iterator tok_iter = tokens.begin();

	// skip identifier
	tok_iter++;

	CourseSpeedMsg.header.stamp = ros::Time::now();
	CourseSpeedMsg.header.frame_id = frame_id_coursespeed;
	// 1) hhmmss.ss = UTC of position 
	double clock = atof((*tok_iter++).c_str()); 
  	//GNSSPositionDataMsg.YEAR = ptm->tm_year +1900;
  	//GNSSPositionDataMsg.MONTH = ptm->tm_mon;
  	//GNSSPositionDataMsg.DAY = ptm->tm_mday;
  	//GNSSPositionDataMsg.HOURS = uint8(clock/10000);
  	//GNSSPositionDataMsg.MINUTES = ((clock>>2) & 0x03);
  	//GNSSPositionDataMsg.SECONDS = (clock & 0x03);

	// 2) llll.ll = latitude of position
	GNSSPositionDataMsg.Latitude = atof((*tok_iter++).c_str()); 

	// 3) a = N or S
	tok_iter++; 

	// 4) yyyyy.yy = Longitude of position
	GNSSPositionDataMsg.Longitude = atof((*tok_iter++).c_str());

	// 5) a = E or W
	tok_iter++; 

	// 6) x = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
	tok_iter++; 

	// 7) xx = Number of satellites in use [not those in view] xx = number of satellites in use 
	GNSSPositionDataMsg.NumberOfSatelites = atoi((*tok_iter++).c_str());
	
	// 8) x.x = Horizontal dilution of position
	GNSSPositionDataMsg.HDOP = atof((*tok_iter++).c_str());

	// 9) x.x = Antenna altitude above/below mean sea level (geoid)
	GNSSPositionDataMsg.Altitude = atof((*tok_iter++).c_str());

	// 10) M = Meters  (Antenna height unit)
	tok_iter++; 

	// 11) x.x = Geoidal separation (Diff. between WGS-84 earth ellipsoid and mean sea level.  -=geoid is below WGS-84 ellipsoid)
	tok_iter++; 

	// 12) M = Meters  (Units of geoidal separation)
	tok_iter++; 

	// 13) x.x = Age in seconds since last update from diff. reference station
	tok_iter++; 

	// 14) xxxx = Differential reference station ID
	tok_iter++; 



	ROS_ERROR("Hour: %i, Min: %i, Sec: %i", GNSSPositionDataMsg.HOURS, GNSSPositionDataMsg.MINUTES,GNSSPositionDataMsg.SECONDS);

}/*

/**************************************************************
   * Parse GPS string of type GPVTG 
  **************************************************************/
/*void RtkGps::processGPS_Serial_GPVTG(tokenizer& tokens,std::string raw){
	tokenizer::iterator tok_iter = tokens.begin();
	// skip identifier
	tok_iter++;

	// Indicates that the direction is relative to true north.
	CourseSpeedMsg.DIRECTION_REFERENCE = "True";

	// GPVTG,33.9,T,,,000.01,N,000.03,K,R*68
	CourseSpeedMsg.header.stamp = ros::Time::now();
	CourseSpeedMsg.header.frame_id = frame_id_coursespeed;
	GNSSPositionDataMsg.header.stamp = ros::Time::now();
	GNSSPositionDataMsg.header.frame_id =frame_id_pos;

	// Converts string to float. 
	CourseSpeedMsg.COURSE_OVER_GROUND = atof((*tok_iter++).c_str());

	tok_iter++;
	tok_iter++;
	tok_iter++;

	// Converts string to float. 
	CourseSpeedMsg.SPEED_OVER_GROUND = atof((*tok_iter++).c_str());
	//ROS_ERROR("msg: %s",raw.c_str());
	ROS_ERROR("DIRECTION_REFERENCE: %s, COURSE_OVER_GROUND: %f, SPEED_OVER_GROUND: %f", CourseSpeedMsg.DIRECTION_REFERENCE.c_str(), CourseSpeedMsg.COURSE_OVER_GROUND,CourseSpeedMsg.SPEED_OVER_GROUND);
	// Publish courseSpeed
	rtk_gps_pub_course_speed.publish(CourseSpeedMsg);
	//pub_freq.tick();

}*/
/**************************************************************
   * Parse data for Course and Speed message with CAN identifier
   * 0x19F8021D and publish it.
  **************************************************************/
void RtkGps::parseCourseSpeedData(const htf_safe_msgs::can::ConstPtr& rx_msg)
{
	CourseSpeedMsg.header.stamp = ros::Time::now();
	CourseSpeedMsg.header.frame_id = frame_id_gps;
	GNSSPositionDataMsg.header.stamp = ros::Time::now();
	GNSSPositionDataMsg.header.frame_id =frame_id_gps;

	switch((rx_msg->data[0] & 0xC0) >> 6)
	{
		case 0:
			CourseSpeedMsg.DIRECTION_REFERENCE = "True";
			break;
		case 1:
			CourseSpeedMsg.DIRECTION_REFERENCE = "Magnetic";
			break;
		case 2:
			CourseSpeedMsg.DIRECTION_REFERENCE = "Error";
			break;
		case 3:
			CourseSpeedMsg.DIRECTION_REFERENCE = "Null";
			break;
	}

	int16_t value = ((rx_msg->data[3] << 8) | rx_msg->data[2]);
	CourseSpeedMsg.COURSE_OVER_GROUND = boost::lexical_cast<float>(value) * 0.0001;
	value = (rx_msg->data[5] + rx_msg->data[4]);
	CourseSpeedMsg.SPEED_OVER_GROUND = boost::lexical_cast<float>(value) * 0.01;

	rtk_gps_pub_course_speed.publish(CourseSpeedMsg);
	pub_freq.tick();
}

/**************************************************************
   * Store data from the 6 different messages with CAN 
   * identifier 0x19F8021D, based on the Frame_PosDat byte [0]
   * If message nr 6 is recieved, calle the parseGNSSPositonData
   * function.
  **************************************************************/
void RtkGps::StoreGNSSPositionData(const htf_safe_msgs::can::ConstPtr& rx_msg)
{
	int i;
	switch(rx_msg->data[0] & 0x0F)
	{
		case 0:
			for(i = 0; i < 8; i++ )
				GNSSFrame0[i] = rx_msg->data[i];
			break;
		case 1:
			for(i = 0; i < 8; i++ )
				GNSSFrame1[i] = rx_msg->data[i];
			break;
		case 2:
			for(i = 0; i < 8; i++ )
				GNSSFrame2[i] = rx_msg->data[i];
			break;
		case 3:
			for(i = 0; i < 8; i++ )
				GNSSFrame3[i] = rx_msg->data[i];
			break;
		case 4:
			for(i = 0; i < 8; i++ )
				GNSSFrame4[i] = rx_msg->data[i];
			break;
		case 5:
			for(i = 0; i < 8; i++ )
				GNSSFrame5[i] = rx_msg->data[i];
			break;
		case 6:
			for(i = 0; i < 8; i++ )
				GNSSFrame6[i] = rx_msg->data[i];
			parseGNSSPositonData(rx_msg);
			break;	
	}		

}

/**************************************************************
   * Parse data from the 6 messages with CAN identifier 0x19F8021D
   * and publish it.
  **************************************************************/
void RtkGps::parseGNSSPositonData(const htf_safe_msgs::can::ConstPtr& rx_msg)
{
	CourseSpeedMsg.header.stamp = ros::Time::now();
	CourseSpeedMsg.header.frame_id = frame_id_gps;

	cout.precision(16);

	uint32_t postime = (GNSSFrame1[1] << 24) | (GNSSFrame0[7] << 16) | (GNSSFrame0[6] << 8) | GNSSFrame0[5];
	time_t seconds = ((time_t)((GNSSFrame0[4] << 8) | GNSSFrame0[3])*86400) + ((postime) * 0.0001);  

	struct tm * ptm;
  	ptm = gmtime ( &seconds );

  	GNSSPositionDataMsg.YEAR = ptm->tm_year +1900;
  	GNSSPositionDataMsg.MONTH = ptm->tm_mon;
  	GNSSPositionDataMsg.DAY = ptm->tm_mday;
  	GNSSPositionDataMsg.HOURS = ptm->tm_hour;
  	GNSSPositionDataMsg.MINUTES = ptm->tm_min;
  	GNSSPositionDataMsg.SECONDS = ptm->tm_sec;


	uint32_t lat = (GNSSFrame2[2] << 24) | (GNSSFrame2[1] << 16) | (GNSSFrame1[7] << 8) | GNSSFrame1[6];
	GNSSPositionDataMsg.Latitude = boost::lexical_cast<double>(lat) * 0.000000429497;

	uint32_t longitude = (GNSSFrame3[3] << 24) | (GNSSFrame3[2] << 16) | (GNSSFrame3[1] << 8) | GNSSFrame2[7];
	GNSSPositionDataMsg.Longitude = boost::lexical_cast<double>(longitude) * 0.000000429497;
	
	uint32_t altitude = (GNSSFrame3[7] << 24) | (GNSSFrame3[6] << 16) | (GNSSFrame3[5] << 8) | GNSSFrame3[4];
	GNSSPositionDataMsg.Altitude = boost::lexical_cast<double>(altitude) * 0.000001;

	GNSSPositionDataMsg.TypeOfSystem = GNSSFrame4[5] & 0x0F;
	GNSSPositionDataMsg.GNSSMethod = (GNSSFrame4[5] & 0xF0) >> 4;
	GNSSPositionDataMsg.GNSSIntegrity = (GNSSFrame4[6] & 0xC0) >> 4;

	uint16_t HDOP = (GNSSFrame5[2] << 8) | GNSSFrame5[1];
	GNSSPositionDataMsg.HDOP = boost::lexical_cast<double>(HDOP) * 0.01;

	uint16_t PDOP = (GNSSFrame5[4] << 8) | GNSSFrame5[3];
	GNSSPositionDataMsg.PDOP = boost::lexical_cast<double>(PDOP) * 0.01;

	GNSSPositionDataMsg.NumberOfSatelites = GNSSFrame4[7];
	GNSSPositionDataMsg.NumberOfRefStations = GNSSFrame6[2];
	
	rtk_gps_pub_GNSSPositionData.publish(GNSSPositionDataMsg);
}
