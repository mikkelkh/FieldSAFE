/****************************************************************************
# Copyright (c) 2016, author Mikkel Kragh Hansen
# mkha@eng.au.dk
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


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

// ROS
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include "htf_delphi_esr/Delphi_radar.h"

#define PI 3.141592653589793f

struct color
{
	float r;
	float g;
	float b;
};
color colors[16] = {{1,0,0},{0,1,0},{0,0,1},{0,0.4470,0.7410},{0.8500,0.3250,0.0980},{0.9290,0.6940,0.1250},{0.4940,0.1840,0.5560},{0.4660,0.6740,0.1880},{0.3010,0.7450,0.9330},{0.6350,0.0780,0.1840},{1,1,0},{1,0,1},{0,1,1},{0.5,0.5},{0.5,0,0.5},{0,0.5,0.5}};

ros::Publisher pubVisualization;
double pitch_angle,height;

struct radar_data
{
	unsigned char CAN_TX_F_VALID_DETECTION_LL;
	unsigned char CAN_TX_DETECT_STATUS_LL;
	double CAN_TX_DET_AMPLITUTDE_LL;
	double CAN_TX_DET_ANGLE_LL;
	double CAN_TX_DET_RANGE_LL;
	double CAN_TX_DET_RANGE_RATE_LL;
	unsigned char CAN_TX_F_VALID_DETECTION_ML;
	unsigned char CAN_TX_DETECT_STATUS_ML;
	double CAN_TX_DET_AMPLITUTDE_ML;
	double CAN_TX_DET_ANGLE_ML;
	double CAN_TX_DET_RANGE_ML;
	double CAN_TX_DET_RANGE_RATE_ML;
};

std::vector<radar_data> msg2array(const htf_delphi_esr::Delphi_radarConstPtr msg)
{
	std::vector<radar_data> data_array;
	radar_data data;
	for (int i=0;i<16;i++)
		data_array.push_back(data);

	// Target 1
	data_array[0].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_1;
	data_array[0].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_1;
	data_array[0].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_1;
	data_array[0].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_1;
	data_array[0].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_1;
	data_array[0].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_1;
	data_array[0].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_1;
	data_array[0].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_1;
	data_array[0].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_1;
	data_array[0].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_1;
	data_array[0].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_1;
	data_array[0].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_1;

	// Target 2
	data_array[1].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_2;
	data_array[1].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_2;
	data_array[1].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_2;
	data_array[1].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_2;
	data_array[1].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_2;
	data_array[1].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_2;
	data_array[1].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_2;
	data_array[1].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_2;
	data_array[1].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_2;
	data_array[1].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_2;
	data_array[1].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_2;
	data_array[1].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_2;

	// Target 3
	data_array[2].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_3;
	data_array[2].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_3;
	data_array[2].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_3;
	data_array[2].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_3;
	data_array[2].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_3;
	data_array[2].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_3;
	data_array[2].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_3;
	data_array[2].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_3;
	data_array[2].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_3;
	data_array[2].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_3;
	data_array[2].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_3;
	data_array[2].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_3;

	// Target 4
	data_array[3].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_4;
	data_array[3].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_4;
	data_array[3].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_4;
	data_array[3].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_4;
	data_array[3].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_4;
	data_array[3].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_4;
	data_array[3].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_4;
	data_array[3].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_4;
	data_array[3].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_4;
	data_array[3].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_4;
	data_array[3].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_4;
	data_array[3].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_4;

	// Target 5
	data_array[4].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_5;
	data_array[4].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_5;
	data_array[4].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_5;
	data_array[4].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_5;
	data_array[4].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_5;
	data_array[4].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_5;
	data_array[4].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_5;
	data_array[4].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_5;
	data_array[4].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_5;
	data_array[4].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_5;
	data_array[4].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_5;
	data_array[4].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_5;

	// Target 6
	data_array[5].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_6;
	data_array[5].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_6;
	data_array[5].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_6;
	data_array[5].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_6;
	data_array[5].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_6;
	data_array[5].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_6;
	data_array[5].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_6;
	data_array[5].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_6;
	data_array[5].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_6;
	data_array[5].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_6;
	data_array[5].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_6;
	data_array[5].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_6;

	// Target 7
	data_array[6].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_7;
	data_array[6].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_7;
	data_array[6].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_7;
	data_array[6].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_7;
	data_array[6].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_7;
	data_array[6].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_7;
	data_array[6].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_7;
	data_array[6].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_7;
	data_array[6].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_7;
	data_array[6].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_7;
	data_array[6].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_7;
	data_array[6].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_7;

	// Target 8
	data_array[7].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_8;
	data_array[7].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_8;
	data_array[7].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_8;
	data_array[7].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_8;
	data_array[7].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_8;
	data_array[7].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_8;
	data_array[7].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_8;
	data_array[7].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_8;
	data_array[7].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_8;
	data_array[7].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_8;
	data_array[7].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_8;
	data_array[7].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_8;

	// Target 9
	data_array[8].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_9;
	data_array[8].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_9;
	data_array[8].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_9;
	data_array[8].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_9;
	data_array[8].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_9;
	data_array[8].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_9;
	data_array[8].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_9;
	data_array[8].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_9;
	data_array[8].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_9;
	data_array[8].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_9;
	data_array[8].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_9;
	data_array[8].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_9;

	// Target 10
	data_array[9].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_10;
	data_array[9].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_10;
	data_array[9].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_10;
	data_array[9].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_10;
	data_array[9].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_10;
	data_array[9].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_10;
	data_array[9].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_10;
	data_array[9].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_10;
	data_array[9].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_10;
	data_array[9].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_10;
	data_array[9].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_10;
	data_array[9].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_10;

	// Target 11
	data_array[10].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_11;
	data_array[10].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_11;
	data_array[10].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_11;
	data_array[10].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_11;
	data_array[10].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_11;
	data_array[10].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_11;
	data_array[10].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_11;
	data_array[10].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_11;
	data_array[10].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_11;
	data_array[10].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_11;
	data_array[10].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_11;
	data_array[10].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_11;

	// Target 12
	data_array[11].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_12;
	data_array[11].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_12;
	data_array[11].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_12;
	data_array[11].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_12;
	data_array[11].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_12;
	data_array[11].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_12;
	data_array[11].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_12;
	data_array[11].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_12;
	data_array[11].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_12;
	data_array[11].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_12;
	data_array[11].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_12;
	data_array[11].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_12;

	// Target 13
	data_array[12].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_13;
	data_array[12].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_13;
	data_array[12].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_13;
	data_array[12].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_13;
	data_array[12].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_13;
	data_array[12].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_13;
	data_array[12].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_13;
	data_array[12].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_13;
	data_array[12].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_13;
	data_array[12].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_13;
	data_array[12].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_13;
	data_array[12].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_13;

	// Target 14
	data_array[13].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_14;
	data_array[13].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_14;
	data_array[13].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_14;
	data_array[13].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_14;
	data_array[13].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_14;
	data_array[13].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_14;
	data_array[13].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_14;
	data_array[13].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_14;
	data_array[13].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_14;
	data_array[13].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_14;
	data_array[13].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_14;
	data_array[13].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_14;

	// Target 15
	data_array[14].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_15;
	data_array[14].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_15;
	data_array[14].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_15;
	data_array[14].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_15;
	data_array[14].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_15;
	data_array[14].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_15;
	data_array[14].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_15;
	data_array[14].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_15;
	data_array[14].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_15;
	data_array[14].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_15;
	data_array[14].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_15;
	data_array[14].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_15;

	// Target 16
	data_array[15].CAN_TX_F_VALID_DETECTION_LL = msg->CAN_TX_F_VALID_DETECTION_LL_16;
	data_array[15].CAN_TX_DETECT_STATUS_LL = msg->CAN_TX_DETECT_STATUS_LL_16;
	data_array[15].CAN_TX_DET_AMPLITUTDE_LL = msg->CAN_TX_DET_AMPLITUTDE_LL_16;
	data_array[15].CAN_TX_DET_ANGLE_LL = msg->CAN_TX_DET_ANGLE_LL_16;
	data_array[15].CAN_TX_DET_RANGE_LL = msg->CAN_TX_DET_RANGE_LL_16;
	data_array[15].CAN_TX_DET_RANGE_RATE_LL = msg->CAN_TX_DET_RANGE_RATE_LL_16;
	data_array[15].CAN_TX_F_VALID_DETECTION_ML = msg->CAN_TX_F_VALID_DETECTION_ML_16;
	data_array[15].CAN_TX_DETECT_STATUS_ML = msg->CAN_TX_DETECT_STATUS_ML_16;
	data_array[15].CAN_TX_DET_AMPLITUTDE_ML = msg->CAN_TX_DET_AMPLITUTDE_ML_16;
	data_array[15].CAN_TX_DET_ANGLE_ML = msg->CAN_TX_DET_ANGLE_ML_16;
	data_array[15].CAN_TX_DET_RANGE_ML = msg->CAN_TX_DET_RANGE_ML_16;
	data_array[15].CAN_TX_DET_RANGE_RATE_ML = msg->CAN_TX_DET_RANGE_RATE_ML_16;

	return data_array;
}

double minML,maxML,minLL,maxLL;


void messageHandler(const htf_delphi_esr::Delphi_radarConstPtr msg)
{
//	ROS_INFO("Radar data received");
	std::vector<radar_data> data = msg2array(msg);

	visualization_msgs::MarkerArray markers;
	visualization_msgs::Marker markerML;
	visualization_msgs::Marker markerLL;

	markerLL.header.frame_id = msg->header.frame_id;
	markerLL.header.stamp = msg->header.stamp;
	markerLL.ns = "my_namespace";
	markerLL.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

	markerLL.type = visualization_msgs::Marker::CYLINDER;
	markerLL.action = visualization_msgs::Marker::ADD;

	markerLL.pose.orientation.x = 0.0;
	markerLL.pose.orientation.y = 0.0;
	markerLL.pose.orientation.z = 0.0;
	markerLL.pose.orientation.w = 1.0;
	markerLL.color.a = 1.0; // Don't forget to set the alpha!

	markerML = markerLL;

	std::vector<geometry_msgs::Point> detections;

	// Remove all markers from previous frame
	markerML.action = visualization_msgs::Marker::DELETEALL;
	markers.markers.push_back(markerML);
	markerML.action = visualization_msgs::Marker::ADD;

	for (int i=0;i<data.size();i++)
	{
		markerML.id = i;
		markerLL.id = 16+i;

		double rangeML = data[i].CAN_TX_DET_RANGE_ML;
		double angleML = data[i].CAN_TX_DET_ANGLE_ML*PI/180;
		double rangeLL = data[i].CAN_TX_DET_RANGE_LL;
		double angleLL = data[i].CAN_TX_DET_ANGLE_LL*PI/180;

		// Convert from polar to cartesian coordinates (and include potential pitch angle of sensor)
		double maxPlanarDistance = height*tan(PI/2-pitch_angle);
		double planarDistanceML = rangeML*cos(pitch_angle);
		geometry_msgs::Point detML;
		detML.x = planarDistanceML*cos(angleLL);
		detML.y = planarDistanceML*sin(angleLL);
		double planarDistanceLL = rangeLL*cos(pitch_angle);
		geometry_msgs::Point detLL;
		detLL.x = planarDistanceLL*cos(angleLL);
		detLL.y = planarDistanceLL*sin(angleLL);



		markerML.pose.position.x = detML.x;
		markerML.pose.position.y = detML.y;
		markerML.pose.position.z = 0.0;

		markerLL.pose.position.x = detLL.x;
		markerLL.pose.position.y = detLL.y;
		markerLL.pose.position.z = 0.0;

		double amplitudeML = data[i].CAN_TX_DET_AMPLITUTDE_ML;
		double amplitudeLL = data[i].CAN_TX_DET_AMPLITUTDE_LL;

		if (amplitudeML < minML)
			minML = amplitudeML;
		if (amplitudeML > maxML)
			maxML = amplitudeML;
		if (amplitudeLL < minLL)
			minLL = amplitudeLL;
		if (amplitudeLL > maxLL)
			maxLL = amplitudeLL;

		double scaleML = 1.0;
		markerML.scale.x = scaleML;
		markerML.scale.y = scaleML;
		markerML.scale.z = scaleML;

		double scaleLL = 1.0;
		markerLL.scale.x = scaleLL;
		markerLL.scale.y = scaleLL;
		markerLL.scale.z = scaleLL;

		markerML.color.r = 0;
		markerML.color.g = 1;
		markerML.color.b = 0;
		markerML.color.a = (amplitudeML-minML)/(maxML-minML);

		markerLL.color.r = 1;
		markerLL.color.g = 0;
		markerLL.color.b = 0;
		markerLL.color.a = (amplitudeLL-minLL)/(maxLL-minLL);

		if (data[i].CAN_TX_F_VALID_DETECTION_ML && data[i].CAN_TX_DETECT_STATUS_ML)
			detections.push_back(detML);

		if (data[i].CAN_TX_F_VALID_DETECTION_LL && data[i].CAN_TX_DETECT_STATUS_LL)
			detections.push_back(detLL);

		markers.markers.push_back(markerML);
		markers.markers.push_back(markerLL);
	}

	pubVisualization.publish( markers );
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "htf_delphi_esr_node");
	ros::NodeHandle n("~");

	minML = std::numeric_limits<double>::max();
	maxML = -std::numeric_limits<double>::max();
	minLL = std::numeric_limits<double>::max();
	maxLL = -std::numeric_limits<double>::max();

	std::string topic_delphi_radar,topic_visualization;

	n.param<std::string>("topic_delphi_radar", topic_delphi_radar, "/Delphi_ESR/RadarData");
	n.param<std::string>("topic_visualization", topic_visualization, "/Delphi_ESR/markers");

	// Setup
	n.param<double>("pitch_angle",pitch_angle,0);
	n.param<double>("height",height,0);

	ros::Subscriber subGps_course_speed = n.subscribe<htf_delphi_esr::Delphi_radar>(topic_delphi_radar, 2, messageHandler);
	pubVisualization = n.advertise<visualization_msgs::MarkerArray>(topic_visualization,1);

	ros::spin();

	return 0;
}
