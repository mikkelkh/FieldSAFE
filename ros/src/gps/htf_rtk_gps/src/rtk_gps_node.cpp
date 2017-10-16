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

int main(int argc, char **argv){

  ros::init(argc, argv, "htf_delphi_esr");
  ros::NodeHandle nh("~");

  RtkGps rtk_gps;

  string subscribe_can_topic_id;
  string subscribe_serial_topic_id;
  string publish_topic_id_Course_Speed;
  string publish_topic_id_Time;
  string publish_topic_id_Position;
  string publish_topic_id_GNSS_position_data;
  string publish_topic_id_SerialSentence;

  rtk_gps.updater.setHardwareID("RTK GPS");
  rtk_gps.updater.add("RTK GPS", &rtk_gps, &RtkGps::diagnostics);

  // Get setting from parameterserver
  nh.param<std::string> ("rtk_pub_course_speed", publish_topic_id_Course_Speed, "/Trimble_rtk_gps/course_speed");
  nh.param<std::string> ("rtk_pub_GNSS_position_data", publish_topic_id_GNSS_position_data, "/Trimble_rtk_gps/GNSS_position_data");
  nh.param<std::string> ("rtk_pub_SerialSentence", publish_topic_id_SerialSentence, "/Trimble_rtk_gps/SerialSentence");
  nh.param<std::string> ("can_rx_topic", subscribe_can_topic_id, "/fmLib/can_rx");		 // Substribe to can
  nh.param<std::string> ("serial_rx_topic", subscribe_serial_topic_id, "/fmLib/serial_rx"); // Subscribe to serial
  nh.param<std::string> ("frame_id", rtk_gps.frame_id_gps, "RTK_GPS");
  
  // Publish rtk_gps
  rtk_gps.rtk_gps_pub_course_speed = nh.advertise<htf_safe_msgs::Course_Speed>(publish_topic_id_Course_Speed, 1);
  rtk_gps.rtk_gps_pub_GNSSPositionData = nh.advertise<htf_safe_msgs::GNSSPositionData>(publish_topic_id_GNSS_position_data, 1);
  rtk_gps.rtk_gps_pub_serialSentence = nh.advertise<nmea_msgs::Sentence>(publish_topic_id_SerialSentence, 1);

  // Subscibe to can_rx
  ros::Subscriber sub_can = nh.subscribe(subscribe_can_topic_id, 1, &RtkGps::getGPSData_CAN, &rtk_gps);
  // Subscibe to serial_rx
  ros::Subscriber sub_serial = nh.subscribe(subscribe_serial_topic_id, 1, &RtkGps::getGPSData_Serial, &rtk_gps);

  ros::spin();
  //while(ros::ok()){

    //ros::spinOnce();
    //rtk_gps.updater.update();
  //}
  
  return 0;
}
