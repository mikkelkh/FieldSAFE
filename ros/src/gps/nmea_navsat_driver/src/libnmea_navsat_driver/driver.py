# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import rospy

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped
from htf_safe_msgs.msg import Course_Speed
from htf_safe_msgs.msg import GPHDT

from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
import libnmea_navsat_driver.parser


class RosNMEADriver(object):
    def __init__(self):
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=1)
        self.vtg_pub = rospy.Publisher('vtg', Course_Speed, queue_size=1)
        self.hdt_pub = rospy.Publisher('hdt', GPHDT, queue_size=1)
        self.vel_pub = rospy.Publisher('vel', TwistStamped, queue_size=1)
        self.time_ref_pub = rospy.Publisher('time_reference', TimeReference, queue_size=1)

        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        self.use_RMC = rospy.get_param('~useRMC', False)
        self.use_GPS_time = rospy.get_param('~useGPStime', False)
        self.time_delay = rospy.get_param('~time_delay',0.0)
        self.time_delay_heading = rospy.get_param('~time_delay_heading',0.0)
        self.gps_time = None

	self.gps_covariance_pos = rospy.get_param('~gps_covariance_pos', [0.0, 0.0, 1.0])

    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not check_nmea_checksum(nmea_string):
            rospy.logwarn("Received a sentence with an invalid checksum. " +
                          "Sentence was: %s" % repr(nmea_string))
            return False
        parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            rospy.logdebug("Failed to parse NMEA sentence. Sentece was: %s" % nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()
        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id
        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = frame_id
        
        if not self.use_RMC and 'GGA' in parsed_sentence:
            #rospy.loginfo('GGA')
            data = parsed_sentence['GGA']
            gps_qual = data['fix_type']
            if gps_qual == 0:
                current_fix.status.status = NavSatStatus.STATUS_NO_FIX
            elif gps_qual == 1:
                current_fix.status.status = NavSatStatus.STATUS_FIX
            elif gps_qual == 2:
                current_fix.status.status = NavSatStatus.STATUS_SBAS_FIX
            elif gps_qual in (4, 5):
                current_fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            else:
                current_fix.status.status = NavSatStatus.STATUS_NO_FIX

            current_fix.status.service = NavSatStatus.SERVICE_GPS

            current_fix.header.stamp = current_time

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            #hdop = data['hdop']
            #current_fix.position_covariance[0] = hdop ** 2
            #current_fix.position_covariance[4] = hdop ** 2
            #current_fix.position_covariance[8] = (2 * hdop) ** 2  # FIXME
            #current_fix.position_covariance_type = \
            #    NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            
            # covariances diagonals as rospy arguments
            current_fix.position_covariance[0] = self.gps_covariance_pos[0]
            current_fix.position_covariance[4] = self.gps_covariance_pos[1]
            current_fix.position_covariance[8] = self.gps_covariance_pos[2]
            current_fix.position_covariance_type =\
                NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            if not math.isnan(data['utc_time']):
                current_time_ref.time_ref = rospy.Time.from_sec(int(int(timestamp.to_sec())/(60*60))*(60*60)+data['utc_time'])
                self.time_ref_pub.publish(current_time_ref)
		if self.use_GPS_time:
                    self.gps_time = current_time_ref.time_ref
                    current_fix.header.stamp = current_time_ref.time_ref
                current_fix.header.stamp = current_fix.header.stamp+rospy.Duration(self.time_delay)
            #print(timestamp.to_sec())
            self.fix_pub.publish(current_fix)

            #if not math.isnan(data['utc_time']):
            #    current_time_ref.time_ref = rospy.Time.from_sec(data['utc_time'])
            #    self.time_ref_pub.publish(current_time_ref)

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']
            #rospy.loginfo('RMC')
            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)

                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rospy.Time.from_sec(data['utc_time'])
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
        elif 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']
            track_made_good_degrees_true = data['track_made_good_degrees_true']
            track_made_good_degrees_magnetic = data['track_made_good_degrees_magnetic']
            speed = data['speed']
            SPEED_OVER_GROUND = data['speed_over_ground']
            if not math.isnan(track_made_good_degrees_true):
                DIRECTION_REFERENCE  = "True"
                COURSE_OVER_GROUND = track_made_good_degrees_true
            elif not math.isnan(track_made_good_degrees_magnetic):
                DIRECTION_REFERENCE  = "Magnetic"
                COURSE_OVER_GROUND = track_made_good_degrees_magnetic
            else:
                DIRECTION_REFERENCE  = "Null"
		COURSE_OVER_GROUND = float(0)
		SPEED_OVER_GROUND = float('NaN')
            current_vtg = Course_Speed()
            current_vtg.header.stamp = current_time
            current_vtg.header.frame_id = frame_id
            current_vtg.DIRECTION_REFERENCE = DIRECTION_REFERENCE
            current_vtg.COURSE_OVER_GROUND  = COURSE_OVER_GROUND 
            current_vtg.SPEED_OVER_GROUND = SPEED_OVER_GROUND
            self.vtg_pub.publish(current_vtg)
            #rospy.loginfo(track_made_good_degrees_magnetic)
        elif 'HDT' in parsed_sentence:
            #rospy.loginfo('HDT')     
            data = parsed_sentence['HDT']
            heading_degrees = data['heading_degrees']
            current_hdt = GPHDT()
            current_hdt.header.stamp = current_time
            current_hdt.header.frame_id = frame_id
            current_hdt.HEADING_DEGREES = heading_degrees
            if self.use_GPS_time and not self.gps_time==None:
                    current_hdt.header.stamp = self.gps_time-rospy.Duration(0,1000000)
            elif self.use_GPS_time:
                    current_hdt.header.stamp.secs = 0
            current_hdt.header.stamp = current_hdt.header.stamp+rospy.Duration(self.time_delay)+rospy.Duration(self.time_delay_heading)
            #print(current_hdt.header.stamp.to_sec())
            self.hdt_pub.publish(current_hdt)
        else:
            rospy.loginfo('Not valid')
            return False

    """Helper method for getting the frame_id with the correct TF prefix"""

    @staticmethod
    def get_frame_id():
        frame_id = rospy.get_param('~frame_id', 'gps')
        if frame_id[0] != "/":
            """Add the TF prefix"""
            prefix = ""
            prefix_param = rospy.search_param('tf_prefix')
            if prefix_param:
                prefix = rospy.get_param(prefix_param)
                if prefix[0] != "/":
                    prefix = "/%s" % prefix
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
