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

#include <cmath>

#include "sat_nav.h"
#include "nmea/conversion.h"

using namespace sat_nav;

SatNav::SatNav()
{
}
SatNav::~SatNav()
{
}

/*
 * GPGGA - lat, lon, alt, gps_qual, hdop
 * GPRMC - lat, lon, speed, track, date, utc_seconds, position_status
 * GPGST - lat_dev, lon_dev, alt_dev
 * GPZDA - utc_seconds, day, month, year
 * GPVTG - speed_k, track_t
 *
 * Twist
 *  linear      : data from both GPVTG and GPRMC can be used.
 *
 * NavSatFix:
 *  status      : GPGGA.gps_qual provides better info then GPRMC.position_status.
 *  latitude    : GPGGA and GPRMC are both OK.
 *  longitude   : GPGGA and GPRMC are both OK.
 *  altitude    : only provided by GPGGA.alt.
 *  covariance  : need both GPGGA.hdop and GPGST to compute.
 *
 * TimeReference:
 *  time_ref    : GPZDA's utc_seconds does not include ms, hence utc_seconds from others is preferred.
 */

void SatNav::addData(nmea_msgs::Gpgga &gga)
{
    if (!isUtcSecondsSet)
    {
        utcSeconds = gga.utc_seconds;
        isUtcSecondsSet = true;
    }
    if (!isLatLonAltSet)
    {
        latitude = nmea::latLonToDeg(gga.lat, gga.lat_dir);
        longitude = nmea::latLonToDeg(gga.lon, gga.lon_dir);
        isLatLonAltSet = true;
    }
    altitude = gga.alt;
    gpsQual = gga.gps_qual;
    isGpsQualSet = true;

    hdop = gga.hdop;
    isHdopSet = true;
}

void SatNav::addData(nmea_msgs::Gpgst &gst)
{
    if (!isUtcSecondsSet)
    {
        utcSeconds = gst.utc_seconds;
        isUtcSecondsSet = true;
    }
    latDev = gst.lat_dev;
    lonDev = gst.lon_dev;
    altDev = gst.alt_dev;
    isDevSet = true;
}

void SatNav::addData(nmea_msgs::Gprmc &rmc)
{
    if (!isUtcSecondsSet)
    {
        utcSeconds = rmc.utc_seconds;
        isUtcSecondsSet = true;
    }
    if (!isLatLonAltSet)
    {
        latitude = nmea::latLonToDeg(rmc.lat, rmc.lat_dir);
        longitude = nmea::latLonToDeg(rmc.lon, rmc.lon_dir);
        isLatLonAltSet = true;
    }
    if (!isSpeedTrackSet)
    {
        speed = nmea::knotsToKilometersPerHour(rmc.speed);
        track = rmc.track;
        isSpeedTrackSet = true;
    }
    date = rmc.date;
    if (!isGpsQualSet)
    {
        gpsQual = rmc.position_status == "A" ? 1 : 0;
        isGpsQualSet = true;
    }
}

void SatNav::addData(nmea_msgs::Gpzda &zda)
{
    if (!isUtcSecondsSet)
    {
        utcSeconds = zda.utc_seconds;
    }
    day = zda.day;
    month = zda.month;
    year = zda.year;
}

void SatNav::addData(nmea_msgs::Gpvtg &vtg)
{
    if (isSpeedTrackSet)
    {
        return;
    }
    speed = vtg.speed_k;
    track = vtg.track_t;
    isSpeedTrackSet = true;
}

void SatNav::setTwist(geometry_msgs::Twist &twist)
{
    twist.linear.x = speed * sin(track);
    twist.linear.y = speed * cos(track);
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
}

void SatNav::setNavSatFix(sensor_msgs::NavSatFix &fix)
{
    sensor_msgs::NavSatStatus status;
    switch (gpsQual)
    {
    case 1:
        status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        break;
    case 2:
        status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
        break;
    case 4:
    case 5:
        status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
        break;
    default:
        status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        break;
    }
    status.service = 1;

    fix.status = status;
    fix.latitude = latitude;
    fix.longitude = longitude;
    fix.altitude = altitude;

    if (isDevSet && isHdopSet)
    {
        fix.position_covariance[0] = pow(hdop * latDev, 2);
        fix.position_covariance[4] = pow(hdop * lonDev, 2);
        fix.position_covariance[8] = pow(hdop * altDev, 2);
        fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    }
    else
    {
        fix.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }
}

void SatNav::setTimeReference(sensor_msgs::TimeReference &tref)
{
    if (year > 0)
    {
        double unixTs = nmea::toUnixTimestamp(year, month, day, utcSeconds);
        tref.time_ref = ros::Time(unixTs);
    }
    else if (!date.empty())
    {
        double unixTs = nmea::toUnixTimestamp(date, utcSeconds);
        tref.time_ref = ros::Time(unixTs);
    }
    else
    {
        return;
    }
    tref.source = "gps";
}
