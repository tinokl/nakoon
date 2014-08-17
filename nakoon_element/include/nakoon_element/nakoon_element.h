/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Johannes Maurer,
 *                      Institute for Software Technology,
 *                      Graz University of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * All advertising materials mentioningfeatures or use of this
 *     software must display the following acknowledgement: “This product
 *     includes software developed by Graz University of Technology and
 *     its contributors.”
 *   * Neither the name of Graz University of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef nakoon_element_h___
#define nakoon_element_h___

#include <ros/ros.h>

#include <nakoon_element/nakoon_element_config.h>
#include <nakoon_element/nakoon_element_state.h>

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

const unsigned char MESSAGE_DELIMITER = static_cast<unsigned char>(13);
const unsigned char ALL_MOTOR_ADDR = static_cast<unsigned char>(128);
const unsigned char RIGHT_MOTOR_ADDR = static_cast<unsigned char>(129);
const unsigned char LEFT_MOTOR_ADDR = static_cast<unsigned char>(130);

const int32_t OVERFLOW_INDOCATOR = 2100000000;
const int32_t MAX_INT32 = std::numeric_limits<int32_t>::max();
const int32_t MIN_INT32 = std::numeric_limits<int32_t>::min();

namespace NakoonElement
{

class NakoonElement
{

protected:

	ros::NodeHandle nh_;

    int fd_;							// File descriptor
    char const* file_name_;			// Name of the port we will be using
    unsigned char buf_[10];			// Buffer for data being read/ written on the i2c bus

    NakoonElementConfig robot_config_;
    NakoonElementState robot_state_;

    int dead_man_counter_;
    bool emergency_stop_;
    int32_t left_track_vel_;
    int32_t right_track_vel_;

    boost::asio::streambuf buffer_;
    boost::thread read_thread_;

    nav_msgs::Odometry odometry_msg_;
    ros::Publisher odometry_pub_;
    geometry_msgs::TransformStamped odometry_tf_;
    tf::TransformBroadcaster odometry_tf_broadcaster_;

public:
    NakoonElement(ros::NodeHandle nh);

    virtual ~NakoonElement();

    void init(const NakoonElementConfig &robot_config);

    void setVelocity(double trans, double rot);

    void sendCmd();

    void emergencyStop(bool stop);

    NakoonElementConfig getRobotConfig();

    NakoonElementState getRobotState();


protected:

    void readCallback(const boost::system::error_code &error, std::size_t bytes_transferred);

    long readEncoderValues(void); // Reads encoder data for both motors and displays to the screen

    void resetEncoders(void);	  // Resets the encoders to 0

};

} // end namespace NakoonElement

#endif // nakoon_element_h___
