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

#include <nakoon_element/nakoon_element.h>

#include <boost/regex.hpp>

using namespace std;

namespace NakoonElement
{

/****************************************************************
 *
 */
NakoonElement::NakoonElement(ros::NodeHandle nh) :
    nh_(nh),
    io_service_(),
    serial_port_(io_service_),
    dead_man_counter_(0),
    emergency_stop_(false),
    left_track_vel_(0),
    right_track_vel_(0)
{
    robot_state_.x = 0.0;
    robot_state_.y = 0.0;
    robot_state_.theta = 0.0;
}

/****************************************************************
 *
 */
NakoonElement::~NakoonElement()
{
    serial_port_.close();
    io_service_.stop();
    read_thread_.join();
}

/****************************************************************
 *
 */
void NakoonElement::init(const NakoonElementConfig &robot_config)
{
    robot_config_ = robot_config;

    // new
    fileName = "/dev/i2c-1";	// Name of the port we will be using
    address = 0x58;			// Address of MD25 shifted one bit
    // new


    try
    {
        odometry_msg_.header.frame_id = robot_config_.frame_id;
        odometry_msg_.child_frame_id = robot_config_.child_frame_id;

        odometry_tf_.header.frame_id = robot_config_.frame_id;
        odometry_tf_.child_frame_id = robot_config_.child_frame_id;

    	if ((fd = open(fileName, O_RDWR)) < 0) {					// Open port for reading and writing
    		ROS_ERROR("Failed to open i2c port\n");
    		exit(1);
    	}

    	if (ioctl(fd, I2C_SLAVE, address) < 0) {					// Set the port options and set the address of the device we wish to speak to
    		ROS_ERROR("Unable to get bus access to talk to slave\n");
    		exit(1);
    	}

    	buf[0] = 13;												// This is the register we wish to read software version from

    	if ((write(fd, buf, 1)) != 1) {								// Send regiter to read software from from
    		ROS_ERROR("Error writing to i2c slave\n");
    		exit(1);
    	}

    	if (read(fd, buf, 1) != 1) {								// Read back data into buf[]
    		ROS_ERROR("Unable to read from slave\n");
    		exit(1);
    	}
    	else {
    		ROS_ERROR("Software version: %u\n", buf[0]);
    	}

    	buf[0] = 15;												// Mode Register
    	buf[1] = 1;												    // Set Mode to 1

    	if ((write(fd, buf, 2)) != 2) {
    		ROS_ERROR("Error writing to i2c slave\n");
    		exit(1);
    	}

    	resetEncoders();											// Reset the encoder values to 0


        odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
    }
    catch (boost::system::system_error& error)
    {
        ROS_ERROR_STREAM("NakoonElement::init" << error.what());
    }
}

/****************************************************************
 *
 */
void NakoonElement::run()
{
    io_service_.run();
}

/****************************************************************
 *
 */
void  NakoonElement::setVelocity(double trans, double rot)
{
	// Mode 1
	// speed registers are interpreted as signed values.
	// literal speeds in the range of
	// -128 (Full Reverse)
	// 0 (Stop)
	// 127 (Full Forward).

    rot = rot * robot_config_.rotation_correction;

    double vel_left  = trans - 0.5 * rot * robot_config_.wheel_base;
    double vel_right = trans + 0.5 * rot * robot_config_.wheel_base;

    left_track_vel_ =  (int32_t) (vel_left * robot_config_.velocity_raw_factor);
    right_track_vel_ =  (int32_t) (vel_right * robot_config_.velocity_raw_factor);

    if(std::abs(left_track_vel_) > robot_config_.max_trans_velocity)
    {
    	left_track_vel_ = robot_config_.max_trans_velocity * left_track_vel_/std::abs(left_track_vel_);
        ROS_WARN_STREAM("NakoonElement::setVelocity: Velocity to high: " << left_track_vel_ << ">" << robot_config_.max_trans_velocity);
    }

    if(std::abs(right_track_vel_) > robot_config_.max_trans_velocity)
    {
    	right_track_vel_ = robot_config_.max_trans_velocity * right_track_vel_/std::abs(right_track_vel_);
        ROS_WARN_STREAM("NakoonElement::setVelocity: Velocity to high: " << right_track_vel_ << ">" << robot_config_.max_trans_velocity);
    }

    ROS_ERROR_STREAM("LEFT VEL: " << left_track_vel_);
    ROS_ERROR_STREAM("RIGHT VEL: " << right_track_vel_);

    dead_man_counter_ = 0;
}

void  NakoonElement::sendCmd()
{
	buf[0] = 0;													// Register to set speed of motor 1
	buf[1] = left_track_vel_;												// speed to be set

	if ((write(fd, buf, 2)) != 2) {
		ROS_ERROR("Error writing to i2c slave\n");
		exit(1);
	}

	buf[0] = 1;													// motor 2 speed
	buf[1] = right_track_vel_;

	if ((write(fd, buf, 2)) != 2) {
		ROS_ERROR("Error writing to i2c slave\n");
		exit(1);
	}


	/*
	++dead_man_counter_;

    if(emergency_stop_ || dead_man_counter_ >= robot_config_.max_dead_man)
    {
        left_track_vel_ = 0;
        right_track_vel_ = 0;
    }

    try
    {
        stringstream msg_stream;
        msg_stream << RIGHT_MOTOR_ADDR << "p=" << left_track_vel_ << MESSAGE_DELIMITER << RIGHT_MOTOR_ADDR << "s=" << right_track_vel_ << MESSAGE_DELIMITER << RIGHT_MOTOR_ADDR << "m=4" << MESSAGE_DELIMITER;

        sendMessage(msg_stream.str());
    }
    catch (boost::system::system_error& error)
    {
        ROS_ERROR_STREAM("NakoonElement::setVelocity: " << error.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("NakoonElement::setVelocity: " << "Unhandled exception!");
    }
    */
}

/****************************************************************
 *
 */
NakoonElementConfig NakoonElement::getRobotConfig()
{
    return robot_config_;
}

/****************************************************************
 *
 */
NakoonElementState NakoonElement::getRobotState()
{
    return robot_state_;
}

/****************************************************************
 *
 */
void NakoonElement::emergencyStop(bool stop)
{
    ROS_DEBUG_STREAM("NakoonElement::emergencyStop: " << stop);
    emergency_stop_ = stop;
    sendCmd();
}

/****************************************************************
 *
 */
void  NakoonElement::readCallback(const boost::system::error_code &error_code, std::size_t bytes_transferred)
{
	/*
    ros::Time now = ros::Time::now();
    if(error_code)
    {
        ROS_ERROR_STREAM("NakoonElement::readCallback: " << boost::system::system_error(error_code).what());
    }
    else
    {
        istream is(&buffer_);
        string new_msg;
        getline(is, new_msg);

        ROS_DEBUG_STREAM("NakoonElement::readCallback: " << new_msg);

        try
        {
            boost::regex reg_ex("ll=(\\-?\\d+)\\$rr=(\\-?\\d+)");
            boost::cmatch matches;

            if(boost::regex_search(new_msg.c_str(), matches, reg_ex))
            {
                int32_t left_pose =  boost::lexical_cast<int32_t>(matches[1].str());
                int32_t right_pose = boost::lexical_cast<int32_t>(matches[2].str());

                static bool first_call = true;

                if (first_call)
                {
                    robot_state_.last_msg_time = ros::Time::now();
                    robot_state_.last_left_position = left_pose;
                    robot_state_.last_right_position = right_pose;
                    first_call = false;
                }
                else
                {
                    int32_t delta_left_pose;
                    if(robot_state_.last_left_position > OVERFLOW_INDOCATOR && left_pose < -OVERFLOW_INDOCATOR) // positiv overflow
                    {
                        delta_left_pose = (left_pose - MIN_INT32) + (MAX_INT32 - robot_state_.last_left_position);
                    }
                    else if(robot_state_.last_left_position < -OVERFLOW_INDOCATOR && left_pose > OVERFLOW_INDOCATOR) // negativ overflow
                    {
                        delta_left_pose = (left_pose - INT32_MAX) + (INT32_MIN - robot_state_.last_left_position);
                    }
                    else
                    {
                        delta_left_pose = left_pose - robot_state_.last_left_position;
                    }

                    int32_t delta_right_pose;
                    if(robot_state_.last_right_position > OVERFLOW_INDOCATOR && right_pose < -OVERFLOW_INDOCATOR) // positiv overflow
                    {
                        delta_right_pose = (right_pose - LONG_MIN) + (LONG_MAX - robot_state_.last_right_position);
                    }
                    else if(robot_state_.last_right_position < -OVERFLOW_INDOCATOR && right_pose > OVERFLOW_INDOCATOR) // negativ overflow
                    {
                        delta_right_pose = (right_pose - LONG_MAX) + (LONG_MIN - robot_state_.last_right_position);
                    }
                    else
                    {
                        delta_right_pose = right_pose - robot_state_.last_right_position;
                    }

                    robot_state_.last_left_position = left_pose;
                    robot_state_.last_right_position = right_pose;

                    double delta_left = (double) delta_left_pose * robot_config_.raw_odometry_factor;
                    double delta_right = (double) delta_right_pose * robot_config_.raw_odometry_factor;

                    ROS_DEBUG_STREAM("NakoonElement::readCallback: delta_left= " << delta_left << " delta_right= " << delta_right);

                    double delta_trans = (delta_left + delta_right) / 2;
                    double delta_rot = (delta_right - delta_left) / robot_config_.wheel_base;
                    ros::Duration delta_t = now - robot_state_.last_msg_time;

                    ROS_DEBUG_STREAM("NakoonElement::readCallback: delta_trans= " << delta_trans << " delta_rot= " << delta_rot);

                    double delta_x, delta_y;
                    if(delta_rot >= 0.0005)
                    {
                        delta_x = -delta_trans/delta_rot * sin(robot_state_.theta) + delta_trans/delta_rot * sin(robot_state_.theta + delta_rot);
                        delta_y =  delta_trans/delta_rot * cos(robot_state_.theta) - delta_trans/delta_rot * cos(robot_state_.theta + delta_rot);
                    }
                    else
                    {
                        delta_x = delta_trans * cos(robot_state_.theta);
                        delta_y = delta_trans * sin(robot_state_.theta);
                    }

                    robot_state_.last_msg_time = now;
                    robot_state_.x += delta_x;
                    robot_state_.y += delta_y;
                    robot_state_.theta += delta_rot;
                    robot_state_.trans = delta_trans / delta_t.toSec();
                    robot_state_.rot = delta_rot / delta_t.toSec();

                    odometry_msg_.header.seq++;
                    odometry_msg_.header.stamp = now;
                    odometry_msg_.pose.pose.position.x = robot_state_.x;
                    odometry_msg_.pose.pose.position.y = robot_state_.y;
                    odometry_msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_state_.theta);
                    odometry_msg_.twist.twist.linear.x = robot_state_.trans;
                    odometry_msg_.twist.twist.angular.z = robot_state_.rot;

                    odometry_pub_.publish(odometry_msg_);

                    if(robot_config_.publish_tf)
                    {
                        odometry_tf_.header = odometry_msg_.header;
                        odometry_tf_.transform.translation.x = robot_state_.x;
                        odometry_tf_.transform.translation.y = robot_state_.y;
                        odometry_tf_.transform.rotation = tf::createQuaternionMsgFromYaw(robot_state_.theta);

                        odometry_tf_broadcaster_.sendTransform(odometry_tf_);
                    }
                }

            }
            else
            {
                ROS_WARN_STREAM(new_msg << " do not match " << reg_ex);
            }
        }
        catch (boost::regex_error& ex)
        {
            ROS_FATAL_STREAM("NakoonElement::readCallback: " << "This should not happen! RegEx is not valid");
        }
    }

    boost::asio::async_read_until(serial_port_, buffer_, MESSAGE_DELIMITER, boost::bind(&NakoonElement::readCallback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    */
}

void NakoonElement::resetEncoders(void) {
	buf[0] = 16;												// Command register
	buf[1] = 32;												// command to set decoders back to zero

	if ((write(fd, buf, 2)) != 2) {
		printf("Error writing to i2c slave\n");
		exit(1);
	}
}

long NakoonElement::readEncoderValues (void) {

	long encoder1, encoder2;

	buf[0] = 2;													// register for start of encoder values

	if ((write(fd, buf, 1)) != 1) {
		printf("Error writing to i2c slave\n");
		exit(1);
	}

	if (read(fd, buf, 8) != 8) {								// Read back 8 bytes for the encoder values into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}
	else {
		encoder1 = (buf[0] <<24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];	// Put encoder values together
		encoder2 = (buf[4] <<24) + (buf[5] << 16) + (buf[6] << 8) + buf[7];
		printf("Encoder 1: %08lX   Encoder 2: %08lX\n",encoder1, encoder2);
	}
	return encoder1;
}

} // end namespace NakoonElement


/*

int main(int argc, char **argv)
{
	printf("**** MD25 test program ****\n");

	if ((fd = open(fileName, O_RDWR)) < 0) {					// Open port for reading and writing
		printf("Failed to open i2c port\n");
		exit(1);
	}

	if (ioctl(fd, I2C_SLAVE, address) < 0) {					// Set the port options and set the address of the device we wish to speak to
		printf("Unable to get bus access to talk to slave\n");
		exit(1);
	}

	buf[0] = 13;												// This is the register we wish to read software version from

	if ((write(fd, buf, 1)) != 1) {								// Send regiter to read software from from
		printf("Error writing to i2c slave\n");
		exit(1);
	}

	if (read(fd, buf, 1) != 1) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}
	else {
		printf("Software version: %u\n", buf[0]);
	}

	resetEncoders();											// Reset the encoder values to 0

	while(readEncoderValues() < 0x2000) {						// Check the value of encoder 1 and stop after it has traveled a set distance
		 driveMotors();
		 usleep(200000);										// This sleep just gives us a bit of time to read what was printed to the screen in driveMortors()
	}

	stopMotors();
	return 0;
}

*/
