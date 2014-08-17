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

#include <nakoon_element/nakoon_element_node.h>

using namespace std;

namespace NakoonElement
{

/****************************************************************
 *
 */
NakoonElementNode::NakoonElementNode() :
    nakoon_element_(nh_)
{

}

/****************************************************************
 *
 */
NakoonElementNode::~NakoonElementNode()
{

}

/****************************************************************
 *
 */
void NakoonElementNode::init()
{
    ros::NodeHandle private_nh("~");

    NakoonElementConfig robot_config;
    private_nh.param<string>("port", robot_config.port, string("/dev/ttyUSB0"));
    private_nh.param<int>("baud_rate", robot_config.baud_rate, 38400);

    private_nh.param<double>("odometry_rate", robot_config.odometry_rate, 10.0);
    private_nh.param<bool>("publish_tf", robot_config.publish_tf, true);
    private_nh.param<double>("max_trans_velocity", robot_config.max_trans_velocity, 125);
    private_nh.param<double>("max_rot_velocity", robot_config.max_rot_velocity, 1.570796);
    private_nh.param<double>("wheel_base", robot_config.wheel_base, 0.47);
    private_nh.param<double>("rotation_correction", robot_config.rotation_correction, 1.0);
    private_nh.param<double>("velocity_raw_factor", robot_config.velocity_raw_factor, 300.0);
    private_nh.param<double>("raw_odometry_factor", robot_config.raw_odometry_factor, 0.0000016129);
    private_nh.param<double>("cmd_rate", robot_config.cmd_rate, 15.0);

    private_nh.param<int>("max_dead_man", robot_config.max_dead_man, 15);

    private_nh.param<std::string>("frame_id", robot_config.frame_id, std::string("odom"));
    private_nh.param<std::string>("child_frame_id", robot_config.child_frame_id, std::string("robot_footprint"));

    nakoon_element_.init(robot_config);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("new_object",10);

    cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &NakoonElementNode::cmdVelCB, this);

    emergency_stop_sub_ = nh_.subscribe<std_msgs::Bool>("emergency_stop", 1, &NakoonElementNode::emergencyStopCB, this);

    send_cmd_timer_ = nh_.createTimer(ros::Duration(1/robot_config.cmd_rate), &NakoonElementNode::sendCmdCB, this, false);
}

/****************************************************************
 *
 */
void NakoonElementNode::cmdVelCB(const geometry_msgs::TwistConstPtr& msg)
{
    ROS_DEBUG_STREAM_NAMED("NakoonElementNode", "cmd_vel recieved: trans=" << msg->linear.x << " rot=" << msg->angular.z);
    nakoon_element_.setVelocity(msg->linear.x, msg->angular.z);
}


/****************************************************************
 *
 */
void NakoonElementNode::emergencyStopCB(const std_msgs::BoolConstPtr& msg)
{
    nakoon_element_.emergencyStop(msg->data);
}

/****************************************************************
 *
 */
void NakoonElementNode::sendCmdCB(const ros::TimerEvent& event)
{
    nakoon_element_.sendCmd();
}

} // end namespace NakoonElement

/****************************************************************
 *
 */
int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "NakoonElementNode");
        NakoonElement::NakoonElementNode nakoon_element_node;

        nakoon_element_node.init();

        ros::spin();
    }
    catch(...)
    {
        ROS_ERROR_NAMED("NakoonElementNode","Unhandled exception!");
        return -1;
    }

    return 0;
}
