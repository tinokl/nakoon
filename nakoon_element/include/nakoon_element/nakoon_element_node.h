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

#ifndef nakoon_element_node_h___
#define nakoon_element_node_h___

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include <nakoon_element/nakoon_element.h>


namespace NakoonElement
{

class NakoonElementNode
{
protected:
    ros::NodeHandle nh_;
    ros::Timer send_cmd_timer_;

    ros::Publisher odom_pub_;
    ros::Subscriber cmd_vel_sub_;

    ros::Subscriber emergency_stop_sub_;

    std::string frame_id_;

    NakoonElement nakoon_element_;

public:
    NakoonElementNode();

    virtual ~NakoonElementNode();

    void init();


protected:
    void cmdVelCB(const geometry_msgs::TwistConstPtr& msg);

    void emergencyStopCB(const std_msgs::BoolConstPtr& msg);

    void sendCmdCB(const ros::TimerEvent& event);

};

} // end namespace NakoonElement

#endif // nakoon_element_node_h___
