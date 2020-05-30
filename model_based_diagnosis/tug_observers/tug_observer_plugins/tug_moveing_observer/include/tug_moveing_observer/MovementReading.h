/*
This file is part of the tug model based diagnosis software for robots
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TUG_MOVEING_OBSERVER_MOVEMENTREADING_H
#define TUG_MOVEING_OBSERVER_MOVEMENTREADING_H

#include <ros/time.h>
#include <geometry_msgs/TwistStamped.h>

struct ThreeAxisMovement
{
    double x;
    double y;
    double z;
};

struct LinearVelocityReading : public ThreeAxisMovement
{ };

struct AngularVelocityReading : public ThreeAxisMovement
{ };

struct MovementReading
{
    ros::Time reading_time;
    ros::Time plot_time;
    LinearVelocityReading linear;
    AngularVelocityReading angular;

    geometry_msgs::TwistStamped toTwistMsg()
    {
      geometry_msgs::TwistStamped result;
      result.header.stamp = plot_time;
      result.twist.angular.x = angular.x;
      result.twist.angular.y = angular.y;
      result.twist.angular.z = angular.z;
      result.twist.linear.x = linear.x;
      result.twist.linear.y = linear.y;
      result.twist.linear.z = linear.z;

      return result;
    }
};


#endif  // TUG_MOVEING_OBSERVER_MOVEMENTREADING_H
