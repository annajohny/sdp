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

#ifndef TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSESTAMPED_H
#define TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSESTAMPED_H

#include <tug_velocity_observer/VelocityConverterTwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/circular_buffer.hpp>
#include <tug_observer_plugin_utils/differentiation/SimpleLinearRegression.h>
#include <string>

class VelocityConverterPoseStamped : public VelocityConverterTwistStamped
{
    std::string topic_;
    unsigned int window_size_;
    boost::circular_buffer<geometry_msgs::PoseStamped> pose_buffer_;
    boost::shared_ptr<SimpleLinearRegression<double> > linear_x_velocity_calc_;
    boost::shared_ptr<SimpleLinearRegression<double> > linear_y_velocity_calc_;
    boost::shared_ptr<SimpleLinearRegression<double> > linear_z_velocity_calc_;
    boost::shared_ptr<SimpleLinearRegression<double> > angular_x_velocity_calc_;
    boost::shared_ptr<SimpleLinearRegression<double> > angular_y_velocity_calc_;
    boost::shared_ptr<SimpleLinearRegression<double> > angular_z_velocity_calc_;

protected:
    VelocityConverterPoseStamped(XmlRpc::XmlRpcValue params, boost::function<void(MovementReading)> call_back);

public:
    VelocityConverterPoseStamped(XmlRpc::XmlRpcValue params, boost::function<void(MovementReading)> call_back,
                                 SubscriberFacade *plugin_base);

    void PoseStampedCB(const geometry_msgs::PoseStamped &msg);

    virtual std::string getName();
};


#endif  // TUG_VELOCITY_OBSERVER_VELOCITYCONVERTERPOSESTAMPED_H
