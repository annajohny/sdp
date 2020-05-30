/*
This file is part of the tug model based diagnosis software for robots
Copyright (c) 2015, Clemens Muehlbacher
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_MEAN_VALUE_FILTER_MEANVALUEFILTERWITHBUFFER_H
#define TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_MEAN_VALUE_FILTER_MEANVALUEFILTERWITHBUFFER_H

#include <tug_observer_plugin_utils/filter/value_filter/ValueFilter.h>
#include <boost/circular_buffer.hpp>
#include <tug_yaml/ProcessYaml.h>
#include <numeric>
#include <boost/thread/mutex.hpp>


template<class T>
class MeanValueFilterWithBuffer : public ValueFilter<T>
{
    boost::circular_buffer<T> buffer_;
    boost::mutex scope_mutex_;

public:
    explicit MeanValueFilterWithBuffer(XmlRpc::XmlRpcValue params)
    {
      unsigned int window_size = ProcessYaml::getValue<unsigned int>("window_size", params);
      if (window_size < 2)
        throw std::invalid_argument("mean value filter must be initialized with a window size of at least 2");
      buffer_ = boost::circular_buffer<T>(window_size);
    }

    virtual void update(const T &new_value)
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      buffer_.push_back(new_value);
    }

    virtual T getValue()
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      if (buffer_.empty())
        return static_cast<T>(0);

      T result = std::accumulate(buffer_.begin(), buffer_.end(), static_cast<T>(0));
      return result / static_cast<T>(buffer_.size());
    }

    virtual void reset()
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      buffer_.clear();
    }

    virtual size_t getSampleSize()
    {
      boost::mutex::scoped_lock scoped_lock(scope_mutex_);
      return buffer_.size();
    }
};

#endif  // TUG_OBSERVER_PLUGIN_UTILS_FILTER_VALUE_FILTER_MEAN_VALUE_FILTER_MEANVALUEFILTERWITHBUFFER_H
