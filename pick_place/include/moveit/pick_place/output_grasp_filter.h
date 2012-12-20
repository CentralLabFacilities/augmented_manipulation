/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
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
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef MOVEIT_PICK_PLACE_OUTPUT_GRASP_FILTER_
#define MOVEIT_PICK_PLACE_OUTPUT_GRASP_FILTER_

#include <moveit/pick_place/grasp_filter.h>
#include <boost/function.hpp>

namespace pick_place
{

class OutputGraspFilter : public GraspFilter
{
public:

  typedef boost::function<void(const Grasp&)> ReceiveOutputCallback;
  
  OutputGraspFilter(const ReceiveOutputCallback &callback = ReceiveOutputCallback()) :
    GraspFilter(1),
    callback_(callback)
  {
  }

  virtual bool evaluate(unsigned int thread_id, const Grasp &grasp) const;
  virtual void push(const Grasp &grasp);
  virtual bool done(void) const;
  
  const std::vector<Grasp>& getOutput(void) const
  {
    return output_;
  }
  
private:

  std::vector<Grasp> output_;
  ReceiveOutputCallback callback_;
};

}

#endif

