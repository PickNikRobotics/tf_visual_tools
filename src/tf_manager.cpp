/****************************************************************************************************
 *  Software License Agreement (BSD License)
 *  
 *  Copyright 2017, Andy McEvoy
 *  
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 *  provided that the following conditions are met:
 *  
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *  and the following disclaimer.
 *  
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of
 *  conditions and the following disclaimer in the documentation and/or other materials provided
 *  with the distribution.
 *  
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to
 *  endorse or promote products derived from this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 *  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 *  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************************************/

/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc      : implementation of rviz_tf_publisher.h. See *.h file for documentation.
 * Created   : 2017 - May - 18
 */

#include <tf_visual_tools/tf_manager.h>

namespace tf_visual_tools
{

RvizTFPublisher::RvizTFPublisher()
  : nh_("~")
{
  create_tf_sub_ = nh_.subscribe("/rviz_tf_create", 10, &RvizTFPublisher::createTF, this);
  remove_tf_sub_ = nh_.subscribe("/rviz_tf_remove", 10, &RvizTFPublisher::removeTF, this);
  update_tf_sub_ = nh_.subscribe("/rviz_tf_update", 10, &RvizTFPublisher::updateTF, this);
}

void RvizTFPublisher::createTF(geometry_msgs::TransformStamped create_tf_msg)
{
  active_tfs_.push_back(create_tf_msg);
}

void RvizTFPublisher::removeTF(geometry_msgs::TransformStamped remove_tf_msg)
{
  for (std::size_t i = 0; i < active_tfs_.size(); i++)
  {
    if (remove_tf_msg.child_frame_id.compare(active_tfs_[i].child_frame_id) == 0 &&
        remove_tf_msg.header.frame_id.compare(active_tfs_[i].header.frame_id) == 0)
    {
      active_tfs_.erase(active_tfs_.begin() + i);
      break;
    }                                            
  }
}

void RvizTFPublisher::updateTF(geometry_msgs::TransformStamped update_tf_msg)
{
  for (std::size_t i = 0; i < active_tfs_.size(); i++)
  {
    if (update_tf_msg.child_frame_id.compare(active_tfs_[i].child_frame_id) == 0 &&
        update_tf_msg.header.frame_id.compare(active_tfs_[i].header.frame_id) == 0)
    {
      active_tfs_[i].transform = update_tf_msg.transform;
    }                                            
  }
}

void RvizTFPublisher::publishTFs()
{
  static tf::TransformBroadcaster br;

  for (std::size_t i = 0; i < active_tfs_.size(); i++)
  {
    tf::StampedTransform tf;
    active_tfs_[i].header.stamp = ros::Time::now();
    tf::transformStampedMsgToTF(active_tfs_[i], tf);
    br.sendTransform(tf);
  } 
}
} // end namespace tf_visual_tools
