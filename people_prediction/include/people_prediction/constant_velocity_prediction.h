/*******************************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015 Marina Kollmitz
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Marina Kollmitz
 ******************************************************************************/

#ifndef CONSTANT_VELOCITY_PREDICTION_H_
#define CONSTANT_VELOCITY_PREDICTION_H_

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <people_msgs/PeoplePrediction.h>
#include <tf/transform_listener.h>

namespace human_aware_navigation
{

/**
 * @brief The ConstantVelocityPrediction class defines a ros node for the prediction
 *        of the future trajectories of people according to the constant velocity
 *        model.
 *
 * It listens to people_msgs::People messages on the 'people' topic and publishes
 * the predicted trajectories as people_msgs::PeoplePrediction messages
 * on the 'people_prediction' topic. Furthermore, the node publishes
 * visualization_msgs::MarkerArray messages on the 'prediction_viz' topic for
 * visualization. The node assumes that both the positions and the
 * velocities of the people are defined in the coordinate frame specified in the
 * header of the people message. The predicted trajectories are calculated based
 * on the people's current positions and their current velocities.
 */
class ConstantVelocityPrediction
{

public:

  /**
   * @brief constructor
   * @param name node name
   */
  ConstantVelocityPrediction(std::string name);

  /**
   * @brief callback for the detected people message
   * @param msg detected people
   */
  void peopleCallback(people_msgs::People msg);

private:

  /**
   * @brief constructor
   */
  ConstantVelocityPrediction();

  double time_resolution_; ///< time resolution of the predicted trajectories
  int num_predictions_; ///< number of prediction steps for each person

  ros::NodeHandle nh_; ///< ros node handle
  ros::Subscriber people_sub_; ///< subscriber for detected people
  ros::Publisher prediction_pub_; ///< publisher for the predicted trajectories
  ros::Publisher marker_pub_; ///< publisher for the prediction markers
  visualization_msgs::Marker prediction_marker_; ///< visualization marker for predictions
};

}

#endif
