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

#include <people_prediction/constant_velocity_prediction.h>

using namespace human_aware_navigation;

ConstantVelocityPrediction::ConstantVelocityPrediction(std::string name)
{
  //initialize publishers and subscribers
  people_sub_ = nh_.subscribe("people", 1, &ConstantVelocityPrediction::peopleCallback, this);
  prediction_pub_ = nh_.advertise<people_msgs::PeoplePrediction>("people_prediction", 1);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("prediction_viz", 1);

  //get the prediction settings from ros params
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("time_resolution", time_resolution_, 0.5);
  private_nh.param("num_predictions", num_predictions_, 20);

  //initialize the marker for the people prediction
  prediction_marker_.type = visualization_msgs::Marker::SPHERE;
  prediction_marker_.action = visualization_msgs::Marker::MODIFY;
  prediction_marker_.ns = "predictor";
  prediction_marker_.pose.orientation.w = 1;
  prediction_marker_.color.r = 0;
  prediction_marker_.color.g = 0;
  prediction_marker_.color.b = 0.5;
  prediction_marker_.scale.x = 0.2;
  prediction_marker_.scale.y = 0.2;
  prediction_marker_.scale.z = 0.2;
}

void ConstantVelocityPrediction::peopleCallback(people_msgs::People msg)
{
  //parse the people message and predict the trajectories of all people
  people_msgs::People people = msg;
  people_msgs::Person person;
  people_msgs::People people_one_timestep;
  people_msgs::Person person_one_timestep;
  people_msgs::PeoplePrediction predictions;

  visualization_msgs::MarkerArray markers;

  //if the message stamp is empty, we assign the current time
  if(people.header.stamp == ros::Time(0))
  {
    people.header.stamp = ros::Time::now();
  }

  //in this loop, we take the people from the message and their velocities and
  //fill the prediction container
  for(int i=0; i<num_predictions_; i++)
  {
    people_one_timestep.people.clear();

    //loop for the number of people in the environment
    for(int j=0; j<people.people.size(); j++)
    {
      person = people.people.at(j);

      //calculate the position for time step i
      person_one_timestep.position.x = person.position.x +
          (i * time_resolution_ * person.velocity.x);
      person_one_timestep.position.y = person.position.y +
          (i * time_resolution_ * person.velocity.y);
      person_one_timestep.position.z = person.position.z +
          (i * time_resolution_ * person.velocity.z);

      //the velocity stays the same
      person_one_timestep.velocity = person.velocity;

      //fill the header
      people_one_timestep.header.frame_id = people.header.frame_id;
      people_one_timestep.header.stamp = people.header.stamp +
          ros::Duration(i * time_resolution_);

      //push back the prediction step
      people_one_timestep.people.push_back(person_one_timestep);

      //create the prediction marker
      prediction_marker_.header.frame_id = people.header.frame_id;
      prediction_marker_.header.stamp = people.header.stamp;
      prediction_marker_.id = i;

      prediction_marker_.pose.position = person_one_timestep.position;
      //the opacity of the marker is adjusted according to the prediction step
      prediction_marker_.color.a = 1- (i * 1.0 / (num_predictions_ * 1.0));

      markers.markers.push_back(prediction_marker_);
    }
    //push back the predictions for this time step to the prediction container
    predictions.predicted_people.push_back(people_one_timestep);
  }

  //publish predictions and prediction markers
  prediction_pub_.publish(predictions);
  marker_pub_.publish(markers);
}

int main(int argc, char **argv)
{
  std::string node_name = "constant_velocity_prediction";
  ros::init(argc, argv, node_name);

  ConstantVelocityPrediction vel_prediction(node_name);

  ros::spin();
  return 0;
}
