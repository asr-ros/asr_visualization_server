/**

Copyright (c) 2016, Allgeyer Tobias, Braun Kai, Heller Florian, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "environment_visualizer.h"
#include "marker_helper.h"
#include <visualization_msgs/MarkerArray.h>

namespace visualization_server {

EnvironmentVisualizer::EnvironmentVisualizer() : nh_(NODE_NAME) {

    double marker_lifetime;
    std::string dome_config_path;
    std::string mild_config_path;
    nh_.getParam("marker_lifetime", marker_lifetime);
    nh_.getParam("dome_config_path", dome_config_path);
    nh_.getParam("mild_config_path", mild_config_path);
    marker_helper_ = MarkerHelper(marker_lifetime, dome_config_path, mild_config_path);

    nh_.getParam("output_topic", output_models_topic_);

    draw_all_dome_service_ = nh_.advertiseService(DRAW_ALL_DOME_SERVICE_NAME, &EnvironmentVisualizer::processDrawAllModelsDomeRequest, this);
    draw_all_mild_service_ = nh_.advertiseService(DRAW_ALL_MILD_SERVICE_NAME, &EnvironmentVisualizer::processDrawAllModelsMildRequest, this);
    clear_all_service_ = nh_.advertiseService(CLEAR_ALL_MODELS_SERVICE_NAME, &EnvironmentVisualizer::processClearAllModelsRequest, this);
    draw_model_dome_service_ = nh_.advertiseService(DRAW_MODEL_DOME_SERVICE_NAME, &EnvironmentVisualizer::processDrawModelDomeRequest, this);
    draw_model_mild_service_ = nh_.advertiseService(DRAW_MODEL_MILD_SERVICE_NAME, &EnvironmentVisualizer::processDrawModelMildRequest, this);
    clear_model_dome_service_ = nh_.advertiseService(CLEAR_MODEL_DOME_SERVICE_NAME, &EnvironmentVisualizer::processClearModelDomeRequest, this);
    clear_model_mild_service_ = nh_.advertiseService(CLEAR_MODEL_MILD_SERVICE_NAME, &EnvironmentVisualizer::processClearModelMildRequest, this);
    show_available_models_service_ = nh_.advertiseService(SHOW_AVAILABLE_MODELS_SERVICE_NAME, &EnvironmentVisualizer::processShowAvailableModelsRequest, this);

    double timer_duration = 0.5;
    nh_.getParam("publish_rate", timer_duration);
    publish_timer_ = nh_.createTimer(ros::Duration(timer_duration), &EnvironmentVisualizer::timerCallback, this);

    model_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(output_models_topic_, 1);
}

void EnvironmentVisualizer::timerCallback(const ros::TimerEvent &e) {
    ROS_DEBUG_STREAM("Publishing environment models");

    std::string dome_marker_names = "[ ";
    for (visualization_msgs::Marker m : dome_markers_.markers) {
        dome_marker_names += m.ns + " ";
    }
    dome_marker_names += "]";
    ROS_DEBUG_STREAM(dome_markers_.markers.size() << " dome markers are published " << dome_marker_names);
    model_pub_.publish(dome_markers_);

    std::string mild_marker_names = "[ ";
    for (visualization_msgs::Marker m : mild_markers_.markers) {
        mild_marker_names += m.ns + " ";
    }
    mild_marker_names += "]";
    ROS_DEBUG_STREAM(mild_markers_.markers.size() << " mild markers are published " << mild_marker_names << std::endl);
    model_pub_.publish(mild_markers_);

}

bool EnvironmentVisualizer::processDrawAllModelsDomeRequest(DrawAllModelsDome::Request &req, DrawAllModelsDome::Response &res) {
    dome_markers_ = marker_helper_.getAllMarkersDome();
    return true;
}

bool EnvironmentVisualizer::processDrawAllModelsMildRequest(DrawAllModelsMild::Request &req, DrawAllModelsMild::Response &res) {
    mild_markers_ = marker_helper_.getAllMarkersMild();
    return true;
}

bool EnvironmentVisualizer::processClearAllModelsRequest(ClearAllModels::Request &req, ClearAllModels::Response &res) {
    dome_markers_ = visualization_msgs::MarkerArray();
    mild_markers_ = visualization_msgs::MarkerArray();
    return true;
}

bool EnvironmentVisualizer::processDrawModelDomeRequest(DrawModelDome::Request &req, DrawModelDome::Response &res) {
    std::string name = req.name;
    for (visualization_msgs::Marker m : dome_markers_.markers) {
        if (name == m.ns) {
            ROS_WARN_STREAM("A marker with the name " << name << " is already being published");
            return true;
        }
    }
    visualization_msgs::MarkerArray marker_array = marker_helper_.getAllMarkersDome();
    bool marker_found = false;
    for (visualization_msgs::Marker m : marker_array.markers) {
        if (name == m.ns) {
            dome_markers_.markers.push_back(m);
            marker_found = true;
            break;
        }
    }
    if (!marker_found) {
        ROS_WARN_STREAM("There is no available marker with the name " << name);
    }
    return true;
}

bool EnvironmentVisualizer::processDrawModelMildRequest(DrawModelMild::Request &req, DrawModelMild::Response &res) {
    std::string name = req.name;
    for (visualization_msgs::Marker m : mild_markers_.markers) {
        if (name == m.ns) {
            ROS_WARN_STREAM("A marker with the name " << name << " is already being published");
            return true;
        }
    }
    visualization_msgs::MarkerArray marker_array = marker_helper_.getAllMarkersMild();
    bool marker_found = false;
    for (visualization_msgs::Marker m : marker_array.markers) {
        if (name == m.ns) {
            mild_markers_.markers.push_back(m);
            marker_found = true;
            break;
        }
    }
    if (!marker_found) {
        ROS_WARN_STREAM("There is no available marker with the name " << name);
    }
    return true;
}

bool EnvironmentVisualizer::processClearModelDomeRequest(ClearModelDome::Request &req, ClearModelDome::Response &res) {
    std::string name = req.name;
    for (std::vector<visualization_msgs::Marker>::iterator iter = dome_markers_.markers.begin(); iter != dome_markers_.markers.end(); ++iter) {
        if (iter->ns == name) {
            dome_markers_.markers.erase(iter);
            break;
        }
    }
    return true;
}

bool EnvironmentVisualizer::processClearModelMildRequest(ClearModelMild::Request &req, ClearModelMild::Response &res) {
    std::string name = req.name;
    for (std::vector<visualization_msgs::Marker>::iterator iter = mild_markers_.markers.begin(); iter != mild_markers_.markers.end(); ++iter) {
        if (iter->ns == name) {
            mild_markers_.markers.erase(iter);
            break;
        }
    }
    return true;
}

bool EnvironmentVisualizer::processShowAvailableModelsRequest(ShowAvailableModels::Request &req, ShowAvailableModels::Response &res) {
    visualization_msgs::MarkerArray dome_markers = marker_helper_.getAllMarkersDome();
    visualization_msgs::MarkerArray mild_markers = marker_helper_.getAllMarkersMild();

    std::vector<std::string> dome_marker_names;
    for (visualization_msgs::Marker m : dome_markers.markers) {
        dome_marker_names.push_back(m.ns);
    }

    std::vector<std::string> mild_marker_names;
    for (visualization_msgs::Marker m : mild_markers.markers) {
        mild_marker_names.push_back(m.ns);
    }

    res.dome_markers = dome_marker_names;
    res.mild_markers = mild_marker_names;

    return true;
}

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "visualization");

    visualization_server::EnvironmentVisualizer env_vis;

    ros::spin();

    return 0;
}
