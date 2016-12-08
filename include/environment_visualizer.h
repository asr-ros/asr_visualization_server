/**

Copyright (c) 2016, Allgeyer Tobias, Braun Kai, Heller Florian, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef ENVIRONMENT_VISUALIZER_H
#define ENVIRONMENT_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_server/DrawAllModelsDome.h>
#include <visualization_server/DrawAllModelsMild.h>
#include <visualization_server/ClearAllModels.h>
#include <visualization_server/DrawModelDome.h>
#include <visualization_server/DrawModelMild.h>
#include <visualization_server/ClearModelDome.h>
#include <visualization_server/ClearModelMild.h>
#include <visualization_server/ShowAvailableModels.h>

#include "marker_helper.h"


namespace visualization_server {

/** The name of this package **/
const static std::string NODE_NAME("visualization");

/** The name of service used for visualizing all models of the dome environment **/
const static std::string DRAW_ALL_DOME_SERVICE_NAME("draw_all_models_dome");

/** The name of service used for visualizing all models of the mild environment **/
const static std::string DRAW_ALL_MILD_SERVICE_NAME("draw_all_models_mild");

/** The name of service used for clearing all published models **/
const static std::string CLEAR_ALL_MODELS_SERVICE_NAME("clear_all_models");

/** The name of service used for visualizing a model of the dome environment **/
const static std::string DRAW_MODEL_DOME_SERVICE_NAME("draw_model_dome");

/** The name of service used for visualizing a model of the mild environment **/
const static std::string DRAW_MODEL_MILD_SERVICE_NAME("draw_model_mild");

/** The name of service used for clearing all published models **/
const static std::string CLEAR_MODEL_DOME_SERVICE_NAME("clear_model_dome");

/** The name of service used for clearing all published models **/
const static std::string CLEAR_MODEL_MILD_SERVICE_NAME("clear_model_mild");

/** The name of service used for clearing all published models **/
const static std::string SHOW_AVAILABLE_MODELS_SERVICE_NAME("show_available_models");

class EnvironmentVisualizer {

private:

    /** Ros' interface for creating subscribers, publishers, etc. */
    ros::NodeHandle nh_;

    ros::Timer publish_timer_;

    /** Ros service handlers used for handling requests */
    ros::ServiceServer draw_all_mild_service_;
    ros::ServiceServer draw_all_dome_service_;
    ros::ServiceServer clear_all_service_;
    ros::ServiceServer draw_model_dome_service_;
    ros::ServiceServer draw_model_mild_service_;
    ros::ServiceServer clear_model_dome_service_;
    ros::ServiceServer clear_model_mild_service_;
    ros::ServiceServer show_available_models_service_;

    ros::Publisher model_pub_;

    std::string output_models_topic_;

    MarkerHelper marker_helper_;

    visualization_msgs::MarkerArray dome_markers_;
    visualization_msgs::MarkerArray mild_markers_;


    void timerCallback(const ros::TimerEvent& e);

    /**
     * Processes the request to draw all models of the dome environment
     *
     * \param req       The request message
     * \param res       The correlated response message.
     */
    bool processDrawAllModelsDomeRequest(DrawAllModelsDome::Request &req, DrawAllModelsDome::Response &res);

    /**
     * Processes the request to draw all models of the mild environment
     *
     * \param req       The request message
     * \param res       The correlated response message.
     */
    bool processDrawAllModelsMildRequest(DrawAllModelsMild::Request &req, DrawAllModelsMild::Response &res);

    /**
     * Processes the request to clear all models
     *
     * \param req       The request message
     * \param res       The correlated response message.
     */
    bool processClearAllModelsRequest(ClearAllModels::Request &req, ClearAllModels::Response &res);

    /**
     * Processes the request to draw a model of the dome environment
     *
     * \param req       The request message
     * \param res       The correlated response message.
     */
    bool processDrawModelDomeRequest(DrawModelDome::Request &req, DrawModelDome::Response &res);

    /**
     * Processes the request to draw a model of the mild environment
     *
     * \param req       The request message
     * \param res       The correlated response message.
     */
    bool processDrawModelMildRequest(DrawModelMild::Request &req, DrawModelMild::Response &res);

    /**
     * Processes the request to clear a model of the dome environment
     *
     * \param req       The request message
     * \param res       The correlated response message.
     */
    bool processClearModelDomeRequest(ClearModelDome::Request &req, ClearModelDome::Response &res);

    /**
     * Processes the request to clear a model of the mild environment
     *
     * \param req       The request message
     * \param res       The correlated response message.
     */
    bool processClearModelMildRequest(ClearModelMild::Request &req, ClearModelMild::Response &res);

    /**
     * Processes the request to show all available models
     *
     * \param req       The request message
     * \param res       The correlated response message.
     */
    bool processShowAvailableModelsRequest(ShowAvailableModels::Request &req, ShowAvailableModels::Response &res);




public:
    EnvironmentVisualizer();

};

}

#endif /* ENVIRONMENT_VISUALIZER_H */

