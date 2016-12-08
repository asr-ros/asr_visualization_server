/**

Copyright (c) 2016, Allgeyer Tobias, Braun Kai, Heller Florian, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef MARKER_HELPER_H
#define MARKER_HELPER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace visualization_server {


class MarkerHelper {

private:

    double DEFAULT_MARKER_LIFETIME = 2.0;

    double marker_lifetime_;

    std::string dome_config_path_;
    std::string mild_config_path_;

    bool parseDoubleCsv(std::string csv_in, std::vector<double> &csv_out, std::string delim);

    visualization_msgs::Marker createMarker(const std::string &name, const std::string &mesh, const std::vector<double> &pose, const std::vector<double> &scale, int id, bool use_mat);

    visualization_msgs::MarkerArray parseXmlFile(std::string xml_path);

public:

    MarkerHelper() : MarkerHelper(DEFAULT_MARKER_LIFETIME, std::string(), std::string()) {}

    MarkerHelper(double marker_lifetime, const std::string &dome_config_path, const std::string &mild_config_path);

    visualization_msgs::MarkerArray getAllMarkersDome();

    visualization_msgs::MarkerArray getAllMarkersMild();

};

}

#endif /* MARKER_HELPER_H */

