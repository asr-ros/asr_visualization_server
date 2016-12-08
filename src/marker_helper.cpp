/**

Copyright (c) 2016, Allgeyer Tobias, Braun Kai, Heller Florian, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "marker_helper.h"
#include <tf/tf.h>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>

namespace visualization_server {

MarkerHelper::MarkerHelper(double marker_lifetime, const std::string &dome_config_path, const std::string &mild_config_path)
    : marker_lifetime_(marker_lifetime),
      dome_config_path_(dome_config_path),
      mild_config_path_(mild_config_path)
{

}


visualization_msgs::MarkerArray MarkerHelper::getAllMarkersDome() {
    visualization_msgs::MarkerArray markers_dome = parseXmlFile(dome_config_path_);
    return markers_dome;
}

visualization_msgs::MarkerArray MarkerHelper::getAllMarkersMild() {

    visualization_msgs::MarkerArray markers_mild = parseXmlFile(mild_config_path_);
    return markers_mild;
}

visualization_msgs::MarkerArray MarkerHelper::parseXmlFile(std::string xml_path) {
    visualization_msgs::MarkerArray markers;

    std::string abs_xml_path;
    if (boost::starts_with(xml_path, ".")) {
        abs_xml_path = ros::package::getPath("visualization_server") + xml_path.substr(1);
    }
    else {
        abs_xml_path = xml_path;
    }

    try {
        rapidxml::file<> xmlFile(abs_xml_path.c_str());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        rapidxml::xml_node<> *root_node = doc.first_node();
        if (root_node) {
            rapidxml::xml_node<> *child_node = root_node->first_node();
            int i = 0;
            while (child_node) {
                rapidxml::xml_attribute<> *name_attribute = child_node->first_attribute("name");
                rapidxml::xml_attribute<> *scale_attribute = child_node->first_attribute("scale");
                rapidxml::xml_attribute<> *mesh_attribute = child_node->first_attribute("mesh");
                rapidxml::xml_attribute<> *mat_attribute = child_node->first_attribute("use_mat");
                if (name_attribute && scale_attribute && mesh_attribute && mat_attribute) {
                    std::string name = name_attribute->value();
                    std::string mesh = mesh_attribute->value();
                    std::string scale = scale_attribute->value();
                    std::string pose_string = child_node->value();
                    bool use_mat = boost::lexical_cast<bool>(mat_attribute->value());
                    std::vector<double> pose_vec;
                    std::vector<double> scale_vec;
                    if (parseDoubleCsv(pose_string, pose_vec, " ,") && pose_vec.size() == 7) {
                        if (parseDoubleCsv(scale, scale_vec, " ,") && scale_vec.size() == 3) {
                            markers.markers.push_back(createMarker(name, mesh, pose_vec, scale_vec, i, use_mat));
                            i++;
                        }
                    }

                }
                child_node = child_node->next_sibling();
            }
        }
    } catch(std::runtime_error err) {
        ROS_DEBUG_STREAM("Can't parse xml-file. Runtime error: " << err.what());
    } catch (rapidxml::parse_error err) {
        ROS_DEBUG_STREAM("Can't parse xml-file Parse error: " << err.what());
    } catch (boost::bad_lexical_cast err) {
        ROS_DEBUG_STREAM("Can't cast use_mat. Cast error: " << err.what());
    }

    return markers;
}

bool MarkerHelper::parseDoubleCsv(std::string csv_in, std::vector<double> &csv_out, std::string delim) {
    std::vector<std::string> strvec;

    boost::algorithm::trim_if(csv_in, boost::algorithm::is_any_of(delim));
    boost::algorithm::split(strvec, csv_in, boost::algorithm::is_any_of(delim), boost::algorithm::token_compress_on);
    for (std::string str : strvec) {
        try {
            csv_out.push_back(boost::lexical_cast<double>(str));
        } catch (boost::bad_lexical_cast err) {
            ROS_DEBUG_STREAM("Can't cast node-value. Cast error: " << err.what());
            return false;
        }
    }
    return true;
}

visualization_msgs::Marker MarkerHelper::createMarker(const std::string &name, const std::string &mesh, const std::vector<double> &pose, const std::vector<double> &scale, int id, bool use_mat) {
    visualization_msgs::Marker m;

    m.header.frame_id = "/map";
    m.type = visualization_msgs::Marker::MESH_RESOURCE;
    m.action = visualization_msgs::Marker::ADD;
    m.color.a = 1.0;
    m.color.r = 0.65;
    m.color.g = 0.65;
    m.color.b = 0.65;
    m.lifetime = ros::Duration(marker_lifetime_);
    m.mesh_resource = mesh;
    m.mesh_use_embedded_materials = use_mat;
    m.id = id;
    m.ns = name;
    m.scale.x = scale.at(0);
    m.scale.y = scale.at(1);
    m.scale.z = scale.at(2);
    m.pose.position.x = pose.at(0);
    m.pose.position.y = pose.at(1);
    m.pose.position.z = pose.at(2);
    m.pose.orientation.w = pose.at(3);
    m.pose.orientation.x = pose.at(4);
    m.pose.orientation.y = pose.at(5);
    m.pose.orientation.z = pose.at(6);

    return m;
}

}
