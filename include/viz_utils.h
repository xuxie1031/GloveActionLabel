#pragma once

#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include "fsr_glove/glove.h"

#define FingerNum 14


class GloveActionViz{
public:
    GloveActionViz(std::string tag);
    ~GloveActionViz();

    void publish_state_finger(int idx_from, int idx_to);
    void publish_state_palm();
    void publish_state();

    void set_glove_tfs(const std::unordered_map<std::string, tf::Transform> &name2tfs);
    void set_glove_finger_qs(const std::vector<tf::Quaternion> &finger_qs);
    void set_glove_forces(const std::vector<float> &forces);

private:
    visualization_msgs::Marker genmarker(tf::Vector3 pt_marker, tf::Quaternion q_marker, float length, float radius, float chroma, std::string ns);
    tf::Vector3 quaternion_rotate(tf::Quaternion q, tf::Vector3 u);

    std::vector<std::string> link_names_;
    std::vector<float> link_lengths_;
    std::vector<int> parents_;

    std::vector<tf::Quaternion> canonical_pose_;
    std::vector<tf::Vector3> canonical_origin_;

    std::unordered_map<std::string, tf::Transform> name2tfs_;
    std::vector<tf::Quaternion> finger_qs_;
    std::vector<float> forces_;

    ros::NodeHandle nh_;

    tf::TransformBroadcaster br_;
    ros::Publisher marker_pub_;

    std::string tag_;

    const float Pi = 3.1415926;
    const float PalmWidth = .075;
    const float PalmLength = .075;
    const float PalmHeight = .020;
    const float ProximalPhalangeLength = .030;
    const float ProximalPhalanxLength = .030;
    const float MiddlePhalangeLength = .025;
    const float DistalPhalanxLength = .021;
    const float Radius = .006;
    const float ArrayLength = .07;
};