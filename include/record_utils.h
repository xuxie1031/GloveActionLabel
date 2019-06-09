#pragma once

#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>

#include <ros/ros.h>
#include <tf/tranform_broadcaster.h>

#define FingerNum 14
#define ForceNum 26

class GloveActionRecord{
public:
    GloveActionRecord(std::string data_dir);
    ~GloveActionRecord();

    void record_state();
    void record_state_finger(int idx_from, int idx_to);

    void set_glove_tfs(const std::unordered_map<std::string, tf::Transform> &name2tfs);
    void set_glove_finger_qs(const std::vector<tf::Quaternion> &finger_qs);
    void set_glove_forces(const std::vector<float> &forces);

    void set_data_file(std::string data_file);
    void unset_data_file();

private:
    std::ofstream ofs_data_;
    std::string data_dir_;

    std::unordered_map<std::string, tf::Transform> name2tfs_;
    std::vector<tf::Quaternion> finger_qs_;
    std::vector<float> forces_;

    std::vector<int> parents_;
    std::vector<tf::Vector3> canonical_origin_; 

    const float Pi = 3.1415926;
    const float PalmWidth = .075;
    const float PalmLength = .075;
    const float PalmHeight = .020;
    const float ProximalPhalangeLength = .030;
    const float ProximalPhalanxLength = .030;
    const float MiddlePhalangeLength = .025;
    const float DistalPhalangeLength = .021;
    const float DistalPhalanxLength = .028;
    const float Radius = .006;
    const float ArrayLength = .07;
};