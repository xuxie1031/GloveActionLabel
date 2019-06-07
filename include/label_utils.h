#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#define ForceNum 26
#define BottleTFNum 19

struct FormattedData{
    std::unordered_map<std::string, tf::Transform> name2tfs;
    std::vector<tf::Quaternion> finger_qs;
    std::vector<float> forces;
};

class GloveActionLabel{
public:
    GloveActionLabel(std::string tag, std::string save_dir, std::string data_file, float frame_step);
    ~GloveActionLabel();

    void file_extract_data();
    std::vector<FormattedData>& get_curr_formatted_data() const;

private:
    std::string tag_;
    std::string save_dir_;
    std::string data_file_;
    float frame_step_;
    std::vector<FormattedData> formatted_data_;
    std::ofstream ofs_label_;
};