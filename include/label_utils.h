#pragma once

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

#define FingerNum 14
#define ForceNum 26
#define BottleTFNum 19

struct FormattedData{
    std::unordered_map<std::string, tf::Transform> name2tfs;
    std::vector<tf::Quaternion> finger_qs;
    std::vector<float> forces;
};

class GloveActionLabel{
public:
    GloveActionLabel(std::string tag, std::string data_dir);
    ~GloveActionLabel();

    void formatted_data_label();
    void set_data_file(std::string data_file);
    void unset_data_file();
    std::vector<FormattedData>& get_formatted_data();

private:
    std::string tag_;
    std::string data_dir_;
    std::vector<FormattedData> formatted_data_;
    std::ofstream ofs_label_;
    std::string label_tag_;
};