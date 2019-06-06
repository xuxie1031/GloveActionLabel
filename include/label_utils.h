#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#define ForceNum 26
#define BottleTFNum 19

class GloveActionLabel{
public:
    GloveActionLabel(std::string save_dir, std::vector<std::string> data_files, float frame_step);
    ~GloveActionLabel();

    void file_action_label();

private:
    std::string save_dir_;
    std::vector<std::string> data_files_;
    std::vector<std::string> tf_frames_;
    float frame_step_;
};