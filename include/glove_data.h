#pragma once

#include "label_utils.h"
#include "viz_utils.h"
#include "record_utils.h"

#include <iostream>

class GloveActionData{
public:
    GloveActionData(std::string tag, std::string data_dir, std::vector<std::string> data_files, float frame_duration);
    ~GloveActionData();

    void data_files_label_viz_record();

private:
    GloveActionLabel gal;
    GloveActionViz gav;
    GloveActionRecord gar;

    std::vector<std::string> data_files_;
    float frame_duration_;
};