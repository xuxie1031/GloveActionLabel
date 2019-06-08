#include "glove_data.h"

GloveActionData::GloveActionData(std::string tag, std::string data_dir, std::vector<std::string> data_files, float frame_duration)
: gal(tag, data_dir), gav(tag), data_files_(data_files), frame_duration_(frame_duration)
{}

GloveActionData::~GloveActionData()
{}

void GloveActionData::data_files_label_viz()
{
    for(auto file : data_files_)
    {
        std::cout << "loading data file ..." << std::endl;
        gal.set_data_file(file);
        std::vector<FormattedData> framed_formatted_data = gal.get_formatted_data();

        std::cout << "ready to label file: " << file << std::endl;
        std::cout << "press any key to continue" << std::endl;
        cv::waitKey();

        ros::Time::init();
        for(auto& formatted_data : framed_formatted_data)
        {
            gal.formatted_data_label();

            gav.set_glove_tfs(formatted_data.name2tfs);
            gav.set_glove_finger_qs(formatted_data.finger_qs);
            gav.set_glove_forces(formatted_data.forces);
            gav.publish_state();

            ros::Duration(frame_duration_).sleep();
        }

        gal.unset_data_file();
        std::cout << "finish label file: " << file << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "label_viz_tac_glove");

    std::string tag = argv[1];
    std::string data_dir = argv[2];
    std::string data_names_file = argv[3];
    float frame_duration;
    std::stringstream ss;
    ss << argv[4]; ss >> frame_duration;

    std::vector<std::string> data_files;
    std::ifstream ifs((data_dir+data_names_file).c_str());
    std::string line;
    while(std::getline(ifs, line))
        data_files.push_back(line);
    ifs.close();
    data_files.pop_back();
    
    GloveActionData gad(tag, data_dir, data_files, frame_duration);
    gad.data_files_label_viz();

    return 0;
}