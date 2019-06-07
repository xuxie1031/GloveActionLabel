#include "label_utils.h"

GloveActionLabel::GloveActionLabel(std::string tag, std::string save_dir, std::string data_file, float frame_step)
: tag_(tag), save_dir_(save_dir), data_file_(data_file), frame_step_(frame_step)
{
    ofs_label_.open((save_dir_+data_file_+"_label").c_str());

    std::ifstream ifs((save_dir_+data_file_).c_str());
    std::string line;

    while(std::getline(ifs, line))
    {
        std::stringstream ss;
        std::vector<std::string> fields;
        boost::algorithm::split(fields, line, boost::is_any_of(","));

        FormattedData fd;

        int offset_idx = 3;
        for(int i=0; i<BottleTFNum; i++)
        {
            int curr_idx = offset_idx+9*i;
            std::string parent_frame = fields[curr_idx];
            std::string child_frame = fields[curr_idx+1];

            float tx, ty, tz;
            ss.clear(); ss.str(""); ss << fields[curr_idx+2];
            ss >> tx;
            ss.clear(); ss.str(""); ss << fields[curr_idx+3];
            ss >> ty;
            ss.clear(); ss.str(""); ss << fields[curr_idx+4];
            ss >> tz;

            float rx, ry, rz, rw;
            ss.clear(); ss.str(""); ss << fields[curr_idx+5];
            ss >> rx;
            ss.clear(); ss.str(""); ss << fields[curr_idx+6];
            ss >> ry;
            ss.clear(); ss.str(""); ss << fields[curr_idx+7];
            ss >> rz;
            ss.clear(); ss.str(""); ss << fields[curr_idx+8];
            ss >> rw;

            if(parent_frame.compare("world") == 0 && child_frame.compare("vicon/wrist/wrist") == 0)
            {
                fd.name2tfs["world_wrist_tf"] = tf::Transform(tf::Quaternion(rx, ry, rz, rw), tf::Vector3(tx, ty, tz));
            }
            else if(parent_frame.compare("vicon/wrist/wrist") == 0 && child_frame.compare("vicon/bottle"+tag_+"/bottle"+tag_) == 0)
            {
                fd.name2tfs["wrist_bottle_tf"] = tf::Transform(tf::Quaternion(rx, ry, rz, rw), tf::Vector3(tx, ty, tz));
            }
            else if(parent_frame.compare("vicon/wrist/wrist") == 0 && child_frame.compare("vicon/wrist/wrist", "vicon/bottle"+tag_+"_lid"+"/bottle"+tag_+"_lid") == 0)
            {
                fd.name2tfs["wrist_lid_tf"] = tf::Transform(tf::Quaternion(rx, ry, rz, rw), tf::Vector3(tx, ty, tz));
            }
            else if(parent_frame.compare("vicon/wrist/wrist") == 0 && child_frame.compare("glove_link") == 0)
            {
                fd.name2tfs["wrist_glove_tf"] = tf::Transform(tf::Quaternion(rx, ry, rz, rw), tf::Vector3(tx, ty, tz));
            }
            else if(parent_frame.compare("glove_link") == 0 && child_frame.compare("palm_link") == 0)
            {
                fd.name2tfs["glove_palm_tf"] = tf::Transform(tf::Quaternion(rx, ry, rz, rw), tf::Vector3(tx, ty, tz));
            }
            else
            {
                fd.finger_qs.push_back(tf::Quaternion(rx, ry, rz, rw));
            }
        }

        offset_idx += 9*BottleTFNum;
        for(int j=0; j<ForceNum; j++)
        {
            int curr_idx = offset+j;
            float f;
            ss.clear(); ss.str(""); ss << fields[curr_idx];
            ss >> f;
            fd.forces.push_back(f);
        }

        formatted_data_.push_back(fd);
    }

    cv::namedWindow("Label Window");
}

GloveActionLabel::~GloveActionLabel()
{
    ofs_label_.close();
}

std::vector<FormattedData>& GloveActionLabel::get_formatted_data() const
{
    return formatted_data_;
}

void GloveActionLabel::formatted_data_label()
{
    char key = cv::waitKey(1);
    switch(key)
    {
        case '0':
            tag_ = "0";
            break;
        case '1':
            tag_ = "1";
            break;
        case '2':
            tag_ = "2";
            break;
        case '3':
            tag_ = "3";
            break;
        case '4':
            tag_ = "4";
            break;
        case '5':
            tag_ = "5";
            break;
        default:
            break;
    }
    ofs_label_ << tag_ << ",";
}