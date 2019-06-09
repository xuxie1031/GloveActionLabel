#include "label_utils.h"

GloveActionLabel::GloveActionLabel(std::string tag, std::string data_dir)
: tag_(tag), data_dir_(data_dir), label_tag_("0")
{
    cv::namedWindow("Label Window");
}

GloveActionLabel::~GloveActionLabel()
{
    cv::destroyWindow("Label Window");
}

void GloveActionLabel::set_data_file(std::string data_file)
{
    ofs_data_.open((data_dir_+data_file+"_formatted_data").c_str());
    ofs_label_.open((data_dir_+data_file+"_label").c_str());

    std::ifstream ifs((data_dir_+data_file+".csv").c_str());
    std::string line;

    while(std::getline(ifs, line))
    {
        std::stringstream ss;
        std::vector<std::string> fields;
        boost::algorithm::split(fields, line, boost::is_any_of(","));

        FormattedData fd;
        fd.finger_qs.push_back(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

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
            else if(parent_frame.compare("vicon/wrist/wrist") == 0 && child_frame.compare("vicon/bottle"+tag_+"_lid"+"/bottle"+tag_+"_lid") == 0)
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
            int curr_idx = offset_idx+j;
            float f;
            ss.clear(); ss.str(""); ss << fields[curr_idx];
            ss >> f;
            fd.forces.push_back(f);
        }

        formatted_data_.push_back(fd);
    }
    ifs.close();
}

void GloveActionLabel::unset_data_file()
{
    ofs_data_.close();
    ofs_label_.close();
    formatted_data_.empty();
}

std::vector<FormattedData>& GloveActionLabel::get_formatted_data()
{
    return formatted_data_;
}

void GloveActionLabel::formatted_data_label()
{
    char key = cv::waitKey(1);

    switch(key)
    {
        case '0':
            label_tag_ = "0";
            break;
        case '1':
            label_tag_ = "1";
            break;
        case '2':
            label_tag_ = "2";
            break;
        case '3':
            label_tag_ = "3";
            break;
        case '4':
            label_tag_ = "4";
            break;
        case '5':
            label_tag_ = "5";
            break;
        default:
            break;
    }
    ofs_label_ << label_tag_ << ",";
}