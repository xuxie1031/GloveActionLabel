#include "record_utils.h"

static tf::Vector3 quaternion_rotate(tf::Quaternion q, tf::Vector3 u)
{
    tf::Quaternion q_u(u.getX(),u.getY(),u.getZ(),0.0);
	tf::Quaternion q_v=q*(q_u*q.inverse());
	tf::Vector3 v(q_v.x(),q_v.y(),q_v.z());

	return v;
}

void GloveActionRecord::GloveActionRecord(std::string data_dir)
: data_dir_(data_dir)
{
    parents_ = {-1, 0, 1, 0, 3, 4, 0, 6, 7, 0, 9, 10, 0, 12, 13};

    canonical_origin_ = {
        tf::Vector3(0.0, 0.0, 0.0),
        tf::Vector3(-.0*PalmLength, 2*(PalmWidth/4), -0.3*PalmHeight),
        tf::Vector3(ProximalPhalanxLength, 0, 0),
        tf::Vector3(PalmLength/2, 1.5*(PalmWidth/4), 0),
        tf::Vector3(ProximalPhalangeLength, 0, 0),
        tf::Vector3(MiddlePhalangeLength, 0, 0),
        tf::Vector3(PalmLength/2, 0.5*(PalmWidth/4), 0),
        tf::Vector3(ProximalPhalangeLength, 0, 0), 
        tf::Vector3(MiddlePhalangeLength, 0, 0),
        tf::Vector3(PalmLength/2, -0.5*(PalmWidth/4), 0),
        tf::Vector3(ProximalPhalangeLength, 0, 0),
        tf::Vector3(MiddlePhalangeLength, 0, 0),
        tf::Vector3(PalmLength/2, -1.5*(PalmWidth/4), 0),
        tf::Vector3(ProximalPhalangeLength, 0, 0),
        tf::Vector3(MiddlePhalangeLength, 0, 0)
    };
}

void GloveActionRecord::~GloveActionRecord()
{}

void GloveActionRecord::set_glove_tfs(const std::unordered_map<std::string, tf::Transform> &name2tfs)
{
    name2tfs_ = name2tfs;
}

void GloveActionRecord::set_glove_finger_qs(const std::vector<tf::Quaternion> &finger_qs)
{
    finger_qs_ = finger_qs;
}

void GloveActionRecord::set_glove_forces(const std::vector<float> &forces)
{
    forces_ = forces;
}

void GloveActionRecord::record_state_finger(int idx_from, int idx_to)
{
    if(idx_to > FingerNum) return;
    tf::Transform hand_tf;
    tf::Vector3 t_link(0.0, 0.0, 0.0);
    for(int i=idx_from; i<=idx_to; i++)
    {
        tf::Quaternion r_link = finger_qs_[i];
        t_link += quaternion_rotate(finger_qs_[parents_[i]], canonical_origin_[i]);

        hand_tf.setOrigin(t_link);
        hand_tf.setRotation(r_link);
        ofs_data_ << hand_tf.getOrigin().getX() << "," << hand_tf.getOrigin().getY() << "," << hand_tf.getOrigin().getZ() << ",";
    }
}

void GloveActionRecord::record_state()
{
    // everything regarding wrist

    tf::Transform wrist_bottle_tf = name2tfs_["wrist_bottle_tf"];
    ofs_data_ << wrist_bottle_tf.getOrigin().getX() << "," << wrist_bottle_tf.getOrigin().getY() << "," << wrist_bottle_tf.getOrigin().getZ() << ",";
    
    tf::Transform wrist_lid_tf = name2tfs_["wrist_lid_tf"];
    ofs_data_ << wrist_lid_tf.getOrigin().getX() << "," << wrist_lid_tf.getOrigin().getY() << "," << wrist_lid_tf.getOrigin().getZ() << ",";

    tf::Transform wrist_glove_tf = name2tfs_["wrist_glove_tf"];
    ofs_data_ << wrist_glove_tf.getOrigin().getX() << "," << wrist_glove_tf.getOrigin().getY() << "," << wrist_glove_tf.getOrigin().getZ() << ",";

    tf::Transform glove_palm_tf = name2tfs_["glove_palm_tf"];
    ofs_data_ << glove_palm_tf.getOrigin().getX() << "," << glove_palm_tf.getOrigin().getY() << "," << glove_palm_tf.getOrigin().getZ() << ",";

    record_state_finger(1, 2);
    record_state_finger(3, 5);
    record_state_finger(6, 8);
    record_state_finger(9, 11);
    record_state_finger(12, 14);
}

void GloveActionRecord::set_data_file(std::string data_file)
{
    ofs_data_.open((data_dir+data_file+"_formatted.csv").c_str());
}

void GloveActionRecord::unset_data_file()
{
    ofs_data_.close();
}