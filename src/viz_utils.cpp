#include "viz_utils.h"

GloveActionViz::GloveActionViz(std::string tag) : tag_(tag)
{
    link_names_ = {
        "palm_link",
        "proximal_phalanx_link_1",
        "distal_phalanx_link_1",
        "proximal_phalange_link_1",
        "middle_phalange_link_1",
        "distal_phalange_link_1",
        "proximal_phalange_link_2",
        "middle_phalange_link_2",
        "distal_phalange_link_2",
        "proximal_phalange_link_3",
        "middle_phalange_link_3",
        "distal_phalange_link_3",
        "proximal_phalange_link_4",
        "middle_phalange_link_4",
        "distal_phalange_link_4"
    };

    link_lengths_ = {
        -1,
        ProximalPhalanxLength,
        DistalPhalanxLength,
        ProximalPhalangeLength,
        MiddlePhalangeLength,
        DistalPhalangeLength,
        ProximalPhalangeLength,
        MiddlePhalangeLength,
        DistalPhalangeLength,
        ProximalPhalangeLength,
        MiddlePhalangeLength,
        DistalPhalangeLength,
        ProximalPhalangeLength,
        MiddlePhalangeLength,
        DistalPhalangeLength
    };

    parents_ = {-1, 0, 1, 0, 3, 4, 0, 6, 7, 0, 9, 10, 0, 12, 13};

    forces_.resize(ForceNum);

    tf::Quaternion q_ident(0.0, 0.0, 0.0, 1.0);
    tf::Quaternion q_thumb, q_1, q_2, q_3, q_4;
    q_thumb = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0.2*Pi);
    q_1 = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0.05*Pi);
    q_2 = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0.0);
    q_3 = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), -0.05*Pi);
    q_4 = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), -0.1*Pi);

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

    canonical_pose_ = {
        q_ident, q_thumb, q_thumb, q_1, q_1, q_1, q_2, q_2, q_2, q_3, q_3, q_3, q_4, q_4, q_4
    };

    // name2tfs_ = {
    //     {"world_wrist_tf", tf::Transform::getIdentity()},
    //     {"wrist_bottle_tf", tf::Transform::getIdentity()},
    //     {"wrist_lid_tf", tf::Transform::getIdentity()},
    //     {"wrist_glove_tf", tf::Transform::getIdentity()},
    //     {"glove_palm_tf", tf::Transform::getIdentity()}
    // };

    // finger_qs_.resize(FingerNum);
    // std::fill(finger_qs_.begin(), finger_qs.end(), tf::Quaternion(0.0, 0.0, 0.0, 1.0));

    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("force_marker", 1000);
}

GloveActionViz::~GloveActionViz()
{}

void GloveActionViz::set_glove_tfs(const std::unordered_map<std::string, tf::Transform> &name2tfs)
{
    name2tfs_ = name2tfs;
}

void GloveActionViz::set_glove_finger_qs(const std::vector<tf::Quaternion> &finger_qs)
{
    finger_qs_ = finger_qs;
}

void GloveActionViz::set_glove_forces(const std::vector<float> &forces)
{
    forces_ = forces;
}

tf::Vector3 GloveActionViz::quaternion_rotate(tf::Quaternion q, tf::Vector3 u)
{
    tf::Quaternion q_u(u.getX(),u.getY(),u.getZ(),0.0);
	tf::Quaternion q_v=q*(q_u*q.inverse());
	tf::Vector3 v(q_v.x(),q_v.y(),q_v.z());

	return v;
}

visualization_msgs::Marker GloveActionViz::genmarker(tf::Vector3 pt_marker, tf::Quaternion q_marker, float length, float radius, float chroma, std::string ns)
{
    visualization_msgs::Marker cylinder;
  	cylinder.header.frame_id = "palm_link";
  	cylinder.header.stamp = ros::Time::now();
  	cylinder.ns = ns.c_str();
  	cylinder.type = visualization_msgs::Marker::CYLINDER;

  	cylinder.pose.position.x = pt_marker.getX();
  	cylinder.pose.position.y = pt_marker.getY();
  	cylinder.pose.position.z = pt_marker.getZ();

  	cylinder.pose.orientation.w = q_marker.w();
  	cylinder.pose.orientation.x = q_marker.x();
  	cylinder.pose.orientation.y = q_marker.y();
  	cylinder.pose.orientation.z = q_marker.z();

  	cylinder.scale.x = radius * 2;
  	cylinder.scale.y = radius * 2;
  	cylinder.scale.z = length;

	float maxF=50.0f,minF=20.0f;
  	if (chroma > maxF) {
       		chroma = 1.0f;
        	cylinder.color.r = chroma;
        	cylinder.color.g = 0.0f;
        	cylinder.color.b = 0.0f;
        	cylinder.color.a = 1.0f;
  	}
  	else if (chroma < minF) {
        	cylinder.color.r = 0.0f;
        	cylinder.color.g = 1.0f;
        	cylinder.color.b = 0.0f;
        	cylinder.color.a = 1.0f;
  	}
  	else{
        	chroma/=maxF;
        	cylinder.color.r = chroma;
        	cylinder.color.g = 1.0f-chroma;
        	cylinder.color.b = 0.0f;
        	cylinder.color.a = 1.0f;
  	}

	return cylinder;
}

void GloveActionViz::publish_state_finger(int idx_from, int idx_to)
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

        tf::Quaternion q_marker = r_link*tf::Quaternion(tf::Vector3(0.0, 1.0, 0.0), 0.5*Pi);
        tf::Vector3 pt_marker = t_link+quaternion_rotate(q_marker, tf::Vector3(0.0, 0.0, link_lengths_[i]/2));

        std::string ns = "tac_glove_marker_"+link_names_[i];
        float force = (i == idx_from) ? forces_[idx_from*2/3] : forces_[idx_from*2/3+1];

        visualization_msgs::Marker mk = genmarker(pt_marker, q_marker, link_lengths_[i], Radius, force, ns);
        br_.sendTransform(tf::StampedTransform(hand_tf, ros::Time::now(), link_names_[0], link_names_[i]));
        marker_pub_.publish(mk);
    }
}

void GloveActionViz::publish_state_palm()
{
    tf::Vector3 t_link = name2tfs_["glove_palm_tf"].getOrigin();
    tf::Quaternion r_link = name2tfs_["glove_palm_tf"].getRotation();
    br_.sendTransform(tf::StampedTransform(name2tfs_["glove_palm_tf"], ros::Time::now(), "glove_link", "palm_link"));

    float x_center = ArrayLength/2-ArrayLength/8;
    float y_center = -(ArrayLength/2-ArrayLength/8);
    int count = 10;
    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            tf::Vector3 pt_marker = t_link+quaternion_rotate(r_link, tf::Vector3(x_center, y_center, 0.0));
            tf::Quaternion q_marker = r_link;
            std::stringstream ss;
            ss << "tac_glove_marker_" << i << "&" << j;
            float force = forces_[count];
            visualization_msgs::Marker mk = genmarker(pt_marker, q_marker, PalmHeight, ArrayLength/8, force, ss.str());
            marker_pub_.publish(mk);

            y_center += ArrayLength/4;
            count++;
        }
        x_center -= ArrayLength/4;
        y_center = -(ArrayLength/2-ArrayLength/8);
    }
}

void GloveActionViz::publish_state()
{
    br_.sendTransform(tf::StampedTransform(name2tfs_["world_wrist_tf"], ros::Time::now(), "world", "vicon/wrist/wrist"));
    br_.sendTransform(tf::StampedTransform(name2tfs_["wrist_bottle_tf"], ros::Time::now(), "vicon/wrist/wrist", "vicon/bottle"+tag_+"/bottle"+tag_));
    br_.sendTransform(tf::StampedTransform(name2tfs_["wrist_lid_tf"], ros::Time::now(), "vicon/wrist/wrist", "vicon/bottle"+tag_+"_lid"+"/bottle"+tag_+"_lid"));
    br_.sendTransform(tf::StampedTransform(name2tfs_["wrist_glove_tf"], ros::Time::now(), "vicon/wrist/wrist", "glove_link"));

    publish_state_palm();

    publish_state_finger(1, 2);
    publish_state_finger(3, 5);
    publish_state_finger(6, 8);
    publish_state_finger(9, 11);
    publish_state_finger(12, 14);
}