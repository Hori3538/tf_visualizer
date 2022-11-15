#include <tf_visualizer/tf_visualizer.h>

TfVisualizer::TfVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    pnh.param("past_tf_num_threshold", past_tf_num_threshold_, 400);
    tf_sub_ = nh.subscribe("/tf", 10, &TfVisualizer::tf_callback, this);
    path_pub_ = nh.advertise<nav_msgs::Path>("tf_path", 10);
}

void TfVisualizer::tf_callback(const tf2_msgs::TFMessageConstPtr &tf_msg)
{
    for(auto tf: tf_msg->transforms){
        int index = check_exist_tf(tf);
        if(index == -1){
            register_tf_info(tf);
        }
        else{
            update_tf_info(index, tf);
        }
    }
    visualize_path(tf_list_);
}

void TfVisualizer::register_tf_info(geometry_msgs::TransformStamped &transform)
{
    TfInfo new_tf_info;
    new_tf_info.base_frame_id = transform.header.frame_id;
    new_tf_info.child_frame_id = transform.child_frame_id;

    new_tf_info.tf_path.header = transform.header;
    new_tf_info.tf_path.poses.push_back(create_pose_from_transform(transform));

    tf_list_.push_back(new_tf_info);
}

geometry_msgs::PoseStamped TfVisualizer::create_pose_from_transform(geometry_msgs::TransformStamped &transform)
{
    geometry_msgs::PoseStamped pose;
    pose.header = transform.header;

    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;

    pose.pose.orientation.x = transform.transform.rotation.x;
    pose.pose.orientation.y = transform.transform.rotation.y;
    pose.pose.orientation.z = transform.transform.rotation.z;
    pose.pose.orientation.w = transform.transform.rotation.w;

    return pose;
}

void TfVisualizer::update_tf_info(int index, geometry_msgs::TransformStamped &transform)
{
    TfInfo &tf_target = tf_list_[index];
    tf_target.tf_path.poses.push_back(create_pose_from_transform(transform));

    if(tf_target.tf_path.poses.size() > past_tf_num_threshold_){
        tf_target.tf_path.poses.erase(tf_target.tf_path.poses.begin());
    }
} 

int TfVisualizer::check_exist_tf(geometry_msgs::TransformStamped &transform)
{
    int index = 0;
    for(const auto& tf_info: tf_list_){
        if(tf_info.base_frame_id == transform.header.frame_id && tf_info.child_frame_id == transform.child_frame_id){
            return index;
        }
        index++;
    }
    return -1;
}

void TfVisualizer::visualize_path(std::vector<TfInfo> &tf_list)
{
    for(const auto& tf_info: tf_list){
        path_pub_.publish(tf_info.tf_path);
    }
}
