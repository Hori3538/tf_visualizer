#ifndef TF_VISUALIZER
#define TF_VISUALIZER

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_msgs/TFMessage.h>

struct TfInfo
{
    std::string base_frame_id;
    std::string child_frame_id;
    nav_msgs::Path tf_path;
};

class TfVisualizer
{
    public:
        TfVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    private:
        void tf_callback(const tf2_msgs::TFMessageConstPtr &tf_msg);

        void register_tf_info(geometry_msgs::TransformStamped &transform);
        void update_tf_info(int index, geometry_msgs::TransformStamped &transform);
        geometry_msgs::PoseStamped create_pose_from_transform(geometry_msgs::TransformStamped &transform);
        int check_exist_tf(geometry_msgs::TransformStamped &transform);
        void visualize_path(std::vector<TfInfo> &tf_list);

        int past_tf_num_threshold_;

        std::vector<TfInfo> tf_list_;

        ros::Subscriber tf_sub_;
        ros::Publisher path_pub_;
};

#endif
