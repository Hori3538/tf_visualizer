#include <tf_visualizer/tf_visualizer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_visualizer_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    TfVisualizer tf_visualizer(nh, pnh);


    ros::spin();
    return 0;
}
