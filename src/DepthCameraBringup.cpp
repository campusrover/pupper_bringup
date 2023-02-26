#include <chrono>
#include <memory>
#include <string>
#include "ros/ros.h"
#include <sensor_msgs/msg/point_cloud.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <vector>

#include "ArducamTOFCamera.hpp"

using namespace std::chrono_literals;
using namespace Arducam;

ArducamTOFCamera tof;

class TOFPublisher : public ros::Node
{
public:
    TOFPublisher() : Node("arducam"), pointsize_(43200)
    {   
        ros::init(argc, argv, "depth_camera");
        pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud>();
        pc2_msg_->points.resize(pointsize_);
        pc2_msg_->channels.resize(1);
        pc2_msg_->channels[0].name = "intensities";
        pc2_msg_->channels[0].values.resize(pointsize_);
        depth_msg_ = std::make_shared<std_msgs::msg::Float32MultiArray>();
        depth_msg_->layout.dim.resize(2);
        depth_msg_->layout.dim[0].label = "height";
        depth_msg_->layout.dim[0].size = 180;
        depth_msg_->layout.dim[0].stride = 43200;
        depth_msg_->layout.dim[1].label = "width";
        depth_msg_->layout.dim[1].size = 240;
        depth_msg_->layout.dim[1].stride = 240;
        
        publisher_ = nh->advertise<sensor_msgs::msg::PointCloud>("point_cloud", 10);
        publisher_depth_ = nh->advertise<std_msgs::msg::Float32MultiArray>("depth_frame", 10);

        timer_ = nh.createTimer(
            ros::Duration(0.05), std::bind(&TOFPublisher::update, this));
    }

private:
    void generateSensorPointCloud()
    {
        ArducamFrameBuffer *frame;
        do
        {
            frame = tof.requestFrame(200);
        } while (frame == nullptr);
        depth_frame.clear();
        float *depth_ptr = (float *)frame->getData(FrameType::DEPTH_FRAME);
        float *amplitude_ptr = (float *)frame->getData(FrameType::AMPLITUDE_FRAME);
        unsigned long int pos = 0;
        for (int row_idx = 0; row_idx < 180; row_idx++)
            for (int col_idx = 0; col_idx < 240; col_idx++, pos++)
            {
                if (amplitude_ptr[pos] > 30)
                {
                    float zz = depth_ptr[pos]; 
                    pc2_msg_->points[pos].x = (((120 - col_idx)) / fx) * zz;
                    pc2_msg_->points[pos].y = ((90 - row_idx) / fy) * zz;
                    pc2_msg_->points[pos].z = zz;
                    pc2_msg_->channels[0].values[pos] = depth_ptr[pos];
                    depth_frame.push_back(depth_ptr[pos]);
                }
                else
                {
                    pc2_msg_->points[pos].x = 0;
                    pc2_msg_->points[pos].y = 0;
                    pc2_msg_->points[pos].z = 0;
                    pc2_msg_->channels[0].values[pos] = 0;
                    depth_frame.push_back(0);
                }
            }
        tof.releaseFrame(frame);
        pc2_msg_->header.frame_id = frame_id_;

        depth_msg_->data = depth_frame;
    }
    void update()
    {
        generateSensorPointCloud();
        pc2_msg_->header.stamp = now();
        publisher_->publish(*pc2_msg_);
        publisher_depth_->publish(*depth_msg_);
    }

    std::string frame_id_ = "sensor_frame";
    std::vector<float> depth_frame;
    sensor_msgs::msg::PointCloud::SharedPtr pc2_msg_;
    std_msgs::msg::Float32MultiArray::SharedPtr depth_msg_;

    size_t pointsize_;
    ros::Timer::SharedPtr timer_;
    ros::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
    ros::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_depth_;
    ros::NodeHandle::SharedPtr nh;
    float fx = 240 / (2 * tan(0.5 * M_PI * 64.3 / 180));
    float fy = 180 / (2 * tan(0.5 * M_PI * 50.4 / 180));
};

int main(int argc, char *argv[])
{
    
    if (tof.init(Connection::CSI))
    {
        printf("initialize fail\n");
        exit(-1);
    }

    if (tof.start())
    {
        printf("start fail\n");
        exit(-1);
    }
    tof.setControl(ControlID::RANGE,4);

    printf("pointcloud publisher start\n");

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    ros::spin(std::make_shared<TOFPublisher>());
    ros::shutdown();
    return 0;
}