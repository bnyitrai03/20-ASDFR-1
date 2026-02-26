// /////////////////////////////////////////////////////////////////////////////////
// Authors:   Bence Nyitrai
// Group:     2x
// License:   Apache License
//
// Brief:     ROS2 node that detects the position of a bright object in an
//            image using grayscale thresholding and CoG computation.
// /////////////////////////////////////////////////////////////////////////////////

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

class ImageProcessingNode : public rclcpp::Node
{
public:
    ImageProcessingNode()
        : Node("image_processing_node")
    {   
        // Brightness threshold parameter
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "This parameter sets the brightness threshold value";
        this->declare_parameter("brightness_threshold", 64.0, param_desc);
        brightness_threshold_ = this->get_parameter("brightness_threshold").as_double();
        // Runtime brightness threshold change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ImageProcessingNode::param_callback, this, std::placeholders::_1)
        );

        // Image topic parameter
        auto param = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "This parameter sets the image subscription topic value";
        this->declare_parameter<std::string>("image_topic", "/image", param);
        image_topic_ = this->get_parameter("image_topic").as_string();

        // Subscription to the image topic
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, 
            10, 
            std::bind(&ImageProcessingNode::image_callback, this, std::placeholders::_1)
        );
        
        // Publish the object pixel coordinates
        object_coordinate_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/object_coordinate", 
            10
        );
        
        RCLCPP_INFO(this->get_logger(),
                    "image_processing_node_node started with"
                    "topic: %s and brightness threshold: %.2f",
                    image_topic_.c_str(), brightness_threshold_);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image message to OpenCV format and grayscale
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;
        cv::Mat gray_image;
        cv::cvtColor(img, gray_image, cv::COLOR_BGR2GRAY);

        // Pixels brighter than threshold become white, rest becomes black
        cv::Mat binary_image;
        cv::threshold(gray_image, binary_image, brightness_threshold_, 255, cv::THRESH_BINARY);

        // Compute the CoG of white pixels
        geometry_msgs::msg::Point coord_msg;
        cv::Moments moments = cv::moments(binary_image, true);
        if (moments.m00 > 0.0) {
            // CoG formula: x_cog = m10 / m00,  y_cog = m01 / m00
            coord_msg.x = moments.m10 / moments.m00;
            coord_msg.y = moments.m01 / moments.m00;
            coord_msg.z = 1.0;  // z = 1: object detected
        } else {
            coord_msg.x = 0.0;
            coord_msg.y = 0.0;
            coord_msg.z = 0.0;
        }
        object_coordinate_publisher_->publish(coord_msg);
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = false;
        for (const auto &param : parameters)
        {
            if (param.get_name() == "brightness_threshold")
            {
                if (param.as_double() < 0.0 || 255.0 < param.as_double())
                    return result;
                brightness_threshold_ = param.as_double();
                RCLCPP_INFO(this->get_logger(),
                            "Brightness threshold updated to: %.2f",
                            brightness_threshold_);
            }
            else if (param.get_name() == "image_topic")
            {
                image_topic_ = param.as_string();
                RCLCPP_INFO(this->get_logger(),
                            "Image topic updated to: %s",
                            image_topic_.c_str());
            }
        }
        result.successful = true;
        return result;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr object_coordinate_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    double brightness_threshold_;
    std::string image_topic_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessingNode>());
    rclcpp::shutdown();
    return 0;
}