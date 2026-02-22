////////////////////////////////////////////////////////////////////////////////////
// Authors:   Bence Nyitrai
// Group:     2x
// License:   Apache License
//
// Brief:     Node that subscribes to a camera image topic and publishes a boolean
//            indicating whether the average brightness of the image is above a
//            configurable threshold. Brightness is determined by converting the
//            image to grayscale and computing the mean pixel value (0-255).
///////////////////////////////////////////////////////////////////////////////////

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

class ImageBrightnessNode : public rclcpp::Node
{
public:
    ImageBrightnessNode()
        : Node("image_brightness_node")
    {
        // Declare and get parameter for brightness threshold
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "This parameter sets the brightness threshold value";
        this->declare_parameter("brightness_threshold", 64.0, param_desc);
        brightness_threshold_ = this->get_parameter("brightness_threshold").as_double();
        
        // Runtime brightness threshold change callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ImageBrightnessNode::param_callback, this, std::placeholders::_1)
        );

        // Subscription to the image topic
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 
            10, 
            std::bind(&ImageBrightnessNode::image_callback, this, std::placeholders::_1)
        );
        
        // Publisher for brightness state
        light_state_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "/light_state", 
            10
        );
        
        RCLCPP_INFO(this->get_logger(), 
                    "image_brightness_node started with threshold: %.2f", 
                    brightness_threshold_);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image message to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        // Calculate brightness
        cv::Mat img = cv_ptr->image;
        cv::Mat gray_image;
        cv::cvtColor(img, gray_image, cv::COLOR_BGR2GRAY);
        double brightness = cv::mean(gray_image)[0];
        bool is_light_on = brightness >= brightness_threshold_;
        
        // Publish light state
        auto light_msg = std_msgs::msg::Bool();
        light_msg.data = is_light_on;
        light_state_publisher_->publish(light_msg);
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
                result.successful = true;
            }
        }
        return result;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr light_state_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    double brightness_threshold_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageBrightnessNode>());
    rclcpp::shutdown();
    return 0;
}