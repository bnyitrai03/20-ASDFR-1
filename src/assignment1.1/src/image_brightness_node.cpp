#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageBrightnessNode : public rclcpp::Node
{
public:
    ImageBrightnessNode()
        : Node("image_brightness_node"), brightness_threshold_(127.0)
    {
        // Declare and get parameter for brightness threshold
        this->declare_parameter("brightness_threshold", 127.0);
        brightness_threshold_ = this->get_parameter("brightness_threshold").as_double();
        
        // Subscription to the image topic
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 
            10, 
            std::bind(&ImageBrightnessNode::image_callback, this, std::placeholders::_1)
        );
        
        // Publisher for light state (true = light on, false = light off)
        light_state_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "/light_state", 
            10
        );
        
        RCLCPP_INFO(this->get_logger(), 
                    "Image Brightness Node started with threshold: %.2f", 
                    brightness_threshold_);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // Calculate brightness
            cv::Mat img = cv_ptr->image;
            cv::Mat gray_image;
            cv::cvtColor(img, gray_image, cv::COLOR_BGR2GRAY);
            
            // Calculate average brightness
            double brightness = cv::mean(gray_image)[0];
            
            // Determine light state based on threshold
            bool is_light_on = brightness >= brightness_threshold_;
            
            // Publish light state
            auto light_msg = std_msgs::msg::Bool();
            light_msg.data = is_light_on;
            light_state_publisher_->publish(light_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr light_state_publisher_;
    double brightness_threshold_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageBrightnessNode>());
    rclcpp::shutdown();
    return 0;
}