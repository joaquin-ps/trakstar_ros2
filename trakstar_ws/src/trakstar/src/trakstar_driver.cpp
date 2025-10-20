/*********************************************************************
*
* This is free and unencumbered software released into the public domain.
* 
* Anyone is free to copy, modify, publish, use, compile, sell, or
* distribute this software, either in source code form or as a compiled
* binary, for any purpose, commercial or non-commercial, and by any
* means.
* 
* In jurisdictions that recognize copyright laws, the author or authors
* of this software dedicate any and all copyright interest in the
* software to the public domain. We make this dedication for the benefit
* of the public at large and to the detriment of our heirs and
* successors. We intend this dedication to be an overt act of
* relinquishment in perpetuity of all present and future rights to this
* software under copyright law.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
* 
* For more information, please refer to <http://unlicense.org/>
* 
**********************************************************************/

/** 
 * Author: Sean Seungkook Yun <seungkook.yun@sri.com>
 * Adapted for MyHand by: Ava Chen <ava.chen@columbia.edu> 
 * ROS2 adaptation by: Joaquin Palacios
*/

#include <string>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "trakstar/PointATC3DG.hpp"
#include "trakstar/msg/trakstar_msg.hpp"

using namespace trakstar;
using std::string;

class TrakstarDriver : public rclcpp::Node
{
public:
    TrakstarDriver() : Node("trakstar_driver")
    {
        // Declare parameters
        this->declare_parameter("hemisphere_back", false);
        this->declare_parameter("frequency", 255);
        this->declare_parameter("range_72inch", false);
        this->declare_parameter("publish_tf", false);
        
        // Pivot point parameters for sensor 0
        this->declare_parameter("pivot_x", 0.0);
        this->declare_parameter("pivot_y", 0.0);
        this->declare_parameter("pivot_z", 0.0);
        this->declare_parameter("attach_roll", 0.0);
        this->declare_parameter("attach_pitch", 0.0);
        this->declare_parameter("attach_yaw", 0.0);
        
        // Pivot point parameters for sensor 1
        this->declare_parameter("pivot_x1", 0.0);
        this->declare_parameter("pivot_y1", 0.0);
        this->declare_parameter("pivot_z1", 0.0);
        this->declare_parameter("attach_roll1", 0.0);
        this->declare_parameter("attach_pitch1", 0.0);
        this->declare_parameter("attach_yaw1", 0.0);

        // Get parameters
        bool hemisphere_back = this->get_parameter("hemisphere_back").as_bool();
        int frequency = this->get_parameter("frequency").as_int();
        bool range_72inch = this->get_parameter("range_72inch").as_bool();
        bool publish_tf = this->get_parameter("publish_tf").as_bool();

        // Initialize hardware
        RCLCPP_INFO(this->get_logger(), "Initializing TRAKSTAR. Please wait....");
        bird_ = std::make_unique<PointATC3DG>();
        if (!bird_->ok()) {
            RCLCPP_ERROR(this->get_logger(), "Can't open trakstar");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Frequency: %d", frequency);
        bird_->setMeasurementRate(static_cast<float>(frequency));

        RCLCPP_INFO(this->get_logger(), "Initialization Complete.");

        bird_->setSuddenOutputChangeLock(0);
        num_sen_ = bird_->getNumberOfSensors();
        RCLCPP_INFO(this->get_logger(), "Number of trackers: %d", num_sen_);

        if (num_sen_ < 2) {
            RCLCPP_ERROR(this->get_logger(), "At least 2 trackers required");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Output is set: position/quaternion");
        for (int i = 0; i < num_sen_; i++) {
            bird_->setSensorQuaternion(i);
            if (hemisphere_back)
                bird_->setSensorHemisphere(i, HEMISPHERE_REAR);
            else 
                bird_->setSensorHemisphere(i, HEMISPHERE_FRONT);
        }

        if (range_72inch)
            bird_->setMaximumRange(true);

        if (publish_tf)
            RCLCPP_INFO(this->get_logger(), "Publishing frame data to TF.");

        // Setup pivot points and orientations
        setupPivotPoints();

        // Initialize ROS2 publishers
        trakstar_pub_ = this->create_publisher<trakstar::msg::TrakstarMsg>("trakstar_msg", 1);
        trakstar_raw_pub_ = this->create_publisher<trakstar::msg::TrakstarMsg>("trakstar_raw_msg", 1);

        // Initialize TF broadcaster if needed
        if (publish_tf) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        }

        // Create timer for main loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / frequency),
            std::bind(&TrakstarDriver::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Trakstar driver started successfully");
    }

private:
    void setupPivotPoints()
    {
        // Get pivot point parameters for sensor 0
        double px = this->get_parameter("pivot_x").as_double();
        double py = this->get_parameter("pivot_y").as_double();
        double pz = this->get_parameter("pivot_z").as_double();
        trakstar_attach_pos_ = tf2::Vector3(px, py, pz);

        double rx = this->get_parameter("attach_roll").as_double();
        double ry = this->get_parameter("attach_pitch").as_double();
        double rz = this->get_parameter("attach_yaw").as_double();
        trakstar_attach_.setEulerYPR(rz, ry, rx);

        // Get pivot point parameters for sensor 1
        double px1 = this->get_parameter("pivot_x1").as_double();
        double py1 = this->get_parameter("pivot_y1").as_double();
        double pz1 = this->get_parameter("pivot_z1").as_double();
        trakstar_attach_pos1_ = tf2::Vector3(px1, py1, pz1);

        double rx1 = this->get_parameter("attach_roll1").as_double();
        double ry1 = this->get_parameter("attach_pitch1").as_double();
        double rz1 = this->get_parameter("attach_yaw1").as_double();
        trakstar_attach1_.setEulerYPR(rz1, ry1, rx1);
    }

    void timer_callback()
    {
        if (!bird_->ok()) {
            RCLCPP_ERROR(this->get_logger(), "Trakstar device not OK");
            return;
        }

        // Create messages
        trakstar::msg::TrakstarMsg msg;
        trakstar::msg::TrakstarMsg msg_raw;
        msg.header.stamp = this->now();
        msg_raw.header.stamp = this->now();
        msg.n_tracker = num_sen_;
        msg_raw.n_tracker = num_sen_;

        std::vector<geometry_msgs::msg::TransformStamped> transforms(num_sen_);

        for (int i = 0; i < num_sen_; ++i) {
            double dX, dY, dZ;
            double quat[4];
            
            bird_->getCoordinatesQuaternion(i, dX, dY, dZ, quat);
            
            tf2::Vector3 pos(dX, dY, dZ);
            tf2::Quaternion q(-quat[1], -quat[2], -quat[3], quat[0]);
            tf2::Matrix3x3 mat(q);

            // Store raw transform
            tf2::Transform raw_transform(mat, pos);
            msg_raw.transform[i] = tf2::toMsg(raw_transform);

            // Apply coordinate frame transformation
            mat = ros_to_trakstar_ * mat;

            // Apply sensor-specific pivot transformations
            if (i < 1) {
                mat = mat * trakstar_attach_;
                pos = ros_to_trakstar_ * pos + mat * trakstar_attach_pos_;
            } else {
                mat = mat * trakstar_attach1_;
                pos = ros_to_trakstar_ * pos + mat * trakstar_attach_pos1_;
            }

            // Store processed transform
            tf2::Transform processed_transform(mat, pos);
            msg.transform[i] = tf2::toMsg(processed_transform);

            // Prepare TF transform
            transforms[i].transform = tf2::toMsg(processed_transform);
            transforms[i].header.stamp = msg.header.stamp;
            transforms[i].header.frame_id = "trakstar_base";
            transforms[i].child_frame_id = "trakstar" + std::to_string(i);
        }

        // Publish messages
        trakstar_pub_->publish(msg);
        trakstar_raw_pub_->publish(msg_raw);

        // Publish TF transforms if enabled
        if (tf_broadcaster_) {
            tf_broadcaster_->sendTransform(transforms);
        }
    }

    // ROS2 to Trakstar coordinate frame transformation matrix
    const tf2::Matrix3x3 ros_to_trakstar_{-1,  0,  0,
                                           0,  1,  0,
                                           0,  0, -1};

    // Hardware interface
    std::unique_ptr<PointATC3DG> bird_;
    int num_sen_;

    // ROS2 publishers
    rclcpp::Publisher<trakstar::msg::TrakstarMsg>::SharedPtr trakstar_pub_;
    rclcpp::Publisher<trakstar::msg::TrakstarMsg>::SharedPtr trakstar_raw_pub_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timer for main loop
    rclcpp::TimerBase::SharedPtr timer_;

    // Pivot point transformations
    tf2::Vector3 trakstar_attach_pos_;
    tf2::Matrix3x3 trakstar_attach_;
    tf2::Vector3 trakstar_attach_pos1_;
    tf2::Matrix3x3 trakstar_attach1_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrakstarDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
