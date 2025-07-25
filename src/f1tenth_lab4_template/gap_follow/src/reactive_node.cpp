#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

/// CHECK: include needed ROS msg type headers and libraries
class ReactiveFollowGap : public rclcpp::Node
{
    // Implement Reactive Follow Gap on the car
    // This is just a template, you are free to implement your own node!
  public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("best_point_marker", 1);

        RCLCPP_INFO(this->get_logger(), "ReactiveFollowGap node initialized");
    }

  private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    // Additional member variables for algorithm state
    std::optional<size_t> previous_best_point_;
    float angle_increment_;
    size_t ranges_size_;

    // PID 관련 멤버 변수
    float pid_integral_ = 0.0f;
    float pid_prev_error_ = 0.0f;

    void preprocess_lidar(float *ranges)
    {
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        const float max_range = 5.0f;
        const float min_range = 0.3f;
        const float vehicle_width = 0.4f;

        float min_dist = max_range;
        size_t min_idx = 0;

        // Find minimum distance and clamp invalid values
        // ranges_size_ = 1080
        for (size_t i = 0; i < ranges_size_; ++i)
        {
            // max_range보다 크거나, NaN, Inf인 값은 max_range로 설정
            if (ranges[i] > max_range || std::isnan(ranges[i]) || std::isinf(ranges[i]))
            {
                ranges[i] = max_range;
            }
            // min_dist보다 작고 0.0f보다 큰 range는 min_dist로 설정
            else if (ranges[i] < min_dist && ranges[i] > 0.0f)
            {
                min_dist = ranges[i];
                min_idx = i;
            }
        }

        // Create safety bubble around closest obstacle
        if (min_dist < min_range)
        {
            float bubble_radius = ((vehicle_width / 2.0f) / min_dist) / angle_increment_;
            size_t bubble_size = static_cast<size_t>(bubble_radius);

            size_t start_idx = (min_idx >= bubble_size) ? min_idx - bubble_size : 0;
            size_t end_idx = std::min(min_idx + bubble_size, ranges_size_ - 1);

            for (size_t i = start_idx; i <= end_idx; ++i)
            {
                ranges[i] = 0.0f;
            }
        }

        return;
    }

    void find_max_gap(float *ranges, int *indice)
    {
        // Return the start index & end index of the max gap in free_space_ranges

        // Set Region of Interest (ROI)
        const float roi_angle_deg = 67.0f;
        const float roi_angle_rad = roi_angle_deg * M_PI / 180.0f;
        size_t roi_angle_steps = static_cast<size_t>(roi_angle_rad / angle_increment_);

        size_t mid_lidar_idx = ranges_size_ / 2;
        // 관심 영역의 시작 인덱스를 계산
        size_t roi_idx_start = (mid_lidar_idx >= roi_angle_steps) ? mid_lidar_idx - roi_angle_steps : 0;
        // 관심 영역의 끝 인덱스를 계산
        size_t roi_idx_end = std::min(mid_lidar_idx + roi_angle_steps, ranges_size_ - 1);

        // Find the largest gap in free space
        const float min_range = 1.5f;
        size_t max_gap_size = 0;
        size_t max_gap_start = roi_idx_start;
        size_t max_gap_end = roi_idx_start;
        size_t gap_start = roi_idx_start;
        bool in_gap = false;

        for (size_t i = roi_idx_start; i <= roi_idx_end; ++i)
        {
            bool is_free = ranges[i] > min_range;

            if (is_free && !in_gap)
            {
                // Start of new gap
                gap_start = i;
                in_gap = true;
            }
            else if (!is_free && in_gap)
            {
                // End of current gap
                size_t gap_size = i - gap_start;
                if (gap_size > max_gap_size)
                {
                    max_gap_size = gap_size;
                    max_gap_start = gap_start;
                    max_gap_end = i - 1;
                }
                in_gap = false;
            }
        }

        // Handle case where gap extends to end of ROI
        if (in_gap)
        {
            size_t gap_size = roi_idx_end - gap_start + 1;
            if (gap_size > max_gap_size)
            {
                max_gap_size = gap_size;
                max_gap_start = gap_start;
                max_gap_end = roi_idx_end;
            }
        }

        // Fallback if no gap found
        if (max_gap_size == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Warning: No gap found!");
            max_gap_start = (roi_idx_start + roi_idx_end) / 2;
            max_gap_end = max_gap_start;
        }

        // Return indices through pointer parameters
        indice[0] = static_cast<int>(max_gap_start);
        indice[1] = static_cast<int>(max_gap_end);
        // std::cout << "Max gap start : " << max_gap_start << std::endl;
        // std::cout << "Max gap end : " << max_gap_end << std::endl;
        return;
    }

    void find_best_point(float *ranges, int *indice)
    {
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
        // Naive: Choose the furthest point within ranges and go there

        size_t gap_start = static_cast<size_t>(indice[0]);
        size_t gap_end = static_cast<size_t>(indice[1]);

        // Find best point using weighted average
        const float alpha = 0.6f;
        float weighted_sum = 0.0f;
        float weight_total = 0.0f;

        for (size_t i = gap_start; i <= gap_end; ++i)
        {
            float weight = ranges[i];
            weighted_sum += static_cast<float>(i) * weight;
            weight_total += weight;
        }

        size_t best_point =
            (weight_total > 0.0f) ? static_cast<size_t>(weighted_sum / weight_total) : (gap_start + gap_end) / 2;

        // Apply EMA filter
        size_t ema_best;
        if (previous_best_point_.has_value())
        {
            float ema_result = alpha * static_cast<float>(best_point) +
                               (1.0f - alpha) * static_cast<float>(previous_best_point_.value());
            ema_best = static_cast<size_t>(std::round(ema_result));
        }
        else
        {
            ema_best = best_point;
        }

        previous_best_point_ = ema_best;

        // Return best point index through pointer parameter
        indice[2] = static_cast<int>(ema_best);

        return;
    }

    float pure_pursuit(float steer_ang_rad, float lookahead_dist)
    {
        const float lidar_to_rear = 0.27f;
        const float wheel_base = 0.32f;

        float bestpoint_x = lookahead_dist * std::cos(steer_ang_rad);
        float bestpoint_y = lookahead_dist * std::sin(steer_ang_rad);

        float lookahead_angle = std::atan2(bestpoint_y, bestpoint_x + lidar_to_rear);
        float lookahead_rear = std::sqrt(std::pow(bestpoint_x + lidar_to_rear, 2) + std::pow(bestpoint_y, 2));

        // Final Pure Pursuit Angle
        return std::atan2(2.0f * wheel_base * std::sin(lookahead_angle), lookahead_rear);
    }

    // 현재(250724) 코드는 best point 한 개만 구해서 그 값을 따라가게끔 하는 구조임.
    // Pure pursuit는 구현이 가능하지만, 현재 단계에서는 Stanley 구현 불가
    // 따라서, waypoint가 2개 주어졌다고 가정하고 뒤쪽의 waypoint를 따라가는 구조로 구현
    float stanley(float car_velocity, float waypoint_x1, float waypoint_y1, float waypoint_x2, float waypoint_y2)
    {
        const float k = 0.5f; // Stanley controller gain
        const float wheel_base = 0.32f;
        const float x_front = 0.05f;
        const float y_front = 0.0f;
        
        float track_error = point_to_line_distance(waypoint_x1, waypoint_y1, waypoint_x2, waypoint_y2, x_front, y_front);
        float heading_error = std::atan2(waypoint_y2 - waypoint_y1, waypoint_x2 - waypoint_x1) - std::atan2(y_front, x_front);
        float steering_angle = heading_error + std::atan2(k * track_error, car_velocity);
        return steering_angle;
    }

    // 점과 직선 사이의 거리 구하는 공식
    float point_to_line_distance(float x1, float y1, float x2, float y2, float px, float py)
    {
        float numerator = std::abs((y2 - y1) * px - (x2 - x1) * py + x2 * y1 - y2 * x1);
        float denominator = std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));
        return numerator / denominator;
    }

    // 차량 제어 파트
    std::pair<float, float> vehicle_control(float *ranges, size_t best_point)
    {
        // Calculate steering angle and speed
        // 차량의 중심 인덱스를 기준으로 
        size_t vehicle_center_idx = ranges_size_ / 2;   // 540
        // Calculate steering angle in radians
        // steer_ang_rad는 차량의 x축을 기준으로 best_point가 몇 rad 떨어져있는지 각도를 계산한 값
        // Pure pursuit에서 알파값에 해당함.
        float steer_ang_rad = (static_cast<float>(best_point) - static_cast<float>(vehicle_center_idx)) * angle_increment_;
        float best_lookahead = std::min(ranges[best_point], 3.0f);
        float steer_ang_deg = std::abs(steer_ang_rad) * 180.0f / M_PI;

        // Adaptive lookahead distance
        float adaptive_lookahead;
        if (steer_ang_deg < 5.0f)
            adaptive_lookahead = best_lookahead * 1.0f;
        else if (steer_ang_deg < 15.0f)
            adaptive_lookahead = best_lookahead * 0.7f;
        else if (steer_ang_deg < 30.0f)
            adaptive_lookahead = best_lookahead * 0.5f;
        else
            adaptive_lookahead = best_lookahead * 0.3f;

        // pure_pursuit_steer 변수는 Pure pursuit 알고리즘에서 delta 즉, 차량의 스티어링 회전 값을 의미.
        float pure_pursuit_steer = pure_pursuit(steer_ang_rad, adaptive_lookahead);
        // float stanley_steer = stanley();
        float final_steer_ang_deg = std::abs(pure_pursuit_steer) * 180.0f / M_PI;

        // 하드코딩으로 linear_velocity를 설정
        float drive_speed;
        if (final_steer_ang_deg < 5.0f)
            drive_speed = 4.0f;
        else if (final_steer_ang_deg < 10.0f)
            drive_speed = 2.5f;
        else if (final_steer_ang_deg < 15.0f)
            drive_speed = 1.2f;
        else
            drive_speed = 0.8f;

        // PID 컨트롤러로 속도 조절
        // drive_speed = pid_controller(drive_speed, current_speed);

        RCLCPP_INFO(this->get_logger(), "Final Steer: %.2f°, Driving Speed: %.2f", final_steer_ang_deg, drive_speed);

        return std::make_pair(pure_pursuit_steer, drive_speed);
    }

    // PID 컨트롤러 함수: 목표 속도와 현재 속도를 받아 linear speed를 반환
    // 자세히 확인하고 다시 적용할 것. 뭔가 잘못된 부분이 있는 것 같음.
    float pid_controller(float target_speed, float current_speed)
    {
        // PID 파라미터 (상황에 맞게 튜닝)
        const float Kp = 1.0f;
        const float Ki = 0.1f;
        const float Kd = 0.05f;

        float error = target_speed - current_speed;
        pid_integral_ += error;
        float derivative = error - pid_prev_error_;
        pid_prev_error_ = error;

        float output_speed = Kp * error + Ki * pid_integral_ + Kd * derivative + current_speed;
        // 속도 제한 (예: 0~4 m/s)
        output_speed = std::max(0.0f, std::min(output_speed, 4.0f));

        return output_speed;
    }

    // Publish the best point marker in RViz
    // This function visualizes the best point in the gap as a red sphere marker
    void publish_best_point_marker(size_t best_point_idx, float *ranges)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "ego_racecar/laser"; // LiDAR 좌표계에 맞게 설정
        marker.header.stamp = this->now();
        marker.ns = "best_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // best_point의 위치 계산
        float range = ranges[best_point_idx];
        float angle = (static_cast<float>(best_point_idx) - static_cast<float>(ranges_size_ / 2)) * angle_increment_;

        marker.pose.position.x = range * std::cos(angle);
        marker.pose.position.y = range * std::sin(angle);
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        marker.lifetime = rclcpp::Duration::from_seconds(0.2);

        marker_publisher_->publish(marker);
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        /// TODO:
        // Find closest point to LiDAR
        // Eliminate all points inside 'bubble' (set them to zero)
        // Find max length gap
        // Find the best point in the gap
        // Publish Drive message

        try
        {
            // Store scan parameters for use in other functions
            angle_increment_ = scan_msg->angle_increment;       // 측정치들간의 각도 간격 (단위 : radian)
            ranges_size_ = scan_msg->ranges.size();             // 측정치의 개수
            // std::cout << "Ranges size: " << ranges_size_ << std::endl;  // Simulation 상에서는 1080개라고 함.
            // Create mutable copy of ranges for processing
            std::vector<float> ranges_vec = scan_msg->ranges;
            float *ranges = ranges_vec.data();

            // Preprocess LiDAR data (includes bubble elimination around closest point)
            // What is bubble elimination?
            // Bubble elimination is setting points within a certain distance of the closest obstacle to zero.
            // This helps to avoid collisions by ignoring nearby obstacles.
            preprocess_lidar(ranges);

            // Find the largest gap
            int gap_indices[3]; // [start_idx, end_idx, best_point_idx]
            find_max_gap(ranges, gap_indices);

            // Find the best point in the gap
            find_best_point(ranges, gap_indices);

            size_t best_point_idx = static_cast<size_t>(gap_indices[2]);
            std::cout << "Best point index: " << best_point_idx << std::endl;

            // RViz에 best_point marker publish
            publish_best_point_marker(best_point_idx, ranges);

            // Calculate control commands
            auto [steering_angle, drive_speed] = vehicle_control(ranges, best_point_idx);

            // Create and publish drive message
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.steering_angle = steering_angle;
            drive_msg.drive.speed = drive_speed;

            drive_publisher_->publish(drive_msg);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error during scan process: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}