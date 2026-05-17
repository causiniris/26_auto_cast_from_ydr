#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

class HorizontalLineDetectorNode : public rclcpp::Node {
public:
  HorizontalLineDetectorNode() : Node("horizontal_line_detector_node") {
    binary_topic_ = declare_parameter<std::string>("binary_topic", "/line/binary_image");
    line_topic_ = declare_parameter<std::string>("line_topic", "/horizontal_line/line");
    angle_topic_ = declare_parameter<std::string>("angle_topic", "/horizontal_line/angle_deg");
    x_topic_ = declare_parameter<std::string>("x_topic", "/horizontal_line/y_at_x_half"); 
    debug_topic_ = declare_parameter<std::string>("debug_topic", "/horizontal_line/debug_image");

    morph_open_ksize_ = declare_parameter<int>("morph_open_ksize", 3);
    morph_close_ksize_ = declare_parameter<int>("morph_close_ksize", 5);
    border_margin_px_ = declare_parameter<int>("border_margin_px", 15);
    hough_threshold_ = declare_parameter<int>("hough_threshold", 15);
    hough_min_length_ = declare_parameter<int>("hough_min_length", 30);
    hough_max_gap_ = declare_parameter<int>("hough_max_gap", 40);
    
    max_abs_angle_deg_ = declare_parameter<double>("max_abs_angle_deg", 40.0); 
    angle_penalty_ = declare_parameter<double>("angle_penalty", 2.0);
    
    publish_debug_ = declare_parameter<bool>("publish_debug", true);
    show_fps_overlay_ = declare_parameter<bool>("show_fps_overlay", true);
    fps_ema_alpha_ = declare_parameter<double>("fps_ema_alpha", 0.2);
    output_ema_alpha_ = declare_parameter<double>("output_ema_alpha", 0.85); 

    skyline_nan_min_y_ratio_ = declare_parameter<double>("skyline_nan_min_y_ratio", 0.45);
    skyline_nan_max_y_ratio_ = declare_parameter<double>("skyline_nan_max_y_ratio", 0.85);

    // 【放宽距离锁死阈值】：防止弯曲横道在中间断裂
    max_track_dist_ = declare_parameter<double>("max_track_dist", 150.0);

    line_pub_ = create_publisher<geometry_msgs::msg::Point>(line_topic_, 10);
    angle_pub_ = create_publisher<std_msgs::msg::Float32>(angle_topic_, 10);
    x_pub_ = create_publisher<std_msgs::msg::Float32>(x_topic_, 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_topic_, 1);

    binary_sub_ = create_subscription<sensor_msgs::msg::Image>(
      binary_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HorizontalLineDetectorNode::onBinaryImage, this, std::placeholders::_1));

    // 终端提示已明确改为强制顺时针旋转
    RCLCPP_INFO(get_logger(), "HorizontalLineDetector (Hardcoded CLOCKWISE Rotation) started.");
  }

private:
  struct CandidateLine {
    double angle_deg;
    double x_at_center;
    std::vector<cv::Point> track_pts; 
  };

  void onBinaryImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    updateRealtimeFps();

    cv_bridge::CvImageConstPtr cv_ptr;
    try { cv_ptr = cv_bridge::toCvShare(msg, msg->encoding); } 
    catch (...) { handleInvalidDetection(); return; }

    if (cv_ptr->image.empty()) { handleInvalidDetection(); return; }

    cv::Mat binary_original;
    if (cv_ptr->image.channels() == 1) {
      binary_original = cv_ptr->image.clone();
    } else {
      cv::cvtColor(cv_ptr->image, binary_original, cv::COLOR_BGR2GRAY);
    }
    cv::threshold(binary_original, binary_original, 127, 255, cv::THRESH_BINARY);

    // =================================================================
    // 【核心写死】：强制顺时针旋转 90 度！
    // 物理映射关系：最终输出的 y_q=0 代表离车最近(底部)，y_q=10000 代表离车最远(顶部)
    // =================================================================
    cv::Mat binary;
    cv::rotate(binary_original, binary, cv::ROTATE_90_CLOCKWISE);

    applyBorderMask(binary);

    CandidateLine best_line;
    const bool found = detectBestVerticalLine(binary, best_line);

    if (found) {
      lost_frames_ = 0; 
      publishDetection(binary.size(), best_line);
    } else {
      handleInvalidDetection();
    }

    if (publish_debug_) publishDebugImage(binary, found, best_line);
  }
  
  void handleInvalidDetection() {
    lost_frames_++; publishInvalid(); 
    if (lost_frames_ >= MAX_LOST_FRAMES_) filter_initialized_ = false;
  }

  void updateRealtimeFps() {
    const auto now_tp = std::chrono::steady_clock::now();
    if (!fps_initialized_) { last_frame_tp_ = now_tp; fps_initialized_ = true; return; }
    const std::chrono::duration<double> dt = now_tp - last_frame_tp_;
    last_frame_tp_ = now_tp;
    if (dt.count() > 1e-6) {
      const double inst_fps = 1.0 / dt.count();
      fps_value_ = fps_value_ <= 0.0 ? inst_fps : fps_ema_alpha_ * inst_fps + (1.0 - fps_ema_alpha_) * fps_value_;
    }
  }

  void applyBorderMask(cv::Mat &binary) const {
    if (binary.empty()) return;
    const int margin = std::max(0, border_margin_px_);
    if (margin == 0) return;
    const int margin_x = std::min(margin, std::max(0, binary.cols / 2 - 1));
    const int margin_y = std::min(margin, std::max(0, binary.rows / 2 - 1));
    if (margin_y > 0) {
      binary.rowRange(0, margin_y).setTo(0);
      binary.rowRange(binary.rows - margin_y, binary.rows).setTo(0);
    }
    if (margin_x > 0) {
      binary.colRange(0, margin_x).setTo(0);
      binary.colRange(binary.cols - margin_x, binary.cols).setTo(0);
    }
  }

  bool detectBestVerticalLine(cv::Mat &binary, CandidateLine &best_line) {
    int h = binary.rows; int w = binary.cols;
    struct TrackPoint { float x; float y; float width; };
    std::vector<TrackPoint> all_pts;

    for (int y = h - 1; y >= 0; y -= 3) {
      const uint8_t* row = binary.ptr<uint8_t>(y);
      std::vector<std::pair<int, int>> segments;
      int start = -1;
      
      for (int x = 0; x < w; ++x) {
        if (row[x] > 127 && start == -1) start = x;
        else if (row[x] <= 127 && start != -1) {
          segments.push_back({start, x - 1}); start = -1;
        }
      }
      if (start != -1) segments.push_back({start, w - 1});

      float expected_x_at_y = filter_initialized_ ? smoothed_x_ + (y - h/2.0f) * std::tan(smoothed_angle_ * CV_PI / 180.0) : w / 2.0f;
      float best_cx = -1; float best_w = -1; float min_dist = 1e9;

      for (auto& seg : segments) {
        int seg_w = seg.second - seg.first;
        if (seg_w < 5 || seg_w > w * 0.45f) continue; 
        
        float cx = (seg.first + seg.second) / 2.0f;
        float dist = std::abs(cx - expected_x_at_y);
        
        // 使用放宽的距离容忍度
        if (filter_initialized_ && dist > max_track_dist_) continue;

        if (dist < min_dist) { min_dist = dist; best_cx = cx; best_w = seg_w; }
      }
      if (best_cx >= 0) all_pts.push_back({best_cx, static_cast<float>(y), best_w});
    }

    if (all_pts.size() < 5) return false;

    std::vector<float> widths;
    for (const auto& p : all_pts) widths.push_back(p.width);
    std::sort(widths.begin(), widths.end());

    int robust_idx = static_cast<int>(widths.size() * 0.15);
    float robust_min_width = widths[std::min(robust_idx, static_cast<int>(widths.size() - 1))];

    std::vector<cv::Point> track_pts;
    std::vector<cv::Point2f> fit_pts;

    for (size_t i = 0; i < all_pts.size(); ++i) {
      const auto& p = all_pts[i];
      if (p.width <= robust_min_width + 15.0f) {
        track_pts.push_back({static_cast<int>(p.x), static_cast<int>(p.y)});
        fit_pts.push_back({p.x, p.y});
      }
    }

    if (fit_pts.size() < 4) return false;

    float min_y = h;
    for (const auto& p : fit_pts) { if (p.y < min_y) min_y = p.y; }
    float min_y_ratio = min_y / static_cast<float>(h);

    if (!is_in_intersection_) {
      if (min_y_ratio > skyline_nan_min_y_ratio_ && min_y_ratio < skyline_nan_max_y_ratio_) {
        is_in_intersection_ = true; return false; 
      }
    } else {
      if (min_y_ratio < (skyline_nan_min_y_ratio_ - 0.10f)) {
        is_in_intersection_ = false; 
      } else {
        return false; 
      }
    }

    cv::Vec4f line_params;
    cv::fitLine(fit_pts, line_params, cv::DIST_L2, 0, 0.01, 0.01);

    float vx = line_params[0]; float vy = line_params[1];
    float x0 = line_params[2]; float y0 = line_params[3];

    if (std::abs(vy) < 1e-6f) return false;

    double dx = vx; double dy = vy;
    if (dy < 0) { dx = -dx; dy = -dy; }

    double raw_angle_deg = std::atan2(dx, dy) * 180.0 / CV_PI;

    if (std::abs(raw_angle_deg) > max_abs_angle_deg_) return false;

    const float target_y = h / 2.0f;
    double raw_x_center = x0 + (target_y - y0) * dx / dy;

    if (!filter_initialized_) {
      smoothed_x_ = raw_x_center; smoothed_angle_ = raw_angle_deg; filter_initialized_ = true;
    } else {
      double alpha = std::clamp(output_ema_alpha_, 0.01, 1.0);
      smoothed_x_ = alpha * raw_x_center + (1.0 - alpha) * smoothed_x_;
      smoothed_angle_ = alpha * raw_angle_deg + (1.0 - alpha) * smoothed_angle_;
    }

    best_line.angle_deg = smoothed_angle_;
    best_line.x_at_center = smoothed_x_;
    best_line.track_pts = track_pts; 
    return true;
  }

  void publishDetection(const cv::Size & image_size, const CandidateLine & line) {
    const double x_norm = line.x_at_center / static_cast<double>(std::max(1, image_size.width - 1));
    geometry_msgs::msg::Point line_msg;
    line_msg.x = std::clamp(x_norm, 0.0, 1.0);
    line_msg.y = 0.5;
    line_msg.z = line.angle_deg;
    line_pub_->publish(line_msg);

    std_msgs::msg::Float32 angle_msg;
    angle_msg.data = static_cast<float>(line.angle_deg);
    angle_pub_->publish(angle_msg);

    std_msgs::msg::Float32 x_msg;
    x_msg.data = static_cast<float>(line_msg.x);
    x_pub_->publish(x_msg);
  }

  void publishInvalid() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    geometry_msgs::msg::Point line_msg;
    line_msg.x = nan; line_msg.y = nan; line_msg.z = nan;
    line_pub_->publish(line_msg);

    std_msgs::msg::Float32 f_msg; f_msg.data = nan;
    angle_pub_->publish(f_msg); x_pub_->publish(f_msg);
  }

  void publishDebugImage(const cv::Mat &binary, bool found, const CandidateLine & line) {
    cv::Mat vis; cv::cvtColor(binary, vis, cv::COLOR_GRAY2BGR);
    const int h = vis.rows; const int w = vis.cols;
    const int y_half = static_cast<int>(std::lround(0.5 * std::max(0, h - 1)));
    
    cv::line(vis, {0, y_half}, {std::max(0, w - 1), y_half}, cv::Scalar(255, 0, 0), 1);
    cv::line(vis, {w / 2, 0}, {w / 2, h - 1}, cv::Scalar(255, 0, 0), 1);

    if (found && !line.track_pts.empty()) {
      for (const auto& pt : line.track_pts) cv::circle(vis, pt, 2, cv::Scalar(0, 165, 255), -1);

      double rad = line.angle_deg * CV_PI / 180.0;
      double dx = std::sin(rad); double dy = std::cos(rad);
      float tangent_len = 100.0f;
      cv::Point2f center_pt(static_cast<float>(line.x_at_center), static_cast<float>(y_half));
      cv::Point2f p_top(center_pt.x - dx * tangent_len, center_pt.y - dy * tangent_len);
      cv::Point2f p_bot(center_pt.x + dx * tangent_len, center_pt.y + dy * tangent_len);

      cv::line(vis, p_top, p_bot, cv::Scalar(0, 255, 0), 3);
      cv::drawMarker(vis, center_pt, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3) << "angle=" << line.angle_deg << " deg";
      cv::putText(vis, oss.str(), {20, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 165, 255), 2);
    } else {
      cv::putText(vis, "horizontal line not found", {20, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    
    if (show_fps_overlay_ && fps_value_ > 0.0) {
      std::ostringstream fps_ss; fps_ss << std::fixed << std::setprecision(1) << "FPS: " << fps_value_;
      cv::putText(vis, fps_ss.str(), {20, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    }
    
    std_msgs::msg::Header header;
    header.stamp = this->now(); header.frame_id = "camera_frame";
    debug_pub_->publish(*cv_bridge::CvImage(header, "bgr8", vis).toImageMsg());
  }

  std::string binary_topic_, line_topic_, angle_topic_, x_topic_, debug_topic_;
  int morph_open_ksize_, morph_close_ksize_, border_margin_px_;
  int hough_threshold_, hough_min_length_, hough_max_gap_;
  double max_abs_angle_deg_, angle_penalty_, fps_ema_alpha_, output_ema_alpha_;
  double skyline_nan_min_y_ratio_, skyline_nan_max_y_ratio_, max_track_dist_;
  bool publish_debug_, show_fps_overlay_;
  
  bool fps_initialized_{false}; double fps_value_{0.0};
  std::chrono::steady_clock::time_point last_frame_tp_{};

  double smoothed_x_{-1.0}, smoothed_angle_{0.0};
  bool filter_initialized_{false};
  int lost_frames_{0};                   
  const int MAX_LOST_FRAMES_{3};         
  bool is_in_intersection_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr binary_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr line_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_, x_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HorizontalLineDetectorNode>());
  rclcpp::shutdown();
  return 0;
}