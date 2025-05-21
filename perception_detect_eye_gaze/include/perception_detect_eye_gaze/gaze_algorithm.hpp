#ifndef PERCEPTION_GAZE_ALGORITHM_HPP
#define PERCEPTION_GAZE_ALGORITHM_HPP

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <map>
#include <string>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <vector>
#include <opencv2/opencv.hpp>

#include <perception_base/utils/options.hpp>
#include <perception_base/utils/exceptions.hpp>
#include <perception_events/event_client.hpp>

#include <attention_calibrator.hpp>
#include <calibrated_attention_detector.hpp>
#include <utils/gaze_metrics.hpp>
#include <utils/gaze_utils.hpp>

namespace perception
{

/**
 * @brief GazeAlgorithm class
 *
 * This class is responsible for managing the gaze algorithm, including calibration,
 * attention detection, and gaze score calculation. It uses OpenCV for video capture
 * and processing, and it integrates with the GazeInterfaceController class for gaze detection.
 *
 */
class GazeAlgorithm : public AlgorithmBase
{
public:
  /**
   * @brief Constructor for GazeAlgorithm
   */
  GazeAlgorithm()
  {
  }

  /**
   * @brief Destructor for GazeAlgorithm
   *
   * Stops the attention detection thread if it is running.
   */
  ~GazeAlgorithm()
  {
    controller_.stop();
    if (main_thread_.joinable())
      main_thread_.join();
  }

  /**
   * @brief Initialize the GazeAlgorithm
   *
   * This function initializes the GazeAlgorithm with the given node and configuration.
   *
   * @param node Shared pointer to the ROS node
   * @param config Algorithm options
   */
  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    // Configure parameters for the nodevision
    node_->declare_parameter("algorithm.GazeAlgorithm.name", "EyeGazeDetection");
    node_->declare_parameter("algorithm.GazeAlgorithm.calibration.time", "10.0");
    node_->declare_parameter("algorithm.GazeAlgorithm.calibration.sample_size", "300");
    node_->declare_parameter("algorithm.GazeAlgorithm.calibration.angle_tolerance", "15.0");
    node_->declare_parameter("algorithm.GazeAlgorithm.detection.attention_threshold", "0.5");
    node_->declare_parameter("algorithm.GazeAlgorithm.detection.pitch_threshold", "15.0");
    node_->declare_parameter("algorithm.GazeAlgorithm.detection.yaw_threshold", "20.0");
    node_->declare_parameter("algorithm.GazeAlgorithm.detection.history_size", "10");
    node_->declare_parameter("algorithm.GazeAlgorithm.detection.model_path", "face_landmark.tflite");
    node_->declare_parameter("algorithm.GazeAlgorithm.detection.attention_state", "false");
    node_->declare_parameter("algorithm.GazeAlgorithm.detection.attention_state", "false");
    node_->declare_parameter("algorithm.GazeAlgorithm.detection.attention_state", "false");
    node_->declare_parameter("algorithm.GazeAlgorithm.detection.attention_state", "false");
    node_->declare_parameter("algorithm.GazeAlgorithm.detection.attention_state", "false");

    // Load parameters from the node
    config_.name = node_->get_parameter("algorithm.GazeAlgorithm.name").as_string();
    calibration_time = node_->get_parameter("algorithm.GazeAlgorithm.calibration.time").as_double();
    samples_needed = node_->get_parameter("algorithm.GazeAlgorithm.calibration.sample_size").as_int();
    gaze_angle_tolerance = node_->get_parameter("algorithm.GazeAlgorithm.calibration.gaze_angle_tolerance").as_double();
    attention_threshold = node_->get_parameter("algorithm.GazeAlgorithm.detection.attention_threshold").as_double();
    pitch_threshold = node_->get_parameter("algorithm.GazeAlgorithm.detection.pitch_threshold").as_double();
    yaw_threshold = node_->get_parameter("algorithm.GazeAlgorithm.detection.yaw_threshold").as_double();
    history_size = node_->get_parameter("algorithm.GazeAlgorithm.detection.history_size").as_int();
    model_path = node_->get_parameter("algorithm.GazeAlgorithm.detection.model_path").as_string();
    attention_state = node_->get_parameter("algorithm.GazeAlgorithm.detection.attention_state").as_bool();

    // Publish about the assigned driver parameters
    event_->info("Assigned driver name: " + config_.name);
    event_->info("Assigned calibration time: " + std::to_string(calibration_time));
    event_->info("Assigned calibration sample size: " + std::to_string(samples_needed));
    event_->info("Assigned calibration gaze angle tolerance: " + std::to_string(gaze_angle_tolerance));
    event_->info("Assigned detection attention threshold: " + std::to_string(attention_threshold));
    event_->info("Assigned detection pitch threshold: " + std::to_string(pitch_threshold));
    event_->info("Assigned detection yaw threshold: " + std::to_string(yaw_threshold));
    event_->info("Assigned detection history size: " + std::to_string(history_size));
    event_->info("Assigned detection model path: " + model_path);
    event_->info("Assigned detection attention state: " + std::to_string(attention_state));

    initialize_base(node);

    calibrator_ = AttentionCalibrator(calibration_time, samples_needed, gaze_angle_tolerance);
    detector_   = CalibratedAttentionDetector(calibrator_, attention_threshold, pitch_threshold, yaw_threshold, history_size)

        controller_.initialize(node);
    controller_setAttentionDetectionMode(false);
    controller_.setGazeAngleTolerance(15.0);  // Set the gaze angle tolerance
    controller_.setAttentionThreshold(0.5);   // Set the attention threshold
    controller_.setPitchThreshold(15.0);      // Set the pitch threshold
    controller_.setYawThreshold(20.0);        // Set the yaw threshold
  }

  void start() override
  {
    controller_.startCalibration();
    controller_.startAttentionDetection();
    main_thread_ = std::thread(&GazeAlgorithm::run, this);
  }

  void stop() override
  {
    controller_.stop();
    if (main_thread_.joinable())
      main_thread_.join();
  }

  std::string getName() const override
  {
    return "GazeAlgorithm";
  }

private:
  std::thread main_thread_;
  rclcpp::Node::SharedPtr node_;
  Double calibration_time;
  int samples_needed;
  double gaze_angle_tolerance;
  double attention_threshold;
  double pitch_threshold;
  double yaw_threshold;
  int history_size;
  std::string model_path;
  bool attention_state;
  double gaze_score_;
  int robot_looks_;
  double gaze_entropy_;

  CalibratedAttentionDetector detector_;
  AttentionCalibrator calibrator_;

  std::thread attention_thread_;
  std::atomic<bool> is_in_attention_detection_mode_;

  std::mutex score_mutex_;
  std::mutex robot_looks_mutex_;
  std::mutex entropy_mutex_;
  std::mutex frame_mutex_;

  cv::Mat visualisation_frame_;
  std::vector<std::pair<double, bool>> attention_window_;

  void run()
  {
    auto start_time = std::chrono::steady_clock::now();
    const int duration_sec = 270;
    const int interval_sec = 3;
    auto next_print = start_time + std::chrono::seconds(interval_sec);
    int time_step_count = 0;

    std::map<std::string, double> save_dictionary;
    std::string run_name = "random_mode_data";

    if (!std::filesystem::exists(run_name))
    {
      std::filesystem::create_directory(run_name);
      std::cout << "Created directory: " << run_name << std::endl;
    }

    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(duration_sec))
    {
      auto now = std::chrono::steady_clock::now();

      if (now >= next_print)
      {
        double score = controller_.getGazeScore();
        double entropy = controller_.getGazeEntropy();
        int looks = controller_.getRobotLooks();
        int state = static_cast<int>(std::round(score / 20.0));

        std::cout << "####### Gaze Score: " << score << std::endl;
        std::cout << "Robot looks: " << looks << std::endl;
        std::cout << "Gaze entropy: " << entropy << std::endl;

        save_dictionary["gaze_score_timestamp" + std::to_string(time_step_count)] = score;
        save_dictionary["nextstate_subject_timestamp" + std::to_string(time_step_count)] = state;
        save_dictionary["timestamp" + std::to_string(time_step_count)] = time_step_count;

        next_print = now + std::chrono::seconds(interval_sec);
        ++time_step_count;
      }

      cv::Mat frame = controller_.getVisualisationFrame();
      if (!frame.empty())
      {
        cv::imshow("Calibrated HRI Attention Detection", frame);
        if (cv::waitKey(5) == 27)
          break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // save_trajectory_ep_to_yaml_3(run_name, save_dictionary); // Replace with your own save function
    std::cout << "Data saved" << std::endl;
    cv::destroyAllWindows();
    controller_.stop();
  }
};

}  // namespace perception

#endif  // PERCEPTION_GAZE_ALGORITHM_HPP
