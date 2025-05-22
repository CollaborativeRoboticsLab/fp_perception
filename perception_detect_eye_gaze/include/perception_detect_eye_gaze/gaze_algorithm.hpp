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

#include <perception_detect_eye_gaze/attention_calibrator.hpp>
#include <perception_detect_eye_gaze/attention_detector.hpp>
#include <perception_detect_eye_gaze/calibrated_attention_detector.hpp>
#include <perception_detect_eye_gaze/utils/gaze_metrics.hpp>
#include <perception_detect_eye_gaze/utils/gaze_utils.hpp>

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
    stop();

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
    node_->declare_parameter("algorithm.GazeAlgorithm.detection.debug_mode", "false");

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
    debug_mode = node_->get_parameter("algorithm.GazeAlgorithm.detection.debug_mode").as_bool();

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
    event_->info("Assigned detection debug mode: " + std::to_string(debug_mode));

    initialize_base(node);


    calibrator_ = std::make_shared<AttentionCalibrator>(calibration_time, samples_needed, gaze_angle_tolerance);
    calib_detector_ = std::make_shared<CalibratedAttentionDetector>(
        calibrator_, attention_threshold, pitch_threshold, yaw_threshold, history_size, model_path, attention_state);
    detector_ = std::make_shared<AttentionDetector>(attention_threshold, pitch_threshold, yaw_threshold, history_size,
                                                    model_path, attention_state);

    is_in_attention_detection_mode_ = false;
  }

  void start() override
  {
    startCalibration();

    is_in_attention_detection_mode_ = true;
    attention_thread_ = std::thread(&GazeAlgorithm::attentionLoop, this);
  }

  void stop() override
  {
    is_in_attention_detection_mode_ = false;

    if (attention_thread_.joinable())
      attention_thread_.join();

    cv::destroyAllWindows();
  }

protected:
  /**
   * @brief Start the attention calibration process
   *
   * This function initializes the calibration process and starts capturing video frames.
   * It processes the frames to detect the user's gaze angles and updates the calibration data.
   * The calibration process continues until the user looks at the robot for a specified number of samples.
   */
  bool startCalibration()
  {
    event_->info("Running Calibration function in Gaze Algorithm");

    if (loadCalibrationData())
    {
      event_->info("Calibration not needed.");
      return true;
    }
    else
    {
      event_->info("Calibration needed. Pre calibrated data not found.");
      event_->info("Starting calibration.");

      calibrator_->initiateCalibration();
      bool is_complete = false;

      while (true)
      {
        cv::Mat frame = std::any_cast<cv::Mat>(vision_driver_->getData());

        cv::Mat vis;
        bool attention, sustained, face_found;
        cv::Vec3d angles;

        detector_->processFrame(frame, vis, attention, sustained, angles, face_found);

        if (face_found and debug_mode)
        {
          std::string msg;
          is_complete = calibrator_->processSample(angles[0], angles[1], msg);
          event_->info(msg);

          cv::putText(vis, msg, { 20, 110 }, cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 165, 0 }, 2);

          if (is_complete)
          {
            event_->info("Calibration complete -> Baseline Pitch: " + std::to_string(calibrator_->getBaselinePitch()) +
                         " Baseline Yaw: " + std::to_string(calibrator_->getBaselineYaw()) +
                         " Pitch Threshold: " + std::to_string(calibrator_->getPitchThreshold()) +
                         " Yaw Threshold: " + std::to_string(calibrator_->getYawThreshold()));

            saveCalibrationData();
            break;
          }
        }

        cv::imshow("Calibration", vis);
        if (cv::waitKey(5) == 27)
          break;
      }

      event_->info("Calibration process finished.");
      cv::destroyAllWindows();

      if (!is_complete)
        throw perception_exception("Calibration failed or interrupted.");
    }
  }

  /**
   * @brief Attention detection loop
   *
   * This function runs in a separate thread and continuously captures video frames,
   * processes them to detect the user's gaze angles, and calculates the gaze score.
   */
  void attentionLoop()
  {
    cv::Mat vis;
    bool attention, sustained, face_found;
    cv::Vec3d angles;

    while (is_in_attention_detection_mode_)
    {
      cv::Mat frame = std::any_cast<cv::Mat>(vision_driver_->getData());

      calib_detector_->processFrame(frame, vis, attention, sustained, angles, face_found);

      double timestamp = getTimeSeconds();
      attention_window_.emplace_back(timestamp, attention);

      GazeMetrics metrics = GazeUtils::calculateAttentionMetrics(attention_window_, 3.0);

      event_->info("[" + std::to_string(timestamp) + "] Gaze Score: " + std::to_string(metrics.gaze_score) +
                   " Robot looks: " + std::to_string(metrics.robot_looks) +
                   " Gaze entropy: " + std::to_string(metrics.gaze_entropy) +
                   " Attention Ratio: " + std::to_string(metrics.attention_ratio));

      if (face_found and debug_mode)
      {
        int h = frame.rows;
        cv::putText(vis, "Baseline Pitch: " + std::to_string(calibrator_->getBaselinePitch()), { 20, 110 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 165, 0 }, 2);
        cv::putText(vis, "Baseline Yaw: " + std::to_string(calibrator_->getBaselineYaw()), { 20, 140 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 165, 0 }, 2);
        cv::putText(vis, "Attention Ratio: " + std::to_string(metrics.attention_ratio), { 20, h - 90 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 165, 0 }, 2);
        cv::putText(vis, "Gaze Entropy: " + std::to_string(metrics.gaze_entropy), { 20, h - 60 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 165, 0 }, 2);
        cv::putText(vis, "Frames in Window: " + std::to_string(metrics.frames_in_interval), { 20, h - 30 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 165, 0 }, 2);

        if (!vis.empty())
        {
          cv::imshow("Calibrated HRI Attention Detection", vis);
          if (cv::waitKey(5) == 27)
            break;
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  /**
   * @brief Load calibration data from a file
   *
   * This function loads the calibration data from a binary file and initializes the calibrator.
   *
   * @return true if calibration data is loaded successfully, false otherwise
   */
  bool loadCalibrationData()
  {
    try
    {
      std::ifstream file("calibration_data.bin", std::ios::binary);

      if (!file)
      {
        event_->info("No calibration data found.");
        return false;
      }
      else
      {
        event_->info("Calibration data found.");

        double bp, by, pt, yt;
        file.read(reinterpret_cast<char*>(&bp), sizeof(double));
        file.read(reinterpret_cast<char*>(&by), sizeof(double));
        file.read(reinterpret_cast<char*>(&pt), sizeof(double));
        file.read(reinterpret_cast<char*>(&yt), sizeof(double));
        file.close();

        calibrator_->setBaselinePitch(bp);
        calibrator_->setBaselineYaw(by);
        calibrator_->setPitchThreshold(pt);
        calibrator_->setYawThreshold(yt);

        event_->info("Calibration data loaded successfully.");

        return true;
      }
    }
    catch (const std::exception& e)
    {
      event_->error(std::string(e.what()) + " in GazeAlgorithm::loadCalibrationData");
      throw perception_exception(std::string(e.what()) + " in GazeAlgorithm::loadCalibrationData");
      return false;
    }
  }

  /**
   * @brief Save calibration data to a file
   *
   * This function saves the calibration data to a binary file.
   */
  void saveCalibrationData()
  {
    try
    {
      std::ofstream file("calibration_data.bin", std::ios::binary);

      double bp = calibrator_->getBaselinePitch();
      double by = calibrator_->getBaselineYaw();
      double pt = calibrator_->getPitchThreshold();
      double yt = calibrator_->getYawThreshold();

      file.write(reinterpret_cast<const char*>(&bp), sizeof(double));
      file.write(reinterpret_cast<const char*>(&by), sizeof(double));
      file.write(reinterpret_cast<const char*>(&pt), sizeof(double));
      file.write(reinterpret_cast<const char*>(&yt), sizeof(double));
      file.close();

      std::cout << "Calibration data saved." << std::endl;
    }
    catch (const std::exception& e)
    {
      event_->error(std::string(e.what()) + " in GazeAlgorithm::saveCalibrationData");
      throw perception_exception(std::string(e.what()) + " in GazeAlgorithm::saveCalibrationData");
    }
  }

  /**
   * @brief Get the current time in seconds
   *
   * This function returns the current time in seconds since the epoch.
   *
   * @return double Current time in seconds
   */
  static double getTimeSeconds()
  {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
  }

  std::thread main_thread_;
  rclcpp::Node::SharedPtr node_;
  double calibration_time;
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
  bool debug_mode;

  std::shared_ptr<CalibratedAttentionDetector> calib_detector_;
  std::shared_ptr<AttentionCalibrator> calibrator_;
  std::shared_ptr<AttentionDetector> detector_;

  std::thread attention_thread_;
  std::atomic<bool> is_in_attention_detection_mode_;

  std::vector<std::pair<double, bool>> attention_window_;
};

}  // namespace perception

#endif  // PERCEPTION_GAZE_ALGORITHM_HPP
