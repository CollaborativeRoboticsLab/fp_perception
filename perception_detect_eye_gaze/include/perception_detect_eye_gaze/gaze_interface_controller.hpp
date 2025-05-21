#ifndef GAZE_INTERFACE_CONTROLLER_HPP
#define GAZE_INTERFACE_CONTROLLER_HPP


class GazeInterfaceController
{
public:


  GazeInterfaceController(double gaze_angle_tolerance = 15.0)
    : calibrator_(10.0, 300, gaze_angle_tolerance)
    , detector_(calibrator_)
    , is_in_attention_detection_mode_(false)
    , gaze_score_(0.0)
    , robot_looks_(0)
    , gaze_entropy_(0.0)
  {
  }

  /**
   * @brief Destructor for GazeInterfaceController
   *
   * Stops the attention detection thread if it is running.
   */
  ~GazeInterfaceController()
  {
    stop();
  }


  /**
   * @brief Start the attention calibration process
   *
   * This function initializes the calibration process and starts capturing video frames.
   * It processes the frames to detect the user's gaze angles and updates the calibration data.
   * The calibration process continues until the user looks at the robot for a specified number of samples.
   */
  void startCalibration()
  {
    std::cout << "Running Calibration function in Gaze Controller" << std::endl;

    if (loadCalibrationData())
      return;

    calibrator_.startCalibration();
    bool is_complete = false;

    if (!cap_.isOpened())
      cap_.open(camera_id_);

    while (cap_.isOpened())
    {
      cv::Mat frame;
      if (!cap_.read(frame))
        break;

      cv::Mat vis;
      bool attention, sustained, face_found;
      cv::Vec3d angles;

      detector_.processFrame(frame, vis, attention, sustained, angles, face_found);

      if (face_found)
      {
        std::string msg;
        is_complete = calibrator_.processSample(angles[0], angles[1], msg);

        cv::putText(vis, msg, { 20, 110 }, cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 165, 0 }, 2);

        if (is_complete)
        {
          std::cout << "Calibration complete!" << std::endl;
          std::cout << "Baseline Pitch: " << calibrator_.getBaselinePitch() << std::endl;
          std::cout << "Baseline Yaw: " << calibrator_.getBaselineYaw() << std::endl;
          std::cout << "Pitch Threshold: " << calibrator_.getPitchThreshold() << std::endl;
          std::cout << "Yaw Threshold: " << calibrator_.getYawThreshold() << std::endl;
          saveCalibrationData();
          break;
        }
      }

      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        visualisation_frame_ = vis;
      }

      cv::imshow("Calibration", vis);
      if (cv::waitKey(5) == 27)
        break;
    }

    cap_.release();
    cv::destroyAllWindows();

    if (!is_complete)
      throw std::runtime_error("Calibration failed or interrupted.");
  }

  /**
   * @brief Start the attention detection process
   *
   * This function starts the attention detection thread, which continuously captures video frames
   * and processes them to detect the user's gaze angles and calculate the gaze score.
   */
  void startAttentionDetection()
  {
    is_in_attention_detection_mode_ = true;
    attention_thread_ = std::thread(&GazeInterfaceController::attentionLoop, this);
  }

  /**
   * @brief Stop the attention detection process
   *
   * This function stops the attention detection thread and releases the video capture.
   */
  void stop()
  {
    is_in_attention_detection_mode_ = false;
    if (cap_.isOpened())
      cap_.release();
    if (attention_thread_.joinable())
      attention_thread_.join();
    cv::destroyAllWindows();
  }

  double getGazeScore()
  {
    std::lock_guard<std::mutex> lock(score_mutex_);
    return gaze_score_;
  }

  int getRobotLooks()
  {
    std::lock_guard<std::mutex> lock(robot_looks_mutex_);
    return robot_looks_;
  }

  double getGazeEntropy()
  {
    std::lock_guard<std::mutex> lock(entropy_mutex_);
    return gaze_entropy_;
  }

  cv::Mat getVisualisationFrame()
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return visualisation_frame_.clone();
  }

private:
  int camera_id_;
  cv::VideoCapture cap_;


  /**
   * @brief Attention detection loop
   *
   * This function runs in a separate thread and continuously captures video frames,
   * processes them to detect the user's gaze angles, and calculates the gaze score.
   */
  void attentionLoop()
  {
    if (!cap_.isOpened())
      cap_.open(camera_id_);

    while (cap_.isOpened() && is_in_attention_detection_mode_)
    {
      cv::Mat frame;
      if (!cap_.read(frame))
        break;

      cv::Mat vis;
      bool attention, sustained, face_found;
      cv::Vec3d angles;

      detector_.processFrame(frame, vis, attention, sustained, angles, face_found);

      double timestamp = getTimeSeconds();
      attention_window_.emplace_back(timestamp, attention);

      GazeMetrics metrics = GazeUtils::calculateAttentionMetrics(attention_window_, 3.0);

      {
        std::lock_guard<std::mutex> lock(score_mutex_);
        gaze_score_ = metrics.gaze_score;
      }
      {
        std::lock_guard<std::mutex> lock(robot_looks_mutex_);
        robot_looks_ = metrics.robot_looks;
      }
      {
        std::lock_guard<std::mutex> lock(entropy_mutex_);
        gaze_entropy_ = metrics.gaze_entropy;
      }

      if (face_found)
      {
        int h = frame.rows;
        cv::putText(vis, "Baseline Pitch: " + std::to_string(calibrator_.getBaselinePitch()), { 20, 110 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 165, 0 }, 2);
        cv::putText(vis, "Baseline Yaw: " + std::to_string(calibrator_.getBaselineYaw()), { 20, 140 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 165, 0 }, 2);
        cv::putText(vis, "Attention Ratio: " + std::to_string(metrics.attention_ratio), { 20, h - 90 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 165, 0 }, 2);
        cv::putText(vis, "Gaze Entropy: " + std::to_string(metrics.gaze_entropy), { 20, h - 60 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 165, 0 }, 2);
        cv::putText(vis, "Frames in Window: " + std::to_string(metrics.frames_in_interval), { 20, h - 30 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 255, 165, 0 }, 2);

        {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          visualisation_frame_ = vis;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
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
    std::ifstream file("calibration_data.dat", std::ios::binary);
    if (!file)
    {
      std::cout << "No calibration data found." << std::endl;
      return false;
    }

    double bp, by, pt, yt;
    file.read(reinterpret_cast<char*>(&bp), sizeof(double));
    file.read(reinterpret_cast<char*>(&by), sizeof(double));
    file.read(reinterpret_cast<char*>(&pt), sizeof(double));
    file.read(reinterpret_cast<char*>(&yt), sizeof(double));
    file.close();

    calibrator_ = AttentionCalibrator();
    calibrator_.startCalibration();  // reinitialize to reset internal state

    // simulate loading
    const_cast<double&>(calibrator_.getBaselinePitch()) = bp;
    const_cast<double&>(calibrator_.getBaselineYaw()) = by;
    const_cast<double&>(calibrator_.getPitchThreshold()) = pt;
    const_cast<double&>(calibrator_.getYawThreshold()) = yt;

    std::cout << "Calibration data loaded." << std::endl;
    return true;
  }

  /**
   * @brief Save calibration data to a file
   *
   * This function saves the calibration data to a binary file.
   */
  void saveCalibrationData()
  {
    std::ofstream file("calibration_data.dat", std::ios::binary);
    double bp = calibrator_.getBaselinePitch();
    double by = calibrator_.getBaselineYaw();
    double pt = calibrator_.getPitchThreshold();
    double yt = calibrator_.getYawThreshold();

    file.write(reinterpret_cast<const char*>(&bp), sizeof(double));
    file.write(reinterpret_cast<const char*>(&by), sizeof(double));
    file.write(reinterpret_cast<const char*>(&pt), sizeof(double));
    file.write(reinterpret_cast<const char*>(&yt), sizeof(double));
    file.close();

    std::cout << "Calibration data saved." << std::endl;
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
};

#endif  // GAZE_INTERFACE_CONTROLLER_HPP
