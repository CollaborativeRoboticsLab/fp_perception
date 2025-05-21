#ifndef ATTENTION_CALIBRATOR_HPP
#define ATTENTION_CALIBRATOR_HPP

#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <sstream>
#include <iostream>

/**
 * @brief AttentionCalibrator class
 *
 * This class is responsible for calibrating the attention detection system.
 *
 */
class AttentionCalibrator
{
public:

  /**
   * @brief Default constructor for AtterntionCalibrator
   *
   */
  AttentionCalibrator()
  {}

  /**
   * @brief Constructor for AttentionCalibrator
   *
   * @param calibration_time Time in seconds to wait for calibration
   * @param samples_needed Number of samples needed for calibration
   * @param angle_tolerance Angle tolerance in degrees
   */
  AttentionCalibrator(double calibration_time = 10.0, size_t samples_needed = 300, double angle_tolerance = 15.0)
    : calibration_time_(calibration_time)
    , samples_needed_(samples_needed)
    , angle_tolerance_(angle_tolerance)
    , is_calibrated_(false)
    , baseline_pitch_(0.0)
    , baseline_yaw_(0.0)
    , pitch_threshold_(0.0)
    , yaw_threshold_(0.0)
  {
  }

  /**
   * @brief Start the calibration process
   *
   * This function clears previous samples and starts a new calibration session.
   * 
   * @param message_out Output message indicating the calibration status
   */
  void startCalibration( std::string& message_out)
  {
    pitch_samples_.clear();
    yaw_samples_.clear();
    is_calibrated_ = false;
    calibration_start_time_ = std::chrono::steady_clock::now();
    message_out = "Starting calibration... Please look directly at the robot.";
  }

  /**
   * @brief Process a sample for calibration
   *
   * This function processes a sample of pitch and yaw angles.
   *
   * @param pitch Pitch angle in degrees
   * @param yaw Yaw angle in degrees
   * @param message_out Output message indicating the calibration status
   * @return true if calibration is complete, false otherwise
   */
  bool processSample(double pitch, double yaw, std::string& message_out)
  {
    if (calibration_start_time_.time_since_epoch().count() == 0)
    {
      message_out = "Calibration not started";
      return false;
    }

    pitch_samples_.push_back(pitch);
    yaw_samples_.push_back(yaw);

    if (pitch_samples_.size() >= samples_needed_)
    {
      baseline_pitch_ = computeMean(pitch_samples_);
      baseline_yaw_ = computeMean(yaw_samples_);

      double pitch_std = computeStdDev(pitch_samples_, baseline_pitch_);
      double yaw_std = computeStdDev(yaw_samples_, baseline_yaw_);

      pitch_threshold_ = std::max(2.0 * pitch_std, angle_tolerance_);
      yaw_threshold_ = std::max(2.0 * yaw_std, angle_tolerance_);

      is_calibrated_ = true;
      message_out = "Calibration complete";
      return true;
    }

    std::ostringstream oss;
    oss << "Calibrating... " << pitch_samples_.size() << "/" << samples_needed_ << " samples";
    message_out = oss.str();
    return false;
  }

  /**
   * @brief Get the calibration status
   *
   * @return true if calibrated, false otherwise
   */
  bool isCalibrated() const
  {
    return is_calibrated_;
  }

  /**
   * @brief Get the baseline pitch angle
   *
   * @return double containing the baseline pitch angle
   */
  double getBaselinePitch() const
  {
    return baseline_pitch_;
  }

  /**
   * @brief Get the baseline yaw angle
   *
   * @return double containing the baseline yaw angle
   */
  double getBaselineYaw() const
  {
    return baseline_yaw_;
  }

  /**
   * @brief Get the pitch threshold
   *
   * @return double containing yaw threshold
   */
  double getPitchThreshold() const
  {
    return pitch_threshold_;
  }

  /**
   * @brief Get the yaw threshold
   *
   * @return double containing the yaw threshold
   */
  double getYawThreshold() const
  {
    return yaw_threshold_;
  }

private:
  /** Calibration time in seconds */
  std::chrono::steady_clock::time_point calibration_start_time_;

  /** Samples for pitch and yaw samples */
  std::vector<double> pitch_samples_;
  std::vector<double> yaw_samples_;

  /** Number of samples needed for calibration */
  size_t samples_needed_;

  /** Calibration parameters */
  double calibration_time_;

  /** Angle tolerance in degrees */
  double angle_tolerance_;

  /** Calibration status */
  bool is_calibrated_;

  /** Baseline pitch and yaw angles */
  double baseline_pitch_;
  double baseline_yaw_;

  /** Pitch and yaw thresholds */
  double pitch_threshold_;
  double yaw_threshold_;

  /**
   * @brief compute the mean of a vector of samples
   * 
   * @param samples samples to compute the mean of
   * @return double sample mean
   */
  static double computeMean(const std::vector<double>& samples)
  {
    if (samples.empty())
      return 0.0;
    double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    return sum / samples.size();
  }

  /**
   * @brief compute the standard deviation of a vector of samples
   * 
   * @param samples samples to compute the standard deviation of
   * @param mean sample mean
   * @return double sample standard deviation
   */
  static double computeStdDev(const std::vector<double>& samples, double mean)
  {
    if (samples.size() < 2)
      return 0.0;
    double sum_sq_diff = 0.0;
    for (double v : samples)
    {
      sum_sq_diff += (v - mean) * (v - mean);
    }
    return std::sqrt(sum_sq_diff / (samples.size() - 1));
  }
};

#endif  // ATTENTION_CALIBRATOR_HPP
