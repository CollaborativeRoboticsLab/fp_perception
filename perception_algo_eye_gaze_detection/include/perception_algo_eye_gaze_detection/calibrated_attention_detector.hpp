#ifndef CALIBRATED_ATTENTION_DETECTOR_HPP
#define CALIBRATED_ATTENTION_DETECTOR_HPP

#include <perception_algo_eye_gaze_detection/attention_detector.hpp>
#include <perception_algo_eye_gaze_detection/attention_calibrator.hpp>

/**
 * @brief CalibratedAttentionDetector class
 *
 * This class extends the AttentionDetector class to include calibration functionality.
 * It uses the AttentionCalibrator to determine if the user is looking at the robot
 * based on calibrated pitch and yaw angles.
 */
class CalibratedAttentionDetector : public AttentionDetector
{
public:
  /**
   * @brief Default constructor for CalibratedAttentionDetector
   *
   */
  CalibratedAttentionDetector() : AttentionDetector(0.5, 15.0, 20.0), calibrator_(nullptr)
  {
  }

  /**
   * @brief Constructor for CalibratedAttentionDetector
   *
   * @param calibrator Reference to the AttentionCalibrator object
   * @param attention_threshold Attention threshold in seconds
   * @param pitch_threshold Pitch threshold in angles
   * @param yaw_threshold Yaw threshold in angles
   * @param history_size History size for smoothing angles
   * @param model_path Path to the face landmark model
   * @param attention_state Initial attention state
   *
   */
  CalibratedAttentionDetector(std::shared_ptr<AttentionCalibrator> calibrator, double attention_threshold = 0.5,
                              double pitch_threshold = 15.0, double yaw_threshold = 20.0, size_t history_size = 10,
                              const std::string& model_path = "face_landmark.tflite")
    : AttentionDetector(attention_threshold, pitch_threshold, yaw_threshold, history_size, model_path)
    , calibrator_(calibrator)
  {
    if (calibrator_->isCalibrated())
    {
      baseline_pitch_ = calibrator_->getBaselinePitch();
      baseline_yaw_ = calibrator_->getBaselineYaw();
    }
  }

protected:
  /**
   * @brief Check if the head pose angles are within the defined thresholds
   *
   * @param pitch head pose pitch angle
   * @param yaw head pose yaw angle
   * @return true if looking at the robot, false otherwise
   */
  bool isLookingAtRobot(double pitch, double yaw) const override
  {
    if (!calibrator_->isCalibrated())
      return false;

    double pitch_diff = std::abs(pitch - calibrator_->getBaselinePitch());
    double yaw_diff = std::abs(yaw - calibrator_->getBaselineYaw());

    return pitch_diff < calibrator_->getPitchThreshold() && yaw_diff < calibrator_->getYawThreshold();
  }

private:
  /** Reference to the AttentionCalibrator object */
  std::shared_ptr<AttentionCalibrator> calibrator_;

  /** Baseline pitch and yaw angles */
  double baseline_pitch_{ 0.0 };
  double baseline_yaw_{ 0.0 };
};

#endif  // CALIBRATED_ATTENTION_DETECTOR_HPP
