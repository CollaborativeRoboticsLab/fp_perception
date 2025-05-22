#ifndef ATTENTION_DETECTOR_HPP
#define ATTENTION_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <deque>
#include <chrono>
#include <vector>
#include <perception_utils/3rdparty/facemesh/facemesh.hpp>

/* * AttentionDetector class
 * This class detects attention based on head pose estimation using a face mesh model.
 * It uses OpenCV for image processing and TensorFlow Lite for model inference.
 * The class provides methods to process video frames, detect attention, and calculate head pose angles.
 * It also includes a smoothing function for the angles to reduce noise.
 *
 */
class AttentionDetector
{
public:
  /**
   * @brief Default constructor for AttentionDetector
   *
   */
  AttentionDetector()
  {
  }

  /**
   * @brief Construct a new Attention Detector object
   *
   * @param attention_threshold Attention threshold in seconds
   * @param pitch_threshold Pitch threshold in degrees
   * @param yaw_threshold  Yaw threshold in degrees
   * @param history_size History size for smoothing angles
   * @param model_path Path to the face landmark model
   * @param attention_state Initial attention state
   */
  AttentionDetector(double attention_threshold = 0.5, double pitch_threshold = 15.0, double yaw_threshold = 20.0,
                    size_t history_size = 10, const std::string& model_path = "face_landmark.tflite",
                    bool attention_state = false)
    : attention_threshold_(attention_threshold)
    , pitch_threshold_(pitch_threshold)
    , yaw_threshold_(yaw_threshold)
    , attention_state_(attention_state)
    , history_size_(history_size)
  {
    face_mesh_.load_model(model_path);

    // Define face model 3D points
    face_3d_model_ = {
      { 0, 0, 0 },              // Nose tip
      { 0, -63.6, -12.5 },      // Chin
      { -43.3, 32.7, -26 },     // Left eye corner
      { -28.9, -28.9, -24.1 },  // Left mouth corner
      { 43.3, 32.7, -26 },      // Right eye corner
      { 28.9, -28.9, -24.1 }    // Right mouth corner
    };
  }

  /**
   * @brief Process a video frame to detect attention and head pose angles
   *
   * @param frame Input video frame
   * @param output Output video frame with overlays
   * @param attention_detected Reference to store attention detection result
   * @param sustained_attention Reference to store sustained attention result
   * @param angles Reference to store head pose angles (pitch, yaw, roll)
   * @param face_found Reference to store face detection result
   * @return true if processing was successful, false otherwise
   */
  bool processFrame(const cv::Mat& frame, cv::Mat& output, bool& attention_detected, bool& sustained_attention,
                    cv::Vec3d& angles, bool& face_found)
  {
    output = frame.clone();
    face_found = false;
    attention_detected = false;
    sustained_attention = false;
    angles = { 0, 0, 0 };

    // Load image and run inference
    face_mesh_.load_image(output);

    // Get the 3D Face landmarks
    auto face_mesh_keypoints = face_mesh_.get_face_mesh_points();

    if (face_mesh_keypoints.size() < 360)  // Ensure needed keypoints are available
      return false;

    face_found = true;

    std::vector<cv::Point2f> face_2d = {
      { face_mesh_keypoints[1].x, face_mesh_keypoints[1].y },      // Nose tip
      { face_mesh_keypoints[152].x, face_mesh_keypoints[152].y },  // Chin
      { face_mesh_keypoints[263].x, face_mesh_keypoints[263].y },  // Left eye corner
      { face_mesh_keypoints[61].x, face_mesh_keypoints[61].y },    // Left mouth corner
      { face_mesh_keypoints[33].x, face_mesh_keypoints[33].y },    // Right eye corner
      { face_mesh_keypoints[291].x, face_mesh_keypoints[291].y }   // Right mouth corner
    };

    // Camera matrix
    int w = frame.cols;
    int h = frame.rows;
    double focal_length = 1.0 * w;

    cv::Mat cam_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, w / 2.0, 0, focal_length, h / 2.0, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

    // Define 3D face model points
    std::vector<cv::Point3d> face_3d_model = {
      { 0.0, 0.0, 0.0 },        // Nose tip
      { 0.0, -63.6, -12.5 },    // Chin
      { -43.3, 32.7, -26 },     // Left eye
      { -28.9, -28.9, -24.1 },  // Left mouth
      { 43.3, 32.7, -26 },      // Right eye
      { 28.9, -28.9, -24.1 }    // Right mouth
    };

    cv::Vec3d rot_vec, trans_vec;
    bool success = cv::solvePnP(face_3d_model, face_2d, cam_matrix, dist_coeffs, rot_vec, trans_vec);
    if (!success)
      return false;

    cv::Mat rot_matrix;
    cv::Rodrigues(rot_vec, rot_matrix);
    angles = smoothAngles(rotationMatrixToEulerAngles(rot_matrix));

    double pitch = angles[0];
    double yaw = angles[1];

    attention_detected = isLookingAtRobot(pitch, yaw);

    auto now = std::chrono::steady_clock::now();
    if (attention_detected)
    {
      if (attention_start_time_.time_since_epoch().count() == 0)
      {
        attention_start_time_ = now;
      }
      else
      {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - attention_start_time_).count();
        if (elapsed >= attention_threshold_ * 1000.0)
        {
          sustained_attention = true;
        }
      }
    }
    else
    {
      attention_start_time_ = std::chrono::steady_clock::time_point();
    }

    // Draw overlays
    cv::Scalar color = sustained_attention ? cv::Scalar(0, 255, 0) :
                                             (attention_detected ? cv::Scalar(0, 165, 255) : cv::Scalar(0, 0, 255));

    cv::putText(output, "Pitch: " + std::to_string(int(pitch)), { 20, 20 }, cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
    cv::putText(output, "Yaw: " + std::to_string(int(yaw)), { 20, 50 }, cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

    std::string status =
        sustained_attention ? "Sustained Attention" : (attention_detected ? "Attention Detected" : "No Attention");
    cv::putText(output, status, { 20, 80 }, cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

    // Nose direction line
    cv::Point2f nose_2d = face_2d[0];
    cv::Point2f p2 = cv::Point2f(nose_2d.x + yaw, nose_2d.y - pitch);
    cv::line(output, nose_2d, p2, color, 2);
    
    return true;
  }

private:
  /**
   * @brief Smooth the angles using a moving average filter
   *
   * @param new_angles new angles to be added to the history
   * @return cv::Vec3d smoothed angles
   */
  cv::Vec3d smoothAngles(const cv::Vec3d& new_angles)
  {
    angle_history_.push_back(new_angles);
    if (angle_history_.size() > history_size_)
      angle_history_.pop_front();

    cv::Vec3d sum(0, 0, 0);
    for (const auto& angle : angle_history_)
      sum += angle;

    return sum / static_cast<double>(angle_history_.size());
  }

  /**
   * @brief Convert rotation matrix to Euler angles
   *
   * @param R rotation matrix
   * @return cv::Vec3d Euler angles (pitch, yaw, roll)
   */
  cv::Vec3d rotationMatrixToEulerAngles(const cv::Mat& R)
  {
    double pitch = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    double yaw = atan2(-R.at<double>(2, 0), sqrt(pow(R.at<double>(0, 0), 2) + pow(R.at<double>(1, 0), 2)));
    double roll = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    return { pitch * 180.0 / CV_PI, yaw * 180.0 / CV_PI, roll * 180.0 / CV_PI };
  }

  /**
   * @brief Check if the head pose angles are within the defined thresholds
   *
   * @param pitch head pose pitch angle
   * @param yaw head pose yaw angle
   * @return true if looking at the robot, false otherwise
   */
  virtual bool isLookingAtRobot(double pitch, double yaw) const
  {
    return std::abs(pitch) < pitch_threshold_ && std::abs(yaw) < yaw_threshold_;
  }

  /** attention threshold in seconds */
  double attention_threshold_;

  /** pitch and yaw thresholds in degrees */
  double pitch_threshold_;
  double yaw_threshold_;

  /** attention start time */
  std::chrono::steady_clock::time_point attention_start_time_;

  /** attention state */
  bool attention_state_;

  /** pitch and yaw angles history */
  std::deque<cv::Vec3d> angle_history_;
  size_t history_size_;

  /** model */
  std::vector<cv::Point3d> face_3d_model_;

  /** Face mesh model */
  CLFML::FaceMesh::FaceMesh face_mesh_;
};

#endif  // ATTENTION_DETECTOR_HPP