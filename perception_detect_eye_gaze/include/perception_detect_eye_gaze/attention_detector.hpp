#ifndef ATTENTION_DETECTOR_HPP
#define ATTENTION_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <deque>
#include <chrono>
#include <vector>
#include <array>
#include <string>
#include <iostream>
#include <torch/script.h>

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
  /* Number of face landmarks */
  static constexpr size_t NUM_OF_FACE_MESH_POINTS = 468;

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
                    size_t history_size = 10, const std::string& model_path = "face_landmark.pt")
    : attention_threshold_(attention_threshold)
    , pitch_threshold_(pitch_threshold)
    , yaw_threshold_(yaw_threshold)
    , history_size_(history_size)
  {
    try
    {
      face_mesh_ = torch::jit::load(model_path);
      face_mesh_.eval();
    }
    catch (const c10::Error& e)
    {
      std::cerr << "Error loading model: " << e.what() << std::endl;
      exit(1);
    }

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

    // Camera matrix
    int w = frame.cols;
    int h = frame.rows;
    double focal_length = 1.0 * w;

    // Initialize ROI offset (if needed, can be set to zero)
    cv::Rect roi_offset = cv::Rect(0, 0, 0, 0);

    cv::Mat preprocessed_image = preprocess_image(output);
    torch::Tensor input_tensor = convert_mat_to_tensor(preprocessed_image);

    // Forward pass
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor);

    torch::jit::IValue output_ivalue = face_mesh_.forward(inputs);

    auto output_list = output_ivalue.toList();
    at::Tensor output_tensor = output_list.get(0).toTensor();

    postprocess_output(output_tensor, w, h, roi_offset);

    if (face_mesh_landmarks.size() < 360)  // Ensure needed keypoints are available
      return false;

    face_found = true;

    std::vector<cv::Point2f> face_2d = {
      { face_mesh_landmarks[1].x, face_mesh_landmarks[1].y },      // Nose tip
      { face_mesh_landmarks[152].x, face_mesh_landmarks[152].y },  // Chin
      { face_mesh_landmarks[263].x, face_mesh_landmarks[263].y },  // Left eye corner
      { face_mesh_landmarks[61].x, face_mesh_landmarks[61].y },    // Left mouth corner
      { face_mesh_landmarks[33].x, face_mesh_landmarks[33].y },    // Right eye corner
      { face_mesh_landmarks[291].x, face_mesh_landmarks[291].y }   // Right mouth corner
    };

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
   * @brief Preprocess the input image for the model
   *
   * @param in Input OpenCV Mat containing the image
   * @return cv::Mat Preprocessed image ready for model input
   */
  cv::Mat preprocess_image(const cv::Mat& in)
  {
    cv::Mat rgb, resized, float_img;
    cv::cvtColor(in, rgb, cv::COLOR_BGR2RGB);
    cv::resize(rgb, resized, cv::Size(192, 192));
    resized.convertTo(float_img, CV_32FC3, 1.0 / 191.5, -1.0);
    return float_img;
  }

  /**
   * @brief Convert OpenCV Mat to Torch tensor
   *
   * @param img Input OpenCV Mat containing the image
   * @return torch::Tensor Converted tensor in NCHW format
   */
  torch::Tensor convert_mat_to_tensor(const cv::Mat& img)
  {
    auto tensor = torch::from_blob(img.data, { 1, img.rows, img.cols, 3 }, torch::kFloat32);
    tensor = tensor.permute({ 0, 3, 1, 2 });  // NHWC to NCHW
    return tensor.clone();                    // clone to ensure memory safety
  }

  /**
   * @brief Postprocess the model output to extract facial landmarks
   *
   * @param output Model output tensor containing the facial landmarks
   * @param img_w Width of the original image
   * @param img_h Height of the original image
   * @param roi_offset Region of interest offset to adjust the landmarks
   */
  void postprocess_output(const at::Tensor& output, int img_w, int img_h, const cv::Rect& roi_offset)
  {
    auto output_flat = output.view({ -1 });
    for (size_t i = 0; i < NUM_OF_FACE_MESH_POINTS; ++i)
    {
      float x = output_flat[i * 3].item<float>();
      float y = output_flat[i * 3 + 1].item<float>();
      float z = output_flat[i * 3 + 2].item<float>();

      face_mesh_landmarks[i].x = (x * img_w) + roi_offset.x;
      face_mesh_landmarks[i].y = (y * img_h) + roi_offset.y;
      face_mesh_landmarks[i].z = z;
    }
  }

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
  torch::jit::script::Module face_mesh_;

  /** Number of face mesh points */
  std::array<cv::Point3f, NUM_OF_FACE_MESH_POINTS> face_mesh_landmarks;
};

#endif  // ATTENTION_DETECTOR_HPP