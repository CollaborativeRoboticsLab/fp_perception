#ifndef GAZE_METRICS_HPP
#define GAZE_METRICS_HPP

/**
 * @struct GazeMetrics
 * @brief Structure to hold gaze metrics
 */
struct GazeMetrics
{
  double attention_ratio = 0.0;
  double gaze_entropy = 0.0;
  int frames_in_interval = 0;
  int robot_looks = 0;
  int non_robot_looks = 0;
  double gaze_score = 0.0;
};

#endif  // GAZE_METRICS_HPP
