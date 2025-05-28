#ifndef GAZE_UTILS_HPP
#define GAZE_UTILS_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>
#include <perception_algo_eye_gaze_detection/utils/gaze_metrics.hpp>

class GazeUtils
{
public:
  /**
   * @brief Calculate attention metrics based on gaze data.
   *
   * @param window A vector of pairs containing timestamps and gaze states.
   * @param interval_duration The duration of the time window for calculating metrics.
   * @return GazeMetrics Struct containing calculated metrics.
   */
  static GazeMetrics calculateAttentionMetrics(const std::vector<std::pair<double, bool>>& window,
                                               double interval_duration = 3.0)
  {
    GazeMetrics metrics;

    if (window.empty())
      return metrics;

    double current_time = window.back().first;
    std::vector<std::pair<double, bool>> filtered_window;

    for (const auto& [timestamp, state] : window)
    {
      if (current_time - timestamp <= interval_duration)
      {
        filtered_window.emplace_back(timestamp, state);
      }
    }

    int frames_in_interval = filtered_window.size();
    int robot_looks = 0;
    for (const auto& [_, state] : filtered_window)
    {
      if (state)
        robot_looks++;
    }

    int non_robot_looks = frames_in_interval - robot_looks;
    double attention_ratio = (frames_in_interval > 0) ? static_cast<double>(robot_looks) / frames_in_interval : 0.0;

    double gaze_entropy = 0.0;
    if (frames_in_interval > 0)
    {
      double p_robot = static_cast<double>(robot_looks) / frames_in_interval;
      double p_non_robot = static_cast<double>(non_robot_looks) / frames_in_interval;

      if (p_robot > 0)
        gaze_entropy -= p_robot * std::log2(p_robot);
      if (p_non_robot > 0)
        gaze_entropy -= p_non_robot * std::log2(p_non_robot);
    }

    double normalized_attention_ratio = std::min(attention_ratio, 1.0);
    double normalized_entropy = 1.0 - std::min(gaze_entropy, 1.0);

    double gaze_score = 0.0;
    if (gaze_entropy == 1.0 || (robot_looks > 30 && gaze_entropy > 0.7 && gaze_entropy < 1.0))
    {
      gaze_score = 100.0 * normalized_attention_ratio;
    }
    else
    {
      gaze_score = 100.0 * (normalized_attention_ratio * normalized_entropy);
    }

    gaze_score = std::clamp(gaze_score, 0.0, 100.0);

    metrics.attention_ratio = attention_ratio;
    metrics.gaze_entropy = gaze_entropy;
    metrics.frames_in_interval = frames_in_interval;
    metrics.robot_looks = robot_looks;
    metrics.non_robot_looks = non_robot_looks;
    metrics.gaze_score = gaze_score;

    return metrics;
  }
};

#endif  // GAZE_UTILS_HPP
