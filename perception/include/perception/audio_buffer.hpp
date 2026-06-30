#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <perception_base/audio/structs.hpp>
#include <perception_base/exceptions.hpp>

namespace perception
{

class AudioBuffer
{
public:
  static constexpr const char* kExpiredAudioSliceError =
      "error: requested audio slice is expired. for future interactions, increase the audio retention window parameter.";

  void append(const audio_data& data, int max_duration_seconds)
  {
    append(data, max_duration_seconds, rclcpp::Clock().now());
  }

  void append(const audio_data& data, int max_duration_seconds, const rclcpp::Time& end_time)
  {
    if (data.samples.empty())
      return;

    std::unique_lock<std::mutex> lock(mutex_);

    const int channels = std::max(1, data.channels);
    const int sample_rate = std::max(1, data.sample_rate);
    max_duration_seconds_ = std::max(1, max_duration_seconds);
    const size_t incoming_frames = data.samples.size() / static_cast<size_t>(channels);
    if (incoming_frames == 0)
      return;

    if (buffer_.sample_rate != data.sample_rate || buffer_.channels != data.channels)
    {
      if (!buffer_.samples.empty())
      {
        RCLCPP_WARN(rclcpp::get_logger("AudioBuffer"),
                    "Resetting public audio buffer due to format change: sample_rate %d -> %d, channels %d -> %d, buffered_samples=%zu",
                    buffer_.sample_rate, data.sample_rate, buffer_.channels, data.channels, buffer_.samples.size());
      }
      buffer_.samples.clear();
      total_samples_ = 0;
      total_frames_ = 0;
      initialized_time_ = false;
      buffer_end_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    const rclcpp::Duration incoming_duration = framesToDuration(incoming_frames, sample_rate);

    if (!initialized_time_)
    {
      buffer_end_time_ = end_time;
      buffer_start_time_ = buffer_end_time_ - incoming_duration;
      initialized_time_ = true;
    }
    else if (end_time > buffer_end_time_)
    {
      const auto observed_gap = end_time - buffer_end_time_;
      if (observed_gap > incoming_duration)
      {
        const auto missing_duration = observed_gap - incoming_duration;
        const size_t missing_frames = durationToFrames(missing_duration, sample_rate);
        const size_t missing_samples = missing_frames * static_cast<size_t>(channels);
        if (missing_samples > 0)
        {
          buffer_.samples.insert(buffer_.samples.end(), missing_samples, 0);
          total_samples_ += static_cast<uint64_t>(missing_samples);
          total_frames_ += static_cast<uint64_t>(missing_frames);
        }
      }
    }

    buffer_.sample_rate = data.sample_rate;
    buffer_.channels = data.channels;
    buffer_.chunk_size = data.chunk_size;
    buffer_.chunk_count = data.chunk_count;
    buffer_.override = data.override;

    buffer_.samples.insert(buffer_.samples.end(), data.samples.begin(), data.samples.end());
    total_samples_ += static_cast<uint64_t>(data.samples.size());
    total_frames_ += static_cast<uint64_t>(incoming_frames);

    const size_t max_buffer_size = static_cast<size_t>(std::max(1, data.sample_rate)) *
                                   static_cast<size_t>(std::max(1, data.channels)) *
                                   static_cast<size_t>(std::max(1, max_duration_seconds));

    if (buffer_.samples.size() > max_buffer_size)
    {
      const size_t excess_samples = buffer_.samples.size() - max_buffer_size;
      const size_t excess_frames = excess_samples / static_cast<size_t>(channels);
      const size_t samples_to_remove = excess_frames * static_cast<size_t>(channels);
      if (samples_to_remove > 0)
      {
        buffer_.samples.erase(buffer_.samples.begin(),
                              buffer_.samples.begin() + static_cast<std::ptrdiff_t>(samples_to_remove));
        buffer_start_time_ = buffer_start_time_ + framesToDuration(excess_frames, sample_rate);
      }
    }

    buffer_end_time_ = end_time;
    const size_t buffered_frames = buffer_.samples.size() / static_cast<size_t>(channels);
    buffer_start_time_ = buffer_end_time_ - framesToDuration(buffered_frames, sample_rate);

    const size_t denom = static_cast<size_t>(std::max(1, data.chunk_size)) *
                         static_cast<size_t>(std::max(1, data.channels));
    buffer_.chunk_count = denom ? (buffer_.samples.size() / denom) : 0;

    const double buffered_seconds = static_cast<double>(buffered_frames) / static_cast<double>(sample_rate);
    if (buffered_seconds < 0.5)
    {
      RCLCPP_WARN(rclcpp::get_logger("AudioBuffer"),
                  "Public audio buffer remains small after append: added_frames=%zu buffered_frames=%zu buffered_seconds=%.3f retention=%d start=%.9f end=%.9f",
                  incoming_frames, buffered_frames, buffered_seconds, max_duration_seconds,
                  buffer_start_time_.seconds(), buffer_end_time_.seconds());
    }

    lock.unlock();
    condition_variable_.notify_all();
  }

  void waitForAudio(std::chrono::seconds timeout = std::chrono::seconds(5))
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (isInitializedLocked())
      return;

    condition_variable_.wait_for(lock, timeout, [this] { return isInitializedLocked(); });

    if (!isInitializedLocked())
      throw perception_exception("public audio buffer not initialized");
  }

  audio_data readLatest(int duration_seconds)
  {
    duration_seconds = std::max(1, duration_seconds);
    waitForAudio();

    std::lock_guard<std::mutex> lock(mutex_);

    const int sample_rate = buffer_.sample_rate;
    const int channels = std::max(1, buffer_.channels);
    const size_t requested_samples = static_cast<size_t>(sample_rate) * static_cast<size_t>(channels) *
                                     static_cast<size_t>(duration_seconds);
    const size_t available_samples = buffer_.samples.size();

    if (available_samples == 0)
      throw perception_exception("public audio buffer is empty");

    audio_data out = buffer_;
    out.sample_rate = sample_rate;
    out.channels = channels;
    out.chunk_count = 1;

    const size_t samples_to_copy = std::min(requested_samples, available_samples);
    const size_t start_index = available_samples - samples_to_copy;

    out.samples.assign(buffer_.samples.begin() + static_cast<std::ptrdiff_t>(start_index), buffer_.samples.end());
    out.chunk_size = static_cast<int>(samples_to_copy / static_cast<size_t>(channels));

    return out;
  }

  audio_data readWindow(const rclcpp::Time& start_time, int duration_seconds)
  {
    duration_seconds = std::max(1, duration_seconds);
    waitForAudio();

    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_time_ || buffer_.samples.empty())
      throw perception_exception("public audio buffer timestamp state is not initialized");

    const int sample_rate = std::max(1, buffer_.sample_rate);
    const int channels = std::max(1, buffer_.channels);
    const rclcpp::Time buffer_end_time = currentEndTimeLocked();
    const rclcpp::Time requested_end_time = start_time + rclcpp::Duration::from_seconds(duration_seconds);

    audio_data out = buffer_;
    const size_t requested_frames = static_cast<size_t>(sample_rate) * static_cast<size_t>(duration_seconds);
    out.samples.assign(requested_frames * static_cast<size_t>(channels), 0);
    out.chunk_size = static_cast<int>(requested_frames);
    out.chunk_count = 1;
    out.sample_rate = sample_rate;
    out.channels = channels;

    const rclcpp::Time overlap_start = start_time < buffer_start_time_ ? buffer_start_time_ : start_time;
    const rclcpp::Time overlap_end = requested_end_time > buffer_end_time ? buffer_end_time : requested_end_time;
    if (overlap_end <= overlap_start)
      return out;

    const size_t source_start_frame = durationToFrames(overlap_start - buffer_start_time_, sample_rate);
    const size_t output_start_frame = durationToFrames(overlap_start - start_time, sample_rate);
    size_t frames_to_copy = durationToFrames(overlap_end - overlap_start, sample_rate);

    const size_t available_frames = buffer_.samples.size() / static_cast<size_t>(channels);
    if (source_start_frame >= available_frames || output_start_frame >= requested_frames)
      return out;

    frames_to_copy = std::min(frames_to_copy, available_frames - source_start_frame);
    frames_to_copy = std::min(frames_to_copy, requested_frames - output_start_frame);

    const size_t source_start_sample = source_start_frame * static_cast<size_t>(channels);
    const size_t output_start_sample = output_start_frame * static_cast<size_t>(channels);
    const size_t samples_to_copy = frames_to_copy * static_cast<size_t>(channels);
    std::copy(buffer_.samples.begin() + static_cast<std::ptrdiff_t>(source_start_sample),
              buffer_.samples.begin() + static_cast<std::ptrdiff_t>(source_start_sample + samples_to_copy),
              out.samples.begin() + static_cast<std::ptrdiff_t>(output_start_sample));

    return out;
  }

  void waitForWindow(const rclcpp::Time& start_time, int duration_seconds,
                     std::chrono::seconds timeout = std::chrono::seconds(30))
  {
    duration_seconds = std::max(1, duration_seconds);
    waitForAudio(timeout);

    const rclcpp::Time requested_end_time = start_time + rclcpp::Duration::from_seconds(duration_seconds);
    std::unique_lock<std::mutex> lock(mutex_);

    condition_variable_.wait_for(lock, timeout, [this, &start_time, &requested_end_time] {
      if (!isInitializedLocked() || !initialized_time_)
        return false;

      if (isExpiredLocked(start_time))
        return true;

      return currentEndTimeLocked() >= requested_end_time;
    });

    if (!isInitializedLocked() || !initialized_time_)
      throw perception_exception("public audio buffer timestamp state is not initialized");

    if (isExpiredLocked(start_time))
    {
      const auto current_end_time = currentEndTimeLocked();
      throw perception_exception(std::string(kExpiredAudioSliceError) +
                                 " requested=[" + std::to_string(start_time.seconds()) + ", " +
                                 std::to_string(requested_end_time.seconds()) + "] buffered=[" +
                                 std::to_string(buffer_start_time_.seconds()) + ", " +
                                 std::to_string(current_end_time.seconds()) + "]");
    }

    if (currentEndTimeLocked() < requested_end_time)
    {
      const auto current_end_time = currentEndTimeLocked();
      throw perception_exception("Timeout waiting for requested audio slice. requested=[" +
                                 std::to_string(start_time.seconds()) + ", " +
                                 std::to_string(requested_end_time.seconds()) + "] buffered=[" +
                                 std::to_string(buffer_start_time_.seconds()) + ", " +
                                 std::to_string(current_end_time.seconds()) + "]");
    }
  }

  rclcpp::Time startTime() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_start_time_;
  }

  rclcpp::Time endTime() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return currentEndTimeLocked();
  }

private:
  static rclcpp::Duration framesToDuration(size_t frames, int sample_rate)
  {
    const int64_t nanoseconds = static_cast<int64_t>((static_cast<long double>(frames) * 1000000000.0L) /
                                                     static_cast<long double>(std::max(1, sample_rate)));
    return rclcpp::Duration(0, nanoseconds);
  }

  static size_t durationToFrames(const rclcpp::Duration& duration, int sample_rate)
  {
    if (duration.nanoseconds() <= 0)
      return 0;
    return static_cast<size_t>((static_cast<long double>(duration.nanoseconds()) *
                                static_cast<long double>(std::max(1, sample_rate))) /
                               1000000000.0L);
  }

  bool isInitializedLocked() const
  {
    return buffer_.sample_rate > 0 && buffer_.channels > 0 && !buffer_.samples.empty();
  }

  rclcpp::Time currentEndTimeLocked() const
  {
    return buffer_end_time_;
  }

  bool isExpiredLocked(const rclcpp::Time& start_time) const
  {
    if (max_duration_seconds_ <= 0)
      return false;

    const auto retention_start = currentEndTimeLocked() - rclcpp::Duration::from_seconds(max_duration_seconds_);
    return retention_start > start_time;
  }

  audio_data buffer_;
  mutable std::mutex mutex_;
  std::condition_variable condition_variable_;
  uint64_t total_samples_{ 0 };
  uint64_t total_frames_{ 0 };
  int max_duration_seconds_{ 0 };
  bool initialized_time_{ false };
  rclcpp::Time buffer_start_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time buffer_end_time_{ 0, 0, RCL_ROS_TIME };
};

}  // namespace perception