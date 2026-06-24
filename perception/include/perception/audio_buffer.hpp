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
    const size_t incoming_frames = data.samples.size() / static_cast<size_t>(channels);
    if (incoming_frames == 0)
      return;

    if (buffer_.sample_rate != data.sample_rate || buffer_.channels != data.channels)
    {
      buffer_.samples.clear();
      total_samples_ = 0;
      total_frames_ = 0;
      initialized_time_ = false;
    }

    if (!initialized_time_)
    {
      buffer_start_time_ = end_time - framesToDuration(incoming_frames, sample_rate);
      initialized_time_ = true;
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

    const size_t denom = static_cast<size_t>(std::max(1, data.chunk_size)) *
                         static_cast<size_t>(std::max(1, data.channels));
    buffer_.chunk_count = denom ? (buffer_.samples.size() / denom) : 0;

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
    const size_t available_frames = buffer_.samples.size() / static_cast<size_t>(channels);
    const rclcpp::Time buffer_end_time = buffer_start_time_ + framesToDuration(available_frames, sample_rate);
    const rclcpp::Time requested_end_time = start_time + rclcpp::Duration::from_seconds(duration_seconds);

    if (requested_end_time <= buffer_start_time_ || start_time >= buffer_end_time)
    {
      throw perception_exception("requested audio time window is outside the buffered audio range");
    }

    const rclcpp::Time clipped_start = start_time < buffer_start_time_ ? buffer_start_time_ : start_time;
    const rclcpp::Time clipped_end = requested_end_time > buffer_end_time ? buffer_end_time : requested_end_time;

    const size_t start_frame = durationToFrames(clipped_start - buffer_start_time_, sample_rate);
    size_t end_frame = durationToFrames(clipped_end - buffer_start_time_, sample_rate);
    end_frame = std::min(end_frame, available_frames);

    if (end_frame <= start_frame)
      throw perception_exception("requested audio time window contains no buffered frames");

    const size_t start_sample = start_frame * static_cast<size_t>(channels);
    const size_t end_sample = end_frame * static_cast<size_t>(channels);

    audio_data out = buffer_;
    out.samples.assign(buffer_.samples.begin() + static_cast<std::ptrdiff_t>(start_sample),
                       buffer_.samples.begin() + static_cast<std::ptrdiff_t>(end_sample));
    out.chunk_size = static_cast<int>(end_frame - start_frame);
    out.chunk_count = 1;
    out.sample_rate = sample_rate;
    out.channels = channels;

    return out;
  }

  rclcpp::Time startTime() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_start_time_;
  }

  rclcpp::Time endTime() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const int sample_rate = std::max(1, buffer_.sample_rate);
    const int channels = std::max(1, buffer_.channels);
    const size_t available_frames = buffer_.samples.size() / static_cast<size_t>(channels);
    return buffer_start_time_ + framesToDuration(available_frames, sample_rate);
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

  audio_data buffer_;
  mutable std::mutex mutex_;
  std::condition_variable condition_variable_;
  uint64_t total_samples_{ 0 };
  uint64_t total_frames_{ 0 };
  bool initialized_time_{ false };
  rclcpp::Time buffer_start_time_{ 0, 0, RCL_ROS_TIME };
};

}  // namespace perception