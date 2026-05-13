#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>

#include <perception_base/audio/structs.hpp>
#include <perception_base/exceptions.hpp>

namespace perception
{

class AudioBuffer
{
public:
  void append(const audio_data& data, int max_duration_seconds)
  {
    if (data.samples.empty())
      return;

    std::unique_lock<std::mutex> lock(mutex_);

    if (buffer_.sample_rate != data.sample_rate || buffer_.channels != data.channels)
    {
      buffer_.samples.clear();
      total_samples_ = 0;
    }

    buffer_.sample_rate = data.sample_rate;
    buffer_.channels = data.channels;
    buffer_.chunk_size = data.chunk_size;
    buffer_.chunk_count = data.chunk_count;
    buffer_.override = data.override;

    buffer_.samples.insert(buffer_.samples.end(), data.samples.begin(), data.samples.end());
    total_samples_ += static_cast<uint64_t>(data.samples.size());

    const size_t max_buffer_size = static_cast<size_t>(std::max(1, data.sample_rate)) *
                                   static_cast<size_t>(std::max(1, data.channels)) *
                                   static_cast<size_t>(std::max(1, max_duration_seconds));

    if (buffer_.samples.size() > max_buffer_size)
    {
      const size_t excess = buffer_.samples.size() - max_buffer_size;
      buffer_.samples.erase(buffer_.samples.begin(), buffer_.samples.begin() + static_cast<std::ptrdiff_t>(excess));
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

private:
  bool isInitializedLocked() const
  {
    return buffer_.sample_rate > 0 && buffer_.channels > 0 && !buffer_.samples.empty();
  }

  audio_data buffer_;
  std::mutex mutex_;
  std::condition_variable condition_variable_;
  uint64_t total_samples_{ 0 };
};

}  // namespace perception