#pragma once

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <perception_msgs/msg/perception_audio.hpp>
#include <perception_msgs/msg/perception_text.hpp>

namespace perception
{

/**
 * @brief Struct to hold audio data.
 */
struct audio_data
{
  std::vector<int16_t> samples;  ///< Audio samples
  int sample_rate = 44100;       ///< Sample rate in Hz
  int channels = 1;              ///< Number of audio channels
  int chunk_size = 256;          ///< Size of each audio chunk in samples
  int chunk_count = 0;           ///< Number of chunks in the audio data
  bool override = false;  ///< if the system sample_rate, channels, and chunk_size should be overridden by message data
};

struct text_data
{
  std::string text;          ///< Text data to be processed
  std::string voice;         ///< Options for processing the text
  std::string instructions;  ///< Model to be used for processing
};

/**
 * @brief Convert perception::audio_data to perception_msgs::msg::PerceptionAudio message.
 *
 * @param data The audio data to convert.
 * @return perception_msgs::msg::PerceptionAudio The converted message.
 */
perception_msgs::msg::PerceptionAudio audio_data_to_msg(const audio_data& data)
{
  perception_msgs::msg::PerceptionAudio msg;

  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "audio_frame";  // Default frame ID, can be changed as needed
  msg.sample_rate = data.sample_rate;
  msg.channels = data.channels;
  msg.chunk_size = data.chunk_size;
  msg.chunk_count = data.chunk_count;
  msg.samples = data.samples;
  msg.override = data.override;

  return msg;
}

/**
 * @brief Convert perception_msgs::msg::PerceptionAudio message to audio_data.
 *
 * @param msg The message to convert.
 * @return audio_data The converted audio data.
 */
audio_data msg_to_audio_data(const perception_msgs::msg::PerceptionAudio& msg)
{
  audio_data data;

  data.samples = msg.samples;
  data.sample_rate = msg.sample_rate;
  data.channels = msg.channels;
  data.chunk_size = msg.chunk_size;
  data.chunk_count = msg.chunk_count;
  data.override = msg.override;

  return data;
}

perception_msgs::msg::PerceptionText text_data_to_msg(const text_data& data)
{
  perception_msgs::msg::PerceptionText msg;

  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "text_frame";  // Default frame ID, can be changed as needed
  msg.text = data.text;
  msg.voice = data.voice;
  msg.instructions = data.instructions;

  return msg;
}

text_data msg_to_text_data(const perception_msgs::msg::PerceptionText& msg)
{
  text_data data;

  data.text = msg.text;
  data.voice = msg.voice;
  data.instructions = msg.instructions;

  return data;
}

}  // namespace perception
