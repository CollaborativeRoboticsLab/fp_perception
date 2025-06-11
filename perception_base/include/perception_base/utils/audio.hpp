#pragma once

#include <fstream>
#include <vector>
#include <cstdint>
#include <string>
#include <cstring>
#include <stdexcept>
#include <perception_base/utils/exceptions.hpp>
#include <perception_msgs/msg/perception_audio.hpp>

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

  return data;
}

/**
 * @brief Write audio data to a WAV file.
 *
 * @param filename The name of the file to write.
 * @param data The audio data to write.
 */
void writeWavFile(const std::string& filename, const audio_data& data)
{
  std::ofstream out(filename, std::ios::binary);
  if (!out)
    throw perception_exception("Failed to open file for writing: " + filename);

  uint32_t data_size = data.samples.size() * sizeof(int16_t);
  uint32_t fmt_chunk_size = 16;
  uint16_t audio_format = 1;  // PCM
  uint16_t bits_per_sample = 16;
  uint32_t byte_rate = data.sample_rate * data.channels * bits_per_sample / 8;
  uint16_t block_align = data.channels * bits_per_sample / 8;
  uint32_t chunk_size = 4 + (8 + fmt_chunk_size) + (8 + data_size);

  // Write WAV header
  out.write("RIFF", 4);
  out.write(reinterpret_cast<const char*>(&chunk_size), 4);
  out.write("WAVE", 4);

  // fmt subchunk
  out.write("fmt ", 4);
  out.write(reinterpret_cast<const char*>(&fmt_chunk_size), 4);
  out.write(reinterpret_cast<const char*>(&audio_format), 2);
  out.write(reinterpret_cast<const char*>(&data.channels), 2);
  out.write(reinterpret_cast<const char*>(&data.sample_rate), 4);
  out.write(reinterpret_cast<const char*>(&byte_rate), 4);
  out.write(reinterpret_cast<const char*>(&block_align), 2);
  out.write(reinterpret_cast<const char*>(&bits_per_sample), 2);

  // data subchunk
  out.write("data", 4);
  out.write(reinterpret_cast<const char*>(&data_size), 4);
  out.write(reinterpret_cast<const char*>(data.samples.data()), data_size);

  out.close();
}

/**
 * @brief Read audio data from a WAV file.
 *
 * @param filepath Path to the WAV file.
 * @return audio_data Struct containing samples, sample rate, and channel count.
 * @throws perception_exception on file errors or unsupported formats.
 */
audio_data readWavFile(const std::string& filepath)
{
  std::ifstream in(filepath, std::ios::binary);
  if (!in)
    throw perception_exception("Failed to open WAV file: " + filepath);

  char riff_header[4];
  in.read(riff_header, 4);
  if (std::strncmp(riff_header, "RIFF", 4) != 0)
    throw perception_exception("Invalid WAV file: Missing 'RIFF'");

  in.seekg(20);
  uint16_t audio_format = 0;
  in.read(reinterpret_cast<char*>(&audio_format), 2);
  if (audio_format != 1)
    throw perception_exception("Unsupported WAV format: Only PCM is supported");

  uint16_t channels = 0;
  in.read(reinterpret_cast<char*>(&channels), 2);

  uint32_t sample_rate = 0;
  in.read(reinterpret_cast<char*>(&sample_rate), 4);

  in.seekg(34);
  uint16_t bits_per_sample = 0;
  in.read(reinterpret_cast<char*>(&bits_per_sample), 2);
  if (bits_per_sample != 16)
    throw perception_exception("Unsupported bit depth: Only 16-bit PCM is supported");

  // Seek to 'data' subchunk
  char chunk_id[4];
  uint32_t chunk_size = 0;
  while (in.read(chunk_id, 4))
  {
    in.read(reinterpret_cast<char*>(&chunk_size), 4);
    if (std::strncmp(chunk_id, "data", 4) == 0)
      break;

    // Skip other chunks
    in.seekg(chunk_size, std::ios::cur);
  }

  if (std::strncmp(chunk_id, "data", 4) != 0)
    throw perception_exception("Invalid WAV file: Missing 'data' chunk");

  // Read samples
  std::vector<int16_t> samples(chunk_size / sizeof(int16_t));
  in.read(reinterpret_cast<char*>(samples.data()), chunk_size);

  audio_data audio;
  audio.samples = std::move(samples);
  audio.sample_rate = static_cast<int>(sample_rate);
  audio.channels = static_cast<int>(channels);
  audio.chunk_size = 256;  // default

  return audio;
}

}  // namespace perception
