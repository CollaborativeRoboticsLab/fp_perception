#pragma once

#include <fstream>
#include <vector>
#include <cstdint>
#include <string>
#include <cstring>
#include <stdexcept>
#include <perception_base/exceptions.hpp>
#include <perception_base/audio/structs.hpp>

namespace perception
{

/**
 * @brief Write audio data to a WAV file using an output stream.
 *
 * @param out The output stream to write the WAV data to.
 * @param data The audio data to write.
 * @throws perception_exception if the output stream is not valid or if there are issues writing the data.
 */
void writeWavStream(std::ostream& out, const audio_data& data)
{
  if (!out)
    throw perception_exception("Output stream is not valid");

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
}

/**
 * @brief Write audio data to a WAV file.
 *
 * @param filename The name of the file to write.
 * @param data The audio data to write.
 *  @throws perception_exception if the file cannot be opened or if there are issues writing the data.
 */
void writeWavFile(const std::string& filename, const audio_data& data)
{
  std::ofstream out(filename, std::ios::binary);
  writeWavStream(out, data);
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
  audio.override = true;

  return audio;
}

/**
 * @brief Encode audio data to a WAV file and return as a byte vector.
 *
 * @param data The audio data to encode.
 * @return std::vector<char> Byte vector containing the WAV file data.
 * @throws perception_exception if there are issues writing the data.
 */
std::vector<char> encodeWavToBytes(const perception::audio_data& data)
{
  std::ostringstream oss(std::ios::binary);
  writeWavStream(oss, data);
  std::string str = oss.str();
  return std::vector<char>(str.begin(), str.end());
}

}  // namespace perception
