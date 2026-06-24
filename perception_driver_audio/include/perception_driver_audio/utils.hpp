#pragma once

#include <portaudio.h>
#include <string>
#include <sstream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <perception_base/exceptions.hpp>

namespace perception
{
/**
 * @brief Get the device ID by name.
 *
 * This function searches for a PortAudio device by its name and returns its ID.
 * If the device is not found, it throws an exception.
 *
 * @param target_name The name of the target device to search for.
 * @return int The ID of the found device.
 * @throws perception::perception_exception if no devices are found or if the target device is not found.
 */
inline std::string getHostApiName(PaHostApiIndex host_api_index)
{
  const PaHostApiInfo* host_api_info = Pa_GetHostApiInfo(host_api_index);
  return host_api_info && host_api_info->name ? host_api_info->name : "unknown";
}

inline std::string describePortAudioDevice(int device_id)
{
  const PaDeviceInfo* device_info = Pa_GetDeviceInfo(device_id);
  if (!device_info)
    return "PortAudio device " + std::to_string(device_id) + ": <null>";

  std::ostringstream out;
  out << "PortAudio device " << device_id << " ['" << (device_info->name ? device_info->name : "unknown") << "']"
      << " host_api='" << getHostApiName(device_info->hostApi) << "'"
      << " max_input=" << device_info->maxInputChannels << " max_output=" << device_info->maxOutputChannels
      << " default_sample_rate=" << static_cast<int>(std::lround(device_info->defaultSampleRate))
      << " low_input_latency=" << device_info->defaultLowInputLatency
      << " low_output_latency=" << device_info->defaultLowOutputLatency;

  return out.str();
}

template <typename LoggerT>
inline void logPortAudioDevices(const LoggerT& logger)
{
  const int num_devices = Pa_GetDeviceCount();
  if (num_devices < 0)
  {
    RCLCPP_WARN(logger, "PortAudio device enumeration failed: %s", Pa_GetErrorText(num_devices));
    return;
  }

  RCLCPP_INFO(logger, "PortAudio device count: %d", num_devices);
  for (int i = 0; i < num_devices; ++i)
  {
    const auto description = describePortAudioDevice(i);
    RCLCPP_INFO(logger, "%s", description.c_str());
  }
}

inline int getDeviceIdByName(const std::string& target_name)
{
  int num_devices = Pa_GetDeviceCount();
  if (num_devices < 0)
    throw perception::perception_exception("No PortAudio devices found");

  for (int i = 0; i < num_devices; ++i)
  {
    const PaDeviceInfo* device_info = Pa_GetDeviceInfo(i);
    if (device_info && std::string(device_info->name).find(target_name) != std::string::npos)
    {
      return i;
    }
  }

  throw perception::perception_exception("Device with name \"" + target_name + "\" not found");
}
}  // namespace perception
