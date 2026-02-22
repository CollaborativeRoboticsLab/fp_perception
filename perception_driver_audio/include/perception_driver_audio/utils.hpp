#pragma once

#include <portaudio.h>
#include <string>
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
int getDeviceIdByName(const std::string& target_name)
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
