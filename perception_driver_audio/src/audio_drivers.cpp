#include <pluginlib/class_list_macros.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_driver_audio/microphone_audio_driver.hpp>
#include <perception_driver_audio/speaker_audio_driver.hpp>

PLUGINLIB_EXPORT_CLASS(perception::MicrophoneAudioDriver, perception::DriverBase);
PLUGINLIB_EXPORT_CLASS(perception::SpeakerAudioDriver, perception::DriverBase);