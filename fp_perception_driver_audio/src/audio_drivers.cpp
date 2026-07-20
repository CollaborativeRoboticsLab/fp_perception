#include <pluginlib/class_list_macros.hpp>
#include <fp_perception_base/driver_base.hpp>
#include <fp_perception_driver_audio/microphone_audio_driver.hpp>
#include <fp_perception_driver_audio/speaker_audio_driver.hpp>

PLUGINLIB_EXPORT_CLASS(fp_perception::MicrophoneAudioDriver, fp_perception::DriverBase);
PLUGINLIB_EXPORT_CLASS(fp_perception::SpeakerAudioDriver, fp_perception::DriverBase);