#pragma once

#include <thread>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_events/event_client.hpp>

namespace perception
{

class PerceptionServer : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Perception Server object
   *
   * @param options Node options for the server
   */
  PerceptionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("perception_server", options)
    , vision_driver_loader_("perception", "perception::DriverBase")
    , listener_driver_loader_("perception", "perception::DriverBase")
    , speaker_driver_loader_("perception", "perception::DriverBase")
  {
    try
    {
      if (shared_from_this())
      {
        initialize();
      }
    }
    catch (const std::bad_weak_ptr&)
    {
      // Not yet safe — probably standalone without make_shared
    }
  }

  ~PerceptionServer()
  {
  }
  
  void initialize()
  {
    event_ = std::make_shared<EventClient>(shared_from_this(), "perception_server", "/events");

    /*************************************************************************
     * vision driver plugin class loader and driver pointer
     ************************************************************************/

    std::string vision_driver_name = this->declare_parameter("vision_driver", "perception::DefaultVisionDriver");
    event_->info("Loading vision driver plugin: " + vision_driver_name);

    vision_driver_ = vision_driver_loader_.createSharedInstance(vision_driver_name);
    vision_driver_->initialize(shared_from_this());

    /*************************************************************************
     * microphone driver plugin class loader and driver pointer
     ************************************************************************/

    std::string mic_driver_name = this->declare_parameter("microphone_driver", "perception::MicrophoneAudioDriver");
    event_->info("Loading microphone driver plugin: " + mic_driver_name);

    microphone_driver_ = listener_driver_loader_.createSharedInstance(mic_driver_name);
    microphone_driver_->initialize(shared_from_this());

    // Load speaker driver
    /*************************************************************************
     * vision driver plugin class loader and driver pointer
     ************************************************************************/

    std::string speaker_driver_name = this->declare_parameter("speaker_driver", "perception::SpeakerAudioDriver");
    event_->info("Loading speaker driver plugin: " + speaker_driver_name);

    speaker_driver_ = speaker_driver_loader_.createSharedInstance(speaker_driver_name);
    speaker_driver_->initialize(shared_from_this());

    event_->info("PerceptionServer initialized.");
  }


protected:
  /** client for publishing events */
  std::shared_ptr<EventClient> event_;

  /** plugin loader for vision driver */
  pluginlib::ClassLoader<perception::DriverBase> vision_driver_loader_;

  /** shared pointer for vision driver */
  std::shared_ptr<perception::DriverBase> vision_driver_;

  /** plugin loader for audio listener driver */
  pluginlib::ClassLoader<perception::DriverBase> listener_driver_loader_;

  /** shared pointer for audio listener driver */
  std::shared_ptr<perception::DriverBase> microphone_driver_;

  /** plugin loader for audio speaker driver */
  pluginlib::ClassLoader<perception::DriverBase> speaker_driver_loader_;

  /** shared pointer for audio speaker driver */
  std::shared_ptr<perception::DriverBase> speaker_driver_;
};

}  // namespace perception