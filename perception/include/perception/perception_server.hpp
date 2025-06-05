#pragma once

#include <thread>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <perception_base/driver_base.hpp>
#include <perception_base/algorithm_base.hpp>
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
    , transcription_driver_loader_("perception", "perception::DriverBase")
    , sentiment_driver_loader_("perception", "perception::DriverBase")
    , eye_gaze_algorithm_loader_("perception", "perception::AlgorithmBase")
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

    event_->info("PerceptionServer initializing...");

    /*************************************************************************
     * Identify what plugins to load
     ************************************************************************/
    this->declare_parameter("use_vision_driver", false);
    this->declare_parameter("use_microphone_driver", false);
    this->declare_parameter("use_speaker_driver", false);
    this->declare_parameter("use_eye_gaze_algorithm", false);
    this->declare_parameter("use_transcription_driver", false);
    this->declare_parameter("use_sentiment_driver", false);

    bool use_vision_driver = this->get_parameter("use_vision_driver").as_bool();
    bool use_microphone_driver = this->get_parameter("use_microphone_driver").as_bool();
    bool use_speaker_driver = this->get_parameter("use_speaker_driver").as_bool();
    bool use_eye_gaze_algorithm = this->get_parameter("use_eye_gaze_algorithm").as_bool();
    bool use_transcription_driver = this->get_parameter("use_transcription_driver").as_bool();
    bool use_sentiment_driver = this->get_parameter("use_sentiment_driver").as_bool();

    if (use_vision_driver) event_->info("Will use vision driver.");
    else event_->info("Will not use vision driver.");

    if (use_microphone_driver) event_->info("Will use microphone driver.");
    else event_->info("Will not use microphone driver.");

    if (use_speaker_driver) event_->info("Will use speaker driver.");
    else event_->info("Will not use speaker driver.");

    if (use_eye_gaze_algorithm) event_->info("Will use eye gaze algorithm.");
    else event_->info("Will not use eye gaze algorithm.");

    if (use_transcription_driver) event_->info("Will use transcription driver.");
    else event_->info("Will not use transcription driver.");

    if (use_sentiment_driver) event_->info("Will use sentiment driver.");
    else event_->info("Will not use sentiment driver.");

    /*************************************************************************
     * vision driver plugin class loader and driver pointer
     ************************************************************************/

    if (use_vision_driver)
    {
      this->declare_parameter("vision_driver", "perception::DefaultDriver");
      std::string vision_driver_name = this->get_parameter("vision_driver").as_string();

      event_->info("Loading vision driver plugin: " + vision_driver_name);

      vision_driver_ = vision_driver_loader_.createSharedInstance(vision_driver_name);
      vision_driver_->initialize(shared_from_this());
      vision_driver_->start();
      vision_driver_->test();

      event_->info("Started vision driver plugin: " + vision_driver_name);
    }
    else
    {
      event_->info("Vision driver plugin not loaded.");
    }

    /*************************************************************************
     * microphone driver plugin class loader and driver pointer
     ************************************************************************/

    if (use_microphone_driver)
    {
      this->declare_parameter("microphone_driver", "perception::MicrophoneAudioDriver");
      std::string mic_driver_name = this->get_parameter("microphone_driver").as_string();

      event_->info("Loading microphone driver plugin: " + mic_driver_name);

      microphone_driver_ = listener_driver_loader_.createSharedInstance(mic_driver_name);
      microphone_driver_->initialize(shared_from_this());
      microphone_driver_->start();

      event_->info("Started microphone driver plugin: " + mic_driver_name);
    }
    else
    {
      event_->info("Microphone driver plugin not loaded.");
    }

    /*************************************************************************
     * speaker driver plugin class loader and driver pointer
     ************************************************************************/

    if (use_speaker_driver)
    {
      this->declare_parameter("speaker_driver", "perception::SpeakerAudioDriver");
      std::string speaker_driver_name = this->get_parameter("speaker_driver").as_string();

      event_->info("Loading speaker driver plugin: " + speaker_driver_name);

      speaker_driver_ = speaker_driver_loader_.createSharedInstance(speaker_driver_name);
      speaker_driver_->initialize(shared_from_this());
      speaker_driver_->start();

      event_->info("Started speaker driver plugin: " + speaker_driver_name);
    }
    else
    {
      event_->info("Speaker driver plugin not loaded.");
    }

    /*************************************************************************
     * transcription driver plugin class loader and driver pointer
     ************************************************************************/
    if (use_transcription_driver)
    {
      this->declare_parameter("transcription_driver", "perception::PromptToolsTranscribeDriver");
      std::string transcription_driver_name = this->get_parameter("transcription_driver").as_string();

      event_->info("Loading transcription driver plugin: " + transcription_driver_name);

      transcription_driver_ = transcription_driver_loader_.createSharedInstance(transcription_driver_name);
      transcription_driver_->initialize(shared_from_this());
      transcription_driver_->start();

      event_->info("Started transcription driver plugin: " + transcription_driver_name);
    }
    else
    {
      event_->info("Transcription driver plugin not loaded.");
    }

    /*************************************************************************
     * sentiment driver plugin class loader and driver pointer
     ************************************************************************/
    if (use_sentiment_driver)
    {
      this->declare_parameter("sentiment_driver", "perception::PromptToolsSentimentDriver");
      std::string sentiment_driver_name = this->get_parameter("sentiment_driver").as_string();

      event_->info("Loading sentiment driver plugin: " + sentiment_driver_name);

      sentiment_driver_ = sentiment_driver_loader_.createSharedInstance(sentiment_driver_name);
      sentiment_driver_->initialize(shared_from_this());
      sentiment_driver_->start();

      event_->info("Started sentiment driver plugin: " + sentiment_driver_name);
    }
    else
    {
      event_->info("Sentiment driver plugin not loaded.");
    }

    /*************************************************************************
     * eye gaze algorithm plugin class loader and driver pointer
     ************************************************************************/

    if (use_eye_gaze_algorithm)
    {
      this->declare_parameter("eye_gaze_algorithm", "perception::GazeAlgorithm");
      std::string eye_gaze_algorithm_name = this->get_parameter("eye_gaze_algorithm").as_string();

      event_->info("Loading eye gaze algorithm plugin: " + eye_gaze_algorithm_name);

      eye_gaze_algorithm_ = eye_gaze_algorithm_loader_.createSharedInstance(eye_gaze_algorithm_name);
      eye_gaze_algorithm_->initialize(shared_from_this());
      eye_gaze_algorithm_->set_vision_driver(vision_driver_);
      eye_gaze_algorithm_->start();

      event_->info("Started eye gaze algorithm plugin: " + eye_gaze_algorithm_name);
    }
    else
    {
      event_->info("Eye gaze algorithm plugin not loaded.");
    }
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

  /** plugin loader for transcription driver */
  pluginlib::ClassLoader<perception::DriverBase> transcription_driver_loader_;

  /** shared pointer for transcription driver */
  std::shared_ptr<perception::DriverBase> transcription_driver_;

  /** plugin loader for sentiment driver */
  pluginlib::ClassLoader<perception::DriverBase> sentiment_driver_loader_;

  /** shared pointer for sentiment driver */
  std::shared_ptr<perception::DriverBase> sentiment_driver_;

  /** plugin loader for eye gaze algorithm */
  pluginlib::ClassLoader<perception::AlgorithmBase> eye_gaze_algorithm_loader_;

  /** shared pointer for eye gaze algorithm */
  std::shared_ptr<perception::AlgorithmBase> eye_gaze_algorithm_;
};

}  // namespace perception