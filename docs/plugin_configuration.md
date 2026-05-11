# Plugin configuration

The perception server reads parameters from [perception/config/config.yaml](perception/perception/config/config.yaml).

At a high level:

1. Enable the feature group with `use_*`.
2. Pick the plugin class to load (e.g. `microphone_driver: perception::MicrophoneAudioDriver`).
3. Configure the ROS interfaces under `interface.*`.
4. Configure per-plugin parameters under `driver.*`.

```yaml
  # Enable/disable plugin groups
  use_microphone_driver: true
  use_speaker_driver: true
  use_transcription_driver: true
  use_speech_driver: true
  use_sentiment_driver: false
  use_vision_driver: false
  use_image_analysis_driver: false

  # Select which pluginlib classes to load
  microphone_driver: perception::MicrophoneAudioDriver
  speaker_driver: perception::SpeakerAudioDriver
  transcription_driver: perception::OpenAIDriver
  speech_synthesis_driver: perception::OpenAISpeechDriver
  sentiment_driver: perception::SentimentDriver
  image_analysis_driver: perception::OpenAIImageAnalysisDriver

  # Server ROS interfaces
  interface:
    audio_input:
      publish: true
      topic: perception/microphone
      frame_id: microphone_frame
      frequency: 10
    transcription:
      provide_service: true
      service: perception/transcription
      buffer_duration: 10
    speech:
      provide_service: true
      service_name: perception/speech
    sentiment:
      provide_service: false
      service_name: perception/sentiment_analysis

    image_analysis:
      provide_service: false
      service_name: perception/image_analysis

  # Per-plugin parameters (maps to dot-separated params)
  driver:
    audio:
      MicrophoneAudioDriver:
        # PortAudio device index (set via find_devices.py)
        device_id: 0
        sample_rate: 48000
        channels: 2
        chunk_size: 48000
        buffer_time: 10
    transcription:
      OpenAIDriver:
        model: whisper-1
        rest:
          uri: https://api.openai.com/v1/audio/transcriptions

    image_analysis:
      OpenAIDriver:
        model: gpt-4.1
        detail: auto
        rest:
          uri: https://api.openai.com/v1/responses
```

build the workspace to update the configuration
```sh
colcon build
```