# Plugin configuration

The fp_perception launch files load parameters from `src/fp_perception/fp_perception/config/config.yaml`, which is installed to `install/fp_perception/share/fp_perception/config/config.yaml` during the build.

At a high level:

1. Enable the feature group with `use_*`.
2. Pick the plugin class to load, for example `microphone_driver: fp_perception::MicrophoneAudioDriver`.
3. Configure the ROS interfaces under `interface.*`.
4. Configure per-plugin parameters under `driver.*`.

## Current config shape

The checked-in config currently looks like this:

```yaml
/**:
  ros__parameters:
    run_tests: false

    # Which plugin groups to load
    use_ros_vision_driver: true
    use_non_ros_vision_driver: false
    use_microphone_driver: true
    use_speaker_driver: true
    use_transcription_driver: true
    use_sentiment_driver: true
    use_speech_driver: true
    use_image_analysis_driver: true

    use_diagnostics: true

    # Pluginlib class selection
    ros_vision_driver: fp_perception::DefaultDriver
    non_ros_vision_driver: fp_perception::OpenCVDriver
    microphone_driver: fp_perception::MicrophoneAudioDriver
    speaker_driver: fp_perception::SpeakerAudioDriver
    transcription_driver: fp_perception::OpenAIDriver
    sentiment_driver: fp_perception::SentimentDriver
    speech_synthesis_driver: fp_perception::OpenAISpeechDriver
    image_analysis_driver: fp_perception::OpenAIImageAnalysisDriver

    interface:
      audio_input:
        publish: true
        topic: perception/microphone
        frame_id: microphone_frame
        frequency: 10
        audio_retention_window: 100
        default_audio_request_window: 10

      audio_output:
        subscribe: true
        topic: perception/speaker

      transcription:
        provide_service: true
        service: perception/transcription

      speech:
        provide_service: true
        service_name: perception/speech

      sentiment:
        provide_service: true
        service_name: perception/sentiment_analysis

      vision_input:
        publish: true
        topic: perception/camera
        frame_id: camera_frame
        frequency: 10

      image_analysis:
        provide_service: true
        service_name: perception/image_analysis

    driver:
      vision:
        DefaultDriver:
          name: DefaultDriver
          topic: /camera/camera/color/image_raw

        OpenCVDriver:
          name: OpenCVDriver
          device_id: 0

      audio:
        MicrophoneAudioDriver:
          name: MicrophoneAudioDriver
          device_name: "ALC285 Analog"
          chunk_size: 1920
          sample_rate: 48000
          channels: 1
          capture_buffer_window: 10

        SpeakerAudioDriver:
          name: SpeakerAudioDriver
          device_name: "ALC285 Analog"
          sample_rate: 48000
          channels: 2
          test_file_path: install/fp_perception_driver_audio/share/fp_perception_driver_audio/audio/hello-1.wav

      transcription:
        OpenAIDriver:
          name: OpenAIDriver
          model: whisper-1
          test_file_path: install/fp_perception_driver_audio/share/fp_perception_driver_audio/audio/hello-1.wav
          rest:
            uri: https://api.openai.com/v1/audio/transcriptions
            method: POST
            auth_type: Bearer
            ssl_verify: true

      sentiment:
        SentimentDriver:
          name: SentimentDriver
          rest:
            uri: https://router.huggingface.co/hf-inference/models/distilbert/distilbert-base-uncased-finetuned-sst-2-english
            method: POST
            auth_type: Bearer
            ssl_verify: true

      speech:
        OpenAISpeechDriver:
          name: OpenAISpeechDriver
          model: gpt-4o-mini-tts
          test_text: "Hello this is a test speech for ROS2 perception speech driver."
          test_file_path: test/speech.wav
          voice: coral
          instructions: "Please speak clearly and slowly."
          chunking_enabled: true
          chunk_max_words: 100
          chunk_parallel_requests: 3
          rest:
            uri: https://api.openai.com/v1/audio/speech
            method: POST
            auth_type: Bearer
            ssl_verify: true

      image_analysis:
        OpenAIImageAnalysisDriver:
          name: OpenAIImageAnalysisDriver
          model: gpt-5-mini
          test_file_path: install/fp_perception_driver_image_analysis/share/fp_perception_driver_image_analysis/image/image.png
          test_prompt: "Does this image contain a cat?"
          detail: low
          rest:
            uri: https://api.openai.com/v1/responses
            method: POST
            auth_type: Bearer
            ssl_verify: true
```

## Notes

- Audio topics and services are configured separately. `interface.audio_input.*` controls microphone publishing, while `interface.audio_output.*` controls speaker subscription.
- The image-analysis plugin namespace is `driver.image_analysis.OpenAIImageAnalysisDriver.*`.
- The speech plugin namespace is `driver.speech.OpenAISpeechDriver.*`.
- `run_tests` is currently `false` by default. Set it to `true` when you want plugin self-tests during startup.

After editing the config, rebuild and re-source so the installed copy is refreshed:

```sh
colcon build --packages-select fp_perception
source install/setup.bash
```