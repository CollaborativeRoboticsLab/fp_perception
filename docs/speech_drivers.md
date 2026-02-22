
# Speech drivers

This page documents the speech synthesis plugin in `perception_driver_speech`.

## OpenAISpeechDriver

Class: `perception::OpenAISpeechDriver` (REST-based TTS)

### What it does

- Accepts `perception::text_data` via `setDataStream()`.
- Uses `RestBase::call_tts()` to request PCM audio from the configured REST endpoint.
- Exposes the synthesized audio via `getData()` as `perception::audio_data`.

### Parameters

- `driver.speech.OpenAISpeechDriver.name` (string)
- `driver.speech.OpenAISpeechDriver.model` (string)
- `driver.speech.OpenAISpeechDriver.test_text` (string)
- `driver.speech.OpenAISpeechDriver.test_file_path` (string)
- `driver.speech.OpenAISpeechDriver.voice` (string)
- `driver.speech.OpenAISpeechDriver.instructions` (string)

REST base parameters:

- `driver.speech.OpenAISpeechDriver.rest.uri`
- `driver.speech.OpenAISpeechDriver.rest.method`
- `driver.speech.OpenAISpeechDriver.rest.ssl_verify`
- `driver.speech.OpenAISpeechDriver.rest.auth_type`

Environment:

- Requires `OPENAI_API_KEY` to be set.

### Usage with the server

- The server offers `perception_msgs/srv/PerceptionSpeech`.
- If `use_device_audio=true`, the server routes synthesized audio to the speaker driver.
- Otherwise, it returns the audio in the service response.

