
# Transcription drivers

This page documents the transcription plugin in `perception_driver_transcribe`.

## OpenAIDriver

Class: `perception::OpenAIDriver` (REST-based audio transcription)

### What it does

- Accepts `perception::audio_data` via `setDataStream()`.
- Encodes audio to WAV bytes and uploads it as multipart form data.
- Parses the JSON response and exposes the transcribed text via `getData()`.

### Parameters

- `driver.transcription.OpenAIDriver.name` (string)
- `driver.transcription.OpenAIDriver.model` (string, default `whisper-1`)
- `driver.transcription.OpenAIDriver.test_file_path` (string)

REST base parameters (declared by `RestBase`):

- `driver.transcription.OpenAIDriver.rest.uri`
- `driver.transcription.OpenAIDriver.rest.method`
- `driver.transcription.OpenAIDriver.rest.ssl_verify`
- `driver.transcription.OpenAIDriver.rest.auth_type`

Environment:

- Requires `OPENAI_API_KEY` to be set.

### Usage with the server

- The server offers `perception_msgs/srv/PerceptionTranscribe`.
- If `use_device_audio=true`, the server transcribes its internal rolling microphone buffer.
- Otherwise, it transcribes the request-provided `PerceptionAudio`.

