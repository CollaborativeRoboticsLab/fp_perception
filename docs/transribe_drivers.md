
# Transcription drivers

This page documents the transcription plugin in `fp_perception_driver_transcribe`.

## OpenAIDriver

Class: `fp_perception::OpenAIDriver` (REST-based audio transcription)

### What it does

- Accepts `transcription_request` via `transcribe()`.
- Encodes audio to WAV bytes and uploads it as multipart form data.
- Parses the JSON response and returns a `transcription_result`.

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

- The server offers `fp_perception_msgs/srv/PerceptionTranscribe`.
- If `use_device_audio=true`, the server transcribes its internal rolling microphone buffer.
- Otherwise, it transcribes the request-provided `PerceptionAudio`.
- If `use_diagnostics=true`, the driver publishes request health on `/diagnostics` via `diagnostic_updater`.

