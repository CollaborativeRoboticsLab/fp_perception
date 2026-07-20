
# Speech drivers

This page documents the speech synthesis plugin in `fp_perception_driver_speech`.

## OpenAISpeechDriver

Class: `fp_perception::OpenAISpeechDriver` (REST-based TTS)

### What it does

- Accepts `fp_perception::text_data` via `synthesize()`.
- Uses `RestBase::call_tts()` to request PCM audio from the configured REST endpoint.
- Splits long text into smaller word-bounded chunks, synthesizes those chunks with bounded parallelism, then concatenates the PCM so playback stays in the original text order.
- Returns the synthesized audio as `fp_perception::audio_data`.

### Parameters

- `driver.speech.OpenAISpeechDriver.name` (string)
- `driver.speech.OpenAISpeechDriver.model` (string)
- `driver.speech.OpenAISpeechDriver.test_text` (string)
- `driver.speech.OpenAISpeechDriver.test_file_path` (string)
- `driver.speech.OpenAISpeechDriver.voice` (string)
- `driver.speech.OpenAISpeechDriver.instructions` (string)
- `driver.speech.OpenAISpeechDriver.chunking_enabled` (bool): enable long-text chunking before TTS requests.
- `driver.speech.OpenAISpeechDriver.chunk_max_words` (int): maximum number of words sent in a single TTS request.
- `driver.speech.OpenAISpeechDriver.chunk_parallel_requests` (int): maximum number of chunk requests to synthesize concurrently.

REST base parameters:

- `driver.speech.OpenAISpeechDriver.rest.uri`
- `driver.speech.OpenAISpeechDriver.rest.method`
- `driver.speech.OpenAISpeechDriver.rest.ssl_verify`
- `driver.speech.OpenAISpeechDriver.rest.auth_type`

Environment:

- Requires `OPENAI_API_KEY` to be set.

### Usage with the server

- The server offers `fp_perception_msgs/srv/PerceptionSpeech`.
- If `use_device_audio=true`, the server routes synthesized audio to the speaker driver.
- Otherwise, it returns the audio in the service response.
- If `use_diagnostics=true`, the driver publishes request health on `/diagnostics` via `diagnostic_updater`.

