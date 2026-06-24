# Perception Server Refactor Plan

## Goals

- Reduce the size and responsibility of `PerceptionServer`.
- Move modality-specific logic behind plugin-facing typed interfaces.
- Replace ad hoc `std::any` payload conventions with shared structs in `perception_base`.
- Keep ROS publishers, subscribers, and services at the node boundary.
- Make future drivers easier to add without repeating boilerplate.
- Support real-time robot audio: always-on microphone capture, request-time extraction of timestamped audio windows, low-latency speaker output, streaming TTS playback, and simultaneous microphone/speaker operation.

## Current Problems

### 1. `PerceptionServer` owns too many responsibilities

Today the server is responsible for:

- Declaring and reading all parameters.
- Loading and initializing every plugin.
- Managing ROS publishers, subscribers, and services.
- Running background audio and video loops.
- Holding the device-audio rolling buffer.
- Translating between ROS messages and driver-specific payloads.
- Coordinating cross-plugin workflows such as:
	- microphone -> transcription
	- microphone -> transcription -> sentiment
	- text -> speech -> speaker
	- vision driver -> image analysis

This makes the server hard to test, hard to extend, and easy to break.

### 2. Plugin contracts were too generic

Before the typed-interface refactor, `DriverBase` exposed `getData`, `setData`, `getDataStream`, and `setDataStream` using `std::any`.

That forces the server to know hidden conventions such as:

- transcription input is `audio_data`
- speech input is `text_data`
- sentiment input is `std::string`
- sentiment output is `std::pair<std::string, double>`
- image analysis input is `std::pair<cv::Mat, std::string>`

This is the main reason server code has become messy.

### 3. Shared data shapes are inconsistent

`audio_data` and `text_data` already exist and work well as shared boundary types.

Other workflows still use plugin-specific payloads instead of shared structs, especially:

- sentiment request/response
- image analysis request/response
- vision frame transport
- transcription request/response

### 4. The device-audio buffer is in the wrong place

The rolling microphone buffer, waiting logic, and duration slicing live in the server even though they are really part of audio-source behavior.

The current buffer is also sample-count based only. For request-driven STT, the system needs a timestamped ring buffer so a request can ask for the audio segment corresponding to a time interval such as:

```text
[request.header.stamp, request.header.stamp + request.duration]
```

or, depending on service semantics, the latest `duration` ending at a request/header timestamp. This avoids guessing based only on "latest N seconds" and makes audio extraction deterministic when service calls arrive after the user finished speaking.

### 5. PortAudio drivers originally used blocking I/O

The original implementation captured with `Pa_ReadStream()` and played with `Pa_WriteStream()`.

That blocking API was acceptable for simple record/play tests, but it was not the preferred architecture for an interactive robot that needs:

- continuous listening,
- no gaps between output chunks,
- streaming TTS,
- low-latency turn-taking,
- and simultaneous microphone/speaker timing.

The current audio backend now uses PortAudio callback streams with bounded queues/ring-buffer style buffering between the real-time audio callback and ROS/workflow threads.

## Refactor Principles

- Keep ROS transport concerns in `PerceptionServer`.
- Move modality-specific request building and normalization into plugins or dedicated workflow classes.
- Prefer typed interfaces over `std::any` for all new work.
- Refactor in small stages so the system remains buildable and testable.
- Preserve existing topics, service names, and public behavior unless there is a deliberate API change.

## Proposed Target Architecture

### 1. Thin node, typed workflows

`PerceptionServer` should become a thin composition root responsible for:

- parameters
- plugin registration/loading
- ROS service and topic wiring
- delegating work to typed workflow objects

### 2. Shared structs in `perception_base`

Add shared structs for every cross-plugin payload. Suggested additions:

- `transcription_request`
- `transcription_result`
- `sentiment_request`
- `sentiment_result`
- `image_analysis_request`
- `image_analysis_result`
- `vision_frame`

Suggested fields:

#### `transcription_request`

- `audio_data audio`
- `bool use_device_audio`
- `int device_buffer_time`

#### `transcription_result`

- `std::string text`
- `bool success`
- `std::string error`

#### `sentiment_request`

- `std::string text`
- `bool use_device_audio`
- `int device_buffer_time`

#### `sentiment_result`

- `std::string label`
- `double score`
- `bool success`
- `std::string error`

#### `vision_frame`

- `cv::Mat image`
- `std::string frame_id`
- `rclcpp::Time stamp`

#### `image_analysis_request`

- `vision_frame frame`
- `std::string prompt`
- `bool use_device_vision`

#### `image_analysis_result`

- `std::string response`
- `bool success`
- `std::string error`

### 3. Role-specific driver interfaces

Keep `DriverBase` only for lifecycle and common utilities, then add typed role interfaces such as:

- `AudioSourceDriver`
- `AudioSinkDriver`
- `TranscriptionDriver`
- `SpeechSynthesisDriver`
- `SentimentAnalysisDriver`
- `VisionSourceDriver`
- `ImageAnalysisDriver`

Examples:

```cpp
class AudioSourceDriver : public DriverBase
{
public:
	virtual audio_data readChunk() = 0;
	virtual audio_data readBufferedAudio(int duration_seconds) = 0;
};

class TranscriptionDriver : public DriverBase
{
public:
	virtual transcription_result transcribe(const transcription_request& request) = 0;
};
```

This removes hidden `std::any_cast` rules from the server.

### 4. Workflow or pipeline classes

Move service-specific orchestration into focused classes:

- `TranscriptionPipeline`
- `SpeechPipeline`
- `SentimentPipeline`
- `ImageAnalysisPipeline`

These classes should:

- accept typed request structs
- coordinate one or more drivers
- return typed result structs
- contain non-ROS business logic currently living in callbacks

That keeps plugins focused on provider/device behavior and keeps the server focused on ROS.

### 5. Audio buffering extracted from the server

Two valid options:

#### Option A. Move buffering into the microphone plugin

The microphone driver owns:

- rolling sample buffer
- wait logic
- duration slicing
- sample format continuity

The server then calls a typed method such as `readBufferedAudio(duration_seconds)`.

#### Option B. Extract a reusable `AudioBuffer` component

Create a small class in `perception_base` or `perception` that owns:

- append
- trim
- wait until initialized
- fetch latest N seconds

This is lower risk if multiple audio sources may need the same behavior.

Recommendation: start with Option B unless the microphone plugin is guaranteed to remain the only device-audio source.

#### Timestamped audio ring buffer requirement

Whichever option is chosen, buffered audio should be stored with timing metadata, not only as a flat vector of samples.

Recommended behavior:

- Maintain a monotonic mapping from sample index to capture time.
- Store either per-block timestamps or enough metadata to derive timestamps from the first sample time, sample rate, channel count, and total captured frame index.
- Expose APIs such as:

```cpp
audio_data readLatest(std::chrono::milliseconds duration);
audio_data readWindow(const rclcpp::Time& start, const rclcpp::Duration& duration);
audio_data readWindow(const rclcpp::Time& start, const rclcpp::Time& end);
```

- Prefer frame counts internally. Convert to sample counts only when indexing interleaved sample vectors.
- Include the selected window start timestamp in the returned audio metadata once the shared `audio_data` type supports it, or return an audio-window wrapper containing `audio_data` plus timestamps.
- Handle missing/partial windows explicitly by either returning the available overlap with a warning flag or returning a structured error.

This enables STT requests to extract the relevant buffered audio using `(header time, header time + duration)` instead of always transcribing the most recent N seconds.

### 6. Real-time audio backend target

For the robot use case, the audio plugin evolved from blocking `Pa_ReadStream()` / `Pa_WriteStream()` calls to callback-driven streams.

Recommended target design:

- Open a microphone input stream with a PortAudio input callback.
- Open a speaker output stream with a PortAudio output callback.
- Consider one full-duplex PortAudio stream if the selected device/API supports it and tight input/output timing is required.
- The input callback appends captured frames into a timestamped ring buffer and optionally publishes/forwards chunks to ROS from a non-real-time thread.
- The output callback pulls frames from a playback queue/ring buffer filled by TTS streaming or complete synthesized audio.
- The callbacks must not call ROS logging, allocate memory, perform network I/O, do STT/TTS work, or block on long mutex waits.
- ROS/service/pipeline threads interact with the audio backend through typed methods such as `enqueuePlayback()`, `readWindow()`, and `readLatest()`.

The production robot audio path is now callback + ring buffer based; further work is about runtime hardening rather than blocking-I/O removal.

### 6. Plugin loading helper

Extract repetitive plugin-loading code into a helper or manager, for example:

- `DriverRegistry`
- `DriverManager`
- `PluginFactory`

Responsibilities:

- declare driver plugin parameter names
- load plugin by name
- initialize plugin
- hold driver instances
- optionally run plugin tests

This will remove a large amount of repeated initialization logic from `PerceptionServer`.

## Recommended Phases

## Phase 1. Introduce shared structs

### Scope

- Add new request/response structs to `perception_base`.
- Keep existing ROS messages and services unchanged.
- Do not change plugin loading yet.

### Changes

- Extend the shared structs area, or create a small set of new headers under `perception_base/include/perception_base/`.
- Replace raw payload types in server internals with typed structs where possible.

### Outcome

- Clear shared language for data crossing boundaries.
- Smaller, safer callback code.

## Phase 2. Extract audio buffering

### Scope

- Move `public_buffer_`, `wait_for_public_audio()`, append/trim logic, and associated synchronization out of `PerceptionServer`.

### Changes

- Create `AudioBuffer` or equivalent.
- Have `publishAudio()` append chunks through that abstraction.
- Use that abstraction from transcription and sentiment flows.

### Outcome

- The biggest concentration of low-level logic leaves the server.
- Audio requests can be served from a timestamped ring buffer rather than from an ambiguous latest-N-seconds sample vector.

## Phase 2b. Introduce real-time audio backend

### Scope

- Add callback-based audio capture/playback for production robot use.
- Keep the existing blocking implementation only as a fallback or debugging path during migration.

### Changes

- Add a timestamped input ring buffer owned by the audio source driver or shared audio component.
- Add a playback queue/ring buffer owned by the audio sink driver.
- Replace microphone capture based on repeated `Pa_ReadStream()` with a PortAudio input callback.
- Replace speaker playback based on direct `Pa_WriteStream()` calls with a PortAudio output callback for streaming playback.
- Add typed methods for `readWindow(start, duration)`, `readLatest(duration)`, `enqueuePlayback(audio_data)`, `stopPlayback()`, and queue drain/underrun status.
- Ensure callbacks avoid blocking, dynamic allocation, ROS logging, network calls, and other non-real-time-safe work.

### Outcome

- Always-on listening, streaming TTS, low-latency interaction, and simultaneous input/output timing are supported by design.

## Phase 3. Add typed driver interfaces

### Scope

- Introduce role-specific abstract classes.
- Make existing plugins implement those interfaces.
- Keep `DriverBase` only as the pluginlib/lifecycle base during migration.

### Changes

- Transcription driver returns `transcription_result`.
- Speech driver accepts `text_data` and returns `audio_data` through typed API.
- Sentiment driver accepts `sentiment_request` and returns `sentiment_result`.
- Image analysis driver accepts `image_analysis_request` and returns `image_analysis_result`.

### Outcome

- `std::any_cast` moves out of the server or disappears entirely.

## Phase 4. Extract workflow classes

### Scope

- Move service callback business logic into pipeline/orchestrator classes.

### Changes

- `transcribe_callback()` becomes request mapping + one pipeline call.
- `sentiment_callback()` becomes request mapping + one pipeline call.
- `speech_callback()` becomes request mapping + one pipeline call.
- `image_analysis_callback()` becomes request mapping + one pipeline call.

### Outcome

- Server callbacks become short, testable adapters.

## Phase 5. Simplify plugin loading and startup

### Scope

- Remove repeated load/initialize/test blocks from `initialize()`.

### Changes

- Centralize plugin creation and registration.
- Group parameter declaration and driver activation cleanly.
- Consider small config structs for interface settings and driver settings.

### Outcome

- `initialize()` becomes readable and maintainable.

## Specific Code Moves

### Move out of `PerceptionServer`

- audio rolling buffer state and synchronization
- duration-based audio slicing
- timestamp-to-sample-window extraction
- plugin-specific `std::any_cast` logic
- sentiment transcription chaining logic
- image-analysis request assembly
- repeated plugin loading blocks

### Keep in `PerceptionServer`

- ROS parameter declaration and reading, or a thin config wrapper around it
- ROS publisher/subscriber/service creation
- conversion between ROS service/message types and internal typed structs
- node startup and shutdown coordination

### Move into plugins

- provider-specific request construction
- provider-specific response parsing
- device-specific capture/playback details
- PortAudio callback stream ownership and real-time-safe audio queues
- provider defaults such as models, voices, detail settings, and file encoding strategy

### Move into workflow classes

- multi-driver orchestration
- fallback and validation logic for `use_device_audio` and `use_device_vision`
- request timestamp + duration mapping to audio ring-buffer windows
- shared error mapping
- typed result assembly

## Low-Risk First Steps

If the goal is to improve structure without destabilizing the system, start here:

1. Add typed structs for sentiment and image analysis.
2. Replace `std::pair<cv::Mat, std::string>` with `image_analysis_request`.
3. Replace raw sentiment string input/output with typed structs.
4. Extract the audio buffer into an `AudioBuffer` helper.
5. Extract callback bodies into private helper classes or functions before changing plugin inheritance.

This sequence reduces complexity early while preserving behavior.

## Higher-Impact Follow-Up Changes

After the low-risk cleanup is in place:

1. Introduce typed role interfaces.
2. Update all concrete plugins to implement them.
3. Replace plugin instance storage from `std::shared_ptr<DriverBase>` to typed pointers where appropriate.
4. Remove remaining `std::any` usage from server workflows and then delete the compatibility hooks.

## Risks and Mitigations

### Risk: plugin migration becomes too wide

Mitigation:

- keep `DriverBase` during transition
- migrate one modality at a time
- preserve existing plugin export classes

### Risk: ROS service behavior changes during refactor

Mitigation:

- keep service definitions unchanged initially
- only change internal orchestration first
- add behavior checks around existing service responses

### Risk: audio buffering regressions

Mitigation:

- refactor buffer logic behind tests before moving ownership again
- preserve current behavior for latest-N-seconds semantics

## Suggested File-Level Work Breakdown

### `perception_base`

- add new typed struct headers
- optionally add `AudioBuffer`
- add role-specific driver interface headers

### `perception`

- slim `PerceptionServer`
- add workflow classes
- add driver/plugin manager helper

### `perception_driver_audio`

- optionally absorb or cooperate with the extracted audio buffer logic
- expose typed read methods through audio source/sink interfaces

### `perception_driver_transcribe`

- expose typed `transcribe()` API

### `perception_driver_sentiment`

- replace raw string boundary with typed request/result

### `perception_driver_speech`

- keep `text_data` input but expose typed speech synthesis API explicitly

### `perception_driver_image_analysis`

- replace `std::pair<cv::Mat, std::string>` with typed image-analysis request/result

## Definition of Done

The refactor is complete when:

- `PerceptionServer` is mostly node wiring and configuration.
- No service callback contains provider-specific request assembly.
- New cross-plugin payloads use typed shared structs.
- Server-side `std::any_cast` calls are removed.
- Audio buffering is owned by a dedicated abstraction, not open-coded in the server.
- Adding a new plugin does not require duplicating large load/init blocks.

## Recommended Execution Order

1. Shared structs
2. Audio buffer extraction
3. Sentiment and image-analysis typing cleanup
4. Workflow extraction
5. Typed driver interfaces
6. Plugin loader cleanup

This order gives the best ratio of cleanup to risk.
