# Perception Refactor Implementation Checklist

This checklist turns the refactor plan into concrete file-by-file implementation work. The order is designed to keep the system compiling after each small batch of changes.

## Working Rules

- Keep ROS message and service definitions stable until internal typed workflows are in place.
- Keep `DriverBase` temporarily during migration so existing plugin exports continue to work.
- Prefer additive changes first, then cut over call sites, then remove legacy code.
- Validate after each phase with a focused build of touched packages.

## Phase 1. Add Shared Types

### `perception_base/include/perception_base/audio/structs.hpp`

- [x] Keep `audio_data` and `text_data` unchanged unless a missing field is required.
- [x] Decide whether this file should remain the shared type home or only audio/text should stay here.
- [x] If splitting by domain, reduce this file to audio/text only and include the new headers where needed.

### `perception_base/include/perception_base/transcription/structs.hpp`

- [x] Create the file if choosing domain-specific headers.
- [x] Add `transcription_request`.
- [x] Add `transcription_result`.
- [x] Keep the types independent from ROS service request/response classes.

### `perception_base/include/perception_base/sentiment/structs.hpp`

- [x] Create `sentiment_request`.
- [x] Create `sentiment_result`.
- [x] Replace the implicit `std::string` and `std::pair<std::string, double>` boundary with these types.

### `perception_base/include/perception_base/vision/structs.hpp`

- [x] Create `vision_frame`.
- [x] Add image metadata fields actually needed by the server and plugins.
- [x] Keep ownership simple; start with `cv::Mat` by value unless copy pressure proves problematic.

### `perception_base/include/perception_base/image_analysis/structs.hpp`

- [x] Create `image_analysis_request`.
- [x] Create `image_analysis_result`.
- [x] Replace the implicit `std::pair<cv::Mat, std::string>` contract.

### `perception_base/CMakeLists.txt`

- [x] Export any new include paths or headers if needed.
- [x] Add OpenCV include requirements here only if the new shared headers require them.
- [x] Confirm no package dependency changes are needed beyond what `perception_base` already exposes.

### `perception_base/package.xml`

- [x] Add any new dependencies only if the new shared headers require them.
- [x] Avoid adding runtime dependencies that are only needed in downstream packages.

### Cutover tasks for Phase 1

- [x] Update include statements in the server and plugins to use the new typed headers.
- [x] Replace internal server locals that currently use raw strings, pairs, or ad hoc payloads.
- [x] Do not remove `std::any` interfaces yet; only improve the types flowing through them.

### Phase 1 status

- [x] Shared structs added for transcription, sentiment, vision, and image analysis.
- [x] Sentiment, image analysis, and transcription internals now use the shared structs.
- [x] Existing ROS service interfaces remain unchanged.
- [x] Existing `std::any` driver interfaces remain in place for later phases.
- [x] Focused builds passed for `perception_base`, `perception`, `perception_driver_sentiment`, `perception_driver_image_analysis`, and `perception_driver_transcribe`.

## Phase 2. Extract Audio Buffering

### `perception/include/perception/audio_buffer.hpp`

- [x] Create an `AudioBuffer` class.
- [x] Move rolling sample storage here.
- [x] Move mutex and condition variable ownership here.
- [x] Add append logic for new `audio_data` chunks.
- [x] Add format reset logic when sample rate or channel count changes.
- [x] Add trim logic for max buffered duration.
- [x] Add `waitForAudio()` or equivalent initialization guard.
- [x] Add `readLatest(duration_seconds)` to fetch the newest buffered audio.

### `perception/perception/include/perception/perception_server.hpp`

- [x] Remove `public_buffer_`, `public_buffer_mutex_`, `public_buffer_cv_`, and `public_buffer_total_samples_`.
- [x] Add an `AudioBuffer` member.
- [x] Change `publishAudio()` to append microphone chunks through `AudioBuffer`.
- [x] Change transcription device-audio reads to use `AudioBuffer`.
- [x] Change sentiment device-audio reads to use `AudioBuffer`.
- [x] Remove `wait_for_public_audio()` once all call sites are migrated.
- [ ] Change transcription device-audio reads from latest-N-seconds to timestamp-window extraction when request headers provide timing.
- [ ] Change sentiment device-audio reads to use the same timestamp-window extraction through the transcription path.
- [ ] Keep latest-N-seconds behavior only as a fallback when no useful request timestamp is available.

### Optional alternative: `perception_driver_audio/include/perception_driver_audio/microphone_audio_driver.hpp`

- [ ] If buffering is moved into the microphone plugin instead of a shared helper, add typed read methods here.
- [ ] Ensure the server no longer knows buffer details such as trimming and duration slicing.
- [ ] If production audio buffering moves into the plugin, implement timestamped ring-buffer ownership here.
- [ ] Add typed APIs such as `readLatest(duration)` and `readWindow(start, duration)`.

### Phase 2 validation

- [x] Build `perception` and `perception_driver_audio`.
- [ ] Verify `use_device_audio` still works for transcription.
- [ ] Verify `use_device_audio` still works for sentiment.
- [ ] Verify a request with `(header stamp, duration)` extracts the expected audio interval.
- [ ] Verify old audio outside the ring-buffer window returns a clear error.
- [ ] Verify partial-window behavior is intentional and documented.

## Phase 2b. Add Real-Time Audio Backend

### Architecture decision

- [x] For the robot use case, prefer PortAudio callbacks plus ring buffers over direct blocking `Pa_ReadStream()` / `Pa_WriteStream()` as the production path.
- [ ] Keep the blocking implementation only as a fallback, diagnostic mode, or temporary migration path.
- [ ] Decide whether to use separate input/output streams or a single full-duplex stream for tighter input/output timing.

### `perception_driver_audio/include/perception_driver_audio/microphone_audio_driver.hpp`

- [ ] Replace production capture loop based on `Pa_ReadStream()` with a PortAudio input callback.
- [ ] Pass `this` as callback user data and keep callback work real-time safe.
- [ ] Append captured frames into a preallocated timestamped ring buffer.
- [ ] Avoid ROS logging, heap allocation, network calls, and blocking waits inside the callback.
- [ ] Move publishing/ROS-facing work to a normal thread that drains callback-produced chunks.
- [ ] Add input overflow/underflow counters readable from non-real-time code.

### `perception_driver_audio/include/perception_driver_audio/speaker_audio_driver.hpp`

- [ ] Replace production playback based on direct `Pa_WriteStream()` with a PortAudio output callback.
- [ ] Add a playback queue/ring buffer filled by `play()` / `enqueuePlayback()`.
- [ ] Add streaming TTS support where chunks can be queued while previous chunks are still playing.
- [ ] Fill underrun output with silence and count underruns.
- [ ] Add `stopPlayback()` / queue clear support for robot interruption behavior.
- [ ] Avoid ROS logging, heap allocation, and blocking waits inside the callback.

### `perception_base/include/perception_base/audio/audio_source_driver.hpp`

- [ ] Add typed `readLatest(duration)` or `readBufferedAudio(duration)` once ownership is decided.
- [ ] Add typed `readWindow(start, duration)` for timestamped STT requests.

### `perception_base/include/perception_base/audio/audio_sink_driver.hpp`

- [ ] Add `enqueuePlayback(const audio_data&)` or define `play()` as queueing/non-blocking for streaming playback.
- [ ] Add optional `stopPlayback()` and queue status methods.

### Phase 2b validation

- [ ] Confirm microphone can capture continuously for several minutes without buffer overrun crashes.
- [ ] Confirm speaker can play a WAV through the plugin without gaps.
- [ ] Confirm streaming TTS chunks play continuously when enqueued incrementally.
- [ ] Confirm microphone capture continues while speaker playback is active.
- [ ] Confirm callback underrun/overflow counters remain acceptable during normal robot interaction.

## Phase 3. Add Typed Driver Interfaces

### `perception_base/include/perception_base/driver_base.hpp`

- [ ] Keep lifecycle helpers and shared utilities only.
- [x] Do not add more untyped virtual methods.
- [ ] If needed, deprecate generic methods in comments while migration is in progress.

### `perception_base/include/perception_base/audio/audio_source_driver.hpp`

- [x] Create a typed interface for microphone-like sources.
- [x] Add `readChunk()`.
- [ ] Add `readBufferedAudio()` if buffering is owned by the source.
- [ ] Add timestamp-window read methods when request-time extraction is implemented.

### `perception_base/include/perception_base/audio/audio_sink_driver.hpp`

- [x] Create a typed interface for speaker-like sinks.
- [x] Add a method such as `play(const audio_data&)`.
- [ ] Decide whether `play()` should block until accepted/played or enqueue and return quickly.
- [ ] Add streaming/queueing methods if `play()` remains blocking for compatibility.

### `perception_base/include/perception_base/transcription/transcription_driver.hpp`

- [x] Create a typed transcription interface.
- [x] Add `transcribe(const transcription_request&)`.

### `perception_base/include/perception_base/speech/speech_synthesis_driver.hpp`

- [x] Create a typed speech synthesis interface.
- [x] Add a method such as `synthesize(const text_data&)` returning `audio_data`.

### `perception_base/include/perception_base/sentiment/sentiment_analysis_driver.hpp`

- [x] Create a typed sentiment interface.
- [x] Add `analyze(const sentiment_request&)`.

### `perception_base/include/perception_base/vision/vision_source_driver.hpp`

- [x] Create a typed vision source interface.
- [x] Add a method such as `captureFrame()` returning `vision_frame`.

### `perception_base/include/perception_base/image_analysis/image_analysis_driver.hpp`

- [x] Create a typed image analysis interface.
- [x] Add `analyze(const image_analysis_request&)`.

### `perception_driver_audio/include/perception_driver_audio/microphone_audio_driver.hpp`

- [x] Make the microphone driver implement `AudioSourceDriver`.
- [x] Keep legacy `getDataStream()` temporarily if needed.
- [x] Add a typed adapter method that the server or pipeline can use immediately.
- [ ] Convert production mode to callback-driven capture for always-on robot listening.
- [ ] Store capture timestamps for audio-window extraction.

### `perception_driver_audio/include/perception_driver_audio/speaker_audio_driver.hpp`

- [x] Make the speaker driver implement `AudioSinkDriver`.
- [x] Add a typed `play()` wrapper around the current stream write path.
- [x] Keep `setDataStream()` temporarily for backward compatibility if needed.
- [ ] Convert production mode to callback-driven queued playback for low-latency streaming TTS.
- [ ] Support gapless chunk enqueueing and controlled interruption.

### `perception_driver_transcribe/include/perception_driver_transcribe/openai_driver.hpp`

- [x] Make the driver implement `TranscriptionDriver`.
- [x] Add a typed `transcribe()` method that internally reuses the current REST logic.
- [x] Keep `setDataStream()` and `getData()` only until server call sites are migrated.

### `perception_driver_speech/include/perception_driver_speech/openai_driver.hpp`

- [x] Make the driver implement `SpeechSynthesisDriver`.
- [x] Add a typed `synthesize(const text_data&)` method.
- [x] Return `audio_data` directly from the typed method.

### `perception_driver_sentiment/include/perception_driver_sentiment/sentiment_driver.hpp`

- [x] Make the driver implement `SentimentAnalysisDriver`.
- [x] Replace typed boundary assumptions based on `std::string` input and `std::pair` output.
- [x] Add a typed `analyze(const sentiment_request&)` method.

### `perception_driver_image_analysis/include/perception_driver_image_analysis/openai_driver.hpp`

- [x] Make the driver implement `ImageAnalysisDriver`.
- [x] Replace the `std::pair<cv::Mat, std::string>` payload with `image_analysis_request`.
- [x] Return `image_analysis_result` from the typed method.

### `perception_driver_vision/include/perception_driver_vision/default_vision_driver.hpp`

- [x] Evaluate whether this plugin should implement `VisionSourceDriver` directly.
- [x] Add a typed frame-returning method if the plugin remains part of the refactor target.

### `perception_driver_vision/include/perception_driver_vision/opencv_vision_driver.hpp`

- [x] Add the same typed `VisionSourceDriver` support for the non-ROS vision plugin.

### Phase 3 cutover tasks

- [x] Change server-held pointers from generic `DriverBase` where practical.
- [x] Keep pluginlib exports stable while changing inheritance chains.
- [x] Remove server-side `std::any_cast` calls only after typed APIs are wired.

## Phase 4. Extract Workflows From `PerceptionServer`

### `perception/include/perception/transcription_pipeline.hpp`

- [x] Create the pipeline class.
- [x] Accept typed dependencies needed for transcription.
- [x] Accept `transcription_request`.
- [x] Return `transcription_result`.
- [x] Own device-audio fallback and validation logic currently in the callback.

### `perception/include/perception/speech_pipeline.hpp`

- [x] Create the pipeline class.
- [x] Accept speech and optional speaker dependencies.
- [x] Handle `use_device_audio` routing.
- [x] Return a typed result that the ROS callback can map easily.

### `perception/include/perception/sentiment_pipeline.hpp`

- [x] Create the pipeline class.
- [x] Accept sentiment and optional transcription dependencies.
- [x] Handle the device-audio -> transcription -> sentiment chain.
- [x] Return `sentiment_result`.

### `perception/include/perception/image_analysis_pipeline.hpp`

- [x] Create the pipeline class.
- [x] Accept image-analysis and vision-source dependencies.
- [x] Handle `use_device_vision` source selection.
- [x] Return `image_analysis_result`.

### `perception/perception/include/perception/perception_server.hpp`

- [x] Add pipeline members or construct them during initialization.
- [x] Shrink `transcribe_callback()` to request mapping plus pipeline call.
- [x] Shrink `speech_callback()` to request mapping plus pipeline call.
- [x] Shrink `sentiment_callback()` to request mapping plus pipeline call.
- [x] Shrink `image_analysis_callback()` to request mapping plus pipeline call.
- [x] Keep ROS message/service conversion only.

### Phase 4 validation

- [x] Build `perception` and all touched driver packages.
- [ ] Smoke test each service path.
- [ ] Verify error responses remain equivalent or intentionally improved.

## Phase 5. Simplify Plugin Loading

### `perception/include/perception/driver_manager.hpp`

- [x] Create a small driver manager or loader helper.
- [x] Centralize declare/load/initialize/test behavior.
- [x] Keep plugin name strings and parameter keys in one place.

### `perception/perception/include/perception/perception_server.hpp`

- [x] Replace repeated plugin loading blocks with helper calls.
- [x] Move plugin-test invocation behind the manager if it reduces repetition.
- [x] Group interface configuration separately from driver loading.

### `perception/perception/src/server_node.cpp`

- [x] Check whether node construction needs to change to support explicit post-construction initialization.
- [x] If constructor-based initialization remains fragile, move startup to a safer lifecycle point.

### `perception/perception/src/server_component.cpp`

- [x] Mirror any startup changes made for the standalone node path.
- [x] Keep component registration behavior unchanged.

### Phase 5 validation

- [x] Build `perception`.
- [ ] Verify plugins still load from parameters.
- [ ] Verify `run_tests` still reaches loaded plugins.

## Phase 6. Remove Legacy Untyped Paths

### `perception/perception/include/perception/perception_server.hpp`

- [x] Remove remaining `std::any_cast` usage.
- [ ] Remove legacy helper code that only existed for untyped flows.
- [ ] Remove dead members left over from the old plugin loading pattern.

### `perception_base/include/perception_base/driver_base.hpp`

- [ ] Remove generic data methods only when all plugins and server call sites are migrated.
- [x] If full removal is too disruptive, mark them as legacy and stop using them internally.

### Driver headers in all plugin packages

- [x] Remove compatibility wrappers that were kept only for migration.
- [x] Ensure typed interfaces are the only code paths used by `PerceptionServer` and pipelines.

### Phase 6 validation

- [ ] Full build of all perception-related packages.
- [ ] Focused runtime smoke tests for microphone, speaker, transcription, sentiment, speech, and image analysis.
- [ ] Runtime audio smoke test: plugin playback works on the same hardware route that `aplay -D plughw:2,0 test/speech.wav` uses.
- [ ] Runtime audio smoke test: plugin recording produces a valid WAV with non-silent samples.
- [ ] Runtime audio smoke test: simultaneous capture and playback works without deadlock or large glitches.

## Suggested Build and Verification Sequence

### After Phase 1

- [ ] Build `perception_base`.
- [ ] Build `perception`.
- [ ] Build affected driver packages.

### After Phase 2

- [ ] Run a focused build for `perception` and `perception_driver_audio`.
- [ ] Verify buffered device audio behavior still matches expectations.

### After Phase 3 and Phase 4

- [ ] Build all touched packages together.
- [ ] Exercise each ROS service path once.

### After Phase 5 and Phase 6

- [ ] Run a full workspace build for the perception stack.
- [ ] Review remaining warnings before removing migration shims.

## Practical Implementation Order

Use this as the execution order if working in small PRs or commits:

1. `perception_base` shared structs.
2. `perception` audio buffer extraction.
3. `perception_driver_sentiment` typed request/result cutover.
4. `perception_driver_image_analysis` typed request/result cutover.
5. `perception` pipeline extraction.
6. typed interface rollout across all driver packages.
7. `perception` driver manager cleanup.
8. legacy untyped API removal.

## Definition of Progress

You are meaningfully done with each phase when:

- the touched packages build
- the new abstractions are used by the server
- old compatibility code remains only where still needed by an unmigrated phase
- no new duplication is introduced while migrating