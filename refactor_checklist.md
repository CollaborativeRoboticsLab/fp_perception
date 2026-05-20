# Perception Refactor Checklist

This checklist tracks the remaining work after the typed-interface, audio-buffer, callback-audio, pipeline, and driver-manager refactors. Historical implementation steps have been collapsed so the file only shows current status and actionable next work.

## Current Status

- [x] Shared request/result structs exist for transcription, sentiment, vision, and image analysis.
- [x] `PerceptionServer` uses typed driver pointers where practical.
- [x] Server-side `std::any_cast` usage has been removed.
- [x] The last live plugin `std::any_cast` path was removed from speech synthesis audio handling.
- [x] `DriverBase` still exists as the pluginlib/lifecycle base.
- [x] Legacy `DriverBase` data hooks are marked as compatibility-only and are no longer used by server workflows.
- [x] `AudioBuffer` owns rolling microphone audio storage, trimming, waiting, latest reads, and timestamp-window reads.
- [x] Device-audio transcription and sentiment use `AudioBuffer` through typed pipelines.
- [x] Timestamped device-audio requests use the requested window when available.
- [x] Partial timestamp-window overlap returns available audio and logs a warning.
- [x] No-overlap timestamp-window requests warn and fall back to latest buffered audio instead of failing the node.
- [x] Microphone capture uses a PortAudio input callback.
- [x] Speaker playback uses a PortAudio output callback and playback queue.
- [x] `AudioSinkDriver` exposes `enqueuePlayback()` and `stopPlayback()` compatibility points.
- [x] `TranscriptionPipeline`, `SpeechPipeline`, `SentimentPipeline`, and `ImageAnalysisPipeline` exist.
- [x] `DriverManager` centralizes plugin declaration, loading, initialization, type casting, and test invocation.
- [x] Main docs describe typed role interfaces instead of the old `std::any` workflow contract.
- [x] `docs/test_the_system.md` documents terminal smoke tests for startup, generated WAV playback, device audio services, and image analysis.

## Remaining Refactor Work

### Legacy API Cleanup

- [ ] Remove `DriverBase::getData()`, `setData()`, `getDataStream()`, and `setDataStream()` after confirming no external packages still call them.
- [ ] Remove compatibility comments and old examples that mention untyped driver data exchange outside planning docs.
- [ ] Check for dead members left from pre-`DriverManager` loading and remove any that are no longer needed.

### Audio Backend Hardening

- [ ] Add callback overflow/underflow counters for microphone and speaker drivers.
- [ ] Expose callback queue/status diagnostics from non-real-time code.
- [ ] Confirm callback bodies avoid ROS logging, network calls, heap-heavy work, and long blocking waits.
- [ ] Decide whether a full-duplex PortAudio stream is needed for tighter input/output timing, or keep separate input/output streams.
- [ ] Add explicit queue drain/underrun status for speaker playback.

### REST Cleanup

- [ ] Replace deprecated `curl_formadd` / `CURLOPT_HTTPPOST` usage in `RestBase::call_audio()` with `curl_mime_*` APIs.
- [ ] Rebuild REST-backed drivers and confirm libcurl deprecation warnings are gone.

## Validation Checklist

### Build Validation

- [x] Focused build passed after timestamp-window handling changes: `perception`.
- [x] Focused build passed after speech-driver typed audio cleanup: `perception_base`, `perception_driver_speech`, and `perception`.
- [ ] Full perception stack build from `/home/ubuntu/colcon_ws`.

### Startup Validation

- [x] Device-name resolution verified in launch logs for microphone and speaker.
- [x] Built-in `run_tests` path reached loaded plugins during launch.
- [x] Microphone test generated a non-silent `test/mic_test.wav`.
- [x] Speaker test played through the PortAudio route.
- [x] Speech synthesis generated `test/speech.wav`.
- [x] `aplay -D plughw:2,0 test/mic_test.wav` works.
- [x] `aplay -D plughw:2,0 test/speech.wav` works.

### Service Smoke Tests

- [ ] `/perception/transcription` with `use_device_audio: true` and zero timestamp.
- [ ] `/perception/transcription` with `use_device_audio: true` and timestamp-window request.
- [ ] `/perception/sentiment_analysis` with `use_device_audio: true` and zero timestamp.
- [ ] `/perception/sentiment_analysis` with `use_device_audio: true` and timestamp-window request.
- [ ] `/perception/speech` with `use_device_audio: true` and audible playback.
- [ ] `/perception/image_analysis` with `use_device_vision: true`.
- [ ] External-image image-analysis service call.

### Runtime Audio Validation

- [ ] Confirm microphone can capture continuously for several minutes without crashes or buffer growth issues.
- [ ] Confirm speaker playback remains gap-free for typical generated speech.
- [ ] Confirm microphone capture continues while speaker playback is active.
- [ ] Confirm simultaneous capture/playback does not deadlock during service calls.
- [ ] Confirm callback underrun/overflow counters remain acceptable once counters exist.

## Deferred Or Out Of Scope

- [ ] Moving the rolling audio buffer into the microphone plugin is deferred; the chosen implementation is the reusable `AudioBuffer` owned by `perception`.
- [ ] Removing `DriverBase` as the pluginlib base is deferred; plugin XML and exports still use `perception::DriverBase`.
- [ ] Changing ROS message or service definitions is deferred; current refactor keeps public ROS interfaces stable.
- [ ] Full-duplex PortAudio is optional and should only be pursued if separate input/output streams do not meet timing needs.

## Next Recommended Steps

1. Run the terminal service smoke tests from `docs/test_the_system.md` and mark the service validation items above.
2. Run a full perception stack build from the colcon workspace root.
3. Modernize `RestBase::call_audio()` to remove libcurl deprecation warnings.
4. Add microphone/speaker callback diagnostics and repeat long-running simultaneous capture/playback tests.