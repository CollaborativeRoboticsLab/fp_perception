# Perception Refactor Checklist

Only active refactor items and validation status are kept here.

## Completed Core Refactor

- [x] Typed request/result structs and typed pipelines are in place.
- [x] `PerceptionServer` and loaded plugins no longer rely on server-side `std::any_cast` paths.
- [x] `AudioBuffer` handles rolling microphone storage and timestamp-window reads.
- [x] Microphone capture and speaker playback use PortAudio callbacks.
- [x] Driver diagnostics publish on `/diagnostics` via `diagnostic_updater` when `use_diagnostics=true`.
- [x] `DriverManager` centralizes plugin loading, initialization, casting, and test execution.
- [x] Main docs and smoke-test docs are updated for the typed workflow.

## Remaining Refactor Work

### Legacy API Cleanup

- [x] Remove `DriverBase::getData()`, `setData()`, `getDataStream()`, and `setDataStream()` after confirming no external packages still call them.
- [x] Remove compatibility comments and old examples that mention untyped driver data exchange outside planning docs.
- [x] Check for dead members left from pre-`DriverManager` loading and remove any that are no longer needed.

### Audio Backend Hardening

- [x] Add callback overflow/underflow counters for microphone and speaker drivers.
- [x] Expose callback queue/status diagnostics from non-real-time code.
- [ ] Confirm callback bodies avoid ROS logging, network calls, heap-heavy work, and long blocking waits.
- [ ] Decide whether a full-duplex PortAudio stream is needed for tighter input/output timing, or keep separate input/output streams.
- [x] Add explicit queue drain/underrun status for speaker playback.

### REST Cleanup

- [x] Replace deprecated `curl_formadd` / `CURLOPT_HTTPPOST` usage in `RestBase::call_audio()` with `curl_mime_*` APIs.
- [x] Rebuild REST-backed drivers and confirm libcurl deprecation warnings are gone.

## Validation Checklist

### Build Validation

- [x] Focused build passed after timestamp-window handling changes: `perception`.
- [x] Focused build passed after speech-driver typed audio cleanup: `perception_base`, `perception_driver_speech`, and `perception`.
- [x] Focused build passed after callback diagnostics changes: `perception_base`, `perception_driver_audio`, and `perception`.
- [x] Full perception stack build from `/home/ubuntu/colcon_ws`.

### Startup Validation

- [x] Device-name resolution verified in launch logs for microphone and speaker.
- [x] Built-in `run_tests` path reached loaded plugins during launch.
- [x] Microphone test generated a non-silent `test/mic_test.wav`.
- [x] Speaker test played through the PortAudio route.
- [x] Speech synthesis generated `test/speech.wav`.
- [x] `aplay -D plughw:2,0 test/mic_test.wav` works.
- [x] `aplay -D plughw:2,0 test/speech.wav` works.

### Service Smoke Tests

- [x] `/perception/transcription` with `use_device_audio: true` and zero timestamp.
- [x] `/perception/transcription` with `use_device_audio: true` and timestamp-window request.
- [x] `/perception/sentiment_analysis` with `use_device_audio: true` and zero timestamp.
- [x] `/perception/sentiment_analysis` with `use_device_audio: true` and timestamp-window request.
- [x] `/perception/speech` with `use_device_audio: true` and audible playback.
- [x] `/perception/image_analysis` with `use_device_vision: true`.
- [x] External-image image-analysis service call.

### Runtime Audio Validation

- [ ] `/diagnostics` investigation reduced microphone `dropped_callback_chunks` to `0` after pacing and ring-buffer fixes, but `input_overflow_count` still remains above `0` with `chunk_size: 1920`; treat this as the current software stopping point pending longer runtime tests.
- [ ] Confirm microphone can capture continuously for several minutes without crashes or buffer growth issues.
- [ ] Confirm speaker playback remains gap-free for typical generated speech.
- [ ] Confirm microphone capture continues while speaker playback is active.
- [ ] Confirm simultaneous capture/playback does not deadlock during service calls.
- [ ] Confirm callback underrun/overflow counters remain acceptable once counters exist.

## Deferred Or Out Of Scope

- [ ] Keep `AudioBuffer` owned by `perception`; moving it into the microphone plugin is deferred.
- [ ] Keep `DriverBase` as the pluginlib base for now.
- [ ] Keep public ROS messages and services stable.
- [ ] Full-duplex PortAudio is optional unless separate streams prove insufficient.

## Next Recommended Steps

1. Repeat long-running simultaneous capture/playback tests and inspect `/diagnostics` counters.
2. Confirm whether microphone `input_overflow_count` stays low and stable over longer runtime at `chunk_size: 1920`.
3. Decide whether stream-latency tuning or full-duplex PortAudio is necessary based on those runtime results.