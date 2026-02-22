
# Base classes

This package is built around a small set of base classes that make it easy to load perception "drivers" as plugins and exchange data through a uniform interface.

## DriverBase

`perception::DriverBase` is the common base class for all perception plugins.

### Lifecycle

- `initialize(const rclcpp::Node::SharedPtr&)`: called after plugin construction (pluginlib creates objects with a default constructor). The default implementation calls `initialize_base(node)`.
- `deinitialize()`: **pure virtual** and must be implemented by every driver.
- `test()`: optional; default logs the driver name.

### Data exchange

Drivers exchange data through `std::any` to keep the base interface generic:

- `getData()` / `setData(const std::any&)`: single-shot data pull/push.
- `getDataStream()` / `setDataStream(const std::any&)`: stream-style pull/push.

Individual drivers define what types they expect inside the `std::any` (for example `perception::audio_data`, `perception::text_data`, `cv::Mat`, etc.).

### Common utilities

- `check_directory(folder)`: creates a directory if missing (used by driver tests).
- `check_file(path)`: throws if file does not exist.

## RestBase

`perception::RestBase` derives from `DriverBase` and provides a reusable REST client built on libcurl.

### REST initialization

Drivers typically call:

`initialize_rest_base(node, "<plugin_prefix>", "<ENV_KEY>")`

This declares and loads common parameters under `<plugin_prefix>.rest.*`:

- `<plugin_prefix>.rest.uri` (string)
- `<plugin_prefix>.rest.method` (string, e.g. `POST`)
- `<plugin_prefix>.rest.ssl_verify` (bool)
- `<plugin_prefix>.rest.auth_type` (string, currently `Bearer`)

If `ENV_KEY` is provided, the API key is read from that environment variable (e.g. `OPENAI_API_KEY`, `HUGGINGFACE_API_KEY`).

### Request helpers

`RestBase` provides:

- `call(req)`: JSON request/response. Requires the derived driver to implement `toJson()` and `fromJson()`.
- `call_audio(req)`: multipart form upload (used for audio transcription).
- `call_tts(req)`: JSON request that returns raw PCM audio (converted to `std::vector<int16_t>` in `RESTResponse::audio_stream`).

### Important note about `deinitialize()`

`RestBase` does **not** implement `deinitialize()` (it remains pure via `DriverBase`). Any REST driver must implement `deinitialize()` to be instantiable by pluginlib.

