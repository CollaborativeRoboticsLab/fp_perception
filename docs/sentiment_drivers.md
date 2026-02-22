
# Sentiment drivers

This page documents the sentiment analysis plugin in `perception_driver_sentiment`.

## SentimentDriver

Class: `perception::SentimentDriver` (REST-based sentiment inference)

### What it does

- Accepts plain text (`std::string`) via `setDataStream()`.
- Sends a JSON request to the configured endpoint (Hugging Face style: `inputs: <text>`).
- Parses a Hugging Face-style response and exposes `(label, score)` via `getData()`.

### Parameters

- `driver.sentiment.SentimentDriver.name` (string)

REST base parameters:

- `driver.sentiment.SentimentDriver.rest.uri`
- `driver.sentiment.SentimentDriver.rest.method`
- `driver.sentiment.SentimentDriver.rest.ssl_verify`
- `driver.sentiment.SentimentDriver.rest.auth_type`

Environment:

- Requires `HUGGINGFACE_API_KEY` to be set.

### Usage with the server

- The server offers `perception_msgs/srv/PerceptionSentiment`.
- If `use_device_audio=true`, the server transcribes device audio first and then runs sentiment on the transcription.
- Otherwise, it runs sentiment directly on the request text.

