
# Image analysis drivers

This document describes the image analysis driver plugins and how they integrate with the perception server.

## OpenAI image analysis driver

Plugin type:

- `perception::OpenAIImageAnalysisDriver`

This driver is REST-backed and uses the OpenAI **Responses API** to analyze an image and return text.

### How requests are formed

The driver sends a request to `https://api.openai.com/v1/responses` with:

- an `input_text` containing the user prompt
- an `input_image` containing a **base64-encoded data URL** (for example `data:image/png;base64,...`)

The server passes both the image and the prompt to the driver.

### Parameters

The driver reads parameters under `driver.image_analysis.OpenAIDriver.*`:

- `name` (string): informational name
- `model` (string): model used for image analysis (default `gpt-4.1`)
- `detail` (string): image detail level (`auto`, `low`, `high`)
- `test_file_path` (string): image path used by the driver `test()` function
- `test_prompt` (string): prompt used by the driver `test()` function

REST settings (provided by `RestBase`):

- `driver.image_analysis.OpenAIDriver.rest.uri` (string): should be `https://api.openai.com/v1/responses`
- `driver.image_analysis.OpenAIDriver.rest.method` (string): `POST`
- `driver.image_analysis.OpenAIDriver.rest.ssl_verify` (bool)
- `driver.image_analysis.OpenAIDriver.rest.auth_type` (string): `Bearer`

Environment variables:

- `OPENAI_API_KEY`: required for authorization

### Output

The driver returns a single string containing the model output text.

If the OpenAI response includes an error payload, the driver surfaces the error message as the returned string.

