# perception_events

`perception_events` is a ROS package within the `perception` stack, designed to handle and process perception-related events in robotic systems.

## Features

- Publishes and subscribes to perception event topics
- Integrates with other perception modules
- Provides event filtering and notification mechanisms

## Dependencies

- ROS (Robot Operating System)
- perception_msgs

## Usage

### Nodes

| Node Name           | Description                           |
|---------------------|-------------------------------------- |
| event_client        | Publishes perception events           |
| event_listener      | Subscribes and prints to terminal     |
| foxglove_bridge     | Subscribes and publishes to network   |

## Contributing

Contributions are welcome! Please submit issues and pull requests via GitHub.

## License

This project is licensed under the MIT License.