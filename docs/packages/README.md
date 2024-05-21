
# Package Name

## Overview
The `package_name` package is part of the `project_name` and contains [brief description]. It [primary function/purpose].

## Common Configuration
The common configuration for this package is specified in `config_file.yaml` and includes parameters used by the node.

### Configuration Parameters
| Parameter Name   | Type   | Default        | Description                     |
|------------------|--------|----------------|---------------------------------|
| `parameter_name` | `type` | Default value. | Description of the parameter.   |
| ...              | ...    | ...            | ...                             |

## Dependencies
List all the dependencies required by the package.

### Dependencies List
- `dependency1`
- `dependency2`
- ...

## Usage
The node(s) in this package can be launched using either the centralized launch service or locally using specific commands.

### Centralized Launch Service
The nodes in this package are typically launched by the `riptide_launch` service. For more details, refer to [riptide_launch](https://github.com/osu-uwrt/riptide_launch).

### Local Launch Command
Some nodes might be launched locally using specific commands. For example:

```bash
ros2 launch package_name node_name.launch.py [additional_arguments]
```

## Diagnostics and Monitoring
To effectively diagnose and monitor the nodes in this package, use the following guidelines:

- **Topic Monitoring**: Monitor relevant topics using `ros2 topic list` and `ros2 topic echo [topic_name]`.
- **Logs**: Check ROS2 logs for any warnings or errors using `ros2 log list` and `ros2 log echo [log_name]`.
- **Custom Diagnostics**: If specific diagnostics are provided by the nodes, refer to the node documentation for details on how to use them.

## Troubleshooting
### Problem: Description of the problem
- **Solution**: Steps to solve the problem.

### Common Issues
- **Issue 1**: Description and solution.
- **Issue 2**: Description and solution.
- ...

## Nodes

### Node 1 Name

#### Overview
The `node_1_name` node [brief description of what it does].

#### ROS2 Interfaces

**Parameters**
| Parameter Name   | Type   | Description                     |
|------------------|--------|---------------------------------|
| `parameter_name` | `type` | Description of the parameter.   |
| ...              | ...    | ...                             |

**Subscriptions**
| Subscription Topic  | Message Type     | Description                     |
|---------------------|------------------|---------------------------------|
| `topic_name`        | `message_type`   | Description of the subscription.|
| ...                 | ...              | ...                             |

**Publishers**
| Publisher Topic     | Message Type     | Description                     |
|---------------------|------------------|---------------------------------|
| `topic_name`        | `message_type`   | Description of the publisher.   |
| ...                 | ...              | ...                             |

#### Functional Description
1. **Feature 1**: Description of the feature.
2. **Feature 2**: Description of the feature.
3. ...

#### Data Flow
- **Input**: Description of input data.
- **Processing**: Description of processing steps.
- **Output**: Description of output data.

#### Key Functions and Classes
- `function_name(parameters)`: Description of the function.
- ...

### Node 2 Name

#### Overview
The `node_2_name` node [brief description of what it does].

#### ROS2 Interfaces

**Parameters**
| Parameter Name   | Type   | Description                     |
|------------------|--------|---------------------------------|
| `parameter_name` | `type` | Description of the parameter.   |
| ...              | ...    | ...                             |

**Subscriptions**
| Subscription Topic  | Message Type     | Description                     |
|---------------------|------------------|---------------------------------|
| `topic_name`        | `message_type`   | Description of the subscription.|
| ...                 | ...              | ...                             |

**Publishers**
| Publisher Topic     | Message Type     | Description                     |
|---------------------|------------------|---------------------------------|
| `topic_name`        | `message_type`   | Description of the publisher.   |
| ...                 | ...              | ...                             |

#### Functional Description
1. **Feature 1**: Description of the feature.
2. **Feature 2**: Description of the feature.
3. ...

#### Data Flow
- **Input**: Description of input data.
- **Processing**: Description of processing steps.
- **Output**: Description of output data.

#### Key Functions and Classes
| Function/Class               | Description                     |
|------------------------------|---------------------------------|
| `function_name(parameters)`  | Description of the function.    |
| ...                          | ...                             |

## Example

For an example of this template, refer to [tensor_detector](https://github.com/osu-uwrt/riptide_perception/tree/master/tensor_detector).