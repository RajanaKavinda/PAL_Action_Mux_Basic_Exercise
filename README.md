# PAL_Action_Mux_Basic_Exercise
 This repo contains the solution for basic pragramming exercise given by PAL robotics under GSoC 2025 contributor selection round. 

1. **Action Server & Client with Topic Subscription**: An action server that receives service requests via a topic subscription by the client. If new requests arrive before completing the current goal, the previous goal is aborted, and a new goal is set. The server waits 5 seconds before succeeding, handling any premature aborts properly.
2. **Generic Subscriber**: A ROS 2 node that can subscribe to any topic, print the received message, and display its type.

## Installation
Clone the repository and build it inside your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repository-url>
cd ~/ros2_ws
colcon build 
source install/setup.bash
```

---
## Task 1: ROS 2 Action Server with Topic Subscription
### Description
- The action server starts and waits for goals.
- Goals are received from a topic (`/goal_topic`) by the action client.
- If a new goal arrives before the previous one completes, the previous goal is aborted.
- The server takes 5 seconds to complete a goal.
- If aborted before completion, it handles the abortion properly.

### Running the Action Server & Client
#### Start the Action Server
```bash
ros2 run action_proj action_server
```
**Expected Output:**
```
[INFO] [1742308871.758388818] [action_server]: Simple action server has been started
```

#### Start the Action Client
```bash
ros2 run action_proj action_client
```
**Expected Output:**
```
[INFO] [1742308871.800361062] [action_client]: Simple action client has been started. Send a goal to /goal_topic.
```

#### Publish Goals to the Topic at a High Rate
```bash
ros2 topic pub /goal_topic std_msgs/msg/Int32 "{data: 10}" --rate 2
```

---
## Task 2: Generic Subscriber for Any Message Type
### Description
- This node subscribes to any topic provided as a command-line argument.
- It prints the message data along with its message type.
- Different publishers can be used to test this functionality.

### Running the Generic Subscriber & Testing with Publishers
#### Subscribe to a Topic
```bash
ros2 run action_proj generic_subscriber /topic
```
**Expected Output (Before Receiving Messages):**
```
[INFO] [1742309000.688543991] [generic_subscriber]: Started. Waiting for topic '/topic' to become available.
```

#### Test with `Int32` Publisher
Start the subscriber and publish messages:
```bash
ros2 run action_proj int32_publisher
```
**Expected Output:**
```
[INFO] [1742309050.132815093] [generic_subscriber]: Subscribed to topic '/topic'
Type: std_msgs/msg/Int32

data: 42
---
```

#### Test with `String` Publisher
Start the subscriber and publish messages:
```bash
ros2 run action_proj string_publisher
```
**Expected Output:**
```
[INFO] [1742309302.507581303] [generic_subscriber]: Subscribed to topic '/topic'
Type: std_msgs/msg/String

data: Hello, world!
---
```
