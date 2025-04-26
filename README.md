# LingoROS 🤖💬

## A Natural Language Control Interface for Your Existing Robot Using LLMs

LingoROS is an ROS package that enables you to control your existing robot using natural language commands. It acts as a plug-and-play interface layer between human language and your robot's ROS system, requiring no changes to your existing robot software stack.

[![ROS1](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![Python](https://img.shields.io/badge/Python-3.10-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Status](https://img.shields.io/badge/Status-Ongoing-blue)](https://github.com/yashphalle/lingoros)
[![Version](https://img.shields.io/badge/Version-0.1.0-brightgreen)](https://github.com/yashphalle/lingoros/releases)
[![Ollama](https://img.shields.io/badge/Ollama-Integrated-orange)](https://ollama.ai/)
[![Contributions Welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg)]((https://github.com/yashphalle/LingoROS/issues))
[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-blue?logo=linkedin)](https://www.linkedin.com/in/yash-phalle-3b596b192/)

## 🤖 What does LingoROS do?

LingoROS connects large language models (LLMs) to ROS-enabled robots, allowing users to control robots with voice or text commands instead of writing code or using specialized interfaces. Simply speak or type commands like "go forward 2 meters" or "turn right 45 degrees," and LingoROS * translates these into the appropriate ROS messages *.

### Key Advantages:

- **Zero Robot Modifications**: Works with your existing ROS robot setup without changing any of your robot's codebase
- **Automatic Adaptation**: Discovers available ROS topics and interfaces at startup
- **Natural Interaction**: Understand everyday language rather than technical commands
- **Intelligent Understanding**: Uses a fine-tuned LLM to comprehend context and intent

## ⚙️ Architecture

LingoROS uses a hybrid architecture that combines LLM natural language understanding with efficient execution:

1. **Interface Discovery**: Automatically detects your robot's ROS topics at startup
2. **LLM-Powered Understanding**: Processes natural language through a specialized model
3. **Session-Based Context**: Maintains awareness of robot capabilities across interactions
4. **Rule-Based Execution**: Efficiently translates intents to ROS commands

## 🛠️ Components

### NLU Node
- Takes text input (via typing or speech)
- Processes language using an LLM (Ollama with phi3:mini by default )
- Outputs structured JSON describing the intended action

### Action Mapping Node
- Subscribes to the NLU node's output
- Handles command execution timing
- Publishes to the robot's control topics

## 📋 Example Commands

- "Move forward 3 meters"
- "Turn left 90 degrees"
- "Navigate to the kitchen"
- "Speed up a little"
- "Stop immediately"

## 📊 Current Progress

### Completed
- ✅ Basic project architecture defined
- ✅ Core NLU node implementation with Ollama integration
- ✅ Action mapping node for basic movement commands
- ✅ Support for linear movement and rotation commands
- ✅ Command caching and optimization 
- ✅ Basic prompt engineering for reliable command parsing
- ✅ Testing with simulated Husky robot in Gazebo

### In Progress
- 🔄 Improving performance and reducing latency
- 🔄 Enhancing topic discovery reliability
- 🔄 Implementation of session-based LLM context
- 🔄 Expanding supported command types & Making it platform independent

## 🚀 Getting Started

### Prerequisites
- ROS 1 Noetic (or compatible version)
- Python 3.x
- Ollama or other LLM runtime

### Installation and Usage

1.  **Clone Repository:** Clone this repository into your ROS catkin workspace's `src` directory:
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/yashphalle/LingoROS.git
    ```
2.  **Install Python Dependencies:**
    ```bash
    pip install ollama
    ```
3.  **Build Workspace:** Build the package using `catkin_make` :
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
4.  **Source Workspace:** Source the workspace's setup file in any terminal where you want to run LingoROS nodes:
    ```bash
    source ~/catkin_ws/devel/setup.bash
    ```

## How to Run (Current Test Setup)

1.  **Terminal 1:** Start ROS Master
    ```bash
    roscore
    ```
2.  **Terminal 2:** Launch the target robot simulation (e.g., Husky):
    ```bash
    roslaunch husky_gazebo husky_playpen.launch
    ```
3.  **Terminal 3:** **Ensure the Ollama service is running** in the background. (You can check with `ollama list` or `sudo systemctl status ollama`). Start it (`ollama serve`) or ensure the service is active if necessary. Make sure the required model (e.g., `phi3:mini`) is available via `ollama list`.
4.  **Terminal 4:** Launch the core LingoROS nodes:
    ```bash
    roslaunch lingoros lingoros_nodes.launch
    ```
    *(You should be able to see log messages from both nodes here, including which Ollama model the NLU node is using).*
5.  **Terminal 5:** Publish a natural language command to the text input topic:
    ```bash
    # Example: Move forward
    rostopic pub /lingoros/text_input std_msgs/String "data: 'go 2 meters forward'"

    # Example: Turn left
    rostopic pub /lingoros/text_input std_msgs/String "data: 'turn left 90 degrees'"
    ```

## Contributing

LingoROS is my first open source project, and I welcome your contributions to help it grow!

> "I believe with advancements in generative AI and LLMs, robots will become more intelligent and accessible. LingoROS is my small step toward contributing to this future."

### Ways to Contribute

- **Code**: Add features or fix bugs
- **Documentation**: Improve docs
- **Testing**: Test with different robots
- **Ideas**: Suggest improvements
- **Spread the Word**: Star and share

### Getting Started

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request




## 🤝 Team up? Count me in for any robotics project  
Wondering if it’s your code or your robot’s mood swings? Let’s team up and sort this out.

LinkedIn: https://www.linkedin.com/in/yash-phalle-3b596b192/
